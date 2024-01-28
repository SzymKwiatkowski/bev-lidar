// Copyright 2024 Szymon
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <sstream>
#include <vector>
#include <errno.h>
#include <assert.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "bev_lidar/cloud_filter/cloud_filter.hpp"
#include "bev_lidar/bev_lidar_offline_node.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>

namespace bev_lidar
{

BevLidarOfflineNode::BevLidarOfflineNode(const rclcpp::NodeOptions & options)
:  Node("bev_lidar", options)
{
  // Init object with logic
  bev_lidar_ = std::make_unique<bev_lidar::BevLidar>();
  cloud_filter_ = std::make_unique<cloud_filter::CloudFilter>();
  
  // Init all params
  grid_min_y_ = this->declare_parameter("grid_min_y", MIN_CROP); // enable non-square BEV images by cropping on the left
  grid_max_y_ = this->declare_parameter("grid_max_y", MAX_CROP); // enable non-square BEV images by cropping on the right
  grid_min_x_ = this->declare_parameter("grid_min_x", MIN_CROP); // enable non-square BEV images by cropping on the bottom
  grid_max_x_ = this->declare_parameter("grid_max_x", MAX_CROP); // enable non-square BEV images by cropping on the top
  camera_fov_ = this->declare_parameter("camera_fov", 110.0);
  intensity_threshold_ = this->declare_parameter("intensity_threshold", 0.05);
  planes_ = this->declare_parameter("planes", 64);
  h_res_ = this->declare_parameter("h_res", 0.2);
  cell_size_ = this->declare_parameter("cell_size", 0.1);
  ground_cell_span_ = this->declare_parameter("ground_cell_span", 40);
  cell_size_height_map_ = this->declare_parameter("cell_size_height_map", 0.25);
  height_threshold_ = this->declare_parameter("height_threshold", 0.10);
  max_height_ = this->declare_parameter("max_height", 3.0);
  grid_dim_ = this->declare_parameter("grid_dim", 1000);
  low_opening_ = this->declare_parameter("low_opening", 24.9);
  v_res_ = this->declare_parameter("v_res", 0.4);
  split_channels_ = this->declare_parameter("split_channels", false); // If True, force save channels as different images
  get_ground_ = this->declare_parameter("get_ground", false); // Generate the ground estimation file
  max_expected_intensity_ = this->declare_parameter("max_expected_intensity", 1.0f);
  remove_floor_ = this->declare_parameter("remove_floor", false);
  std::string cloud_topic;
  cloud_topic = this->declare_parameter("cloud_topic", "/kitti/velo/pointcloud");
  std::string output_topic;
  output_topic = this->declare_parameter("output_topic", "filtered_cloud");
  std::string lidar_tf_frame;
  lidar_tf_frame = this->declare_parameter("lidar_tf_frame", "/velo_link");
  std::string camera_tf_frame;
  camera_tf_frame = this->declare_parameter("camera_tf_frame", "/camera_color_left");
  kitti_dir_ = this->declare_parameter("kitti_dir", "kitti");
  split_dir_ = this->declare_parameter("split_dir", "training");
  saving_path_ = this->declare_parameter("saving_path", "bev_images");
  package_dir_ = this->declare_parameter("package_dir", "package");

  if(split_dir_.compare("training") != 0 && split_dir_.compare("testing") != 0){
    std::cout << "[ERROR] Invalid split directory " << split_dir_ << std::endl;
    return;
  }

  // Init the lidar - camera transformation for the filter
  cloud_filter_->setIntensityNormalization(max_expected_intensity_);
  this->initTF(lidar_tf_frame, camera_tf_frame);
  cloud_filter_->initMaxPointsMap(grid_dim_, cell_size_, 0, max_height_, num_slices_, 
    planes_, low_opening_, h_res_, v_res_);
  
  // Init private publishers/subscribers
  bird_view_pub_ = std::make_unique<image_transport::Publisher>(image_transport_->advertise("bird_view", 1));
  bird_ground_pub_ = std::make_unique<image_transport::Publisher>(image_transport_->advertise("bird_ground", 1));
  ground_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ground_cloud", 1);
  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 1);
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, 
      1, 
      std::bind(
        &BevLidarOfflineNode::cloud_callback, 
        this, 
        std::placeholders::_1));
}

void BevLidarOfflineNode::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) const
{
    // Change the intensity field name, so we can use it with pcl point type
    cloud_msg->fields[3].name = "intensity";

    // Convert cloud msg to pcl type
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg, *cloud_ptr);

    // Update the filter cloud
    cloud_filter_->setInputCloud(cloud_ptr);

    // Remove points out of the camera FOV
    //    filter.filterFOV(camera_fov);

    std::shared_ptr<cv::Mat> bird_ground = cloud_filter_->birdGround(cell_size_, ground_cell_span_, grid_dim_);
    // Remove floor points
    if(remove_floor_){
        cloud_filter_->removeFloor(cell_size_height_map_, height_threshold_, grid_dim_height_map_);
    }

    std::shared_ptr<cv::Mat> bird_view = cloud_filter_->birdView(cell_size_, max_height_, grid_dim_, false);

    int grid_cells = grid_dim_ / cell_size_; // Number of col/rows of the birdview

    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0; i<bird_ground->rows; i++){
        float* row_ptr = bird_ground->ptr<float>(i);
        for(int j=0; j<bird_ground->cols; j++){
            float z = row_ptr[j] - cloud_filter_->getBaseVeloTF().getOrigin().z();
            double x = (grid_cells/2. - i)*cell_size_;
            double y = (grid_cells/2. - j)*cell_size_;

            // Push the ground XYZ point
            pcl::PointXYZ point;
            point.x = x;
            point.y = y;
            point.z = z;
            ground_cloud_ptr->push_back(point);
        }
    }

    // Publish the ground pointcloud
    sensor_msgs::msg::PointCloud2 ground_ros;
    pcl::toROSMsg(*ground_cloud_ptr, ground_ros);
    ground_ros.header = cloud_msg->header;
    ground_cloud_pub_->publish(ground_ros);

    // Publish bird_view img
    cv_bridge::CvImage cv_bird_view;
    cv_bird_view.header = cloud_msg->header;
    cv_bird_view.encoding = "bgr8";
    cv_bird_view.image = *bird_view;
    bird_view_pub_->publish(cv_bird_view.toImageMsg());

    // Publish ground img
    cv_bridge::CvImage cv_ground_view;
    cv_ground_view.header = cloud_msg->header;
    cv_ground_view.encoding = "32FC1";
    cv_ground_view.image = *bird_ground;
    bird_ground_pub_->publish(cv_ground_view.toImageMsg());

    sensor_msgs::msg::PointCloud2 output_cloud_msg;
    pcl::toROSMsg(*cloud_ptr, output_cloud_msg);

    // Publish the filtered cloud
    cloud_pub_->publish(output_cloud_msg);
}

/* Wait for the transform lidar -> camera and update velo_cam_transform_ */
void BevLidarOfflineNode::initTF(std::string lidar_frame, std::string camera_frame){
    tfBuffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    bool tf_error = true;
    while(tf_error)
    {
        try
        {
          tf2::Stamped<tf2::Transform> velo_cam_transform;
          tf2::Stamped<tf2::Transform> base_velo_transform;
          geometry_msgs::msg::TransformStamped velo_cam_transform_msg;
          velo_cam_transform_msg = tfBuffer_->lookupTransform(lidar_frame, camera_frame, tf2::TimePointZero);
          tf2::fromMsg(velo_cam_transform_msg, velo_cam_transform);
          
          geometry_msgs::msg::TransformStamped base_velo_transform_msg;
          base_velo_transform_msg = tfBuffer_->lookupTransform("base_footprint", lidar_frame, tf2::TimePointZero);
          tf2::fromMsg(base_velo_transform_msg, base_velo_transform);
          tf_error = false;

          cloud_filter_->setVeloToBaseTransform(velo_cam_transform);
          cloud_filter_->setVeloToBaseTransform(base_velo_transform);
        }
        catch (tf2::TransformException ex)
        {
            // RCLCPP_INFO(
            // this->get_logger(), "Could not transform %s to %s: %s",
            // toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
            //throw (ex);
        }

    }
    // std::cout << "New transform: " << velo_cam_transform.getOrigin().x() << ", " << velo_cam_transform_.getOrigin().y() << std::endl;
}

void BevLidarOfflineNode::init(){
        // Init the lidar - base transformation for the filter
    tf2::Stamped<tf2::Transform> base_velo_transform;

    // WARNING: lidar height fixed
    base_velo_transform.setOrigin(tf2::Vector3(0.0, 0.0, 1.73));
    cloud_filter_->setVeloToBaseTransform(base_velo_transform);
    cloud_filter_->initMaxPointsMap(grid_dim_, cell_size_, 0, max_height_, num_slices_, planes_, low_opening_, h_res_, v_res_);
    std::cout << "Saving imgs in: " << saving_path_ << std::endl;
    mkdir(saving_path_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    // Load and store the tf transforms in a map
    // Key: timestamp
    // Value: transform
    std::cout << "Reading tf data..." << std::endl;
    std::map<tf2::TimePoint, tf2::Stamped<tf2::Transform>> tf_map;

    struct dirent *entry;
    std::string calib_dir = package_dir_ + "/" + kitti_dir_ + "/" + split_dir_ + "/calib/";
    std::cout << calib_dir << std::endl;
    DIR *dir = opendir(calib_dir.c_str());

    if(dir == NULL){
        std::cout << "[ERROR] Unable to read calibration files." << std::endl;
        std::cout << strerror(errno) << std::endl;
        return;
    }

    // Parse all calibration files and build corresponding map
    int tfcount = 0;
    while ((entry = readdir(dir)) != NULL) {
        std::string file_name = calib_dir + entry->d_name;
        tf2::Transform velo2cam = read_calib(file_name);

        std::string frame_num = file_name.erase(0, file_name.find_last_of("/") + 1);
        frame_num.erase(frame_num.find_last_of("."), std::string::npos);
        if (frame_num.length() != 6){
            continue;
        }

        tf2::Stamped<tf2::Transform> tf_transform(
                velo2cam, 
                tf2::TimePoint(std::chrono::seconds(stoi(frame_num))), 
                "stereo_camera");
        tf_map[tf_transform.stamp_] = tf_transform;
        ++tfcount;
    }

    if (tfcount == 0){
        std::cout << "[ERROR] Unable to read calibration files." << std::endl;
        return;
    }else{
        std::cout << "Finished processing calibration files " << std::endl;
    }

    std::cout << "Processing pointclouds..." << std::endl;

    // Build list of channels to compute
    std::vector<std::string> channel_names;
    if (min_height_ != -9999.9){
        channel_names.push_back("min_height");
    }
    channel_names.push_back("max_height");
    for (int i = 0; i < num_slices_; i++){
        channel_names.push_back("dchop_" + std::to_string(i) + "_" + std::to_string(i+1));
    }
    channel_names.push_back("avg_intensity");

    int frames = 0;
    std::string velo_dir = package_dir_ + "/" + kitti_dir_ + "/" + split_dir_ + "/velodyne/";
    // cout << velo_dir << endl;
    DIR *dir_velo= opendir(velo_dir.c_str());
    while ((entry = readdir(dir_velo)) != NULL) {
        std::string pc_dir = velo_dir + entry->d_name;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr = getPointCloud(pc_dir);

        std::string frame_num = pc_dir.erase(0, pc_dir.find_last_of("/") + 1);
        frame_num.erase(frame_num.find_last_of("."), std::string::npos);
        if (frame_num.length() != 6){
            continue;
        }

        if (cloud_ptr != NULL){
            bool already_computed = true;
            for (int i = 0; i < channel_names.size(); ++i){
                std::stringstream saving_absolute;
                saving_absolute << saving_path_ << "/";
                saving_absolute << std::setfill('0') << std::setw(6) << frame_num;
                saving_absolute << "_" << channel_names[i] << ".png";
                std::ifstream f(saving_absolute.str());
                if(!f.good()){
                    already_computed = false;
                    break;
                }
                f.close();
            }

            // Skip iteration if existing output
            if(already_computed){
                std::cout << "Skipping frame " << frame_num << std::endl;
                continue;
            }
            // Update the filter cloud
            cloud_filter_->setInputCloud(cloud_ptr);

            // Get the transform from the tf map at the cloud timestamp
            tf2::Stamped<tf2::Transform> velo_cam_transform = tf_map[tf2::TimePoint(std::chrono::seconds(stoi(frame_num)))];
            // Update lidar transform in filter
            cloud_filter_->setVeloToCamTransform(velo_cam_transform);

            // Remove points out of the camera FOV
            cloud_filter_->filterFOV(camera_fov_);

            // Remove floor points
            if(remove_floor_){
                cloud_filter_->removeFloor(cell_size_height_map_, height_threshold_, grid_dim_height_map_);
            }

            // Compute the birdview
            std::shared_ptr<cv::Mat> bird_view = cloud_filter_->birdView(cell_size_, max_height_, num_slices_, grid_dim_);

            cv::Mat final_birdview = bird_view->clone();

            // Crop to desired region
            // if(grid_min_x!=MIN_CROP || grid_min_y!=MIN_CROP || grid_max_x!=MAX_CROP || grid_max_y!=MAX_CROP){
            assert(grid_min_x_>=-grid_dim_/2.);
            assert(grid_min_y_>=-grid_dim_/2.);
            assert(grid_max_x_<=grid_dim_/2.);
            assert(grid_max_y_<=grid_dim_/2.);
            assert(grid_min_y_<grid_max_y_);
            assert(grid_min_x_<grid_max_x_);

            int x_max = final_birdview.rows/2.-grid_min_x_/cell_size_;
            int y_min = final_birdview.cols/2.+grid_min_y_/cell_size_;

            int h_pixels = (grid_max_x_-grid_min_x_)/cell_size_;
            int w_pixels = (grid_max_y_-grid_min_y_)/cell_size_;

            int x_min = x_max - h_pixels;

            final_birdview = final_birdview(cv::Rect(y_min, x_min, w_pixels, h_pixels));

            if(get_ground_){
                std::stringstream saving_ground_absolute;
                saving_ground_absolute << saving_path_ << "/ground_";
                saving_ground_absolute << std::setfill('0') << std::setw(6) << frame_num;
                saving_ground_absolute << ".txt";

                // Compute the ground birdview
                std::shared_ptr<cv::Mat> bird_ground = cloud_filter_->birdGround(cell_size_, ground_cell_span_, grid_dim_);
                cv::Mat final_birdground = bird_ground->clone();

                // Crop to desired region
                final_birdground = final_birdground(cv::Rect(y_min, x_min, w_pixels, h_pixels));

                // Save the ground bird image in disk
                std::ofstream fout(saving_ground_absolute.str());
                // cout << saving_ground_absolute.str() << endl;
                if(!fout){
                    std::cout<<"[ERROR] Unable to create ground file at " << saving_ground_absolute.str() << ". Skipping." << std::endl;
                }else{
                    for(int i=0; i<final_birdground.rows; i++)
                    {
                        for(int j=0; j<final_birdground.cols; j++)
                        {
                            fout<<final_birdground.at<float>(i,j)<<" ";
                        }
                        fout<<std::endl;
                    }
                }
                fout.close();
                bird_ground->release();
                final_birdground.release();
            }

            // Save birdview images
            std::vector<cv::Mat> channels;
            cv::split(final_birdview, channels);
            if (min_height_ == -9999.9){
                channels.erase(channels.begin()); // First channel is min_height
            }

            if (channels.size() == 3 && !split_channels_){
                // Single png image
                cv::Mat three_ch;
                cv::merge(channels, three_ch);

                std::stringstream saving_absolute;
                saving_absolute << saving_path_ << "/";
                saving_absolute << std::setfill('0') << std::setw(6) << frame_num;
                saving_absolute << ".png";

                cv::imwrite(saving_absolute.str(), three_ch);
            } else {
                // Save image channels separately
                for (int i = 0; i < channels.size(); ++i){
                    std::stringstream saving_absolute;
                    saving_absolute << saving_path_ << "/";
                    saving_absolute << std::setfill('0') << std::setw(6) << frame_num;
                    saving_absolute << "_" << channel_names[i] << ".png";
                    cv::imwrite(saving_absolute.str(), channels[i]);
                }
            }
            std::cout << "Finished processing frame " <<  frame_num << std::endl;

            // Increase index
            final_birdview.release();
            bird_view->release();
            frames++;
        }
    }

    if (frames == 0){
        std::cout << "[ERROR] Unable to process lidar files." << std::endl;
        return;
    }

    std::cout << "[SUCCESS] Process fininshed. Total number of frames: " << frames << std::endl;
}


tf2::Transform BevLidarOfflineNode::read_calib(std::string calib_name){
    sensor_msgs::msg::CameraInfo camera_info_left, camera_info_right;

    tf2::Matrix3x3 velo2cam_rot_;
    tf2::Vector3 velo2cam_origin_;
    // tf::Quaternion velo2cam_quat_;

    // tf::Matrix3x3 imu2velo_rot_;
    // tf::Vector3 imu2velo_origin_;

    FILE * calib_file = fopen(calib_name.c_str(),"r");
    char str[255];
    double trash;
    double P[12], P2[12];
    int read = fscanf(calib_file, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                       str,
                       &trash,    &trash,    &trash,    &trash,
                       &trash,    &trash,    &trash,    &trash,
                       &trash,    &trash,    &trash,    &trash);
    read = fscanf(calib_file, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                   str,
                   &trash,    &trash,    &trash,    &trash,
                   &trash,    &trash,    &trash,    &trash,
                   &trash,    &trash,    &trash,    &trash);
    read = fscanf(calib_file, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                   str,
                   &P[0],    &P[1],    &P[2],    &P[3],
                   &P[4],    &P[5],    &P[6],    &P[7],
                   &P[8],    &P[9],    &P[10],   &P[11]);
    for (int ix=0; ix<12; ix++){
        camera_info_left.p[ix] = P[ix];
    }

    read = fscanf(calib_file, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                   str,
                   &P2[0],    &P2[1],    &P2[2],    &P2[3],
                   &P2[4],    &P2[5],    &P2[6],    &P2[7],
                   &P2[8],    &P2[9],    &P2[10],   &P2[11]);
    for (int ix=0; ix<12; ix++){
        camera_info_right.p[ix] = P2[ix];
    }

    camera_info_right.p[3] -= camera_info_left.p[3];


    //Velo To cam
    //R0_rect
    double R0[9];
    read = fscanf(calib_file, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                   str,
                   &R0[0],    &R0[1],    &R0[2],
                   &R0[3],    &R0[4],    &R0[5],
                   &R0[6],    &R0[7],    &R0[8]);

    //Tr_velo_to_cam
    double TR[12];
    read = fscanf(calib_file, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                   str,
                   &TR[0],    &TR[1],    &TR[2],    &TR[3],
                   &TR[4],    &TR[5],    &TR[6],    &TR[7],
                   &TR[8],    &TR[9],    &TR[10],   &TR[11]);

    velo2cam_rot_.setValue(TR[0], TR[1], TR[2],
                           TR[4], TR[5], TR[6],
                           TR[8], TR[9], TR[10]);

    velo2cam_origin_.setValue(TR[3]-P[3]/P[0], TR[7]-P[7]/P[5], TR[11]-P[11]);
    // velo2cam_rot_.getRotation(velo2cam_quat_);
    tf2::Transform velo2cam;
    velo2cam.setOrigin(velo2cam_origin_);
    velo2cam.setBasis(velo2cam_rot_);

    // double IMU[12];
    // fscanf(calib_file, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
    //                str,
    //                &IMU[0],    &IMU[1],    &IMU[2],    &IMU[3],
    //                &IMU[4],    &IMU[5],    &IMU[6],    &IMU[7],
    //                &IMU[8],    &IMU[9],    &IMU[10],   &IMU[11]);

    // imu2velo_rot_.setValue(IMU[0], IMU[1], IMU[2],
    //                        IMU[4], IMU[5], IMU[6],
    //                        IMU[8], IMU[9], IMU[10]);

    // imu2velo_origin_.setValue(IMU[3], IMU[7], IMU[11]);


    fclose(calib_file);
    return velo2cam;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr BevLidarOfflineNode::getPointCloud(std::string path){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    std::fstream file(path.c_str(), std::ios::in | std::ios::binary);
    if(file.good()){
        file.seekg(0, std::ios::beg);
        int i;
        for (i = 0; file.good() && !file.eof(); i++) {
            pcl::PointXYZI point;
            file.read((char *) &point.x, 3*sizeof(float));
            file.read((char *) &point.intensity, sizeof(float));
            cloud->push_back(point);
        }
        file.close();
    }

    return cloud;
}

}  // namespace bev_lidar

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(bev_lidar::BevLidarOfflineNode)
