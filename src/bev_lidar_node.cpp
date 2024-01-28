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

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "bev_lidar/cloud_filter/cloud_filter.hpp"
#include "bev_lidar/bev_lidar_node.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>

namespace bev_lidar
{

BevLidarNode::BevLidarNode(const rclcpp::NodeOptions & options)
:  Node("bev_lidar", options)
{
  // Init object with logic
  bev_lidar_ = std::make_unique<bev_lidar::BevLidar>();
  cloud_filter_ = std::make_unique<cloud_filter::CloudFilter>();
  
  // Init all params
  camera_fov_ = this->declare_parameter("camera_fov", 110.0);
  intensity_threshold_ = this->declare_parameter("intensity_threshold", 0.05);
  planes_ = this->declare_parameter("planes", 32);
  h_res_ = this->declare_parameter("h_res", 0.0034889);
  cell_size_ = this->declare_parameter("cell_size", 0.1);
  ground_cell_span_ = this->declare_parameter("ground_cell_span", 40);
  cell_size_height_map_ = this->declare_parameter("cell_size_height_map", 0.25);
  height_threshold_ = this->declare_parameter("height_threshold", 0.10);
  max_height_ = this->declare_parameter("max_height", 3.0);
  grid_dim_ = this->declare_parameter("grid_dim", 1000);
  low_opening_ = this->declare_parameter("low_opening", 24.9);
  v_res_ = this->declare_parameter("v_res", 0.4);
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
        &BevLidarNode::cloud_callback, 
        this, 
        std::placeholders::_1));
}

void BevLidarNode::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) const
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
void BevLidarNode::initTF(std::string lidar_frame, std::string camera_frame){
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

}  // namespace bev_lidar

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(bev_lidar::BevLidarNode)
