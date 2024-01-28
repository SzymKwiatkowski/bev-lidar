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

#ifndef BEV_LIDAR_OFFLINE__BEV_LIDAR_NODE_HPP_
#define BEV_LIDAR_OFFLINE__BEV_LIDAR_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rclcpp/node.hpp"
#include "bev_lidar/bev_lidar.hpp"
#include "bev_lidar/cloud_filter/cloud_filter.hpp"

namespace bev_lidar
{
class BEV_LIDAR_PUBLIC BevLidarOfflineNode : public rclcpp::Node
{
public:
  explicit BevLidarOfflineNode(const rclcpp::NodeOptions & options);

private:
  std::unique_ptr<bev_lidar::BevLidar> bev_lidar_{nullptr};
  std::unique_ptr<cloud_filter::CloudFilter> cloud_filter_{nullptr};
  std::unique_ptr<image_transport::ImageTransport> image_transport_{nullptr};
  std::unique_ptr<image_transport::Publisher> bird_view_pub_{nullptr};
  std::unique_ptr<image_transport::Publisher> bird_ground_pub_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  
  // Trasform listener to get the TFs
  std::shared_ptr<tf2_ros::TransformListener> tf_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;

  double camera_fov_{110.0};
  double intensity_threshold_{0.05};
  int16_t planes_{32};
  double h_res_{0.0034889};
  double cell_size_{0.1};
  int ground_cell_span_{40};
  double height_threshold_{0.1};
  double max_height_{3.0};
  double min_height_{-9999.9};
  double cell_size_height_map_{0.25};
  int grid_dim_{1000};
  int grid_dim_height_map_{300};
  int num_slices_{3};
  bool remove_floor_{false};
  double low_opening_{24.9};
  double v_res_{0.4};
  float max_expected_intensity_{1.0f};
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) const;
  /* Wait for the transform lidar -> camera and update velo_cam_transform_ */
  void initTF(std::string lidar_frame, std::string camera_frame);
  void init();
  pcl::PointCloud<pcl::PointXYZI>::Ptr getPointCloud(std::string path);
  tf2::Transform read_calib(std::string calib_name);
  int grid_min_x_, grid_max_x_, grid_min_y_, grid_max_y_;
  bool split_channels_;
  bool get_ground_;
  std::string kitti_dir_;
  std::string split_dir_;
  std::string saving_path_;
  std::string package_dir_;

  const int MAX_CROP = 70;
  const int MIN_CROP = -70;
};
}  // namespace bev_lidar

#endif  // BEV_LIDAR_OFFLINE__BEV_LIDAR_NODE_HPP_
