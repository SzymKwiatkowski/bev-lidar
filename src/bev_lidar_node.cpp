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

namespace bev_lidar
{

BevLidarNode::BevLidarNode(const rclcpp::NodeOptions & options)
:  Node("bev_lidar", options)
{
  bev_lidar_ = std::make_unique<bev_lidar::BevLidar>();
  cloud_filter_ = std::make_unique<cloud_filter::CloudFilter>();
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

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
}

}  // namespace bev_lidar

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(bev_lidar::BevLidarNode)
