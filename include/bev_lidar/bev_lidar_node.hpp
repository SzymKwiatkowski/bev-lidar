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

#ifndef BEV_LIDAR__BEV_LIDAR_NODE_HPP_
#define BEV_LIDAR__BEV_LIDAR_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "bev_lidar/bev_lidar.hpp"

namespace bev_lidar
{
using BevLidarPtr = std::unique_ptr<bev_lidar::BevLidar>;

class BEV_LIDAR_PUBLIC BevLidarNode : public rclcpp::Node
{
public:
  explicit BevLidarNode(const rclcpp::NodeOptions & options);

private:
  BevLidarPtr bev_lidar_{nullptr};
  double camera_fov_{110.0};
  double intensity_threshold_{0.05};
  int16_t planes_{32};
  double h_res_{0.0034889};
  double cell_size_{0.1};
  int ground_cell_span_{40};
  double height_threshold_;
  double max_height_;
  double cell_size_height_map_;
  int grid_dim_;
  int grid_dim_height_map_;
  int num_slices_;
  bool remove_floor_;
  double low_opening_;
  double v_res_;
  float max_expected_intensity_;
};
}  // namespace bev_lidar

#endif  // BEV_LIDAR__BEV_LIDAR_NODE_HPP_