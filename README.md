# bev_lidar
<!-- Required -->
<!-- Package description -->

## Installation
<!-- Required -->
<!-- Things to consider:
    - How to build package? 
    - Are there any other 3rd party dependencies required? -->

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On --packages-up-to bev_lidar
```

## Usage
<!-- Required -->
<!-- Things to consider:
    - Launching package. 
    - Exposed API (example service/action call. -->

```bash
ros2 launch bev_lidar bev_lidar.launch.py
```

## API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

### Input

| Name         | Type                  | Description  |
| ------------ | --------------------- | ------------ |
| `topic_name` | std_msgs::msg::String | Sample desc. |

### Output

| Name         | Type                  | Description  |
| ------------ | --------------------- | ------------ |
| `topic_name` | std_msgs::msg::String | Sample desc. |

### Services and Actions

| Name           | Type                   | Description  |
| -------------- | ---------------------- | ------------ |
| `service_name` | std_srvs::srv::Trigger | Sample desc. |

### Parameters

| Name         | Type | Description  |
| ------------ | ---- | ------------ |
| `param_name` | int  | Sample desc. |

## Run configuration

### Bird's Eye View parameters
There are many parameters to configure the BEV. Default values lead to results published in BirdNet and BirdNet+. These are: *camera_fov*, *planes*, *h_res*, *v_res*, *low_opening*, *cell_size*, *grid_dim*, *grid_min_x*, *grid_max_x*, *grid_min_y*, *grid_max_y*, *max_height*, *min_height*, *num_slices*, *height_threshold*, *cell_size_height_map*, *grid_dim_height_map*, *get_ground*, *remove_floor*.

### Offline mode
* *kitti_dir*: Path to `kitti/object` folder.
* *split_dir*: KITTI split set. Can be *training* or *testing*.
* *saving_path*: folder name (or full path) to save resulting images. If no full path is provided, the folder will be created in `$HOME/.ros/`

### Online mode (ROS node)
* *lidar_tf_frame*: name of the LiDAR frame
* *camera_tf_frame*: name of the camera frame

#### Inputs (Subscribed Topics)
* *cloud_topic* ([sensor_msgs::PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html)): LiDAR pointcloud

#### Outputs (Published Topics)
* *bird_view* ([sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html)): BEV image
* *bird_ground* ([sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html)): Image representing the ground estimation
* *ground_cloud* ([sensor_msgs::PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html)): Pointcloud to visualize the ground estimation in Rviz.

# TODO
Things to do after this is working:
- refactor of dependencies with proper arch
- cleanup
- adding launch files for launching specific functionality
- redo scripts