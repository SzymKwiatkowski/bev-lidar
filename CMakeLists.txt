cmake_minimum_required(VERSION 3.14)
project(bev_lidar)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/bev_lidar/bev_lidar.hpp
  include/bev_lidar/visibility_control.hpp
  src/bev_lidar.cpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  include/bev_lidar/bev_lidar_node.hpp
  src/bev_lidar_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "bev_lidar::BevLidarNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_bev_lidar
    test/test_bev_lidar.cpp
  )
  ament_target_dependencies(test_bev_lidar ${TEST_SOURCES})
  target_link_libraries(test_bev_lidar ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
