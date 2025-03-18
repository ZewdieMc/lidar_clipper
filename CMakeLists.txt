cmake_minimum_required(VERSION 3.8)
project(lidar_clipper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

# Add Eigen dependency (required for custom point types)
find_package(Eigen3 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Include directories
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Add lidar_clipper_node
add_executable(lidar_clipper_node src/lidar_clipper_node.cpp)
ament_target_dependencies(lidar_clipper_node
  rclcpp
  sensor_msgs
  rosbag2_cpp
  pcl_conversions
  PCL
)
target_link_libraries(lidar_clipper_node
  ${PCL_LIBRARIES}
  pcl_common
  pcl_io
  pcl_filters
)

# Add pcd_publisher_node
add_executable(pcd_publisher_node src/publish_pcd.cpp)
ament_target_dependencies(pcd_publisher_node
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
)
target_link_libraries(pcd_publisher_node
  ${PCL_LIBRARIES}
  pcl_common
  pcl_io
)

# Install targets
install(TARGETS
  lidar_clipper_node
  pcd_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()