## How to run
### To clip lidar and reduce bag size
```
<!-- ros2 run lidar_clipper lidar_clipper_node -->
ros2 launch lidar_clipper lidar_clipper_launch.py
```
### To publish pcd file to a ros2 topic
```
ros2 run lidar_clipper pcd_publisher_node --ros-args -p pcd_file_path:=/path/to/your/pcd/file.pcd
```
