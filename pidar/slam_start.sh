#!/bin/bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source /home/joshjab/ydlidar_ros2_driver/install/setup.bash
source /home/joshjab/ros2_ws/install/setup.bash

ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 &
#ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py &
#ros2 run rf2o_laser_odometry rf2o_laser_odometry_node &
#ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link &
#ros2 launch slam_toolbox online_async_launch.py &
ros2 run cartographer_ros cartographer_node -configuration_directory /opt/ros/humble/share/cartographer/configuration_files -configuration_basename pidar_2d.lua & 
ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.05 &
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
