#!/bin/bash

# ------------------------------------------------
# Step 1: Update and Pull Submodules
# ------------------------------------------------
git submodule update --init --recursive

# ------------------------------------------------
# Step 2: Build YDLidar SDK
# ------------------------------------------------
cd ../submodules/YDLidar_SDK && mkdir build
cd build
cmake ..
make
sudo make install

# ------------------------------------------------
# Step 3: Build YDLidar ROS2 Driver
# ------------------------------------------------
cd ../ydlidar_ros2_driver
colcon build --symlink-install --exectutor sequential # Can't build with a ton of parellel processing on RPI3

# ------------------------------------------------
# Step 4: Set Environment Variables
# ------------------------------------------------
echo "source $PWD/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# ------------------------------------------------
# Step 5: Create Serial Port Aliases
# ------------------------------------------------
chmod 0777 ./src/ydlidar_ros2_driver/startup/*
sudo sh src/ydlidar_ros2_driver/startup/initenv.sh