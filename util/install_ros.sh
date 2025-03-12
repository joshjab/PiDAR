#!/bin/bash

# ===============================================
# ROS 2 Humble Installation Script for Ubuntu 22.04 Server on Raspberry Pi 3B+ and newer
# Author: Joshua Jablonowski
# Date: 02/15/2025
# ===============================================

# Stop on any error
set -e

echo "Starting ROS 2 Humble installation on Ubuntu 22.04 Server (Raspberry Pi 3B+)"

# Prevent sudo timeout
sudo -v # ask for sudo password up-front
while true; do
  # Update user's timestamp without running a command
  sudo -nv; sleep 1m
  # Exit when the parent process is not running any more. In fact this loop
  # would be killed anyway after being an orphan(when the parent process
  # exits). But this ensures that and probably exit sooner.
  kill -0 $$ 2>/dev/null || exit
done &

# ------------------------------------------------
# Step 1: Update & Upgrade the System
# ------------------------------------------------
echo "Updating and upgrading system packages..."
sudo apt update
sudo apt upgrade -y

# ------------------------------------------------
# Step 2: Install Required System Dependencies and Docker for Webviz
# ------------------------------------------------
echo "Installing essential dependencies..."
sudo apt install -y \
    curl \
    gnupg \
    lsb-release \
    build-essential \
    python3 \
    x11-apps xauth xorg openbox \
    net-tools \
    htop \
    git \
    docker 

# ------------------------------------------------
# Step 3: Add ROS 2 Repositories and Keys
# ------------------------------------------------
echo "Adding ROS 2 Humble repository and keys..."
# Import the ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the ROS 2 repository
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list

# ------------------------------------------------
# Step 4: Install ROS 2 Humble
# ------------------------------------------------
echo "Updating package lists and installing ROS 2 Humble..."
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools ros-humble-cartographer ros-humble-cartographer-ros
sudo apt install -y ros-humble-foxglove-bridge
# ------------------------------------------------
# Step 5: Initialize rosdep (Dependency Manager)
# ------------------------------------------------
echo "Installing and Initializing rosdep..."
sudo apt install -y python3-rosdep
sudo rosdep init || echo "rosdep already initialized"
rosdep update

# ------------------------------------------------
# Step 6: Configure Environment Variables
# ------------------------------------------------
echo "Adding ROS 2 environment setup to .bashrc..."
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi
source ~/.bashrc

# ------------------------------------------------
# Step 7: Configure SSH X11 Forwarding
# ------------------------------------------------
echo "Configuring SSH for X11 forwarding..."
SSH_CONFIG="/etc/ssh/sshd_config"
sudo sed -i 's/^#X11Forwarding no/X11Forwarding yes/' "$SSH_CONFIG"
sudo sed -i 's/^#X11UseLocalhost yes/X11UseLocalhost yes/' "$SSH_CONFIG"

# ------------------------------------------------
# Step 8: Network Optimization for ROS 2
# ------------------------------------------------
echo "Configuring network settings for ROS 2 DDS communication..."
if ! grep -q "ROS_DOMAIN_ID" ~/.bashrc; then
    echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
    echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
fi
source ~/.bashrc

# ------------------------------------------------
# Step 9: Test the Installation
# ------------------------------------------------
echo "Testing ROS 2 Humble installation..."
# Run turtlesim node and list topics
ros2 run turtlesim turtlesim_node & sleep 5
ros2 topic list

# ------------------------------------------------
# Step 10: Display Completion Message
# ------------------------------------------------
echo "🎉 ROS 2 Humble installation complete on Raspberry Pi 3B+! 🎉"
echo "Rebooting the system to apply all changes..."

sudo reboot
