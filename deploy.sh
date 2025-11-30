#!/bin/bash

# Variables
SRC_DIR="/home/misyaath/projects/fp_rp_lidar/src/"
DEST_USER="ras"
DEST_IP="192.168.1.5"  # Replace with actual IP
DEST_DIR="/home/ras/fp_rp_lidar/src"


rsync -avz --exclude 'rplidar_sdk/.git'  --exclude 'rplidar_sdk/obj' --exclude 'rplidar_sdk/output' "$SRC_DIR" "${DEST_USER}@${DEST_IP}:${DEST_DIR}"

#ssh "${DEST_USER}@${DEST_IP}" << 'EOF'
#  source /opt/ros/jazzy/setup.bash
#  cd /home/ras/fp_rp_lidar
#
#  # Build the project
#  colcon build
#
#  # Remove unwanted files (adjust as needed)
#  rm -rf src .git build log
#
#  # Source the setup
#  source install/setup.bash
#
#  # Run publisher nodes (replace with actual commands)
##  ros2 run motor_controller motor_driver_node
#
#  # Wait a bit and then return
#  sleep 5
#  exit
#EOF