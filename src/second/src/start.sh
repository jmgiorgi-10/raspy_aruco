#!/bin/bash
sleep 1
cd ~/src/Firmware
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
sleep 0.5
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
sleep 0.5
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
sleep 0.5
roslaunch px4 mavros_posix_sitl.launch

#roslaunch second master.launch
#rosrun image_view image_view image:=/aruco_image