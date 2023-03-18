#!/bin/bash

# /usr/local/bin/rtabmap_start_neo.sh

export CMAKE_PREFIX_PATH="/home/everis/ros/catkin_ws/devel:/opt/ros/melodic"
export LD_LIBRARY_PATH="/home/everis/ros/catkin_ws/devel/lib:/opt/ros/melodic/lib"
export PATH="/opt/ros/melodic/bin:/usr/local/bin:/usr/bin:/bin:/usr/local/games:/usr/games"
export PKG_CONFIG_PATH="/home/everis/ros/catkin_ws/devel/lib/pkgconfig:/opt/ros/melodic/lib/pkgconfig"
export PYTHONPATH="/home/everis/ros/catkin_ws/devel/lib/python2.7/dist-packages:/opt/ros/melodic/lib/python2.7/dist-packages"
export ROSLISP_PACKAGE_DIRECTORIES="/home/everis/ros/catkin_ws/devel/share/common-lisp"
export ROS_DISTRO="melodic"
export ROS_ETC_DIR="/opt/ros/melodic/etc/ros"
export ROS_MASTER_URI="http://localhost:11311"
export ROS_PACKAGE_PATH="/home/everis/ros/catkin_ws/src:/opt/ros/melodic/share"
export ROS_PYTHON_VERSION="2"
export ROS_ROOT="/opt/ros/melodic/share/ros"
export ROS_VERSION="1"

export ROS_MASTER_URI=http://192.168.32.198:11311
export ROS_IP=192.168.32.198
roslaunch rtabmap_ros ouster_neo.launch

#rosrun frontiers_exploration slamsharedvelocity
#roslaunch frontiers_exploration robot_neo.launch
