#!/bin/bash

# /usr/local/bin/rtabmap_start_neo.sh

export CMAKE_PREFIX_PATH="/opt/rosslam_ws/devel/:/opt/ros/noetic"
export LD_LIBRARY_PATH="/opt/rosslam_ws/devel//lib:/opt/ros/noetic/lib"
export PATH="/opt/ros/noetic/bin:/usr/local/bin:/usr/bin:/bin:/usr/local/games:/usr/games"
export PKG_CONFIG_PATH="/opt/rosslam_ws/devel/devel/lib/pkgconfig:/opt/ros/noetic/lib/pkgconfig"
export PYTHONPATH="/opt/rosslam_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages"
export ROSLISP_PACKAGE_DIRECTORIES="/opt/rosslam_ws/devel/share/common-lisp"
export ROS_DISTRO="noetic"
export ROS_ETC_DIR="/opt/ros/noetic/etc/ros"
export ROS_MASTER_URI="http://localhost:11311"
export ROS_PACKAGE_PATH="/opt/rosslam_ws/src:/opt/ros/noetic/share"
export ROS_PYTHON_VERSION="2"
export ROS_ROOT="/opt/ros/noetic/share/ros"
export ROS_VERSION="1"

export ROS_MASTER_URI=http://192.168.32.198:11311
export ROS_IP=192.168.32.198

if [ -n "$1" ]
then
        SENSOR_HOSTNAME="sensor_hostname:=$1"

else
        SENSOR_HOSTNAME=""
fi

if [ -n "$2" ]
then
        UDP_DEST="udp_dest:=$2"
else
        UDP_DEST=""
fi

roslaunch rtabmap_ros ouster_neo.launch $SENSOR_HOSTNAME $UDP_DEST

#rosrun frontiers_exploration slamsharedvelocity
#roslaunch frontiers_exploration robot_neo.launch
