sudo cp ~/rosslam_ws/src/rtabmap_ros/launch/rtabmap_start_neo.sh /usr/local/bin/
sudo cp ~/rosslam_ws/src/rtabmap_ros/launch/rtabmap_start_next.sh /usr/local/bin/
sudo cp ~/rosslam_ws/src/rtabmap_ros/launch/robot_start_neo.sh /usr/local/bin/
#o
sudo cp ~/ros/ros_slam/rtabmap_ros/launch/rtabmap_start_neo.sh /usr/local/bin/
sudo cp ~/ros/ros_slam/rtabmap_ros/launch/rtabmap_start_next.sh /usr/local/bin/
sudo cp ~/ros/ros_slam/rtabmap_ros/launch/robot_start_neo.sh /usr/local/bin/


mkdir -p ~/.config/systemd/user/
cp ~/rosslam_ws/src/serviceFiles/*.service ~/.config/systemd/user/
#o
cp ~/ros/ros_slam/serviceFiles/*.service ~/.config/systemd/user/

systemctl --user daemon-reload

#systemctl --user enable --now roscore_service.service
#systemctl --user status roscore_service.service
#systemctl --user start roscore_service.service
