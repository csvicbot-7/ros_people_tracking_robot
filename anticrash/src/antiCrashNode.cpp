#include <ros/ros.h>
#include <antiCrash.h>

int main (int argc, char** argv){
  ros::init(argc, argv, "antiCrash");

  auto m_anticrash = antiCrash();

  ros::spin();

  return 0;
}