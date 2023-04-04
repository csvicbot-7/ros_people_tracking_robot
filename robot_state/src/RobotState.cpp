#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <cmath>
#include <cstdlib>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <bits/stdc++.h>

#include "rossharedmemory.h"
#include "rossharedmemoryRobotState.h"

#include "robot_state/wheelSpeed.h"

//#include "robot_state/RobotState.h"
#include "nav_msgs/Odometry.h"

double m_fpLength = 3.0;
double m_fpWidth = 3.0;
double m_fpHeight = 5.0;
double m_tolerance = 0.05;
double m_wheelBase = 0.70;
int m_firstIter = 0;

ros::Publisher m_odomFeedback;
ros::Publisher m_wheelFeedback;

robot_state::wheelSpeed m_feedback;

/* Not working ....
void odometryCallback(const nav_msgs::Odometry& msg);

void odometryCallback(const nav_msgs::Odometry& odomRtabmap) {

	nav_msgs::Odometry msg = odomRtabmap;
	float linearX = (m_feedback.wheelSpeedLeft + m_feedback.wheelSpeedRight) / 2.0f;

	// Test 1: filter direction
	if (linearX == 0)
		msg.twist.twist.linear.x = 0;
	else if (msg.twist.twist.linear.x < 0 && linearX >= 0)
		msg.twist.twist.linear.x = 0;
	else if (msg.twist.twist.linear.x > 0 && linearX <= 0)
		msg.twist.twist.linear.x = 0;

	// Test 2: use wheels speed
	//msg.twist.twist.linear.x = linearX;

	// Test 3: use both 
	//msg.twist.twist.linear.x = (msg.twist.twist.linear.x + linearX) / 2.0f;

	float angularZ = (m_feedback.wheelSpeedLeft - m_feedback.wheelSpeedRight) / m_wheelBase;
	// Test 1: filter direction
	if (angularZ == 0)
		msg.twist.twist.angular.z = 0;
	else if (msg.twist.twist.angular.z < 0 && angularZ >= 0)
		msg.twist.twist.angular.z = 0;
	else if (msg.twist.twist.angular.z > 0 && angularZ <= 0)
		msg.twist.twist.angular.z = 0;

	// Test 2: use wheels speed
	//msg.twist.twist.angular.z = angularZ;

	// Test 3: use both 
	//msg.twist.twist.angular.z = (msg.twist.twist.angular.z + angularZ) / 2.0f;

	m_odomFeedback.publish(msg);
}
*/

int main(int argc, char** argv){
	ros::init(argc, argv, "RobotState");

	tf::TransformBroadcaster m_br;
	tf::Transform m_transform;

	stSharedRtoRS m_sRtoRS;
	CRosSharedMemory<stSharedRtoRS, void*> m_shmMemory(SHARED_NAME_ROBOT_STATE, false);

	ros::NodeHandle n;
	//ros::Subscriber odom           =  n.subscribe("rtabmap/odom", 1000, &odometryCallback);

	m_wheelFeedback = n.advertise<robot_state::wheelSpeed>("aunav_velocity_feedback/cmd_wheel", 200);
	m_odomFeedback  = n.advertise<nav_msgs::Odometry>("aunav_velocity_feedback/odom", 200);

	ros::Rate rate(10);

	double lidarTfX, lidarTfY, lidarTfZ, lidarTfYaw;
	
	while (ros::ok()){
		memset(&m_sRtoRS , 0, sizeof(stSharedRtoRS));
		bool bLock = m_shmMemory.getData(m_sRtoRS);
		//ROS_INFO_ONCE("Start Robot State");
		//ROS_INFO_STREAM("bLock: " << bLock);

		if (bLock) {
			lidarTfX = static_cast<double>(m_sRtoRS.lidarTfX);
			lidarTfY = static_cast<double>(m_sRtoRS.lidarTfY);
			lidarTfZ = static_cast<double>(m_sRtoRS.lidarTfZ);
			lidarTfYaw = static_cast<double>(m_sRtoRS.lidarTfYaw);

			double fpHeight = static_cast<double>(m_sRtoRS.fpHeight);
			double fpLength = static_cast<double>(m_sRtoRS.fpLength);
			double fpWidth = static_cast<double>(m_sRtoRS.fpWidth);

			if (abs(fpHeight - m_fpHeight) > m_tolerance
				|| abs(fpLength - m_fpLength) > m_tolerance
				|| abs(fpWidth - m_fpWidth) > m_tolerance) {

				string str;
				const char *command = nullptr;
				str = "rosparam set /rtabmap/rtabmap/Grid/FootprintHeight ";
				m_fpHeight = fpHeight;
				str = str + to_string(m_fpHeight);
				command = str.c_str();
				int result = system(command);
				if (result != 0)
					ROS_WARN("RobotState: Set footprint height returned %d", result);

				str = "rosparam set /rtabmap/rtabmap/Grid/FootprintLength ";
				m_fpLength = fpLength;
				str = str + to_string(m_fpLength);
				command = str.c_str();
				result = system(command);
				if (result != 0)
					ROS_WARN("RobotState: Set footprint length returned %d", result);

				str = "rosparam set /rtabmap/rtabmap/Grid/FootprintWidth ";
				m_fpWidth = fpWidth;
				str = str + to_string(m_fpWidth);
				command = str.c_str();
				result = system(command);
				if (result != 0)
					ROS_WARN("RobotState: Set footprint width returned %d", result);

				str = "rosservice call /rtabmap/update_parameters";
				command = str.c_str();
				result = system(command);
				if (result != 0)
					ROS_WARN("RobotState: Update parameters returned %d", result);
			}

			m_feedback.wheelSpeedLeft = m_sRtoRS.vtankL;
			m_feedback.wheelSpeedRight = m_sRtoRS.vtankR;
			//ROS_INFO_STREAM("vtankL: " << m_sRtoRS.vtankL);
			//ROS_INFO_STREAM("vtankR: " << m_sRtoRS.vtankR);
			m_wheelFeedback.publish(m_feedback);

			nav_msgs::Odometry odom;

			float linearX = (m_sRtoRS.vtankL + m_sRtoRS.vtankR) / 2.0f;
			odom.twist.twist.linear.x = linearX;
			odom.twist.twist.linear.y = 0;
			odom.twist.twist.linear.z = 0;

			float angularZ = (m_sRtoRS.vtankL - m_sRtoRS.vtankR) / m_wheelBase;
			odom.twist.twist.angular.x = 0;
			odom.twist.twist.angular.y = 0;
			odom.twist.twist.angular.z = angularZ;
			m_odomFeedback.publish(odom);

		} else {
			ROS_WARN("RobotState: bLock = false");
			lidarTfX = 0.35;
			lidarTfY = -0.2;
			lidarTfZ = 1.15;
			lidarTfYaw = 0;
		}
/*		ROS_INFO_STREAM("lidarTfX: " << lidarTfX);
		ROS_INFO_STREAM("lidarTfY: " << lidarTfY);
		ROS_INFO_STREAM("lidarTfZ: " << lidarTfZ);
		ROS_INFO_STREAM("lidarTfYaw: " << lidarTfYaw);
		ROS_INFO_STREAM("m_fpHeight: " << m_fpHeight);
		ROS_INFO_STREAM("m_fpLength: " << m_fpLength);
		ROS_INFO_STREAM("m_fpWidth: " << m_fpWidth); */
		m_transform.setOrigin( tf::Vector3(lidarTfX, lidarTfY, lidarTfZ) );
		m_transform.setRotation( tf::createQuaternionFromRPY(0, 0, -lidarTfYaw) );
		m_br.sendTransform(
			tf::StampedTransform(
				m_transform, ros::Time::now(), "base_link", "os_sensor"));

		rate.sleep();
	}
	return 0;
}