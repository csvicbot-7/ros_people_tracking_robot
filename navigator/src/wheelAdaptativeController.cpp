//#include "rossharedmemory.h"
//#include "rossharedmemoryEtoR.h"
#include <ros/ros.h>
#include "wheelSpeed.h"
#include "nav_msgs/Odometry.h"
#include <stdio.h>
#include <stdlib.h>
#include <tf/tf.h>

#include <termios.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <math.h>

#define TURN_PID_P            0.01
#define TURN_PID_I            0.00
#define TURN_PID_D            0.00
#define TURN_PID_MAX_ERROR   10.00

using namespace std;

//FlipperMode myMode;

bool stopRobot = false;

void publishOdom(ros::Publisher &odomFeedback);
void velocityCallback(const geometry_msgs::Twist& msg);
void odometryCallback(const nav_msgs::Odometry& msg);
void feedbackCallback(const frontiers_exploration::wheelSpeed& msg);

bool m_init = true;
static bool m_send = false;
////
float m_slipFactor = 0.0;
float m_slipFactorError = 0.0;
float m_slipFactorErrorSum = 0.0;
////
float m_targetAngularZ = 0.0; // Angular reference velocity
float m_targetLinearX = 0.0; // Linear  reference velocity
float m_lastTargetAngularZ = 0.0;
//
float m_realSpeedRight = 0.0;   // Linear velocity of the right track
float m_realSpeedLeft = 0.0;   // Linear velocity of the left  track
//
float m_wheelBase =  0.70;  // Distance between wheels

nav_msgs::Odometry m_odom;
float m_odomLineal, m_odomAngular, m_posX, m_posY, m_orientation;
float m_odomLinealPrevious, m_odomAngularPrevious;

float m_xRf = 0.0;
float m_yRf = 0.0;
float m_thetaRf = 0.0;

double m_prevSecs;
double m_interval;

bool m_increasingFriction = true;
int  m_counterNoise = 0;
int  m_counterFriction = 0;

int main(int argc, char **argv) {
	// Some kind of read to check update currentMode
	//myMode = CRosSharedMemory::FlipperMode::FMODE_1;

	ROS_INFO_STREAM("Adaptative controller in ros");
	ros::init(argc, argv, "adaptativeController");
	ros::NodeHandle n;
	ros::Subscriber robotVels      =  n.subscribe("aunav_velocity_controller/cmd_vel", 1000, &velocityCallback);
	ros::Subscriber odom           =  n.subscribe("rtabmap/odom", 1000, &odometryCallback);
	ros::Subscriber wheelFeedback  =  n.subscribe("aunav_velocity_feedback/cmd_wheel", 1000, &feedbackCallback);
	ros::Publisher wheelController =  n.advertise<frontiers_exploration::wheelSpeed>("aunav_velocity_controller/cmd_wheel", 200);
	ros::Publisher wheelOdom       =  n.advertise<frontiers_exploration::wheelSpeed>("aunav_velocity_controller/odom_wheel", 200);
	ros::Publisher increment       =  n.advertise<frontiers_exploration::wheelSpeed>("aunav_velocity_controller/incremental", 200);
	ros::Publisher odomFeedback    =  n.advertise<nav_msgs::Odometry>("aunav_velocity_feedback/odom_filtered", 200);
	ros::Rate loop_rate(10);

	while (ros::ok()) {

		//std::system("clear");
		float vrRf = m_targetLinearX + (m_wheelBase / 2) * m_targetAngularZ; // Reference linear velocity of the right track
		float vlRf = m_targetLinearX - (m_wheelBase / 2) * m_targetAngularZ; // Reference linear velocity of the left  track
		////
		ROS_INFO_STREAM("vrRf (wheel speed_R from m_wheelBase): " << vrRf << " vlRf (wheel speed_L from m_wheelBase): " << vlRf);
		ROS_INFO_STREAM("vr  : " << m_realSpeedRight << " vl: " << m_realSpeedLeft);
		////
		float odomLinearFiltered  = (m_odomLineal + m_odomLinealPrevious) / 2;
		float odomAngularFiltered = (m_odomAngular + m_odomAngularPrevious) / 2;
		float vrSl = odomLinearFiltered + (m_wheelBase / 2) * odomAngularFiltered; // Linear velocity under slip effects of the right track
		float vlSl = odomLinearFiltered - (m_wheelBase / 2) * odomAngularFiltered; // Linear velocity under slip effects of the right track
		float xSldot = odomLinearFiltered * cos(m_orientation);
		float ySldot = odomLinearFiltered * sin(m_orientation);
		float thetaSldot = odomAngularFiltered;
		float xSl = m_posX + (xSldot * m_interval); // Should be measured also
		float ySl = m_posY + (ySldot * m_interval); // Should be measured also
		float thetaSl = m_orientation + (thetaSldot * m_interval); // Should be measured also

		//// Diff ec Ref for next iteration
		float xRfdot = m_targetLinearX * cos(m_thetaRf);
		float yRfdot = m_targetLinearX * sin(m_thetaRf);
		float thetaRfdot = m_targetAngularZ;
		m_xRf = m_posX + (xRfdot * m_interval);
		m_yRf = m_posY + (yRfdot * m_interval);
		m_thetaRf = m_orientation + (thetaRfdot * m_interval);

		float ex =  cos(thetaSl) * (m_xRf - xSl) + sin(thetaSl) * (m_yRf - ySl);
		float ey = -sin(thetaSl) * (m_xRf - xSl) + cos(thetaSl) * (m_yRf - ySl);
		float etheta = m_thetaRf - thetaSl;

		// Turn error calculation:
		if ((abs(m_targetLinearX) > 0.01 || abs(m_lastTargetAngularZ) > 0.01) && abs(m_realSpeedLeft) >= 0.005 || abs(m_realSpeedRight) >= 0.005) {
			float error;
			if (m_lastTargetAngularZ > 0.0f) 
				error = m_lastTargetAngularZ - odomAngularFiltered;
			else
				error = -(m_lastTargetAngularZ - odomAngularFiltered);
			m_slipFactorErrorSum += error;
			if (m_slipFactorErrorSum > TURN_PID_MAX_ERROR)
				m_slipFactorErrorSum = TURN_PID_MAX_ERROR;
			else if (m_slipFactorErrorSum < -TURN_PID_MAX_ERROR)
				m_slipFactorErrorSum = -TURN_PID_MAX_ERROR;
			m_slipFactor += (error * TURN_PID_P) + (m_slipFactorErrorSum * TURN_PID_I) + ((m_slipFactorError - error) * TURN_PID_D);
			m_slipFactorError = error;

			if (m_slipFactor > 2.0f)
				m_slipFactor = 2.0f;
			else if (m_slipFactor < -1.0f)
				m_slipFactor = -1.0f;

			/* ROS_WARN("Slip Factor: %f",m_slipFactor);
			ROS_WARN("	Target: %f measured: %f, error: %f", m_lastTargetAngularZ, odomAngularFiltered, error);
			ROS_WARN("	m_realSpeedLeft: %f m_realSpeedRight: %f", m_realSpeedLeft, m_realSpeedRight); */
		}

		// ROS_INFO_STREAM("Error ex:" << ex << " ey" << ey << " etheta" << etheta);

		// controller
		float eps   = 1.0;
		float Beta  = 20.0;
		float theta = (m_realSpeedRight - m_realSpeedLeft) / m_wheelBase;
		float k1    = 2 * eps * sqrt((m_targetAngularZ * m_targetAngularZ) + (Beta * m_targetLinearX * m_targetLinearX));
		float k2    = Beta * abs(m_targetLinearX) + (m_targetAngularZ * theta);
		float k3    = 2 * eps * sqrt((m_targetAngularZ * m_targetAngularZ) + (Beta * m_targetLinearX * m_targetLinearX));

		//// Control values
		float u1 = -k1 * ex;
		float u2;
		if (m_targetLinearX > 0)
			u2 = -k2 * m_targetLinearX * ey - k3 * etheta;
		else
			u2 = k2 * m_targetLinearX * ey - k3 * etheta;

		//// Wheel info
		frontiers_exploration::wheelSpeed wheelValues;
		frontiers_exploration::wheelSpeed odomVelValues;
		frontiers_exploration::wheelSpeed incrementalVal;
		if (m_targetLinearX == 0 && m_targetAngularZ == 0) {
			wheelValues.wheelSpeedRight = 0.0;
			wheelValues.wheelSpeedLeft  = 0.0;
		} else {
			/* wheelValues.wheelSpeedRight = (vrRf - u1 - m_wheelBase / 2 * u2);
			wheelValues.wheelSpeedLeft = (vlRf - u1 + m_wheelBase / 2 * u2); */
			float targetAngZ = m_targetAngularZ * (1 + m_slipFactor);
			wheelValues.wheelSpeedRight = (m_targetLinearX + (m_wheelBase / 2) * targetAngZ);
			wheelValues.wheelSpeedLeft = (m_targetLinearX - (m_wheelBase / 2) * targetAngZ);
			//ROS_INFO_STREAM("Inr (wheelSpeedRight a_c): " << wheelValues.wheelSpeedRight << " Inl (wheelSpeedLeft a_c): " << wheelValues.wheelSpeedLeft);

			odomVelValues.wheelSpeedLeft = m_targetAngularZ;
			odomVelValues.wheelSpeedRight = odomAngularFiltered;
			incrementalVal.wheelSpeedLeft = m_slipFactor;
			incrementalVal.wheelSpeedRight = m_slipFactor;
			wheelOdom.publish(odomVelValues);
			increment.publish(incrementalVal);
		}
		ROS_INFO_STREAM("wheelSpeedRight a_c: " << wheelValues.wheelSpeedRight << " wheelSpeedLeft a_c: " << wheelValues.wheelSpeedLeft);
		ROS_INFO_STREAM("Slip Factor: " << m_slipFactor);
		wheelController.publish(wheelValues);
		if (stopRobot) {
			wheelValues.wheelSpeedRight = 0.0;
			wheelValues.wheelSpeedLeft  = 0.0;
			wheelController.publish(wheelValues);
			stopRobot = false;
		}

		publishOdom(odomFeedback);

		m_send = false;
		ros::spinOnce();
		loop_rate.sleep();
  	}
  	return 0;
}

void publishOdom(ros::Publisher &odomFeedback) {
	float linearX = (m_realSpeedLeft + m_realSpeedRight) / 2.0f;

	// Test 1: filter direction
	/* if (linearX == 0)
		m_odom.twist.twist.linear.x = 0;
	else if (m_odom.twist.twist.linear.x < 0 && linearX >= 0)
		m_odom.twist.twist.linear.x = 0;
	else if (m_odom.twist.twist.linear.x > 0 && linearX <= 0)
		m_odom.twist.twist.linear.x = 0; */

	//Test 2: use wheels speed
	m_odom.twist.twist.linear.x = linearX;

	// Test 3: use both 
	//m_odom.twist.twist.linear.x = (m_odom.twist.twist.linear.x + linearX) / 2.0f;

	float angularZ = (m_realSpeedRight - m_realSpeedLeft) / m_wheelBase;
	// Test 1: filter direction
	if (angularZ == 0)
		m_odom.twist.twist.angular.z = 0;
	else if (m_odom.twist.twist.angular.z < 0 && angularZ >= 0)
		m_odom.twist.twist.angular.z = 0;
	else if (m_odom.twist.twist.angular.z > 0 && angularZ <= 0)
		m_odom.twist.twist.angular.z = 0;

	// Test 2: use wheels speed
	//m_odom.twist.twist.angular.z = angularZ;

	// Test 3: use both 
	//m_odom.twist.twist.angular.z = (m_odom.twist.twist.angular.z + angularZ) / 2.0f;

	odomFeedback.publish(m_odom);
}

void velocityCallback(const geometry_msgs::Twist& msg) {
	m_lastTargetAngularZ = m_targetAngularZ;
	m_targetLinearX  = msg.linear.x;
	m_targetAngularZ = msg.angular.z;
	ROS_INFO_STREAM("Linear speed from m_wheelBase: " << m_targetLinearX << "Angular speed from m_wheelBase: " << m_targetAngularZ);
}

void feedbackCallback(const frontiers_exploration::wheelSpeed& msg) {
	m_realSpeedRight = msg.wheelSpeedRight;
	m_realSpeedLeft = msg.wheelSpeedLeft;
	ROS_INFO_STREAM("Feedback wheel speed rigth: " << m_realSpeedRight << " Feedback wheel speed left: " << m_realSpeedLeft);
}

void odometryCallback(const nav_msgs::Odometry& msg) {
	m_interval = ros::Time::now().toSec() - m_prevSecs;
	m_prevSecs = ros::Time::now().toSec();
	m_odom = msg;
	m_odomLinealPrevious = m_odomLineal;
	m_odomAngularPrevious = m_odomAngular;
	m_odomLineal  = msg.twist.twist.linear.x;
	m_odomAngular = msg.twist.twist.angular.z;
	m_posX = msg.pose.pose.position.x;
	m_posY = msg.pose.pose.position.y;
	tf::Quaternion q(
		msg.pose.pose.orientation.x,
		msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z,
		msg.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	m_orientation = yaw;
	if (m_init) {
		m_prevSecs = ros::Time::now().toSec();
		m_xRf = m_posX;
		m_yRf = m_posY;
		m_thetaRf = m_orientation;
		ROS_INFO_STREAM("Starting: \n x: " << m_posX << " y" << m_posY << " theta" << m_orientation);
		m_init = false;
	}
	m_send = true;
}
