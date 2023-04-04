#ifndef WHEELCHAIRNAV_H
#define WHEELCHAIRNAV_H

#include "rossharedmemory.h"
#include "rossharedmemoryExploration.h"
#include "wheelSpeed.h"

#include <termios.h>
#include <stdio.h>
#include <ros/ros.h>
#include <vector>
#include <ctime>
#include <math.h>
#include <stdlib.h>

#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/angles.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

using namespace std;

//#define USE_TRAVERSABILITY

//Shared memory vars
stSharedRtoE m_sRtoE;
stSharedEtoR m_sEtoR;
CRosSharedMemory<stSharedRtoE, stSharedEtoR> shmMemory(SHARED_NAME_EXPLORATION, false);
bool m_startSm, m_bLockGet, m_bLockSet;
unsigned char m_counterError = 0;

//ROS
ros::Subscriber m_mapSub;
ros::Subscriber m_wheelSpeedSub;
ros::Subscriber m_humanPoseSub;

//Wheel controller vars
float m_wheelLeft;
float m_wheelRight;

//Occupancy grid vars
nav_msgs::OccupancyGrid m_map;
bool m_mapReceived = false;

//Action lib vars
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* m_ac;

//wheelChair vars
float m_radiusWheelchair = 1.0;
float m_radiusValidPose = 0.25;
bool m_firstRefresWheelchairPose = true;
geometry_msgs::Pose m_wheelchairPose, m_prevWheelchairPose;
enum WheelchairStatus {STAND_BY = 0, LOCATING_HUMAN = 1, HUMAN_LOCATED = 2, MOVING_TO_GOAL = 3, OBSTACLE_DETECTED = 4};
WheelchairStatus m_wheelchairStatus = STAND_BY;
geometry_msgs::Pose m_target;

//human vars
float m_radiusHuamn = 0.5;
float m_humanOrientation;
bool m_side = 0;                              // 0: left 1: rigth
bool m_firstRefreshHumanPose = true;
geometry_msgs::Point m_humanPose;
geometry_msgs::Point m_prevHumanPose;
std::vector<geometry_msgs::Point> m_vectorHumanPose;

//Transform
tf::TransformListener* m_listener;

// Callbacks functions
void occupancygridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
void wheelSpeedCallback(const frontiers_exploration::wheelSpeed &msg);
void humanPoseCallback(const geometry_msgs::Point::ConstPtr& humanPose);

// Navigation functions
bool refreshWheelchairPose();
bool pointInsideZone(const geometry_msgs::Point &evalPoint, const geometry_msgs::Point &refPoint, float radius);
geometry_msgs::Point estimatedWheelchairPose(const geometry_msgs::Point &humanPose, float dist);
bool transPointBL2M(const geometry_msgs::Point &pointIn, geometry_msgs::Point &pointOut);
bool transPointOS2BL(const geometry_msgs::Point &pointIn, geometry_msgs::Point &pointOut);
bool transPoseBL2M(const geometry_msgs::Pose &poseIn, geometry_msgs::Pose &posetOut);
bool poseValid(const geometry_msgs::Point& pose, float radius);
bool detectObstacle(const geometry_msgs::Point &startPoint, const geometry_msgs::Point &endPoint);
bool moveWheelchair(const geometry_msgs::Pose& goal);
void moveWheelchairDone(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResultConstPtr& result);
void moveRobotActive();

// Functions tools
unsigned long int cellValue(unsigned long int cell);
unsigned long int floor0(float value);
unsigned long int point2Cell(float x, float y, unsigned long int width = m_map.info.width);
geometry_msgs::Point iterateRayCast(geometry_msgs::Point startingPoint, float i, float lengthX, float lengthY, float slope);
float getDistance2D(float x1, float x2, float y1, float y2);

//ROS spin
void spin();

#endif // WHEELCHAIRNAV_H