#include "rossharedmemory.h"
#include "rossharedmemoryExploration.h"
#include "navigator/wheelchairNav.h"
#include "wheelSpeed.h"

#include "nav_msgs/OccupancyGrid.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"

#include <visualization_msgs/Marker.h>

#include <std_msgs/Float32.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <nav_msgs/GetPlan.h>
#include "nav_msgs/Path.h"

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/angles.h>

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
	memset(&m_sEtoR, 0, sizeof(stSharedEtoR));
	ROS_INFO_STREAM("Navigator node under developement");
	ros::init(argc, argv, "navigator_node");
	m_ac = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true);
	m_listener = new tf::TransformListener();
	spin();
	delete m_ac;
	return 0;
}

void spin() {

  bool first = true, finish = false;
	ros::NodeHandle n;
	ros::NodeHandle pnh;
	pnh.getParam("radius_wheelchair", m_radiusWheelchair);
	pnh.getParam("radius_human", m_radiusHuamn);
  pnh.getParam("radius_Valid_Pose", m_radiusValidPose);
	m_mapSub = n.subscribe("/anticrash/processedGrid", 1, occupancygridCallback);
	m_wheelSpeedSub = n.subscribe("aunav_velocity_controller/cmd_wheel", 1000, &wheelSpeedCallback);
  m_humanPoseSub = n.subscribe<geometry_msgs::Point>("/people_follower/humanGoal", 1000, humanPoseCallback);

	m_pointMarkerPub = n.advertise<visualization_msgs::Marker>("/people_follower/markerPoint", 1);

	ros::Rate loop_rate(10);

	while (ros::ok()) {
		m_bLockGet = shmMemory.getData(m_sRtoE);
		m_startSm = m_sRtoE.start;

		if (m_startSm) {
			if (m_mapReceived) {
				if (refreshWheelchairPose()) {
					ROS_WARN("After refreshWheelchairPose");
					if (m_humanPose.y > 0) //TODO Define if m_humanPose.y == 0;
						m_side = 0;
					else
						m_side = 1;
					ROS_WARN("m_humanPose X = %f Y = %f", m_humanPose.x, m_humanPose.y);
					ROS_WARN("Before switch");
					switch (m_wheelchairStatus)
					{
						case WheelchairStatus::STAND_BY: {
							ROS_WARN("Into STAND_BY");
							geometry_msgs::Point evalWheelchairPose, refWheelchairPose;
							evalWheelchairPose = estimatedWheelchairPose(m_humanPose, 1.5);
							ROS_WARN("evalWheelchairPose X = %f Y = %f", evalWheelchairPose.x, evalWheelchairPose.y);
							publishPointMarker(evalWheelchairPose, "os_sensor");
							refWheelchairPose.x = 0.0;
							refWheelchairPose.y = 0.0;
							refWheelchairPose.z = 0.0;
							if (!pointInsideZone(evalWheelchairPose, refWheelchairPose, m_radiusWheelchair)) {
								geometry_msgs::Point wheelchairPointOut; 
								geometry_msgs::Pose wheelchairPoseBL;
								if (transPointBL2M(evalWheelchairPose, wheelchairPointOut)) {
									wheelchairPoseBL.position = wheelchairPointOut;
									tf::quaternionTFToMsg (tf::createQuaternionFromRPY(0.0,0.0,m_humanOrientation), wheelchairPoseBL.orientation);
									if (transPoseBL2M(wheelchairPoseBL, m_target)) {
										if (poseValid(m_target.position, m_radiusValidPose)) {
											moveWheelchair(m_target);
											m_wheelchairStatus = WheelchairStatus::LOCATING_HUMAN;
											ROS_WARN("Navigator [mainloop]: Moving to next human position; Status =  LOCATING_HUMAN");
										} else {
											//ERROR
											m_sEtoR.finish = true;
											shmMemory.setData(m_sEtoR);
											ROS_WARN("Navigator [mainloop]: The pose selected is not valid, the human guide must look for another path");
										}
									}
								}
							}
						} break;
						case WheelchairStatus::LOCATING_HUMAN: {
							/* if (!poseValid(m_target.position, m_radiusValidPose)) {
								m_ac->cancelGoal();
							} */
							publishPointMarker(m_target.position, "map");
						} break;
						case WheelchairStatus::HUMAN_LOCATED: {
							geometry_msgs::Pose wheelchairPoseBL;
							float inc = 0.0;
							for (int i = 0; i < 3; i++) {
								wheelchairPoseBL.position.x = 2.5 + inc;
								wheelchairPoseBL.position.y = 0.0;
								wheelchairPoseBL.position.z = 0.0;
								tf::quaternionTFToMsg (tf::createQuaternionFromRPY(0.0,0.0,m_humanOrientation), wheelchairPoseBL.orientation);
								if (transPoseBL2M(wheelchairPoseBL, m_target)) {
									if (poseValid(m_target.position, m_radiusValidPose)) {
										moveWheelchair(m_target);
										publishPointMarker(m_target.position, "map");
										m_wheelchairStatus = WheelchairStatus::MOVING_TO_GOAL;
										ROS_WARN("Navigator [mainloop]: Moving to goal valided; Status =  MOVING_TO_GOAL");
										break;
									}
									inc++;
								}
							}
							// ERROR
							m_sEtoR.finish = true;
							shmMemory.setData(m_sEtoR);
							ROS_WARN("Navigator [mainloop]: There are many obstacles, is not posible find a pose valid");
						} break;
						case WheelchairStatus::MOVING_TO_GOAL: {
							geometry_msgs::Point refHumanPose;
							if (m_side == 0) {
								refHumanPose.x = 0.0;
								refHumanPose.y = 1.5;
								refHumanPose.z = 0.0;
							} else {
								refHumanPose.x = 0.0;
								refHumanPose.y = -1.5;
								refHumanPose.z = 0.0;
							}
							publishPointMarker(refHumanPose, "os_sensor");
							if (!pointInsideZone(m_humanPose, refHumanPose, m_radiusHuamn)) {
								m_ac->cancelGoal();
								ROS_WARN("Navigator [mainloop]: Human point outside the zone, aborted plan");
							}
							geometry_msgs::Point startPoint, endPoint;
							startPoint.x = m_wheelchairPose.position.x;
							startPoint.y = m_wheelchairPose.position.y;
							startPoint.z = m_wheelchairPose.position.z;
							endPoint.x = m_target.position.x;
							endPoint.y = m_target.position.y;
							endPoint.z = m_target.position.z;
							if (detectObstacle(startPoint, endPoint)) {
								m_wheelchairStatus = WheelchairStatus::OBSTACLE_DETECTED;
								ROS_WARN("Navigator [mainloop]: There is a obstacle in the path; Status =  OBSTACLE_DETECTED");
							}
							/* if (!poseValid(m_target.position, m_radiusValidPose)) {
								m_ac->cancelGoal();
							} */
						} break;
						case WheelchairStatus::OBSTACLE_DETECTED: {
							/* if (!poseValid(m_target.position, m_radiusValidPose)) {
								m_ac->cancelGoal();
							} */
							publishPointMarker(m_target.position, "map");
						} break;
					}
				} else {
					ROS_WARN("Exploration: Couldn't get robot position!");
				}
			} else {
				ROS_WARN("Exploration: Waiting for first map");
			}
		} else {
			if (m_bLockGet) {
				m_side = 0;
				m_counterError = 0;
				m_mapReceived = false;
				m_firstRefresWheelchairPose = true;
				m_wheelchairStatus = WheelchairStatus::STAND_BY;
				m_sEtoR.speedLeft = 0;
				m_sEtoR.speedRight = 0;
				m_sEtoR.finish = false;
				m_sEtoR.error = false;
				m_bLockSet = shmMemory.setData(m_sEtoR);
				ROS_WARN("The navigator is stoped");
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void occupancygridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
	ROS_WARN("m_startSm 1 [occupancygridCallback] = %d", m_startSm);
	if(m_startSm) {
		ROS_WARN("Into occupancy grid callback");
		m_map = *msg;
		m_mapReceived = true;
	}
	ROS_WARN("m_startSm 2 [occupancygridCallback] = %d", m_startSm);
}

void humanPoseCallback(const geometry_msgs::Point::ConstPtr& humanPose) {
	if(m_startSm) {
		ROS_WARN("Into human pose callback");
		if (transPointOS2BL(*humanPose, m_humanPose)) {
			if (m_firstRefreshHumanPose) {
				m_prevHumanPose = m_humanPose;
				m_firstRefreshHumanPose = false;
			}
			if ((abs(m_humanPose.x - m_prevHumanPose.x) >= 0.04) && (abs(m_humanPose.y - m_prevHumanPose.y) >= 0.04)){
				if (m_vectorHumanPose.size() >= 10)
					m_vectorHumanPose.erase(m_vectorHumanPose.begin());
				m_vectorHumanPose.push_back(m_humanPose);
				m_humanOrientation = atan2((m_vectorHumanPose[10].y - m_vectorHumanPose[0].y), (m_vectorHumanPose[10].y - m_vectorHumanPose[0].y));
				m_prevHumanPose = m_humanPose;
			}
		}
	}
}

bool refreshWheelchairPose(){
	ROS_WARN("Into refresh wheelchair pose");
	ROS_WARN("m_startSm 1 [refreshWheelchairPose] %d = ", m_startSm);
	tf::StampedTransform transform;
	std::string parentFrame = "map";
	std::string childFrame = "base_link";
	try {
		ROS_WARN("m_startSm 2 [refreshWheelchairPose] %d = ", m_startSm);
		if (m_listener->waitForTransform(parentFrame, childFrame, ros::Time(0), ros::Duration(5.0))) {
			ROS_WARN("m_startSm 3 [refreshWheelchairPose] %d = ", m_startSm);
			m_listener->lookupTransform(parentFrame, childFrame, ros::Time(0), transform);
			ROS_WARN("m_startSm 4 [refreshWheelchairPose] %d = ", m_startSm);
			m_counterError = 0;
		} else {
			m_counterError++;
			ROS_ERROR("Navigator: refreshWheelchairPose: no transform between frames %s and %s", parentFrame.c_str(), childFrame.c_str());
			if (m_counterError > 10) {
				m_sEtoR.error = true;
				shmMemory.setData(m_sEtoR);
				ROS_WARN("Navigator: m_counterError: %d",m_counterError);
				ROS_WARN("Navigator: m_sEtoR.error: %d", m_sEtoR.error);
				ROS_WARN("Navigator: m_bLockSet error get_plan service: %d", m_bLockSet);
			}
			return false;
		}
	} catch(tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
		return false;
	}
	ROS_WARN("m_startSm 5 [refreshWheelchairPose] %d = ", m_startSm);
	m_wheelchairPose.position.x = transform.getOrigin().x();
	m_wheelchairPose.position.y = transform.getOrigin().y();
	m_wheelchairPose.position.z = 0.0;
	m_wheelchairPose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(transform.getRotation()));
	ROS_WARN("m_startSm 6 [refreshWheelchairPose] %d = ", m_startSm);
	if (m_firstRefresWheelchairPose) {
		m_prevWheelchairPose = m_wheelchairPose;
		m_firstRefresWheelchairPose = false;
	}

	m_prevWheelchairPose = m_wheelchairPose;
	ROS_WARN("m_startSm 7 [refreshWheelchairPose] %d = ", m_startSm);
	return true;
}

geometry_msgs::Point estimatedWheelchairPose(const geometry_msgs::Point &humanPose, float dist) {
	ROS_WARN("m_humanOrientation = %f; m_side = %d", m_humanOrientation, m_side);
	geometry_msgs::Point estimatedPose;
	if (m_side == 0) {
		ROS_WARN("m_side == 0");
		if (m_humanOrientation == 0) {
			ROS_WARN("m_humanOrientation == 0");
			estimatedPose.x = humanPose.x;
			estimatedPose.y = humanPose.y - dist;

		} else if (0 < m_humanOrientation && m_humanOrientation < M_PI/2) {
			ROS_WARN("0 < m_humanOrientation && m_humanOrientation < M_PI/2");
			estimatedPose.x = humanPose.x + dist * sin(m_humanOrientation);
			estimatedPose.y = humanPose.y - dist * cos(m_humanOrientation);

		} else if (m_humanOrientation == M_PI/2) {
			ROS_WARN("m_humanOrientation == M_PI/2");
			estimatedPose.x = humanPose.x + dist;
			estimatedPose.y = humanPose.y;

		} else if (M_PI/2 < m_humanOrientation && m_humanOrientation < M_PI) {
			ROS_WARN("M_PI/2 < m_humanOrientation && m_humanOrientation < M_PI");
			estimatedPose.x = humanPose.x - dist * sin(M_PI - m_humanOrientation);
			estimatedPose.y = humanPose.y - dist * cos(M_PI - m_humanOrientation);

		} else if (m_humanOrientation == M_PI || m_humanOrientation == -M_PI) {
			ROS_WARN("m_humanOrientation == M_PI || m_humanOrientation == -");
			estimatedPose.x = humanPose.x;
			estimatedPose.y = humanPose.y - dist;

		} else if (-M_PI/2 < m_humanOrientation && m_humanOrientation < 0) {
			ROS_WARN("-M_PI/2 < m_humanOrientation && m_humanOrientation < 0");
			estimatedPose.x = humanPose.x - dist * sin(abs(m_humanOrientation));
			estimatedPose.y = humanPose.y - dist * cos(abs(m_humanOrientation));

		} else if (m_humanOrientation == -M_PI/2) {
			ROS_WARN("m_humanOrientation == -M_PI/2");
			estimatedPose.x = humanPose.x - dist;
			estimatedPose.y = humanPose.y;

		} else if (-M_PI < m_humanOrientation && m_humanOrientation < -M_PI/2) {
			ROS_WARN("-M_PI < m_humanOrientation && m_humanOrientation < -M_PI/2");
			estimatedPose.x = humanPose.x + dist * sin(M_PI - abs(m_humanOrientation));
			estimatedPose.y = humanPose.y - dist * cos(M_PI - abs(m_humanOrientation));
		}
	} else {
		ROS_WARN("m_side == 1");
		if (m_humanOrientation == 0) {
			ROS_WARN("m_humanOrientation == 0");
			ROS_WARN("dist = %f", dist);
			estimatedPose.x = humanPose.x;
			estimatedPose.y = humanPose.y + dist;
			ROS_WARN("estimatedPose.y = %f", estimatedPose.y);

		} else if (-M_PI/2 < m_humanOrientation && m_humanOrientation < 0) {
			ROS_WARN("-M_PI/2 < m_humanOrientation && m_humanOrientation < 0");
			estimatedPose.x = humanPose.x + dist * sin(abs(m_humanOrientation));
			estimatedPose.y = humanPose.y + dist * cos(abs(m_humanOrientation));

		} else if (m_humanOrientation == -M_PI/2) {
			ROS_WARN("m_humanOrientation == -M_PI/2");
			estimatedPose.x = humanPose.x + dist;
			estimatedPose.y = humanPose.y;

		} else if (-M_PI < m_humanOrientation && m_humanOrientation < -M_PI/2) {
			ROS_WARN("-M_PI < m_humanOrientation && m_humanOrientation < -M_PI/2");
			estimatedPose.x = humanPose.x - dist * sin(M_PI - abs(m_humanOrientation));
			estimatedPose.y = humanPose.y + dist * cos(M_PI - abs(m_humanOrientation));
			
		} else if (m_humanOrientation == M_PI || m_humanOrientation == -M_PI) {
			ROS_WARN("m_humanOrientation == M_PI || m_humanOrientation == -M_PI");
			estimatedPose.x = humanPose.x;
			estimatedPose.y = humanPose.y + dist;

		} else if (0 < m_humanOrientation && m_humanOrientation < M_PI/2) {
			ROS_WARN("0 < m_humanOrientation && m_humanOrientation < M_PI/2");
			estimatedPose.x = humanPose.x - dist * sin(m_humanOrientation);
			estimatedPose.y = humanPose.y + dist * cos(m_humanOrientation);

		} else if (m_humanOrientation == M_PI/2) {
			ROS_WARN("m_humanOrientation == M_PI/2");
			estimatedPose.x = humanPose.x - dist;
			estimatedPose.y = humanPose.y;

		} else if (M_PI/2 < m_humanOrientation && m_humanOrientation < M_PI) {
			ROS_WARN("M_PI/2 < m_humanOrientation && m_humanOrientation < M_PI");
			estimatedPose.x = humanPose.x + dist * sin(M_PI - m_humanOrientation);
			estimatedPose.y = humanPose.y + dist * cos(M_PI - m_humanOrientation);
		}
	}
	return estimatedPose;
}

bool transPointBL2M(const geometry_msgs::Point &pointIn, geometry_msgs::Point &pointOut) {
	ROS_WARN("Into transPointBL2M");
	std::string parentFrame = "map";
	std::string childFrame = "base_link";

	try {
		if (m_listener->waitForTransform(parentFrame, childFrame, ros::Time(0), ros::Duration(5.0))) {
			m_counterError = 0;
		} else {
			m_counterError++;
			ROS_ERROR("Navigator [transPointBL2M]: no transform between frames %s and %s", parentFrame.c_str(), childFrame.c_str());
			if (m_counterError > 10) {
				m_sEtoR.error = true;
				ROS_WARN("Navigator: m_counterError: %d",m_counterError);
				ROS_WARN("Navigator: m_sEtoR.error: %d", m_sEtoR.error);
				ROS_WARN("Navigator: m_bLockSet error get_plan service: %d", m_bLockSet);
			}
			return false;
		}
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
		return false;
	}
	geometry_msgs::PointStamped poseInS, poseOutS;
	poseInS.header.frame_id = childFrame;
	poseInS.header.stamp = ros::Time::now();
	poseInS.point = pointIn;
	ROS_WARN("Before transform point [transPointBL2M]");
	m_listener->transformPoint(parentFrame, poseInS, poseOutS);
	ROS_WARN("After transform point [transPointBL2M]");
  pointOut = poseOutS.point;
	ROS_WARN("Navigator [transPointBL2M]: Point in base_link = (%.2f, %.2f. %.2f) -> Point in map = (%.2f, %.2f, %.2f)",
						pointIn.x, pointIn.y, pointIn.z, pointOut.x, pointOut.y, pointOut.z);
	return true;
}

bool transPoseBL2M(const geometry_msgs::Pose &poseIn, geometry_msgs::Pose &posetOut) {
	std::string parentFrame = "map";
	std::string childFrame = "base_link";

	try {
		if (m_listener->waitForTransform(parentFrame, childFrame, ros::Time(0), ros::Duration(5.0))) {
			m_counterError = 0;
		} else {
			m_counterError++;
			ROS_ERROR("Navigator [transPoseBL2M]: no transform between frames %s and %s", parentFrame.c_str(), childFrame.c_str());
			if (m_counterError > 10) {
				m_sEtoR.error = true;
				ROS_WARN("Navigator: m_counterError: %d",m_counterError);
				ROS_WARN("Navigator: m_sEtoR.error: %d", m_sEtoR.error);
				ROS_WARN("Navigator: m_bLockSet error get_plan service: %d", m_bLockSet);
			}
			return false;
		}
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
		return false;
	}
	geometry_msgs::PoseStamped poseInS, poseOutS;
  poseInS.header.frame_id = childFrame;
  poseInS.header.stamp = ros::Time::now();
	poseInS.pose = poseIn;
	m_listener->transformPose(parentFrame, poseInS, poseOutS);
	posetOut = poseOutS.pose;
	ROS_WARN("Navigator [transPoseBL2M]: Pose in base_link = (%.2f, %.2f. %.2f) -> Pose in map = (%.2f, %.2f, %.2f)",
						poseIn.position.x, poseIn.position.y, poseIn.position.z, posetOut.position.x, posetOut.position.y, posetOut.position.z);
	return true;
}

bool transPointOS2BL(const geometry_msgs::Point &pointIn, geometry_msgs::Point &pointOut) {
	std::string parentFrame = "base_link";
	std::string childFrame = "os_sensor";

	try {
		if (m_listener->waitForTransform(parentFrame, childFrame, ros::Time(0), ros::Duration(5.0))) {
			m_counterError = 0;
		} else {
			m_counterError++;
			ROS_ERROR("Navigator [transPointOS2BL]: no transform between frames %s and %s", parentFrame.c_str(), childFrame.c_str());
			if (m_counterError > 10) {
				m_sEtoR.error = true;
				ROS_WARN("Navigator: m_counterError: %d",m_counterError);
				ROS_WARN("Navigator: m_sEtoR.error: %d", m_sEtoR.error);
				ROS_WARN("Navigator: m_bLockSet error get_plan service: %d", m_bLockSet);
			}
			return false;
		}
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
		return false;
	}
	geometry_msgs::PointStamped poseInS, poseOutS;
	poseInS.header.frame_id = childFrame;
	poseInS.header.stamp = ros::Time::now();
	poseInS.point = pointIn;
	m_listener->transformPoint(parentFrame, poseInS, poseOutS);
  pointOut = poseOutS.point;
	ROS_WARN("Navigator [transPointOS2BL]: Point in os_sensor = (%.2f, %.2f. %.2f) -> Point in base_link = (%.2f, %.2f, %.2f)",
						pointIn.x, pointIn.y, pointIn.z, pointOut.x, pointOut.y, pointOut.z);
	return true;
}

bool detectObstacle(const geometry_msgs::Point &startPoint, const geometry_msgs::Point &endPoint) {
	float radius = getDistance2D(startPoint.x, endPoint.x, startPoint.y, endPoint.y);
	float yaw = atan2((endPoint.y - startPoint.y), (endPoint.x - startPoint.x)) - 0.1*M_PI;
	geometry_msgs::Point evalPoint;
	for (int i = 0; i < 3; i++) {
		evalPoint.x = endPoint.x + radius * cos(yaw);
		evalPoint.y = endPoint.y + radius * sin(yaw);
		float lengthX = endPoint.x - evalPoint.x;
		float lengthY = endPoint.y - evalPoint.y;
		float maxValue = max(abs(lengthX),abs(lengthY));
		float slope = lengthY/lengthX;
		for (float i = m_map.info.resolution; i < maxValue - m_map.info.resolution; i = i + m_map.info.resolution) {
			geometry_msgs::Point newPoint = iterateRayCast(evalPoint, i, lengthX, lengthY, slope);
			unsigned long int cell = point2Cell(newPoint.x, newPoint.y);
			if (cellValue(cell) == 100)
				return true;
		}
		yaw += 0.1*M_PI;
	}
	return false;
}

bool pointInsideZone(const geometry_msgs::Point &evalPoint, const geometry_msgs::Point &refPoint, float radius) {
	float dist = getDistance2D(evalPoint.x, refPoint.x, evalPoint.y, refPoint.y);
	if (dist <= radius)
		return true;
	else 
		return false;
}

geometry_msgs::Point iterateRayCast(geometry_msgs::Point startingPoint, float i, float lengthX, float lengthY, float slope) {
	geometry_msgs::Point newPoint;
	if (lengthX  == 0) {
		newPoint.x = startingPoint.x;
		if (lengthY > 0) {
			newPoint.y = startingPoint.y + i;
		} else {
			newPoint.y = startingPoint.y - i;
		}
	} else if (lengthY == 0) {
		newPoint.y = startingPoint.y;
		if (lengthX > 0) {
			newPoint.x = startingPoint.x + i;
		} else {
			newPoint.x = startingPoint.x - i;
		}
	} else if (lengthX > 0){
		if (abs(slope)<1){
			newPoint.x = startingPoint.x + i;
			newPoint.y = startingPoint.y + slope*i;
		} else {
			if (lengthY>0) {
				newPoint.y = startingPoint.y + i;
			} else {
				newPoint.y = startingPoint.y - i;
			}
			newPoint.x = startingPoint.x + i/abs(slope);
		}
	} else {
		if (abs(slope)<1){
			newPoint.x = startingPoint.x - i;
			newPoint.y = startingPoint.y - slope*i;
		} else {
			if (lengthY>0) {
				newPoint.y = startingPoint.y + i;
			} else {
				newPoint.y = startingPoint.y - i;
			}
			newPoint.x = startingPoint.x - i/abs(slope);
		}
	}
	return newPoint;
}

unsigned long int floor0(float value) {
	if (value < 0.0)
		return (unsigned long int)ceil(value);
	else
		return (unsigned long int)floor(value);
}

unsigned long int point2Cell(float x, float y, unsigned long int width) {
	unsigned long int x_cell = floor0((x - m_map.info.origin.position.x) / m_map.info.resolution);
	unsigned long int y_cell = floor0((y - m_map.info.origin.position.y) / m_map.info.resolution);
	unsigned long int cell = x_cell + y_cell * width;
	return cell;
}

unsigned long int cellValue(unsigned long int cell) {
	return m_map.data[cell];
}

bool poseValid(const geometry_msgs::Point& pose, float radius) {

	float robotRadius = radius; 
	int footprintDiscretized = ceil(robotRadius / m_map.info.resolution);
	int index = point2Cell(pose.x, pose.y);

	if (index == -1 || cellValue(index) == 100)
		return false;

	for (int j = -footprintDiscretized; j <= footprintDiscretized; j++) {
		for (int i = -footprintDiscretized; i <= footprintDiscretized; i++) {
			unsigned long int cell = i * m_map.info.width + j + index;
			if (cell > 0 && cell < m_map.data.size())
				if (cellValue(cell) == 100)
					return false;
		}
	}
	return true;
}

float getDistance2D(float x1, float x2, float y1, float y2) {
	return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

bool moveWheelchair(const geometry_msgs::Pose& goal) {
	if (!m_ac->waitForServer(ros::Duration(5.0))) {
		ROS_WARN("NavigationSlam::moveRobot moveRobot: Waiting for the move_base action server to come up");
		return false;
	}
	move_base_msgs::MoveBaseGoal nextGoal;
	nextGoal.target_pose.header.frame_id = "map";
	nextGoal.target_pose.header.stamp = ros::Time::now();
	nextGoal.target_pose.pose = goal;

	// ROS_INFO("moveRobot: Sending Goal: x=%4.2f, y=%4.2f, yaw=%4.2f",
	// nextGoal.target_pose.pose.position.x,
	// nextGoal.target_pose.pose.position.y,
	// tf::getYaw(nextGoal.target_pose.pose.orientation));

  ROS_WARN("Navigator: [moveRobot] Moving to goal: X= %f ; Y= %f", goal.position.x, goal.position.y);
	m_ac->sendGoal(nextGoal,
                boost::bind(&moveWheelchairDone, _1, _2),
                boost::bind(&moveRobotActive));
	return true;
}

void moveWheelchairDone(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResultConstPtr& result) {
	switch (m_wheelchairStatus)
	{
	case WheelchairStatus::LOCATING_HUMAN:
		if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
			geometry_msgs::Point refHumanPose;
			if (m_side == 0) {
				refHumanPose.x = 0.0;
				refHumanPose.y = 1.5;
				refHumanPose.z = 0.0;
			} else {
				refHumanPose.x = 0.0;
				refHumanPose.y = -1.5;
				refHumanPose.z = 0.0;
			}
			if (pointInsideZone(m_humanPose, refHumanPose, m_radiusHuamn)) {
				m_wheelchairStatus = WheelchairStatus::HUMAN_LOCATED;
				ROS_WARN("Navigator [moveWheelchairDone]: Position next to human ok; Status =  HUMAN_LOCATED");
			} else {
				m_wheelchairStatus = WheelchairStatus::STAND_BY;
				ROS_WARN("Navigator [moveWheelchairDone]: Human point outside the zone;  Status =  STAND_BY");
			}
		} else {
			m_wheelchairStatus = WheelchairStatus::STAND_BY;
			ROS_WARN("Navigator [moveWheelchairDone]: Aborted plan; Status =  STAND_BY");
		}
		break;
	case WheelchairStatus::MOVING_TO_GOAL:
		if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
			m_wheelchairStatus = WheelchairStatus::HUMAN_LOCATED;
			ROS_WARN("Navigator [moveWheelchairDone]: Path goal reached; Status =  HUMAN_LOCATED");
		} else {
			m_wheelchairStatus = WheelchairStatus::STAND_BY;
			ROS_WARN("Navigator [moveWheelchairDone]: Path goal no reached; Status =  STAND_BY");
		}
		break;
	case WheelchairStatus::OBSTACLE_DETECTED:
		if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
			m_wheelchairStatus = WheelchairStatus::HUMAN_LOCATED;
			ROS_WARN("Navigator [moveWheelchairDone]: Obstacle successfully avoided; Status =  HUMAN_LOCATED");
		} else {
			m_wheelchairStatus = WheelchairStatus::STAND_BY;
			ROS_WARN("Navigator [moveWheelchairDone]: Obstacle no successfully avoided; Status =  STAND_BY");
		}
		break;
	}
}

void moveRobotActive() {
}

void wheelSpeedCallback(const frontiers_exploration::wheelSpeed &msg) {
	if (m_startSm) {
		m_wheelLeft = msg.wheelSpeedLeft;
		m_wheelRight = msg.wheelSpeedRight;
		m_sEtoR.speedLeft = m_wheelLeft;
		m_sEtoR.speedRight = m_wheelRight;
		m_bLockSet = shmMemory.setData(m_sEtoR);
	}
}

void publishPointMarker(const geometry_msgs::Point &point, std::string frameId) {
	visualization_msgs::Marker markerPoint;

	markerPoint.header.frame_id = frameId;
	markerPoint.header.stamp = ros::Time::now();
	markerPoint.id = 0;
  markerPoint.type = visualization_msgs::Marker::SPHERE;
  markerPoint.action = visualization_msgs::Marker::ADD;
  markerPoint.ns = "navigator";
  markerPoint.scale.x = 0.2;
  markerPoint.scale.y = 0.2;
  markerPoint.scale.z = 0.2;
  markerPoint.color.a = 1.0;
  markerPoint.color.r = 0.0;
  markerPoint.color.g = 1.0;
  markerPoint.color.b = 0.0;

	markerPoint.pose.position.x = point.x;
	markerPoint.pose.position.y = point.y;
	markerPoint.pose.position.z = point.z;
	m_pointMarkerPub.publish(markerPoint);
}