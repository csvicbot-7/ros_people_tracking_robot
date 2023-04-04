#include <stdio.h>
#include <math.h>
#include <termios.h>
#include <iostream>
#include <vector>
#include <ctime>

#include<ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include<antiCrash.h>

antiCrash::antiCrash() : m_privateNh("~"), m_shmMemory(SHARED_NAME_ANTICRASH, false){
  m_mapReceived = false;
  m_mapInit = false;

  m_odomSub = m_nh.subscribe("rtabmap/odom",1, &antiCrash::odometryCallback, this);
  m_mapSub = m_nh.subscribe("/rtabmap/grid_map", 1, &antiCrash::occupancyGridCallback, this);
  m_gridModPub = m_nh.advertise<nav_msgs::OccupancyGrid>("/anticrash/processedGrid", 1000);
  m_cropedGridPub = m_nh.advertise<nav_msgs::OccupancyGrid>("/anticrash/cropedGrid", 1000);
  
  m_listener = new tf::TransformListener();
}

void antiCrash::occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  memset(&m_sAtoR, 0, sizeof(stSharedAtoR));
  m_map = *msg;
  if (!m_mapInit)
    fillStartingGrid(2.0, m_map.info.resolution);
  float density = expandFreeCells(m_map.info.resolution * 2, m_map.info.resolution);
  m_gridModPub.publish(m_map);
  m_mapReceived = true;
}

void antiCrash::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  static bool first = true;
  tf::StampedTransform transform;
  std::string parentFrame = "map";
  std::string childFrame = "base_link";

  try {
    if (m_listener->waitForTransform(parentFrame, childFrame, ros::Time(0), ros::Duration(5.0))) {
        m_listener->lookupTransform(parentFrame, childFrame, ros::Time(0), transform);
    } else {
      ROS_ERROR("refreshRobotPose: no transform between frames %s and %s", parentFrame.c_str(), childFrame.c_str());
    }
  } catch(tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
  }

  m_posX = transform.getOrigin().x();
  m_posY = transform.getOrigin().y();
  m_orientation = tf::getYaw(transform.getRotation());
  
  if (m_mapReceived) {
    cropMap(OCCUPANCY_MAP_WIDE, OCCUPANCY_MAP_WIDE);
    m_cropedGridPub.publish(m_cropedMap);
  }
}

void antiCrash::cropMap(int cols, int rows) {
  m_cropedMap.data.clear();
  float gridCellSize = m_map.info.resolution;
  unsigned long int sensorIndexX = floor0((m_posX - m_map.info.origin.position.x)/gridCellSize);
  unsigned long int sensorIndexY = floor0((m_posY - m_map.info.origin.position.y)/gridCellSize);
  int startMapIndexX = sensorIndexX - floor0(cols/2);
  int startMapIndexY = sensorIndexY - floor0(rows/2);
  
  m_cropedMap.info.resolution = gridCellSize;
  m_cropedMap.info.origin.position.x = cell2Point(point2Cell(m_posX, m_posY)).x - floor0(cols/2)*gridCellSize;
  m_cropedMap.info.origin.position.y = cell2Point(point2Cell(m_posX, m_posY)).y - floor0(rows/2)*gridCellSize;
  m_cropedMap.info.origin.position.z = 0.0;
  m_cropedMap.info.origin.orientation.x = 0.0;
  m_cropedMap.info.origin.orientation.y = 0.0;
  m_cropedMap.info.origin.orientation.z = 0.0;
  m_cropedMap.info.origin.orientation.w = 1.0;
  m_cropedMap.info.width = cols;
  m_cropedMap.info.height = rows;

  m_cropedMap.data.resize(m_cropedMap.info.width * m_cropedMap.info.height);

  for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
      int mapIndexX = startMapIndexX + x;
      int mapIndexY = startMapIndexY + y;
      int cropedMapIndex = x + (y * m_cropedMap.info.width);

      unsigned char value;
      // Check if current cell is inside the map:
      if (!validateCell(mapIndexX, mapIndexY, m_map)){
        // If current cell is outside of map -> set unknown
        value = OCCUPANCY_CELL_UNKNOWN;
      } else {
        int mapIndex = mapIndexX + (mapIndexY * m_map.info.width);
        value = m_map.data[mapIndex];
      }
      m_sAtoR.occupancyGrid[x][y] = value;
      m_cropedMap.data[cropedMapIndex] = value;
    }
  }

  m_sAtoR.orientation = m_orientation;
  m_sAtoR.gridSize = gridCellSize;
  m_sAtoR.mapPositionX = m_cropedMap.info.origin.position.x;
  m_sAtoR.mapPositionY = m_cropedMap.info.origin.position.y;
  m_sAtoR.mapIndexX = startMapIndexX;
  m_sAtoR.mapIndexY = startMapIndexY;
  m_shmMemory.setData(m_sAtoR);
  
}

void antiCrash::fillStartingGrid(float radius ,float resolution) {
  int discretizedRadius = int(radius/resolution);

  // Start with the center of the cicle, if it's unknown, mark as free:
  float gridCellSize = m_map.info.resolution;
  long int centerCellX = floor0((0 - m_map.info.origin.position.x)/gridCellSize);
  long int centerCellY = floor0((0 - m_map.info.origin.position.y)/gridCellSize);
  unsigned long int centerCell = getCellIndex(centerCellX, centerCellY, m_map);

  //ROS_INFO("Center cell: [%ld,%ld]=%ld", centerCellX, centerCellY, cellValue(centerCell));
  if (cellValue(centerCell) == -1) {
    m_map.data[centerCell] = 0;
  }

  bool modif = false;
  for (int i = 1; i < discretizedRadius; i++) {
    long int cellX, cellY, interiorCellX, interiorCellY;
    // Mark unknown cells as free if it's previous cell is free:

    // Corner 1:
    cellX = centerCellX + i;
    cellY = centerCellY + i;
    if (fillStartingGridCell(cellX, cellY, cellX - 1, cellY - 1, m_map))
      modif = true;

    // Corner 2:
    cellX = centerCellX - i;
    cellY = centerCellY + i;
    if (fillStartingGridCell(cellX, cellY, cellX + 1, cellY - 1, m_map))
      modif = true;

    // Corner 3:
    cellX = centerCellX + i;
    cellY = centerCellY - i;
    if (fillStartingGridCell(cellX, cellY, cellX - 1, cellY + 1, m_map))
      modif = true;

    // Corner 4:
    cellX = centerCellX - i;
    cellY = centerCellY - i;
    if (fillStartingGridCell(cellX, cellY, cellX + 1, cellY + 1, m_map))
      modif = true;

    // Border 1: Y negative
    for (int x = -i + 1; x <= i - 1; x++) {
      cellX = centerCellX + x;
      cellY = centerCellY - i;
      if (fillStartingGridCell(cellX, cellY, cellX, cellY + 1, m_map))
        modif = true;
    }

    // Border 2: Y positive
    for (int x = -i + 1; x <= i - 1; x++) {
      cellX = centerCellX + x;
      cellY = centerCellY + i;
      if (fillStartingGridCell(cellX, cellY, cellX, cellY - 1, m_map))
        modif = true;
    }

    // Border 3: X negative
    for (int y = -i + 1; y <= i - 1; y++) {
      cellX = centerCellX - i;
      cellY = centerCellY + y;
      if (fillStartingGridCell(cellX, cellY, cellX + 1, cellY, m_map))
        modif = true;
    }

    // Border 4: X positive
    for (int y = -i + 1; y <= i - 1; y++) {
      cellX = centerCellX + i;
      cellY = centerCellY + y;
      if (fillStartingGridCell(cellX, cellY, cellX - 1, cellY, m_map))
        modif = true;
    }
  }
  m_mapInit = !modif;
}

bool antiCrash::fillStartingGridCell(long int cellX, long int cellY, long int interiorCellX, long int interiorCellY, nav_msgs::OccupancyGrid &map) {
  if (validateCell(cellX, cellY, map) && validateCell(interiorCellX, interiorCellY, map)) {
    unsigned long int interiorCell = getCellIndex(interiorCellX, interiorCellY, map);
    unsigned long int cell = getCellIndex(cellX, cellY, map);
    if (cellValue(interiorCell) == 0 && cellValue(cell) == -1) {
      //ROS_INFO("Cell: [%ld,%ld]=%ld, Interior cell [%ld,%ld]=%ld Mark as FREE", cellX, cellY, cellValue(cell), interiorCellX, interiorCellY, cellValue(interiorCell));
      map.data[cell] = 0;
      return true;
    } else {
      //ROS_INFO("Cell: [%ld,%ld]=%ld, Interior cell [%ld,%ld]=%ld DON'T MARK", cellX, cellY, cellValue(cell), interiorCellX, interiorCellY, cellValue(interiorCell));
    }
  } else {
    //ROS_INFO("Cell: [%ld,%ld], Interior cell [%ld,%ld] Out of map", cellX, cellY, interiorCellX, interiorCellY);
  }
  return false;
}

float antiCrash::expandFreeCells(float radius, float resolution) {
  nav_msgs::OccupancyGrid mapCopy = m_map;
  unsigned long int counterObstacle = 0;
  unsigned long int counterFree = 0;
  int discretizedRadius = int(radius/resolution);
  for (int k = 0; k < m_map.info.width*m_map.info.height; k++) {
    if (cellValue(k) == 0) {
      counterFree++;
      geometry_msgs::Point cellPoint = cell2Point(k);
      for (int i = 0; i <= discretizedRadius*2; i++) {
        for (int j = 0; j <= discretizedRadius*2; j++) {
          float distance = getDistance2D(cellPoint.x,cellPoint.x-radius+i*resolution,cellPoint.y,cellPoint.y-radius+j*resolution);
          if (distance <= radius) {
            unsigned long int cell = point2Cell(cellPoint.x-radius+i*resolution,cellPoint.y-radius+j*resolution);
            if (cellValue(cell) == -1) {
              mapCopy.data[cell]=0;
            }
          }
        }
      }
    }
    if (cellValue(k) == 100) {
      counterObstacle++;
    }
  }
  m_map = mapCopy;
  return counterObstacle/(counterFree+counterObstacle);
}

geometry_msgs::Point antiCrash::cell2Point(unsigned long int cell) {
  geometry_msgs::Point cellPoint;
  cellPoint.x = m_map.info.origin.position.x + (cell%m_map.info.width)*m_map.info.resolution;
  cellPoint.y = m_map.info.origin.position.y + (cell/m_map.info.width)*m_map.info.resolution;
  return cellPoint;
}

unsigned long int antiCrash::point2Cell(float x,float y) {
  unsigned long int x_cell = floor0((x - m_map.info.origin.position.x)/m_map.info.resolution);
  unsigned long int y_cell = floor0((y - m_map.info.origin.position.y)/m_map.info.resolution);
  unsigned long int cell = x_cell + y_cell*m_map.info.width;
  return cell;
}

unsigned long int antiCrash::floor0(float value) {
  if (value < 0.0)
    return (unsigned long int)ceil(value);
  else
    return (unsigned long int)floor(value);
}

unsigned long int antiCrash::cellValue(unsigned long int cell){
  return m_map.data[cell];
}

float antiCrash::getDistance2D(float x1, float x2, float y1, float y2){
  return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

bool antiCrash::validateCell(long int cellX, long int cellY, nav_msgs::OccupancyGrid &map) {
  return cellX >= 0 && cellY >= 0 && cellX < map.info.width && cellY < map.info.height;
}

unsigned long int antiCrash::getCellIndex(long int cellX, long int cellY, nav_msgs::OccupancyGrid &map) {
  return cellX + (cellY * map.info.width);
}
