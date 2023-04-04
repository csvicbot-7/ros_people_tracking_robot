#ifndef ANTI_CRASH_CRASH_H
#define ANTI_CRASH_CRASH_H

#include <stdio.h>
#include <math.h>

#include<ros/ros.h>
#include "geometry_msgs/Point.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "rossharedmemory.h"
#include "rossharedmemoryAnticrash.h"

using namespace std;

class antiCrash {
 public:
  antiCrash();
  void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

 private:

  ros::NodeHandle m_nh;
  ros::NodeHandle m_privateNh;

  ros::Subscriber m_mapSub;
  ros::Subscriber m_odomSub;
  ros::Publisher m_gridModPub;
  ros::Publisher m_cropedGridPub;

  nav_msgs::OccupancyGrid m_map;
  nav_msgs::OccupancyGrid m_cropedMap;

  tf::TransformListener* m_listener;
  
  float m_posX, m_posY, m_orientation;

  bool m_mapInit = false;
  bool m_mapReceived;

  // Shared memory variables
  stSharedAtoR m_sAtoR;
  CRosSharedMemory<void*, stSharedAtoR> m_shmMemory;

  /*!
  * @brief     Function to mark a the cells under the robot as Free.
  *             When robot is started, the LIDAR can't see under to robot, so there is a round area mark as Unknown cells.
  *             This cells makes that the robot can't move. This function marks Unknown cells inter the robot as Free.
  *             This functuion must be called just once when the map is received.
  * @post      m_mapInit is set to true if all cells under robot are discovered.
  * @param     radius: Distance between the sensor position and the robot's farthest edge.
  * @param     resolution: Map resolution.
  */
  void fillStartingGrid(float radius,float resolution);

  /*!
  * @brief     Function to mark a cell as Free if the interior cell is Free.
  * @param     cellX: X coordinate of the cell mark.
  * @param     cellY: Y coordinate of the cell mark.
  * @param     interiorCellX: X coordinate of the interior cell.
  * @param     interiorCellY: Y coordinate of the interior cell.
  * @param     map: Occupancy Grid map.
  * @returns   Returns true if the cell has been modified.
  */
  bool fillStartingGridCell(long int cellX, long int cellY, long int interiorCellX, long int interiorCellY, nav_msgs::OccupancyGrid &map);

  /*!
  * @brief     Function that sets the value of the cells to 0 within a radius, those cells close to the free cells.
  * @param     radius: Free cell expansion distance.
  * @param     resolution: Map resolution.
  * @returns   obstacle density.
  */
  float expandFreeCells(float radius,float resolution); // return density

  /*!
  * @brief     Function to transform a map cell into a point.
  * @param     cell: Map cell number.
  * @returns   The point coordinates (X,Y).
  */
  geometry_msgs::Point cell2Point(unsigned long int cell);

  /*!
  * @brief     Function to transform the point coordinates into a cell of the map.
  * @param     x: x coordinate.
  *            y: y coordinate.
  * @returns   Map cell number.
  */
  unsigned long int point2Cell(float x,float y);

  /*!
  * @brief     Function to get the value of a cell. This Function get the value of the map cell.
  * @param     cell: Cell to get the value.
  * @returns   Returns the value of a cell:
  *            -1: Unknown
  *            0: Free
  *            100: Occuped
  */
  unsigned long int cellValue(unsigned long int cell);
  unsigned long int floor0(float value);
  
  float getDistance2D(float x1, float x2, float y1, float y2);

  /*!
  * @brief     Function that copies a cols x rows matrix of cells from the rtabmap grid map to a new map, 
  *            setting as the center of the matrix the position of the lidar sensor.
  * @param     cols: Cols number of the new map.
  * @param     rows: Rows number of the new map.
  * @returns   New cols x rows map
  */
  void cropMap(int cols, int rows);

  /*!
  * @brief     Function to check if a X & Y coordinates exists in a OccupancyGrid map.
  * @param     cellX: X coordinate of the cell to check.
  * @param     cellY: Y coordinate of the cell to check.
  * @param     map: Occupancy Grid map.
  * @returns   Returns true if the cell exists in the map.
  */
  bool validateCell(long int cellX, long int cellY, nav_msgs::OccupancyGrid &map);

  /*!
  * @brief     Function to get the cell index of an OccupancyGrid map from a X & Y coordinates.
  * @param     cellX: X coordinate of the cell.
  * @param     cellY: Y coordinate of the cell.
  * @param     map: Occupancy Grid map.
  * @returns   Returns the cell index.
  */
  unsigned long int getCellIndex(long int cellX, long int cellY, nav_msgs::OccupancyGrid &map);
};

#endif //ANTI_CRASH_CRASH_H