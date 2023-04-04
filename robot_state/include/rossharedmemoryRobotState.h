#ifndef ROSSHAREDMEMORYROBOTSTATE_H
#define ROSSHAREDMEMORYROBOTSTATE_H

#define SHARED_NAME_ROBOT_STATE		"/RobotStateSharedmemory"

//Robot to RobotState
struct stSharedRtoRS {
	/*!
	 * \brief Left wheel advance speed, in m/s
	 */
	float vtankL;
	/*!
	 * \brief Right wheel advance speed, in m/s.
	 */
	float vtankR;
	/*!
	 * \brief TF from the base_link (robot's footprint center) to the LIDAR position, X coordenate (in meters).
	 */
	float lidarTfX;
	/*!
	 * \brief TF from the base_link (robot's footprint center) to the LIDAR position, Y coordenate (in meters).
	 */
	float lidarTfY;
	/*!
	 * \brief TF from the base_link (robot's footprint center) to the LIDAR position, Z coordenate (in meters).
	 */
	float lidarTfZ;
	/*!
	 * \brief TF from the base_link (robot's footprint center) to the LIDAR position, roll angle (in radians).
	 */
	float lidarTfRoll;
	/*!
	 * \brief TF from the base_link (robot's footprint center) to the LIDAR position, pitch angle (in radians).
	 */
	float lidarTfPitch;
	/*!
	 * \brief TF from the base_link (robot's footprint center) to the LIDAR position, yaw angle (in radians).
	 */
	float lidarTfYaw;
	/*!
	 * \brief Robot's footprint length (in meters).
	 */
	float fpLength;
	/*!
	 * \brief Robot's footprint width (in meters).
	 */
	float fpWidth;
	/*!
	 * \brief Robot's footprint height (in meters).
	 */
	float fpHeight;
};

#endif // ROSSHAREDMEMORYROBOTSTATE_H
