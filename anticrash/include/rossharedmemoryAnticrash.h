#ifndef ROSSHAREDMEMORYANTICRASH_H
#define ROSSHAREDMEMORYANTICRASH_H

#define SHARED_NAME_ANTICRASH "/AnticrashSharedmemory"

#define OCCUPANCY_MAP_WIDE			800
#define OCCUPANCY_CELL_UNKNOWN		static_cast<unsigned char>(-1)
#define OCCUPANCY_CELL_FREE			static_cast<unsigned char>(0)
#define OCCUPANCY_CELL_OCCUPED		static_cast<unsigned char>(100)

// Anticrash to Robot
struct stSharedAtoR {
	/*!
	 * \brief Gris size, size of each side of a grid (in meters).
	 */
	float gridSize;
	/*!
	 *\brief Robot is centered in position OCCUPANCY_MAP_WIDE/2, OCCUPANCY_MAP_WIDE/2.
	 *		First index is X coordeante, second index is Y coordenate.
	 *		Cell [0][0] is the bottom left cell.
	 *		Must control yaw to consider robot orientation. Values:
	 *             -1: Unknown
	 *              0: Free
	 *            100: Occuped
	 */
	unsigned char occupancyGrid[OCCUPANCY_MAP_WIDE][OCCUPANCY_MAP_WIDE];
	/*!
	 * \brief Robot orientation in the map (in radians). Angle 0 means robot heading to positive X, angle clockwise watching from top view.
	 */
	float orientation;

	/*!
	 * \brief Map's X position. Coordenate X of the bottom left corner of the map, in meters.
	 */
	float mapPositionX;
	/*!
	 * \brief Map's Y position. Coordenate Y of the bottom left corner of the map, in meters.
	 */
	float mapPositionY;
	/*!
	 * \brief Map's X index. Index X of the bottom left corner of the map.
	 */
	int mapIndexX;
	/*!
	 * \brief Map's Y index. Index Y of the bottom left corner of the map.
	 */
	int mapIndexY;
};

#endif // ROSSHAREDMEMORYANTICRASH_H
