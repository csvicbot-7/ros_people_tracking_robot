#ifndef ROSSHAREDMEMORYEXPLORATION_H
#define ROSSHAREDMEMORYEXPLORATION_H

#define SHARED_NAME_EXPLORATION "/ExplorationSharedmemory"

//Exploration to Robot
struct stSharedEtoR {
	float speedLeft;
	float speedRight;
	bool finish;
	bool error;
	bool calculating;
};
//Robot to Exploration

enum DensityLevel {HIGH_DENSITY = 1, REGULAR_DENSITY = 2, LOW_DENSITY =3};

struct stSharedRtoE {
	bool start;
	DensityLevel densityLevel;
};

#endif // ROSSHAREDMEMORYEXPLORATION_H
