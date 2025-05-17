#ifndef IMUDATA_H
#define IMUDATA_H
#include "main.h"
typedef struct IMUData{
	int16_t rawAccelX;
	int16_t rawAccelY;
	int16_t rawAccelZ;
	float accelX;
	float accelY;
	float accelZ;
	float totalAcc;
	float angleRadians;
	int8_t angleDegrees;
}IMUData;

void getAngle(IMUData* imuData);
void getTotalAcc(IMUData* imuData);

#endif
