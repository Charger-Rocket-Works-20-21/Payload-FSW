#ifndef IMU_H_
#define IMU_H_

// Necessary Includes
#include "system.h"

// Globals
Adafruit_BNO055 bno;
sensors_event_t accelEvent;
sensors_event_t orientEvent;

// Functions
bool imuInit(Adafruit_BNO055* sensor);
gyroStruct getSmoothAccel(double smoothingFactor, gyroStruct smoothAcceleration);
gyroStruct getSmoothOrient(double smoothingFactor, gyroStruct smoothOrientation);
uint8_t getCalibration();

#endif /* IMU_H_ */