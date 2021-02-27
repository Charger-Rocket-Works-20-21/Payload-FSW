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
std::vector<double> getSmoothAccel(double smoothingFactor, std::vector<double> smoothAcceleration);
std::vector<double> getSmoothOrient(double smoothingFactor, std::vector<double> smoothOrientation);

#endif /* IMU_H_ */