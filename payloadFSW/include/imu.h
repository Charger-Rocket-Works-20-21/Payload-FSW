#ifndef IMU_H_
#define IMU_H_

// Necessary Includes
#include "system.h"

// Functions
bool imuInit(Adafruit_BNO055* sensor);
std::vector<double> getSmoothAccel(double smoothingFactor, std::vector<double> smoothAcceleration);
std::vector<double> getSmoothOrient(double smoothingFactor, std::vector<double> smoothOrientation);
uint8_t getCalibration();

#endif /* IMU_H_ */