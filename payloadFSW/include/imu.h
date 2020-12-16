#ifndef IMU_H_
#define IMU_H_

// Necessary Includes
#include "system.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Globals
extern sensors_event_t event;
extern Adafruit_BNO055 bno;

// Functions
bool imu_init(Adafruit_BNO055* sensor);
std::vector<double> resultantAccel(double smoothingFactor, std::vector<double> smoothAcceleration);
std::vector<double> resultantOrient(double smoothingFactor, std::vector<double> smoothOrientation);

// Test Functions
#ifdef DEBUG
void bno055_test(void);
#endif /* DEBUG */

#endif /* IMU_H_ */