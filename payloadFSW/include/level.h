#ifndef LEVEL_H_
#define LEVEL_H_

#include "system.h"

extern uint8_t calibration;

int hasChanged (double currentOrient, double previousOrient);
void driveMotor (int motorNumber, int direction);
void resetCalibration();
void calibrateLeveler(double radialOrient, double tangentialOrient);

#endif /* LEVEL_H_ */