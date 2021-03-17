#include "system.h"

#ifndef LEVEL_H_
#define LEVEL_H_

double tolerance = 5.0;
bool calibrated, initialized;
int oriented1, oriented2, oriented3; //0 for untested, 1 for helpful, 2 for hurtful
double resultCurrent, resultPrevious, resultInitial;

extern uint8_t calibration;

int hasChanged (double currentOrient, double previousOrient);
void driveMotor (int motorNumber, int direction);
void resetCalibration();
void calibrateLeveler(double radialOrient, double tangentialOrient);

#endif /* LEVEL_H_ */