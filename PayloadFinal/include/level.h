#ifndef LEVEL_H_
#define LEVEL_H_

#include <Arduino.h>

#define MOTOR1 14
#define MOTOR2 15
#define MOTOR3 18
#define MOTOR1R 19
#define MOTOR2R 20
#define MOTOR3R 21

extern uint8_t calibration;

int hasChanged (double currentOrient, double previousOrient);
void driveMotor (int motorNumber, int direction);
void resetCalibration();
void calibrateLeveler(double radialOrient, double tangentialOrient);

#endif /* LEVEL_H_ */