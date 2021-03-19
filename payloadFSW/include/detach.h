#ifndef RANGEFINDER_H_
#define RANGEFINDER_H_

// Necessary Includes
#include "system.h"

#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

// Functions
bool rangefinderInit();
double getSmoothDistance(double smoothingFactor, double smoothDistance);
void actuateServo(bool locked);

#endif /* RANGEFINDER_H_ */