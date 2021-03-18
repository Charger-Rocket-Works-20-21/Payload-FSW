#ifndef RANGEFINDER_H_
#define RANGEFINDER_H_

// Necessary Includes
#include "system.h"

#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensor(Wire2, SHUTDOWN_PIN, INTERRUPT_PIN);

// Functions
bool rangefinderInit();
double getSmoothDistance(double smoothingFactor, double smoothDistance);
void actuateServo(bool locked);

#endif /* RANGEFINDER_H_ */