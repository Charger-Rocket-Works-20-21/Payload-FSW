#ifndef RANGEFINDER_H_
#define RANGEFINDER_H_

// Necessary Includes
#include "system.h"

#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensor(Wire2, SHUTDOWN_PIN, INTERRUPT_PIN);

// Functions
bool rangefinderInit(SFEVL53L1X distanceSensor);
double getDistance(SFEVL53L1X distanceSensor);

#endif /* RANGEFINDER_H_ */