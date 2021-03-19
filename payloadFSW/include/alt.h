#ifndef ALT_H_
#define ALT_H_

// Necessary Includes
#include "system.h"

// Functions
bool altInit(Adafruit_BMP3XX*);
double getSmoothTemp(double smoothingFactor, double smoothTemp);
double getSmoothPres(double smoothingFactor, double smoothPres);
double getSmoothAlt(double smoothingFactor, double smoothAlt);
double getSmoothVel(double smoothingFactor, double smoothVel, double smoothAlt, double pastTime, double currentTime);

#endif /* ALT_H_ */