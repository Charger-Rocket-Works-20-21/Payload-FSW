#ifndef ALT_H_
#define ALT_H_

// Necessary Includes
#include "system.h"

// Functions
bool altInit(Adafruit_BMP3XX* sensor);
double getSmoothBmpTemp(Adafruit_BMP3XX* sensor, double smoothingFactor, double smoothTemp);
double getSmoothPres(Adafruit_BMP3XX* sensor, double smoothingFactor, double smoothPres);
double getSmoothAlt(Adafruit_BMP3XX* sensor, double smoothingFactor, double smoothAlt);
double getSmoothVel(double smoothingFactor, double smoothVel, double smoothAlt, double pastTime, double missionTime);

#endif /* ALT_H_ */