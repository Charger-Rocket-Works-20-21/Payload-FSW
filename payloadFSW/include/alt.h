#ifndef ALT_H_
#define ALT_H_

// Necessary Includes
#include "system.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

// Globals
extern Adafruit_BMP3XX bmp;

// Functions
bool alt_init(Adafruit_BMP3XX*);
double getSmoothTemp(double smoothingFactor, double smoothTemp);
double getSmoothPres(double smoothingFactor, double smoothPres);
double getSmoothAlt(double smoothingFactor, double smoothAlt);
double getSmoothVel(double smoothingFactor, double smoothVel, double smoothAlt, double pastTime, double currentTime);

// Test Functions
#ifdef DEBUG
void bmp3XX_test(void);
#endif /* DEBUG */

#endif /* ALT_H_ */