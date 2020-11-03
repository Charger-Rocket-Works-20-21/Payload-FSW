#ifndef ALT_H_
#define ALT_H_

// Necessary Includes
#include "system.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

// Globals
extern Adafruit_BMP3XX bmp;

// Functions
void alt_init(Adafruit_BMP3XX*);

// Test Functions
#ifdef DEBUG
void bmp3XX_test(void);
#endif /* DEBUG */

#endif /* ALT_H_ */