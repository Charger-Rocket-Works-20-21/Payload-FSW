/*
 * System Definitions file for Charger Rocket Works Payload Flight Software
 * NASA SLI 2020-21
 */

// All included files

#define DEBUG

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <utility/imumaths.h>
#include "RingBuffer.h"


// Definitions
#define SEALEVELPRESSURE     1013.25   // hPa
#define SEALEVELTEMPERATRE   273.15    // Kelvin


// Global Variables
extern sensors_event_t event;
extern Adafruit_BNO055 bno;
extern Adafruit_BMP3XX bmp;


void imu_init(Adafruit_BNO055*);
void alt_init(Adafruit_BMP3XX*);

// For Debugging
#ifdef DEBUG

void bno055_test(void);
void bmp3XX_test(void);

#endif /* DEBUG */

#endif /* RINGBUFFER_H_ */