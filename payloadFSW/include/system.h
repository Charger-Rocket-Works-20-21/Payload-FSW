/*
 * System Definitions file for Charger Rocket Works Payload Flight Software
 * NASA SLI 2020-21
 */

// All included files

#define DEBUG
// #define USESD

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include <vector>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#ifdef USESD
    #include <SD.h>
#endif
// #include <utility/imumaths.h> Giving an error?
#include "RingBuffer.h" //Giving errors?

// Constant Definitions
#define SEALEVELPRESSURE     1013.25   // hPa
#define SEALEVELTEMPERATRE   273.15    // Kelvin

#define DROPTEST

// Global Variables
#endif /* SYSTEM_H_ */