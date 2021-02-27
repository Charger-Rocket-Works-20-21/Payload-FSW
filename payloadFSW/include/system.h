/*
 * System Definitions file for Charger Rocket Works Payload Flight Software
 * NASA SLI 2020-21
 */

// All included files

#define DEBUG

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include <vector>

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>

#include "alt.h"
#include "imu.h"
#include "thermistor.h"
#include "FlightStates.h"
#include "pid.h"
#include "rangefinder.h"

// #include <utility/imumaths.h> Giving an error?
#include "RingBuffer.h" //Giving errors?

// Constant Definitions
#define SEALEVELPRESSURE     1013.25   // hPa
#define SEALEVELTEMPERATRE   273.15    // Kelvin

#define THERMISTORPIN A9

#define DROPTEST

// Global Variables
#endif /* SYSTEM_H_ */