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
#include "memorysaver.h"

#include "alt.h"
#include "camera.h"
#include "imu.h"
#include "thermistor.h"
#include "FlightStates.h"
#include "level.h"
#include "detach.h"

// #include <utility/imumaths.h> Giving an error?
#include "RingBuffer.h" //Giving errors?

// Constant Definitions
#define SEALEVELPRESSURE     1013.25   // hPa
#define SEALEVELTEMPERATRE   273.15    // Kelvin

#define THERMISTORPIN A9

#define MOTOR1 8//14
#define MOTOR2 9//15
#define MOTOR3 10//18
#define MOTOR1R 19
#define MOTOR2R 20
#define MOTOR3R 21
#define RELEASE 36

// Global Variables
struct gyroStruct {
	float x;
	float y;
	float z;
};
#endif /* SYSTEM_H_ */