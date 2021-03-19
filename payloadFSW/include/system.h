/*
 * System Definitions file for Charger Rocket Works Payload Flight Software
 * NASA SLI 2020-21
 */

// All included files

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
// #include "memorysaver.h"
// #include "HardwareSerial.h"
// #include <pins_arduino.h>

#include "alt.h"
#include "ArduCAM.h"
#include "camera.h"
#include "detach.h"
#include "FlightStates.h"
#include "imu.h"
#include "level.h"
#include "thermistor.h"

// #include <utility/imumaths.h> Giving an error?
// #include "RingBuffer.h" //Giving errors?

// Constant Definitions
#define SEALEVELPRESSURE     1013.25   // hPa
#define SEALEVELTEMPERATURE  273.15    // Kelvin

#define THERMISTORPIN A9

#define MOTOR1 14
#define MOTOR2 15
#define MOTOR3 18
#define MOTOR1R 19
#define MOTOR2R 20
#define MOTOR3R 21
#define RELEASE 36

// Global Variables
// extern Adafruit_BMP3XX bmp;
// extern Adafruit_BNO055 bno;

#endif /* SYSTEM_H_ */