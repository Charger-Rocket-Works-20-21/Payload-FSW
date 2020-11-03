// All included files
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <utility/imumaths.h>
#include "include/RingBuffer.h"


// Definitions
#define SEALEVELPRESSURE     1013.25   // hPa
#define SEALEVELTEMPERATRE   273.15    // Kelvin