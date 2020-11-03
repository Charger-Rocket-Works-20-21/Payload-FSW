/*
 * Main sketch for Charger Rocket Works Payload Flight Software
 * NASA SLI 2020-21
 * 
 * Uses the following libraries:
 * Adafruit Unified Sensor
 * Adafruit BNO055
 * Adafruit BusIO
 * Adafruit BMP3XX
 */

#include "src/system.h"
#include "src/alt.h"
#include "src/imu.h"

void setup() {
  // Initialize
  Serial.begin(115200);

  // Initialize IMU
  imu_init(&bno);

  // Initialize Altimeter
  alt_init(&bmp); 
}

void loop() {
  // Get IMU event
  bno055_test();
  bmp3XX_test();
    
  delay(100);
}
