/* Reference: 
 *    - https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
 *    - https://learn.adafruit.com/using-the-adafruit-unified-sensor-driver/how-does-it-work
 */
#include "imu.h"

// ------------------- Functions -----------------------

/*
 * Inertial Measurement Unit Inititialization
 */
bool imuInit(Adafruit_BNO055* sensor){
	Serial.println("Checking IMU Init");
	if (! sensor->begin()){
    	Serial.println("BNO055 Not Detected");
		return false;
	}

	Serial.println("BNO055 Detected");  
	delay(1000);
  
	sensor->setExtCrystalUse(true);
	return true;
}

gyroStruct getSmoothAccel(double smoothingFactor, gyroStruct smoothAcceleration) {
	bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);

	gyroStruct resultantVector;
	resultantVector.x = smoothingFactor * accelEvent.acceleration.x + (1 - smoothingFactor) * smoothAcceleration.x;
	resultantVector.y = smoothingFactor * accelEvent.acceleration.y + (1 - smoothingFactor) * smoothAcceleration.y;
	resultantVector.z = smoothingFactor * accelEvent.acceleration.z + (1 - smoothingFactor) * smoothAcceleration.z;

	return resultantVector;
}

gyroStruct getSmoothOrient(double smoothingFactor, gyroStruct smoothOrientation) {
	bno.getEvent(&orientEvent);

	gyroStruct resultantVector;
	resultantVector.x = smoothingFactor * orientEvent.orientation.x + (1 - smoothingFactor) * smoothOrientation.x;
	resultantVector.y = smoothingFactor * orientEvent.orientation.y + (1 - smoothingFactor) * smoothOrientation.y;
	resultantVector.z = smoothingFactor * orientEvent.orientation.z + (1 - smoothingFactor) * smoothOrientation.z;

	return resultantVector;
}

uint8_t getCalibration() {
	uint8_t sys, gyro, accel, mag = 0;
	bno.getCalibration(&sys, &gyro, &accel, &mag);
	return sys + gyro + accel + mag;
}