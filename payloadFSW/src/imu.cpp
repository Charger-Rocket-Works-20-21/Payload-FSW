/* Reference: 
 *    - https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
 *    - https://learn.adafruit.com/using-the-adafruit-unified-sensor-driver/how-does-it-work
 */
#include "imu.h"

// Globals
Adafruit_BNO055 bno;
sensors_event_t accelEvent;
sensors_event_t orientEvent;

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

std::vector<double> getSmoothAccel(double smoothingFactor, std::vector<double> smoothAcceleration) {
	bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
	Serial.print(accelEvent.acceleration.x);
	Serial.print(accelEvent.acceleration.y);
	Serial.println(accelEvent.acceleration.z);
	std::vector<double> resultantVector;
	double accelx = smoothingFactor * accelEvent.acceleration.x + (1 - smoothingFactor) * smoothAcceleration.at(0);
	double accely = smoothingFactor * accelEvent.acceleration.y + (1 - smoothingFactor) * smoothAcceleration.at(1);
	double accelz = smoothingFactor * accelEvent.acceleration.z + (1 - smoothingFactor) * smoothAcceleration.at(2);

	resultantVector.push_back(accelx);
	resultantVector.push_back(accely);
	resultantVector.push_back(accelz);
	return resultantVector;
}

std::vector<double> getSmoothOrient(double smoothingFactor, std::vector<double> smoothOrientation) {
	bno.getEvent(&orientEvent);

	std::vector<double> resultantVector;
	double orientx = smoothingFactor * orientEvent.orientation.x + (1 - smoothingFactor) * smoothOrientation.at(0);
	double orienty = smoothingFactor * orientEvent.orientation.y + (1 - smoothingFactor) * smoothOrientation.at(1);
	double orientz = smoothingFactor * orientEvent.orientation.z + (1 - smoothingFactor) * smoothOrientation.at(2);

	resultantVector.push_back(orientx);
	resultantVector.push_back(orienty);
	resultantVector.push_back(orientz);
	return resultantVector;
	return resultantVector;
}

uint8_t getCalibration() {
	uint8_t sys, gyro, accel, mag = 0;
	bno.getCalibration(&sys, &gyro, &accel, &mag);
	return sys + gyro + accel + mag;
}