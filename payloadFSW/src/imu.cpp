/* Reference: 
 *    - https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
 *    - https://learn.adafruit.com/using-the-adafruit-unified-sensor-driver/how-does-it-work
 */
#include "imu.h"


// Global Variables
sensors_event_t event;
Adafruit_BNO055 bno = Adafruit_BNO055(55);


// ------------------- Functions -----------------------

/*
 * Inertial Measurement Unit Inititialization
 */
bool imu_init(Adafruit_BNO055* sensor){
	if (! sensor->begin()){
    	Serial.println("BNO055 Not Detected");
		return false;
	}
  
	delay(1000);
  
	sensor->setExtCrystalUse(true);
	return true;
}

std::vector<double> resultantAccel(double smoothingFactor, std::vector<double> smoothAcceleration) {
	bno.getEvent(&event);

	double accelx = smoothingFactor * event.acceleration.x + (1 - smoothingFactor) * smoothAcceleration.at(0);
	double accely = smoothingFactor * event.acceleration.y + (1 - smoothingFactor) * smoothAcceleration.at(1);
	double accelz = smoothingFactor * event.acceleration.z + (1 - smoothingFactor) * smoothAcceleration.at(2);

	std::vector<double> resultantVector;
	resultantVector.push_back(accelx);
	resultantVector.push_back(accely);
	resultantVector.push_back(accelz);
	return resultantVector;
}

std::vector<double> resultantOrient(double smoothingFactor, std::vector<double> smoothOrientation) {
	bno.getEvent(&event);

	double orientx = smoothingFactor * event.orientation.x + (1 - smoothingFactor) * smoothOrientation.at(0);
	double orienty = smoothingFactor * event.orientation.y + (1 - smoothingFactor) * smoothOrientation.at(1);
	double orientz = smoothingFactor * event.orientation.z + (1 - smoothingFactor) * smoothOrientation.at(2);

	std::vector<double> resultantVector;
	resultantVector.push_back(orientx);
	resultantVector.push_back(orienty);
	resultantVector.push_back(orientz);
	return resultantVector;
}

// -------------------- Test Functions ---------------------
#ifdef DEBUG

void bno055_test(void){
    bno.getEvent(&event);

    Serial.print("X: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.orientation.z, 4);
    Serial.println("");
}

#endif