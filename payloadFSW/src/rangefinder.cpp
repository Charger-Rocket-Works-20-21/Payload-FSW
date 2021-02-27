#include "rangefinder.h"

bool rangefinderInit(SFEVL53L1X distanceSensor) {
    if (distanceSensor.begin() != 0) { //Begin returns 0 on a good init
		Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
        return false;
	}
	Serial.println("Sensor online!");
    return true;
}

double getDistance(SFEVL53L1X distanceSensor) {
    distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
	while (!distanceSensor.checkForDataReady()) {
		delay(1);
	}
	int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
	distanceSensor.clearInterrupt();
	distanceSensor.stopRanging();

	double distanceInches = distance * 0.0393701;
	double distanceFeet = distanceInches / 12.0;
    return distanceFeet;
}