#include "detach.h"

SFEVL53L1X distanceSensor(Wire2, SHUTDOWN_PIN, INTERRUPT_PIN);

bool rangefinderInit() {
    if (distanceSensor.begin() != 0) { //Begin returns 0 on a good init
		Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
        return false;
	}
	Serial.println("Sensor online!");
    return true;
}

double getSmoothDistance(double smoothingFactor, double smoothDistance) {
    distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
	while (!distanceSensor.checkForDataReady()) {
		delay(1);
	}
	int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
	distanceSensor.clearInterrupt();
	distanceSensor.stopRanging();

	double distanceInches = distance * 0.0393701;
	double distanceFeet = distanceInches / 12.0;

    return smoothingFactor * distanceFeet + (1 - smoothingFactor) * smoothDistance;
}

void actuateServo(bool locked) {
	if (locked) {
		analogWrite(RELEASE, 255);
	}
	else {
		analogWrite(RELEASE, 0);
	}
}