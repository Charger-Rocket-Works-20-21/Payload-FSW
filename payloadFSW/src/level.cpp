#include "level.h"

double tolerance = 5.0;
bool calibrated, initialized;
int oriented1, oriented2, oriented3; //0 for untested, 1 for helpful, 2 for hurtful
double resultCurrent, resultPrevious, resultInitial;

// Return 0 for no change
// Return 1 for good change
// Return 2 for bad change
int hasChanged (double currentOrient, double previousOrient) {
	if (previousOrient - currentOrient > tolerance) {
		return 1;
	}
	else if (previousOrient - currentOrient < -tolerance) {
		return 2;
	}
	else {
		return 0;
	}
}

void driveMotor (int motorNumber, int direction) {
	int onPin, reversePin;
	if (motorNumber == 1) {
		onPin = MOTOR1;
		reversePin = MOTOR1R;
	}
	else if (motorNumber == 2) {
		onPin = MOTOR2;
		reversePin = MOTOR2R;
	}
	else { // (motorNumber == 3)
		onPin = MOTOR3;
		reversePin = MOTOR3R;
	}
	if (direction == 0) {
		digitalWrite(onPin, LOW);
	}
	else if (direction == 1) {
		digitalWrite(reversePin, LOW);
		digitalWrite(onPin, HIGH);
	}
	else if (direction == 2) {
		digitalWrite(reversePin, HIGH);
		digitalWrite(onPin, HIGH);
	}
}

void resetCalibration() {
	oriented1 = 0;
	oriented2 = 0;
	oriented3 = 0;

	driveMotor(1, 0);
	driveMotor(2, 0);
	driveMotor(3, 0);
}

void calibrateLeveler(double radialOrient, double tangentialOrient) {
	if (!calibrated && calibration >= 8) {
		calibrated = true;
	}

	if (calibrated && !initialized) {
		resultInitial = sqrt(pow((radialOrient), 2) + pow(tangentialOrient, 2)); // Resultant vector REMEMBER TO ADD BACK 90 TO RADIAL FOR SLED CONFIGURATION
		initialized = true;
	}

	if (initialized && oriented1 == 0) {
		driveMotor(1, 1);
		bool helping = hasChanged(resultCurrent, resultPrevious);
		if (helping != 0) {
			oriented1 = helping;
			driveMotor(1, 0);
		}
	}

	else if (oriented1 != 0 && oriented2 == 0) {
		driveMotor(2, 1);
		bool helping = hasChanged(resultCurrent, resultPrevious);
		if (helping != 0) {
			oriented2 = helping;
			driveMotor(2, 0);
		}
	}

	else if (oriented2 != 0 && oriented3 == 0) {
		driveMotor(3, 1);
		bool helping = hasChanged(resultCurrent, resultPrevious);
		if (helping != 0) {
			oriented3 = helping;
			driveMotor(3, 0);
		}
	}
}