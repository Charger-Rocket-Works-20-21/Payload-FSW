#include "FlightStates.h"

flightState currentFS = UNARMED;

void States::unarmed() {
	currentFS = UNARMED;
	//Hard lock on doing nothing - transmits sensor stuff and that's it
}

void States::standby(double altitude, double initialAltitude, double velocity) {
	currentFS = STANDBY;
	//Perform PreLaunch Operations

	if (altitude - initialAltitude >= 100 && velocity >= 10) {
		currentFS = ASCENT;
	}
}

void States::ascent(double altitude, double initialAltitude, double velocity) {
	currentFS = ASCENT;
	//Perform Ascent Operations

	if ((altitude - initialAltitude) >= 1000 && fabs(velocity) <= 5) {
		currentFS = DESCENT;
	}
	
}

void States::descent(double altitude, double velocity, double accelx, double accely, double accelz, double distance) {
	currentFS = DESCENT;
	//Perform Descent Operations

	if (altitude < 100) {
		if (distance < 4.0) {
			actuateServo(false);
		}
	}
	if (fabs(velocity) <= 5 && (accelx + accely + accelz) < 10.0 && altitude < 50) {
		currentFS = LEVELLING;
	}
}

void States::levelling(double radialOrient, double tangentialOrient) {
    currentFS = LEVELLING;
	//Perform Levelling Operations
	double resultCurrent = sqrt(pow((radialOrient), 2) + pow(tangentialOrient, 2)); // Resultant vector REMEMBER TO ADD BACK 90 TO RADIAL FOR SLED CONFIGURATION
	
	if (resultCurrent >= 5.0) {
		calibrateLeveler(radialOrient, tangentialOrient);

		if (oriented1 != 0 && oriented2 != 0 && oriented3 != 0) {
			if (oriented1 == 1) {
				driveMotor(1, 1);
			}
			if (oriented2 == 1) {
				driveMotor(2, 1);
			}
			if (oriented3 == 1) {
				driveMotor(3, 1);
			}
			if (hasChanged(resultCurrent, resultPrevious) != 1) {
				resetCalibration();
			}
		}
	}
	else {
		driveMotor(1, 0);
		driveMotor(2, 0);
		driveMotor(3, 0);
	}

	// If Levelled:
	// Transmit Photos

	// If recieve receipt confirmation:
	currentFS = FINISHED;
}

void States::finished() {
	currentFS = FINISHED;
	//Decrease transmission rate
	//Run until powered off
}

void States::actuateServo(bool locked) {
	if (locked) {
		analogWrite(RELEASE_PWM, 255);
	}
	else {
		analogWrite(RELEASE_PWM, 0);
	}
}

/*void States::whichState(flightState newState) {
	switch (newState) {
	case UNARMED:
		//Just chillin'.
		cout << "UNARMED" << endl;
		unarmed();
		break;
	case STANDBY:
		//I'M READY!!!
		cout << "STANDBY" << endl;
		standby(smoothAltitude, initialAltitude);
		break;
	case ASCENT:
		//Here we go!
		cout << "ASCENT" << endl;
		ascent(velocity);
		break;
	case DESCENT:
		//Down, down, down...
		cout << "DESCENT" << endl;
		descent(velocity, gForce);
		break;
	case LANDING:
		//Getting kinda lonely, someone come find me.
		cout << "LANDING" << endl;
		landing();
		break;
	}
}*/
