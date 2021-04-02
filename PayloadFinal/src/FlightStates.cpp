#include "FlightStates.h"

flightState currentFS = STANDBY;

double currentTime, transitionTime;
double minTimes[] = {10.0, 30.0};

void States::unarmed() {
	currentFS = UNARMED;
	//Hard lock on doing nothing - transmits sensor stuff and that's it
}

void States::standby(double altitude, double initialAltitude, double velocity) {
	currentFS = STANDBY;
	//Perform PreLaunch Operations

	if (altitude - initialAltitude >= 100.0 && velocity >= 10.0) {
		currentFS = ASCENT;
		transitionTime = millis()/1000;
	}
}

void States::ascent(double altitude, double initialAltitude, double velocity) {
	currentFS = ASCENT;
	//Perform Ascent Operations

	currentTime = millis()/1000;
	if ((currentTime - transitionTime) >= minTimes[0] && (altitude - initialAltitude) >= 1000.0 && fabs(velocity) <= 5.0) {
		currentFS = DESCENT;
		transitionTime = millis()/1000;
	}
	
}

void States::descent(double altitude, double initialAltitude, double velocity, double accelx, double accely, double accelz, double distance) {
	currentFS = DESCENT;
	//Perform Descent Operations

	currentTime = millis()/1000;
	if ((currentTime - transitionTime) >= minTimes[1] && altitude - initialAltitude < -15.0) { // TODO CHANGE BACK TO 100 METERS
		if (distance < 1.0) {
			actuateServo(false);
			delay(500);
		}
	}
	if (fabs(velocity) <= 5 && (accelx + accely + accelz) < 15.0 && altitude - initialAltitude < 50.0) {
		landedOrientx = orientx;
		landedOrienty = orienty;
		landedOrientz = orientz;
		currentFS = LEVELLING;
	}
}

void States::levelling(double radialOrient, double tangentialOrient) {
    currentFS = LEVELLING;
	//Perform Levelling Operations
	double resultCurrent = sqrt(pow((radialOrient), 2) + pow(tangentialOrient, 2)); // Resultant vector REMEMBER TO ADD BACK 90 TO RADIAL FOR SLED CONFIGURATION
	
	if (resultCurrent >= 1.0) {
		calibrateLeveler(radialOrient, tangentialOrient);

		if (oriented1 != 0 && oriented2 != 0 && oriented3 != 0) {
			if (oriented1 == 1) {
				driveMotor(1, 1);
			}
			else if (oriented1 == 2) {
				driveMotor(1, 2);
			}
			if (oriented2 == 1) {
				driveMotor(2, 1);
			}
			else if (oriented2 == 2) {
				driveMotor(2, 2);
			}
			if (oriented3 == 1) {
				driveMotor(3, 1);
			}
			else if (oriented3 == 2) {
				driveMotor(3, 2);
			}
			if (hasChanged(resultCurrent, resultPrevious) != 1) {
				resetCalibration();
			}
		}
	}
	else {	// If Levelled:
		driveMotor(1, 0);
		driveMotor(2, 0);
		driveMotor(3, 0);

		leveledx = orientx;
		leveledy = orienty;
		leveledz = orientz;

		String packet = "";
		packet += "LEVELLING RESULTS";
		packet += ",";
		packet += landedOrientx;
		packet += ",";
		packet += landedOrienty;
		packet += ",";
		packet += landedOrientz;
		packet += ",";
		packet += leveledx;
		packet += ",";
		packet += leveledy;
		packet += ",";
		packet += leveledz;
		packet += "LEVELLING RESULTS END";
				
		File dataFile = SD.open("datalog.txt", FILE_WRITE);
		if (dataFile) {
			dataFile.println(packet);
			dataFile.close();
		}
		else {
			Serial.println("Could not open datalog.txt");
		}
		
		delay(1000);
		currentFS = FINISHED;
	}	
}

void States::finished() {
	currentFS = FINISHED;
	//Decrease transmission rate
	//Run until powered off
}

void States::actuateServo(bool locked) {
	Servo release1;
	Servo release2;
	if (locked) {
		release1.detach();
		release2.detach();
		digitalWrite(RELEASE_POWER1, LOW);
		digitalWrite(RELEASE_POWER2, LOW);	
	}
	else {
		digitalWrite(RELEASE_POWER1, HIGH);
		digitalWrite(RELEASE_POWER2, HIGH);
		release1.attach(RELEASE1);
		release2.attach(RELEASE2);
		release1.write(160);
		release2.write(0);
		delay(250);
		release1.detach();
		release2.detach();
		digitalWrite(RELEASE_POWER1, LOW);
		digitalWrite(RELEASE_POWER2, LOW);
	}
}

void States::setCurrentState(uint8_t stateID) {
	if (stateID == 0) {
		this->currentState = UNARMED;
	} 
	else if (stateID == 1) {
		this->currentState = STANDBY;
	}
	else if (stateID == 2) {
		this->currentState = ASCENT;
	}
	else if (stateID == 3) {
		this->currentState = DESCENT;
	}
	else if (stateID == 4) {
		this->currentState = LEVELLING;
	}
	else if (stateID == 5) {
		this->currentState = FINISHED;
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
