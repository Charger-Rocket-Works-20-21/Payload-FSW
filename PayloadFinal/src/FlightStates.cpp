#include "FlightStates.h"

flightState currentState = UNARMED;

double currentTime, transitionTime;
double minTimes[] = {10.0, 30.0};
double tolerance = 0.025;
int activeMotor = 1;

void States::unarmed() {
	currentState = UNARMED;
	//Hard lock on doing nothing - transmits sensor stuff and that's it
}

void States::standby(double altitude, double initialAltitude, double velocity) {
	currentState = STANDBY;
	//Perform PreLaunch Operations

	if (altitude - initialAltitude >= 100.0 && velocity >= 10.0) {
		currentState = ASCENT;
		transitionTime = millis()/1000;
	}
}

void States::ascent(double altitude, double initialAltitude, double velocity) {
	currentState = ASCENT;
	//Perform Ascent Operations

	currentTime = millis()/1000;
	if ((currentTime - transitionTime) >= minTimes[0] && (altitude - initialAltitude) >= 1000.0 && fabs(velocity) <= 5.0) {
		currentState = DESCENT;
		transitionTime = millis()/1000;
	}
	
}

void States::descent(double altitude, double initialAltitude, double velocity, double accelx, double accely, double accelz, double distance) {
	currentState = DESCENT;
	//Perform Descent Operations

	currentTime = millis()/1000;
	if ((currentTime - transitionTime) >= minTimes[1] && altitude - initialAltitude < -15.0) { // TODO CHANGE BACK TO 100 METERS
		if (distance < 7.0) {
			actuateServo(false);
			delay(500);
			if (fabs(velocity) <= 5 && (accelx + accely + accelz) < 15.0 && altitude - initialAltitude < 50.0) {
				landedOrientx = orientxCorrected;
				landedOrienty = orienty;
				landedOrientz = orientz;
				currentState = LEVELLING;
			}
		}
	}
}

void States::leveling(double current, double previous) {
    currentState = LEVELLING;
	//Perform Levelling Operations
	
	if (current >= 4.0) {
		if (activeMotor > 3) {
			activeMotor = 1;
		}

		if (hasChanged(current, previous) != 2) {
			driveMotor(activeMotor, 1);
		}

		if (hasChanged(current, previous) == 2) {
			driveMotor(activeMotor, 2);
			delay(100);
			driveMotor(activeMotor, 0);
			activeMotor++;
		}
	}
	else {	// If Levelled:
		driveMotor(1, 0);
		driveMotor(2, 0);
		driveMotor(3, 0);

		leveledOrientx = orientxCorrected;
		leveledOrienty = orienty;
		leveledOrientz = orientz;

		String packet = "";
		packet += "LEVELLING RESULTS";
		packet += ",";
		packet += landedOrientx;
		packet += ",";
		packet += landedOrienty;
		packet += ",";
		packet += landedOrientz;
		packet += ",";
		packet += leveledOrientx;
		packet += ",";
		packet += leveledOrienty;
		packet += ",";
		packet += leveledOrientz;
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
		currentState = FINISHED;
	}	
}

void States::finished() {
	currentState = FINISHED;
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
		release1.write(60);
		release2.write(60);
		// while(1);
		delay(250);
		release1.detach();
		release2.detach();
		digitalWrite(RELEASE_POWER1, LOW);
		digitalWrite(RELEASE_POWER2, LOW);
	}
}

// Return 0 for no change
// Return 1 for good change
// Return 2 for bad change
int States::hasChanged (double currentOrient, double previousOrient) {
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

void States::driveMotor (int motorNumber, int direction) {
	int onPin, reversePin;
	if (motorNumber == 1) {
		onPin = MOTOR1;
		reversePin = MOTOR1R;
	}
	else if (motorNumber == 2) {
		onPin = MOTOR2;
		reversePin = MOTOR2R;
	}
	else if (motorNumber == 3) {
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
