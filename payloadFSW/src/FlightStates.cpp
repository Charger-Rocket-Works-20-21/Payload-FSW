#include "FlightStates.h"
#include <math.h>

using namespace std;

flightState currentFS = UNARMED;

void States::unarmed() {
	currentFS = UNARMED;
	//Hard lock on doing nothing - transmits sensor stuff and that's it
	
}

void States::standby(double altitude, double initialAltitude, double velocity) {
	currentFS = STANDBY;
	//Perform Ground Operations

	if (altitude - initialAltitude >= 30 && velocity >= 20) {
		currentFS = ASCENT;
	}
}

void States::ascent(double altitude, double initialAltitude, double velocity) {
	currentFS = ASCENT;
	//Perform Ascent Operations

	if ((altitude - initialAltitude) >= 500 && fabs(velocity) <= 5) {
		currentFS = DESCENT;
	}
	
}

void States::descent(double velocity, std::vector<double> accel) {
	currentFS = DESCENT;
	//Perform Descent Operations

	if (fabs(velocity) <= 5 && accel.at(0) < .05) {
		currentFS = LEVELLING;
	}
}

void States::levelling(){
    currentFS = LEVELLING;
	//Perform Levelling Operations

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

bool States::dropTest(uint16_t packetCount, double altitude) {
	currentFS = TEST;

	if (packetCount % 200 == 0) {
		oldAlt = currentAlt;
		currentAlt = altitude;
		if (oldAlt - currentAlt > 4) {
			return true;
		}
		else {
			return false;
		}
	}
	return false;
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