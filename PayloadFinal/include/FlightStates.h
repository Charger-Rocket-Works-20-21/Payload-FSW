#ifndef _FLIGHTSTATES_H
#define _FLIGHTSTATES_H

#include <math.h>
#include <Arduino.h>
#include <Servo.h>
#include "level.h"

#define RELEASE_POWER 36
#define RELEASE_PWM	2

#define RELEASE1 36
#define RELEASE2 2

enum flightState { UNARMED, STANDBY, ASCENT, DESCENT, LEVELLING, FINISHED };

extern bool calibrated, initialized;
extern int oriented1, oriented2, oriented3; //0 for untested, 1 for helpful, 2 for hurtful
extern double resultCurrent, resultPrevious, resultInitial;

class States {
public:
	//void whichState(flightState newState);
	void unarmed();
	void standby(double altitude, double initialAltitude, double velocity);
	void ascent(double altitude, double initialAltitude, double velocity);
	void descent(double altitude, double velocity, double accelx, double accely, double accelz, double distance);
	void levelling(double radialOrient, double tangentialOrient);
    void finished();
	void actuateServo(bool locked);	

	flightState currentState;
private:
	// double oldAlt = 0;
	// double currentAlt = 0;
};


#endif // !_FLIGHTSTATES_H