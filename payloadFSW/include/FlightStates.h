#ifndef _FLIGHTSTATES_H
#define _FLIGHTSTATES_H

#include <stdint.h>
#include "system.h"

enum flightState { UNARMED, STANDBY, ASCENT, DESCENT, LEVELLING, FINISHED };

class States {
public:
	//void whichState(flightState newState);
	void unarmed();
	void standby(double altitude, double initialAltitude, double velocity);
	void ascent(double altitude, double initialAltitude, double velocity);
	void descent(double altitude, double velocity, gyroStruct accel, double distance);
	void levelling(double radialOrient, double tangentialOrient);
    void finished();

private:
	double oldAlt = 0;
	double currentAlt = 0;	
};


#endif // !_FLIGHTSTATES_H