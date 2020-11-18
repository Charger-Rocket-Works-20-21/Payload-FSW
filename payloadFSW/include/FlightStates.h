#ifndef _FLIGHTSTATES_H
#define _FLIGHTSTATES_H

#include <stdint.h>

enum flightState { UNARMED, STANDBY, ASCENT, DESCENT, LEVELLING, FINISHED };

class States {
public:
	//void whichState(flightState newState);
	void unarmed();
	void standby(double altitude, double initialAltitude, double velocity);
	void ascent(double altitude, double initialAltitude, double velocity);
	void descent(double velocity, double gForce);
	void levelling();
    void finished();
};


#endif // !_FLIGHTSTATES_H