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
	void descent(double velocity, std::vector<double> accel);
	void levelling(std::vector<double> orientation, uint32_t timems);
    void finished();

private:
	double oldAlt = 0;
	double currentAlt = 0;	
};


#endif // !_FLIGHTSTATES_H