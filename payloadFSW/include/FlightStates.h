#ifndef _FLIGHTSTATES_H
#define _FLIGHTSTATES_H

#include "system.h"

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
	void descent(double altitude, double velocity, std::vector<double> accel, double distance);
	void levelling(double radialOrient, double tangentialOrient);
    void finished();

private:
	// double oldAlt = 0;
	// double currentAlt = 0;	
};


#endif // !_FLIGHTSTATES_H