#ifndef _FLIGHTSTATES_H
#define _FLIGHTSTATES_H

#include <math.h>
#include <stdlib.h>
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
// #include "level.h"
#include "ArduCAM.h"

#define RELEASE_POWER1 36
#define RELEASE_POWER2 32
#define RELEASE1 2
#define RELEASE2 4

#define MOTOR1 14
#define MOTOR2 15
#define MOTOR3 18
#define MOTOR1R 19
#define MOTOR2R 20
#define MOTOR3R 21

enum flightState { UNARMED, STANDBY, ASCENT, DESCENT, LEVELLING, FINISHED };

extern double resultCurrent, resultPrevious;
extern double orientxCorrected, orienty, orientz;
extern double landedOrientx, landedOrienty, landedOrientz;
extern double leveledOrientx, leveledOrienty, leveledOrientz;

class States {
public:
	//void whichState(flightState newState);
	void unarmed();
	void standby(double altitude, double initialAltitude, double velocity);
	void ascent(double altitude, double initialAltitude, double velocity);
	void descent(double altitude, double initialAltitude, double velocity, double accelx, double accely, double accelz, double distance);
	void leveling(double current, double previous);
    void finished();
	
	void actuateServo(bool locked);
	int hasChanged (double currentOrient, double initialOrient);
	void driveMotor (int motorNumber, int direction);

	void setCurrentState(uint8_t stateID);
	flightState currentState;

	// set pins 8, 9, 10 as the slave selects for SPI:
	const int CS1 = 10;
	const int CS2 = 9;
	const int CS3 = 8;
	bool CAM1_EXIST = false; 
	bool CAM2_EXIST = false;
	bool CAM3_EXIST = false;
private:
	// double oldAlt = 0;
	// double currentAlt = 0;
};


#endif // !_FLIGHTSTATES_H