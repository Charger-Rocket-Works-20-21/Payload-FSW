/*
 * Main sketch for Charger Rocket Works Payload Flight Software
 * NASA SLI 2020-21
 * 
 * Uses the following libraries:
 * Adafruit Unified Sensor
 * Adafruit BNO055
 * Adafruit BusIO
 * Adafruit BMP3XX
 */

#include "system.h"
#include "alt.h"
#include "imu.h"
#include "FlightStates.h"

// Initialize Classes
States states;

// Initialize Variables
extern flightState currentFS;

double initialTemp;
double initialPres;
double initialAlt;

double smoothingFactor = 0.5;
double smoothTemperature;		// Degrees Celsius
double smoothPressure;			// Pascals
double smoothAltitude;			// Meters
double smoothVelocity;			// Meters/Second
double smoothAcceleration;		// Meters/Second/Second

double currentTime;
double pastTime;

void setup() {
//--Initialize Board
	Serial.begin(115200);
	Serial.println("Payload FSW Software Initializing.");
	Wire.begin();
	SPI.begin();

//--Initialize Sensors
	imu_init(&bno); // Initialize IMU
	alt_init(&bmp); // Initialize Altimeter

//--Initialize Ground Parameters
	// Get Accurate Pressure
	// Get Accurate Temperature
	// Get Accurate Altitude
	
	// Dummy Definitions
	initialTemp = 27.0;
	initialPres = 125300;
	initialAlt = 0;
}

void loop() {
//--Sensor Readings
	Serial.println("Reading Sensors.");
	//Tests:
	bno055_test();
	bmp3XX_test();
    
	// Read Temperature, Pressure, and Altitude from BMP388
	smoothTemperature = getSmoothTemp(smoothingFactor, smoothTemperature);
	smoothPressure = getSmoothPres(smoothingFactor, smoothPressure);
	smoothAltitude = getSmoothPres(smoothingFactor, smoothAltitude);

	currentTime = millis();
	smoothVelocity = getSmoothVel(smoothingFactor, smoothVelocity, smoothAltitude, pastTime, currentTime);

	// Read Net Acceleration from BNO055
	smoothAcceleration = resultantAccel(smoothingFactor, smoothAcceleration);

	Serial.println(currentFS);
	switch (currentFS) {
	case UNARMED:
		states.unarmed();
		break;
	case STANDBY:
		states.standby(smoothAltitude, initialAlt, smoothVelocity);
		break;
	case ASCENT:
		states.ascent(smoothAltitude, initialAlt, smoothVelocity);
		break;
	case DESCENT:
		states.descent(smoothVelocity, smoothAcceleration);
		break;
	case LEVELLING:
		states.levelling();
		break;
	case FINISHED:
		states.finished();
		break;
	}

	pastTime = currentTime;
	delay(100);
}
