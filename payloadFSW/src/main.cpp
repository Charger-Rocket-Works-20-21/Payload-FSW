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

uint16_t packetCount = 0;

double initialTemp;
double initialPres;
double initialAlt;

double smoothingFactor = 0.5;
double smoothTemperature;					// Degrees Celsius
double smoothPressure;						// Pascals
double smoothAltitude;						// Meters
double smoothVelocity;						// Meters/Second
std::vector<double> smoothAcceleration;		// Meters/Second/Second
std::vector<double> smoothOrientation;		// Degrees

double currentTime;
double pastTime;
double diffTime;

const char * file;

void fileNamer();

void setup() {
//--Initialize Board
	Serial.begin(115200);
	Serial.println("Payload FSW Software Initializing.");
	Wire.begin();
	SPI.begin();
	#ifdef USESD
	SD.begin(BUILTIN_SDCARD);

//--Initialize Start of SD Card write for given test/run
	fileNamer();
	#endif

	File dataFile = SD.open(file, FILE_WRITE);
	if (dataFile) {
		dataFile.println("weewoo weewoo");
		dataFile.close();
	}

//--Initialize Sensors
	if(!imu_init(&bno)) { // Initialize IMU
		dataFile = SD.open(file, FILE_WRITE);
		if (dataFile) {
			dataFile.println("BNO055 Not Detected");
			dataFile.close();
		}
	} 
	if(!alt_init(&bmp)) { // Initialize Altimeter
		dataFile = SD.open(file, FILE_WRITE);
		if (dataFile) {
			dataFile.println("BMP388 Not Detected");
			dataFile.close();
		}
	} 

	dataFile = SD.open(file, FILE_WRITE);
	if (dataFile) {
		dataFile.println("post init");
		dataFile.close();
	}

//--Initialize Ground Parameters
	smoothTemperature = 27.0;
	smoothPressure = 125300;
	smoothAltitude = 0;
	for (int i = 0; i < 5; i++) {
		smoothTemperature = getSmoothTemp(smoothingFactor, smoothTemperature);
		smoothPressure = getSmoothPres(smoothingFactor, smoothPressure);
		smoothAltitude = getSmoothPres(smoothingFactor, smoothAltitude);
	}
	initialTemp = smoothTemperature;
	initialPres = smoothPressure;
	initialAlt = smoothAltitude;

	#ifdef DROPTEST
		currentFS = TEST;
	#endif

	digitalWrite(13, HIGH);
}

void loop() {
	packetCount++;
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

	// Read Acceleration and Orientation from BNO055
	smoothAcceleration = resultantAccel(smoothingFactor, smoothAcceleration);
	smoothOrientation = resultantOrient(smoothingFactor, smoothOrientation);

	// Prepping Packet for SD Card Write
	String packet = "";
	packet += String(packetCount);
	packet += ",";
	packet += String(currentTime);
	packet += ",";
	packet += String(smoothTemperature);
	packet += ",";
	packet += String(smoothPressure);
	packet += ",";
	packet += String(smoothAltitude);
	packet += ",";
	packet += String(smoothVelocity);
	packet += ",";
	packet += String(smoothAcceleration.at(0));
	packet += ",";
	packet += String(smoothAcceleration.at(1));
	packet += ",";
	packet += String(smoothAcceleration.at(2));
	packet += ",";
	packet += String(smoothOrientation.at(0));
	packet += ",";
	packet += String(smoothOrientation.at(1));
	packet += ",";
	packet += String(smoothOrientation.at(2));
	packet += ",";
	packet += String(currentFS);

	#ifdef USESD
	// Writing Packet to SD Card
	File dataFile = SD.open(file, FILE_WRITE);
	if (dataFile) {
		dataFile.println(packet);
		dataFile.close();
	}
	#endif

	// Determining current Flight State, including logic to go to the next state
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
	case TEST:
		if (states.dropTest(packetCount, smoothAltitude)) {
			#ifdef USESD
			// Writing Drop Test Warning to SD Card
			File dataFile = SD.open(file, FILE_WRITE);
			if (dataFile) {
				dataFile.println("POSSIBLE DROP TEST END");
				dataFile.close();
			}
			#endif
		}
		break;
	}

	diffTime = currentTime - pastTime;
	pastTime = currentTime;
	delay(50);
}

void fileNamer() {
	for (int i = 0; i < 1000; i++) {
		String fileName = "datalog";
		fileName += String(i);
		fileName += ".txt";
		file = fileName.c_str();
		if (!SD.exists(file)){
			File dataFile = SD.open(file, FILE_WRITE);
			if (dataFile) {
				dataFile.println("STARTING OUTPUT");
				dataFile.close();
			}
			return;
		}
	}
}