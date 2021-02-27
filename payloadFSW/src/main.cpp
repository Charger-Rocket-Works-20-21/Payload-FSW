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

// Initialize Classes
States states;

// Initialize Variables
extern Adafruit_BMP3XX bmp;
extern flightState currentFS;
extern Adafruit_BNO055 bno;
extern SFEVL53L1X distanceSensor(Wire2, SHUTDOWN_PIN, INTERRUPT_PIN);

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

bool ledOn;
int ledPin = 13;

void fileNamer();
void toBlinkOrNotToBlink(uint16_t packetNumber, bool lightsOn);

void setup() {
//--Initialize Board
	pinMode(ledPin, OUTPUT);
	digitalWrite(ledPin, HIGH);
	delay(2000);
	ledOn = true;
	Serial.begin(115200);
	Serial.println("Payload FSW Software Initializing.");

	SD.begin(BUILTIN_SDCARD);

//--Initialize Start of SD Card write for given test/run
	File dataFile = SD.open("datalog.txt", FILE_WRITE);
	if (dataFile) {
		dataFile.println("DROPTEST FSW INITIALIZED");
		dataFile.println(" , , Acceleration, , , Orientation, ");
		dataFile.println("Packet, Time, x, y, z, x, y, z");
		dataFile.close();
	}
	else {
		Serial.println("Could not open datalog.txt");
	}


//--Initialize Sensors
	if(!imuInit(&bno)) { // Initialize IMU
		dataFile = SD.open(file, FILE_WRITE);
		if (dataFile) {
			dataFile.println("BNO055 Not Detected");
			dataFile.close();
		}
	} 
	if(!altInit(&bmp)) { // Initialize Altimeter
		dataFile = SD.open(file, FILE_WRITE);
		if (dataFile) {
			dataFile.println("BMP388 Not Detected");
			dataFile.close();
		}
	}
	
//--Initialize Ground Parameters
	Serial.println("Declaring Initial Parameters");
	smoothTemperature = 27.0;
	smoothPressure = 1000000;
	smoothAltitude = 0;
	for (int i = 0; i < 5; i++) {
		smoothTemperature = getSmoothTemp(smoothingFactor, smoothTemperature);
		smoothPressure = getSmoothPres(smoothingFactor, smoothPressure);
		smoothAltitude = getSmoothPres(smoothingFactor, smoothAltitude);
	}
	initialTemp = smoothTemperature;
	initialPres = smoothPressure;
	initialAlt = smoothAltitude;

	String initParams = "";
	initParams += String(initialTemp);
	initParams += ",";
	initParams += String(initialPres);
	initParams += ",";
	initParams += String(initialAlt);
	initParams += ",";
	Serial.println(initParams);

//--Initialize PID
	pidInit(10.0, 0.1, 3.0, 0.0);

	Serial.println("End of Setup");
}

void loop() {
	toBlinkOrNotToBlink(packetCount, ledOn);

	packetCount++;
	currentTime = millis();

//--Sensor Readings
	Serial.println("Reading Sensors.");
    
	// Read Temperature, Pressure, and Altitude from BMP388
	Serial.print("Reading Temperature... ");
	smoothTemperature = getSmoothTemp(smoothingFactor, smoothTemperature);
	Serial.print("Reading Pressure... ");
	smoothPressure = getSmoothPres(smoothingFactor, smoothPressure);
	Serial.print("Reading Altitude...\n");
	smoothAltitude = getSmoothPres(smoothingFactor, smoothAltitude);

	Serial.print("Calculating Velocity...\n");
	smoothVelocity = getSmoothVel(smoothingFactor, smoothVelocity, smoothAltitude, pastTime, currentTime);

	// Read Acceleration and Orientation from BNO055
	Serial.print("Reading Acceleration... ");
	smoothAcceleration = getSmoothAccel(smoothingFactor, smoothAcceleration);
	Serial.print("Reading Orientation... ");
	smoothOrientation = getSmoothOrient(smoothingFactor, smoothOrientation);

	Serial.println("Finished Polling Sensors.");

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

	Serial.println(packet);

	// Writing Packet to SD Card
	File dataFile = SD.open(file, FILE_WRITE);
	if (dataFile) {
		dataFile.println(packet);
		dataFile.close();
	}

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
		states.levelling(smoothOrientation.at(0), smoothOrientation.at(2), currentTime); // Uses sensor X and Z vectors
		break;
	case FINISHED:
		states.finished();
		break;
	}

	diffTime = currentTime - pastTime;
	pastTime = currentTime;
	delay(50);
}

void toBlinkOrNotToBlink(uint16_t packetNumber, bool lightsOn) {
	if (packetNumber % 100 == 0) {
		if (lightsOn) {
			lightsOn = false;
			digitalWrite(13, LOW);
		}
		else {
			lightsOn = true;
			digitalWrite(13, HIGH);
		}
	}
}