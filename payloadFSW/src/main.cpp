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

//#include "system.h"
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <SoftwareSerial.h>
#include <SD.h>

#define SAMPLERATE_DELAY_MS 50

// Initialize Classes
//States states;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_BMP3XX bmp;

// Initialize Variables
uint16_t packetCount = 0;

// double initialTemp;
// double initialPres;
// double initialAlt;

// double smoothingFactor = 0.75;
// double smoothingFactorDistance = 0.90;
// double smoothTemperature;					// Degrees Celsius
// double smoothPressure;						// Pascals
// double smoothAltitude;						// Meters
// double smoothVelocity;						// Meters/Second
// std::vector<double> smoothAcceleration;		// Meters/Second/Second
// std::vector<double> smoothOrientation;		// Degrees
uint8_t calibration;
// double smoothDistance;

double currentTime;
double pastTime;
double diffTime;

bool ledOn;
uint16_t blinkRate;

void setup() {
//--Initialize Board
	Serial.begin(115200);
	delay(1000);
	Serial.println("Payload FSW Software Initializing.");

	pinMode(LED_BUILTIN, OUTPUT);
	// pinMode(MOTOR1, OUTPUT);
	// pinMode(MOTOR2, OUTPUT);
	// pinMode(MOTOR3, OUTPUT);
	// pinMode(MOTOR1R, OUTPUT);
	// pinMode(MOTOR2R, OUTPUT);
	// pinMode(MOTOR3R, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
	delay(500);
	ledOn = true;

	SD.begin(BUILTIN_SDCARD);

//--Initialize Sensors
	// if(!imuInit(&bno)) { // Initialize IMU
	// 	dataFile = SD.open("datalog.txt", FILE_WRITE);
	// 	if (dataFile) {
	// 		dataFile.println("BNO055 Failed to Initialize");
	// 		dataFile.close();
	// 	}
	// } 
	// if(!altInit(&bmp)) { // Initialize Altimeter
	// 	dataFile = SD.open("datalog.txt", FILE_WRITE);
	// 	if (dataFile) {
	// 		dataFile.println("BMP388 Failed to Initialize");
	// 		dataFile.close();
	// 	}
	// }
	if (!bmp.begin_I2C(0x77, &Wire1)) {   // hardware I2C mode, can pass in address & alt Wire
		Serial.println("Could not find a valid BMP3 sensor, check wiring!");
  	}
	else {
		Serial.println("BMP388 Detected");
	}
	if (!bno.begin()) {
		Serial.println("BNO055 Not Detected...");
	}
	else {
		Serial.println("BNO055 Detected");
	}
	
	delay(1000);
	bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
	bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
	bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
	bmp.setOutputDataRate(BMP3_ODR_50_HZ);
	bno.setExtCrystalUse(true);
	// if (distanceSensor.begin() != 0) { // Initialize Rangefinder
	// 	dataFile = SD.open("datalog.txt", FILE_WRITE);
	// 	if (dataFile) {
	// 		dataFile.println("Rangefinder Failed to Initialize");
	// 		dataFile.close();
	// 	}
	// }
	
//--Initialize Start of SD Card write for given test/run
	File dataFile = SD.open("datalog.txt", FILE_WRITE);
	if (dataFile) {
		dataFile.println("DROPTEST FSW INITIALIZED");
		dataFile.println(" , , , , , Acceleration, , , Orientation, ");
		dataFile.println("Packet, Time, Temp, Pressure, Altitude, x, y, z, x, y, z");
		dataFile.close();
	}
	else {
		Serial.println("Could not open datalog.txt");
	}

//--Initialize Ground Parameters
	// Serial.println("Declaring Initial Parameters");
	// smoothTemperature = 27.0;
	// smoothPressure = 100000;
	// smoothAltitude = 100;
	// // for (int i = 0; i < 10; i++) {
	// // 	smoothTemperature = getSmoothBmpTemp(&bmp, smoothingFactor, smoothTemperature);
	// // 	smoothPressure = getSmoothPres(&bmp, smoothingFactor, smoothPressure);
	// // 	smoothAltitude = getSmoothAlt(&bmp, smoothingFactor, smoothAltitude);
	// // }
	// initialTemp = smoothTemperature;
	// initialPres = smoothPressure;
	// initialAlt = smoothAltitude;

	// String initParams = "";
	// initParams += String(initialTemp);
	// initParams += ",";
	// initParams += String(initialPres);
	// initParams += ",";
	// initParams += String(initialAlt);
	// initParams += ",";
	// Serial.println(initParams);

	for (int i = 0; i < 10; i++) {
		digitalWrite(LED_BUILTIN, HIGH);
		delay(100);
		digitalWrite(LED_BUILTIN, LOW);
		delay(100);
	}
}

void loop() {
	if (calibration >= 8) {
		blinkRate = 5;
	}
	else {
		blinkRate = 30;
	}
	if (packetCount % blinkRate == 0) {
		if (ledOn) {
			ledOn = false;
			digitalWrite(LED_BUILTIN, LOW);
		}
		else {
			ledOn = true;
			digitalWrite(LED_BUILTIN, HIGH);
		}
	}
	packetCount++;
	currentTime = millis()/1000;

//--Sensor Readings    
	// Read Temperature, Pressure, and Altitude from Barometer
	if (!bmp.performReading()) {
		Serial.println("Failed To Perform Reading");
	}

	// Read Accelerometer and Magnetometer data from IMU
	sensors_event_t accelEvent;
	sensors_event_t orientEvent;
	bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
	bno.getEvent(&orientEvent);
	uint8_t sys, gyro, accel, mag = 0;
	bno.getCalibration(&sys, &gyro, &accel, &mag);
	calibration = sys + gyro + accel + mag;
	// Serial.print("Reading Temperature... ");
	// Serial.println(bmp.temperature);
	// smoothTemperature = getSmoothBmpTemp(&bmp, smoothingFactor, smoothTemperature);
	// Serial.print("Reading Pressure... ");
	// smoothPressure = getSmoothPres(&bmp, smoothingFactor, smoothPressure);
	// Serial.print("Reading Altitude...\n");
	// smoothAltitude = getSmoothAlt(&bmp, smoothingFactor, smoothAltitude);

	// Serial.print("Calculating Velocity...\n");
	// smoothVelocity = getSmoothVel(smoothingFactor, smoothVelocity, smoothAltitude, pastTime, currentTime);

	// Read Acceleration and Orientation from BNO055
	// Serial.print("Reading Acceleration... ");
	// smoothAcceleration = getSmoothAccel(smoothingFactor, smoothAcceleration);
	// Serial.print("Reading Orientation... ");
	// smoothOrientation = getSmoothOrient(smoothingFactor, smoothOrientation);
	// calibration = getCalibration();

	// Read Distance from Rangefinder
	// smoothDistance = getSmoothDistance(smoothingFactorDistance, smoothDistance);

	// Serial.println("Finished Polling Sensors.");

	// Prepping Packet for SD Card Write
	String packet = "";
	packet += String(packetCount);
	packet += ",";
	packet += String(currentTime);
	packet += ",";
	packet += String(bmp.temperature);
	packet += ",";
	packet += String(bmp.pressure);
	packet += ",";
	packet += String(bmp.readAltitude(1013.25));
	packet += ",";
	packet += String(accelEvent.acceleration.x);
	packet += ",";
	packet += String(accelEvent.acceleration.y);
	packet += ",";
	packet += String(accelEvent.acceleration.z);
	packet += ",";
	packet += String(orientEvent.orientation.x);
	packet += ",";
	packet += String(orientEvent.orientation.y);
	packet += ",";
	packet += String(orientEvent.orientation.z);
	packet += ",";
	packet += String(calibration);
	// packet += ",";
	// packet += String(smoothVelocity);
	// packet += ",";
	// packet += String(smoothAcceleration.at(0));
	// packet += ",";
	// packet += String(smoothAcceleration.at(1));
	// packet += ",";
	// packet += String(smoothAcceleration.at(2));
	// packet += ",";
	// packet += String(smoothOrientation.at(0));
	// packet += ",";
	// packet += String(smoothOrientation.at(1));
	// packet += ",";
	// packet += String(smoothOrientation.at(2));
	// packet += ",";
	// packet += String(states.currentState);

	Serial.println(packet);

	// Writing Packet to SD Card
	File dataFile = SD.open("datalog.txt", FILE_WRITE);
	if (dataFile) {
		dataFile.println(packet);
		dataFile.close();
	}
	else {
		Serial.println("Could not open datalog.txt");
	}

	// // Determining current Flight State, including logic to go to the next state
	// Serial.println(states.currentState);
	// switch (states.currentState) {
	// case UNARMED:
	// 	states.unarmed();
	// 	break;
	// case STANDBY:
	// 	states.standby(smoothAltitude, initialAlt, smoothVelocity);
	// 	break;
	// case ASCENT:
	// 	states.ascent(smoothAltitude, initialAlt, smoothVelocity);
	// 	break;
	// case DESCENT:
	// 	states.descent(smoothAltitude, smoothVelocity, smoothAcceleration, smoothDistance);
	// 	break;
	// case LEVELLING:
	// 	states.levelling(smoothOrientation.at(0), smoothOrientation.at(1)); // Uses sensor X and Z vectors
	// 	break;
	// case FINISHED:
	// 	states.finished();
	// 	break;
	// }

	// diffTime = currentTime - pastTime;
	// pastTime = currentTime;
	delay(SAMPLERATE_DELAY_MS);
}