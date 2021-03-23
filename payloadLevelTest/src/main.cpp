#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SoftwareSerial.h>
#include <utility/imumaths.h>

#define USESD

#ifdef USESD
#include <SD.h>
#endif

#define SAMPLERATE_DELAY_MS 1000

#define MOTOR1 14
#define MOTOR2 15
#define MOTOR3 18
#define MOTOR1R 19
#define MOTOR2R 20
#define MOTOR3R 21

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire1);

uint16_t packetCount = 0;
double currentTime;
bool ledOn;
uint8_t calibration;
uint16_t blinkRate;
double smoothingFactor = 0.9;
// std::vector<double> smoothOrientation;
// std::vector<double> initialOrientation;
struct gyroStruct
{
	float x;
	float y;
	float z;
};

struct gyroStruct smoothOrientation;

double initialOrientation;
double radialOrient;
double tangentialOrient;

bool calibrated, initialized, calibrating;
int oriented1, oriented2, oriented3; //0 for untested, 1 for helpful, 2 for hurtful
double resultCurrent, resultPrevious;
double tolerance = 1.0;

int hasChanged (double currentOrient, double initialOrient);
void driveMotor (int motorNumber, int direction);
void resetCalibration();
void calibrateLeveler();

void setup() {
	Serial.begin(9600);
	delay(1000);
	Serial.println("Beginning Payload Autogyro Drop Test...");
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
	delay(500);
	ledOn = true;

	pinMode(MOTOR1, OUTPUT);
	pinMode(MOTOR2, OUTPUT);
	pinMode(MOTOR3, OUTPUT);

	if (!bno.begin()) {
		Serial.println("BNO055 Not Detected...");
		while(1);
	}
	else {
		Serial.println("BNO055 Detected");
	}
	
	delay(1000);
	bno.setExtCrystalUse(true);

	#ifdef USESD
	SD.begin(BUILTIN_SDCARD);

	File dataFile = SD.open("datalog.txt", FILE_WRITE);
	if (dataFile) {
		dataFile.println("LEVELLING TEST FSW INITIALIZED");
		dataFile.println(" , , Acceleration, , , Orientation, ");
		dataFile.println("Packet, Time, x, y, z, x, y, z");
		dataFile.close();
	}
	else {
		Serial.println("Could not open datalog.txt");
	}
	#endif

	for (int i = 0; i < 10; i++) {
		digitalWrite(LED_BUILTIN, HIGH);
		delay(100);
		digitalWrite(LED_BUILTIN, LOW);
		delay(100);
	}
}

void loop() {
	if (calibration >= 8) {
		blinkRate = 2;
	}
	else {
		blinkRate = 20;
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
	currentTime = millis()/1000.0;

	// Read Accelerometer and Magnetometer data from IMU
	sensors_event_t accelEvent;
	sensors_event_t orientEvent;
	bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
	bno.getEvent(&orientEvent);
	uint8_t sys, gyro, accel, mag = 0;
	bno.getCalibration(&sys, &gyro, &accel, &mag);
	calibration = sys + gyro + accel + mag;

	smoothOrientation.x = smoothingFactor * orientEvent.orientation.x + (1 - smoothingFactor) * smoothOrientation.x;
	smoothOrientation.y = smoothingFactor * orientEvent.orientation.y + (1 - smoothingFactor) * smoothOrientation.y;
	smoothOrientation.z = smoothingFactor * orientEvent.orientation.z + (1 - smoothingFactor) * smoothOrientation.z;

	radialOrient = smoothOrientation.y;
	tangentialOrient = smoothOrientation.z;
	resultCurrent = sqrt(pow((radialOrient), 2) + pow(tangentialOrient+90, 2)); // Resultant vector
	
	if (resultCurrent >= 5.0) {
		calibrateLeveler();

		if (oriented1 != 0 && oriented2 != 0 && oriented3 != 0) {
			if (oriented1 == 1) {
				driveMotor(1, 1);
			}
			if (oriented2 == 1) {
				driveMotor(2, 1);
			}
			if (oriented3 == 1) {
				driveMotor(3, 1);
			}
			if (hasChanged(resultCurrent, resultPrevious) != 1) {
				resetCalibration();
			}
		}
	}
	else {
		driveMotor(1, 0);
		driveMotor(2, 0);
		driveMotor(3, 0);
	}

	String packet = "";
	packet += String(packetCount);
	packet += ",";
	packet += String(currentTime);
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
	packet += ",";
	packet += String(resultCurrent);
	Serial.println(packet);

	String debugPacket = "";
	debugPacket += String(calibrated);
	debugPacket += ",";
	debugPacket += String(initialized);
	debugPacket += ",";
	debugPacket += String(oriented1);
	debugPacket += ",";
	debugPacket += String(oriented2);
	debugPacket += ",";
	debugPacket += String(oriented3);
	debugPacket += ",";
	debugPacket += String(resultPrevious);
	Serial.println(debugPacket);

	#ifdef USESD
	File dataFile = SD.open("datalog.txt", FILE_WRITE);
	if (dataFile) {
		dataFile.println(packet);
		dataFile.close();
	}
	else {
		Serial.println("Could not open datalog.txt");
	}
	#endif

	resultPrevious = resultCurrent;
	delay(SAMPLERATE_DELAY_MS);
}

// Return 0 for no change
// Return 1 for good change
// Return 2 for bad change
int hasChanged (double currentOrient, double previousOrient) {
	if (previousOrient - currentOrient > tolerance) {
		return 1;
	}
	else if (previousOrient - currentOrient < -tolerance) {
		return 2;
	}
	else {
		return 0;
	}
}

void driveMotor (int motorNumber, int direction) {
	int onPin, reversePin;
	if (motorNumber == 1) {
		onPin = MOTOR1;
		reversePin = MOTOR1R;
	}
	else if (motorNumber == 2) {
		onPin = MOTOR2;
		reversePin = MOTOR2R;
	}
	else if (motorNumber == 3) {
		onPin = MOTOR3;
		reversePin = MOTOR3R;
	}
	if (direction == 0) {
		digitalWrite(onPin, LOW);
	}
	else if (direction == 1) {
		digitalWrite(reversePin, LOW);
		digitalWrite(onPin, HIGH);
	}
	else if (direction == 2) {
		digitalWrite(reversePin, HIGH);
		digitalWrite(onPin, HIGH);
	}
}

void resetCalibration() {
	oriented1 = 0;
	oriented2 = 0;
	oriented3 = 0;

	driveMotor(1, 0);
	driveMotor(2, 0);
	driveMotor(3, 0);
}

void calibrateLeveler() {
	if (!calibrated && calibration >= 8) {
		calibrated = true;
	}

	if (calibrated && !initialized) {
		initialOrientation = sqrt(pow((radialOrient), 2) + pow(tangentialOrient+90, 2)); // Resultant vector
		initialized = true;
	}

	if (initialized && oriented1 == 0) {
		driveMotor(1, 1);
		bool helping = hasChanged(resultCurrent, resultPrevious);
		if (helping != 0) {
			oriented1 = helping;
			driveMotor(1, 0);
		}
	}

	else if (oriented1 != 0 && oriented2 == 0) {
		driveMotor(2, 1);
		bool helping = hasChanged(resultCurrent, resultPrevious);
		if (helping != 0) {
			oriented2 = helping;
			driveMotor(2, 0);
		}
	}

	else if (oriented2 != 0 && oriented3 == 0) {
		driveMotor(3, 1);
		bool helping = hasChanged(resultCurrent, resultPrevious);
		if (helping != 0) {
			oriented3 = helping;
			driveMotor(3, 0);
		}
	}
}