#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <SoftwareSerial.h>
#include <utility/imumaths.h>
//#include <SD.h>
#include <Softwire.h>
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>

#include "FlightStates.h"

#define SAMPLERATE_DELAY_MS 50

#define SDA_PIN 16
#define SCL_PIN 17
#define BNO_ADDRESS 0x28
#define BMP_ADDRESS 0x76

#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

#define SEALEVELPRESSURE_HPA 1013.25

#define SCB_AIRCR (*(volatile uint32_t *)0xE000ED0C) // Application Interrupt and Reset Control location

States states;
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO_ADDRESS);
Adafruit_BMP3XX bmp;
// SFEVL53L1X distanceSensor(Wire2, SHUTDOWN_PIN, INTERRUPT_PIN);
AsyncDelay readInterval;
SoftwareSerial XBee(2,3);

uint16_t packetCount = 0;
double currentTime;
bool ledOn;
uint16_t blinkRate;

double initialAlt;
double temperature;
double pressure;
double altitude;
double velocity;
double accelx;
double accely;
double accelz;
double orientx;
double orienty;
double orientz;
uint8_t calibration;
double distance;

//void toBlinkOrNotToBlink(uint16_t packetNumber, bool lightsOn);
void readCommand();

void setup() {
  	// put your setup code here, to run once:
	Serial.begin(9600);
	XBee.begin(115200);
	delay(1000);
	Serial.println("Beginning Payload Autogyro Drop Test...");
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(RELEASE_POWER, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
	digitalWrite(RELEASE_POWER, HIGH);
	delay(500);
	ledOn = true;

	// SD.begin(BUILTIN_SDCARD);

	// if (!bmp.begin_I2C(BMP_ADDRESS, &Wire1)) {   // hardware I2C mode, can pass in address & alt Wire
	// 	Serial.println("BMP388 Not Detected");
  	// }
	// else {
	// 	Serial.println("BMP388 Detected");
	// }
	if (!bno.begin()) {
		Serial.println("BNO055 Not Detected");
	}
	else {
		Serial.println("BNO055 Detected");
	}
	// if (distanceSensor.begin() != 0) {
	// 	Serial.println("Rangefinder Not Detected");
	// }
	// else {
	// 	Serial.println("Rangefinder Detected");
	// }
	
	delay(1000);
	bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
	bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
	bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
	bmp.setOutputDataRate(BMP3_ODR_50_HZ);
	bno.setExtCrystalUse(true);

	// File dataFile = SD.open("datalog.txt", FILE_WRITE);
	// if (dataFile) {
	// 	dataFile.println("DROPTEST FSW INITIALIZED");
	// 	dataFile.println(" , , , , , Acceleration, , , Orientation, ");
	// 	dataFile.println("Packet, Time, Temp, Pressure, Altitude, x, y, z, x, y, z");
	// 	dataFile.close();
	// }
	// else {
	// 	Serial.println("Could not open datalog.txt");
	// }

	initialAlt = bmp.readAltitude(SEALEVELPRESSURE_HPA);

	for (int i = 0; i < 10; i++) {
		digitalWrite(LED_BUILTIN, HIGH);
		delay(100);
		digitalWrite(LED_BUILTIN, LOW);
		delay(100);
	}
}

void loop() {
	if (calibration >= 10) {
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
	
	readCommand();
	packetCount++;
	currentTime = millis()/1000.0;

	// Read Temperature, Pressure, and Altitude from Barometer
	if (!bmp.performReading()) {
		Serial.println("Failed To Perform Reading");
	}
	temperature = bmp.temperature;
	pressure = bmp.pressure;
	altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

	// Read Accelerometer and Magnetometer data from IMU
	sensors_event_t accelEvent;
	sensors_event_t orientEvent;
	bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
	bno.getEvent(&orientEvent);
	accelx = accelEvent.acceleration.x;
	accely = accelEvent.acceleration.y;
	accelz = accelEvent.acceleration.z;
	orientx = orientEvent.orientation.x;
	orienty = orientEvent.orientation.y;
	orientz = orientEvent.orientation.z;
	uint8_t sys, gyro, accel, mag = 0;
	bno.getCalibration(&sys, &gyro, &accel, &mag);
	calibration = sys + gyro + accel + mag;

	String packet = "";
	packet += String(packetCount);
	packet += ",";
	packet += String(currentTime);
	packet += ",";
	packet += String(temperature);
	packet += ",";
	packet += String(pressure);
	packet += ",";
	packet += String(altitude);
	packet += ",";
	packet += String(accelx);
	packet += ",";
	packet += String(accely);
	packet += ",";
	packet += String(accelz);
	packet += ",";
	packet += String(orientx);
	packet += ",";
	packet += String(orienty);
	packet += ",";
	packet += String(orientz);
	packet += ",";
	packet += String(calibration);
	Serial.println(packet);

	// File dataFile = SD.open("datalog.txt", FILE_WRITE);
	// if (dataFile) {
	// 	dataFile.println(packet);
	// 	dataFile.close();
	// }
	// else {
	// 	Serial.println("Could not open datalog.txt");
	// }

	XBee.println(packet);

  	// Determining current Flight State, including logic to go to the next state
	Serial.println(states.currentState);
	switch (states.currentState) {
	case UNARMED:
		states.unarmed();
		break;
	case STANDBY:
		states.standby(altitude, initialAlt, velocity);
		break;
	case ASCENT:
		states.ascent(altitude, initialAlt, velocity);
		break;
	case DESCENT:
		// distanceSensor.startRanging();
		// while (!distanceSensor.checkForDataReady()) {delay(1);}
		// distance = distanceSensor.getDistance();
		// distanceSensor.clearInterrupt();
		// distanceSensor.stopRanging();
		// distance = distance * 0.0032808417;
		states.descent(altitude, velocity, accelx, accely, accelz, distance);
		break;
	case LEVELLING:
		states.levelling(orientx, orienty); // Uses sensor X and Z vectors
		break;
	case FINISHED:
		states.finished();
		break;
	}

	delay(SAMPLERATE_DELAY_MS);
}

void readCommand() {
	if (XBee.available()) {
		String command = XBee.readString();
		if (command.equalsIgnoreCase("RST")) {
			// Reset Teensy
			Serial.end();
			SCB_AIRCR = 0x5FA0004; // Write Value for Restart
		}
		else if (command.equalsIgnoreCase("LVL")) {
			// Restart Levelling process
		}
		else if (command.equalsIgnoreCase("PIC")) {
			// Retake Picture
		}
		else if (command.equalsIgnoreCase("RSD")) {
			// Resend Picture
		}
		else if (command.equalsIgnoreCase("CAL")) {
			// Calibrate Initial Altitude
			initialAlt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
		}
		else if (command.equalsIgnoreCase("REL")) {
			// Release Detach Mechanism
			states.actuateServo(false);
		}
		else if (command.equalsIgnoreCase("LCK")) {
			// Lock Detach Mechanism
			states.actuateServo(true);
		}
	}
}


