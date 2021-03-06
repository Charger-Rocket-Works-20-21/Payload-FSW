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

#define SAMPLERATE_DELAY_MS 100

#define MOTOR1 14
#define MOTOR2 15
#define MOTOR3 18
#define MOTOR1R 19
#define MOTOR2R 20
#define MOTOR3R 21

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

uint16_t packetCount = 0;
double missionTime;
bool ledOn;
uint8_t calibration;
uint16_t blinkRate;
double smoothingFactor = 0.25;
// std::vector<double> smoothOrientation;
// std::vector<double> initialOrientation;
// struct gyroStruct
// {
// 	float x;
// 	float y;
// 	float z;
// };

// struct gyroStruct smoothOrientation;

double accelx;
double accely;
double accelz;
double orientx;
double orienty;
double orientz;

//double initialOrientation;
//double radialOrient;
//double tangentialOrient;

bool calibrated, initialized, calibrating;
int oriented1, oriented2, oriented3; //0 for untested, 1 for helpful, 2 for hurtful
//double resultCurrent, resultPrevious;
double resultBuffer[3];
double resultAccel[10];
double posTolerance = 5.0;
double negTolerance = -5.0;

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
	
	// digitalWrite(MOTOR1, HIGH);
	// digitalWrite(MOTOR2, HIGH);
	// digitalWrite(MOTOR3, HIGH);
	// while(1);
	// driveMotor(1,2);
	// driveMotor(2,2);
	// driveMotor(3,2);
	// while(1);

	if (!bno.begin(bno.OPERATION_MODE_NDOF)) {
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

	sensors_event_t accelInitialEvent;
	bno.getEvent(&accelInitialEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
	accelx = smoothingFactor * accelInitialEvent.acceleration.x + (1 - smoothingFactor) * accelx;
	accely = smoothingFactor * accelInitialEvent.acceleration.y + (1 - smoothingFactor) * accely;
	accelz = smoothingFactor * accelInitialEvent.acceleration.z + (1 - smoothingFactor) * accelz;

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
	missionTime = millis()/1000.0;
	// imu::Quaternion quat = bno.getQuat();
	// Serial.println(quat.w());
	// Serial.println(quat.x());
	// Serial.println(quat.y());
	// Serial.println(quat.z());
	// String quatPacket = "";
	// quatPacket += String(quat.w);

	// Read Accelerometer and Magnetometer data from IMU
	for (int i = 0; i < 5; i++) {
		sensors_event_t accelEvent;
		sensors_event_t orientEvent;
		bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
		bno.getEvent(&orientEvent);
		uint8_t sys, gyro, accel, mag = 0;
		bno.getCalibration(&sys, &gyro, &accel, &mag);
		calibration = sys + gyro + accel + mag;
		resultAccel[i] = smoothingFactor * accelEvent.acceleration.x + (1 - smoothingFactor) * resultAccel[i];
		// accelx = smoothingFactor * accelEvent.acceleration.x + (1 - smoothingFactor) * accelx;
		// accely = smoothingFactor * accelEvent.acceleration.y + (1 - smoothingFactor) * accely;
		// accelz = smoothingFactor * accelEvent.acceleration.z + (1 - smoothingFactor) * accelz;
		// orientx = orientEvent.orientation.x;
		// orienty = orientEvent.orientation.y;
		// orientz = orientEvent.orientation.z;
	}
	accelx = (resultAccel[0] + resultAccel[1] + resultAccel[2] + resultAccel[3] + resultAccel[4])/5.0;
	//radialOrient = accely;
	//tangentialOrient = accelz;
	//resultCurrent = sqrt(pow((radialOrient), 2) + pow(tangentialOrient, 2)); // Resultant vector
	for (int i = 0; i < 2; i++) {
		resultBuffer[i] = resultBuffer[i+1];
	}
	resultBuffer[2] = 1000.0*sqrt(pow((accelx-9.81), 2.0)/* + pow(accely, 2) + pow(accelz, 2)*/); // Resultant vector
	if (resultBuffer[2] >= 150.0) { // while the resultant acceleration is greater than 1.0 m/s^2. This is over estimate, 5 degrees seems to be closer to 1.75 m/s^2
		calibrateLeveler();

		if (oriented1 != 0 && oriented2 != 0 && oriented3 != 0) {
			if (oriented1 == 0) {
				driveMotor(1,0);
			}
			else if (oriented1 == 1) {
				Serial.print("RAISING 1\t");
				driveMotor(1,1);
			}
			else if (oriented1 == 2) {
				Serial.print("LOWERING 1\t");
				driveMotor(1,2);
			}
			if (oriented2 == 0) {
				driveMotor(2,0);
			}
			else if (oriented2 == 1) {
				Serial.print("RAISING 2\t");
				driveMotor(2,1);
			}
			else if (oriented2 == 2) {
				Serial.print("LOWERING 2\t");
				driveMotor(2,2);
			}
			if (oriented3 == 0) {
				driveMotor(3,0);
			}
			else if (oriented3 == 1) {
				Serial.print("RAISING 3\t");
				driveMotor(3,1);
			}
			else if (oriented3 == 2) {
				Serial.print("LOWERING 3\t");
				driveMotor(3,2);
			}
			if (hasChanged(resultBuffer[2], (resultBuffer[0]+resultBuffer[1])/2) == 0) {
				Serial.println("HERE");
				resetCalibration();
				// calibrateLeveler();
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
	packet += String(missionTime);
	packet += ",";
	packet += String(accelx);
	// packet += ",";
	// packet += String(accelEvent.acceleration.y);
	// packet += ",";
	// packet += String(accelEvent.acceleration.z);
	// packet += ",";
	// packet += String(orientEvent.orientation.x);
	// packet += ",";
	// packet += String(orientEvent.orientation.y);
	// packet += ",";
	// packet += String(orientEvent.orientation.z);
	packet += ",";
	packet += String(calibration);
	packet += ",";
	packet += String(resultBuffer[2]);
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
	debugPacket += String((resultBuffer[0]+resultBuffer[1])/2);
	debugPacket += ",";
	debugPacket += String("");
	Serial.println(debugPacket);

	#ifdef USESD
	File dataFile = SD.open("datalog.txt", FILE_WRITE);
	if (dataFile) {
		dataFile.println(packet);
		dataFile.close();
	}
	else {
		//Serial.println("Could not open datalog.txt");
	}
	#endif

	delay(SAMPLERATE_DELAY_MS);
}

// Return 0 for no change
// Return 1 for good change
// Return 2 for bad change
int hasChanged (double currentOrient, double previousOrient) {
	if (previousOrient - currentOrient > posTolerance) {
		return 1;
	}
	else if (previousOrient - currentOrient < negTolerance) {
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
		digitalWrite(LED_BUILTIN, HIGH);
		delay(5000);
		digitalWrite(LED_BUILTIN, LOW);
	}

	if (calibrated && !initialized) {
		//initialOrientation = sqrt(pow((accelx), 2) + pow(accely, 2) + pow(accelz, 2)); // Resultant vector
		initialized = true;
	}

	if (initialized && oriented1 == 0) {
		driveMotor(1, 1);
		int helping = hasChanged(resultBuffer[2], (resultBuffer[0]+resultBuffer[1])/2);
		if (helping != 0) {
			oriented1 = helping;
			driveMotor(1, 0);
		}
		// if (helping != 2) {
		// 	driveMotor(1,1);
		// }
		// else {
		// 	driveMotor(1,2);
		// }
	}

	else if (oriented1 != 0 && oriented2 == 0) {
		Serial.println("DRIVING 2");
		driveMotor(2, 1);
		int helping = hasChanged(resultBuffer[2], (resultBuffer[0]+resultBuffer[1])/2);
		if (helping != 0) {
			oriented2 = helping;
			driveMotor(2, 0);
		}
		// if (helping != 2) {
		// 	driveMotor(2,1);
		// }
		// else {
		// 	driveMotor(2,2);
		// }
	}

	else if (oriented2 != 0 && oriented3 == 0) {
		driveMotor(3, 1);
		int helping = hasChanged(resultBuffer[2], (resultBuffer[0]+resultBuffer[1])/2);
		if (helping != 0) {
			oriented3 = helping;
			driveMotor(3, 0);
		}
		// if (helping != 2) {
		// 	driveMotor(3,1);
		// }
		// else {
		// 	driveMotor(3,2);
		// }
	}
}