#include <Arduino.h>
#include <vector>
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

#define SAMPLERATE_DELAY_MS 250

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
double smoothingFactor = 0.75;
// std::vector<double> smoothOrientation;
// std::vector<double> initialOrientation;
// struct gyroStruct
// {
// 	float x;
// 	float y;
// 	float z;
// };

// struct gyroStruct smoothOrientation;

double accelx, accely, accelz;
double orientx, orienty, orientz, orientxCorrected;
double targetw, targetx, targety, targetz;
double quatw, quatx, quaty, quatz;
imu::Vector<3> eulerAngle;
double angle;

double initialOrientation;
double radialOrient;
double tangentialOrient;

bool calibrated, initialized, calibrating;
int oriented1, oriented2, oriented3; //0 for untested, 1 for helpful, 2 for hurtful
double resultCurrent, resultPrevious, orientResult;
double tolerance = 0.025;
int activeMotor = 1;

int hasChanged (double currentOrient, double initialOrient);
void driveMotor (int motorNumber, int direction);
void resetCalibration();
void calibrateLeveler();
imu::Quaternion invert(imu::Quaternion quat);

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
	
	if (!calibrated && calibration >= 8) {
		calibrated = true;
		digitalWrite(LED_BUILTIN, HIGH);
		delay(5000);
		digitalWrite(LED_BUILTIN, LOW);
	}

	packetCount++;
	missionTime = millis()/1000.0;

	// Read Accelerometer and Magnetometer data from IMU
	sensors_event_t accelEvent;
	sensors_event_t orientEvent;
	bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
	bno.getEvent(&orientEvent);
	uint8_t sys, gyro, accel, mag = 0;
	bno.getCalibration(&sys, &gyro, &accel, &mag);
	calibration = sys + gyro + accel + mag;

	accelx = smoothingFactor * accelEvent.acceleration.x + (1 - smoothingFactor) * accelx;
	accely = smoothingFactor * accelEvent.acceleration.y + (1 - smoothingFactor) * accely;
	accelz = smoothingFactor * accelEvent.acceleration.z + (1 - smoothingFactor) * accelz;
	orientx = orientEvent.orientation.x; //smoothingFactor * orientEvent.orientation.x + (1 - smoothingFactor) * orientx;
	orienty = smoothingFactor * orientEvent.orientation.y + (1 - smoothingFactor) * orienty;
	orientz = smoothingFactor * orientEvent.orientation.z + (1 - smoothingFactor) * orientz;

	if (orientx >= 180) {
		orientxCorrected = orientx - 360;
	}
	else {
		orientxCorrected = orientx;
	}

	double cy = cos(orientxCorrected*PI/180 * 0.5);
    double sy = sin(orientxCorrected*PI/180 * 0.5);
    double cp = cos(-PI/2 * 0.5);
    double sp = sin(-PI/2 * 0.5);
    double cr = cos(0.0 * 0.5);
    double sr = sin(0.0 * 0.5);

    targetw = cr * cp * cy + sr * sp * sy;
    targetx = sr * cp * cy - cr * sp * sy;
    targety = -(cr * sp * cy + sr * cp * sy);
    targetz = cr * cp * sy - sr * sp * cy;
	imu::Quaternion targetQuat = imu::Quaternion(targetw, targetx, targety, targetz);
	// targetQuat.toAxisAngle(eulerAngle, angle);
	// Serial.print(targetw); Serial.print("\t");
	// Serial.print(targetx); Serial.print("\t");
	// Serial.print(targety); Serial.print("\t");
	// Serial.print(targetz); Serial.print("\t"); Serial.println(" ");
	targetQuat.normalize();
	imu::Quaternion quatEvent = bno.getQuat();
	quatw = smoothingFactor * quatEvent.w() + (1 - smoothingFactor) * quatw;
	quatx = smoothingFactor * quatEvent.x() + (1 - smoothingFactor) * quatx;
	quaty = smoothingFactor * quatEvent.y() + (1 - smoothingFactor) * quaty;
	quatz = smoothingFactor * quatEvent.z() + (1 - smoothingFactor) * quatz;
	// Serial.print(quatw); Serial.print("\t");
	// Serial.print(-quatx); Serial.print("\t");
	// Serial.print(-quaty); Serial.print("\t");
	// Serial.print(-quatz); Serial.print("\t"); Serial.println(" ");
	
	imu::Quaternion quat = imu::Quaternion(quatw, quatx, quaty, quatz);
	quat.normalize();

	// Serial.print(quat.w()); Serial.print("\t");
	// Serial.print(quat.x()); Serial.print("\t");
	// Serial.print(quat.y()); Serial.print("\t");
	// Serial.print(quat.z()); Serial.print("\t"); Serial.println(" ");
	imu::Quaternion quatDiff = targetQuat * quat;
	// Serial.println(quatDiff.w());
	double angleDiff = 180*2*acos(quatDiff.w())/PI;

	if (angleDiff > 270.0) {
		targetQuat = invert(targetQuat);
		// Serial.print(targetw); Serial.print("\t");
		// Serial.print(targetx); Serial.print("\t");
		// Serial.print(targety); Serial.print("\t");
		// Serial.print(targetz); Serial.print("\t"); Serial.println(" ");
		quatDiff = targetQuat * quat;
		angleDiff = 180*2*acos(quatDiff.w())/PI;
	}

	radialOrient = orienty - 90;
	if (fabs(orientz) <= 90) {
		tangentialOrient = fabs(orientz);
	}
	else if (orientz >= 0) {
		tangentialOrient = fabs(orientz - 180);
	}
	else {
		tangentialOrient = fabs(orientz + 180);
	}
	orientResult = sqrt(pow(radialOrient, 2) + pow(tangentialOrient, 2)); // Resultant vector
	resultCurrent = angleDiff;

	if (resultCurrent >= 2.5) { 
		// calibrateLeveler();
		// delay(25);

		// if (oriented1 != 0 && oriented2 != 0 && oriented3 != 0) {
		// 	if (oriented1 == 1) {
		// 		driveMotor(1, 1);
		// 	}
		// 	else if (oriented1 == 2) {
		// 		driveMotor(1, 2);
		// 	}
		// 	if (oriented2 == 1) {
		// 		driveMotor(2, 1);
		// 	}
		// 	else if (oriented2 == 2) {
		// 		driveMotor(2, 2);
		// 	}
		// 	if (oriented3 == 1) {
		// 		driveMotor(3, 1);
		// 	}
		// 	else if (oriented3 == 2) {
		// 		driveMotor(3, 2);
		// 	}
		// 	if (hasChanged(resultCurrent, resultPrevious) == 0) {
		// 		resetCalibration();
		// 	}
		// }

		//TAKE 2 - Run each motor one at a time until it's level
		if (calibrated) {

			if (activeMotor > 3) {
				activeMotor = 1;
			}

			if (hasChanged(resultCurrent, resultPrevious) != 2) {
				driveMotor(activeMotor, 1);
			}

			if (hasChanged(resultCurrent, resultPrevious) == 2) {
				driveMotor(activeMotor, 2);
				delay(100);
				driveMotor(activeMotor, 0);
				activeMotor++;
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
	packet += ",";
	packet += String(accely);
	packet += ",";
	packet += String(accelz);
	packet += ",";
	packet += String(orientxCorrected);
	packet += ",";
	packet += String(orienty);
	packet += ",";
	packet += String(orientz);
	packet += ",";
	packet += String(calibration);
	packet += ",";
	packet += String(orientResult);
	// packet += ",";
	// packet += String(quat.w());
	// packet += ",";
	// packet += String(quat.x());
	// packet += ",";
	// packet += String(quat.y());
	// packet += ",";
	// packet += String(quat.z());
	packet += ",";
	packet += String(resultCurrent);
	packet += ",";
	
	Serial.println(packet);

	String debugPacket = "";
	// debugPacket += String(calibrated);
	// debugPacket += ",";
	// debugPacket += String(initialized);
	// debugPacket += ",";
	// debugPacket += String(oriented1);
	// debugPacket += ",";
	// debugPacket += String(oriented2);
	// debugPacket += ",";
	// debugPacket += String(oriented3);
	// debugPacket += ",";
	debugPacket += String(orientEvent.orientation.x);
	debugPacket += ",";
	debugPacket += String(orientx);
	debugPacket += ",";
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
	delay(15);
}

void calibrateLeveler() {
	if (!calibrated && calibration >= 8) {
		calibrated = true;
		digitalWrite(LED_BUILTIN, HIGH);
		delay(5000);
		digitalWrite(LED_BUILTIN, LOW);
	}

	if (calibrated && !initialized) {
		initialOrientation = sqrt(pow((radialOrient), 2) + pow(tangentialOrient, 2)); // Resultant vector, not actually used
		initialized = true;
	}

	if (initialized && oriented1 == 0) {
		driveMotor(1, 1);
		int helping = hasChanged(resultCurrent, resultPrevious);
		if (helping != 0) {
			oriented1 = helping;
			driveMotor(1, 0);
			delay(25);
		}
	}

	else if (oriented1 != 0 && oriented2 == 0) {
		driveMotor(2, 1);
		int helping = hasChanged(resultCurrent, resultPrevious);
		if (helping != 0) {
			oriented2 = helping;
			driveMotor(2, 0);
			delay(25);
		}
	}

	else if (oriented2 != 0 && oriented3 == 0) {
		driveMotor(3, 1);
		int helping = hasChanged(resultCurrent, resultPrevious);
		if (helping != 0) {
			oriented3 = helping;
			driveMotor(3, 0);
			delay(25);
		}
	}
}

imu::Quaternion invert(imu::Quaternion quat) {
	double iquatw = -quat.w();
	double iquatx = -quat.x();
	double iquaty = -quat.y();
	double iquatz = -quat.z();

	return imu::Quaternion(iquatw, iquatx, iquaty, iquatz);
}