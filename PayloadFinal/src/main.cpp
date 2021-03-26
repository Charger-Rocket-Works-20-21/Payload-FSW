#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <SoftwareSerial.h>
#include <utility/imumaths.h>
#include <SD.h>
#include <Softwire.h>
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include <gp20u7.h>

#include "FlightStates.h"

#define POLLDELAY 50
#define TRANSMITDELAY 1000

#define SDA_PIN 16
#define SCL_PIN 17
#define BNO_ADDRESS 0x29
#define BNO_ADD_SEL 40
#define BMP_ADDRESS 0x76

#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

#define SEALEVELPRESSURE_HPA 1013.25

#define FRAMES_NUM 0x00

#define SCB_AIRCR (*(volatile uint32_t *)0xE000ED0C) // Application Interrupt and Reset Control location

States states;
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO_ADDRESS, &Wire1);
Adafruit_BMP3XX bmp;
SFEVL53L1X distanceSensor(Wire2, SHUTDOWN_PIN, INTERRUPT_PIN);
AsyncDelay readInterval;
SoftwareSerial XBee(28,29);
GP20U7 gps = GP20U7(Serial);

ArduCAM myCAM1(OV5642, states.CS1);
ArduCAM myCAM2(OV5642, states.CS2);
ArduCAM myCAM3(OV5642, states.CS3);

uint16_t packetCount = 0;
double missionTime, previousTime, diffTime, previousXbeeTime;
bool ledOn;
uint16_t blinkRate;
bool transmitAllowed = true;
bool sentPhotos = false;

double initialAlt;
double temperature;
double pressure;
double altitude, previousAltitude;
double velocity;
double accelx, accely, accelz;
double orientx, orienty, orientz;
double landedOrientx, landedOrienty, landedOrientz;
uint8_t calibration;
double distance;

Geolocation currentLocation;

char cam1String[8] = {'c', 'a', 'm', '1'};
char cam2String[8] = {'c', 'a', 'm', '2'};
char cam3String[8] = {'c', 'a', 'm', '3'};

void readCommand();
void initCameras();
void sendPhotos(char str[8]);

void setup() {
  	// put your setup code here, to run once:
	Serial.begin(115200);
	Serial7.begin(115200);
	XBee.begin(115200);
	delay(1000);
	Serial.println("Beginning Payload Flight Software...");
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(RELEASE_POWER, OUTPUT);
	pinMode(MOTOR1, OUTPUT);
	pinMode(MOTOR2, OUTPUT);
	pinMode(MOTOR3, OUTPUT);
	pinMode(MOTOR1R, OUTPUT);
	pinMode(MOTOR2R, OUTPUT);
	pinMode(MOTOR3R, OUTPUT);
	pinMode(BNO_ADD_SEL, OUTPUT);
	pinMode(states.CS1, OUTPUT);
	pinMode(states.CS2, OUTPUT);
	pinMode(states.CS3, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
	digitalWrite(RELEASE_POWER, HIGH);
	digitalWrite(BNO_ADD_SEL, HIGH);
	digitalWrite(states.CS1, HIGH);
	digitalWrite(states.CS2, HIGH);
	digitalWrite(states.CS3, HIGH);
	delay(500);
	ledOn = true;

	SPI.begin();

	SD.begin(BUILTIN_SDCARD);

	if (!bmp.begin_I2C(BMP_ADDRESS, &Wire1)) {   // hardware I2C mode, can pass in address & alt Wire
		Serial.println("BMP388 Not Detected");
  	}
	else {
		Serial.println("BMP388 Detected");
	}
	if (!bno.begin()) {
		Serial.println("BNO055 Not Detected");
	}
	else {
		Serial.println("BNO055 Detected");
	}
	if (distanceSensor.begin() != 0) {
		Serial.println("Rangefinder Not Detected");
	}
	else {
		Serial.println("Rangefinder Detected");
	}
	// gps.begin(); // Freezes Code if included, will need to fix
	
	delay(1000);
	bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
	bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
	bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
	bmp.setOutputDataRate(BMP3_ODR_50_HZ);
	bno.setExtCrystalUse(true);

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

	// // Record Initial Altitude and store it to EEPROM, if not already saved
	// if (EEPROM.read(0) != 0) {
	// 	initialAlt = EEPROM.read(0);
	// }
	// else {
	// 	initialAlt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
	// 	EEPROM.update(0, initialAlt);
	// }

	// states.setCurrentState(EEPROM.read(1));
	// packetCount = EEPROM.read(2);
	// landedOrientx = EEPROM.read(3);
	// landedOrienty = EEPROM.read(4);
	// landedOrientz = EEPROM.read(5);


	for (int i = 0; i < 10; i++) {
		digitalWrite(LED_BUILTIN, HIGH);
		delay(100);
		digitalWrite(LED_BUILTIN, LOW);
		delay(100);
	}

	delay(2000);
}

void loop() {
	packetCount++;
	previousTime = missionTime;
	missionTime = millis()/1000.0;
	
	if (calibration >= 8) {
		blinkRate = 20;
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

	// Read Temperature, Pressure, and Altitude from Barometer
	if (!bmp.performReading()) {
		Serial.println("Failed To Perform Reading");
	}
	temperature = bmp.temperature;
	pressure = bmp.pressure;
	previousAltitude = altitude;
	altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
	velocity = (altitude - previousAltitude)/(missionTime - previousTime);

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
	packet += "UAH Charger RocketWorks";
	packet += ",";
	packet += String(packetCount);
	packet += ",";
	packet += String(missionTime);
	packet += ",";
	packet += String(states.currentState);
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
	packet += "UAH Charger RocketWorks End";
	Serial.println(packet);

	File dataFile = SD.open("datalog.txt", FILE_WRITE);
	if (dataFile) {
		dataFile.println(packet);
		dataFile.close();
	}
	else {
		Serial.println("Could not open datalog.txt");
	}

	if (millis() - previousXbeeTime >= 990) {
		readCommand();
		if (transmitAllowed) {
			// XBee.println(packet);
			Serial7.println(packet);
			if (states.currentState != 0) {
				EEPROM.update(1, states.currentState);
				EEPROM.update(2, packetCount);
				EEPROM.update(3, landedOrientx);
				EEPROM.update(4, landedOrienty);
				EEPROM.update(5, landedOrientz);
			}
		}
		previousXbeeTime = millis();
	}
	
	
  	// Determining current Flight State, including logic to go to the next state
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
		distanceSensor.startRanging();
		while (!distanceSensor.checkForDataReady()) {delay(1);}
		distance = distanceSensor.getDistance();
		distanceSensor.clearInterrupt();
		distanceSensor.stopRanging();
		distance = distance * 0.0032808417;
		states.descent(altitude, initialAlt, velocity, accelx, accely, accelz, distance);
		break;
	case LEVELLING:
		states.levelling(orientx, orienty); // Uses sensor X and Z vectors
		break;
	case FINISHED:
		if (!sentPhotos) {
			transmitAllowed = false;
			sendPhotos(cam1String);
			sendPhotos(cam2String);
			sendPhotos(cam3String);
			transmitAllowed = true;
			sentPhotos = true;
		}
		states.finished();
		break;
	}

	diffTime = 1000*(millis()/1000.0 - missionTime);
	delay(POLLDELAY-diffTime);
}

void readCommand() {
	// if (XBee.available()) {
	// 	String command = XBee.readString();
	// 	if (command.equalsIgnoreCase("RST")) {
	// 		// Reset Teensy
	// 		Serial.end();
	// 		SCB_AIRCR = 0x5FA0004; // Write Value for Restart
	// 	}
	// 	else if (command.equalsIgnoreCase("LVL")) {
	// 		// Restart Levelling process
			
	// 	}
	// 	else if (command.equalsIgnoreCase("PIC")) {
	// 		// Retake Pictures
	// 		states.myCAMSaveToSDFile(myCAM1, cam1String);
	// 		states.myCAMSaveToSDFile(myCAM2, cam2String);
	// 		states.myCAMSaveToSDFile(myCAM3, cam3String);
	// 	}
	// 	else if (command.equalsIgnoreCase("RSD")) {
	// 		// Resend Pictures
	// 		sendPhotos(cam1String);
	// 		sendPhotos(cam2String);
	// 		sendPhotos(cam3String);
	// 	}
	// 	else if (command.equalsIgnoreCase("CAL")) {
	// 		// Calibrate Initial Altitude
	// 		initialAlt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
	// 	}
	// 	else if (command.equalsIgnoreCase("REL")) {
	// 		// Release Detach Mechanism
	// 		states.actuateServo(false);
	// 	}
	// 	else if (command.equalsIgnoreCase("LCK")) {
	// 		// Lock Detach Mechanism
	// 		states.actuateServo(true);
	// 	}
	// 	else if (command.equalsIgnoreCase("CEE")) {
	// 		// Clear the EEPROM
	// 		for (int i = 0; i < EEPROM.length(); i++) {
	// 			EEPROM.update(i, 0);
	// 		}
	// 	}
	// 	// else if (command.equalsIgnoreCase("BLK")) {
	// 	// 	// Blink Onboard LED
	// 	// 	for (int i = 0; i < 10; i++) {
	// 	// 		digitalWrite(LED_BUILTIN, HIGH);
	// 	// 		delay(100);
	// 	// 		digitalWrite(LED_BUILTIN, LOW);
	// 	// 		delay(100);
	// 	// 	}
	// 	// }
	// 	else if (command.equalsIgnoreCase("FS0")) {
	// 		states.currentState = UNARMED;
	// 	}
	// 	else if (command.equalsIgnoreCase("FS1")) {
	// 		states.currentState = STANDBY;
	// 	}
	// 	else if (command.equalsIgnoreCase("FS2")) {
	// 		states.currentState = ASCENT;
	// 	}
	// 	// else if (command.equalsIgnoreCase("FS3")) {
	// 	// 	states.currentState = DESCENT;
	// 	// }
	// 	else if (command.equalsIgnoreCase("FS4")) {
	// 		states.currentState = LEVELLING;
	// 	}
	// 	else if (command.equalsIgnoreCase("FS5")) {
	// 		states.currentState = FINISHED;
	// 	}
	// }
	if (Serial7.available()) {
		String command = Serial7.readString();
		if (command.equalsIgnoreCase("BLK")) {
			// Blink Onboard LED
			for (int i = 0; i < 10; i++) {
				digitalWrite(LED_BUILTIN, HIGH);
				delay(100);
				digitalWrite(LED_BUILTIN, LOW);
				delay(100);
			}
		}
		else if (command.equalsIgnoreCase("FS3")) {
			states.currentState = DESCENT;
			for (int k = 0; k < 10; k++) {
				Serial.println("SETTING FLIGHT STATE TO DESCENT");
				delay(50);
			}
		}
	}
}

void initCameras() {
	uint8_t vid, pid;
	uint8_t temp;

	//Reset the CPLD
	myCAM1.write_reg(0x07, 0x80);
	delay(100);
	myCAM1.write_reg(0x07, 0x00);
	delay(100); 
	myCAM2.write_reg(0x07, 0x80);
	delay(100);
	myCAM2.write_reg(0x07, 0x00);
	delay(100); 
	myCAM3.write_reg(0x07, 0x80);
	delay(100);
	myCAM3.write_reg(0x07, 0x00);
	delay(100);

	//Check if the 3 ArduCAM Mini 5MP PLus Cameras' SPI bus is OK
	while(1) {
		myCAM1.write_reg(ARDUCHIP_TEST1, 0x55);
		temp = myCAM1.read_reg(ARDUCHIP_TEST1);
		if(temp != 0x55)
		{
			Serial.println(F("SPI1 interface Error!"));
		}else{
			states.CAM1_EXIST = true;
			Serial.println(F("SPI1 interface OK."));
		}
		myCAM2.write_reg(ARDUCHIP_TEST1, 0x55);
		temp = myCAM2.read_reg(ARDUCHIP_TEST1);
		if (temp != 0x55)
		{
			Serial.println(F("SPI2 interface Error!"));
		} else {
			states.CAM2_EXIST = true;
			Serial.println(F("SPI2 interface OK."));
		}
		myCAM3.write_reg(ARDUCHIP_TEST1, 0x55);
		temp = myCAM3.read_reg(ARDUCHIP_TEST1);
		if(temp != 0x55) {
			Serial.println(F("SPI3 interface Error!"));
		} else {
			states.CAM3_EXIST = true;
			Serial.println(F("SPI3 interface OK."));
		}
		if (!(states.CAM1_EXIST||states.CAM2_EXIST||states.CAM3_EXIST)) {
			delay(1000);
			continue;
		} else {
			break;
		}
	}

	#if defined (OV5640_MINI_5MP_PLUS)
	while(1){
		//Check if the camera module type is OV5640
		myCAM1.rdSensorReg16_8(OV5640_CHIPID_HIGH, &vid);
		myCAM1.rdSensorReg16_8(OV5640_CHIPID_LOW, &pid);
		if ((vid != 0x56) || (pid != 0x40)){
		Serial.println(F("Can't find OV5640 module!"));
		delay(1000);continue;
		}else{
		Serial.println(F("OV5640 detected."));break;
		}   
	}
	#else
	while(1){
		//Check if the camera module type is OV5642
		myCAM1.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
		myCAM1.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
		if ((vid != 0x56) || (pid != 0x42)) {
			Serial.println(F("Can't find OV5642 module!"));
			delay(1000);
			continue;
		} else {
			Serial.println(F("OV5642 detected."));
			break;
		}  
	}
	#endif

	//Change to JPEG capture mode and initialize the OV5640 module
	myCAM1.set_format(JPEG);
	myCAM1.InitCAM();
	myCAM1.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);   //VSYNC is active HIGH
	myCAM2.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);   //VSYNC is active HIGH
	myCAM3.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);   //VSYNC is active HIGH
	myCAM1.clear_fifo_flag();
	myCAM1.write_reg(ARDUCHIP_FRAMES, FRAMES_NUM);
	myCAM2.write_reg(ARDUCHIP_FRAMES, FRAMES_NUM);
	myCAM3.write_reg(ARDUCHIP_FRAMES, FRAMES_NUM);
	#if defined (OV5640_MINI_5MP_PLUS)
	myCAM1.OV5640_set_JPEG_size(OV5640_320x240);delay(1000);
	#else
	myCAM1.OV5642_set_JPEG_size(OV5642_320x240);delay(1000);
	#endif
	delay(1000);
	myCAM1.clear_fifo_flag();
	myCAM2.clear_fifo_flag();
	myCAM3.clear_fifo_flag();
}

void sendPhotos(char str[8]) {
	// byte buf[256];
	// uint32_t length = 0;
	File photoFile;
	strcat(str, ".jpg");
	photoFile = SD.open(str, O_READ);
	if (!photoFile) {
		Serial.print("Failed to open ");
		Serial.println(str);
	}

	// length = photoFile.size();
	XBee.print("Image,");

	// Send Image
	// while (length--) {
	// 	photoFile.readBytes(buf, )
	// }
	while (photoFile.available()) {
		XBee.print(photoFile.read());
	}

	XBee.println(",Image End");
}
