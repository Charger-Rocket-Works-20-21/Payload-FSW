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
#define BNO_ADDRESS 0x28
#define BNO_ADD_SEL 40
#define BMP_ADDRESS 0x76
#define BSIZE  20

#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

#define SEALEVELPRESSURE_HPA 1013.25

#define FRAMES_NUM 0x00

#define SCB_AIRCR (*(volatile uint32_t *)0xE000ED0C) // Application Interrupt and Reset Control location

States states;
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO_ADDRESS, &Wire1);
Adafruit_BMP3XX bmp;
SFEVL53L1X distanceSensor(Wire1, SHUTDOWN_PIN, INTERRUPT_PIN);
AsyncDelay readInterval;
SoftwareSerial XBee(28,29);
GP20U7 gps = GP20U7(Serial);

ArduCAM myCAM1(OV5642, states.CS1);
ArduCAM myCAM2(OV5642, states.CS2);
ArduCAM myCAM3(OV5642, states.CS3);

uint16_t packetCount = 0;
double smoothingFactor = 0.75;
double missionTime, previousTime, diffTime, previousXbeeTime;
bool ledOn;
uint16_t blinkRate;
bool transmitAllowed = true;
bool sentPhotos = false;
bool rangefinderInit = false;
char buf[BSIZE];
int buf_pos = 0;

double initialAlt;
double temperature;
double pressure;
double altitude, previousAltitude;
double velocity;
double accelx, accely, accelz;
double orientx, orienty, orientz, orientxCorrected;
double targetw, targetx, targety, targetz;
double quatw, quatx, quaty, quatz;
double landedOrientx, landedOrienty, landedOrientz;
double leveledOrientx, leveledOrienty, leveledOrientz;
double resultCurrent, resultPrevious;
uint8_t calibration;
double distance;

Geolocation currentLocation;

char cam1String[16] = {'c', 'a', 'm', '1', '.', 'j', 'p', 'g'};
char cam2String[16] = {'c', 'a', 'm', '2', '.', 'j', 'p', 'g'};
char cam3String[16] = {'c', 'a', 'm', '3', '.', 'j', 'p', 'g'};

void readCommand();
void initCameras();
void myCAMSaveToSDFile(ArduCAM myCAM,  char str[8]);
void sendPhotos(char str[8]);
imu::Quaternion invert(imu::Quaternion quat);

void setup() {
  	// put your setup code here, to run once:
	Serial.begin(115200);
	Serial7.begin(115200);
	XBee.begin(115200);
	delay(1000);
	Serial.println("Beginning Payload Flight Software...");
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(MOTOR1, OUTPUT);
	pinMode(MOTOR2, OUTPUT);
	pinMode(MOTOR3, OUTPUT);
	// pinMode(MOTOR1R, OUTPUT);
	// pinMode(MOTOR2R, OUTPUT);
	// pinMode(MOTOR3R, OUTPUT);
	pinMode(BNO_ADD_SEL, OUTPUT);
	pinMode(7, OUTPUT);
	pinMode(RELEASE_POWER1, OUTPUT);
	pinMode(RELEASE_POWER2, OUTPUT);
	pinMode(states.CS1, OUTPUT);
	pinMode(states.CS2, OUTPUT);
	pinMode(states.CS3, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
	digitalWrite(BNO_ADD_SEL, HIGH);
	digitalWrite(7, HIGH);
	digitalWrite(states.CS1, HIGH);
	digitalWrite(states.CS2, HIGH);
	digitalWrite(states.CS3, HIGH);
	delay(500);
	ledOn = true;

	// FOR LEVEL TESTING
	// states.driveMotor(1,1);
	// states.driveMotor(2,1);
	// states.driveMotor(3,1);
	// while(1);
	// states.driveMotor(1,2);
	// states.driveMotor(2,2);
	// states.driveMotor(3,2);
	// while(1);

	SPI.begin();

	SD.begin(BUILTIN_SDCARD);

	if (!bmp.begin_I2C(BMP_ADDRESS, &Wire1)) {   // hardware I2C mode, can pass in address & alt Wire
		Serial.print("BMP388 Not Detected\t");
		XBee.print("BMP388 Not Detected\t");
  	}
	else {
		Serial.print("BMP388 Detected\t\t");
		XBee.print("BMP388 Detected\t\t");
	}
	if (!bno.begin()) {
		Serial.print("BNO055 Not Detected\t\t");
		XBee.print("BNO055 Not Detected\t\t");
	}
	else {
		Serial.print("BNO055 Detected\t\t");
		XBee.print("BNO055 Detected\t\t");
	}
	if (distanceSensor.begin() != 0) {
		Serial.println("Rangefinder Not Detected\t\t");
		XBee.println("Rangefinder Not Detected\t\t");
	}
	else {
		Serial.println("Rangefinder Detected\t\t");
		XBee.println("Rangefinder Detected\t\t");
		rangefinderInit = true;
	}
	// gps.begin(); // Freezes Code if included, will need to fix
	
	delay(500);
	bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
	bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
	bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
	bmp.setOutputDataRate(BMP3_ODR_50_HZ);
	bno.setExtCrystalUse(true);

	File dataFile = SD.open("datalog.txt", FILE_WRITE);
	if (dataFile) {
		dataFile.println("CRW PAYLOAD FSW INITIALIZED");
		dataFile.println(" , , , , , Acceleration, , , Orientation, ");
		dataFile.println("Packet, Time, Temp, Pressure, Altitude, x, y, z, x, y, z");
		dataFile.close();
	}
	else {
		Serial.println("Could not open datalog.txt");
	}

	// Record Initial Altitude and store it to EEPROM, if not already saved
	Serial.println("Reading EEPROM for previous state.");
	if (EEPROM.read(1) != 0) {
		initialAlt = EEPROM.read(0);
	}
	else {
		for (int i = 0; i < 5; i++) {
			initialAlt = smoothingFactor * bmp.readAltitude(SEALEVELPRESSURE_HPA) + (1 - smoothingFactor) * initialAlt;

		}
		Serial.println(initialAlt);
		EEPROM.update(0, initialAlt);
	}

	states.setCurrentState(EEPROM.read(1));
	packetCount = EEPROM.read(2);
	landedOrientx = EEPROM.read(3);
	landedOrienty = EEPROM.read(4);
	landedOrientz = EEPROM.read(5);

	initCameras();

	for (int i = 0; i < 10; i++) {
		digitalWrite(LED_BUILTIN, HIGH);
		delay(100);
		digitalWrite(LED_BUILTIN, LOW);
		delay(100);
	}

	// noInterrupts();
	delay(500);
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

	String packet = "";
	packet += ",UAH Charger RocketWorks";
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
	packet += String(orientxCorrected);
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
			Serial7.print(packet);
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
		if (rangefinderInit) {
			distanceSensor.startRanging();
			while (!distanceSensor.checkForDataReady()) {delay(1);}
			distance = distanceSensor.getDistance();
			distanceSensor.clearInterrupt();
			distanceSensor.stopRanging();
			distance = distance * 0.0032808417;
			Serial.print(distance);
		}
		else {
			distance = 20;
		}
		states.descent(altitude, initialAlt, velocity, accelx, accely, accelz, distance);
		break;
	case LEVELLING:
	{
		resultPrevious = resultCurrent;
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
		resultCurrent = angleDiff;
		states.leveling(resultCurrent, resultPrevious);
		break;
	}
	case FINISHED:
		if (!sentPhotos) {
			transmitAllowed = false;			
			if(states.CAM1_EXIST) {myCAMSaveToSDFile(myCAM1, cam1String);}
			if(states.CAM2_EXIST) {myCAMSaveToSDFile(myCAM2, cam2String);}
			if(states.CAM3_EXIST) {myCAMSaveToSDFile(myCAM3, cam3String);}
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
			// Retake Pictures
			delay(500);
			if(states.CAM1_EXIST) {myCAMSaveToSDFile(myCAM1, cam1String);}
			if(states.CAM2_EXIST) {myCAMSaveToSDFile(myCAM2, cam2String);}
			if(states.CAM3_EXIST) {myCAMSaveToSDFile(myCAM3, cam3String);}
			delay(500);
		}
		else if (command.equalsIgnoreCase("I1")) {
			// Resend Image 1
			sendPhotos(cam1String);
		}
		else if (command.equalsIgnoreCase("I2")) {
			// Resend Image 2
			sendPhotos(cam2String);
		}
		else if (command.equalsIgnoreCase("I3")) {
			// Resend Image 3
			sendPhotos(cam3String);
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
		else if (command.equalsIgnoreCase("CEE")) {
			// Clear the EEPROM
			for (int i = 0; i < EEPROM.length(); i++) {
				EEPROM.update(i, 0);
			}
		}
		else if (command.equalsIgnoreCase("BLK")) {
			// Blink Onboard LED
			for (int i = 0; i < 10; i++) {
				digitalWrite(LED_BUILTIN, HIGH);
				delay(100);
				digitalWrite(LED_BUILTIN, LOW);
				delay(100);
			}
		}
		else if (command.equalsIgnoreCase("FS0")) {
			states.currentState = UNARMED;
		}
		else if (command.equalsIgnoreCase("FS1")) {
			states.currentState = STANDBY;
		}
		else if (command.equalsIgnoreCase("FS2")) {
			states.currentState = ASCENT;
		}
		else if (command.equalsIgnoreCase("FS3")) {
			states.currentState = DESCENT;
		}
		else if (command.equalsIgnoreCase("FS4")) {
			states.currentState = LEVELLING;
		}
		else if (command.equalsIgnoreCase("FS5")) {
			states.currentState = FINISHED;
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
	for (int i = 0; i < 3; i++) {
		myCAM1.write_reg(ARDUCHIP_TEST1, 0x55);
		temp = myCAM1.read_reg(ARDUCHIP_TEST1);
		if(temp != 0x55)
		{
			Serial.print(F("CAMERA 1 NOT DETECTED\t"));
		}else{
			states.CAM1_EXIST = true;
			Serial.print(F("Camera 1 Detected\t"));
		}
		myCAM2.write_reg(ARDUCHIP_TEST1, 0x55);
		temp = myCAM2.read_reg(ARDUCHIP_TEST1);
		if (temp != 0x55)
		{
			Serial.print(F("CAMERA 2 NOT DETECTED\t"));
		} else {
			states.CAM2_EXIST = true;
			Serial.print(F("Camera 2 Detected\t"));
		}
		myCAM3.write_reg(ARDUCHIP_TEST1, 0x55);
		temp = myCAM3.read_reg(ARDUCHIP_TEST1);
		if(temp != 0x55) {
			Serial.print(F("CAMERA 3 NOT DETECTED\t"));
		} else {
			states.CAM3_EXIST = true;
			Serial.print(F("Camera 3 Detected\t"));
		}
		Serial.println("");
		if (!(states.CAM1_EXIST||states.CAM2_EXIST||states.CAM3_EXIST)) {
			delay(250);
			continue;
		} else {
			break;
		}
	}

	if ((states.CAM1_EXIST||states.CAM2_EXIST||states.CAM3_EXIST)) {
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
			for (int i = 0; i < 3; i++) {
			//Check if the camera module type is OV5642
			myCAM1.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
			myCAM1.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
			if ((vid != 0x56) || (pid != 0x42)) {
				Serial.println(F("Can't find OV5642 module!"));
				delay(250);
				continue;
			} else {
				Serial.println(F("OV5642 detected."));
				break;
			}  
		}
		#endif
	}

	// //Change to JPEG capture mode and initialize the OV5640 modules
	// if (states.CAM1_EXIST) {
	// 	Serial.print("Initializing CAM1\t");
	// 	myCAM1.set_format(JPEG);
	// 	myCAM1.InitCAM();
	// 	myCAM1.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);   //VSYNC is active HIGH
	// 	myCAM1.clear_fifo_flag();
	// 	myCAM1.write_reg(ARDUCHIP_FRAMES, FRAMES_NUM);
	// 	myCAM1.OV5642_set_JPEG_size(OV5642_1024x768);
	// 	delay(1000);
	// 	myCAM1.clear_fifo_flag();
	// }
	// if (states.CAM2_EXIST) {
	// 	Serial.print("Initializing CAM2\t");
	// 	myCAM2.set_format(JPEG);
	// 	myCAM2.InitCAM();
	// 	myCAM2.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);   //VSYNC is active HIGH
	// 	myCAM2.clear_fifo_flag();
	// 	myCAM2.write_reg(ARDUCHIP_FRAMES, FRAMES_NUM);
	// 	myCAM2.OV5642_set_JPEG_size(OV5642_1024x768);
	// 	delay(1000);
	// 	myCAM2.clear_fifo_flag();
	// }
	// if (states.CAM3_EXIST) {
	// 	Serial.println("Initializing CAM3\t");
	// 	myCAM3.set_format(JPEG);
	// 	myCAM3.InitCAM();
	// 	myCAM3.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);   //VSYNC is active HIGH
	// 	myCAM3.clear_fifo_flag();
	// 	myCAM3.write_reg(ARDUCHIP_FRAMES, FRAMES_NUM);
	// 	myCAM3.OV5642_set_JPEG_size(OV5642_1024x768);
	// 	delay(1000);
	// 	myCAM3.clear_fifo_flag();
	// }
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
	myCAM1.clear_fifo_flag();
	myCAM2.clear_fifo_flag();
	myCAM3.clear_fifo_flag();
}

void myCAMSaveToSDFile(ArduCAM myCAM,  char str[8]) {

	XBee.print("Image,");


	byte buf[256];
	static int i = 0;
	uint8_t temp = 0,temp_last=0;
	uint32_t length = 0;
	bool is_header = false;
	//File outFile;
	//Flush the FIFO
	myCAM.flush_fifo();
	//Clear the capture done flag
	myCAM.clear_fifo_flag();
	//Start capture
	myCAM.start_capture();
	Serial.println(F("start Capture"));
	while(!myCAM.get_bit(ARDUCHIP_TRIG , CAP_DONE_MASK));
	Serial.println(F("Capture Done."));  
	length = myCAM.read_fifo_length();
	Serial.print(F("The fifo length is :"));
	Serial.println(length, DEC);
	if (length >= MAX_FIFO_SIZE) { //8M
		Serial.println(F("Over size."));
		return;
	}
	if (length == 0 ) { //0 kb
		Serial.println(F("Size is 0."));
		return;
	}
	//Open the new file
	//File photoFile = SD.open(str, FILE_WRITE);
	//photoFile = SD.open(str, O_WRITE | O_CREAT | O_TRUNC);
	//if(!photoFile){
	//	Serial.println(F("File open faild"));
	//	return;
	//}
	myCAM.CS_LOW();
	myCAM.set_fifo_burst();
	while ( length--) {
		temp_last = temp;
		temp =  SPI.transfer(0x00);
		//Read JPEG data from FIFO
		if ((temp == 0xD9) && (temp_last == 0xFF)) { //If find the end ,break while,
			buf[i++] = temp;  //save the last  0XD9     
			//Write the remain bytes in the buffer
			myCAM.CS_HIGH();
			XBee.write(buf, i);    
			//Close the file
			//photoFile.close();
			Serial.println(F("Image save OK."));
			is_header = false;
			i = 0;
		}  
		if (is_header == true) { 
			//Write image data to buffer if not full
			if (i < 256)
			buf[i++] = temp;
			else {
			//Write 256 bytes image data to file
			myCAM.CS_HIGH();
			XBee.write(buf, 256);
			i = 0;
			buf[i++] = temp;
			myCAM.CS_LOW();
			myCAM.set_fifo_burst();
			delay(10);
			}        
		}
		else if ((temp == 0xD8) & (temp_last == 0xFF)) {
			is_header = true;
			buf[i++] = temp_last;
			buf[i++] = temp;   
		} 
	} 


	XBee.println(",Image End");
}

void sendPhotos(char str[8]) {
	delay(1000);
	// byte buf[256];
	// uint32_t length = 0;
	File photoFile;
	photoFile = SD.open(str, O_READ);
	if (!photoFile) {
		Serial.print("Failed to open ");
		Serial.println(str);
		return;
	}

	// length = photoFile.size();
	XBee.print("Image,");

	// Send Image
	// while (length--) {
	// 	photoFile.readBytes(buf, )
	// }
	while (photoFile.available()) {
		XBee.write(photoFile.read());
	}

	XBee.println(",Image End");

	// Serial.print("Image,");

	// // Send Image
	// // while (length--) {
	// // 	photoFile.readBytes(buf, )
	// // }
	// while (photoFile.available()) {
	// 	Serial.print(photoFile.read());
	// 	delay(5);
	// }

	// Serial.println(",Image End");
}

imu::Quaternion invert(imu::Quaternion quat) {
	double iquatw = -quat.w();
	double iquatx = -quat.x();
	double iquaty = -quat.y();
	double iquatz = -quat.z();

	return imu::Quaternion(iquatw, iquatx, iquaty, iquatz);
}