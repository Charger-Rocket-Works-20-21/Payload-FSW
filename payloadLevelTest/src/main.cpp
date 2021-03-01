#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <SoftwareSerial.h>
#include <utility/imumaths.h>
#include <SD.h>

#define SAMPLERATE_DELAY_MS 50

#define BMP_SCL 16
#define BMP_SDI 17

#define SEALEVELPRESSURE_HPA 1013.25

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_BMP3XX bmp;

uint16_t packetCount = 0;
double currentTime;
bool ledOn;
uint8_t calibration;
uint16_t blinkRate;

// PID Constants
double kp, ki, kd;
double output, targetPoint;
double errSum, lastErr;
uint32_t pidLastMillis = 0;

void pidUpdate (double xorient, double zorient, uint32_t millis);

void setup() {
  	// put your setup code here, to run once:
	Serial.begin(115200);
	delay(1000);
	Serial.println("Beginning Payload Autogyro Drop Test...");
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
	delay(500);
	ledOn = true;

	SD.begin(BUILTIN_SDCARD);

	if (!bmp.begin_I2C(0x77, &Wire1)) {   // hardware I2C mode, can pass in address & alt Wire
		Serial.println("Could not find a valid BMP3 sensor, check wiring!");
		while (1);
  	}
	else {
		Serial.println("BMP388 Detected");
	}
	if (!bno.begin()) {
		Serial.println("BNO055 Not Detected...");
		while(1);
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

	File dataFile = SD.open("datalog.txt", FILE_WRITE);
	if (dataFile) {
		dataFile.println("LEVELLING TEST FSW INITIALIZED");
		dataFile.println(" , , , , , Acceleration, , , Orientation, ");
		dataFile.println("Packet, Time, Temp, Pressure, Altitude, x, y, z, x, y, z");
		dataFile.close();
	}
	else {
		Serial.println("Could not open datalog.txt");
	}

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

	String packet = "";
	packet += String(packetCount);
	packet += ",";
	packet += String(currentTime);
	packet += ",";
	packet += String(bmp.temperature);
	packet += ",";
	packet += String(bmp.pressure);
	packet += ",";
	packet += String(bmp.readAltitude(SEALEVELPRESSURE_HPA));
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
	Serial.println(packet);

	File dataFile = SD.open("datalog.txt", FILE_WRITE);
	if (dataFile) {
		dataFile.println(packet);
		dataFile.close();
	}
	else {
		Serial.println("Could not open datalog.txt");
	}

	delay(SAMPLERATE_DELAY_MS);
}



void pidUpdate (double xorient, double zorient, uint32_t millis) {
	double rorient = sqrt(pow((xorient+90), 2) + pow(zorient, 2)); // Resultant vector

	double dt = ((double)(millis - pidLastMillis))/1000.0;
	
	double error = targetPoint - rorient;
	
	errSum += (error * dt);
	double dErr = (error - lastErr) / dt;
	
	//printf("P: = %f I: %f D: %f\n",kp*error, ki*errSum, kd*dErr);
	output = kp*error + ki*errSum + kd*dErr;
	
	pidLastMillis = millis;
	lastErr = error;
}