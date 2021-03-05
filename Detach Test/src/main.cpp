#include <Arduino.h>
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include <Wire.h>

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensor;
//Uncomment the following line to use the optional shutdown and interrupt pins.
// SFEVL53L1X distanceSensor(Wire2, SHUTDOWN_PIN, INTERRUPT_PIN);

double smoothDistance = 0;
double smoothingFactor = 0.5;

void setup() {
	Wire.begin();

	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(8, OUTPUT);

	delay(500);
	
	Serial.begin(9600);
	Serial.println("VL53L1X Qwiic Test");

	if (distanceSensor.begin() != 0) { //Begin returns 0 on a good init
		Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
	}
	Serial.println("Sensor online!");
}

void loop() {
	distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
	while (!distanceSensor.checkForDataReady()) {
		delay(1);
	}
	int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
	distanceSensor.clearInterrupt();
	distanceSensor.stopRanging();

	Serial.print("Distance(mm): ");
	Serial.print(distance);

	float distanceInches = distance * 0.0393701;
	float distanceFeet = distanceInches / 12.0;

	Serial.print("\tDistance(ft): ");
	Serial.print(distanceFeet, 2);

	smoothDistance = smoothingFactor * distanceFeet + (1 - smoothingFactor) * smoothDistance;
	Serial.print("\tSmoothDistance(ft): ");
	Serial.print(smoothDistance, 2);
	Serial.println();

	if (distanceFeet <= 4.0) {
		digitalWrite(8, HIGH);
		digitalWrite(LED_BUILTIN, HIGH);
	}
	else {
		digitalWrite(8, LOW);
		digitalWrite(LED_BUILTIN, LOW);
	}
	delay(10);
}