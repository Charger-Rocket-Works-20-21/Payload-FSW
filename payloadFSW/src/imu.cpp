/* Reference: 
 *    - https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
 *    - https://learn.adafruit.com/using-the-adafruit-unified-sensor-driver/how-does-it-work
 */
#include "imu.h"


// Global Variables
sensors_event_t event;
Adafruit_BNO055 bno = Adafruit_BNO055(55);


// ------------------- Functions -----------------------

/*
 * Inertial Measurement Unit Inititialization
 */
void imu_init(Adafruit_BNO055* sensor){
	if (! sensor->begin()){
    	Serial.println("BNO055 Not Detected");
		while(1);
	}
  
	delay(1000);
  
	sensor->setExtCrystalUse(true);
}

double resultantAccel(double smoothingFactor, double smoothAcceleration) {
	bno.getEvent(&event);

	double accelx = event.acceleration.x;
	double accely = event.acceleration.y;
	double accelz = event.acceleration.z;

	double resultantVector = sqrt(pow(accelx, 2.0) + pow(accely, 2.0) + pow(accelz, 2.0));
	return smoothingFactor * resultantVector + (1 - smoothingFactor) * smoothAcceleration;
}

// -------------------- Test Functions ---------------------
#ifdef DEBUG

void bno055_test(void){
    bno.getEvent(&event);

    Serial.print("X: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.orientation.z, 4);
    Serial.println("");
}

#endif