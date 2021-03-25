/* Reference: 
 *    - https://learn.adafruit.com/adafruit-bmp388-bmp390-bmp3xx/arduino
 *    - https://adafruit.github.io/Adafruit_BMP3XX/html/index.html
 */

#include "alt.h"


// ------------------- Functions -----------------------

/*
 * Atmospheric Pressure Sensor Inititialization
 */
bool altInit(Adafruit_BMP3XX* sensor){
	if (!sensor->begin_I2C(0x77, &Wire1)) {
		Serial.println("BMP388 Not Detected");
    	return false;
	}
  
  	Serial.println("BMP388 Detected");

	sensor->setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
	sensor->setPressureOversampling(BMP3_OVERSAMPLING_4X);
	sensor->setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
	sensor->setOutputDataRate(BMP3_ODR_50_HZ);
	return true;
}

double getSmoothBmpTemp(Adafruit_BMP3XX* sensor, double smoothingFactor, double smoothTemp) {
	double newTemp = sensor->readTemperature();
	Serial.println(newTemp);
	return smoothingFactor * newTemp + (1 - smoothingFactor) * smoothTemp;
}

double getSmoothPres(Adafruit_BMP3XX* sensor, double smoothingFactor, double smoothPres) {
	double newPres = sensor->readPressure();
	Serial.println(newPres);
	return smoothingFactor * newPres + (1 - smoothingFactor) * smoothPres;
}

double getSmoothAlt(Adafruit_BMP3XX* sensor, double smoothingFactor, double smoothAlt) {
	double newAlt = sensor->readAltitude(SEALEVELPRESSURE);
	Serial.println(newAlt);
	return smoothingFactor * newAlt + (1 - smoothingFactor) * smoothAlt;
}

double getSmoothVel(Adafruit_BMP3XX* sensor, double smoothingFactor, double smoothVel, double smoothAlt, double pastTime, double missionTime) {
	double newAlt = getSmoothAlt(sensor, smoothingFactor, smoothAlt);
	double newVel = (newAlt - smoothAlt) / (missionTime - pastTime);
	return smoothingFactor * newVel + (1 - smoothingFactor) * smoothVel;
}

// -------------------- Test Functions ---------------------
#ifdef DEBUG
void bmp3XX_test(void){
    if (!bmp.performReading()) {
        Serial.println("Failed to perform reading :(");
        return;
    }

    Serial.print("Temperature = ");
    Serial.print(bmp.temperature);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bmp.pressure / 100.0);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bmp.readAltitude(SEALEVELPRESSURE));
    Serial.println(" m");
    Serial.println();
}
#endif /* DEBUG */