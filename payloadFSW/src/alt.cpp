/* Reference: 
 *    - https://learn.adafruit.com/adafruit-bmp388-bmp390-bmp3xx/arduino
 *    - https://adafruit.github.io/Adafruit_BMP3XX/html/index.html
 */

#include "alt.h"

// Global Variable Declarations
Adafruit_BMP3XX bmp;

// ------------------- Functions -----------------------

/*
 * Atmospheric Pressure Sensor Inititialization
 */
bool alt_init(Adafruit_BMP3XX* sensor){
	if (!sensor->begin_I2C()) {
		Serial.println("BMP388 Not Detected");
    	return false;
	}
  
	sensor->setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
	sensor->setPressureOversampling(BMP3_OVERSAMPLING_4X);
	sensor->setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
	sensor->setOutputDataRate(BMP3_ODR_50_HZ);
	return true;
}

double getSmoothTemp(double smoothingFactor, double smoothTemp) {
	double newTemp = bmp.readTemperature();
	return smoothingFactor * newTemp + (1 - smoothingFactor) * smoothTemp;
}

double getSmoothPres(double smoothingFactor, double smoothPres) {
	double newPres = bmp.readPressure();
	return smoothingFactor * newPres + (1 - smoothingFactor) * smoothPres;
}

double getSmoothAlt(double smoothingFactor, double smoothAlt) {
	double newAlt = bmp.readAltitude(SEALEVELPRESSURE);
	return smoothingFactor * newAlt + (1 - smoothingFactor) * smoothAlt;
}

double getSmoothVel(double smoothingFactor, double smoothVel, double smoothAlt, double pastTime, double currentTime) {
	double newAlt = getSmoothAlt(smoothingFactor, smoothAlt);
	double newVel = (newAlt - smoothAlt) / (currentTime - pastTime);
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