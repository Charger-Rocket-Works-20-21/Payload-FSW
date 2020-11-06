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
void alt_init(Adafruit_BMP3XX* sensor){
  if (!sensor->begin_I2C()) {
    Serial.println("BMP388 Not Detected");
    while(1);
  }
  
  sensor->setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  sensor->setPressureOversampling(BMP3_OVERSAMPLING_4X);
  sensor->setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  sensor->setOutputDataRate(BMP3_ODR_50_HZ);
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