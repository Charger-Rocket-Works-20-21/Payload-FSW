#include "system.h"

// Global Variable Declarations
sensors_event_t event;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BMP3XX bmp;

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

void bno055_test(void){
    bno.getEvent(&event);

    Serial.print("X: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.orientation.z, 4);
    Serial.println("");

    if (!bmp.performReading()) {
        Serial.println("Failed to perform reading :(");
        return;
    }
}

void bmp3XX_test(void){
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