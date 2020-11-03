/*
 * Main sketch for Charger Rocket Works Payload Flight Software
 * NASA SLI 2020-21
 * 
 * Uses the following libraries:
 * Adafruit Unified Sensor
 * Adafruit BNO055
 * Adafruit BusIO
 * Adafruit BMP3XX
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <utility/imumaths.h>

#define SEALEVELPRESSURE     1013.25   // hPa
#define SEALEVELTEMPERATRE   273.15    // Kelvin

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BMP3XX bmp;

void setup() {
  // Initialize 
  Serial.begin(9600);

  // Initialize IMU
  imu_init(&bno);

  
  // Initialize Altimeter
  alt_init(&bmp);
  
 
}

void loop() {
  // Get IMU event
  sensors_event_t event;
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

  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
  Serial.println();
  
  delay(100);
}

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
