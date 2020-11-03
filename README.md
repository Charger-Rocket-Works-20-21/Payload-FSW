# Payload-FSW
Payload flight software for the 2021 USLI Competition Team from the University of Alabama in Huntsville

Uses:
* a Teensy 4.1 (with Teensyduino add-on)
* a Bosch BMP386
* a Bosch BNO055
* a NTCLE thermistor
* a Xbee Radio, and
* 

Dependencies:
* Arduino Libraries (Wire, SPI)
* Adafruit BNO055 Library
* Adafruit BMP3XX Library

Tasks:
* Query Positional Sensor
* Query Pressure Sensor
* Query Temperature Sensor
* Calculate Altitude from Pressure and Temperature
* Detect Landing
* Level Payload
* Take Pictures of Landed Surroundings
* Transmit Telemetry and Photographed Surroundings