EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L teensy:Teensy4.1 U?
U 1 1 5FD1783C
P 900 -2600
F 0 "U?" H 900 -35 50  0000 C CNN
F 1 "Teensy4.1" H 900 -126 50  0000 C CNN
F 2 "" H 500 -2200 50  0001 C CNN
F 3 "" H 500 -2200 50  0001 C CNN
	1    900  -2600
	1    0    0    -1  
$EndComp
$Comp
L Sensor_Motion:BNO055 U?
U 1 1 5FD29B92
P 6500 -4400
F 0 "U?" H 6500 -3519 50  0000 C CNN
F 1 "BNO055" H 6500 -3610 50  0000 C CNN
F 2 "Package_LGA:LGA-28_5.2x3.8mm_P0.5mm" H 6750 -5050 50  0001 L CNN
F 3 "https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST_BNO055_DS000_14.pdf" H 6500 -4200 50  0001 C CNN
	1    6500 -4400
	1    0    0    -1  
$EndComp
$Comp
L Sensor_Pressure:BMP388 U?
U 1 1 5FD30692
P 6000 -2150
F 0 "U?" H 6500 -869 50  0000 C CNN
F 1 "BMP388" H 6500 -960 50  0000 C CNN
F 2 "" H 6500 -1650 50  0001 C CNN
F 3 "" H 6500 -1650 50  0001 C CNN
	1    6000 -2150
	1    0    0    -1  
$EndComp
$Comp
L Sensor_Distance:VL53L1CXV0FY1 U?
U 1 1 5FD335B3
P -950 -5450
F 0 "U?" H -620 -5404 50  0000 L CNN
F 1 "VL53L1CXV0FY1" H -620 -5495 50  0000 L CNN
F 2 "Sensor_Distance:ST_VL53L1x" H -275 -6000 50  0001 C CNN
F 3 "https://www.st.com/resource/en/datasheet/vl53l1x.pdf" H -850 -5450 50  0001 C CNN
	1    -950 -5450
	1    0    0    -1  
$EndComp
$EndSCHEMATC
