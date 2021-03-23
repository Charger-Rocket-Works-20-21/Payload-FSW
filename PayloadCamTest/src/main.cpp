#include <Arduino.h>
#include <SPI.h>
// #include <SD.h>
#include "ArduCAM.h"
#include "ov5642_regs.h"

#define FRAMES_NUM 0x00
#define OV5642_MINI_5MP_PLUS
#define CS 9

bool is_header = false;
int total_time = 0;
uint8_t resolution = OV5642_640x480;

ArduCAM myCAM(OV5642, CS);

void myCAMSaveToSDFile(ArduCAM myCAM);

void setup() {
	uint8_t vid, pid, temp;
	Wire.begin();
	delay(5000);
	Serial.begin(9600);
	Serial.println("ArduCAM Start!");
	pinMode(CS, OUTPUT);
	pinMode(7, OUTPUT);
	digitalWrite(CS, HIGH);
	digitalWrite(7, HIGH);
	SPI.begin();

	myCAM.write_reg(0x07, 0x80);
	delay(100);
	myCAM.write_reg(0x07, 0x00);
	delay(100);

	while(1) {
		myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
		temp = myCAM.read_reg(ARDUCHIP_TEST1);
		if (temp != 0x55) {
			Serial.println("SPI Interface Error");
			delay(1000);
			continue;
		}
		else {
			Serial.println("SPI Interface OK");
			break;
		}
	}

	// Checks the module is OV5642, requires I2C
	while(1) {
	  myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
	  myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
	  if ((vid != 0x56) || (pid != 0x42)) {
	    Serial.println("Can't find OV5642 Module");
	    delay(1000);
	    continue;
	  }
	  else {
	    Serial.println("OV5642 Detected.");
	    break;
	  }
	}

	// SD.begin(BUILTIN_SDCARD);

	myCAM.set_format(JPEG);
	myCAM.InitCAM();
	myCAM.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
	myCAM.clear_fifo_flag();
	myCAM.write_reg(ARDUCHIP_FRAMES, FRAMES_NUM);
	myCAM.OV5642_set_JPEG_size(resolution);
	delay(1000);
	myCAM.clear_fifo_flag();
}

void loop() {
	myCAMSaveToSDFile(myCAM);
	delay(5000);
}

void myCAMSaveToSDFile(ArduCAM myCAM) {
	char str[8];
	byte buf[256];
	static int i = 0, k = 0;
	uint8_t temp = 0, temp_last = 0;
	uint32_t length = 0;
	bool is_header = false;
	// File outFile;

	myCAM.flush_fifo();
	myCAM.clear_fifo_flag();
	// Start Capture
	myCAM.start_capture();
	Serial.println("Start Capture.");
	while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
	Serial.println("CAM Capture Done.");
	length = myCAM.read_fifo_length();
	Serial.print("Fifo length: ");
	Serial.println(length);
	if (length >= MAX_FIFO_SIZE) {
		Serial.println("Over sized.");
		return;
	}
	if (length == 0) {
		Serial.println("Size is 0.");
		return;
	}
	k = k + 1;
	// __itoa(k, str, 10);
	strcat(str, ".jpg");
	// outFile = SD.open(str, O_WRITE | O_CREAT | O_TRUNC);
	// if (!outFile) {
	// 	Serial.println("File open Error.");
	// 	return;
	// }
	
	myCAM.CS_LOW();
	myCAM.set_fifo_burst();
	while (length--) {
		temp_last = temp;
		temp = SPI.transfer(0x00);
		//Read JPEG data from FIFO
		if ((temp == 0xD9) && (temp_last == 0xFF)) {
			buf[i++] = temp;
			myCAM.CS_HIGH();
			// outFile.write(buf, i);
			// outFile.close();
			Serial.println("IMage Saved OK.");
			is_header = false;
			i = 0;
		}
		if (is_header == true) {
			if (i < 256) {
				buf[i++] = temp;
			}
			else {
				//Write 256 bytes image data to file
				myCAM.CS_HIGH();
				// outFile.write(buf, 256);
				i = 0;
				buf[i++] = temp;
				myCAM.CS_LOW();
				myCAM.set_fifo_burst();
			}
		}
		else if ((temp == 0xD8) & (temp_last == 0xFF)) {
			is_header = true;
			buf[i++] = temp_last;
			buf[i++] = temp;
		}
	}
}