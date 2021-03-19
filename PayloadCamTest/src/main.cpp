#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "ArduCAM.h"
#include "ov5642_regs.h"

#define FRAMES_NUM 0x00

const int CS = 7;
bool is_header = false;
int total_time = 0;
uint8_t resolution = OV5642_640x480;
uint32_t line, column;

ArduCAM myCAM(OV5642, CS);
uint8_t saveRAW(void);

void setup() {
  uint8_t vid, pid, temp;

  Serial.begin(9600);
  Serial.println("ArduCAM Start!");
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
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

  SD.begin(BUILTIN_SDCARD);

  myCAM.set_format(RAW);
  myCAM.InitCAM();
  myCAM.set_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
}

void loop() {
  File outFile;
  char VL;
  char str[8];
  byte buf[256];
  static int k = 0, m = 0;
  int i, j = 0;

  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  myCAM.OV5642_set_RAW_size(resolution);
  delay(1000);
  myCAM.start_capture();
  Serial.println("Start Capture.");
  total_time = millis();

  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
  Serial.println("CAM Capture Done.");
  total_time = millis() - total_time;
  Serial.println(total_time);
  Serial.println("Saving the image...");
  total_time = millis();
  k = k + 1;
  __itoa(k, str, 10);
  strcat(str, ".raw");
  outFile = SD.open(str, O_WRITE | O_CREAT | O_TRUNC);
  if (!outFile) {
    Serial.println("File open Error.");
    return;
  }
  if (resolution == OV5642_640x480) {
    line = 640; column = 480;
  }

  for (i = 0; i < line; i++) {
    for (j = 0; j < column; j++) {
      VL = myCAM.read_fifo();
      buf[m++] = VL;
      if (m >= 256) {
        outFile.write(buf, 256);
        m = 0;
      }
    }
  }

  if (m > 0) {
    outFile.write(buf, m);
    m = 0;
  }

  outFile.close();
  Serial.println("Image saved OK.");

  myCAM.clear_fifo_flag();
  delay(5000);
}