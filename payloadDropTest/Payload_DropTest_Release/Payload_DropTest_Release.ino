#include <Servo.h>

#define pulse   3
#define min_p   600
#define max_p   2400
#define s_close 650
#define s_open  1000
#define b_size  20
#define camera  2

Servo myServo;
char buf[b_size];
int buf_pos = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Testing");

  myServo.attach(pulse);
  myServo.writeMicroseconds(s_open);

  clear_buf();

  pinMode(camera,OUTPUT);
  digitalWrite(camera, HIGH);
}

// int pwidth = min_p; // DEBUG
void loop() {
  /* DEBUG
    // put your main code here, to run repeatedly:
    myServo.writeMicroseconds(pwidth);
    pwidth += 10;
    delay(100);
    if(pwidth == max_p){
    pwidth = min_p;
    myServo.writeMicroseconds(pwidth);
    delay(1000);
    }
  */

  if (Serial.available()) {
    char c = Serial.read();

    /*
      Serial.print(c);
      Serial.print('\t');
      Serial.println((int) c);
    */

    if (c == '\n') {
      //Serial.println(buf);

      if (String(buf) == "Lock") {
        cam_toggle();
        Serial.println("Locked");
        myServo.writeMicroseconds(s_close);
      }
      else if (String(buf) == "Release") {
        cam_toggle();
        Serial.println("Released");
        myServo.writeMicroseconds(s_open);
      }
      else if (String(buf) == "Video Toggle"){
        cam_toggle();
        Serial.println("Toggled");
      }

      //Serial.println(buf);
      clear_buf();
      buf_pos = 0;
      //Serial.println(buf);
    }
    else {
      buf[buf_pos] = c;
      buf_pos = (buf_pos + 1) % b_size;
      //Serial.println(buf);
    }
  }
}

void clear_buf() {
  for (int i = 0; i < b_size; i++) buf[i] = 0;
}

void cam_toggle() {
  digitalWrite(camera, LOW);
  delay(1000);
  digitalWrite(camera, HIGH);
}
