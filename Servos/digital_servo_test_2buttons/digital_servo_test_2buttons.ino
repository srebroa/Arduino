#include <Servo.h>
Servo servo1;

int right = 7; // button Right
int left = 8;  // button Left
int right_state;
int left_state;
int servo_val;

void setup(){
  Serial.begin(9600);
  pinMode(right, INPUT);
  pinMode(left, INPUT);
  servo1.attach(6);
  servo_val = servo1.read();
}

void loop(){
  right_state = digitalRead(right);
  left_state = digitalRead(left);
 
  if(right_state == HIGH)
  {
    servo1.write(servo_val - 1);
    delay(14);
    servo_val = servo1.read();
    Serial.println("Turn Right");
  }
 
  if(left_state == HIGH)
  {
  servo1.write(servo_val + 1);
  delay(14);
  servo_val = servo1.read();
  Serial.println("Turn Left");
  }
}
