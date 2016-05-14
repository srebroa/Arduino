// Testing Digital Servo LF-20MG
#include <Servo.h>
#define INCREASE_STEP 5
#define DECREASE_STEP 2
Servo servo1;

int right = 7; // Right button
int left = 8;  // Left button
int right_state;
int left_state;
int servo_val;

void setup(){
  Serial.begin(9600);
  pinMode(right, INPUT);
  pinMode(left, INPUT);
  servo1.attach(6);
  servo_val = servo1.read();
}// void setup()

void loop(){
  right_state = digitalRead(right);
  left_state = digitalRead(left);
 
  if(right_state == HIGH && servo_val>10+DECREASE_STEP){
    servo1.write(servo_val - DECREASE_STEP);
    delay(14);
    servo_val = servo1.read();
    Serial.println("Angle: "+String(servo_val));
  } 
  if(left_state == HIGH && servo_val<=180-INCREASE_STEP){
  servo1.write(servo_val + INCREASE_STEP);
  delay(14);
  servo_val = servo1.read();
  Serial.println("Angle: "+String(servo_val));
  }
}// void loop()
