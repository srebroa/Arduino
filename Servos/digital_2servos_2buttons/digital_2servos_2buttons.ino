// Testing Digital Servo LF-20MG
#include <Servo.h>
#define INCREASE_STEP 5
#define DECREASE_STEP 2
Servo servo1;
Servo servo2;

int s1_up_pin = 7; 
int s1_down_pin = 8; 
int s2_up_pin = 9; 
int s2_down_pin = 10;  
int s1_up_button;
int s1_down_button;
int s2_up_button;
int s2_down_button;
int servo1_val;
int servo2_val;

void setup(){
  Serial.begin(9600);
  pinMode(s1_up_pin, INPUT);
  pinMode(s1_down_pin, INPUT);
  pinMode(s2_up_pin, INPUT);
  pinMode(s2_down_pin, INPUT);
  servo1.attach(5);
  servo2.attach(6);
  //servo1_val = servo1.read();
  servo2_val = servo2.read();
  servo1_val = servo1.read();
}// void setup()

void loop(){
  s1_up_button = digitalRead(s1_up_pin);
  s1_down_button = digitalRead(s1_down_pin);
  s2_up_button = digitalRead(s2_up_pin);
  s2_down_button = digitalRead(s2_down_pin);
 
  if(s1_up_button == HIGH && servo1_val>10+DECREASE_STEP){
    servo1.write(servo1_val - DECREASE_STEP);
    delay(14);
    servo1_val = servo1.read();
    Serial.println("Angle1: "+String(servo1_val));
  }   
  if(s1_down_button == HIGH && servo1_val<=180-INCREASE_STEP){
    servo1.write(servo1_val + INCREASE_STEP);
    delay(14);
    servo1_val = servo1.read();
    Serial.println("Angle: "+String(servo1_val));
  }  
  if(s2_up_button == HIGH && servo2_val>10+DECREASE_STEP){
    servo2.write(servo2_val - DECREASE_STEP);
    delay(14);
    servo2_val = servo2.read();
    Serial.println("Angle2: "+String(servo2_val));
  }    
  if(s2_down_button == HIGH && servo2_val<=180-INCREASE_STEP){
    servo2.write(servo2_val + INCREASE_STEP);
    delay(14);
    servo2_val = servo2.read();
    Serial.println("Angle2: "+String(servo2_val));
  }  
}// void loop()
