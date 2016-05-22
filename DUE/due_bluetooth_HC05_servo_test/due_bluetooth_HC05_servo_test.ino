/*
Arduino Due + HC-05 bluetooth + servo
- servo controlled via bluetooth

Serial (Tx/Rx) communicate to PC via USB
Serial1 (Tx1/Rx1) connect to HC-05
HC-05 Rx - Due Tx1 (18)
HC-05 Tx - Due Rx1 (19)
HC-05 GND - Due GND
HC-05 VCC - Due 3.3V
*/
#include <Servo.h>

#define HC05 Serial1
#define LED 2

#define SERVO1_MIN 10
#define SERVO1_MAX 180

#define SERVO1_PWM_PIN 3
Servo servo1;
int servo1_val;

void setup(){
  pinMode(LED, OUTPUT);
  flashLed(500);
  Serial.begin(9600);
  HC05.begin(9600);
  Serial.write("\nServo test\n");
  servo1.attach(SERVO1_PWM_PIN);
  servo1_val = servo1.read();
}

void loop(){
  while(HC05.available()){
    processInput();
  }
}// void loop()

void processInput (){
  byte c = HC05.read ();
  //char data = HC05.read();
  //Serial.write(c);
  switch (c){   
    case 'c':
    {
      //turnLed(0);     
      //int deg = HC05.parseInt();
      //Serial.println("deg1: "+String(deg));
      //setServo(1, deg);
      setServo(1, 50);
      break;
    }

    case 'p':
    {
      //turnLed(1);      
      //int deg = HC05.parseInt();
      // Serial.println("deg2: "+String(deg));
      setServo(1, 110);
      break;
    }
  }// switch (c)
}// void processInput ()

void flashLed(int delay_ms){
  digitalWrite(LED, HIGH); 
  delay(delay_ms);
  digitalWrite(LED, LOW);
}

void turnLed(int state){
  if(state==1){
      digitalWrite(LED,  HIGH); 
  }
  else{
      digitalWrite(LED, LOW); 
  }
}

void setServo(int servoNumber, int deg){
  if(servoNumber==1 && deg>=SERVO1_MIN && deg<=SERVO1_MAX){
    servo1.write(deg);
    servo1_val = servo1.read();
    Serial.println("servo1: "+String(servo1_val));
  }
}// void setServo(int servoNumber, int deg)



