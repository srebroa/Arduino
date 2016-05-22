/*
Arduino Due + HC-05 bluetooth 
- remote turn on/off LED

Serial (Tx/Rx) communicate to PC via USB
Serial1 (Tx1/Rx1) connect to HC-05
HC-05 Rx - Due Tx1 (18)
HC-05 Tx - Due Rx1 (19)
HC-05 GND - Due GND
HC-05 VCC - Due 3.3V
LED - Due (2)
*/
#include <Servo.h>

#define HC05 Serial1
#define LED 2

#define SERVO1_MIN 10
#define SERVO1_MAX 180

void setup(){
  pinMode(LED, OUTPUT);
  flashLed(500);
  Serial.begin(9600);
  HC05.begin(9600);
  Serial.write("\nHC-05 bluetooth LED test\n");
}

void loop(){
  while(HC05.available()){
    processInput();
  }
}// void loop()

void processInput (){
  byte c = HC05.read ();
  switch (c){   
    case 'c':
    {
      turnLed(0);     
      Serial.println("LED ON");
      break;
    }

    case 'p':
    {
      turnLed(1);     
      Serial.println("LED OFF");
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




