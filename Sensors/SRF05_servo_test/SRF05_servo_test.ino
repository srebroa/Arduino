/*
SRF05_servo_test - Print the distance from the ultrasonic sensor to the nearest obstacle in a given direction (scanning close surrounding)

Connections:
 SRF05 -> Arduino Mega 2560
 Vcc - 5V 
 GND - GND
 TRIG - 11
 ECHO - 12 
 
 Servo -> Arduino Mega 2560
 RED - 5V
 BROWN - GND
 ORANGE - 10
*/
#include <Servo.h>  // servo library, note that this library disables PWM on pins 9 and 10!
#define TRIG 11 // the SRF05 Trig pin
#define ECHO 12 // the SRF05 Echo pin
unsigned long pulseTime;
Servo servo1;  // servo control object

void setup(){
  pinMode(TRIG,OUTPUT);
  pinMode(ECHO,INPUT);
  servo1.attach(10);
  Serial.begin(9600);
}// void setup()

float measurement(){
  digitalWrite(TRIG,HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG,LOW);
  // wait for the pulse to return. The pulse
  // goes from low to HIGH to low, so we specify
  // that we want a HIGH-going pulse below:
  pulseTime = pulseIn(ECHO,HIGH);
  return pulseTime / 58.00;
}// float measurement()

void loop(){
  int position;
  servo1.write(0);     // Tell servo to go to 0 degrees
  delay(1000);         // Pause to get it time to move
  for(position = 0; position < 180; position += 5){ // Tell servo to go to 180 degrees, stepping by 5 degrees
    servo1.write(position);  // Move to next position
    delay(30);               // Short pause to allow it to move, min 20ms
    Serial.println("Angle: "+String(position)+ ", Distance: "+String(measurement())+" cm");
  }
  delay(10000); // wait 10s
  for(position = 180; position >= 0; position -= 5){                                
    servo1.write(position);  // Move to next position
    delay(30);               // Short pause to allow it to move, min 20ms
    Serial.println("Angle: "+String(position)+ ", Distance: "+String(measurement())+" cm");
  }
  delay(10000);
}// void loop()
