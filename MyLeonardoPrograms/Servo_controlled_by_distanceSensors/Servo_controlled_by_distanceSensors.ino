/*********************************************************************************************
Servo_controlled_by_distanceSensors 

This example code show how to control Pan/Tilt mechanism using analog distance sensors (Sharp GP2Y0A21YK)  
See how it works at: https://www.youtube.com/watch?v=iB_XI3Zix28
Visit: http://www.mobilerobots.pl/index.php?p=1_85

Code developed by Adam Srebro (17.03.2014) in Arduino 1.0.5, on the Arduino Leonardo board.

 The circuit:
 * Arduino Leonardo -> Analog Distance Sensors (GP2Y0A21YK):
 - A0 -> Sensor Right
 - A1 -> Sensor Front
 - A2 -> Sensor Left
 * Arduino Leonardo -> Digital Sensor:
 - Pin 7 -> Digital Sensor
 * Arduino Leonardo -> Servo:
 - Pin 9 -> Servo Signal Pin ("orange or white" wire)
 - GND   -> Servo "red" wire
 - 5V    -> Servo "brown or black" wire 
**********************************************************************************************/

#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo, a maximum of eight servo objects can be created 
 
int targetServoPosition;    // variable to store the target servo position 
int currentServoPosition;   // variable to store the current servo position 
int detectionThreshold;    

int distanceSensorRight; 
int distanceSensorFront; 
int distanceSensorLeft;

int digitalSensorPin = 7; // digital IR sensor connected to digital pin 7

// Analog IR sensors Pins
int distanceSensorRightPin = 0;
int distanceSensorFrontPin = 1;
int distanceSensorLeftPin = 2;
 
void setup() 
{ 
  targetServoPosition = 0;
  currentServoPosition = 0;
  detectionThreshold = 440; // can be changed as needed ;)
  distanceSensorRight = 0;
  distanceSensorFront = 0;
  distanceSensorLeft = 0;  
  pinMode(digitalSensorPin, INPUT);  // sets the digital pin 7 as input
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
  Serial.begin(9600);
   while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  Serial.println("Servo controlled by distance sensors :)");
  setServoPosition(targetServoPosition);
} 
 
 
void loop() {  
  int detection = readDistanceSensors(); 
  Serial.println("Detection: " +String(detection));
  if(detection==1){ // position of the right sensor
    targetServoPosition = 0;
  }
  else if(detection==2){ // position between the right and front sensor
    targetServoPosition = 45;
  }
  else if(detection==3){ // position of the front sensor
    targetServoPosition = 90;
  }
  else if(detection==4){  // position between the front and left sensor    
    targetServoPosition = 135;
  }
  else if(detection==5){ // position of the left sensor
    targetServoPosition = 180;
  }
  else{
    delay(500); 
  }
  setServoPosition(targetServoPosition);
}// void loop() 

void setServoPosition(int targetPosition){
  if(currentServoPosition==targetPosition){
    // do nothing
  }
  else if(digitalRead(digitalSensorPin)==0){
      if(currentServoPosition<targetPosition){
        for(currentServoPosition; currentServoPosition<targetPosition; currentServoPosition +=1)// in steps of 1 degree   
        {                                 
          myservo.write(currentServoPosition); // tell servo to go to "currentServoPosition" 
          delay(15);                       // waits 15ms for the servo to reach the position 
        } 
      }
      else if(currentServoPosition>targetPosition){
          for(currentServoPosition; currentServoPosition>targetPosition; currentServoPosition -=1)  
          {                                  
            myservo.write(currentServoPosition);              
            delay(15);                      
          } 
      }
  }
  else{
          myservo.write(targetPosition);      // tell servo to go to "targetPosition"
          delay(100);
          currentServoPosition=targetPosition;
  }
  delay(1000);
}// void setServoPosition(int targetPosition)

int readDistanceSensors(){
    int detection = 0;
    distanceSensorRight = analogRead(distanceSensorRightPin);
    distanceSensorFront = analogRead(distanceSensorFrontPin);
    distanceSensorLeft = analogRead(distanceSensorLeftPin);
    
    Serial.println("SensorRight: "+String(distanceSensorRight)+", SensorFront: "+String(distanceSensorFront)+", SensorLeft: "+String(distanceSensorLeft));
    
    if(distanceSensorRight>detectionThreshold){
      if((distanceSensorFront<detectionThreshold)&&(distanceSensorLeft<detectionThreshold)){
        detection = 1;  // position of the right sensor
      }
      else if ((distanceSensorFront>detectionThreshold)&&(distanceSensorLeft<detectionThreshold)){
        detection = 2; // position between the right and front sensor
      }
      else {
        detection = 3; // position of the front sensor
      }     
    }
    else if(distanceSensorFront>detectionThreshold){
      if((distanceSensorRight<detectionThreshold)&&(distanceSensorLeft<detectionThreshold)){
        detection = 3; // position of the front sensor
      }
      else if((distanceSensorRight<detectionThreshold)&&(distanceSensorLeft>detectionThreshold)){
        detection = 4; // position between the front and left sensor       
      }
      else{
        detection = 3; // position of the front sensor
      }
    }
    else if(distanceSensorLeft>detectionThreshold){
      if((distanceSensorRight<detectionThreshold)&&(distanceSensorFront<detectionThreshold)){
        detection = 5; // position of the left sensor
      }
    }   
    return detection;
}// int readDistanceSensors() 
