/*
SRF05_servo_nearest_obstacle - Find the nearest obstacle and turn the sensor in its direction 

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
#define SENSOR_STEP 10 // 5 degrees
#define NUMBER_OF_REPEATED_MEASURES 3
int numberOfSteps = int(180 / SENSOR_STEP);
unsigned long pulseTime;
unsigned long srfDistanceArray[180 / SENSOR_STEP]; // 180/step (where step is equal to 5)
unsigned long minimumDistanceThreshold = 5;
unsigned long maximumDistanceThreshold = 100;
//unsigned long SRFmeasurementsArray[NUMBER_OF_REPEATED_MEASURES];
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

boolean checkNeighborhood(int arrayIndex){
  unsigned long loweNeighborDiff = abs(srfDistanceArray[arrayIndex]-srfDistanceArray[arrayIndex-1]);
  unsigned long upperNeighborDiff = abs(srfDistanceArray[arrayIndex]-srfDistanceArray[arrayIndex+1]);
  if (loweNeighborDiff<20 || upperNeighborDiff<20){
    return true;
  }
  else{
    return false;
  }
}// boolean checkNeighborhood(int arrayIndex)

int measurementsDataMin(){
  unsigned long minDistance = maximumDistanceThreshold; // srfDistanceArray[0];
  int index = 0;
  for (int i=1; i<numberOfSteps-2; i++){
    if(srfDistanceArray[i]>minimumDistanceThreshold && srfDistanceArray[i] < minDistance){
      if(checkNeighborhood(i)){ // if TRUE
        index = i;
        minDistance = srfDistanceArray[i];
      }
    }  
  }// for
  index = index*SENSOR_STEP;
  return index;
}// int measurementsDataMin()

/*
unsigned long getAverageDistance(){
  unsigned long averageDistance;
  unsigned long currentMeasurement;
  unsigned long sumOfDistances = 0;
  unsigned int numberOfValidMeasurements = 0;
  for(int i=0; i<NUMBER_OF_REPEATED_MEASURES-1; i++){
       currentMeasurement = measurement();
       delay(10);
       if(currentMeasurement>5){
         sumOfDistances = sumOfDistances+currentMeasurement;
         numberOfValidMeasurements++;
       }
  }// for
  averageDistance = sumOfDistances/numberOfValidMeasurements;
  return averageDistance;
}
*/

void loop(){
  int position;
  int nearestObstaclePosition;
  int arrayIndex = 0;
  servo1.write(0);     // Tell servo to go to 0 degrees
  delay(1000);         // Pause to get it time to move

  for(position = 0; position < 180; position += SENSOR_STEP){ // Tell servo to go to 180 degrees, stepping by 5 degrees
    servo1.write(position);  // Move to next position
    delay(31);               // Short pause to allow it to move, min 20ms
    if(arrayIndex<numberOfSteps-1){
       srfDistanceArray[arrayIndex] = measurement(); //getAverageDistance();
       //arrayIndex++;
    }
    Serial.println("Angle: "+String(position)+ ", Distance: "+String(srfDistanceArray[arrayIndex])+" cm");
    arrayIndex++;
  }
  nearestObstaclePosition = measurementsDataMin();
  servo1.write(nearestObstaclePosition);     // Tell servo to go to nearest obstacle position
  Serial.println("nearestObstaclePosition: "+String(nearestObstaclePosition));
  delay(5000);         // Pause to get it time to move 
}// void loop()
