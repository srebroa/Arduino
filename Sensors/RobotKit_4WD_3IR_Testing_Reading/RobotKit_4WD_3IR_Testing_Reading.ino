/*
RobotKit_4WD_3IR_Testing_Reading - Read data from 3 IR sensors):
- Front: GP2Y0A02YK0F (20-150 cm)
- Right and Left: GP2Y0A21YK (10-80 cm)
- Tested with Arduino Mega 2560
www: http://www.mobilerobots.pl
 
 Connections:
 IR Analog Sensors -> Arduino Mega 2560
 VIN - 5V
 GND - GND
 VOUT - A7, A8, A9
 */
#include <math.h>
//#define NUMBER_OF_REPEATED_MEASURES 4

// IR Sensors
int IRsensorFront = A7;
int IRsensorRight = A8;
int IRsensorLeft = A9;
int distanceIRsensorFront;
int distanceIRsensorRight;
int distanceIRsensorLeft;

void setup(){ 
  Serial.begin(9600);
  // IR SENSORS
  pinMode(IRsensorFront, INPUT);  // declare Front IR Sensor as input
  pinMode(IRsensorRight, INPUT);  // declare Right IR Sensor as input
  pinMode(IRsensorLeft, INPUT);  // declare Left IR Sensor as input
}// void setup()

void loop(){  
  readIRsensors(); //getAverageDistance();
  delay(1000);
}// void loop()

/*
int getAverageDistance(){
  int averageDistance;
  int currentMeasurement;
  int sumOfDistances = 0;
  //unsigned int numberOfValidMeasurements = 0;
  for(int i=0; i<NUMBER_OF_REPEATED_MEASURES-1; i++){
       currentMeasurement = readIRsensor();
       sumOfDistances = sumOfDistances+currentMeasurement;
       delay(35);
  }// for
  averageDistance = (int)(sumOfDistances/NUMBER_OF_REPEATED_MEASURES);
  return averageDistance;
}
*/

void readIRsensors(){
  distanceIRsensorFront = analogRead(IRsensorFront);
  distanceIRsensorRight = analogRead(IRsensorRight);
  distanceIRsensorLeft = analogRead(IRsensorLeft);
  Serial.println("Front: "+String(distanceIRsensorFront)+" Right: "+String(distanceIRsensorRight)+" Left: "+String(distanceIRsensorLeft)); 
}// void readIRsensors()

