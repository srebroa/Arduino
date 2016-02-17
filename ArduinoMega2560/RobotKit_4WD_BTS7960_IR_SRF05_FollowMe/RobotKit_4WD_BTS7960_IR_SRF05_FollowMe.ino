/*
RobotKit_4WD_BTS7960_IR_SRF05_FollowMe - Follow the moving object (based on proximity sensor GP2Y0A02YK0F 20-150 cm and ultrasonic sensor SRF05) 
- Tested with Arduino Mega 2560
www: http://www.mobilerobots.pl
 
 Connections:
 BTS7960 -> Arduino Mega 2560
 MotorRight_R_EN - 4
 MotorRight_L_EN - 5
 MotorLeft_R_EN - 8
 MotorLeft_L_EN - 9
 Rpwm1 - 6
 Lpwm1 - 7
 Rpwm2 - 2
 Lpwm2 - 3
 
 SRF05 Ultrasonic Sensor -> Arduino Mega 2560
 Vcc - 5V 
 GND - GND
 TRIG - 11
 ECHO - 12 
 
 IR GP2Y0A02YK0F Analog Sensor -> Arduino Mega 2560
 VIN - 5V
 GND - GND
 VOUT - A7
 
 Servo -> Arduino Mega 2560
 RED - 5V
 BROWN - GND
 ORANGE - 10
   
 ROBOT CONTROL STATES:
 0 - stop_Robot
 1 - turn_Right
 2 - turn_Left
 3 - go_Forward
 4 - go_Backward
 */
#include <math.h>
#include <Servo.h>  // servo library, note that this library disables PWM on pins 9 and 10!

/*BTS7960 Motor Driver Carrier*/
const int MotorRight_R_EN = 4; 
const int MotorRight_L_EN = 5; 
const int MotorLeft_R_EN = 8;
const int MotorLeft_L_EN = 9;
const int Rpwm1 = 6; 
const int Lpwm1 = 7;
const int Rpwm2 = 2; 
const int Lpwm2 = 3; 
long pwmLvalue = 255;
long pwmRvalue = 255;
byte pwmChannel;

/*SRF05 Ultrasonic Sensor*/
#define TRIG 11 // the SRF05 Trig pin
#define ECHO 12 // the SRF05 Echo pin
#define SENSOR_STEP 10 // 5 degrees
int numberOfSteps = int(180 / SENSOR_STEP);
unsigned long pulseTime;
unsigned long srfDistanceArray[180 / SENSOR_STEP]; // 180/step (where step is equal to 5)
unsigned long minimumDistanceThreshold = 5;
unsigned long maximumDistanceThreshold = 70;

/*IR */
int IRsensorFront = A7;
int distanceIRsensorFront;

/*Servo for rotating SRF05 sensor*/
#define SERVO_PIN 10
Servo servo1;  // servo control object
int servoPosition;

int nearestObstaclePosition;
unsigned long nearestObstacleDistance;
int robotControlState;
int last_mspeed;
int robotTurningCounter;

void setup(){  
  //Serial.begin(9600);
  // IR SENSOR
  pinMode(IRsensorFront, INPUT);  // declare Front IR Sensor as input
  
  //SRF05 Ultrasonic Sensor
  pinMode(TRIG,OUTPUT);
  pinMode(ECHO,INPUT);
    
  //Servo
  servo1.attach(SERVO_PIN);

  //Setup Channel A - Drive Motor
  pinMode(MotorRight_R_EN, OUTPUT); //Initiates Motor Channel A1 pin
  pinMode(MotorRight_L_EN, OUTPUT); //Initiates Motor Channel A2 pin

  //Setup Channel B - Steering Motor
  pinMode(MotorLeft_R_EN, OUTPUT); //Initiates Motor Channel B1 pin
  pinMode(MotorLeft_L_EN, OUTPUT); //Initiates Motor Channel B2 pin
  
  //Setup PWM pins as Outputs
  pinMode(Rpwm1, OUTPUT);
  pinMode(Lpwm1, OUTPUT);
  pinMode(Rpwm2, OUTPUT);
  pinMode(Lpwm2, OUTPUT);
  
  stop_Robot();
  robotControlState = 0;
  robotTurningCounter = 0;
  servoPosition = 0;
  servo1.write(servoPosition);     // Tell servo to go to 0 degrees
  delay(1000);         // Pause to get it time to move
}// void setup()

void loop(){
  distanceIRsensorFront = readIRsensor(); //getAverageDistance();
  if(distanceIRsensorFront>550){
    go_Backwad(100);
  }
  else if((distanceIRsensorFront>=430 && distanceIRsensorFront<=550)){ // || distanceIRsensorFront<150)
    stop_Robot();
    delay(50);
  }
  else if(distanceIRsensorFront>=300 && distanceIRsensorFront<430){
    go_Forward(50);
    resetTurningCounter();
  }
  else if(distanceIRsensorFront>=150 && distanceIRsensorFront<300){
    servo1.write(90); 
    go_Forward(120);
    resetTurningCounter();
  }
  else if(distanceIRsensorFront<150 && robotControlState!=1 && robotControlState!=2){
    stop_Robot();
    if(FindTheNearestObstacle()!=0){
      turnRobotToObstaclePosition(nearestObstaclePosition);
    }
  }
  delay(40);
}// void loop()

void resetTurningCounter(){
  robotTurningCounter = 0;
}

void turnRobotToObstaclePosition(int obstaclePosition){
  if(robotTurningCounter<10 ){
    if(obstaclePosition<90){
      turn_Right(120);    
    }
    else {  //if(obstaclePosition>90)
      turn_Left(120);
    }
    robotTurningCounter++;
  }
  else{
    stop_Robot();
    delay(100);
  }     
}// turnRobotToObstaclePosition(int obstaclePosition)

int readIRsensor(){
  distanceIRsensorFront = analogRead(IRsensorFront);
  return distanceIRsensorFront;
}// void readIRsensors()

void updateSRFdistance(){
 nearestObstacleDistance = measurement();
}

int FindTheNearestObstacle(){
  int position;
  int arrayIndex = 0;
  servo1.write(0);     // Tell servo to go to 0 degrees
  delay(1000);         // Pause to get it time to move

  for(position = 0; position <= 180; position += SENSOR_STEP){ // Tell servo to go to 180 degrees, stepping by 5 degrees
    servo1.write(position);  // Move to next position
    delay(31);               // Short pause to allow it to move, min 20ms
    srfDistanceArray[arrayIndex] = measurement();
    //Serial.println("Angle: "+String(position)+ ", Distance: "+String(srfDistanceArray[arrayIndex])+" cm");
    arrayIndex++;    
  }// for
  nearestObstaclePosition = measurementsDataMin();
  //Serial.println("nearestObstaclePosition: "+String(nearestObstaclePosition)+", "+String(nearestObstacleDistance));
  if(nearestObstacleDistance<maximumDistanceThreshold){
    servo1.write(nearestObstaclePosition);     // Tell servo to go to nearest obstacle position
    delay(900);         // Pause to get it time to move 
    return nearestObstaclePosition;
  }
  return 0;
}// FindTheNearestObstacle()

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
  nearestObstacleDistance = minDistance;
  index = index*SENSOR_STEP;
  return index;
}// int measurementsDataMin()

void go_Robot(){
  SetMotors(1);
  analogWrite(Rpwm1, pwmRvalue);
  analogWrite(Lpwm1, 0);
  analogWrite(Rpwm2, pwmRvalue);
  analogWrite(Lpwm2, 0);
}// void go_Robot()

void go_Forward(int mspeed){ // robotControlState = 3
  if(robotControlState!=3 || last_mspeed!=mspeed){
    SetMotors(1);
    analogWrite(Rpwm1, mspeed);
    analogWrite(Lpwm1, 0);
    analogWrite(Rpwm2, mspeed);
    analogWrite(Lpwm2, 0);
    robotControlState=3;
    last_mspeed=mspeed;
  }
}// void goForward(int mspeed)

void go_Backwad(int mspeed){
  if(robotControlState!=4){
    SetMotors(1);
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, mspeed);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm2, mspeed);
    robotControlState=4;
  }
}// void goBackwad(int mspeed)

void turn_Right(int mspeed){ // robotControlState = 1
  if(robotControlState!=1){
    SetMotors(1);
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, mspeed);
    analogWrite(Rpwm2, mspeed);
    analogWrite(Lpwm2, 0);
    robotControlState=1;
  }
}// void turn_Right(int mspeed)

void move_RightForward(int mspeed){
  SetMotors(1);
  analogWrite(Rpwm1, mspeed*0.4);
  analogWrite(Lpwm1, 0);
  analogWrite(Rpwm2, mspeed);
  analogWrite(Lpwm2, 0);
}// void move_RightForward(int mspeed)

void move_LeftForward(int mspeed){
  SetMotors(1);
  analogWrite(Rpwm1, mspeed);
  analogWrite(Lpwm1, 0);
  analogWrite(Rpwm2, mspeed*0.4);
  analogWrite(Lpwm2, 0);
}// move_LeftForward(int mspeed)

void move_RightBackward(int mspeed){
  SetMotors(1);
  analogWrite(Rpwm1, 0);
  analogWrite(Lpwm1, mspeed*0.4);
  analogWrite(Rpwm2, 0);
  analogWrite(Lpwm2, mspeed);
}// void move_RightBackward(int mspeed)

void move_LeftBackward(int mspeed){
  SetMotors(1);
  analogWrite(Rpwm1, 0);
  analogWrite(Lpwm1, mspeed);
  analogWrite(Rpwm2, 0);
  analogWrite(Lpwm2, mspeed*0.4);
}// void move_RightBackward(int mspeed)

void turn_Left(int mspeed){ // robotControlState = 2
  if(robotControlState!=2){
    SetMotors(1);
    analogWrite(Rpwm1, mspeed);
    analogWrite(Lpwm1, 0);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm2, mspeed);
    robotControlState=2;
  }
}// void turn_Left(int mspeed)

void goForward(int mspeed, int time_ms){
  SetMotors(1);
  analogWrite(Rpwm1, mspeed);
  analogWrite(Lpwm1, 0);
  analogWrite(Rpwm2, mspeed);
  analogWrite(Lpwm2, 0);
  delay(time_ms);
}// void goForward(int mspeed, int time_ms)

void stopRobot(int delay_ms){
  SetMotors(2);
  analogWrite(Rpwm1, 0);
  analogWrite(Lpwm1, 0);
  analogWrite(Rpwm2, 0);
  analogWrite(Lpwm2, 0);
  delay(delay_ms);
}// void stopRobot(int delay_ms)

void stop_Robot(){ // robotControlState = 0
  if(robotControlState!=0){
    SetMotors(2); 
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, 0);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm2, 0);
    robotControlState = 0;
  }
}// void stopRobot()

void SetPWM(const long pwm_num, byte pwm_channel){
  if(pwm_channel==1){ // DRIVE MOTOR
    analogWrite(Rpwm1, pwm_num);
    pwmRvalue = pwm_num;
  }
  else if(pwm_channel==2){ // STEERING MOTOR
    analogWrite(Lpwm1, pwm_num);
    pwmLvalue = pwm_num;
  }
}// void SetPWM (const long pwm_num, byte pwm_channel)  

void SetMotors(int controlCase){
  switch(controlCase){
    case 1:
      digitalWrite(MotorRight_R_EN, HIGH);  
      digitalWrite(MotorRight_L_EN, HIGH); 
      digitalWrite(MotorLeft_R_EN, HIGH);  
      digitalWrite(MotorLeft_L_EN, HIGH); 
    break;
    case 2:
      digitalWrite(MotorRight_R_EN, LOW);  
      digitalWrite(MotorRight_L_EN, LOW); 
      digitalWrite(MotorLeft_R_EN, LOW);  
      digitalWrite(MotorLeft_L_EN, LOW); 
    break;
  } 
}// void SetMotors(int controlCase)



