/*
RobotKit_4WD_IR_GP2Y0A02YK0F_FollowMe - Follow the moving object (based on proximity sensor GP2Y0A02YK0F 20-150 cm) 
- Tested with Arduino Mega 2560
www: http://www.mobilerobots.pl
 
 Connections:
 IR Analog Sensors -> Arduino Mega 2560
 VIN - 5V
 GND - GND
 VOUT - A7
 */
 /*
 ROBOT CONTROL STATES
 0 - stop
 1 - turn_Right
 2 - turn_Left
 3 - go_Forward
 4 - go_Backward
 */
#include <math.h>
#define NUMBER_OF_REPEATED_MEASURES 4

/*MotorDriver_BTS7960*/
const int MotorRight_R_EN = 4; // Pin to control the clockwise direction of Right Motor
const int MotorRight_L_EN = 5; // Pin to control the counterclockwise direction of Right Motor 
const int MotorLeft_R_EN = 8; // Pin to control the clockwise direction of Left Motor
const int MotorLeft_L_EN = 9; // Pin to control the counterclockwise direction of Left Motor
// SDA = 2
// SCL = 3
const int Rpwm1 = 6; // pwm output - motor A
const int Lpwm1 = 7; // pwm output - motor B
const int Rpwm2 = 2; // pwm output - motor A
const int Lpwm2 = 3; // pwm output - motor B
long pwmLvalue = 255;
long pwmRvalue = 255;
byte pwmChannel;

int IRsensorFront = A7;
int distanceIRsensorFront;

int robotControlState;
int last_mspeed;
int turnRightCounter;

void setup(){ 
  // IR SENSORS
  pinMode(IRsensorFront, INPUT);  // declare Front IR Sensor as input

  //Setup Right Motors 
  pinMode(MotorRight_R_EN, OUTPUT); 
  pinMode(MotorRight_L_EN, OUTPUT); 

   //Setup Left Motors 
  pinMode(MotorLeft_R_EN, OUTPUT); 
  pinMode(MotorLeft_L_EN, OUTPUT); 
  
  //Setup PWM pins as Outputs
  pinMode(Rpwm1, OUTPUT);
  pinMode(Lpwm1, OUTPUT);
  pinMode(Rpwm2, OUTPUT);
  pinMode(Lpwm2, OUTPUT);
  
  robotControlState = 0;
  turnRightCounter = 0;
  stop_Robot();
}// void setup()

void loop(){  
  //readIRsensors();
  distanceIRsensorFront = readIRsensor(); //getAverageDistance();
  if(distanceIRsensorFront>550){
    go_Backwad(100);
  }
  else if((distanceIRsensorFront>=430 && distanceIRsensorFront<=550)){ // || distanceIRsensorFront<150)
    stop_Robot();
    delay(100);
  }
  else if(distanceIRsensorFront>=300 && distanceIRsensorFront<430){
    go_Forward(50);
    //resetTurningCounter();
  }
  else if(distanceIRsensorFront>=150 && distanceIRsensorFront<300){
    go_Forward(120);
    resetTurningCounter();
  }
  else if(distanceIRsensorFront<150 ){
    if(turnRightCounter<10 ){     
      turn_Right(120);
      turnRightCounter++;
    }
    else{
      stop_Robot();
    }
  }
  delay(50);
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
void resetTurningCounter(){
  if(turnRightCounter!=0){
    turnRightCounter = 0;
  }
}// void resetTurningCounter()

int readIRsensor(){
  distanceIRsensorFront = analogRead(IRsensorFront);
  return distanceIRsensorFront;
}// void readIRsensors()

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
}// void goForward(int mspeed, int time_ms)

void go_Backwad(int mspeed){
  if(robotControlState!=4){
    SetMotors(1);
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, mspeed);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm2, mspeed);
    robotControlState=4;
  }
}// void goBackwad(int mspeed, int time_ms)

void turn_Right(int mspeed){ // robotControlState = 1
  if(robotControlState!=1){
    SetMotors(1);
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, mspeed);
    analogWrite(Rpwm2, mspeed);
    analogWrite(Lpwm2, 0);
    robotControlState=1;
  }
}// void goBackwad(int mspeed, int time_ms)

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



