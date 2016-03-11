/*
RobotKit_MonsterMotoShield_4WD_3IR_FollowMe - Follow the moving object (based on data from 3 IR proximity sensors GP2Y) 
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

/*MonsterMoto Shield Carrier*/
const int MotorRight_INA0 = 7; // Pin to control the clockwise direction of Right Motor
const int MotorRight_INB0 = 8; // Pin to control the counterclockwise direction of Right Motor 
const int MotorLeft_INA1 = 4; // Pin to control the clockwise direction of Left Motor
const int MotorLeft_INB1 = 9;// Pin to control the counterclockwise direction of Left Motor
// SDA = 2
// SCL = 3
const int pwm_R = 5; // pwm output - motor A
const int pwm_L = 6; // pwm output - motor B
//const int cs_R = 2; // Current sense ANALOG input R
//const int cs_L =3;  // Current sense ANALOG input L
long pwmLvalue = 255;
long pwmRvalue = 255;
byte pwmChannel;
// IR Sensors
int IRsensorFront = A7;
int IRsensorRight = A8;
int IRsensorLeft = A9;
int distanceIRsensorFront;
int distanceIRsensorRight;
int distanceIRsensorLeft;

int robotControlState;
int last_mspeed;
int turnCounter;
int noDetectionCounter;

void setup(){ 
  //Serial.begin(9600);
  // IR SENSORS
  pinMode(IRsensorFront, INPUT);  // declare Front IR Sensor as input
  pinMode(IRsensorRight, INPUT);  // declare Front IR Sensor as input
  pinMode(IRsensorLeft, INPUT);  // declare Front IR Sensor as input

  //Setup Channel A - Drive Motor
  pinMode(MotorRight_INA0, OUTPUT); //Initiates Motor Channel A1 pin
  pinMode(MotorRight_INB0, OUTPUT); //Initiates Motor Channel A2 pin

  //Setup Channel B - Steering Motor
  pinMode(MotorLeft_INA1, OUTPUT); //Initiates Motor Channel B1 pin
  pinMode(MotorLeft_INB1, OUTPUT); //Initiates Motor Channel B2 pin
  
  //Setup PWM pins as Outputs
  pinMode(pwm_R, OUTPUT);
  pinMode(pwm_L, OUTPUT);
  
  robotControlState = 0;
  turnCounter = 0;
  noDetectionCounter = 0;
  stop_Robot();
}// void setup()

void loop(){  
  //readIRsensors();
  readIRsensors(); //getAverageDistance();
  if(distanceIRsensorFront>410){
    //Serial.println("distanceIRsensorFront: "+String(distanceIRsensorFront)); 
    go_Backwad(60);
  }
  else if((distanceIRsensorFront>=350 && distanceIRsensorFront<=410)){ // || distanceIRsensorFront<150)
    //Serial.println("FRONT: "+String(distanceIRsensorFront)); 
    stop_Robot();
    delay(10);
  }
  else if(distanceIRsensorFront>=230 && distanceIRsensorFront<350){
    //Serial.println("FRONT: "+String(distanceIRsensorFront)); 
    go_Forward(50);
    resetTurningCounter();
  }
  else if(distanceIRsensorFront>=150 && distanceIRsensorFront<230){
    //Serial.println("FRONT: "+String(distanceIRsensorFront)); 
    go_Forward(220);
    resetTurningCounter();
  }
  else if(distanceIRsensorRight>110) {
    if(turnCounter<10 ){     
      turn_Right(200);
      turnCounter++;
    }
    else{
      stop_Robot();
    }
  }
  else if(distanceIRsensorLeft>110) {
    if(turnCounter<10 ){     
      turn_Left(200);
      turnCounter++;
    }
    else{
      stop_Robot();
    }
  }
  else{
    if(noDetectionCounter<30){
      noDetectionCounter++;
    }
    else{
      stop_Robot();
    }
  }
  delay(100);
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
  if(turnCounter!=0){
    turnCounter = 0;
  }
}// void resetTurningCounter()

void resetNoDetectionCounter(){
  if(noDetectionCounter!=0){
    noDetectionCounter = 0;
  }
}// void resetTurningCounter()

void readIRsensors(){
  distanceIRsensorFront = analogRead(IRsensorFront);
  distanceIRsensorRight = analogRead(IRsensorRight);
  distanceIRsensorLeft = analogRead(IRsensorLeft);
}// void readIRsensors()

void go_Robot(){
  digitalWrite(MotorRight_INA0, LOW);  
  digitalWrite(MotorRight_INB0, HIGH); 
  digitalWrite(MotorLeft_INA1, LOW);  
  digitalWrite(MotorLeft_INB1, HIGH); 
  analogWrite(pwm_R, pwmRvalue);
  analogWrite(pwm_L, pwmLvalue);
}// void go_Robot()

void go_Forward(int mspeed){ // robotControlState = 3
  if(robotControlState!=3 || last_mspeed!=mspeed){
    digitalWrite(MotorRight_INA0, HIGH); 
    digitalWrite(MotorRight_INB0, LOW); 
    digitalWrite(MotorLeft_INA1, HIGH);  
    digitalWrite(MotorLeft_INB1, LOW); 
    analogWrite(pwm_R, mspeed); //Spins the motor on Channel A at mspeed
    analogWrite(pwm_L, mspeed);
    robotControlState=3;
    last_mspeed=mspeed;
  }
}// void goForward(int mspeed, int time_ms)

void go_Backwad(int mspeed){
  if(robotControlState!=4){
    digitalWrite(MotorRight_INA0, LOW); 
    digitalWrite(MotorRight_INB0, HIGH); 
    digitalWrite(MotorLeft_INA1, LOW);  
    digitalWrite(MotorLeft_INB1, HIGH); 
    analogWrite(pwm_R, mspeed);//Spins the motor on Channel A at mspeed
    analogWrite(pwm_L, mspeed);
    robotControlState=4;
  }
}// void goBackwad(int mspeed, int time_ms)

void turn_Right(int mspeed){ // robotControlState = 1
  if(robotControlState!=1){
    //Motor B mspeed full speed
    digitalWrite(MotorLeft_INA1, LOW); 
    digitalWrite(MotorLeft_INB1, HIGH);   
    //Motor A forward mspeed full speed
    digitalWrite(MotorRight_INA0, HIGH); 
    digitalWrite(MotorRight_INB0, LOW); 
    analogWrite(pwm_L, mspeed);
    analogWrite(pwm_R, mspeed);
    robotControlState=1;
  }
}// void goBackwad(int mspeed, int time_ms)

void move_RightForward(int mspeed){
  digitalWrite(MotorRight_INA0, LOW);  
  digitalWrite(MotorRight_INB0, HIGH); 
  digitalWrite(MotorLeft_INA1, LOW);  
  digitalWrite(MotorLeft_INB1, HIGH); 
  analogWrite(pwm_R, mspeed*0.4);
  analogWrite(pwm_L, mspeed);
}// void move_RightForward(int mspeed)

void move_LeftForward(int mspeed){
  digitalWrite(MotorRight_INA0, LOW);  
  digitalWrite(MotorRight_INB0, HIGH); 
  digitalWrite(MotorLeft_INA1, LOW);  
  digitalWrite(MotorLeft_INB1, HIGH); 
  analogWrite(pwm_R, mspeed);
  analogWrite(pwm_L, mspeed*0.4);
}// move_LeftForward(int mspeed)

void move_RightBackward(int mspeed){
  digitalWrite(MotorRight_INA0, HIGH); 
  digitalWrite(MotorRight_INB0, LOW); 
  digitalWrite(MotorLeft_INA1, HIGH);  
  digitalWrite(MotorLeft_INB1, LOW); 
  analogWrite(pwm_R, mspeed*0.4);
  analogWrite(pwm_L, mspeed);
}// void move_RightBackward(int mspeed)

void move_LeftBackward(int mspeed){
  digitalWrite(MotorRight_INA0, HIGH); 
  digitalWrite(MotorRight_INB0, LOW); 
  digitalWrite(MotorLeft_INA1, HIGH);  
  digitalWrite(MotorLeft_INB1, LOW); 
  analogWrite(pwm_R, mspeed);
  analogWrite(pwm_L, mspeed*0.4);
}// void move_RightBackward(int mspeed)

void turn_Left(int mspeed){ // robotControlState = 2
  if(robotControlState!=2){
    //Motor B mspeed full speed
    digitalWrite(MotorLeft_INA1, HIGH); 
    digitalWrite(MotorLeft_INB1, LOW);
    //Motor A forward mspeed full speed
    digitalWrite(MotorRight_INA0, LOW); 
    digitalWrite(MotorRight_INB0, HIGH); 
    analogWrite(pwm_L, mspeed);
    analogWrite(pwm_R, mspeed);
    robotControlState=2;
  }
}// void turn_Left(int mspeed)

void goForward(int mspeed, int time_ms){
  //analogWrite(pwm_L, 0);
  digitalWrite(MotorRight_INA0, LOW); 
  digitalWrite(MotorRight_INB0, HIGH); 
  analogWrite(pwm_R, mspeed); //Spins the motor on Channel A at mspeed
  delay(time_ms);
}// void goForward(int mspeed, int time_ms)

void stopRobot(int delay_ms){
  analogWrite(pwm_R, 0);
  analogWrite(pwm_L, 0);
  /*
  digitalWrite(MotorRight_INA0, LOW); 
  digitalWrite(MotorRight_INB0, LOW); 
  digitalWrite(MotorLeft_INA1, LOW);  
  digitalWrite(MotorLeft_INB1, LOW);
  */
  delay(delay_ms);
}// void stopRobot(int delay_ms)

void stop_Robot(){ // robotControlState = 0
  if(robotControlState!=0){
    analogWrite(pwm_R, 0);
    analogWrite(pwm_L, 0);
    robotControlState = 0;
    //Serial.println("************ STOP!");
    resetTurningCounter();
    resetNoDetectionCounter();
  }
}// void stopRobot()

void SetPWM (const long pwm_num, byte pwm_channel){
  if(pwm_channel==1){ // DRIVE MOTOR
    analogWrite(pwm_R, pwm_num);
    pwmRvalue = pwm_num;
  }
  else if(pwm_channel==2){ // STEERING MOTOR
    analogWrite(pwm_L, pwm_num);
    pwmLvalue = pwm_num;
  }
}// void SetPWM (const long pwm_num, byte pwm_channel)



