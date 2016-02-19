/*
RobotKit_BTS7960_4WD_Bluetooth_Controlled - Allows you to control mobile robot via Bluetooth using Android app) 
- Tested with Arduino Mega 2560
Android app: https://play.google.com/store/apps/details?id=pl.mobilerobots.vacuumcleanerrobot&hl=pl
www: http://www.mobilerobots.pl
 
 Connections:
 Bluetooth (e.g HC-05)-> Arduino Mega 2560
 TXD - RX1 (19)
 RXD - TX1 (18)
 VCC - 5V
 GND - GND
 
 BTS7960 -> Arduino Mega 2560
 MotorRight_R_EN - 4
 MotorRight_L_EN - 5
 MotorLeft_R_EN - 8
 MotorLeft_L_EN - 9
 Rpwm1 - 6
 Lpwm1 - 7
 Rpwm2 - 2
 Lpwm2 - 3
 
 ROBOT CONTROL STATES:
 0 - stop_Robot
 1 - turn_Right
 2 - turn_Left
 3 - go_Forward
 4 - go_Backward
 5 - move_RightForward
 6 - move_LeftForward
 7 - move_RightBackward
 8 - move_LeftBackward
 */
#include <math.h>

/*BTS7960 Motor Driver Carrier*/
const int MotorRight_R_EN = 4; // Pin to control the clockwise direction of Right Motor
const int MotorRight_L_EN = 5; // Pin to control the counterclockwise direction of Right Motor 
const int MotorLeft_R_EN = 8; // Pin to control the clockwise direction of Left Motor
const int MotorLeft_L_EN = 9; // Pin to control the counterclockwise direction of Left Motor
const int Rpwm1 = 6; // pwm output - motor A
const int Lpwm1 = 7; // pwm output - motor B
const int Rpwm2 = 2; // pwm output - motor A
const int Lpwm2 = 3; // pwm output - motor B
long pwmLvalue = 255;
long pwmRvalue = 255;
byte pwmChannel;

const char startOfNumberDelimiter = '<';
const char endOfNumberDelimiter = '>';
int robotControlState;
int last_mspeed;

void setup(){
  /*
  For Arduino Mega 2560
  Serial1 RX - pin 19
  Serial1 TX - pin 18
  */
  Serial1.begin(9600);//Default Bluetooth Baudrate for HC-05

  //Setup Right Motors
  pinMode(MotorRight_R_EN, OUTPUT); //Initiates Motor Channel A1 pin
  pinMode(MotorRight_L_EN, OUTPUT); //Initiates Motor Channel A2 pin

  //Setup Left Motors
  pinMode(MotorLeft_R_EN, OUTPUT); //Initiates Motor Channel B1 pin
  pinMode(MotorLeft_L_EN, OUTPUT); //Initiates Motor Channel B2 pin
  
  //Setup PWM pins as Outputs
  pinMode(Rpwm1, OUTPUT);
  pinMode(Lpwm1, OUTPUT);
  pinMode(Rpwm2, OUTPUT);
  pinMode(Lpwm2, OUTPUT);
  
  stop_Robot();
}// void setup()

void loop(){
  //int i = 0;  
  if (Serial1.available()) {
    processInput();
  }
}// void loop()

void processInput (){
  static long receivedNumber = 0;
  static boolean negative = false;
  byte c = Serial1.read ();

  switch (c){
  case endOfNumberDelimiter:
    if (negative)
      SetPWM(- receivedNumber, pwmChannel);
    else
      SetPWM(receivedNumber, pwmChannel);

    // fall through to start a new number
  case startOfNumberDelimiter:
    receivedNumber = 0;
    negative = false;
    pwmChannel = 0;
    break;

  case 'f': // Go FORWARD
    go_Forward(255);
    //Serial.println("forward");
    break;

  case 'b': // Go BACK
    go_Backwad(255);
    break;

  case 'r':
    turn_Right(255);
    break;

  case 'l':
    turn_Left(255);
    break;

  case 'c': // Top Right
    move_RightForward(255);
    break; 

  case 'd': // Top Left
    move_LeftForward(255);
    break;  

  case 'e': // Bottom Right
    move_RightBackward(255);
    break; 

  case 'h': // Bottom Left
    move_LeftBackward(255);
    break;  

  case 's':
    stop_Robot();
    break;

  case 'x':
    pwmChannel = 1; // Rpwm1
    break;
  case 'y': // Lpwm1
    pwmChannel = 2;
    break;

  case '0' ... '9':
    receivedNumber *= 10;
    receivedNumber += c - '0';
    break;

  case '-':
    negative = true;
    break;
  } // end of switch
} // void processInput ()

void stop_Robot(){ // robotControlState = 0
  if(robotControlState!=0){
    //SetMotors(2); 
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, 0);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm2, 0);
    robotControlState = 0;
  }
}// void stopRobot()

void turn_Right(int mspeed){ // robotControlState = 1
  if(robotControlState!=1 || last_mspeed!=mspeed){
    SetMotors(1);
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, mspeed);
    analogWrite(Rpwm2, mspeed);
    analogWrite(Lpwm2, 0);
    robotControlState=1;
    last_mspeed=mspeed;
  }
}// void turn_Right(int mspeed)

void turn_Left(int mspeed){ // robotControlState = 2
  if(robotControlState!=2 || last_mspeed!=mspeed){
    SetMotors(1);
    analogWrite(Rpwm1, mspeed);
    analogWrite(Lpwm1, 0);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm2, mspeed);
    robotControlState=2;
    last_mspeed=mspeed;
  }
}// void turn_Left(int mspeed)

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

void go_Backwad(int mspeed){ // robotControlState = 4
  if(robotControlState!=4 || last_mspeed!=mspeed){
    SetMotors(1);
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, mspeed);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm2, mspeed);
    robotControlState=4;
    last_mspeed=mspeed;
  }
}// void goBackwad(int mspeed)

void move_RightForward(int mspeed){ // robotControlState = 5
  if(robotControlState!=5 || last_mspeed!=mspeed){
    SetMotors(1);
    analogWrite(Rpwm1, mspeed*0.4);
    analogWrite(Lpwm1, 0);
    analogWrite(Rpwm2, mspeed);
    analogWrite(Lpwm2, 0);
    robotControlState=5;
    last_mspeed=mspeed;
  }
}// void move_RightForward(int mspeed)

void move_LeftForward(int mspeed){ // robotControlState = 6
  if(robotControlState!=6 || last_mspeed!=mspeed){
    SetMotors(1);
    analogWrite(Rpwm1, mspeed);
    analogWrite(Lpwm1, 0);
    analogWrite(Rpwm2, mspeed*0.4);
    analogWrite(Lpwm2, 0);
    robotControlState=6;
    last_mspeed=mspeed;  
  }
}// move_LeftForward(int mspeed)

void move_RightBackward(int mspeed){ // robotControlState = 7
  if(robotControlState!=7 || last_mspeed!=mspeed){
    SetMotors(1);
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, mspeed*0.4);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm2, mspeed);
    robotControlState=7;
    last_mspeed=mspeed;  
  }
}// void move_RightBackward(int mspeed)

void move_LeftBackward(int mspeed){ // robotControlState = 8
  if(robotControlState!=8 || last_mspeed!=mspeed){
    SetMotors(1);
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, mspeed);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm2, mspeed*0.4);
    robotControlState=8; 
    last_mspeed=mspeed; 
  }
}// void move_LeftBackward(int mspeed)

void stopRobot(int delay_ms){
  SetMotors(2);
  analogWrite(Rpwm1, 0);
  analogWrite(Lpwm1, 0);
  analogWrite(Rpwm2, 0);
  analogWrite(Lpwm2, 0);
  delay(delay_ms);
}// void stopRobot(int delay_ms)

void SetPWM(const long pwm_num, byte pwm_channel){
  if(pwm_channel==1){ // DRIVE MOTOR
    analogWrite(Rpwm1, 0);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm1, pwm_num);
    analogWrite(Lpwm2, pwm_num);
    pwmRvalue = pwm_num;
  }
  else if(pwm_channel==2){ // STEERING MOTOR
    analogWrite(Lpwm1, 0);
    analogWrite(Lpwm2, 0);
    analogWrite(Rpwm1, pwm_num);
    analogWrite(Rpwm2, pwm_num);
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


