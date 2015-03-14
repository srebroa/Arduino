/*
RC CAR BLUETOOTH CONTROL v1.0
 - Allows you to control a mobile robot via bluetooth
 - Tested with Arduino Pro Mini
 - Android application - https://play.google.com/store/apps/details?id=pl.mobilerobots.vacuumcleanerrobot&hl=pl
 Author: Adam Srebro
 www: http://www.mobilerobots.pl/
 http://www.mobilerobots.pl/index.php?p=1_91_Monster-Truck-Hack
 
 Connections:
 Bluetooth (e.g HC-06) -> Arduino Pro Mini (3.3V)
 Rx - Tx 
 Tx - Rx 
 
 TB6612FNG Dual Motor Driver -> Arduino Pro Mini
 AIN1 - 4
 AIN2 - 7
 BIN1 - 8
 BIN2 - 9
 PWMA - 5
 PWMB - 6
 STBY - Vcc
 VMOT - motor voltage (4.5 to 13.5 V) - 6V from RC Car battery
 Vcc - logic voltage (2.7 to 5.5) - 3.3V from Arduino Pro Mini (VCC)
 
 TB6612FNG Dual Motor Driver -> DC Motors
 AO1 - drive motor A
 A02 - drive motor A
 B01 - steering motor B
 B02 - steering motor B 
 
 LEDs -> Arduino Pro Mini
 front right led - 2
 front left led - 3
 back right led - 14
 back left led - 15
 */
#include <Wire.h>
#include <math.h>

/*TB6612FNG Dual Motor Driver Carrier*/
const int MotorA_AIN1 = 4; // control Input AIN1 - motor A
const int MotorA_AIN2 = 7; // control Input AIN2 - motor A
const int MotorB_BIN1 = 8; // control Input BIN1 - motor B
const int MotorB_BIN2 = 9; // control Input BIN2 - motor B
//Front LEDs
const int FrontRightLed = 2;
const int FrontLeftLed = 3;
//Back LEDs
const int BackRightLed = 14;
const int BackLeftLed = 15;

const int pwm_A = 5; // pwm output - motor A
const int pwm_B = 6; // pwm output - motor B
long pwmLvalue = 255;
long pwmRvalue = 255;
byte pwmChannel;
const char startOfNumberDelimiter = '<';
const char endOfNumberDelimiter = '>';

void setup(){
  Serial.begin(9600);// HC-06 default baudrate: 9600

  // Front LEDs
  pinMode(FrontRightLed, OUTPUT);  // declare FrontRightLed as input
  pinMode(FrontLeftLed, OUTPUT);   // declare FrontLeftLed as input
  
  // Back LEDs
  pinMode(BackRightLed, OUTPUT);  // declare BackRightLed as input
  pinMode(BackLeftLed, OUTPUT);   // declare BackLeftLed as input

  //Setup Channel A - Drive Motor
  pinMode(MotorA_AIN1, OUTPUT); //Initiates Motor Channel A1 pin
  pinMode(MotorA_AIN2, OUTPUT); //Initiates Motor Channel A2 pin

  //Setup Channel B - Steering Motor
  pinMode(MotorB_BIN1, OUTPUT); //Initiates Motor Channel B1 pin
  pinMode(MotorB_BIN2, OUTPUT); //Initiates Motor Channel B2 pin
  Wire.begin();

  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
}// void setup()

void loop(){
  int i = 0;  
  if (Serial.available()) {
    processInput();
  }
}// void loop()

void processInput (){
  static long receivedNumber = 0;
  static boolean negative = false;
  byte c = Serial.read ();

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

  case 'g': // robot GO!
    go_Robot();
    break;

  case 'f': // Go FORWARD
    go_Forward(255);
    //Serial.println("forward");
    break;

  case 'b': // Go BACK
    go_Backwad(255);
    //Serial.println("backward");
    break;

  case 'r':
    turn_Right(255);
    break;

  case 'l':
    turn_Left(255);
    break;

  case 'c': // Top Right
    turn_Right(255);
    break; 

  case 'd': // Top Left
    turn_Left(255);
    break;  

  case 'e': // Bottom Right
    turn_RightBack(255);
    break; 

  case 'h': // Bottom Left
    turn_LeftBack(255);
    break;  

  case 's':
    stop_Robot();
    break;

  case 'x':
    pwmChannel = 1; // pwm_A
    break;
  case 'y': // pwm_B
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

void go_Robot(){
  digitalWrite(MotorA_AIN1, LOW);  
  digitalWrite(MotorA_AIN2, HIGH); 
  analogWrite(pwm_A, pwmRvalue);
  turnStopLeds(0);
}// void go_Robot()

void go_Forward(int mspeed){
  analogWrite(pwm_B, 0);
  digitalWrite(MotorA_AIN1, LOW); 
  digitalWrite(MotorA_AIN2, HIGH); 
  analogWrite(pwm_A, mspeed); //Spins the motor on Channel A at mspeed
  turnFrontLeds(1);
  turnStopLeds(0);
}// void goForward(int mspeed, int time_ms)

void go_Backwad(int mspeed){
  analogWrite(pwm_B, 0);
  digitalWrite(MotorA_AIN1, HIGH); 
  digitalWrite(MotorA_AIN2, LOW); 
  analogWrite(pwm_A, mspeed);//Spins the motor on Channel A at mspeed
  turnFrontLeds(1);
  turnStopLeds(1);
}// void goBackwad(int mspeed, int time_ms)

void turn_Right(int mspeed){
  //Motor B mspeed full speed
  digitalWrite(MotorB_BIN1, LOW); 
  digitalWrite(MotorB_BIN2, HIGH); 
  analogWrite(pwm_B, mspeed);
  //Motor A forward mspeed full speed
  digitalWrite(MotorA_AIN1, LOW); 
  digitalWrite(MotorA_AIN2, HIGH); 
  analogWrite(pwm_A, mspeed);
  turnFrontLeds(1);
  turnStopLeds(0);
}// void goBackwad(int mspeed, int time_ms)

void turn_RightBack(int mspeed){
  //Motor B mspeed full speed
  digitalWrite(MotorB_BIN1, LOW); 
  digitalWrite(MotorB_BIN2, HIGH); 
  analogWrite(pwm_B, mspeed);
  //Motor A forward mspeed full speed
  digitalWrite(MotorA_AIN1, HIGH); 
  digitalWrite(MotorA_AIN2, LOW); 
  analogWrite(pwm_A, mspeed);
  turnFrontLeds(1);
}// void goBackwad(int mspeed, int time_ms)

void turn_Left(int mspeed){
  //Motor B mspeed full speed
  digitalWrite(MotorB_BIN1, HIGH); 
  digitalWrite(MotorB_BIN2, LOW); 
  analogWrite(pwm_B, mspeed);
  //Motor A forward mspeed full speed
  digitalWrite(MotorA_AIN1, LOW); 
  digitalWrite(MotorA_AIN2, HIGH); 
  analogWrite(pwm_A, mspeed);
  turnFrontLeds(1);
  turnStopLeds(0);
}// void turn_Left(int mspeed)

void turn_LeftBack(int mspeed){
  //Motor B left mspeed full speed
  digitalWrite(MotorB_BIN1, HIGH); 
  digitalWrite(MotorB_BIN2, LOW); 
  analogWrite(pwm_B, mspeed);
  //Motor A forward mspeed full speed
  digitalWrite(MotorA_AIN1, HIGH); 
  digitalWrite(MotorA_AIN2, LOW); 
  analogWrite(pwm_A, mspeed);
  turnFrontLeds(1);
}// turn_LeftBack(int mspeed)

void goForward(int mspeed, int time_ms){
  analogWrite(pwm_B, 0);
  digitalWrite(MotorA_AIN1, LOW); 
  digitalWrite(MotorA_AIN2, HIGH); 
  analogWrite(pwm_A, mspeed); //Spins the motor on Channel A at mspeed
  turnFrontLeds(1);
  turnStopLeds(0);
  delay(time_ms);
}// void goForward(int mspeed, int time_ms)

void stopRobot(int delay_ms){
  //analogWrite(pwm_out, 240);
  analogWrite(pwm_A, 0);
  analogWrite(pwm_B, 0);
  turnStopLeds(1);
  turnFrontLeds(0);
  delay(delay_ms);
}// void stopRobot(int delay_ms)

void stop_Robot(){
  analogWrite(pwm_A, 0);
  analogWrite(pwm_B, 0);
  turnStopLeds(1);
  turnFrontLeds(0);
}// void stopRobot()

void turnStopLeds(int state){
  if(state==1){
      digitalWrite(BackRightLed, LOW); 
      digitalWrite(BackLeftLed, LOW); 
  }
  else{
      digitalWrite(BackRightLed, HIGH); 
      digitalWrite(BackLeftLed, HIGH); 
  }
}

void turnFrontLeds(int state){
  if(state==1){
      digitalWrite(FrontRightLed, LOW); 
      digitalWrite(FrontLeftLed, LOW); 
  }
  else{
      digitalWrite(FrontRightLed, HIGH); 
      digitalWrite(FrontLeftLed, HIGH); 
  }
} 

void SetPWM (const long pwm_num, byte pwm_channel){
  if(pwm_channel==1){ // DRIVE MOTOR
    analogWrite(pwm_A, pwm_num);
    pwmRvalue = pwm_num;
  }
  else if(pwm_channel==2){ // STEERING MOTOR
    analogWrite(pwm_B, pwm_num);
    pwmLvalue = pwm_num;
  }
}// void SetPWM (const long pwm_num, byte pwm_channel)

