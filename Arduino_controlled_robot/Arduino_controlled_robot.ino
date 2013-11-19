/* 
ARDUINO CONTROLLED ROBOT v1.0
   - Allows you to control a mobile robot via bluetooth
   - Tested with Arduino Leonardo
   - Android application - https://play.google.com/store/apps/details?id=pl.mobilerobots.vacuumcleanerrobot&hl=pl
   Author: Adam Srebro
   www: http://www.mobilerobots.pl/
*/
#include <Wire.h>
#include <math.h>

/*Arduino Motor Shield*/
const int MotorR = 13;      // H bridge control Input
const int MotorL = 12;      // H bridge control Input
const int BrakeR = 9;
const int BrakeL = 8;
// SDA = 2
// SCL = 3
const int pwm_L = 5;         // pwm output
const int pwm_R = 11;        // pwm output
long pwmLvalue = 255;
long pwmRvalue = 255;
byte pwmChannel;
const char startOfNumberDelimiter = '<';
const char endOfNumberDelimiter   = '>';

void setup(){
  Serial1.begin(115200); // Bluetooth Baudrate
  
  //Setup Channel A - RIGHT MOTOR
  pinMode(MotorR, OUTPUT); //Initiates Motor Channel A pin
  pinMode(BrakeR, OUTPUT); //Initiates Brake Channel A pin
  
  //Setup Channel B - LEFT MOTOR
  pinMode(MotorL, OUTPUT); //Initiates Motor Channel B pin
  pinMode(BrakeL, OUTPUT);  //Initiates Brake Channel B pin
  Wire.begin();

  while (!Serial1) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
}// void setup()

void loop(){ 
  int i = 0;
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
    
    case 'g': // robot GO!
      go_Robot();
      break; 
      
    case 'f': // Go FORWARD
      go_Forward(255);
      break; 
      
    case 'b': // Go BACK
      go_Backwad(255);
      break; 
      
    case 'r':
      turn_Right(255);
      //Serial.write("right");
      break; 
     
    case 'l':
      turn_Left(255);
      //Serial.write("left");
      break; 
      
    case 's':
      stop_Robot();
      break; 
      
    case 'x':
      pwmChannel = 1; // pwm_R
      //Serial.write("pwm_R ");
      break; 
    case 'y':         // pwm_L
      pwmChannel = 2;
      //Serial.write("pwm_L ");
      break; 
         
    case '0' ... '9': 
      receivedNumber *= 10;
      receivedNumber += c - '0';
      break;
      
    case '-':
      negative = true;
      break;
      
    case 'c':
      //setProgramNumber(0);
      break;
      
    } // end of switch  
  }  // end of processInput

void go_Robot(){
  digitalWrite(BrakeR, LOW);  //Disable Motor R Brake
  digitalWrite(BrakeL, LOW);  //Disable Motor L Brake
  //Motor A forward 
  digitalWrite(MotorR, HIGH);  //Establishes forward direction of Channel A
  analogWrite(pwm_R, pwmRvalue);
  //Motor B forward 
  digitalWrite(MotorL, HIGH); //Establishes forward direction of Channel B
  analogWrite(pwm_L, pwmLvalue);
}// void go_Robot()

void go_Forward(int mspeed){
  digitalWrite(BrakeR, LOW);  //Disable Motor R Brake
  digitalWrite(BrakeL, LOW);  //Disable Motor L Brake
  //Motor A forward mspeed full speed
  digitalWrite(MotorR, HIGH); //Establishes forward direction of Channel A
  analogWrite(pwm_R, mspeed); //Spins the motor on Channel A at full speed 
  //Motor B forward mspeed full speed
  digitalWrite(MotorL, HIGH); //Establishes forward direction of Channel B
  analogWrite(pwm_L, mspeed); //Spins the motor on Channel B at full speed
}// void goForward(int mspeed, int time_ms)

void go_Backwad(int mspeed){
  digitalWrite(BrakeR, LOW); //Disable Motor R Brake
  digitalWrite(BrakeL, LOW); //Disable Motor L Brake
  //Motor A forward mspeed full speed
  digitalWrite(MotorR, LOW); //Establishes backward direction of Channel A
  analogWrite(pwm_R, mspeed);//Spins the motor on Channel A at full speed 
  //Motor B forward mspeed full speed
  digitalWrite(MotorL, LOW); //Establishes backward direction of Channel B
  analogWrite(pwm_L, mspeed); //Spins the motor on Channel B at full speed
}// void goBackwad(int mspeed, int time_ms)

void turn_Right(int mspeed){
  digitalWrite(BrakeR, LOW);  //Disable Motor R Brake
  digitalWrite(BrakeL, LOW);  //Disable Motor L Brake  
  //Motor A backward mspeed full speed
  digitalWrite(MotorR, LOW);  //Establishes backward direction of Channel A
  analogWrite(pwm_R, mspeed); //Spins the motor on Channel A at full speed 
  //Motor B forward mspeed full speed
  digitalWrite(MotorL, HIGH); //Establishes forward direction of Channel B
  analogWrite(pwm_L, mspeed); //Spins the motor on Channel B at full speed
}// void goBackwad(int mspeed, int time_ms)

void turn_Left(int mspeed){
  digitalWrite(BrakeR, LOW);  //Disable Motor R Brake
  digitalWrite(BrakeL, LOW);  //Disable Motor L Brake
  //Motor A forward mspeed full speed
  digitalWrite(MotorR, HIGH); //Establishes forward direction of Channel A
  analogWrite(pwm_R, mspeed); //Spins the motor on Channel A at half speed 
  //Motor B backward mspeed full speed
  digitalWrite(MotorL, LOW);  //Establishes backward direction of Channel B
  analogWrite(pwm_L, mspeed); //Spins the motor on Channel B at full speed
}// void goBackwad(int mspeed, int time_ms)

void goForward(int mspeed, int time_ms){
  digitalWrite(BrakeR, LOW);  //Disable Motor R Brake
  digitalWrite(BrakeL, LOW);  //Disable Motor L Brake
  //Motor A forward mspeed full speed
  digitalWrite(MotorR, HIGH); //Establishes forward direction of Channel A
  analogWrite(pwm_R, mspeed); //Spins the motor on Channel A at half speed 
  //Motor B forward mspeed full speed
  digitalWrite(MotorL, HIGH); //Establishes forward direction of Channel B
  analogWrite(pwm_L, mspeed); //Spins the motor on Channel B at full speed
  delay(time_ms);
}// void goForward(int mspeed, int time_ms)

void stopRobot(int delay_ms){
  //analogWrite(pwm_out, 240); 
  digitalWrite(BrakeR, HIGH);  //Engage the Brake for Channel A
  digitalWrite(BrakeL, HIGH);  //Engage the Brake for Channel B
  delay(delay_ms);
}// void  stopRobot(int delay_ms)

void stop_Robot(){
  digitalWrite(BrakeR, HIGH);  //Engage the Brake for Channel A
  digitalWrite(BrakeL, HIGH);  //Engage the Brake for Channel B
}// void stopRobot()
  
void SetPWM (const long pwm_num, byte pwm_channel){
  if(pwm_channel==1){ // RIGHT
    analogWrite(pwm_R, pwm_num);
    pwmRvalue = pwm_num;
    //Serial.write("pwm_RIGHT ");
    //Serial.println(pwm_num);
  }
  else if(pwm_channel==2){ // LEFT
    analogWrite(pwm_L, pwm_num);
    pwmLvalue = pwm_num;
    //Serial.write("pwm_LEFT ");
    //Serial.println(pwm_num);
  }
}// void SetPWM (const long pwm_num, byte pwm_channel)

