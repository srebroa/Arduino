// Mobile robot based on Rover 5 motor controller.
#include <math.h>

volatile int rotaryCount = 0;

#define PWM_FrontRight_Ch1 5 //R
#define PWM_FrontLeft_Ch2 6 //L
#define PWM_RearRight_Ch3 9 //R
#define PWM_RearLeft_Ch4 10 //L

#define Direction_R_Ch1 2
#define Direction_L_Ch2 4
#define Direction_R_Ch3 7
#define Direction_L_Ch4 8

// IR SENSORS
int IRsensorRightFront_In = 0; // Analog 0
int IRsensorCenterFront_In = 1; // Analog 1
int IRsensorLeftFront_In = 2; // Analog 2
int IRsensorRear_In = 3; // Analog 3
int IR_FrontValue;  // Center IR
int IR_RearValue;  // Back IR
int IR_RightValue;  // Right IR
int IR_LeftValue;  // Left IR


long pwmLvalue = 255;
long pwmRvalue = 255;
byte pwmChannel;
const char startOfNumberDelimiter = '<';
const char endOfNumberDelimiter   = '>';

void setup ()
{
  //Serial.begin(9600);
  Serial1.begin(19200);// BTM222 (115200 dla BTM777); // Bluetooth Baudrate
  pinMode (PWM_FrontRight_Ch1, OUTPUT);
  pinMode (PWM_FrontLeft_Ch2, OUTPUT);
  pinMode (PWM_RearRight_Ch3, OUTPUT);
  pinMode (PWM_RearLeft_Ch4, OUTPUT);
    
  pinMode (Direction_R_Ch1, OUTPUT);
  pinMode (Direction_L_Ch2, OUTPUT);
  pinMode (Direction_R_Ch3, OUTPUT);
  pinMode (Direction_L_Ch4, OUTPUT);
  
  // IR Proximity Sensors
  pinMode(IRsensorCenterFront_In, INPUT);
  pinMode(IRsensorRightFront_In, INPUT);
  pinMode(IRsensorLeftFront_In, INPUT);
  pinMode(IRsensorRear_In, INPUT);
  
  while (!Serial1) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
}  // end of setup

byte phase;
unsigned long start;
int time_to_go;

void loop ()
{
  int i = 0;
  if (Serial1.available()) { 
    processInput();
    readIRsensors();
  }
}  // end of loop

void readIRsensors(){
  IR_FrontValue = IRsensorRead(IRsensorCenterFront_In,3);//analogRead(IRsensorLeftFront_In);
  if(IR_FrontValue>300){
    go_Backwad(500);
    delay(400);
    stop_Robot();   
  }
  else{
    IR_RearValue = IRsensorRead(IRsensorRear_In,3);//analogRead(IRsensorLeftFront_In);
      if(IR_RearValue >300){
      goForward(100, 500);
      stop_Robot();   
    }
  }
}// void readIRsensors()

void processInput (){
  static long receivedNumber = 0;
  static boolean negative = false;
  byte c = Serial1.read ();
  //Serial.println("Motor Driver Test :)"+c);
  
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
      pwmChannel = 1; // PWM_FrontRight_Ch1
      //Serial.write("PWM_FrontRight_Ch1 ");
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
  //Motor A forward 
  digitalWrite(Direction_R_Ch1, HIGH);  //Establishes forward direction of Channel A
  analogWrite(PWM_FrontRight_Ch1, pwmRvalue);
  //Motor B forward 
  digitalWrite(Direction_L_Ch2, HIGH); //Establishes forward direction of Channel B
  analogWrite(PWM_FrontLeft_Ch2, pwmLvalue);
}// void go_Robot()

void go_Forward(int mspeed){
  //Motor A forward mspeed full speed
  digitalWrite(Direction_R_Ch1, HIGH); //Establishes forward direction of Channel A
  analogWrite(PWM_FrontRight_Ch1, mspeed); //Spins the motor on Channel A at full speed 
  //Motor B forward mspeed full speed
  digitalWrite(Direction_L_Ch2, HIGH); //Establishes forward direction of Channel B
  analogWrite(PWM_FrontLeft_Ch2, mspeed); //Spins the motor on Channel B at full speed
  
  digitalWrite(Direction_R_Ch3, HIGH); //Establishes forward direction of Channel A
  analogWrite(PWM_RearRight_Ch3, mspeed); //Spins the motor on Channel A at full speed 
  digitalWrite(Direction_L_Ch4, HIGH); //Establishes forward direction of Channel B
  analogWrite(PWM_RearLeft_Ch4, mspeed); //Spins the motor on Channel B at full speed
}// void goForward(int mspeed, int time_ms)

void go_Backwad(int mspeed){
  //Motor A forward mspeed full speed
  digitalWrite(Direction_R_Ch1, LOW); //Establishes backward direction of Channel A
  analogWrite(PWM_FrontRight_Ch1, mspeed);//Spins the motor on Channel A at full speed 
  //Motor B forward mspeed full speed
  digitalWrite(Direction_L_Ch2, LOW); //Establishes backward direction of Channel B
  analogWrite(PWM_FrontLeft_Ch2, mspeed); //Spins the motor on Channel B at full speed
  
  digitalWrite(Direction_R_Ch3, LOW); //Establishes forward direction of Channel A
  analogWrite(PWM_RearRight_Ch3, mspeed); //Spins the motor on Channel A at full speed 
  digitalWrite(Direction_L_Ch4, LOW); //Establishes forward direction of Channel B
  analogWrite(PWM_RearLeft_Ch4, mspeed); //Spins the motor on Channel B at full speed
}// void goBackwad(int mspeed, int time_ms)

void turn_Right(int mspeed){ 
  //Motor A backward mspeed full speed
  digitalWrite(Direction_R_Ch1, LOW);  //Establishes backward direction of Channel A
  analogWrite(PWM_FrontRight_Ch1, mspeed); //Spins the motor on Channel A at full speed 
  digitalWrite(Direction_R_Ch3, LOW); //Establishes forward direction of Channel A
  analogWrite(PWM_RearRight_Ch3, mspeed); //Spins the motor on Channel A at full speed 
  //Motor B forward mspeed full speed
  digitalWrite(Direction_L_Ch2, HIGH); //Establishes forward direction of Channel B
  analogWrite(PWM_FrontLeft_Ch2, mspeed); //Spins the motor on Channel B at full speed
  digitalWrite(Direction_L_Ch4, HIGH); //Establishes forward direction of Channel B
  analogWrite(PWM_RearLeft_Ch4, mspeed); //Spins the motor on Channel B at full speed
}// void goBackwad(int mspeed, int time_ms)

void turn_Left(int mspeed){
  //Motor A forward mspeed full speed
  digitalWrite(Direction_R_Ch1, HIGH); //Establishes forward direction of Channel A
  analogWrite(PWM_FrontRight_Ch1, mspeed); //Spins the motor on Channel A at half speed 
  digitalWrite(Direction_R_Ch3, HIGH); //Establishes forward direction of Channel A
  analogWrite(PWM_RearRight_Ch3, mspeed); //Spins the motor on Channel A at full speed 
  //Motor B backward mspeed full speed
  digitalWrite(Direction_L_Ch2, LOW);  //Establishes backward direction of Channel B
  analogWrite(PWM_FrontLeft_Ch2, mspeed); //Spins the motor on Channel B at full speed
  digitalWrite(Direction_L_Ch4, LOW); //Establishes forward direction of Channel B
  analogWrite(PWM_RearLeft_Ch4, mspeed); //Spins the motor on Channel B at full speed
}// void goBackwad(int mspeed, int time_ms)

void goForward(int mspeed, int time_ms){
  //Motor A forward mspeed full speed
  digitalWrite(Direction_R_Ch1, HIGH); //Establishes forward direction of Channel A
  analogWrite(PWM_FrontRight_Ch1, mspeed); //Spins the motor on Channel A at half speed 
  //Motor B forward mspeed full speed
  digitalWrite(Direction_L_Ch2, HIGH); //Establishes forward direction of Channel B
  analogWrite(PWM_FrontLeft_Ch2, mspeed); //Spins the motor on Channel B at full speed
  delay(time_ms);
}// void goForward(int mspeed, int time_ms)

void stopRobot(int delay_ms){
  analogWrite (PWM_FrontRight_Ch1, 0);
  analogWrite (PWM_FrontLeft_Ch2, 0);
  analogWrite (PWM_RearRight_Ch3, 0);
  analogWrite (PWM_RearLeft_Ch4, 0);
  delay(delay_ms);
}// void  stopRobot(int delay_ms)

void stop_Robot(){
  analogWrite (PWM_FrontRight_Ch1, 0);
  analogWrite (PWM_FrontLeft_Ch2, 0);
  analogWrite (PWM_RearRight_Ch3, 0);
  analogWrite (PWM_RearLeft_Ch4, 0);
}// void stopRobot()
  
void SetPWM (const long pwm_num, byte pwm_channel){
  if(pwm_channel==1){ // RIGHT
    analogWrite(PWM_FrontRight_Ch1, pwm_num);
    analogWrite(PWM_RearRight_Ch3, pwm_num);
    pwmRvalue = pwm_num;
    //Serial.write("PWM_FrontRight_Ch1IGHT ");
    //Serial.println(pwm_num);
  }
  else if(pwm_channel==2){ // LEFT
    analogWrite(PWM_FrontLeft_Ch2, pwm_num);
    analogWrite(PWM_RearLeft_Ch4, pwm_num);
    pwmLvalue = pwm_num;
    //Serial.write("pwm_LEFT ");
    //Serial.println(pwm_num);
  }
}// void SetPWM (const long pwm_num, byte pwm_channel)

int IRsensorRead(int sensor_number, int number_of_measurements){
  int average_value = 0;
  for(int i=0; i<number_of_measurements; i++){
    average_value += analogRead(sensor_number);
  }
  average_value = (int)(average_value/number_of_measurements);
  return average_value;
}
