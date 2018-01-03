/*
6WD robot example program 
- Tested with Arduino Mega 2560
www: http://www.mobilerobots.pl
 
 Connections:
 BTS7960 -> Arduino Mega 2560
 MotorRight_R_EN - 22
 MotorRight_L_EN - 23
 MotorLeft_R_EN - 26
 MotorLeft_L_EN - 27
 Rpwm1 - 2
 Lpwm1 - 3
 Rpwm2 - 4
 Lpwm2 - 5
 
 ​FlySky FS 2.4GHz Receiver -> Arduino Mega 2560
 ch2 - 7 // Aileron
 ch3 - 8 // Elevator
 */
#define LOWER_STOP_RANGE_MOVE -20
#define UPPER_STOP_RANGE_MOVE 20
#define LOWER_STOP_RANGE_TURN -20
#define UPPER_STOP_RANGE_TURN 20
 
/*BTS7960 Motor Driver Carrier*/
const int MotorRight_R_EN = 22; 
const int MotorRight_L_EN = 23; 

const int MotorLeft_R_EN = 26;
const int MotorLeft_L_EN = 27;

const int Rpwm1 = 2; 
const int Lpwm1 = 3;
const int Rpwm2 = 4; 
const int Lpwm2 = 5; 
long pwmLvalue = 255;
long pwmRvalue = 255;
byte pwmChannel;
int robotControlState;
int last_mspeed;
boolean stop_state = true;

// MODE2 
int ch1; // Throttle
int ch2; // Aileron
int ch3; // Elevator
int ch4; // Rudder

int moveValue;
int turnValue;

void setup(){
  //pinMode(6, INPUT); 
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  //pinMode(9, INPUT);
  Serial.begin(9600);

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

void loop() {
  //ch1 = pulseIn(6, HIGH, 25000); // Read the pulse width of each channel
  ch2 = pulseIn(7, HIGH, 25000); 
  ch3 = pulseIn(8, HIGH, 25000);
  //ch4 = pulseIn(9, HIGH, 25000);
  
  moveValue = map(ch3, 980, 1999, -255, 255); 
  moveValue = constrain(moveValue, -255, 255);                                    
  turnValue = map(ch2, 980, 1999, -255, 255);
  turnValue = constrain(turnValue, -255, 255);
  
  //Serial.println("moveValue: "+String(moveValue)+ ", turnValue: "+String(turnValue));
  if (moveValue>LOWER_STOP_RANGE_MOVE && moveValue<UPPER_STOP_RANGE_MOVE && turnValue>LOWER_STOP_RANGE_TURN && turnValue<UPPER_STOP_RANGE_TURN){
    if(stop_state == false){
      stop_Robot();
      stop_state = true;
      Serial.println("Stop Robot");
    }
  }
  //GO FORWARD & BACKWARD
  else if(turnValue>LOWER_STOP_RANGE_TURN && turnValue<UPPER_STOP_RANGE_TURN){
    if(moveValue>UPPER_STOP_RANGE_MOVE){
      go_Forward(moveValue);
      stop_state = false;
      Serial.println("Go Forward");
    }
    else if(moveValue<LOWER_STOP_RANGE_MOVE){
      go_Backwad(abs(moveValue));
      stop_state = false;
      Serial.println("Go Backward");
    }
  } 
  //TURN RIGHT & LEFT
  else if(moveValue>LOWER_STOP_RANGE_MOVE && moveValue<UPPER_STOP_RANGE_MOVE){
    if(turnValue>UPPER_STOP_RANGE_TURN){
      turn_Right(turnValue);
      stop_state = false;
      Serial.println("Turn Right");
    }
    else if(turnValue<LOWER_STOP_RANGE_TURN){
      turn_Left(abs(turnValue));
      stop_state = false;
      Serial.println("Turn Left");
    }
  } 
  delay(200);
}

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
