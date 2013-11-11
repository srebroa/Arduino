
/*Arduino Motor Shield*/
const int MotorR = 12;      // H bridge control Input
const int MotorL = 13;   // H bridge control Input
const int BrakeR = 9;
const int BrakeL = 8;
const int pwm_R = 3;         // pwm output
const int pwm_L = 11;         // pwm output


void setup()
{
  //Serial1.begin(115200); // Bluetooth Baudrate
  //Setup Channel A - RIGHT MOTOR
  pinMode(MotorR, OUTPUT); //Initiates Motor Channel A pin
  pinMode(BrakeR, OUTPUT); //Initiates Brake Channel A pin
  
  //Setup Channel B - LEFT MOTOR
  pinMode(MotorL, OUTPUT); //Initiates Motor Channel B pin
  pinMode(BrakeL, OUTPUT);  //Initiates Brake Channel B pin
 
  //pinMode(pwm_R, OUTPUT);   
  //pinMode(pwm_L, OUTPUT); 
  
  //pinMode(sensor1, INPUT);   // sets the pin as input
  //pinMode(sensor2, INPUT);
  //pinMode(sensor3, INPUT);
}

void loop(){
// obrot dookola nieruchomego kola
/*
icr_circle(255, 255, 1500);// 0 - 100
stopRobot(1000);
icr_circle(-255, -255, 1500);
stopRobot(1000);
*/

// obrot dookola nieruchomego kola
icr_circle(255, 0, 3000); // Turn RIGHT
icr_circle(0, 255, 3000); // Turn LEFT
stopRobot(10000);
// osemka
icr_circle(255, 65, 7800); // Turn RIGHT
icr_circle(65, 255, 7800); // Turn LEFT
stopRobot(10000);
//Spirala Archimedesa
archimedian_spiral(50, 255);
stopRobot(10000);

//icr_circle(100, 255, 4000);
/*
stopRobot(1000);
icr_circle(-255, 255, 4000);
stopRobot(1000);
icr_circle(255, 60, 4000);
stopRobot(1000);
icr_circle(60, 255, 4000);
stopRobot(1000);
*/
}

void  stopRobot(int delay_ms){
  //analogWrite(pwm_out, 240); 
  digitalWrite(BrakeR, HIGH);  //Engage the Brake for Channel A
  digitalWrite(BrakeL, HIGH);  //Engage the Brake for Channel B
  delay(delay_ms);
  //digitalWrite(BrakeR, LOW);  //Disable Motor R Brake
  //digitalWrite(BrakeL, LOW);  //Disable Motor L Brake
  //analogWrite(pwm_out, 40); 
}// void  stopRobot(int delay_ms)

void  stop_Robot(){
  //analogWrite(pwm_out, 240); 
  digitalWrite(BrakeR, HIGH);  //Engage the Brake for Channel A
  digitalWrite(BrakeL, HIGH);  //Engage the Brake for Channel B
}// void  stopRobot(int delay_ms)

void go_Forward(int mspeed){
  //Motor A forward @ full speed
  digitalWrite(MotorR, HIGH);  //Establishes backward direction of Channel A
  analogWrite(pwm_R, mspeed);    //Spins the motor on Channel A at half speed 
  //Motor B forward @ full speed
  digitalWrite(MotorL, HIGH); //Establishes forward direction of Channel B
  analogWrite(pwm_L, mspeed);   //Spins the motor on Channel B at full speed
}// void goForward(int mspeed, int time_ms)

void go_Backwad(int mspeed){
  //Motor A forward @ full speed
  digitalWrite(MotorR, LOW);  //Establishes backward direction of Channel A
  analogWrite(pwm_R, mspeed);    //Spins the motor on Channel A at half speed 
  //Motor B forward @ full speed
  digitalWrite(MotorL, LOW); //Establishes forward direction of Channel B
  analogWrite(pwm_L, mspeed);   //Spins the motor on Channel B at full speed
}// void goBackwad(int mspeed, int time_ms)

void goForward(int mspeed, int time_ms){
  //Motor A forward @ full speed
  digitalWrite(MotorR, HIGH);  //Establishes backward direction of Channel A
  analogWrite(pwm_R, mspeed);    //Spins the motor on Channel A at half speed 
  //Motor B forward @ full speed
  digitalWrite(MotorL, HIGH); //Establishes forward direction of Channel B
  analogWrite(pwm_L, mspeed);   //Spins the motor on Channel B at full speed
  delay(time_ms);
}// void goForward(int mspeed, int time_ms)

void goBackwad(int mspeed, int time_ms){
  //Motor A forward @ full speed
  digitalWrite(MotorR, LOW);  //Establishes backward direction of Channel A
  analogWrite(pwm_R, mspeed);    //Spins the motor on Channel A at half speed 
  //Motor B forward @ full speed
  digitalWrite(MotorL, LOW); //Establishes forward direction of Channel B
  analogWrite(pwm_L, mspeed);   //Spins the motor on Channel B at full speed
  delay(time_ms);
}// void goBackwad(int mspeed, int time_ms)

void icr_circle(int Vr, int Vl, int time_ms){
    
  int Vr_real = Vr; //(unsigned int)Vr*2.55;
  int Vl_real = Vl;
  
  //analogWrite(pwm_R, Vr_real); 
  //analogWrite(pwm_L, Vl_real); 
  digitalWrite(BrakeR, LOW);  //Disable Motor R Brake
  digitalWrite(BrakeL, LOW);  //Disable Motor L Brake
  
  if(Vr>0){
    digitalWrite(MotorR, HIGH);
  }
  else{
    digitalWrite(MotorR, LOW);
    Vr_real = -1*Vr;
  }
  if(Vl>0){
    digitalWrite(MotorL, HIGH);
  }
  else{
    digitalWrite(MotorL, LOW);
    Vl_real = -1*Vl;
  }
  
  analogWrite(pwm_R, Vr_real); 
  analogWrite(pwm_L, Vl_real); 
  
  delay(time_ms);
}// void rotate(int Vr, int Vl)

void archimedian_spiral (int Vr, int Vl){
        //m3pi.left_motor(Vl);
        //m3pi.right_motor(Vr);   
        //t.start();
        icr_circle(Vr, Vl, 50);
        while(Vr<250){
          Vr = Vr+1;
          analogWrite(pwm_R, Vr); 
          delay(100);
        }
}

/*
void checkSensorsValues(){
  
  if(sensor1State != digitalRead(sensor1)){
    sensor1State = digitalRead(sensor1);
    sensorStateChanged = true;
    Serial.print("sensor1 = ");
    Serial.println(sensor1State);
  }
  else if(sensor2State != digitalRead(sensor2)){
    sensor2State = digitalRead(sensor2);
    sensorStateChanged = true;
    Serial.print("sensor2 = ");
    Serial.println(sensor2State);
  }
  else if(sensor3State != digitalRead(sensor3)){
    sensor3State = digitalRead(sensor3);
    sensorStateChanged = true;
    Serial.print("sensor3 = ");
    Serial.println(sensor3State);
  }
  else{
    sensorStateChanged = false;
  }
}// void checkSensorsValues()
*/
