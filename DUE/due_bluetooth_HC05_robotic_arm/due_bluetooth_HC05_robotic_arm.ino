/*
Arduino Due + HC-05 bluetooth + servos
- servos controlled via bluetooth

Serial (Tx/Rx) communicate to PC via USB
Serial1 (Tx1/Rx1) connect to HC-05
HC-05 Rx - Due Tx1 (18)
HC-05 Tx - Due Rx1 (19)
HC-05 GND - Due GND
HC-05 VCC - Due 3.3V

Examples:
a45     // base rotation - analog servo
b90
c45
a45b45c45

// gripper control
g120c // close half speed
g255o // open full speed
*/

#include <Servo.h>

#define HC05 Serial1
#define LED 2

#define BASE_MIN 15
#define BASE_MAX 170
#define SHOULDER_MIN 15
#define SHOULDER_MAX 170
#define ELBOW_MIN 15
#define ELBOW_MAX 170

#define BASE_PWM_PIN 3 // Base
#define SHOULDER_PWM_PIN 5 // Shoulder
#define ELBOW_PWM_PIN 6 // Elbow

/*TB6612FNG Dual Motor Driver Carrier*/
#define GRIPPER_MOTOR_PWM 11 // pwm output
#define GRIPPER_MOTOR_AIN1 12 
#define GRIPPER_MOTOR_AIN2 13

Servo servoBase;
Servo servoShoulder;
Servo servoElbow;

int servoBase_val;
int servoShoulder_val;
int servoElbow_val;

void setup(){
  pinMode(LED, OUTPUT);
  flashLed(500);
  Serial.begin(9600);
  HC05.begin(9600);
  HC05.setTimeout(10);
  Serial.write("\nServo test\n");
  
  servoBase.attach(BASE_PWM_PIN);
  servoShoulder.attach(SHOULDER_PWM_PIN);
  servoElbow.attach(ELBOW_PWM_PIN);
  servoBase_val = servoBase.read();
  servoShoulder_val = servoShoulder.read();
  servoElbow_val = servoElbow.read();
}

void loop(){
  while(HC05.available()){
    processInput();
  }
}// void loop()

void processInput (){
  byte c = HC05.read ();
  switch (c){    
    
    case 'b': // Base
    {   
      int deg_base = HC05.parseInt();
      Serial.println("deg_base: "+String(deg_base));
      setServo(1, deg_base);
      break;
    }
    
    case 's': // Shoulder
    {   
      int deg_shoulder = HC05.parseInt();
      Serial.println("deg_shoulder: "+String(deg_shoulder));
      setServo(2, deg_shoulder);
      break;
    }

    case 'e': // Elbow
    {
      //turnLed(1);      
      int deg_elbow = HC05.parseInt();
      Serial.println("deg_elbow: "+String(deg_elbow));
      setServo(3, deg_elbow);
      break;
    }
    case 'g':
    {
      // Gripper Control
      int mspeed = HC05.parseInt(); //max 255
      char mdirection = HC05.read ();
      Serial.println("mspeed: "+String(mspeed)+" mdirection: "+String(mdirection));
      gripperControl(mdirection, mspeed);
      break;
    }
    case 'p':
    {
      // Set Default Position
      setDefaultPosition(90, 120, 120);
      break;
    }

    case 'i':
    {
      // Print current servos position
      printServosPosition();
      break;
    }
  }// switch (c)
}// void processInput ()

void gripperControl(char mdirection, int mspeed){
  if (mdirection == 'c'){
    digitalWrite(GRIPPER_MOTOR_AIN1, LOW); 
    digitalWrite(GRIPPER_MOTOR_AIN2, HIGH); 
  }
  else if (mdirection == 'o'){
    digitalWrite(GRIPPER_MOTOR_AIN1, HIGH); 
    digitalWrite(GRIPPER_MOTOR_AIN2, LOW); 
  }
  analogWrite(GRIPPER_MOTOR_PWM, mspeed);
  delay(500);
  analogWrite(GRIPPER_MOTOR_PWM, 0);
}// void gripperControl(int mdirection, int mspeed)

void printServosPosition(){
   Serial.println("servoBase: "+String(servoBase_val));
   Serial.println("servoShoulder: "+String(servoShoulder_val));
   Serial.println("servoElbow: "+String(servoElbow_val));
}

void setDefaultPosition(int s1_deg, int s2_deg, int s3_deg){
    setServo(1, s1_deg);
    setServo(2, s2_deg);
    setServo(3, s3_deg);
}

void flashLed(int delay_ms){
  digitalWrite(LED, HIGH); 
  delay(delay_ms);
  digitalWrite(LED, LOW);
}

void turnLed(int state){
  if(state==1){
      digitalWrite(LED,  HIGH); 
  }
  else{
      digitalWrite(LED, LOW); 
  }
}

void setServo(int servoNumber, int deg){
  if(servoNumber==1 && deg>=BASE_MIN && deg<=BASE_MAX){
    servoBase.write(deg);
    servoBase_val = servoBase.read();
    Serial.println("servoBase: "+String(servoBase_val));
  }
  else if(servoNumber==2 && deg>=SHOULDER_MIN && deg<=SHOULDER_MAX){
    servoShoulder.write(deg);
    servoShoulder_val = servoShoulder.read();
    Serial.println("servoShoulder: "+String(servoShoulder_val));
  }
  else if(servoNumber==3 && deg>=ELBOW_MIN && deg<=ELBOW_MAX){
    servoElbow.write(deg);
    servoElbow_val = servoElbow.read();
    Serial.println("servoElbow: "+String(servoElbow_val));
  }
}// void setServo(int servoNumber, int deg)



