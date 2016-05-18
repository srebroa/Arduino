/* Testing Robotic Arm (2 Digital Servos LF-20MG and 1 analog HD-1501MG) controlled via Serial Port
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
#define SERVO1_MIN 10
#define SERVO1_MAX 180
#define SERVO2_MIN 10
#define SERVO2_MAX 180
#define SERVO3_MIN 10
#define SERVO3_MAX 180

#define SERVO1_PWM_PIN 3
#define SERVO2_PWM_PIN 5
#define SERVO3_PWM_PIN 6

/*TB6612FNG Dual Motor Driver Carrier*/
#define GRIPPER_MOTOR_PWM 11 // pwm output
#define GRIPPER_MOTOR_AIN1 12 
#define GRIPPER_MOTOR_AIN2 13

/*Servos*/
Servo servo1;
Servo servo2;
Servo servo3;

int servo1_val;
int servo2_val;
int servo3_val;

void setup(){
  Serial.begin(9600);
  servo1.attach(SERVO1_PWM_PIN);
  servo2.attach(SERVO2_PWM_PIN);
  servo3.attach(SERVO3_PWM_PIN);
  servo1_val = servo1.read();
  servo2_val = servo2.read();
  servo3_val = servo3.read();
}// void setup()

void loop(){
  while (Serial.available()) {
    processInput();
  } 
}// void loop()

void processInput (){
  byte c = Serial.read ();
  switch (c){   
    case 'a':
    {
      int deg = Serial.parseInt();
      setServo(1, deg);
      break;
    }

    case 'b':
    {
      int deg = Serial.parseInt();
      setServo(2, deg);
      break;
    }
    
    case 'c':
    {
      int deg = Serial.parseInt();
      setServo(3, deg);
      break;
    }
    
    case 'g':
    {
      // Gripper Control
      int mspeed = Serial.parseInt(); //max 255
      char mdirection = Serial.read ();
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
   Serial.println("servo1: "+String(servo1_val));
   Serial.println("servo2: "+String(servo2_val));
   Serial.println("servo3: "+String(servo3_val));
}

void setDefaultPosition(int s1_deg, int s2_deg, int s3_deg){
    setServo(1, s1_deg);
    setServo(2, s2_deg);
    setServo(3, s3_deg);
}

void setServo(int servoNumber, int deg){
  if(servoNumber==1 && deg>=SERVO1_MIN && deg<=SERVO1_MAX){
    servo1.write(deg);
    servo1_val = servo1.read();
    Serial.println("servo1: "+String(servo1_val));
  }
  else if(servoNumber==2 && deg>=SERVO2_MIN && deg<=SERVO2_MAX){
    servo2.write(deg);
    servo2_val = servo2.read();
    Serial.println("servo2: "+String(servo2_val));
  }
  else if(servoNumber==3 && deg>=SERVO3_MIN && deg<=SERVO3_MAX){
    servo3.write(deg);
    servo3_val = servo3.read();
    Serial.println("servo3: "+String(servo3_val));
  }
}// void setServo(int servoNumber, int deg)

