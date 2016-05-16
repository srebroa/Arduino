/* Testing Digital Servos LF-20MG controlled via Serial Port
Examples:
a45
b90
a180b40
*/
#include <Servo.h>
#define SERVO1_MIN 10
#define SERVO1_MAX 180
#define SERVO2_MIN 10
#define SERVO2_MAX 180

#define SERVO1_PWM_PIN 5
#define SERVO2_PWM_PIN 6

Servo servo1;
Servo servo2;
int servo1_val;
int servo2_val;

void setup(){
  Serial.begin(9600);
  servo1.attach(SERVO1_PWM_PIN);
  servo2.attach(SERVO2_PWM_PIN);
  servo1_val = servo1.read();
  servo2_val = servo2.read();
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

    case 'p':
    {
      // Set Default Position
      setDefaultPosition(90, 120);
      break;
    }

    case 'i':
    {
      // Print current servos position
      printServosPosition();
      break;
    }
  }// switch (c)
}

void printServosPosition(){
   Serial.println("servo1: "+String(servo1_val));
   Serial.println("servo2: "+String(servo2_val));
}

void setDefaultPosition(int s1_deg, int s2_deg){
    setServo(1, s1_deg);
    setServo(2, s2_deg);
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
}// void setServo(int servoNumber, int deg)

