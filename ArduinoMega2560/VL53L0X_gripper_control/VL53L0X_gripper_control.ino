/* VL53L0X_gripper_control - Automatic gripping of an object that has been detected by the VL53L0X laser sensor
The range readings are in units of mm. */

#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>

#define SERVO_PWM_PIN 3 // Servo
#define DEFAULT_SERVO_ANGLE 90
#define GRIPPER_MIN 40
#define GRIPPER_MAX 120
#define OPEN_MAX 100
#define THRESHOLD_CLOSING_DISTANCE_FAR 200
#define THRESHOLD_CLOSING_DISTANCE_NEAR 60

VL53L0X sensor;
Servo servoGripper;
int servoGripper_val;
const int  gripperOpenButtonPin = 2;    // the pin that the pushbutton is attached to
int gripperOpenButtonState = 0;         // current state of the button

void setup()
{
  // initialize the button pin as a input:
  pinMode(gripperOpenButtonPin, INPUT);
  Serial.begin(9600);
  Wire.begin();
  sensor.init();
  sensor.setTimeout(500);
  initializeServo();
  //gripperServoCurrentState = 0; //opened

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous();
}

void loop()
{
  int distance_mm = sensor.readRangeContinuousMillimeters();
  Serial.println(distance_mm);
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  if(distance_mm>THRESHOLD_CLOSING_DISTANCE_FAR || distance_mm<THRESHOLD_CLOSING_DISTANCE_NEAR){
      setServoBaseDirection(0); // close gripper
  }
  else{
      setServoBaseDirection(1); // open gripper 
  }
  // read the pushbutton input pin:
  gripperOpenButtonState = digitalRead(gripperOpenButtonPin);
  Serial.println("ButtonState: "+String(gripperOpenButtonState));
  if (gripperOpenButtonState == HIGH) {
    //open gripper and wait until object will be take off
    openGripper();
  }
  delay(10);
}

void initializeServo(){ 
  Serial.write("\nServo test\n"); 
  servoGripper.attach(SERVO_PWM_PIN);
}

void setDefaulRoboticArmPosition(){
  setServo(1, DEFAULT_SERVO_ANGLE);
  servoGripper_val = servoGripper.read();
  //currentRoboticArmState = 0;
}

void openGripper(){
  servoGripper_val = servoGripper.read();
  Serial.println("servoGripper: "+String(servoGripper_val));
      Serial.println("open gripper");
      if(servoGripper_val<OPEN_MAX){
        for(servoGripper_val; servoGripper_val<OPEN_MAX; servoGripper_val +=1)// in steps of 1 degree   
        {                                 
          servoGripper.write(servoGripper_val); // tell servo to go to "currentServoPosition" 
          delay(10);                       // waits 10ms for the servo to reach the position 
          //Serial.println("servoBaseN: "+String(servoGripper_val));
        }// for 
      }// if
      delay(2000); 
}

void setServoBaseDirection(int servoGripperDirection){
  servoGripper_val = servoGripper.read();
  Serial.println("servoGripper: "+String(servoGripper_val));
  if(servoGripperDirection==1){ // open gripper
      Serial.println("open gripper");
      if(servoGripper_val<GRIPPER_MAX){
        servoGripper_val +=1;                             
        servoGripper.write(servoGripper_val); // tell servo to go to "currentServoPosition" 
        delay(10);                       // waits 10ms for the servo to reach the position 
      }// if
  }
  else{ // close gripper
      Serial.println("close gripper");
      if(servoGripper_val>GRIPPER_MIN){
        servoGripper_val -=1;                                 
        servoGripper.write(servoGripper_val); // tell servo to go to "currentServoPosition" 
        delay(10);                       // waits 15ms for the servo to reach the position 
      }// if
  }
  delay(20);
}// void setServoBasePosition(int servoGripperDirection)

void setServo(int servoNumber, int deg){
  if(servoNumber==1 && deg>=GRIPPER_MIN && deg<=GRIPPER_MAX){
    servoGripper.write(deg);
    servoGripper_val = servoGripper.read();
    Serial.println("servoGripper: "+String(servoGripper_val));    
  }
}// void setServo(int servoNumber, int deg)
