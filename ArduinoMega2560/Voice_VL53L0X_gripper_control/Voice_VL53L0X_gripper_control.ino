/* Voice_VL53L0X_gripper_control - Automatic gripping of an object using VL53L0X laser sensor and voice commands
The range readings are in units of mm. 

Connections:
 Bluetooth (e.g HC-06)-> Arduino Mega 2560
 TXD - TX1 (19)
 RXD - RX1 (18)
 VCC - 5V
 GND - GND
*/

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

#define SERVO_DELAY_FAST 5
#define SERVO_DELAY_SLOW 40

VL53L0X sensor;
Servo servoGripper;
int servoGripper_val;
const int  gripperOpenButtonPin = 2;    // the pin that the pushbutton is attached to
int gripperOpenButtonState = 0;         // current state of the button
int servoDelay_ms;
String voice;

void setup()
{
  Serial1.begin(9600);// HC-06 default baudrate: 9600
  // initialize the button pin as a input:
  pinMode(gripperOpenButtonPin, INPUT);
  Serial.begin(9600);
  Wire.begin();
  sensor.init();
  sensor.setTimeout(500);
  servoDelay_ms = 10;
  initializeServo();
  //gripperServoCurrentState = 0; //opened
  Wire.begin();
  sensor.startContinuous();
}

void loop()
{
  int distance_mm = sensor.readRangeContinuousMillimeters();
  Serial.println(distance_mm);
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  if(distance_mm>THRESHOLD_CLOSING_DISTANCE_FAR || distance_mm<THRESHOLD_CLOSING_DISTANCE_NEAR){
      setServoBaseDirection(0, servoDelay_ms); // close gripper
  }
  else{
      setServoBaseDirection(1, servoDelay_ms); // open gripper 
  }
  // read the pushbutton input pin:
  gripperOpenButtonState = digitalRead(gripperOpenButtonPin);
  Serial.println("ButtonState: "+String(gripperOpenButtonState));
  if (gripperOpenButtonState == HIGH) {
    //open gripper and wait until object will be take off
    openGripper(servoDelay_ms);
  }
  while (Serial1.available()) {
    delay(5); //Delay added to make thing stable 
    char c = Serial1.read(); //Conduct a serial read
    if (c == '#') {
      break; //Exit the loop when the # is detected after the word
    } 
    voice += c; //Shorthand for voice = voice + c
  }// while (Serial1.available())
  processInput(voice);
  voice="";
  //delay(1);
}// void loop()

void initializeServo(){ 
  Serial.write("\nServo test\n"); 
  servoGripper.attach(SERVO_PWM_PIN);
}

void setDefaulRoboticArmPosition(){
  setServo(1, DEFAULT_SERVO_ANGLE);
  servoGripper_val = servoGripper.read();
  //currentRoboticArmState = 0;
}

void openGripper(int servo_delay_ms){
  servoGripper_val = servoGripper.read();
  Serial.println("servoGripper: "+String(servoGripper_val));
      Serial.println("open gripper");
      if(servoGripper_val<OPEN_MAX){
        for(servoGripper_val; servoGripper_val<OPEN_MAX; servoGripper_val +=1)// in steps of 1 degree   
        {                                 
          servoGripper.write(servoGripper_val); // tell servo to go to "currentServoPosition" 
          delay(servo_delay_ms);                       // waits 10ms for the servo to reach the position 
          //Serial.println("servoBaseN: "+String(servoGripper_val));
        }// for 
      }// if
      delay(3000); 
}// void openGripper()

void closeGripper(int servo_delay_ms){
  servoGripper_val = servoGripper.read();
  Serial.println("servoGripper: "+String(servoGripper_val));
      Serial.println("close gripper");
      if(servoGripper_val>GRIPPER_MIN){
        for(servoGripper_val; servoGripper_val>GRIPPER_MIN; servoGripper_val -=1)// in steps of 1 degree   
        {                                 
          servoGripper.write(servoGripper_val); // tell servo to go to "currentServoPosition" 
          delay(servo_delay_ms);                       // waits 10ms for the servo to reach the position 
          //Serial.println("servoBaseN: "+String(servoGripper_val));
        }// for 
      }// if
      delay(3000); 
}// void closeGripper()

void processInput (String voice_command){  
if (voice_command.length() > 0) {
    Serial.println(voice_command); 
    
  if (voice_command == "*open") {
    openGripper(servoDelay_ms);
    Serial.println("open gripper");
  }
  else if (voice_command == "*close") {
    closeGripper(servoDelay_ms);
    Serial.println("close gripper");
  }
  else if (voice_command == "*open fast") {
    servoDelay_ms = SERVO_DELAY_FAST;
    openGripper(servoDelay_ms);
    Serial.println("open gripper fast");
  }
  else if (voice_command == "*close fast") {
    servoDelay_ms = SERVO_DELAY_FAST;
    closeGripper(servoDelay_ms);
    Serial.println("close gripper fast");
  }
  else if (voice_command == "*open slowly") {
    servoDelay_ms = SERVO_DELAY_SLOW;
    openGripper(servoDelay_ms);
    Serial.println("open gripper slowly");
  }
  else if (voice_command == "*close slowly") {
    servoDelay_ms = SERVO_DELAY_SLOW;
    closeGripper(servoDelay_ms);
    Serial.println("close gripper slowly");
  }
  else if (voice_command == "*high speed") {
    servoDelay_ms = SERVO_DELAY_FAST;
    Serial.println("high speed: "+String(servoDelay_ms));
  }
  else if (voice_command == "*low speed"){
    servoDelay_ms = SERVO_DELAY_SLOW;
    Serial.println("low speed: "+String(servoDelay_ms));
  }
  else if(voice_command == "*stop") {
    Serial.println("stop");
  }  
 }//Reset the variable after initiating
} // void processInput ()

void setServoBaseDirection(int servoGripperDirection, int servo_delay_ms){
  servoGripper_val = servoGripper.read();
  Serial.println("servoGripper: "+String(servoGripper_val));
  if(servoGripperDirection==1){ // open gripper
      Serial.println("open gripper");
      if(servoGripper_val<GRIPPER_MAX){
        servoGripper_val +=1;                             
        servoGripper.write(servoGripper_val); // tell servo to go to "currentServoPosition" 
        delay(servo_delay_ms);                       // waits 10ms for the servo to reach the position 
      }// if
  }
  else{ // close gripper
      Serial.println("close gripper");
      if(servoGripper_val>GRIPPER_MIN){
        servoGripper_val -=1;                                 
        servoGripper.write(servoGripper_val); // tell servo to go to "currentServoPosition" 
        delay(servo_delay_ms);                       // waits 15ms for the servo to reach the position 
      }// if
  }
  //delay(1);
}// void setServoBasePosition(int servoGripperDirection)

void setServo(int servoNumber, int deg){
  if(servoNumber==1 && deg>=GRIPPER_MIN && deg<=GRIPPER_MAX){
    servoGripper.write(deg);
    servoGripper_val = servoGripper.read();
    Serial.println("servoGripper: "+String(servoGripper_val));    
  }
}// void setServo(int servoNumber, int deg)
