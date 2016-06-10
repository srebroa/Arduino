 /*
Arduino Due + HC-05 bluetooth + servos + oled + IR sensors
- servos controlled via bluetooth

DUE -> HC-05
Tx1 (18) - Rx
Rx1 (19) - Tx
GND - GND
3.3V - VCC

DUE -> OLED 128x64
3.3V - VCC
GND - GND
Scl (21) - Scl
Sda (20) - Sda

DUE -> Analog Distance Sensors (GP2Y0A21YK):
A0 -> Sensor Right
A1 -> Sensor Front
A2 -> Sensor Left

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
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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

//Control State
#define SEMI_AUTONOMOUS 1

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

Servo servoBase;
Servo servoShoulder;
Servo servoElbow;

int servoBase_val;
int servoShoulder_val;
int servoElbow_val;

// Analog IR sensors distance variables
int distanceSensorRight; 
int distanceSensorFront; 
int distanceSensorLeft;
// Analog IR sensors Pins
int distanceSensorRightPin = 0;
int distanceSensorFrontPin = 1;
int distanceSensorLeftPin = 2;
int detectionThreshold;

int controlState = 0;
int baseServoSpeedFlag = 1;


void setup(){
  //pinMode(LED, OUTPUT);
  setOledDisplay();
  //flashLed(300);
  Serial.begin(9600);
  initializeBluetooth();
  initializeServos();
  initializeDistanceSensors();
  delay(2000);
  setDefaulRoboticArmPosition();
}

void initializeBluetooth(){
  HC05.begin(9600);
  HC05.setTimeout(10);
}

void initializeServos(){
  Serial.write("\nServo test\n"); 
  servoBase.attach(BASE_PWM_PIN);
  servoShoulder.attach(SHOULDER_PWM_PIN);
  servoElbow.attach(ELBOW_PWM_PIN);
}

void initializeDistanceSensors(){
  detectionThreshold = 440; // can be changed as needed ;)
  distanceSensorRight = 0;
  distanceSensorFront = 0;
  distanceSensorLeft = 0; 
}

void setDefaulRoboticArmPosition(){
  setServoBasePosition(90);
  setServo(2, 90);
  setServo(3, 90);
  
  servoBase_val = servoBase.read();
  servoShoulder_val = servoShoulder.read();
  servoElbow_val = servoElbow.read();
}

void setOledDisplay(){
    // Set up the display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Initialize with the I2C addr 0x3D if not working use 0x3C (for the 128x64)
  //display.setTextColor(WHITE);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(25,0);
  display.println("instructables");
  display.println();
  display.display();
  //delay(2000);
}

void loop(){
  int detection = readDistanceSensors(); 
  //Serial.println("Detection: " +String(detection));
  if(controlState==SEMI_AUTONOMOUS){
    checkIfObjectWasDetected(detection);
  }
  
  while(HC05.available()){
    processInput();
  }
}// void loop()

void checkIfObjectWasDetected(int detection){
  //int targetServoPosition; 
  if(detection==1){ // position of the right sensor
    //targetServoPosition = 0;
    setServoBasePosition(0);
  }
  else if(detection==2){ // position of the front sensor
    //targetServoPosition = 90;
    setServoBasePosition(90);
  }
  else if(detection==3){ // position of the left sensor
    //targetServoPosition = 180;
    setServoBasePosition(180);
  }
  else{
    // not found any object
    delay(500); 
  }
  //setServoBasePosition(targetServoPosition);
}

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

int readDistanceSensors(){
    int detection = 0;
    distanceSensorRight = analogRead(distanceSensorRightPin);
    distanceSensorFront = analogRead(distanceSensorFrontPin);
    distanceSensorLeft = analogRead(distanceSensorLeftPin);
    
    Serial.println("SensorRight: "+String(distanceSensorRight)+", SensorFront: "+String(distanceSensorFront)+", SensorLeft: "+String(distanceSensorLeft));
    
    if(distanceSensorFront>detectionThreshold){
      detection = 2;
    }
    else if(distanceSensorRight>detectionThreshold){
      detection = 1;
    }  
    else if(distanceSensorLeft>detectionThreshold){
      detection = 3;
    }   
    return detection;
}// int readDistanceSensors() 

void gripperControl(char mdirection, int mspeed){
  if (mdirection == 'o'){
    digitalWrite(GRIPPER_MOTOR_AIN1, LOW); 
    digitalWrite(GRIPPER_MOTOR_AIN2, HIGH); 
    DisplayGripperStateOnOled("open");
  }
  else if (mdirection == 'c'){
    digitalWrite(GRIPPER_MOTOR_AIN1, HIGH); 
    digitalWrite(GRIPPER_MOTOR_AIN2, LOW); 
    DisplayGripperStateOnOled("close");
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

void setServoBasePosition(int targetPosition){
  if(servoBase_val==targetPosition){
    // do nothing
  }
  else if(baseServoSpeedFlag==0){
      if(servoBase_val<targetPosition){
        for(servoBase_val; servoBase_val<targetPosition; servoBase_val +=1)// in steps of 1 degree   
        {                                 
          servoBase.write(servoBase_val); // tell servo to go to "currentServoPosition" 
          delay(15);                       // waits 15ms for the servo to reach the position 
        } 
      }
      
      else if(servoBase_val>targetPosition){
          for(servoBase_val; servoBase_val>targetPosition; servoBase_val -=1)  
          {                                  
            servoBase.write(servoBase_val);              
            delay(15);                      
          } 
      }
  }
  else{
          servoBase.write(targetPosition);      // tell servo to go to "targetPosition"
          delay(100);
          servoBase_val=targetPosition;
  }
  delay(1000);
}// void setServoPosition(int targetPosition)

void setServo(int servoNumber, int deg){
  if(servoNumber==1 && deg>=BASE_MIN && deg<=BASE_MAX){
    servoBase.write(deg);
    servoBase_val = servoBase.read();
    Serial.println("servoBase: "+String(servoBase_val));    
    DisplayAngleOnOled("B:",servoBase_val);
  }
  else if(servoNumber==2 && deg>=SHOULDER_MIN && deg<=SHOULDER_MAX){
    servoShoulder.write(deg);
    servoShoulder_val = servoShoulder.read();
    Serial.println("servoShoulder: "+String(servoShoulder_val));   
    DisplayAngleOnOled("S:",servoShoulder_val); 
  }
  else if(servoNumber==3 && deg>=ELBOW_MIN && deg<=ELBOW_MAX){
    servoElbow.write(deg);
    servoElbow_val = servoElbow.read();
    Serial.println("servoElbow: "+String(servoElbow_val));    
    DisplayAngleOnOled("E:",servoElbow_val); 
  }
}// void setServo(int servoNumber, int deg)

void DisplayAngleOnOled(String armString, int agle){
    display.clearDisplay();
    display.setTextSize(3);
    display.setTextColor(WHITE);
    display.setCursor(20,20);
    display.print(armString);
    display.println(agle);
    display.println();
    display.display();
}

void DisplayGripperStateOnOled(String gripperStateString){    
    display.clearDisplay();
    display.setTextSize(3);
    display.setTextColor(WHITE);
    display.setCursor(20,20);
    display.print(gripperStateString);
    display.println();
    display.display();
}



