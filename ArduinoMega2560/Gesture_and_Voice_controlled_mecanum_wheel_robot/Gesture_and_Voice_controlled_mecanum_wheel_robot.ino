/*
MECANUM WHEEL ROBOT - VOICE and GESTURE CONTROLLED v1.0
 - Allows you to control a mecanum robot via gesture and voice commands
 - Tested with Arduino Mega 2560
 - Android application - https://play.google.com/store/apps/details?id=robotspace.simplelabs.amr_voice
 - Project description - http://www.instructables.com/id/Mecanum-wheel-robot-bluetooth-controlled
 Author: Adam Srebro
 www: http://www.mobilerobots.pl/
 
 Connections:
 Bluetooth (e.g HC-06)-> Arduino Mega 2560
 TXD - TX1 (19)
 RXD - RX1 (18)
 VCC - 5V
 GND - GND
 
 TB6612FNG Dual Motor Driver -> Arduino Mega 2560
 //PWM control
 RightFrontMotor_PWMA - 2
 LeftFrontMotor_PWMB - 3
 RightRearMotor_PWMA - 4
 LeftRearMotor_PWMB - 5
 //Control of rotation direction
 RightFrontMotor_AIN1 - 22
 RightFrontMotor_AIN2 - 23
 LeftFrontMotor_BIN1 - 24
 LeftFrontMotor_BIN2 - 25
 RightRearMotor_AIN1 - 26
 RightRearMotor_AIN2 - 27
 LeftRearMotor_BIN1 - 28
 LeftRearMotor_BIN2 - 29
 //The module and motors power supply
 STBY - Vcc
 VMOT - motor voltage (4.5 to 13.5 V) - 11.1V from LiPo battery
 Vcc - logic voltage (2.7 to 5.5) - 5V from Arduino
 GND - GND
 
 TB6612FNG Dual Motor Driver -> DC Motors
 MotorDriver1_AO1 - RightFrontMotor
 MotorDriver1_A02 - RightFrontMotor
 MotorDriver1_B01 - LeftFrontMotor
 MotorDriver1_B02 - LeftFrontMotor
 
 MotorDriver2_AO1 - RightRearMotor
 MotorDriver2_A02 - RightRearMotor
 MotorDriver2_B01 - LeftRearMotor
 MotorDriver2_B02 - LeftRearMotor
 
 Arduino Pin  APDS-9930 Board  Function 
 3.3V         VCC              Power
 GND          GND              Ground
 A4           SDA              I2C Data
 A5           SCL              I2C Clock
 */
#include <Wire.h>
#include <math.h>
#include <APDS9930.h>
#define DUMP_REGS

// APDS-9930 Global Variables
APDS9930 apds = APDS9930();
float ambient_light = 0; // can also be an unsigned long
uint16_t ch0 = 0;
uint16_t ch1 = 1;
/*TB6612FNG Dual Motor Driver Carrier*/
const int RightFrontMotor_PWM = 2; // pwm output
const int LeftFrontMotor_PWM = 3; // pwm output 
const int RightRearMotor_PWM = 4; // pwm output
const int LeftRearMotor_PWM = 5; // pwm output 
//Front motors
const int RightFrontMotor_AIN1 = 22; // control Input AIN1 - right front motor 
const int RightFrontMotor_AIN2 = 23; // control Input AIN2 - right front motor
const int LeftFrontMotor_BIN1 = 24; // control Input BIN1 - left front motor
const int LeftFrontMotor_BIN2 = 25; // control Input BIN2 - left front motor
//Rear motors
const int RightRearMotor_AIN1 = 26; // control Input AIN1 - right rear motor 
const int RightRearMotor_AIN2 = 27; // control Input AIN2 - right rear motor
const int LeftRearMotor_BIN1 = 28; // control Input BIN1 - left rear motor
const int LeftRearMotor_BIN2 = 29; // control Input BIN2 - left rear  motor

long pwmLvalue = 255;
long pwmRvalue = 255;
int maxPWM = 255;
int minPWM = 50;
int currentPWM = 100;
float maxAmbient;
float minAmbient = 20.0;
int deltaAmbient;
int deltaPWM;

byte pwmChannel;
String voice;
const int defaultDelayBeforeStopping = 1500; //1.5 s
const int longDelayBeforeStopping = 20000; //1.5 s

void setup(){
  Serial1.begin(9600);// HC-06 default baudrate: 9600
  APDS_9930_Init();

  //Setup RightFrontMotor 
  pinMode(RightFrontMotor_AIN1, OUTPUT); //Initiates Motor Channel A1 pin
  pinMode(RightFrontMotor_AIN2, OUTPUT); //Initiates Motor Channel A2 pin

  //Setup LeftFrontMotor 
  pinMode(LeftFrontMotor_BIN1, OUTPUT); //Initiates Motor Channel B1 pin
  pinMode(LeftFrontMotor_BIN2, OUTPUT); //Initiates Motor Channel B2 pin

  //Setup RightFrontMotor 
  pinMode(RightRearMotor_AIN1, OUTPUT); //Initiates Motor Channel A1 pin
  pinMode(RightRearMotor_AIN2, OUTPUT); //Initiates Motor Channel A2 pin

  //Setup LeftFrontMotor 
  pinMode(LeftRearMotor_BIN1, OUTPUT); //Initiates Motor Channel B1 pin
  pinMode(LeftRearMotor_BIN2, OUTPUT); //Initiates Motor Channel B2 pin  
  Wire.begin();
  delay(500);
  maxAmbient = getMaxAmbientLightLevelInCurrentConditions();
  computeDeltaVariables(maxAmbient);
}// void setup()

void loop(){
  while (Serial1.available()) {
    delay(10); //Delay added to make thing stable 
    char c = Serial1.read(); //Conduct a serial read
    if (c == '#') {
      break; //Exit the loop when the # is detected after the word
    } 
    voice += c; //Shorthand for voice = voice + c
  }
  //processLightLevel();
  //float ambientLevel = getAmbientLightLevel();
  currentPWM = computePWMbasedOnAmbientLight(getAmbientLightLevel()); 
  Serial.print("PWM: ");
  Serial.println(currentPWM);
  processInput(voice, currentPWM);
  voice="";
}// void loop()

void APDS_9930_Init(){
  // Initialize Serial port
  Serial.begin(9600);
  Serial.println(F("APDS-9930 - Ambient light sensor"));
  // Initialize APDS-9930 (configure I2C and initial values)
  if ( apds.init() ) {
    Serial.println(F("APDS-9930 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9930 init!"));
  }
  
  // Start running the APDS-9930 light sensor (no interrupts)
  if ( apds.enableLightSensor(false) ) {
    Serial.println(F("Light sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during light sensor init!"));
  }
#ifdef DUMP_REGS
  /* Register dump */
  uint8_t reg;
  uint8_t val;

  for(reg = 0x00; reg <= 0x19; reg++) {
    if( (reg != 0x10) && \
        (reg != 0x11) )
    {
      apds.wireReadDataByte(reg, val);
      Serial.print(reg, HEX);
      Serial.print(": 0x");
      Serial.println(val, HEX);
    }
  }
  apds.wireReadDataByte(0x1E, val);
  Serial.print(0x1E, HEX);
  Serial.print(": 0x");
  Serial.println(val, HEX);
#endif
  // Wait for initialization and calibration to finish
  delay(500);  
}// void APDS_9930_Init()

int computePWMbasedOnAmbientLight(float currentAmbient){
  int currentPWM = (int)(minPWM + ((currentAmbient - minAmbient)*deltaPWM)/deltaAmbient);
  if(currentPWM > maxPWM){
    currentPWM = maxPWM;
  }
  else if(currentPWM < minPWM){
    currentPWM = minPWM;
  }
  return currentPWM;  
}

void computeDeltaVariables(float maxAmbient){
  deltaAmbient = maxAmbient - minAmbient;
  deltaPWM = maxPWM - minPWM;
}

void motorControl(String motorStr,int mdirection, int mspeed){
  int IN1;
  int IN2;
  int motorPWM;
  if (motorStr == "rf") {       //right front
    IN1 = RightFrontMotor_AIN1; 
    IN2 = RightFrontMotor_AIN2;
    motorPWM = RightFrontMotor_PWM;
  }   
  else if (motorStr == "lf") { //left front
    IN1 = LeftFrontMotor_BIN1; 
    IN2 = LeftFrontMotor_BIN2;
    motorPWM = LeftFrontMotor_PWM;
  }
  else if (motorStr == "rr") {
    IN1 = RightRearMotor_AIN1; 
    IN2 = RightRearMotor_AIN2;
    motorPWM = RightRearMotor_PWM;
  }
  else if (motorStr == "lr") {
    IN1 = LeftRearMotor_BIN1; 
    IN2 = LeftRearMotor_BIN2;
    motorPWM = LeftRearMotor_PWM;
  }
  if (mdirection == 1){
    digitalWrite(IN1, LOW); 
    digitalWrite(IN2, HIGH); 
  }
  else if (mdirection == -1){
    digitalWrite(IN1, HIGH); 
    digitalWrite(IN2, LOW); 
  }
  analogWrite(motorPWM, mspeed);
}

void processLightLevel(){
  if (!apds.readAmbientLightLux(ambient_light)) {
    Serial.println(F("Error reading light values"));
  }
  else {
    Serial.print(F("Ambient: "));
    Serial.println(ambient_light);
  } 
  // Wait 1 second before next reading
  delay(200);
}// void processLightLevel()

float getAmbientLightLevel(){
  // Read the light levels (ambient)
  float outputValue;
  if (!apds.readAmbientLightLux(ambient_light)){
     outputValue = 0;
     //Serial.println(F("Error reading light values"));
  }
  else {
    outputValue = ambient_light;
  } 
  return outputValue; 
}// float getAmbientLightLevel()

float getMaxAmbientLightLevelInCurrentConditions(){
  // Read the light levels (ambient)
  float outputValue = 0.0; 
  float sumOfMeasurements = 0.0; 
  int counter = 0;
  for (int i=1; i <= 10; i++){
    if (!apds.readAmbientLightLux(ambient_light)){
      //outputValue = 0;
    }
    else {
      sumOfMeasurements += ambient_light;
      counter++;
      Serial.print(F("INIT Ambient: "));
      Serial.println(ambient_light);
    } 
      delay(50);
  }// for 
  outputValue = sumOfMeasurements/counter;
  Serial.print("maxAmbient: ");
  Serial.println(outputValue);
  return outputValue; 
}// getMaxAmbientLightLevelInCurrentConditions()

void processInput (String voice_command, int currentPWM){
  
if (voice_command.length() > 0) {
    Serial.println(voice_command); 
    
  if(voice_command == "*stop") {
    hardStop();
  }  
  else if (voice_command == "*go forward") {
    //goForward(currentPWM);
    selectMovement("goForward", currentPWM);
    waitAndStop(defaultDelayBeforeStopping);
  }
  else if (voice_command == "*go backward") {
    //goBackwad(currentPWM);
    selectMovement("goBackwad", currentPWM);
    waitAndStop(defaultDelayBeforeStopping);
  }
  else if (voice_command == "*go forward and go backward") {
    //goForward(currentPWM);
    selectMovement("goForward", currentPWM);
    waitAndStop(defaultDelayBeforeStopping);
    //goBackwad(currentPWM);
    selectMovement("goBackwad", currentPWM);
    waitAndStop(defaultDelayBeforeStopping);
  }
  else if (voice_command == "*move right") {
    //moveRight(currentPWM);
    selectMovement("moveRight", currentPWM);
    waitAndStop(defaultDelayBeforeStopping);
  }
  else if (voice_command == "*move left") {
    //moveLeft(currentPWM);
    selectMovement("moveLeft", currentPWM);
    waitAndStop(defaultDelayBeforeStopping);
  }
  else if (voice_command == "*move right and move left") {
    //moveRight(currentPWM);
    selectMovement("moveRight", currentPWM);
    waitAndStop(defaultDelayBeforeStopping);
    //moveLeft(currentPWM);
    selectMovement("moveLeft", currentPWM);
    waitAndStop(defaultDelayBeforeStopping);
  }
  else if (voice_command == "*top right") {
    //moveRightForward(currentPWM);
    selectMovement("moveRightForward", currentPWM);
    waitAndStop(defaultDelayBeforeStopping);
  }
  else if (voice_command == "*top left") {
    //moveLeftForward(currentPWM);
    selectMovement("moveLeftForward", currentPWM);
    waitAndStop(defaultDelayBeforeStopping);
  }
  else if (voice_command == "*top right and top left") {
    //moveRightForward(currentPWM);
    selectMovement("moveRightForward", currentPWM);
    waitAndStop(defaultDelayBeforeStopping);
    //moveLeftForward(currentPWM);
    selectMovement("moveLeftForward", currentPWM);
    waitAndStop(defaultDelayBeforeStopping);
  }
  else if (voice_command == "*bottom right") {
    //moveRightBackward(currentPWM);
    selectMovement("moveRightBackward", currentPWM);
    waitAndStop(defaultDelayBeforeStopping);
  }
  else if (voice_command == "*bottom left") {
    //moveLeftBackward(currentPWM);
    selectMovement("moveLeftBackward", currentPWM);
    waitAndStop(defaultDelayBeforeStopping);
  }
  else if (voice_command == "*bottom right and bottom left") {
    //moveRightBackward(currentPWM);
    selectMovement("moveRightBackward", currentPWM);
    waitAndStop(defaultDelayBeforeStopping);
    //moveLeftBackward(currentPWM);
    selectMovement("moveLeftBackward", currentPWM);
    waitAndStop(defaultDelayBeforeStopping);
  }
  else if (voice_command == "*turn right") {
    //turnRight(currentPWM);
    selectMovement("turnRight", currentPWM);
    waitAndStop(longDelayBeforeStopping);
  }
  else if (voice_command == "*turn left") {
    //turnLeft(currentPWM);
    selectMovement("turnLeft", currentPWM);
    waitAndStop(longDelayBeforeStopping);
  }
  else if (voice_command == "*turn right and turn left") {
    //turnRight(currentPWM);
    selectMovement("turnRight", currentPWM);
    waitAndStop(defaultDelayBeforeStopping);
    //turnLeft(currentPWM);
    selectMovement("turnLeft", currentPWM);
    waitAndStop(defaultDelayBeforeStopping);
  }
 }//Reset the variable after initiating
} // void processInput ()

void waitAndStop(int time_ms){
  int sensorReadTimeout = 3; // 3ms
  int maxTime = (int)time_ms/sensorReadTimeout;
  for (int i=0; i < maxTime; i++){
    currentPWM = computePWMbasedOnAmbientLight(getAmbientLightLevel());
    updatePWM(currentPWM);
    delay(sensorReadTimeout);
  }
  hardStop(); 
}

void updatePWM(int pwm_num){
  analogWrite(RightFrontMotor_PWM, pwm_num);
  analogWrite(LeftFrontMotor_PWM, pwm_num);
  analogWrite(RightRearMotor_PWM, pwm_num);
  analogWrite(LeftRearMotor_PWM, pwm_num);
}

void selectMovement (String movement, int mspeed){
  if(movement == "goForward"){
    motorControl("rf", 1, mspeed);
    motorControl("lf", 1, mspeed);
    motorControl("rr", 1, mspeed);
    motorControl("lr", 1, mspeed);
  }// void goForward(int mspeed)
  
  else if(movement == "goBackwad"){
    motorControl("rf", -1, mspeed);
    motorControl("lf", -1, mspeed);
    motorControl("rr", -1, mspeed);
    motorControl("lr", -1, mspeed);
  }// void goBackwad(int mspeed)
  
  else if(movement == "moveRight"){
    motorControl("rf", -1, mspeed);
    motorControl("lf", 1, mspeed);
    motorControl("rr", 1, mspeed);
    motorControl("lr", -1, mspeed);
  }// void moveRight(int mspeed)
  
  else if(movement == "moveLeft"){
    motorControl("rf", 1, mspeed);
    motorControl("lf", -1, mspeed);
    motorControl("rr", -1, mspeed);
    motorControl("lr", 1, mspeed);
  }// void moveLeft(int mspeed)
  
  else if(movement == "moveRightForward"){
    motorControl("rf", 1, 0);
    motorControl("lf", 1, mspeed);
    motorControl("rr", 1, mspeed);
    motorControl("lr", 1, 0);
  }// void  moveRightForward(int mspeed)
  
  else if(movement == "moveRightBackward"){
    motorControl("rf", -1, mspeed);
    motorControl("lf", 1, 0);
    motorControl("rr", 1, 0);
    motorControl("lr", -1, mspeed);
  }// void  moveRightBackward(int mspeed)
  
  else if(movement == "moveLeftForward"){
    motorControl("rf", 1, mspeed);
    motorControl("lf", 1, 0);
    motorControl("rr", 1, 0);
    motorControl("lr", 1, mspeed);
  }// void  moveLeftForward(int mspeed)
  
  else if(movement == "moveLeftBackward"){
    motorControl("rf", 1, 0);
    motorControl("lf", -1, mspeed);
    motorControl("rr", -1, mspeed);
    motorControl("lr", 1, 0);
  }// void  moveLeftBackward(int mspeed)
  
  else if(movement == "turnRight"){
    motorControl("rf", -1, mspeed);
    motorControl("lf", 1, mspeed);
    motorControl("rr", -1, mspeed);
    motorControl("lr", 1, mspeed);
  }// void turnRight(int mspeed)
  
  else if(movement == "turnLeft"){
    motorControl("rf", 1, mspeed);
    motorControl("lf", -1, mspeed);
    motorControl("rr", 1, mspeed);
    motorControl("lr", -1, mspeed);
  }// void turnRight(int mspeed)
}
void stopRobot(int delay_ms){
  analogWrite(RightFrontMotor_PWM, 0);
  analogWrite(LeftFrontMotor_PWM, 0);
  analogWrite(RightRearMotor_PWM, 0);
  analogWrite(LeftRearMotor_PWM, 0);
  delay(delay_ms);
}// void stopRobot(int delay_ms)

void hardStop(){
  analogWrite(RightFrontMotor_PWM, 0);
  analogWrite(LeftFrontMotor_PWM, 0);
  analogWrite(RightRearMotor_PWM, 0);
  analogWrite(LeftRearMotor_PWM, 0);
}// void stopRobot()

void SetPWM (const long pwm_num, byte pwm_channel){
  if(pwm_channel==1){ // DRIVE MOTOR
    analogWrite(RightFrontMotor_PWM, pwm_num);
    pwmRvalue = pwm_num;
  }
  else if(pwm_channel==2){ // STEERING MOTOR
    analogWrite(LeftFrontMotor_PWM, pwm_num);
    pwmLvalue = pwm_num;
  }
}// void SetPWM (const long pwm_num, byte pwm_channel)
