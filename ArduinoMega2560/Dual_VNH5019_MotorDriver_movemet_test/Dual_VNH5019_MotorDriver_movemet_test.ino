#include "DualVNH5019MotorShield.h"
#define MAX_SPEED 400
#define HALF_SPEED 200
#define LOW_SPEED 60
#define MOVEMENT_TIME_MS 2000

DualVNH5019MotorShield md;
int speedArray[3] = {MAX_SPEED, HALF_SPEED, LOW_SPEED};
int current_speed;

void stopIfFault(){
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}

void setup(){
  Serial.begin(9600);
  Serial.println("Dual VNH5019 Motor Driver Shield Test");
  md.init();
}

void waitAndStop(int delay_ms){
  delay(delay_ms);
  md.setSpeeds(0, 0);
  delay(200);
}

void loop(){
  // Set speed for motor 1, speed is a number betwenn -400 and 400
  for (int i = 0; i<3; i++){
    current_speed = speedArray[i];
    md.setSpeeds(current_speed, current_speed);
    waitAndStop(MOVEMENT_TIME_MS);
    md.setSpeeds(-current_speed, -current_speed);
    waitAndStop(MOVEMENT_TIME_MS);
    md.setSpeeds(current_speed, -current_speed);
    waitAndStop(MOVEMENT_TIME_MS);
    md.setSpeeds(-current_speed, current_speed);
    waitAndStop(MOVEMENT_TIME_MS);
  }// for

  /*
  for (int i = LOOP_MAX; i >= -LOOP_MAX; i--){
    md.setM1Speed(i);
    stopIfFault();
    if (i%200 == 100)
    {
      Serial.print("M1 current: ");
      Serial.println(md.getM1CurrentMilliamps());
    }
    delay(2);
  }
  
  for (int i = -LOOP_MAX; i <= 0; i++){
    md.setM1Speed(i);
    stopIfFault();
    if (i%200 == 100)
    {
      Serial.print("M1 current: ");
      Serial.println(md.getM1CurrentMilliamps());
    }
    delay(2);
  }
  */
}
