/* 
 *  Testing MiniRaceCar Movement - turn right and left with the speed change
 *  
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
// Set up the left motor on port M4
Adafruit_DCMotor *leftMotor = AFMS.getMotor(4);
// Set up the right motor on port M3
Adafruit_DCMotor *rightMotor = AFMS.getMotor(3);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(FORWARD); //LEFT MOTOR FULL STEAM AHEAD!
  rightMotor->run(BACKWARD); //RIGHT MOTOR FULL STEAM AHEAD!
  // turn on motor
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

void loop() {
  uint8_t i;
  
  Serial.print("tick");

  rightMotor->run(FORWARD);
  leftMotor->run(BACKWARD);
  for (i=0; i<255; i++) {
    rightMotor->setSpeed(i); 
    leftMotor->setSpeed(i);   
    delay(10);
  }
  for (i=255; i!=0; i--) {
    rightMotor->setSpeed(i); 
    leftMotor->setSpeed(i);    
    delay(10);
  }
  
  Serial.print("tock");

  rightMotor->run(BACKWARD);
  leftMotor->run(FORWARD);
  for (i=0; i<255; i++) {
    rightMotor->setSpeed(i); 
    leftMotor->setSpeed(i);   
    delay(10);
  }
  for (i=255; i!=0; i--) {
    rightMotor->setSpeed(i); 
    leftMotor->setSpeed(i);    
    delay(10);
  }

  Serial.print("tech");
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  delay(5000);
}
