/****************************************************************
I2c_mini_moto_test

Example code demonstrating the use of the Arduino Library for
the SparkFun MiniMoto board, which uses the TI DRV8830 IC for I2C
low-voltage DC motor control.

modified 8 March 2014 by Adam Srebro based on Mike Hord code

Code developed in Arduino 1.0.5, on an Arduino Leonardo board.

PINS CONNECTION:
-----------------------------------------
DRV8830 Motor Driver ->  Arduino Leonardo
-----------------------------------------
Vcc -> Vcc
GND -> GND
SDA -> SDA
SCL -> SCL
FLT -> A2
-----------------------------------------
DRV8830 Motor Driver board operates on power supply voltages from 2.75V to 6.8V!
****************************************************************/

#include "minimoto.h"  // Include the MiniMoto library
#define FAULTn  16     // Pin used for fault detection.
MiniMoto motor1(0xD0); // A1 = 1, A0 = 1 (default address setting)

void setup()
{
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  Serial.println("DRV8830 I2c Motor Driver!");
  pinMode(FAULTn, INPUT);
}

// The loop() function just spins the motors one way, then the other
void loop(){
  Serial.println("Forward");
  motor1.drive(100);
  delay(3000);
  Serial.println("Stop");
  motor1.stop();
  delay(3000);
  Serial.println("Reverse");
  motor1.drive(-10);
  delay(4000);
  Serial.println("Brake");
  motor1.brake();
  delay(3000);
}// void loop()

