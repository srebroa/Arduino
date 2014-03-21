/***********************************************************************
I2c_PCF8574_expander_test

Example of I2C I/O expander (PCF8574AP) usage

PINS CONNECTION:
-------------------------------
*Power Supply -> PCF8574A
-------------------------------
GND -> Pins: 1,2,3,8 (A0,A1,A2,GND) - to get the address 0x38
Vcc -> Pin 16 (Vcc)
-------------------------------
*Arduino Leonardo -> PCF8574A
-------------------------------
SDA -> Pin 15 (SDA)
SCL -> Pin 14 (SCL)
----------------------------------------
*Button (or digital sensor) -> PCF8574A
----------------------------------------
sensor output (or button) -> Pin 6 (P2)

Do not forget to connect two 4k7 resistors to SDA, SCL and Vcc (it is required for I2C bus)!
Code developed in Arduino 1.0.5, on an Arduino Leonardo board.
***********************************************************************/
#include <PCF8574.h>
#include <Wire.h>

PCF8574 io_expander;

void setup() {
  io_expander.begin(0x38); // only for PCF8574AP
  io_expander.pinMode(2, INPUT); // connect the digital sensor or button to P2 (Pin 6 of PCF8574) 
  io_expander.pullUp(2);
  Serial.begin(9600);
    while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
}// void setup()

void loop() {
  Serial.print("Read: ");
  byte value = io_expander.digitalRead(2);
  Serial.println(value, DEC);
  delay(1000);
}// void loop()

