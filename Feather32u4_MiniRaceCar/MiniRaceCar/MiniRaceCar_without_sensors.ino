/*********************************************************************
 
 Mini Race Car kit -- Formula E Feather robot without HTU21D-F 
 Temperature/Humidity sensor
 
 This is an example for our nRF51822 based Bluefruit LE modules
  
 Modified to drive a 3-wheeled BLE Robot Rover! by http://james.devi.to

 Pick one up today in the Adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_SPI.h>

#include "BluefruitConfig.h"

#include <Adafruit_MotorShield.h>

#include <Wire.h>
#include "Adafruit_HTU21DF.h"
// Connect Vin to 3V DC
// Connect GND to ground
// Connect SCL to I2C clock pin 
// Connect SDA to I2C data pin

// Create the temp/humidity sensor object
Adafruit_HTU21DF htu = Adafruit_HTU21DF();


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// And connect 2 DC motors to port M3 & M4 !
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(4);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(3);

//Name your RC here
String BROADCAST_NAME = "Julek Formula Racer";

String BROADCAST_CMD = String("AT+GAPDEVNAME=" + BROADCAST_NAME);

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

char buf[60];

// Set your forward, reverse, and turning speeds
#define ForwardSpeed                255
#define ReverseSpeed                255
#define TurningSpeed                100


/**************************************************************************/
/*!
    @brief  Sets up the HW and the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void) {
  /*
  Serial.begin(9600);
  if (!htu.begin()) { //start the temp/humidity sensor
    Serial.println("Couldn't find sensor!");
    while (1);
  }
  */

  AFMS.begin();  // create with the default frequency 1.6KHz

  // turn on motors
  L_MOTOR->setSpeed(0);
  L_MOTOR->run(RELEASE);

  R_MOTOR->setSpeed(0);
  R_MOTOR->run(RELEASE);
    
  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Robot Controller Example"));
  Serial.println(F("-----------------------------------------"));
  Serial.println(BROADCAST_CMD);

  /* Initialize the module */
  BLEsetup();
}

void loop(void)
{
  // read new packet data
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);

  readController();

  //for the plotter
  /*
  ble.print("Temperature C: "); 
  ble.print(htu.readTemperature()); 
  ble.print("\t");
  
  ble.print("\tHumidity: "); 
  ble.println(htu.readHumidity()); 
  */

}


bool isMoving = false;
unsigned long lastPress = 0;

bool readController(){
  uint8_t maxspeed;

 // Buttons
  if (packetbuffer[1] == 'B') {

    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';

    if (pressed) {
      /*
      if(buttnum == 1){
        ble.print("Temperature C: "); ble.println(htu.readTemperature());
      }
      
      if(buttnum == 2){
        ble.print("Temperature F: "); ble.println((htu.readTemperature()*1.8)+32);
      }

      if(buttnum == 3){
        ble.print("Humidity: "); ble.println(htu.readHumidity());
      }

      if(buttnum == 4){
        
      }
      */
      if(buttnum == 5){
        isMoving = true;
        L_MOTOR->run(FORWARD);
        R_MOTOR->run(FORWARD);
        maxspeed = ForwardSpeed;
        ble.println("Forward");
      }
      
      if(buttnum == 6){
        isMoving = true;
        L_MOTOR->run(BACKWARD);
        R_MOTOR->run(BACKWARD);
        maxspeed = ReverseSpeed;
        ble.println("Backward");        
      }
      
      if(buttnum == 7){
        isMoving = true;
        R_MOTOR->run(RELEASE);
        L_MOTOR->run(FORWARD);
        maxspeed = TurningSpeed;
        ble.println("Left");
      }
      
      if(buttnum == 8){
        isMoving = true;
        R_MOTOR->run(FORWARD);
        L_MOTOR->run(RELEASE);
        maxspeed = TurningSpeed;
        ble.println("Right");        
      }

      lastPress = millis();

      // speed up the motors
      for (int speed=0; speed < maxspeed; speed+=5) {
        L_MOTOR->setSpeed(speed);
        R_MOTOR->setSpeed(speed);
        delay(5); // 250ms total to speed up
      }
  } else {
      isMoving = false;
      // slow down the motors
      for (int speed = maxspeed; speed >= 0; speed-=5) {
        L_MOTOR->setSpeed(speed);
        R_MOTOR->setSpeed(speed);
        delay(5); // 50ms total to slow down
      }
      L_MOTOR->run(RELEASE);
      R_MOTOR->run(RELEASE);
    }
}
}

void BLEsetup(){
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Perform a factory reset to make sure everything is in a known state */
  Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset"));
  }

  //Convert the name change command to a char array
  BROADCAST_CMD.toCharArray(buf, 60);

  //Change the broadcast device name here!
  if(ble.sendCommandCheckOK(buf)){
    Serial.println("name changed");
  }
  delay(250);

  //reset to take effect
  if(ble.sendCommandCheckOK("ATZ")){
    Serial.println("resetting");
  }
  delay(250);

  //Confirm name change
  ble.sendCommandCheckOK("AT+GAPDEVNAME");

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("*****************"));

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("*****************"));
}


