/*
 Bluetooth SD card datalogger

 This example shows how to log events from Bluetooth to an SD card using the SD library.

 SD card attached to SPI bus as follows (Arduino Mega 2560):
 ** MISO - pin 50
 ** MOSI - pin 51
 ** CLK - pin 52
 ** CS - pin 53
 */

#include <SPI.h>
#include <SD.h>

// CS pin 53 on the Mega must be left as an output or the SD library functions will not work.
const int chipSelect = 53;
String dataString = "";
byte pwmChannel;
const char startOfNumberDelimiter = '<';
const char endOfNumberDelimiter = '>';

void setup(){
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  Serial1.begin(19200);//for BTM222
  SDcardInit();
}

void loop(){
  if (Serial1.available()) {
    processInput();
    writeToSDcard();
    //readIRsensors();
  }
}// void loop()

void SDcardInit(){
  Serial.print("Initializing SD card...");
  pinMode(chipSelect, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
}// void SDcardInit()

void writeToSDcard(){
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}// void writeToSDcard()

void processInput(){
  static long receivedNumber = 0;
  static boolean negative = false;
  byte c = Serial1.read();
  //Serial.println(c);

  switch (c){
  case endOfNumberDelimiter:
    if (negative)
      SetPWM(- receivedNumber, pwmChannel);
    else
      SetPWM(receivedNumber, pwmChannel);

    // fall through to start a new number
  case startOfNumberDelimiter:
    receivedNumber = 0;
    negative = false;
    pwmChannel = 0;
    break;

  case 'f': // Go FORWARD
    dataString = "forward";
    //Serial.println("forward");
    break;

  case 'b': // Go BACK
    dataString = "backward";
    //Serial.println("backward");
    break;

  case 'r':
    dataString = "RIGHT";
    //Serial.println("RIGHT");
    break;

  case 'l':
    dataString = "LEFT";
    //Serial.println("LEFT");
    break;
    
  case '0' ... '9':
    receivedNumber *= 10;
    receivedNumber += c - '0';
    break;

  case '-':
    negative = true;
    break;
  } // end of switch
} // void processInput ()

void SetPWM (const long pwm_num, byte pwm_channel){
  if(pwm_channel==1){ 
    dataString = "PWM Right: "+pwm_num;
  }
  else if(pwm_channel==2){ 
    dataString = "PWM Left: "+pwm_num;
  }
}// void SetPWM (const long pwm_num, byte pwm_channel)
