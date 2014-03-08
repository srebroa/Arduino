/*
 SD_card_AnalogSensors_Datalogger
 ---------------------------------
 This example shows how to log data from two analog sensors 
 to an SD card using the SD library.
 	
 The circuit:
 * Arduino Pro Micro board(Sparkfun) -> Analog Sensors:
 - A0 -> Sensor1
 - A1 -> Sensor2
 * Arduino Pro Micro board (Sparkfun) -> SD card shield:
 - pin 16 -> MOSI 
 - pin 14 -> MISO 
 - pin 15 -> CLK 
 - pin 10 -> CS
 
 modified 7 March 2014
 by Adam Srebro
 
 This example code is in the public domain.	 
 */

#include <SD.h>

// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.
const int chipSelect = 4;
int sensor1;
int sensor2;

void setup(){
 // Open serial communications and wait for port to open:
  sensor1 = 0;
  sensor2 = 0;
  Serial.begin(9600);
   while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
}// void setup()

void loop(){
  // make a string for assembling the data to log:
  String dataString = "";
  // read two sensors and append to the string:
  sensor1 = analogRead(0);
  sensor2 = analogRead(1);
  dataString = "Sensor1: "+String(sensor1)+", Sensor2: "+String(sensor2); 

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
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
  
  delay(1000); //waits for a second
}// void loop()









