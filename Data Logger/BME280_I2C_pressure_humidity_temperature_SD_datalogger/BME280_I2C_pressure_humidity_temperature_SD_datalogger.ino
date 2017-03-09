/*BME280_I2C_pressure_humidity_temperature_SD_datalogger - Print and save to SD card: temperature, humidity, pressure and altitude from the BME280 sensor
Connections:
 BME280 -> Arduino Mega 2560
 1. GND - GND
 2. 3.3V - 3.3V 
 3. SDA - SDA (20)
 4. SCL - SCL (21)
 SD card attached to SPI bus as follows (Arduino Mega 2560):
 ** MISO - pin 50
 ** MOSI - pin 51
 ** CLK - pin 52
 ** CS - pin 53
 CS pin 53 on the Mega must be left as an output or the SD library functions will not work.
 BME280 based on samples from https://github.com/sparkfun/SparkFun_BME280_Arduino_Library
*/
#include <stdint.h>
#include "SparkFunBME280.h"
//Library allows either I2C or SPI
#include "Wire.h"
#include "SPI.h"
// SD Card
#include <SPI.h>
#include <SD.h>
//Global sensor object
BME280 mySensor;
const int chipSelect = 53;
String dataString = "";

void setup(){
	BME280settings();
        SDcardInit();
	Serial.begin(9600);
	Serial.print("Sensor BME280, result of .begin(): 0x");	
	delay(10);  //BME280 requires at least 2ms to start up.
	Serial.println(mySensor.begin(), HEX); //Calling .begin() causes the settings to be loaded
}// void setup()

void loop(){
	//Start with temperature, as that data is needed for accurate compensation - updates the compensators of the other functions.
	Serial.println("Read data from sensor and save to SD card");
	processSensorInput();
        writeToSDcard(dataString);
	delay(2000);
}// void loop()

void processSensorInput(){
	String temp = String(mySensor.readTempC(), 2);
	String humidity = String(mySensor.readFloatHumidity(), 2);
	String pressure = String(mySensor.readFloatPressure(), 2);
	String altitudes = String(mySensor.readFloatAltitudeMeters(), 2);
        dataString = temp+", "+humidity+", "+pressure+", "+altitudes;
}// void processSensorInput()

void printSensorMeasurements(){
  	Serial.print("Temperature: ");
	Serial.print(mySensor.readTempC(), 2);
	Serial.print(" C, ");
	Serial.print(mySensor.readTempF(), 2);
	Serial.print(" F; ");

	Serial.print("Humidity: ");
	Serial.print(mySensor.readFloatHumidity(), 2);
	Serial.print(" %; ");

	Serial.print("Pressure: ");
	Serial.print(mySensor.readFloatPressure(), 2);
	Serial.print(" Pa; ");

        Serial.print("Altitude: ");
	Serial.print(mySensor.readFloatAltitudeMeters(), 2);
	Serial.print(" m, ");
	Serial.print(mySensor.readFloatAltitudeFeet(), 2);
	Serial.println(" ft");	
}// void printSensorMeasurements()

void BME280settings(){
  	//specify I2C address.  Can be 0x77(default) or 0x76	
	//For I2C, enable the following and disable the SPI section
	mySensor.settings.commInterface = I2C_MODE;
	mySensor.settings.I2CAddress = 0x77;
	mySensor.settings.runMode = 3; //Normal mode
	mySensor.settings.tStandby = 0; //0.5ms
	mySensor.settings.filter = 0; //filter off
	mySensor.settings.tempOverSample = 1;
        mySensor.settings.pressOverSample = 1;
	mySensor.settings.humidOverSample = 1;
}// void BME280settings()

void SDcardInit(){
  Serial.print("Initializing SD card...");
  pinMode(chipSelect, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    return;
  }
  Serial.println("card initialized.");
}// void SDcardInit()

void writeToSDcard(String currentDataString){
  File dataFile = SD.open("datalog.csv", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(currentDataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(currentDataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.csv");
  }
}// void writeToSDcard()

