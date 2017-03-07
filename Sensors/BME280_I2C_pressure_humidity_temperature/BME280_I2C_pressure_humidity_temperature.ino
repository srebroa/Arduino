/*BME280_I2C_pressure_humidity_temperature - Print the: temperature, humidity, pressure and altitude from the BME280 sensor
Connections:
 BME280 -> Arduino Mega 2560
 1. GND - GND
 2. 3.3V - 3.3V 
 3. SDA - SDA (20)
 4. SCL - SCL (21)
based on samples from https://github.com/sparkfun/SparkFun_BME280_Arduino_Library
*/
#include <stdint.h>
#include "SparkFunBME280.h"
//Library allows either I2C or SPI
#include "Wire.h"
#include "SPI.h"
//Global sensor object
BME280 mySensor;

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

void setup(){
	BME280settings();
	Serial.begin(9600);
	Serial.print("Sensor BME280, result of .begin(): 0x");	
	//Calling .begin() causes the settings to be loaded
	delay(10);  //BME280 requires at least 2ms to start up.
	Serial.println(mySensor.begin(), HEX);
}// void setup()

void loop(){
	//Start with temperature, as that data is needed for accurate compensation - updates the compensators of the other functions.
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
	delay(2000);
}// void loop()
