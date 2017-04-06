/*
MobileMeasuringStation - allows you to measure the following physical quantities:
- temperature,
- humidity,
- gases: 
       - LPG,
       - Propane, 
       - Hydrogen and Methane
- position coordinates

Connections MQ-2 -> Arduino Mega 2560
OUT -> A0
VCC -> +5V
GND -> GND

Connections SD card shield -> Arduino Mega 2560:
** MISO - pin 50
** MOSI - pin 51
** CLK - pin 52
** CS - pin 53

Connections KY-015 -> Arduino Mega 2560
S -> pin 8
middle -> +5V
- -> GND
*/

#include <stdlib.h>
// GPS libraries
#include <TinyGPS.h>
// SD card libraries
#include <SPI.h>
#include <SD.h>
// Timer
//#include <SimpleTimer.h>
#define POWER_LED 48
#define FAULT_LED 49
#define LEDS_DELAY_MS 1000
#define GPS_PARSE_TIME 2000

//RGB LED
int redPin = 3;   // Red LED,   connected to digital pin 3
int greenPin = 4;  // Green LED, connected to digital pin 4
int bluePin = 5;  // Blue LED,  connected to digital pin 5

// Color arrays
int black[3]  = { 0, 0, 0 };
int white[3]  = { 100, 100, 100 };
int red[3]    = { 100, 0, 0 };
int green[3]  = { 0, 100, 0 };
int blue[3]   = { 0, 0, 100 };
int yellow[3] = { 40, 95, 0 };
int dimWhite[3] = { 30, 30, 30 };
//RGB LED END ---

// BUZZER
const int buzzerPin = 6;

//MQ-2 Smoke Sensor
const int MQ2sensorPin= A0;
int smoke_level;
byte mapped_smoke_level;
char info[96];

//DHT11 (KY-015) Sensor
int DHpin = 8; //Digital Pin
byte dat [5];
//GPS init
TinyGPS gps; 
static char dtostrfbuffer[20];
//int CS = 53;
String SD_date_time = "invalid";
String SD_lat = "invalid";
String SD_lon = "invalid";
//int gps_parse_time = 2000

// CS pin 53 on the Mega must be left as an output or the SD library functions will not work.
const int chipSelect = 53;
//String dataString = "";

const char startOfNumberDelimiter = '<';
const char endOfNumberDelimiter = '>';
String dataString = "";


void setup() {
  RGBledInit();
  BuzzerInit();
  Serial.begin(9600);
  Serial.println("DHT11 Test");
  //Serial1.begin(19200);//9600 for HC-05, 115200 for BTM777, 19200 for BTM222
  Serial2.begin(9600); //GPS
  SDcardInit();
  DHT11sensorIinit();
  MQ2sensorInit();
  RGBLedTest(LEDS_DELAY_MS);
  BuzzerTest(500);
}

void loop() {
  processTemperatureHumidity();
  processSmokeSensor();
  processGPS();
  delay(250);
  //delay(1000);
}// void loop()

byte readDHT11data () {
  byte data;
  for (int i = 0; i < 8; i ++) {
    if (digitalRead (DHpin) == LOW) {
      while (digitalRead (DHpin) == LOW); // wait for 50us
      delayMicroseconds (30); // determine the duration of the high level to determine the data is '0 'or '1'
      if (digitalRead (DHpin) == HIGH)
        data |= (1 << (7-i)); // high front and low in the post
      while (digitalRead (DHpin) == HIGH); // data '1 ', wait for the next one receiver
     }
  }
return data;
}

void DHT11sensorIinit(){
  pinMode (DHpin, OUTPUT);
}

void MQ2sensorInit(){
  pinMode(MQ2sensorPin, INPUT);;
}

void RGBledInit() {
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);  
}

void BuzzerInit(){
   pinMode(buzzerPin, OUTPUT);
}

void setColor(int red, int green, int blue)
{
  #ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  #endif
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}

byte read_data () {
  byte data;
  for (int i = 0; i < 8; i ++) {
    if (digitalRead (DHpin) == LOW) {
      while (digitalRead (DHpin) == LOW); // wait for 50us
      delayMicroseconds (30); // determine the duration of the high level to determine the data is '0 'or '1'
      if (digitalRead (DHpin) == HIGH)
        data |= (1 << (7-i)); // high front and low in the post
      while (digitalRead (DHpin) == HIGH); // data '1 ', wait for the next one receiver
     }
  }
return data;
}

void processTemperatureHumidity() {
  digitalWrite (DHpin, LOW); // bus down, send start signal
  delay (30); // delay greater than 18ms, so DHT11 start signal can be detected
 
  digitalWrite (DHpin, HIGH);
  delayMicroseconds (40); // Wait for DHT11 response
  
  pinMode (DHpin, INPUT);
  while (digitalRead (DHpin) == HIGH);
  delayMicroseconds (80); // DHT11 response, pulled the bus 80us
  if (digitalRead (DHpin) == LOW);
  delayMicroseconds (80); // DHT11 80us after the bus pulled to start sending data
 
  for (int i = 0; i < 4; i ++) // receive temperature and humidity data, the parity bit is not considered
    dat[i] = read_data ();
  pinMode (DHpin, OUTPUT);
  digitalWrite (DHpin, HIGH); // send data once after releasing the bus, wait for the host to open the next Start signal
  
  String dataString = "humdity "+String(dat[0])+'.'+String(dat[1])+" %"+", "+"temperature "+String(dat[2])+'.'+String(dat[3])+" C";
  //Serial.println (dataString);
  writeToSDcard(dataString);
}

void processSmokeSensor() {
  smoke_level= analogRead(MQ2sensorPin); //arduino reads the value from the smoke sensor - values from 0 to 1023
  mapped_smoke_level = map(smoke_level, 0, 1023, 0, 100); // mapp the value of 1023 to 100
  sprintf(info, "MQ-2: %d (%d)", smoke_level, mapped_smoke_level);// prints to see what values the sensor is picking up
  if(mapped_smoke_level > 20){ //if mapped smoke level is greater than 20, the buzzer will go off
    digitalWrite(buzzerPin, HIGH);
    Serial.print(info);
    Serial.println(F(" WARNING! DETECTED GAS OR SMOKE "));
  }
  else{
    digitalWrite(buzzerPin, LOW);
    Serial.println(info);
  }
}

void processGPS() {
  // For one second we parse GPS data and report some key values
  //for (unsigned long start = millis(); millis() - start < GPS_PARSE_TIME;)
  //{
  setColor(0, 0, 0); 
  while (Serial2.available())
  {
    setColor(255, 0, 0); 
    char c = Serial2.read();
    //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
    if (gps.encode(c)) // Did a new valid sentence come in?
      //newData = true;
      //Serial.write(c); 
      getgps(gps); // then grab the data and save it to SD card
  }// while
  //}
}// void processGPS()

// The getgps function will get and print the values we want.
void getgps(TinyGPS &gps) {
  // Define the variables that will be used
  float latitude, longitude;
  // Then call this function
  gps.f_get_position(&latitude, &longitude);
  print_float(latitude, TinyGPS::GPS_INVALID_F_ANGLE, 9, 5, 1); //LATITUDE
  print_float(longitude, TinyGPS::GPS_INVALID_F_ANGLE, 10, 5, 2); //LONGITUDE
  // Same goes for date and time
  int year;
  uint8_t month, day, hour, minute, second, hundredths;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths);

  // Get Date and Time
  char gps_date[32];
  sprintf(gps_date, "%02d/%02d/%02d %02d:%02d:%02d", month, day, year, hour, minute, second);
  //Serial.print(gps_date);
  SD_date_time = gps_date;

  String dataString = SD_date_time + "," + SD_lat + "," + SD_lon;
  saveDataToSDcard(dataString);
}

static void print_int(unsigned long val, unsigned long invalid, int len) {
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  ////Serial.print(sz);
}// static void print_int(unsigned long val, unsigned long invalid, int len)


static void print_float(float val, float invalid, int len, int prec, int SD_val) {
  char sz[32];
  if (val == invalid)
  {
    strcpy(sz, "*******");
    sz[len] = 0;
    if (len > 0) {
      sz[len - 1] = ' ';
    }
    for (int i = 7; i < len; ++i) {
      sz[i] = ' ';
    }
    ////Serial.print(sz);
    if (SD_val == 1) {
      SD_lat = sz;
    }
    else if (SD_val == 2) SD_lon = sz;
  }
  else
  {
    //Serial.print(val, prec);
    if (SD_val == 1) {
      SD_lat = dtostrf(val, 10, 5, dtostrfbuffer);
    }
    else if (SD_val == 2) SD_lon = dtostrf(val, 10, 5, dtostrfbuffer);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
  }
}// static void print_float(float val, float invalid, int len, int prec, int SD_val)


void writeStringToSDcard(String dataString) {
  writeToSDcard(dataString);
}

void RGBLedTest(int delay_ms) {
  setColor(255, 0, 0);  // red
  delay(delay_ms);
  setColor(0, 255, 0);  // green
  delay(delay_ms);
  setColor(0, 0, 255);  // blue
  delay(delay_ms);
  setColor(255, 255, 0);  // yellow
  delay(delay_ms);  
  setColor(80, 0, 80);  // purple
  delay(delay_ms);
  setColor(0, 255, 255);  // aqua
  delay(delay_ms);
  setColor(255, 255, 255);  // white
}

void BuzzerTest(int delay_ms){
  tone(buzzerPin, delay_ms); // Send 1KHz sound signal...
  delay(delay_ms);        // ...for 1 sec
  noTone(buzzerPin);     // Stop sound...
}

void SDcardInit() {
  //Serial.print("Initializing SD card...");
  pinMode(chipSelect, OUTPUT);
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    //Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  saveDataToSDcard("Start Measurements");
}// void SDcardInit()

void writeToSDcard(String dataString) {
  File dataFile = SD.open("temp.csv", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    Serial.println("temp data");
    //Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}// void writeToSDcard()

void saveDataToSDcard(String dataString) {
  Serial.println(dataString);
  //Open the Data CSV File
  File dataFile = SD.open("gps.csv", FILE_WRITE);
  if (dataFile)
  {
    dataFile.println(dataString);
    //Serial.println(dataString);
    dataFile.close();
    Serial.println("gps data");
  }
  else
  {
    Serial.println("\nCouldn't open the log file!");
  }
}// void saveDataToSDcard(String dataString)

