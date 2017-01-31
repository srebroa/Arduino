/*
SD card attached to SPI bus as follows -> Arduino Mega 2560:
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
#define LEDS_DELAY_MS 2000
#define GPS_PARSE_TIME 2000

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
  diagnosticLedsInit();
  Serial.begin(9600);
  //Serial.println("Dual VNH5019 Motor Driver Shield Test");
  //Serial1.begin(19200);//9600 for HC-05, 115200 for BTM777, 19200 for BTM222
  
  //Serial2.begin(9600); //GPS
  SDcardInit();
  DHT11sensorIinit();
  diagnosticLedsTest(LEDS_DELAY_MS);
}

void loop() {
  processTemperatureHumidity();
  //processGPS();
  //delay(150);
  delay(1000);
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

void diagnosticLedsInit() {
  //Diagnostic LEDs
  pinMode(POWER_LED, OUTPUT);
  pinMode(FAULT_LED, OUTPUT);
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
  Serial.println (dataString);
  saveDataToSDcard(dataString);
}

void processGPS() {
  // For one second we parse GPS data and report some key values
  //for (unsigned long start = millis(); millis() - start < GPS_PARSE_TIME;)
  //{
  while (Serial2.available())
  {
    char c = Serial2.read();
    //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
    if (gps.encode(c)) // Did a new valid sentence come in?
      //newData = true;
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
    ////Serial.////Serial.print(val, prec);
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

void diagnosticLedsTest(int delay_ms) {
  digitalWrite(POWER_LED, HIGH);
  digitalWrite(FAULT_LED, HIGH);
  delay(delay_ms);
  digitalWrite(POWER_LED, LOW);
  digitalWrite(FAULT_LED, LOW);
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
  //Serial.println("card initialized.");
}// void SDcardInit()

void writeToSDcard(String dataString) {
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    //Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    //Serial.println("error opening datalog.txt");
  }
}// void writeToSDcard()

void saveDataToSDcard(String dataString) {
  //Open the Data CSV File
  File dataFile = SD.open("LOG.csv", FILE_WRITE);
  if (dataFile)
  {
    dataFile.println(dataString);
    ////Serial.println(dataString);
    dataFile.close();
  }
  else
  {
    ////Serial.println("\nCouldn't open the log file!");
  }
}// void saveDataToSDcard(String dataString)

