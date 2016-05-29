/*
SD card attached to SPI bus as follows (Arduino Mega 2560):
** MISO - pin 50
** MOSI - pin 51
** CLK - pin 52
** CS - pin 53
*/
#include "DualVNH5019MotorShield.h"
#include <stdlib.h>
// GPS libraries
#include <TinyGPS.h>
// SD card libraries
#include <SPI.h>
#include <SD.h>
// Timer
//#include <SimpleTimer.h>

#define MAX_SPEED 210 // 6V for 7.4 LiPol
#define HALF_SPEED 105
#define LOW_SPEED 60
#define MOVEMENT_TIME_MS 2000

#define POWER_LED 48
#define FAULT_LED 49
#define LEDS_DELAY_MS 2000
#define MAX_CURRENT_2MOTORS 9000
#define MEDIUM_CURRENT_2MOTORS 1000

#define GPS_PARSE_TIME 2000

//SimpleTimer timer; // The timer object
unsigned long timer; // the timer
boolean timedOut = false; // set to true when timer fired
unsigned long INTERVAL = 3000; // the timeout interval
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

long pwmLvalue = MAX_SPEED; // around 6,5V for 6V DC motors! (MAX_SPEED = 7.4V)
long pwmRvalue = MAX_SPEED;
byte pwmChannel;
const char startOfNumberDelimiter = '<';
const char endOfNumberDelimiter = '>';
String dataString = "";

DualVNH5019MotorShield md;
//int speedArray[3] = {MAX_SPEED, HALF_SPEED, LOW_SPEED};
int current_speed;

unsigned int M1current;
unsigned int M2current;
int motorsCurrentMeasurementsCounter;

void stopIfFault() {
  if (md.getM1Fault()) {
    md.setM1Speed(0);
    //Serial.println("M1 fault");
    dataString = "Motor1 FAULT";
    writeStringToSDcard(dataString);
  }
  if (md.getM2Fault()) {
    md.setM2Speed(0);
    //Serial.println("M2 fault");
    dataString = "Motor2 FAULT";
    writeStringToSDcard(dataString);
  }
}

void setup() {
  diagnosticLedsInit();
  //Serial.begin(9600);
  //Serial.println("Dual VNH5019 Motor Driver Shield Test");
  Serial1.begin(19200);//9600 for HC-05, 115200 for BTM777, 19200 for BTM222
  Serial2.begin(9600); //GPS
  md.init();
  SDcardInit();
  //timer.setInterval(1000, repeatMe);
  diagnosticLedsTest(LEDS_DELAY_MS);
}

void loop() {
  // Set speed for motor 1, speed is a number betwenn -255 and 255
  checkStopTimer();
  if (Serial1.available()) {
    processInput();
    //readIRsensors();
  }
  processGPS();
  motorsCurrentMeasurements();
  stopIfFault();
  delay(150);
}// void loop()

void checkStopTimer(){
  if ((!timedOut) && ((millis() - timer) > INTERVAL)) {
    // timed out
    timedOut = true; // don't do this again
    stop_Robot();
  }
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


void motorsCurrentMeasurements() {
  //String dataString = "";
  M1current = md.getM1CurrentMilliamps();
  M2current = md.getM2CurrentMilliamps();
  //dataString = "m1: "+String(M1current)+", "+ "m2: "+String(M2current)
  if (M1current > MAX_CURRENT_2MOTORS || M2current > MAX_CURRENT_2MOTORS) {
    //Serial.println("M1current: "+String(M1current));
    md.setBrakes(60, 60);
    dataString = String(M1current) + ", " + String(M2current) + " OVERLOAD";
    writeStringToSDcard(dataString);
  }
  else if (M1current > MEDIUM_CURRENT_2MOTORS || M2current > MEDIUM_CURRENT_2MOTORS) {
    dataString = String(M1current) + ", " + String(M2current);
    writeStringToSDcard(dataString);
  }
  /*
  if(motorsCurrentMeasurementsCounter<5){
      motorsCurrentMeasurementsCounter++;
  }
  else{
    dataString = String(M1current)+", "+String(M2current);
    writeStringToSDcard(dataString);
    //Serial.println("m1: "+String(M1current)+", "+ "m2: "+String(M2current));
  }
  */
}

void writeStringToSDcard(String dataString) {
  writeToSDcard(dataString);
  motorsCurrentMeasurementsCounter = 0;
}

void diagnosticLedsInit() {
  //Diagnostic LEDs
  pinMode(POWER_LED, OUTPUT);
  pinMode(FAULT_LED, OUTPUT);
}

void diagnosticLedsTest(int delay_ms) {
  digitalWrite(POWER_LED, HIGH);
  digitalWrite(FAULT_LED, HIGH);
  delay(delay_ms);
  digitalWrite(POWER_LED, LOW);
  digitalWrite(FAULT_LED, LOW);
}

void SDcardInit() {
  motorsCurrentMeasurementsCounter = 0;
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

void runStopTimer(){
  timedOut = false; // allow timer to fire
  timer = millis(); // start timer
}

void processInput () {
  static long receivedNumber = 0;
  static boolean negative = false;
  byte c = Serial1.read();
  //Serial.println(c);

  switch (c) {
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
      goForward(MAX_SPEED);
      runStopTimer();
      //Serial.println("forward");
      break;

    case 'b': // Go BACK
      //moveLeftBackward(MAX_SPEED);
      goBackwad(MAX_SPEED);
      runStopTimer();
      //Serial.println("backward");
      break;

    case 'r':
      turnRight(HALF_SPEED);
      runStopTimer();
      //Serial.println("RIGHT");
      break;

    case 'l':
      turnLeft(HALF_SPEED);
      runStopTimer();
      //Serial.println("LEFT");
      break;

    case 'c': // Top Right
      moveRightForward(MAX_SPEED);
      runStopTimer();
      break;

    case 'd': // Top Left
      moveLeftForward(MAX_SPEED);
      runStopTimer();
      break;

    case 'e': // Bottom Right
      moveRightBackward(MAX_SPEED);
      runStopTimer();
      break;

    case 'h': // Bottom Left
      moveLeftBackward(MAX_SPEED);
      runStopTimer();
      //Serial.println("LeftBackward");
      break;

    case 's':
      stop_Robot();
      break;

    case 'x':
      pwmChannel = 1; // pwm_R
      break;
    case 'y': // pwm_L
      pwmChannel = 2;
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

void waitAndStop(int delay_ms) {
  delay(delay_ms);
  md.setSpeeds(0, 0);
  delay(200);
}

void goForward(int mspeed) {
  md.setSpeeds(mspeed, -mspeed);
}// void goForward(int mspeed, int time_ms)

void goBackwad(int mspeed) {
  md.setSpeeds(-mspeed, mspeed);
}// void goBackwad(int mspeed, int time_ms)

void turnRight(int mspeed) {
  md.setSpeeds(mspeed, mspeed);
}// void goBackwad(int mspeed, int time_ms)

void turnLeft(int mspeed) {
  md.setSpeeds(-mspeed, -mspeed);
}// void turnLeft(int mspeed)

void moveRightForward(int mspeed) {
  md.setSpeeds(mspeed, (int)(-mspeed * 0.4));
}// void moveRightForward(int mspeed)

void moveLeftForward(int mspeed) {
  md.setSpeeds((int)(mspeed * 0.4), -mspeed);
}// moveLeftForward(int mspeed)

void moveRightBackward(int mspeed) {
  md.setSpeeds(-mspeed, (int)(mspeed * 0.4));
}// void moveRightBackward(int mspeed)

void moveLeftBackward(int mspeed) {
  md.setSpeeds((int)(-mspeed * 0.4), mspeed);
}// void moveRightBackward(int mspeed)

void stopRobot(int delay_ms) {
  md.setSpeeds(0, 0);
  delay(delay_ms);
}// void stopRobot(int delay_ms)

void stop_Robot() {
  md.setSpeeds(0, 0);
  delay(10);
  md.setBrakes(100, 100);
  delay(150);
}// void stopRobot()

void SetPWM (const long pwm_num, byte pwm_channel) {
  if (pwm_channel == 1) { // DRIVE MOTOR
    md.setM1Speed(pwm_num);
  }
  else if (pwm_channel == 2) { // STEERING MOTOR
    md.setM2Speed(pwm_num);
  }
}// void SetPWM (const long pwm_num, byte pwm_channel)


