#include <TinyGPS.h>
#include <SD.h>
#include <stdlib.h>
#include <SoftwareSerial.h>

/* This sample code demonstrates the normal use of a TinyGPS object.
   It requires the use of SoftwareSerial, and aSerial2umes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
TinyGPS gps;
static char dtostrfbuffer[20];
int CS = 8;
SoftwareSerial Serial2(4, 3);

//Define String
String SD_date_time = "invalid";
String SD_lat = "invalid";
String SD_lon = "invalid";
//String dataString ="";
void setup()
{
  pinMode(CS, OUTPUT);  //Chip Select Pin for the SD Card
  //Serial.begin(115200);
  Serial2.begin(9600); // GPS
  
  //Connect to the SD Card
  if(!SD.begin(CS))
  {
    //Serial.println("Card Failure");
    return;
  }
}

void loop(){
  //bool newData = false;
  //unsigned long chars;
  //unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 2000;){
    while (Serial2.available()){
      char c = Serial2.read();
      //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        //newData = true;
        getgps(gps); // then grab the data and save it to SD card
    }
  }
}// void loop()

  

// The getgps function will get and print the values we want.
void getgps(TinyGPS &gps){
  // Define the variables that will be used
  float latitude, longitude;
  // Then call this function
  gps.f_get_position(&latitude, &longitude);
  print_float(latitude, TinyGPS::GPS_INVALID_F_ANGLE, 9, 5, 1); //LATITUDE
  print_float(longitude, TinyGPS::GPS_INVALID_F_ANGLE, 10, 5, 2); //LONGITUDE
  // Same goes for date and time
  int year;
  byte month, day, hour, minute, second, hundredths;
  gps.crack_datetime(&year,&month,&day,&hour,&minute,&second,&hundredths);
  
  // Get Date and Time
  char gps_date[32];
  sprintf(gps_date, "%02d/%02d/%02d %02d:%02d:%02d",month, day, year, hour, minute, second);
  //Serial.print(gps_date);
  SD_date_time = gps_date;
  
  String dataString = SD_date_time+"," +SD_lat+"," +SD_lon;
  saveDataToSDcard(dataString);
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  //Serial.print(sz);
  //feedgps();
}

static void print_float(float val, float invalid, int len, int prec, int SD_val)
{
  char sz[32];
  if (val == invalid)
  {
    strcpy(sz, "*******");
    sz[len] = 0;
        if (len > 0) 
          sz[len-1] = ' ';
    for (int i=7; i<len; ++i)
        sz[i] = ' ';
    //Serial.print(sz);
    if(SD_val == 1) SD_lat = sz;
    else if(SD_val == 2) SD_lon = sz;
  }
  else
  {
    //Serial.//Serial.print(val, prec);
    if (SD_val == 1) SD_lat = dtostrf(val,10,5,dtostrfbuffer);
    else if (SD_val == 2) SD_lon = dtostrf(val,10,5,dtostrfbuffer);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i){
      //Serial.print(" ");
    }
  }

}

void saveDataToSDcard(String dataString){
  //Open the Data CSV File
  File dataFile = SD.open("LOG.csv", FILE_WRITE);
  if (dataFile)
  {
    dataFile.println(dataString);
    //Serial.println(dataString);
    dataFile.close();
  }
  else
  {
    //Serial.println("\nCouldn't open the log file!");
  }
}// void saveDataToSDcard(String dataString)
