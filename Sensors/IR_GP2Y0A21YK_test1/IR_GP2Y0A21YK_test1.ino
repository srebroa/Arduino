
/*IR_GP2Y0A21YK_test1 - Print the distance from the IR proximity sensor Sharp GP2Y0A21YK

Connections:
 GP2Y0A21YK -> Arduino Mega 2560
 1. Vo (YELLOW) - A7
 2. GND (BLACK) - GND 
 3. Vcc (RED) - 5V 
*/

//#define IR_MIN_DETECTION_THRESHOLD 482 // 10cm
//#define IR_MAX_DETECTION_THRESHOLD 150

int IRsensorFront = A7; 
int distanceIRsensorFront;

void setup() {
  pinMode(IRsensorFront, INPUT);  // declare Front IR Sensor as input
  Serial.begin(9600);
}

void loop() {
  Serial.println("distance: "+String(readIRsensor()));
  delay(200);      
}

int readIRsensor(){
  distanceIRsensorFront = analogRead(IRsensorFront);
  return distanceIRsensorFront;
}// void readIRsensors()
