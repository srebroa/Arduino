/*
SRF05_test1 - Print the distance from the ultrasonic sensor to the nearest obstacle

Connections:
 SRF05 -> Arduino Mega 2560
 Vcc - 5V 
 GND - GND
 TRIG - 11
 ECHO - 12 
*/
 
#define TRIG 11 // the SRF05 Trig pin
#define ECHO 12 // the SRF05 Echo pin
unsigned long pulseTime;

void setup() {
  pinMode(TRIG,OUTPUT);
  pinMode(ECHO,INPUT);
  Serial.begin(9600);
}

float measurement(){
  digitalWrite(TRIG,HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG,LOW);
  // wait for the pulse to return. The pulse
  // goes from low to HIGH to low, so we specify
  // that we want a HIGH-going pulse below:
  pulseTime = pulseIn(ECHO,HIGH);
  return pulseTime / 58.00;
}

void loop() {
  Serial.print("Distance: ");
  Serial.print(measurement());
  Serial.println("cm");
  delay(500);
}
