/*Test Code for MQ-2 Smoke Sensor and Arduino Nano*/
/*
Connection MQ-2 -> Arduino Nano
OUT -> A0
VCC -> +5V
GND -> GND
*/

const int MQ2sensorPin= A0;
const int buzzerPin= PD2;
int smoke_level;

void setup() {
Serial.begin(9600); 
pinMode(MQ2sensorPin, INPUT);//the smoke sensor will be an input for arduino
pinMode(buzzerPin, OUTPUT);//the buzzer serves an output in the circuit
}

void loop() {
  smoke_level= analogRead(MQ2sensorPin); //arduino reads the value from the smoke sensor
  Serial.print("smoke level = ");
  Serial.println(smoke_level);//prints just for debugging purposes, to see what values the sensor is picking up
  if(smoke_level > 230){ //if smoke level is greater than 230, the buzzer will go off
    digitalWrite(buzzerPin, HIGH);
  }
  else{
    digitalWrite(buzzerPin, LOW);
  }
  delay (1000);
}
