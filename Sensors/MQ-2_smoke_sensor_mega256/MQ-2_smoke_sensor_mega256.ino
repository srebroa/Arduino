/*Test Code for MQ-2 Smoke Sensor and Arduino Mega 2560*/
/*
Connection MQ-2 -> Arduino Mega 2560
OUT -> A0
VCC -> +5V
GND -> GND
*/

const int MQ2sensorPin= A0;
const int buzzerPin= PD2;
int smoke_level;
byte mapped_smoke_level;
char info[96];

void setup() {
Serial.begin(9600); 
pinMode(MQ2sensorPin, INPUT);//the smoke sensor will be an input for arduino
pinMode(buzzerPin, OUTPUT);//the buzzer serves an output in the circuit
}

void loop() {
  smoke_level= analogRead(MQ2sensorPin); //arduino reads the value from the smoke sensor - values from 0 to 1023
  mapped_smoke_level = map(smoke_level, 0, 1023, 0, 100); // mapp the value of 1023 to 100
  sprintf(info, "MQ-2: %d (%d)", smoke_level, mapped_smoke_level);// prints to see what values the sensor is picking up
  if( mapped_smoke_level > 20){ //if mapped smoke level is greater than 20, the buzzer will go off
    digitalWrite(buzzerPin, HIGH);
    Serial.print(info);
    Serial.println(F(" WARNING! DETECTED GAS OR SMOKE "));
  }
  else{
    digitalWrite(buzzerPin, LOW);
    Serial.println(info);
  }
  delay (1000);
}
