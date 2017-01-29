/* Testing MQ-3 Alcohol Sensor Circuit with Arduino Nano */
/*
Connection MQ-3 -> Arduino Nano
A0 -> A0
D0 -> D2
GND -> GND
VCC -> +5V
*/
const int AOsensorPin = A0; //the AOUT pin of the alcohol sensor goes into analog pin A0 of the arduino
const int DOsensorPin = PD2; //the DOUT pin of the alcohol sensor goes into digital pin D2 of the arduino
const int ledPin = PD3; //the anode of the LED connects to digital pin PD3 of the arduino

int limit;
int alcoholValue;

void setup() {
  Serial.begin(9600);
  pinMode(DOsensorPin, INPUT);//sets the pin as an input to the arduino
  pinMode(ledPin, OUTPUT);//sets the pin as an output of the arduino
}

void loop(){
  alcoholValue = analogRead(AOsensorPin); //reads the analaog value from the alcohol sensor's AOUT pin
  limit = digitalRead(DOsensorPin); //reads the digital value from the alcohol sensor's DOUT pin
  Serial.print(" Alcohol value: ");
  Serial.println(alcoholValue); //prints the alcohol value
  Serial.print("Limit: ");
  Serial.print(limit); //prints the limit reached as either LOW or HIGH (above or underneath)
  delay(500);
  if (limit == HIGH){
    digitalWrite(ledPin, HIGH); //if limit has been reached, LED turns on as status indicator
  }
  else{
    digitalWrite(ledPin, LOW); //if threshold not reached, LED remains off
  }
}
