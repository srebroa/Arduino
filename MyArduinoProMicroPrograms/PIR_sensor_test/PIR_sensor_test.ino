

/* TEST PIR SENSOR - Arduino Pro Micro 

 PINS CONNECTION:
 --------------------------------
 PIR SENSOR     Arduino Pro Micro
 --------------------------------
 Vcc        ->  Vcc
 GND        ->  GND
 OUT        ->  2
 --------------------------------
 LED        ->  10
 
 Instead the PIR sensor you can connect a button!
 */
 
int ledPin = 10;                // choose the Arduino pin 10 for the LED
int inputPin = 2;               // choose the input pin (for PIR sensor)
int pirState = LOW;             // we start, assuming no motion detected
int val = 0;                    // variable for reading the pin status
 
void setup() {
  pinMode(ledPin, OUTPUT);      // declare LED as output
  pinMode(inputPin, INPUT);     // declare sensor as input
 
  Serial.begin(9600);
}
 
void loop(){
  val = digitalRead(inputPin);  // read input value
  if (val == HIGH) {            // check if the input is HIGH
    digitalWrite(ledPin, LOW);  // turn LED ON
    if (pirState == LOW) {
      // we have just turned on
      Serial.println("Motion detected");
      // We only want to print on the output change, not state
      pirState = HIGH;
    }
  } else {
    digitalWrite(ledPin, HIGH); // turn LED OFF
    if (pirState == HIGH){
      // we have just turned of
      Serial.println("Motion ended");
      // We only want to print on the output change, not state
      pirState = LOW;
    }
  }
}// void loop()


