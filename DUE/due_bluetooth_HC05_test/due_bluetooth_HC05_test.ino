/*
Arduino Due + HC-05 bluetooth 
-echo bluetooth data

Serial (Tx/Rx) communicate to PC via USB
Serial1 (Tx1/Rx1) connect to HC-05
HC-05 Rx - Due Tx1 (18)
HC-05 Tx - Due Rx1 (19)
HC-05 GND - Due GND
HC-05 VCC - Due 3.3V
*/
#define HC05 Serial1

void setup()
{
  delay(500);
  Serial.begin(9600);
  HC05.begin(9600);
  Serial.write("\nHC-05 bluetooth test\n");
}

void loop(){
  while(HC05.available()){
    char data = HC05.read();
    Serial.write(data);
  }
}// void loop()

