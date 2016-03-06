/*
elevator.ino - control elevator via Android App
 */
 
/*H bridge pins*/
const int Hbridge_In1 = 2;    // H bridge control Input
const int Hbridge_In2 = 3;    // H bridge control Input
const int pwm_out = 5;        // pwm output
/*Digital Sensors Inputs*/
const int sensor1 = 8;
const int sensor2 = 9;
const int sensor3 = 10;
/*Sensors States*/
int sensor1State = 1;
int sensor2State = 1;
int sensor3State = 1;
int rotation_direction;
boolean sensorStateChanged = false;

void setup()
{
  pinMode(Hbridge_In1, OUTPUT);   // sets the pin as output
  pinMode(Hbridge_In2, OUTPUT);   
  pinMode(pwm_out, OUTPUT);   
  pinMode(sensor1, INPUT);   // sets the pin as input
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  rotation_direction = 1; // right direction
  Serial.begin(9600); 
  analogWrite(pwm_out, 255);  // analogRead values go from 0 to 1023, analogWrite values from 0 to 255
}

void loop(){
  checkSensorsValues();
  if((sensor1State == 0) && (sensorStateChanged == true)){
    stopAtTheFloor();
    digitalWrite(Hbridge_In1, HIGH);
    digitalWrite(Hbridge_In2, LOW);
    analogWrite(pwm_out, 255);
  }
  else if((sensor2State == 0) && (sensorStateChanged == true)){
  }  
  else if((sensor3State == 0) && (sensorStateChanged == true)){
    stopAtTheFloor();
    digitalWrite(Hbridge_In1, LOW);
    digitalWrite(Hbridge_In2, HIGH);
    analogWrite(pwm_out, 120);
  }    
}// void loop()

void  stopAtTheFloor(){
  digitalWrite(Hbridge_In1, LOW);
  digitalWrite(Hbridge_In2, LOW);
  delay(1000);
}

void checkSensorsValues(){
  if(sensor1State != digitalRead(sensor1)){
    sensor1State = digitalRead(sensor1);
    sensorStateChanged = true;
    Serial.print("sensor1 = ");
    Serial.println(sensor1State);
  }
  else if(sensor2State != digitalRead(sensor2)){
    sensor2State = digitalRead(sensor2);
    sensorStateChanged = true;
    Serial.print("sensor2 = ");
    Serial.println(sensor2State);
  }
  else if(sensor3State != digitalRead(sensor3)){
    sensor3State = digitalRead(sensor3);
    sensorStateChanged = true;
    Serial.print("sensor3 = ");
    Serial.println(sensor3State);
  }
  else{
    sensorStateChanged = false;
  }
}// void checkSensorsValues()
