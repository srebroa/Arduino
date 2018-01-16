/*
Testing FrSky V8FR-II 2.4GHz Receiver Connected to Arduino.
Taranis Q X7 2.4GHz 7CH Transmitter is set up in Mode2 - Right control stick: Aileron, Elevator 
- Tested with Arduino Mega 2560
www: http://www.mobilerobots.pl
How to bind FrSky V8FR-II to Taranis Q X7: https://www.youtube.com/watch?v=FN0vjDh2Pdg 
 
 Connections:
 FrSky V8FR-II 2.4GHz Receiver -> Arduino Mega 2560
 ch2 - 7 // Aileron
 ch3 - 8 // Elevator
 */
 
#define LOWER_STOP_RANGE_MOVE -20
#define UPPER_STOP_RANGE_MOVE 20
#define LOWER_STOP_RANGE_TURN -20
#define UPPER_STOP_RANGE_TURN 20
 
/*FrSky V8FR-II 2.4GHz Receiver*/
//const int Channel1 = 6; 
const int Channel2 = 7;
const int Channel3 = 8;
//const int Channel4 = 9;
boolean stop_state = true;

// MODE2 
int ch1; // Throttle
int ch2; // Aileron
int ch3; // Elevator
int ch4; // Rudder

int moveValue;
int turnValue;

void setup(){
  //pinMode(6, INPUT); //6
  pinMode(Channel2, INPUT); //7
  pinMode(Channel3, INPUT); //8
  //pinMode(Channel4, INPUT); //9
  Serial.begin(9600);
}// void setup()

void loop() {
  // put your main code here, to run repeatedly:
  //ch1 = pulseIn(6, HIGH, 25000); // Read the pulse width of each channel
  ch2 = pulseIn(Channel2, HIGH, 25000); 
  ch3 = pulseIn(Channel3, HIGH, 25000);
  //ch4 = pulseIn(9, HIGH, 25000);
  
  moveValue = map(ch3, 980, 1999, -255, 255); //center over zero
  moveValue = constrain(moveValue, -255, 255);
                                   
  turnValue = map(ch2, 980, 1999, -255, 255);
  turnValue = constrain(turnValue, -255, 255);
  
  Serial.println("moveValue: "+String(moveValue)+ ", turnValue: "+String(turnValue));
  if (moveValue>LOWER_STOP_RANGE_MOVE && moveValue<UPPER_STOP_RANGE_MOVE && turnValue>LOWER_STOP_RANGE_TURN && turnValue<UPPER_STOP_RANGE_TURN){
    if(stop_state == false){
      stop_state = true;
      Serial.println("Stop");
    }
  }
  //GO FORWARD & BACKWARD
  else if(turnValue>LOWER_STOP_RANGE_TURN && turnValue<UPPER_STOP_RANGE_TURN){
    if(moveValue>UPPER_STOP_RANGE_MOVE){
      stop_state = false;
      Serial.println("Go Forward "+String(moveValue));
    }
    else if(moveValue<LOWER_STOP_RANGE_MOVE){
      stop_state = false;
      Serial.println("Go Backward "+String(moveValue));
    }
  } 
  //TURN RIGHT & LEFT
  else if(moveValue>LOWER_STOP_RANGE_MOVE && moveValue<UPPER_STOP_RANGE_MOVE){
    if(turnValue>UPPER_STOP_RANGE_TURN){
      stop_state = false;
      Serial.println("Turn Right "+String(turnValue));
    }
    else if(turnValue<LOWER_STOP_RANGE_TURN){
      stop_state = false;
      Serial.println("Turn Left "+String(turnValue));
    }
  } 
  delay(200);
}

