/*  VNH2SP30_single_motor_driver_test - Testing single motor driver VNH2SP30*/
#define CW   1
#define CCW  2
#define CS_THRESHOLD 500
#define PWM_MAX 225
#define PWM_SLOW 50
#define RUNNING_TIME_MS 5000

/*  VNH2SP30 pin definitions */
int INA = 7;  // INA: Clockwise input
int INB = 8; // INB: Counter-clockwise input
int PWM_out = 5; // PWM output
int CS = 2; // CS: Current sense analog input
//int EN[2] = {0, 1}; // EN: Status of switches output (Analog pin)

void setup(){
  Serial.begin(9600);
  // Initialize digital pins as outputs
    pinMode(INA, OUTPUT);
    pinMode(INB, OUTPUT);
    pinMode(PWM_out, OUTPUT);
  // Turn Off all motors
    motorOff();
}// void setup()

void loop(){
  motorGo(CW, PWM_SLOW);
  waitAndTurnMotorsOff(RUNNING_TIME_MS);
  motorGo(CCW, PWM_SLOW);
  waitAndTurnMotorsOff(RUNNING_TIME_MS);
  
  motorGo(CW, PWM_MAX);
  waitAndTurnMotorsOff(RUNNING_TIME_MS);
  motorGo(CCW, 100);
  waitAndTurnMotorsOff(RUNNING_TIME_MS);
  //waitAndTurnMotorsOff(RUNNING_TIME_MS);
}// void loop()

void printCurrentSense(){
  int cs0 = analogRead(CS);
  Serial.println("cs0:"+String(cs0));
  if (cs0 > CS_THRESHOLD){
    Serial.println("OVERLOAD!!!");
  }
}// void printCurrentSense()

void waitAndTurnMotorsOff(int time_ms){
  int unit_delay = 200;
  delay(200);
  for (int i=0; i<int(time_ms/unit_delay); i++){
    printCurrentSense();
    delay(unit_delay);
  }
  motorOff();
  delay(2000);
}// void waitAndTurnMotorsOff(int time_ms)

void motorOff(){
  digitalWrite(INA, LOW); // Brake to GND
  digitalWrite(INB, LOW); // Brake to GND
  analogWrite(PWM_out, 0);
}// void motorOff()

/* motorGo() will set a motor going in a specific direction
 - motors is labeled as 0
 - direct: Should be between 1 and 2, with the following result
 1: Clockwise
 2: CounterClockwise
 - PWM_out: should be a value between 0 and 255
 */
void motorGo(uint8_t direct, uint8_t m_pwm){
  if (m_pwm <=255){
      // Set inA[motor]
      if (direct == 1){
        digitalWrite(INA, HIGH);
        digitalWrite(INB, LOW);
      }
      else if (direct == 2){
        digitalWrite(INA, LOW);
        digitalWrite(INB, HIGH);
      }
      analogWrite(PWM_out, m_pwm);
  }// if (m_PWM_out <=255)
}// void motorGo(uint8_t direct, uint8_t m_pwm)
