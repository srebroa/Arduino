/* 
DRAGINO OMNI WHEEL ROBOT v1.0
   - Allows you to control a omni wheel mobile robot via wifi
   - Tested with Arduino Mega 2560 and Dragino Yun Shield
   - Android application:
   Author: Adam Srebro
   www: http://www.mobilerobots.pl/
 
 Possible commands e.g.:
 http://192.168.0.11/arduino/control/tr  //192.168.0.11 should be replaced by Dragino IP address in your local network!  
 http://192.168.0.11/arduino/control/tl
 http://192.168.0.11/arduino/control/ip
 ...
*/

#include <Bridge.h>
#include <YunServer.h>
#include <YunClient.h>
#include <SoftwareSerial.h>
#include <PololuMaestro.h>

#define default_delay 500 // 500ms delay
#define neutral_position 6000 //150us *4
#define clockwise_direction 8000
#define counter_clockwise_direction 4000

// Listen on default port 5555, the webserver on the Yun
// will forward there all the HTTP requests for us.
YunServer server;
MiniMaestro maestro(Serial1);

void setup() {
  Serial1.begin(9600);  
  Bridge.begin(); // Bridge startup
  server.begin();
}

void loop() {
  // Get clients coming from server
  YunClient client = server.accept();
  // There is a new client?
  if (client) {
    // Process request
    process(client);

    // Close connection and free resources.
    client.stop();
  }
  delay(50); // Poll every 50ms
}

void process(YunClient client) {
  // read the command
  String command = client.readStringUntil('/');
  // is "control" command?
  if (command == "control") {
    controlCommand(client);
  }
}

void controlCommand(YunClient client) {
  String mode = client.readStringUntil('\r');  
  if (mode == "tr") {
    // Send feedback to client
    client.print(F("TURN RIGHT :)"));
    turn_right();
    return;
  }   
  if (mode == "tl") {
    client.print(F("TURN LEFT :)"));
    turn_left();
    return;
  } 
  if (mode == "stop") {
    client.print(F("Robot STOP!"));
    stop_robot();
    return;
  }
  if (mode == "mf") {
    client.print(F("move_forward!"));
    move_forward();
    return;
  }
  if (mode == "mb") {
    client.print(F("move_backward!"));
    move_backward();
    return;
  }
  if (mode == "mr") {
    client.print(F("move_right!"));
    move_right();
    return;
  }     
  if (mode == "ml") {
    client.print(F("move_left!"));
    move_left();
    return;
  }  
    if (mode == "fr") {
    client.print(F("forward right!"));
    forward_right();
    return;
  }  
  if (mode == "fl") {
    client.print(F("forward left!"));
    forward_left();
    return;
  }   
  if (mode == "br") {
    client.print(F("backward right!"));
    backward_right();
    return;
  }  
  if (mode == "bl") {
    client.print(F("backward left!"));
    backward_left();
    return;
  } 
  if (mode == "ip") {
    client.print(F("successfully connected"));
    return;
  } 
  client.print(F("error: invalid mode "));
  client.print(mode);
}//void controlCommand(YunClient client)

void stop_robot(void){
   maestro.setTarget(1, neutral_position);
   maestro.setTarget(2, neutral_position);
   maestro.setTarget(3, neutral_position);
   maestro.setTarget(4, neutral_position);
   delay(default_delay);
}

 void turn_left(void){
   maestro.setTarget(1, counter_clockwise_direction);
   maestro.setTarget(2, counter_clockwise_direction);
   maestro.setTarget(3, counter_clockwise_direction);
   maestro.setTarget(4, counter_clockwise_direction);
   delay(default_delay);
 }
 
 void turn_right(void){
   maestro.setTarget(1, clockwise_direction);
   maestro.setTarget(2, clockwise_direction);
   maestro.setTarget(3, clockwise_direction);
   maestro.setTarget(4, clockwise_direction);
   delay(default_delay);
 }
 
 void forward_right(void){
   maestro.setTarget(1, clockwise_direction);
   maestro.setTarget(2, neutral_position);
   maestro.setTarget(3, counter_clockwise_direction);
   maestro.setTarget(4, neutral_position);
   delay(default_delay);
 }
  
 void forward_left(void){
   maestro.setTarget(1, neutral_position);
   maestro.setTarget(2, counter_clockwise_direction);
   maestro.setTarget(3, neutral_position);
   maestro.setTarget(4, clockwise_direction);
   delay(default_delay);
 }
 
 void backward_right(void){
   maestro.setTarget(1, neutral_position);
   maestro.setTarget(2, clockwise_direction);
   maestro.setTarget(3, neutral_position);
   maestro.setTarget(4, counter_clockwise_direction);
   delay(default_delay);
 }
 
 void backward_left(void){
   maestro.setTarget(1, counter_clockwise_direction);
   maestro.setTarget(2, neutral_position);
   maestro.setTarget(3, clockwise_direction);
   maestro.setTarget(4, neutral_position);
   delay(default_delay);
 }
 
 void move_right(void){
   maestro.setTarget(1, clockwise_direction);
   maestro.setTarget(2, clockwise_direction);
   maestro.setTarget(3, counter_clockwise_direction);
   maestro.setTarget(4, counter_clockwise_direction);
   delay(default_delay);
 }
 
 void move_left(void){
   maestro.setTarget(1, counter_clockwise_direction);
   maestro.setTarget(2, counter_clockwise_direction);
   maestro.setTarget(3, clockwise_direction);
   maestro.setTarget(4, clockwise_direction);
   delay(default_delay);
 }

 void move_forward(void){
   maestro.setTarget(1, clockwise_direction);
   maestro.setTarget(2, counter_clockwise_direction);
   maestro.setTarget(3, counter_clockwise_direction);
   maestro.setTarget(4, clockwise_direction);
   delay(default_delay);
 }
 
 void move_backward(void){
   maestro.setTarget(1, counter_clockwise_direction);
   maestro.setTarget(2, clockwise_direction);
   maestro.setTarget(3, clockwise_direction);
   maestro.setTarget(4, counter_clockwise_direction);
   delay(default_delay);
 }
