/* Servo Encoder Test_Code
Purpose: Control two servos and test the encoder 
Author: Roboball 09/2018
*/
#include <Servo.h>

// init servo globals
Servo arm_down;  // create servo object
Servo arm_up; // create servo object
int servo_01 = 10;
int servo_02 = 11;
int pos_down = 0;   
int pos_up = 90; 
// init encoder globals
const int trackingPin = 3; //the tracking module attach to pin 2
const int ledPin = 13; //pin13 built-in led
int sum_high = 0; // sum up all high values
int sum_low= 0;  // sum up all low values

void setup() {
  // setup servos
  arm_up.attach(servo_01);  // attach servos to pins
  arm_down.attach(servo_02);
  arm_up.write(pos_up);              // tell servo to go to position in variable 'pos'
  // setup encoder
  pinMode(trackingPin, INPUT); // set trackingPin as INPUT
  pinMode(ledPin, OUTPUT); //set ledPin as OUTPUT
  Serial.begin(9600);
  Serial.println("Start Recording");
  Serial.println(""); 
}

void loop() {
  
   boolean val = digitalRead(trackingPin); // read the value of tracking module
  if(val == HIGH){ //if it is HiGH{ 
    digitalWrite(ledPin, HIGH); //turn off the led
    sum_high = sum_high +1;
    //Serial.println("Low");
  }
  else{
    digitalWrite(ledPin, LOW); //turn on the led
    sum_low = sum_low +1;
    //Serial.println("High");
  }
 
  for (pos_down = 50; pos_down <= 160; pos_down += 1) { // goes from 0 degrees to 180 degrees
       arm_down.write(pos_down);              // tell servo to go to position in variable 'pos'
       delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos_up = 90; pos_up >= 50; pos_up -= 1) { // goes from 180 degrees to 0 degrees
       arm_up.write(pos_up);              // tell servo to go to position in variable 'pos'
       delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos_down = 160; pos_down >= 50; pos_down -= 1) { // goes from 180 degrees to 0 degrees
       arm_down.write(pos_down);              // tell servo to go to position in variable 'pos'
       delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos_up = 50; pos_up <= 90; pos_up += 1) { // goes from 180 degrees to 0 degrees
       arm_up.write(pos_up);              // tell servo to go to position in variable 'pos'
       delay(15);                       // waits 15ms for the servo to reach the position
  }
  Serial.print("Sum High: ");
  Serial.println(sum_high);
  Serial.print("Sum Low: ");
  Serial.println(sum_low);
}
