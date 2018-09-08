/* Servo Encoder Test_Code
Purpose: Control two servos and test the encoder 
Author: Roboball 09/2018
*/
#include <Servo.h>

// init servo globals
Servo arm_down;  // create servo object
Servo arm_up; // create servo object
const byte servo_pin_01 = 10;
const byte servo_pin_02 = 11;
int pos_down = 0;   
int pos_up = 90; 
// init encoder globals
const byte interrupt_pin = 3;
volatile unsigned long reward = 0;
const byte led_pin = 13;
volatile byte state = LOW;

void setup() {
  // setup servos
  arm_up.attach(servo_pin_01);  // attach servos to pins
  arm_down.attach(servo_pin_02);
  arm_up.write(pos_up); // init position of servo
  // setup encoder
  pinMode(interrupt_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interrupt_pin), encoder, CHANGE);
  pinMode(led_pin, OUTPUT);
  Serial.begin(9600);
  Serial.println("Start Recording");
  Serial.println(""); 
}

// ISR routines
void encoder() {
  reward += 1; // increment total reward
  state = !state; // optional for LED status
}

void loop() {
  
  for (pos_down = 50; pos_down <= 150; pos_down += 1) { // goes from 0 degrees to 180 degrees
       arm_down.write(pos_down);              // tell servo to go to position in variable 'pos'
       delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos_up = 90; pos_up >= 50; pos_up -= 1) { // goes from 180 degrees to 0 degrees
       arm_up.write(pos_up);              // tell servo to go to position in variable 'pos'
       delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos_down = 150; pos_down >= 50; pos_down -= 1) { // goes from 180 degrees to 0 degrees
       arm_down.write(pos_down);              // tell servo to go to position in variable 'pos'
       delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos_up = 50; pos_up <= 90; pos_up += 1) { // goes from 180 degrees to 0 degrees
       arm_up.write(pos_up);              // tell servo to go to position in variable 'pos'
       delay(15);                       // waits 15ms for the servo to reach the position
  }
  digitalWrite(led_pin, state); // check current status by LED (optional)
  //Serial.print("State: ");
  //Serial.print(state);
  Serial.print(" Total Reward: "); // print out total reward
  Serial.println(reward);
}
