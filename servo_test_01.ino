/* Servo Test_code
Purpose: Control Two Servos
Author: Roboball 09/2018
*/

#include <Servo.h>

// create up to 12 servo objects
Servo arm_down;  
Servo arm_up;
// init constants
int servo_01 = 10;
int servo_02 = 11;
int pos_down = 0;    
int pos_up = 90; 

void setup() {
  // attach servos to pins
  arm_up.attach(servo_01);  
  arm_down.attach(servo_02);
  arm_up.write(pos_up);              // tell servo to go to position in variable 'pos'
}

void loop() {
 
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
}
