/* Servo Rotary Encoder Test_code
Purpose: Control Two Servos and read out two Optical Rotary Encoders
Author: Roboball 11/2018
*/
#include <Servo.h>

// init rotary encoders
static int pin2 = 2;
static int pin3 = 3;
volatile int last_encoded = 0;
volatile long encoder_value = 0;
int last_MSB = 0;
int last_LSB = 0;
volatile int MSB = LOW;
volatile int LSB = LOW;
volatile int encoded;
volatile int sum;
volatile long pos_value = 0;
volatile long neg_value = 0;

// init servos
Servo arm_down;  
Servo arm_up;
// init constants
static int servo_01 = 9;
static int servo_02 = 10;
volatile int pos_up = 120;  // initial position servo 01 
volatile int pos_down = 0;  // initial position servo 02 

// manually adjust servo boundaries
static int low_bnd_01 = 50; // lower boundary, servo 01
static int up_bnd_01 = 120; // upper boundary, servo 01
static int low_bnd_02 = 30; // lower boundary, servo 02
static int up_bnd_02 = 140; // upper boundary, servo 02
 
void setup() {
  Serial.begin(9600); // start the serial monitor link
  // setup rotary encoders
  pinMode(pin2, INPUT_PULLUP); // pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pin3, INPUT_PULLUP); // pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(digitalPinToInterrupt(pin2), isr_update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin3), isr_update, CHANGE);
  // setup servos
  arm_up.attach(servo_01);  
  arm_down.attach(servo_02);
  arm_up.write(pos_up); // go to init position
}

void loop() {
 
  for (pos_down = low_bnd_02; pos_down <= up_bnd_02; pos_down += 1) { // goes from 0 degrees to 180 degrees
       arm_down.write(pos_down);              // tell servo to go to position in variable 'pos'
       delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos_up = up_bnd_01; pos_up >= low_bnd_01; pos_up -= 1) { // goes from 180 degrees to 0 degrees
       arm_up.write(pos_up);              // tell servo to go to position in variable 'pos'
       delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos_down = up_bnd_02; pos_down >=low_bnd_02; pos_down -= 1) { // goes from 180 degrees to 0 degrees
       arm_down.write(pos_down);              // tell servo to go to position in variable 'pos'
       delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos_up = low_bnd_01; pos_up <= up_bnd_01; pos_up += 1) { // goes from 180 degrees to 0 degrees
       arm_up.write(pos_up);              // tell servo to go to position in variable 'pos'
       delay(15);                       // waits 15ms for the servo to reach the position
  }
  // print out reward info
  Serial.print("Total: ");
  Serial.print(encoder_value);
  Serial.print(", Negative Reward: ");
  Serial.print(neg_value);
  Serial.print(", Positive Reward: ");
  Serial.println(pos_value);
  pos_value = 0;
  neg_value = 0;
  delay(500); //just for observation    
}

void isr_update(){
  MSB = digitalRead(pin2); //MSB = most significant bit
  LSB = digitalRead(pin3); //LSB = least significant bit

  encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  sum  = (last_encoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011){ 
    encoder_value ++;
    pos_value ++;
  }
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000){ 
    encoder_value --;
    neg_value --;
  }
  last_encoded = encoded; //store this value for next time
}
