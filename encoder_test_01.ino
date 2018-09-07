/* Encoder Test_code
Purpose: Encoder Test
Author: Roboball 09/2018
*/

const int trackingPin = 3; //the tracking module attach to pin 2
const int ledPin = 13; //pin13 built-in led
int sum_high = 0; // sum up all high values
int sum_low= 0;  // sum up all low values

void setup(){
  pinMode(trackingPin, INPUT); // set trackingPin as INPUT
  pinMode(ledPin, OUTPUT); //set ledPin as OUTPUT
  Serial.begin(9600);
  Serial.println("Start Recording");
  Serial.println(""); 
  
}
void loop(){
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
  Serial.print("Sum High: ");
  Serial.println(sum_high);
  Serial.print("Sum Low: ");
  Serial.println(sum_low);
}
