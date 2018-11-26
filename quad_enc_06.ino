
/*
Purpose: Read Signals from Optical Rotary Encoder
Sensor: 2x SKU201381A
Author: roboball 11/2018
Link: https://robu.in/run-rotary-encoder-arduino-code/
*/
//these pins can not be changed 2/3 are special pins
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

void setup(){
  Serial.begin(9600); // start the serial monitor link
  pinMode(pin2, INPUT_PULLUP); // pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pin3, INPUT_PULLUP); // pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(digitalPinToInterrupt(pin2), isr_update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin3), isr_update, CHANGE);
}

void loop(){
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
