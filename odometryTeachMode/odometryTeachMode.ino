//odometryTeachMode - counts the number of pulses sent by the odometer and displays 
//them in the serial monitor
//Written by Nathaniel Okunwobi

//definitions for the pins used
#define encoderLeftPin 2 
#define encoderRightPin 3

//defining the global variables for the encoder count for each side
long leftEncoderCount = 0;
long rightEncoderCount = 0;

//sets the mode for the encoder pins, attaches interupts to them and initialises 
//serial for debugging
void setup() {
  pinMode(encoderLeftPin, INPUT);
  pinMode(encoderRightPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderLeftPin), encoderLeftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRightPin), encoderRightISR, CHANGE);
  Serial.begin(4800); //initializes serial for debugging
}

//repeatedly called when the program is run
void loop() {
  encoderCount();
}

//prints the current encoder counts to the serial monitor
void encoderCount() {
  Serial.println(leftEncoderCount);
  Serial.println(rightEncoderCount);
}

//adds one to the left encoder count
void encoderLeftISR(){
  leftEncoderCount++;
}

//adds one to the right encoder count
void encoderRightISR(){
  rightEncoderCount++;
}