//odometerTesting - uses the odometers on either side of the robot to determine the 
//distance travelled by each side of the robot in centimetres

//definitions for the pins used
#define rightSpeedPin 9 //pwm output for motor controller for right motors for speed
#define rightMotorDirPin1 12 //digital output for direction of right motor 1
#define rightMotorDirPin2 11 //digital output for direction of right motor 2
#define leftSpeedPin 6 //pwm output for motor controller for left motors for speed
#define leftMotorDirPin1 7 //digital output for direction for left motor 1
#define leftMotorDirPin2 8 //digital output for direction of right motor 2
#define encoderLeft 2 //
#define encoderRight 3 //

//
String currentMovement = String("You should see this.");
long leftEncoderCount = 0;
long leftLastEncoderCount = 0;
long rightEncoderCount = 0;
long rightLastEncoderCount = 0;

//forward() function - moves the robot forward
void forward(){
  digitalWrite(rightMotorDirPin1,HIGH);
  digitalWrite(rightMotorDirPin2,LOW);
  digitalWrite(leftMotorDirPin1,HIGH);
  digitalWrite(leftMotorDirPin2,LOW);
  currentMovement = String("Forward");
}
//left() function - turns the robot left
void left(){
  digitalWrite(rightMotorDirPin1,HIGH);
  digitalWrite(rightMotorDirPin2,LOW);
  digitalWrite(leftMotorDirPin1,LOW);
  digitalWrite(leftMotorDirPin2,HIGH);
  analogWrite(leftSpeedPin,150); 
  analogWrite(rightSpeedPin,150);
  currentMovement = String("Left");
}
//right() function - turns the robot right
void right(){
  digitalWrite(rightMotorDirPin1,LOW);
  digitalWrite(rightMotorDirPin2,HIGH);
  digitalWrite(leftMotorDirPin1,HIGH);
  digitalWrite(leftMotorDirPin2,LOW);
  analogWrite(leftSpeedPin,150); 
  analogWrite(rightSpeedPin,150);
  currentMovement = String("Right"); 
}
//backward() function - moves the robot backward
void backward(){
  digitalWrite(rightMotorDirPin1,LOW);
  digitalWrite(rightMotorDirPin2,HIGH);
  digitalWrite(leftMotorDirPin1,LOW);
  digitalWrite(leftMotorDirPin2,HIGH);
  analogWrite(leftSpeedPin,150); 
  analogWrite(rightSpeedPin,150);  
  currentMovement = String("Backward");
}
//stop() function - stops the movement of the robot
void stop(){
  digitalWrite(rightMotorDirPin1,LOW);
  digitalWrite(rightMotorDirPin2,LOW);
  digitalWrite(leftMotorDirPin1,LOW);
  digitalWrite(leftMotorDirPin2,LOW);
  analogWrite(leftSpeedPin,0); 
  analogWrite(rightSpeedPin,0);  
  currentMovement = String("Stop");
}
//
void setup() {
  pinMode(rightMotorDirPin1, OUTPUT); 
  pinMode(rightMotorDirPin2, OUTPUT); 
  pinMode(leftSpeedPin, OUTPUT);  
  pinMode(leftMotorDirPin1, OUTPUT);
  pinMode(leftMotorDirPin2, OUTPUT); 
  pinMode(rightSpeedPin, OUTPUT); 
  pinMode(encoderLeft, INPUT);
  pinMode(encoderRight, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderLeft), encoderLeftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRight), encoderRightISR, CHANGE);
  stop();  
  Serial.begin(9600); //initialize serial for debugging
}
//
void loop() {
  testPathOne();
}
//
void testPathOne() {
  leftLastEncoderCount = leftEncoderCount;
  rightLastEncoderCount = rightEncoderCount;
  forward();
  Serial.println(currentMovement);
  delay(1500);
  stop();
  encoderCounter(leftLastEncoderCount, rightLastEncoderCount);

  leftLastEncoderCount = leftEncoderCount;
  rightLastEncoderCount = rightEncoderCount;
  backward();
  Serial.println(currentMovement);
  delay(1500);
  stop();
  encoderCounter(leftLastEncoderCount, rightLastEncoderCount);

  leftLastEncoderCount = leftEncoderCount;
  rightLastEncoderCount = rightEncoderCount;
  left();
  Serial.println(currentMovement);
  delay(1500);
  stop();
  encoderCounter(leftLastEncoderCount, rightLastEncoderCount);

  leftLastEncoderCount = leftEncoderCount;
  rightLastEncoderCount = rightEncoderCount;
  right();
  Serial.println(currentMovement);
  delay(1500);  
  stop();
  encoderCounter(leftLastEncoderCount, rightLastEncoderCount);
}
//
void testPathTwo() {
  leftLastEncoderCount = leftEncoderCount;
  rightLastEncoderCount = rightEncoderCount;
  forward();
  Serial.println(currentMovement);
  delay(1500);
  stop();
  encoderCounter(leftLastEncoderCount, rightLastEncoderCount);
}
//
void encoderCounter(int leftLastEncoderCount, int rightLastEncoderCount){
  Serial.println("Left Current Encoder Counter: " + leftEncoderCount);
  Serial.println("Change In Left Encoder Counter: " + (leftEncoderCount - leftLastEncoderCount));
  Serial.println("Right Current Encoder Counter: " + rightEncoderCount);
  Serial.println("Change In Right Encoder Counter: " + (rightEncoderCount - rightLastEncoderCount));
}
//
void encoderLeftISR(){
  leftEncoderCount++;
}
//
void encoderRightISR(){
  rightEncoderCount++;
}