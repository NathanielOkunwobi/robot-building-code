//lineTrackingAlgorithm - uses sensors to determing whether or not the robot is on the
//and adjusts accordingly to make sure it remains on the line
//Written by Nathaniel Okunwobi

//defining the constants for the individual sensors on the tracking sensor module
#define lineSensor0 A0  
#define lineSensor1 A1
#define lineSensor2 A2
#define lineSensor3 A3
#define lineSensor4 A4  

//defining constants for the motor speeds and pins for controlling the motors
#define fastSpeed 150 
#define midSpeed 140 
#define slowSpeed 130 //back speed
#define rightSpeedPin 9 //pwm output for motor controller for right motors for speed
#define rightMotorDirPin1 12 //digital output for direction of right motor 1
#define rightMotorDirPin2 11 //digital output for direction of right motor 2
#define leftSpeedPin 6 //pwm output for motor controller for left motors for speed
#define leftMotorDirPin1 7 //digital output for direction for left motor 1
#define leftMotorDirPin2 8 //digital output for direction of right motor 2

//functions for motor control
//forward() function - moves the robot forward
void forward(void){
  digitalWrite(rightMotorDirPin1,HIGH);
  digitalWrite(rightMotorDirPin2,LOW);
  digitalWrite(leftMotorDirPin1,HIGH);
  digitalWrite(leftMotorDirPin2,LOW);
  analogWrite(leftSpeedPin,200);
  analogWrite(rightSpeedPin,200);
}
//left() function - turns the robot left
void left(int t=0){
  digitalWrite(rightMotorDirPin1,HIGH);
  digitalWrite(rightMotorDirPin2,LOW);
  digitalWrite(leftMotorDirPin1,LOW);
  digitalWrite(leftMotorDirPin2,HIGH);
  analogWrite(leftSpeedPin,200);
  analogWrite(rightSpeedPin,200);
  delay(t);
}
//right() function - turns the robot right
void right(int t=0){
  digitalWrite(rightMotorDirPin1,LOW);
  digitalWrite(rightMotorDirPin2,HIGH);
  digitalWrite(leftMotorDirPin1,HIGH);
  digitalWrite(leftMotorDirPin2,LOW);
  analogWrite(leftSpeedPin,200);
  analogWrite(rightSpeedPin,200);
  delay(t);
}
//backward() function - moves the robot backward
void backward(int t=0){
  digitalWrite(rightMotorDirPin1,LOW);
  digitalWrite(rightMotorDirPin2,HIGH);
  digitalWrite(leftMotorDirPin1,LOW);
  digitalWrite(leftMotorDirPin2,HIGH);
  analogWrite(leftSpeedPin,200);
  analogWrite(rightSpeedPin,200);
  delay(t);
}
//stop() function - stops the movement of the robot
void stop(){
  digitalWrite(rightMotorDirPin1,LOW);
  digitalWrite(rightMotorDirPin2,LOW);
  digitalWrite(leftMotorDirPin1,LOW);
  digitalWrite(leftMotorDirPin2,LOW);
}
//setSpeed() function - sets the speed of the motors based on input
void setSpeed(int leftSpeed,int rightSpeed){
  analogWrite(leftSpeedPin,leftSpeed); 
  analogWrite(rightSpeedPin,rightSpeed);   
}

//setup() function - sets the pin modes of each of the pins used
void setup(){
  pinMode(rightMotorDirPin1, OUTPUT); 
  pinMode(rightMotorDirPin2, OUTPUT); 
  pinMode(leftSpeedPin, OUTPUT);  
  pinMode(leftMotorDirPin1, OUTPUT);
  pinMode(leftMotorDirPin2, OUTPUT); 
  pinMode(rightSpeedPin, OUTPUT); 
  stop();  
  Serial.begin(9600); //initialize serial for debugging
}

//loop() function - continuously runs the lineTracking() function
void loop(){ 
 lineTracking();
}

//creates an empty character array consisting of 5 items
char sensor[5];

//getSensorValues() function - obtains the digital output from the sensors and
//then puts them into a string that is then returned where 1 stands for black and
//0 stands for white
String getSensorValues(){   
  int sensorValue=32;
  sensor[0]= !digitalRead(lineSensor0);
  sensor[1]=!digitalRead(lineSensor1);
  sensor[2]=!digitalRead(lineSensor2);
  sensor[3]=!digitalRead(lineSensor3);
  sensor[4]=!digitalRead(lineSensor4);
  sensorValue +=sensor[0]*16+sensor[1]*8+sensor[2]*4+sensor[3]*2+sensor[4];
  String senseStr= String(sensorValue,BIN);
  senseStr=senseStr.substring(1,6);
  return senseStr;
}

//lineTracking() function - moves the robot to remain on the line based on the 
//output from the getSensorValues() function
void lineTracking(){
 String sensorVal= getSensorValues();
  Serial.println(sensorVal);
  if (sensorVal=="10000"){
    //turn left as black line is on the left of the robot 
    left();
    setSpeed(fastSpeed,fastSpeed);
  }
  if (sensorVal=="10100"||sensorVal=="01000"||sensorVal=="01100"||sensorVal=="11100"||sensorVal=="10010"||sensorVal=="11010"){
    forward();
    setSpeed(0,fastSpeed); //slight left
  }
  if (sensorVal=="00001"){
    //turn right as the black line is on the right of the car
    right();  
    setSpeed(fastSpeed,fastSpeed);
  }
  if (sensorVal=="00011"||sensorVal=="00010"||sensorVal=="00101"||sensorVal=="00110"||sensorVal=="00111"||sensorVal=="01101"||sensorVal=="01111"||sensorVal=="01011"||sensorVal=="01001"){
    forward();
    setSpeed(fastSpeed,0); //slight right
  } 
  if (sensorVal=="11111"){
    stop();
    setSpeed(0,0); //stopping the car as the goal has been reached
  } 
}