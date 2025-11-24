/*
To Do (REMOVE ONCE DONE):
Test the code to make sure that it works
See how it copes with sharp angles needing to be navigated (may need to have a seek mode i.e. turning right and moving forward for when the wall is not present)
*/
//wallFollowingAlgorithm - 
//Written by Nathaniel Okunwobi (240349422)

//includes the servo library in the program
#include <Servo.h>

//defining constants for pin numbers and the scan loop counter
#define rightSpeedPin 3 //pwm output for right motors for speed
#define rightDirectionPin1 12 //pin for forward direction
#define rightDirectionPin2 11 //pin for backward direction
#define leftSpeedPin 6 //pwm output for right motors for speed
#define leftDirectionPin1 7 //pin for forward direction
#define leftDirectionPin2 8 //pin for backward direction
#define LPT 1 //check that this works as expected with 1 rather than 2, then replace with a comment
#define servoPin 9 //pin for signal to servo
#define echoPin 2 //pin for signal from ultrasonic sensor
#define triggerPin 10 //pin for signal to ultrasonic to send a sound wave out
#define fastSpeed 250 //value for fast motor speed
#define speed 120 //value for slower motor speed
#define turnSpeed 200 //value for motor speed when turning
#define highBackSpeed 255 //value for high motor speed
#define lowBackSpeed 90 //value for low motor speed

//defining variables for different positions in front of the robot as well as the
//parameters for the detection of objects
int centerScan;
const int distanceLimit = 10; //in cm
int distance;
int numCycles = 0;
const int turnTime = 275; //in miliseconds
int thereis;

//creating an object of the servo class called head
Servo head;

//functions for controlling movement of the robot
//forward() function - moves the  robot forward
void forward(void){ 
  digitalWrite(rightDirectionPin1, HIGH);
  digitalWrite(rightDirectionPin2,LOW);
  digitalWrite(leftDirectionPin1,HIGH);
  digitalWrite(leftDirectionPin2,LOW);
}
//left() function - turns the robot left
void left(){ 
  digitalWrite(rightDirectionPin1, HIGH);
  digitalWrite(rightDirectionPin2,LOW);
  digitalWrite(leftDirectionPin1,LOW);
  digitalWrite(leftDirectionPin2,HIGH);
}
//right() function - turns the robot right
void right(){
  digitalWrite(rightDirectionPin1, LOW);
  digitalWrite(rightDirectionPin2,HIGH);
  digitalWrite(leftDirectionPin1,HIGH);
  digitalWrite(leftDirectionPin2,LOW);
}
//backward() function - moves the robot backward
void backward(){
  digitalWrite(rightDirectionPin1, LOW);
  digitalWrite(rightDirectionPin2,HIGH);
  digitalWrite(leftDirectionPin1,LOW);
  digitalWrite(leftDirectionPin2,HIGH);
}
//stop() function - stops the robot
void stop(){ 
  digitalWrite(rightDirectionPin1, LOW);
  digitalWrite(rightDirectionPin2,LOW);
  digitalWrite(leftDirectionPin1,LOW);
  digitalWrite(leftDirectionPin2,LOW);
  setSpeed(0,0);
}
//setSpeed() function - sets the speed of the motors on left and right sides of the robot seperately
void setSpeed(int leftSpeed,int rightSpeed){
  analogWrite(leftSpeedPin,leftSpeed); 
  analogWrite(rightSpeedPin,rightSpeed);   
}

//watch() function - returns the calculated distance from the ultrasonic distance sensor
int watch(){
  long echoDistance;
  digitalWrite(triggerPin,LOW);
  delayMicroseconds(5);                                                                              
  digitalWrite(triggerPin,HIGH);
  delayMicroseconds(15);
  digitalWrite(triggerPin,LOW);
  echoDistance = pulseIn(echoPin,HIGH);
  echoDistance = echoDistance*0.01657; //object distance in cm
  return round(echoDistance);
}

//watchSide() function - checks if the distance from returned from the watch() function is less than the
//distance limit and returns 1 if it is
String watchSide(){
  int obstacleStatus = 0; //binary integer with one bit per direction
  centerScan = watch();
  if(centerScan < distanceLimit){
    stop();
    obstacleStatus = 1;
  }
  //delay(300); #check it functions fine without it
  String obstacleStr = String(obstacleStatus);
  return obstacleStr; //return 
}
//wallFollow() function - repeatedly calls watchSide() function and makes adjustments based on what it returns
void wallFollow(){
  ++numCycles;
  if(numCycles >= LPT){ //stops to check the surrounds every LPT cycles
    stop();
    String obstacleSign = watchSide(); //
    if(obstacleSign == "1"){ //move slightly left
      setSpeed(speed,fastSpeed);
      forward();
      delay(turnTime);
      stop();
    }
    numCycles=0; //restart cycle counter
  } 
  else{
    setSpeed(speed,speed);
    forward();  // if nothing is present in front moves forward
    delay(turnTime);
    stop();
  }
  distance = watch(); // use the watch() function to see if anything is ahead (when the robot is just moving forward and not looking around it will test the distance in front)
  if (distance < distanceLimit){ //the robot will just stop if it is completely sure there is an obstacle ahead after 25 tests
    left();
    setSpeed(speed,fastSpeed);
    delay(turnTime*3/2);
    ++thereis;
  }
  if (distance > distanceLimit){
    thereis = 0; //count of distance checks is restarted
  }
  if (thereis > 25){
    stop(); //stop moving as an obstacle is ahead
    thereis = 0;
  }
}
//setup() function - assigns the mode for each of the pins and starts serial communications
void setup(){
  pinMode(rightDirectionPin1, OUTPUT); 
  pinMode(rightDirectionPin2, OUTPUT); 
  pinMode(leftSpeedPin, OUTPUT);  
  pinMode(leftDirectionPin1, OUTPUT);
  pinMode(leftDirectionPin2, OUTPUT); 
  pinMode(rightSpeedPin, OUTPUT); 
  stop();//stop move
  pinMode(triggerPin, OUTPUT); 
  pinMode(echoPin,INPUT); 
  digitalWrite(triggerPin,LOW);
  head.attach(servoPin); 
  head.write(180);
  delay(2000);
  Serial.begin(9600);
}
//loop() functions - continuously runs the obstacleAvoidance() function
void loop(){
  wallFollow();
}