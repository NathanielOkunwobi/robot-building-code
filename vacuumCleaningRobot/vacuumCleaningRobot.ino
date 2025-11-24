/*
To Do (REMOVE ONCE DONE):
Comment the code
Get the wheel odometer and get that to work i.e. be giving output
Make the robot only rotate left, rotate right, move forward and move backward
Determine calculations for angle and distance move
Implement that into code 
The robot should go forward for a fixed distance and then go back on itself but slightly to one side
It should also beep when there is an obstacle in its way and it hasn't reached its distance goal
If another wheel odometer can be found then can simply get it to navigate to an x,y coordinate over and over again
Test the code to make sure that it works
*/
//vacuumCleaningRobot - uses
//Written by Nathaniel Okunwobi (240349422)

//
#include <Servo.h>

//defining constants for pin numbers and the scan loop counter
#define rightSpeedPin 3 //pwm output for right motors for speed
#define rightDirectionPin1 12 //
#define rightDirectionPin2 11 //
#define leftSpeedPin 6 //
#define leftDirectionPin1 7 //
#define leftDirectionPin2 8 //
#define LPT 1 //check that this works as expected with 1 rather than 2, then replace with a comment
#define servoPin 9 //
#define echoPin 2 //
#define triggerPin 10 //
#define fastSpeed 250 //
#define speed 120 //
#define turnSpeed 200 //
#define highBackSpeed 255 //
#define lowBackSpeed 90 //

//defining variables for different positions in front of the robot as well as the
//parameters for the detection of objects
int centerScan, leftDiagonalScan, rightDiagonalScan;
const int distanceLimit = 10; //in cm
int distance;
int numCycles = 0;
const int turnTime = 275; //in miliseconds
int thereis;

//
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
//setSpeed() function - 
void setSpeed(int leftSpeed,int rightSpeed){
  analogWrite(leftSpeedPin,leftSpeed); 
  analogWrite(rightSpeedPin,rightSpeed);   
}

//watch() function - 
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

//watchSurrounding() function - calls watch() function for 5 different directions
//and stores them in a binary integer that is then returned
String watchSurrounding(){
  int obstacleStatus = B1000; //binary integer with one bit per direction
  centerScan = watch();
  if(centerScan < distanceLimit){
    stop();
    obstacleStatus = obstacleStatus | B10;
    }
  head.write(120); //angle turned by the servo
  delay(100);
  leftDiagonalScan = watch();
  if(leftDiagonalScan < distanceLimit){
    stop();
    obstacleStatus = obstacleStatus | B100;
  }
  head.write(170);
  head.write(90);
  delay(100);
  centerScan = watch();
  if(centerScan<distanceLimit){
    stop();
    obstacleStatus = obstacleStatus | B10;
  }
  head.write(40);
  delay(100);
  rightDiagonalScan = watch();
  if(rightDiagonalScan<distanceLimit){
    stop();
    obstacleStatus = obstacleStatus | B1;
  }
  head.write(0);
  head.write(90); //head is facing forward again
  delay(300);
  String obstacleStr = String(obstacleStatus,BIN);
  obstacleStr = obstacleStr.substring(1,4);
  return obstacleStr; //return 3-character string standing for 3 direction obstacle status
}
//cleaningPath() function - 
void cleaningPath(){
  ++numCycles;
  if(numCycles >= LPT){ //stops to check the surrounds every LPT cycles
    stop();
    String obstacleSign = watchSurrounding(); //3 digits of obstacle_sign binary value means the 3 direction obstacle status
    if((obstacleSign == "100") || (obstacleSign == "110")){ //move left and back
      right();
      setSpeed(speed,fastSpeed);
      delay(turnTime);
      stop();
    }
    else if((obstacleSign == "011") || (obstacleSign == "001")){ //move right and back
	    left();
		  setSpeed(fastSpeed,speed);
      delay(turnTime);
      stop();
    }
    else if(obstacleSign == "111"){ //move backward
	    backward();
		  setSpeed(speed,speed);
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
    right();
    setSpeed(speed,fastSpeed);
    delay(turnTime*3/2);
    ++thereis;
  }
  if (distance > distanceLimit){
    thereis = 0; //count of distance checks is restarted
  }
  if (thereis > 25){
    stop(); //stop moving as an obstacle is ahead
    left()
    setSpeed(speed,speed);
    delay(turnTime);
    stop();
    thereis = 0;
  }
}
//setup() function - 
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
  //head.write(90); #check it still works without the head
  delay(2000);
  Serial.begin(9600);
}
//loop() functions - 
void loop(){
  cleaningPath();
}