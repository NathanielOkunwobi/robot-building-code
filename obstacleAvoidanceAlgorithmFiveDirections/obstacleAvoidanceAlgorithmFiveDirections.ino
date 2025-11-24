/*
To Do (REMOVE ONCE DONE):
Comment the code
Test the code to make sure that it works
*/
//obstacleAvoidanceAlgorithm - uses the ultrasonic sensor to prevent collision with
//objects by turning away from them when they get too close in a give scan direction
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
#define LPT 2 //
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
int leftScan, centerScan, rightScan, leftDiagonalScan, rightDiagonalScan;
const int distanceLimit = 15; //in cm, was 15
int distance;
int numCycles = 0;
const int turnTime = 350; //in miliseconds, was 275
int thereis;

//
Servo head;

//functions for controlling movement of the robot
//forward() function - moves the  robot forward
void forward(){ //removed void
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
  int obstacleStatus =B100000;
  centerScan = watch();
  if(centerScan<distanceLimit){
    stop();
    obstacleStatus  =obstacleStatus | B100;
    }
  head.write(120);
  delay(100);
  leftDiagonalScan = watch();
  if(leftDiagonalScan<distanceLimit){
    stop();
     obstacleStatus  =obstacleStatus | B1000;
    }
  head.write(170); //Didn't use 180 degrees because my servo is not able to take this angle
  delay(300);
  leftScan = watch();
  if(leftScan<distanceLimit){
    stop();
     obstacleStatus  =obstacleStatus | B10000;
    }

  head.write(90); //use 90 degrees if you are moving your servo through the whole 180 degrees
  delay(100);
  centerScan = watch();
  if(centerScan<distanceLimit){
    stop();
    obstacleStatus  =obstacleStatus | B100;
    }
  head.write(40);
  delay(100);
  rightDiagonalScan = watch();
  if(rightDiagonalScan<distanceLimit){
    stop();
    obstacleStatus  =obstacleStatus | B10;
    }
  head.write(0);
  delay(100);
  rightScan = watch();
  if(rightScan<distanceLimit){
    stop();
    obstacleStatus  =obstacleStatus | 1;
    }
  head.write(90); //Finish looking around (look forward again)
  delay(300);
   String obstacleStr= String(obstacleStatus,BIN);
  obstacleStr= obstacleStr.substring(1,6);
  
  return obstacleStr; //return 5-character string standing for 5 direction obstacle status

}
//obstacleAvoidance() function - 
void obstacleAvoidance(){
  ++numCycles;
  if(numCycles >= LPT){ //stops to check the surrounds every LPT cycles
    stop();
    String obstacleSign = watchSurrounding(); //5 digits of obstacle_sign binary value means the 3 direction obstacle status
    if(obstacleSign == "10000"){ //move slightly right
      setSpeed(fastSpeed,speed);
      forward();
      delay(turnTime);
      stop();
    }
    else if(obstacleSign == "00001"){ //move slightly left
      setSpeed(speed,fastSpeed);
      forward();
      delay(turnTime);
      stop();
    }
    else if(obstacleSign=="11100" || obstacleSign=="01000" || obstacleSign=="11000"  || obstacleSign=="10100"  || obstacleSign=="01100" ||obstacleSign=="00100"  ||obstacleSign=="01000" ){ //move hard right (?) and back
	    right();
		  setSpeed(turnSpeed,turnSpeed);
      delay(turnTime);
      stop();
    }
    else if(obstacleSign=="00010" || obstacleSign=="00111" || obstacleSign=="00011"  || obstacleSign=="00101" || obstacleSign=="00110" || obstacleSign=="01010"){
      left();
      setSpeed(turnSpeed,turnSpeed);
      delay(turnTime);
      stop();
    }
    else if(obstacleSign=="01111" || obstacleSign=="10111" || obstacleSign=="11111"){
	    left();
		  setSpeed(fastSpeed,speed);
      delay(turnTime);
      stop();
    } 
    else if(obstacleSign=="11011" || obstacleSign=="11101"  ||  obstacleSign=="11110"  || obstacleSign=="01110"  ){
      right();
      setSpeed(speed,fastSpeed);
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
  obstacleAvoidance();
}