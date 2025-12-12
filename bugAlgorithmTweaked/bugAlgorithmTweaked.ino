//bugAlgorithmTweaked - uses sensors to both operate in and switch between two modes
//of operation: line following and wall following
//Written by Nathaniel Okunwobi 

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
#define fastSpeed 230 //value for fast motor speed
#define speed 100 //value for slower motor speed
#define turnSpeed 180 //value for motor speed when turning
#define highBackSpeed 235 //value for high motor speed
#define lowBackSpeed 70 //value for low motor speed
#define rightObstacleSensor 4  //Right obstacle sensor to D2 (front direction is from arduino point to voltage meter)
#define leftObstacleSensor 5  //Left obstacle sensor to D3
#define lineSensor0 A0  
#define lineSensor1 A1
#define lineSensor2 A2
#define lineSensor3 A3
#define lineSensor4 A4  

//defining variables for different positions in front of the robot as well as the
//parameters for the detection of objects
int centerScan;
int distance;
int numCycles = 0;
const int turnTime = 90; //in miliseconds (was 150)
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
long watch(){
  long echoDistance;
  digitalWrite(triggerPin,LOW);
  delayMicroseconds(5);                                                                              
  digitalWrite(triggerPin,HIGH);
  delayMicroseconds(15);
  digitalWrite(triggerPin,LOW);
  echoDistance = pulseIn(echoPin,HIGH);
  echoDistance = echoDistance*0.01657; //object distance in cm
  return echoDistance;
}

//wallFollow() function - repeatedly calls watch() function and makes adjustments based on what it returns
void wallFollow(){
  stop();
  int leftIRVal= digitalRead(rightObstacleSensor);
  int rightIRVal=digitalRead(leftObstacleSensor);
  long obstacleDist = watch(); //
  if(leftIRVal==LOW && rightIRVal==LOW){
    setSpeed(speed,speed);
    backward();
    delay(200);
    setSpeed(fastSpeed,speed);
    right();
    delay(800);//*3/2);
  }
  if(obstacleDist <= 10){ 
    setSpeed(fastSpeed,speed);
    right();
    delay(turnTime);//*3/2);
  }
  if (obstacleDist > 10){ //the robot will just stop if it is completely sure there is an obstacle ahead after 25 tests
    setSpeed(speed,fastSpeed);
    left();
    delay(turnTime);//*3/2);
  }
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
    delay(turnTime);
  }
  if (sensorVal=="10100"||sensorVal=="01000"||sensorVal=="01100"||sensorVal=="11100"||sensorVal=="10010"||sensorVal=="11010"||sensorVal=="11000"){
    forward();
    setSpeed(0,fastSpeed); //slight left
    delay(turnTime);
  }
  if (sensorVal=="00001"){
    //turn right as the black line is on the right of the car
    right();  
    setSpeed(fastSpeed,fastSpeed);
    delay(turnTime);
  }
  if (sensorVal=="00011"||sensorVal=="00010"||sensorVal=="00101"||sensorVal=="00110"||sensorVal=="00111"||sensorVal=="01101"||sensorVal=="01111"||sensorVal=="01011"||sensorVal=="01001"){
    forward();
    setSpeed(fastSpeed,0); //slight right
    delay(turnTime);
  } 
  if (sensorVal=="11111"||sensorVal=="00000"||sensorVal=="01110"){
    //turn right as the black line is on the right of the car
    right();  
    setSpeed(fastSpeed,fastSpeed);
    delay(turnTime);
  } 
  stop();
  setSpeed(0,0);
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
  pinMode(rightObstacleSensor,INPUT); 
  pinMode(leftObstacleSensor,INPUT); 
  delay(2000);
  Serial.begin(9600);
}
//loop() function - continuously runs goalSequence() function until reaching the goal
bool goalReached = false;
bool wallHit = false;
bool lineHit = false;
String lastHit = "";
void loop(){
  if (!goalReached){
    goalSequence();
  }
  else{
    stop();
    setSpeed(0,0);
  }
}
//
void goalSequence(){
  int leftIRVal = digitalRead(rightObstacleSensor);
  int rightIRVal = digitalRead(leftObstacleSensor);
  if (!wallHit){
    String sensorVal= getSensorValues();
    if ((sensorVal=="11111") && (lastHit == "line")){
      stop();
      setSpeed(0,0); //stopping the car as the goal has been reached
      goalReached = true;
    }
    else if (leftIRVal==LOW && rightIRVal==LOW){
      stop();
      setSpeed(0,0); 
      setSpeed(fastSpeed,speed);
      right();
      delay(turnTime*3/4);
      stop();
      setSpeed(0,0); 
      lastHit = "wall";
      wallHit = true;
      lineHit = false;
    } 
    else{
      lineTracking();
    }
  }
  else if(!lineHit){
    String sensorVal= getSensorValues();
    if (sensorVal.indexOf("1") > 0){
      stop();
      setSpeed(0,0); //stopping the car as the goal has been reached
      right();
      setSpeed(fastSpeed,speed);
      delay(turnTime);//*3/2);
      stop();
      lastHit = "line";
      wallHit = false;
      lineHit = true;
    }
    else{
      wallFollow();
    }
  }
}