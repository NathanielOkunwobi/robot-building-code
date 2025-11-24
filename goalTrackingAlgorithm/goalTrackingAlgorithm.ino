//[REMOVE ONCE DONE] TO DO:
//Add watch() and watchSurrounding() with checks for 3 directions
//switch to obstacle avoidance if any direction detects somethings
//switch back to goalTrack once it is done
//repeat until the goal position is reached

//goalTrackingAlgorithm - 
//
#include <Servo.h>

//definitions for the pins used
#define rightSpeedPin 9 //pwm output for motor controller for right motors for speed
#define rightMotorDirPin1 12 //digital output for direction of right motor 1
#define rightMotorDirPin2 11 //digital output for direction of right motor 2
#define leftSpeedPin 6 //pwm output for motor controller for left motors for speed
#define leftMotorDirPin1 7 //digital output for direction for left motor 1
#define leftMotorDirPin2 8 //digital output for direction of right motor 2
#define encoderPin 3 //
//#define LPT 2 //
#define servoPin 1 //
#define echoPin 2 //
#define triggerPin 10 //

//
long xEncoderCount = 0;
long yEncoderCount = 0;
bool xAxis = true;
bool yAxis = false;
int rotationTime = 1500; //tweak to get the robot to turn 90 degrees
//[REMOVE IF NOT NEEDED] add a constant for the speed of the robot
int leftScan, centerScan, rightScan, leftDiagonalScan, rightDiagonalScan;
const int distanceLimit = 15; //in cm, was 15
const int xGoal = 100;
const int yGoal = 100;

//forward() function - moves the robot forward
void forward(){
  digitalWrite(rightMotorDirPin1,HIGH);
  digitalWrite(rightMotorDirPin2,LOW);
  digitalWrite(leftMotorDirPin1,HIGH);
  digitalWrite(leftMotorDirPin2,LOW);
  analogWrite(leftSpeedPin,150); 
  analogWrite(rightSpeedPin,150);
}
//left() function - turns the robot left
void left(){
  if xAxis == true{
    xAxis = false;
    yAxis = true;
  }else{
    xAxis = true;
    yAxis = false;
  }
  digitalWrite(rightMotorDirPin1,HIGH);
  digitalWrite(rightMotorDirPin2,LOW);
  digitalWrite(leftMotorDirPin1,LOW);
  digitalWrite(leftMotorDirPin2,HIGH);
  analogWrite(leftSpeedPin,150); 
  analogWrite(rightSpeedPin,150);
  delay(rotationTime);
  stop();
}
//right() function - turns the robot right
void right(){
  if xAxis == true{
    xAxis = false;
    yAxis = true;
  }else{
    xAxis = true;
    yAxis = false;
  }
  digitalWrite(rightMotorDirPin1,LOW);
  digitalWrite(rightMotorDirPin2,HIGH);
  digitalWrite(leftMotorDirPin1,HIGH);
  digitalWrite(leftMotorDirPin2,LOW);
  analogWrite(leftSpeedPin,150); 
  analogWrite(rightSpeedPin,150);
  delay(rotationTime);
  stop(); 
}
//backward() function - moves the robot backward
void backward(){
  digitalWrite(rightMotorDirPin1,LOW);
  digitalWrite(rightMotorDirPin2,HIGH);
  digitalWrite(leftMotorDirPin1,LOW);
  digitalWrite(leftMotorDirPin2,HIGH);
  analogWrite(leftSpeedPin,150); 
  analogWrite(rightSpeedPin,150);  
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
  pinMode(encoderPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderISR, CHANGE);
  stop();  
  pinMode(triggerPin, OUTPUT); 
  pinMode(echoPin,INPUT); 
  digitalWrite(triggerPin,LOW);
  head.attach(servoPin); 
  head.write(90);
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
//[REMOVE ONCE DONE] may need to added a margin of error so that it can slightly undershoot or overshoot the exact x and y position 
void loop(){
  if ((xGoal == xEncoderCount) && (yGoal == yEncoderCount)){
    stop();
  }else{
    String obstacleSign = watchSurrounding();
    if(obstacleSign.indexOf("1") > 0){
      obstacleAvoidance(obstacleSign);
    }else{
      goalTrack();
    }
  }
}
//[REMOVE ONCE DONE, ADAPT TO RUN PROPERLY WITH THIS ALGORITHM]
void obstacleAvoidance(String obstacleSign) {
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
//
void goalTrack(){
  if ((xEncoderCount < xGoal)){
    if (xAxis == true){
      forward();
      delay(1000);
      stop();
    }else{
      right();
    }
  }
  else if ((xEncoderCount > xGoal)){
    if (xAxis == true){
      backward();
      delay(1000);
      stop();
    }else{
      right();
    }
  }
  else if ((yEncoderCount < yGoal)){
    if (yAxis == true){
      forward();
      delay(1000);
      stop();
    }else{
      left();
    }
  }
  else if ((yEncoderCount > yGoal)){
    if (yAxis == true){
      backward();
      delay(1000);
      stop();
    }else{
      left();
    }
  }
}
//
void encoderISR(){
  if backward == true{
    if xAxis == true{
      xAxis --;
    }else{
      yAxis --;
    }
  }else{
    if xAxis == true{
      xAxis ++;
    }else{
      yAxis ++;
    }
  }
}