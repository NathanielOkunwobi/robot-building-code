//commandFollower - follows the commands contained within an array 
//Written by Nathaniel Okunwobi

//definitions for the pins used
#define rightSpeedPin 9 //pwm output for motor controller for right motors for speed
#define rightMotorDirPin1 12 //digital output for direction of right motor 1
#define rightMotorDirPin2 11 //digital output for direction of right motor 2
#define leftSpeedPin 6 //pwm output for motor controller for left motors for speed
#define leftMotorDirPin1 7 //digital output for direction for left motor 1
#define leftMotorDirPin2 8 //digital output for direction of right motor 2
#define encoderRightPin 3 //right encoder output
#define encoderLeftPin 2 //left encoder output


//creating global variables
long rightEncoderCount = 0;
long leftEncoderCount = 0;
//routes to be followed, format is command then amount (encoder count for rotation and encoder count for translation), select between one of the two
//String robotSteps[] = {"Forward", "100", "Left", "50", "Forward", "100"}; //seqeunce to reach a given (x,y) location
String robotSteps[] = {"Forward", "50", "Left", "25", "Forward", "15", "Left", "30", "Forward", "50", "Right", "25", "Forward", "15", "Right", "30", "Forward", "50"}; //sequence for vacuum cleaner robot
int moveTime = 125;
int turnTime = 25;
bool runComplete = false;

//forward() function - moves the robot forward
void forward(){
  digitalWrite(rightMotorDirPin1,HIGH);
  digitalWrite(rightMotorDirPin2,LOW);
  digitalWrite(leftMotorDirPin1,HIGH);
  digitalWrite(leftMotorDirPin2,LOW);
  analogWrite(leftSpeedPin,150); 
  analogWrite(rightSpeedPin,150);  
  Serial.println("Forward");
}

 //backward() function - moves the robot backward
void backward(){
  digitalWrite(rightMotorDirPin1,LOW);
  digitalWrite(rightMotorDirPin2,HIGH);
  digitalWrite(leftMotorDirPin1,LOW);
  digitalWrite(leftMotorDirPin2,HIGH);
  analogWrite(leftSpeedPin,150); 
  analogWrite(rightSpeedPin,150);  
  Serial.println("Backward");
}

//left() function - turns the robot left
void left(){
  digitalWrite(rightMotorDirPin1,HIGH);
  digitalWrite(rightMotorDirPin2,LOW);
  digitalWrite(leftMotorDirPin1,LOW);
  digitalWrite(leftMotorDirPin2,HIGH);
  analogWrite(leftSpeedPin,150); 
  analogWrite(rightSpeedPin,0);
  Serial.println("Left");
}

//right() function - turns the robot right
void right(){
  digitalWrite(rightMotorDirPin1,LOW);
  digitalWrite(rightMotorDirPin2,HIGH);
  digitalWrite(leftMotorDirPin1,HIGH);
  digitalWrite(leftMotorDirPin2,LOW);
  analogWrite(leftSpeedPin,0); 
  analogWrite(rightSpeedPin,150);
  Serial.println("Right"); 
}

//stop() function - stops the movement of the robot
void stop(){
  digitalWrite(rightMotorDirPin1,LOW);
  digitalWrite(rightMotorDirPin2,LOW);
  digitalWrite(leftMotorDirPin1,LOW);
  digitalWrite(leftMotorDirPin2,LOW);
  analogWrite(leftSpeedPin,0); 
  analogWrite(rightSpeedPin,0);  
  Serial.println("Stop");
}

//sets the mode of all of the defined pins, attaches the ISR and sets up serial
void setup() {
  pinMode(rightMotorDirPin1, OUTPUT); 
  pinMode(rightMotorDirPin2, OUTPUT); 
  pinMode(leftSpeedPin, OUTPUT);  
  pinMode(leftMotorDirPin1, OUTPUT);
  pinMode(leftMotorDirPin2, OUTPUT); 
  pinMode(rightSpeedPin, OUTPUT); 
  pinMode(encoderRightPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderRightPin), encoderRightISR, RISING);
  pinMode(encoderLeftPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderLeftPin), encoderLeftISR, RISING);
  stop();  
  Serial.begin(4800); //initialize serial for debugging
}

//calls pathToGoal() continuously when the robot is on
void loop() {
  pathToGoal();
}

//goes through the robotSteps string array to get instructions and calls actionReader to run them
void pathToGoal(){
  String currentAction = "";
  int actionAmount = "";
  int arrLength = sizeof(robotSteps)/sizeof(robotSteps[0]);
  if (!runComplete){
    for (int i=0; i<arrLength; i+=2){
      actionReader(robotSteps[i],(robotSteps[i+1]).toInt());
    }
    runComplete = true;
  }
  else{
    stop();
  }
}

//interprets a string and a number into an instruction that the robot needs to perform
void actionReader(String action, int amount){
  rightEncoderCount = 0;
  leftEncoderCount = 0;
  if (action == "Forward"){
    while (rightEncoderCount < amount){
      forward();
      delay(moveTime);
      stop();
    }
    stop();
  }else if (action == "Backward"){
    while (rightEncoderCount < amount){
      backward();
      delay(moveTime);
      stop();
    }
    stop();
  }else if (action == "Left"){
    while (leftEncoderCount < amount){
      left();
      delay(turnTime);
      stop();
    }
    stop();
  }else if (action == "Right"){
    while (rightEncoderCount < amount){
      right();
      delay(turnTime);
      stop();
    }
    stop();
  }else{
    stop();
  }
  rightEncoderCount = 0;
  leftEncoderCount = 0;
}

//adds one to the left encoder count whenever a pulse is recieved from it
void encoderRightISR(){
  rightEncoderCount ++;
}

//adds one to the left encoder count whenever a pulse is recieved from it
void encoderLeftISR(){
  leftEncoderCount ++;
}