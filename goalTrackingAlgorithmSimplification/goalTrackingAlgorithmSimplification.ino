//goalTrackingSimplification - uses the odometers on either side of the robot to determine the 
//distance travelled by each side of the robot
//Written by Nathaniel Okunwobi

//definitions for the pins used
#define rightSpeedPin 9 //pwm output for motor controller for right motors for speed
#define rightMotorDirPin1 12 //digital output for direction of right motor 1
#define rightMotorDirPin2 11 //digital output for direction of right motor 2
#define leftSpeedPin 6 //pwm output for motor controller for left motors for speed
#define leftMotorDirPin1 7 //digital output for direction for left motor 1
#define leftMotorDirPin2 8 //digital output for direction of right motor 2
#define encoderLeftPin 2 //left encoder output

//creating global variables
long leftEncoderCount = 0;
String robotSteps[] = {}; //fill in with route determined, format is command then amount (degrees for rotation and encoder count for translation)
int moveTime = 125;
int turnTime = 250;

//forward() function - moves the robot forward
void forward(){
  digitalWrite(rightMotorDirPin1,HIGH);
  digitalWrite(rightMotorDirPin2,LOW);
  digitalWrite(leftMotorDirPin1,HIGH);
  digitalWrite(leftMotorDirPin2,LOW);
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
  analogWrite(rightSpeedPin,150);
  Serial.println("Left");
}

//right() function - turns the robot right
void right(){
  digitalWrite(rightMotorDirPin1,LOW);
  digitalWrite(rightMotorDirPin2,HIGH);
  digitalWrite(leftMotorDirPin1,HIGH);
  digitalWrite(leftMotorDirPin2,LOW);
  analogWrite(leftSpeedPin,150); 
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
  pinMode(encoderLeftPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderLeftPin), encoderLeftISR, CHANGE);
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
  for (int i=0; i<arrLength; i+=2){
    actionReader(robotSteps[i],(robotSteps[i+1]).toInt());
  }
}

//interprets a string and a number into an instruction that the robot needs to perform
void actionReader(String action, int amount){
  if (action.indexOf("Forward") > 0){
    while (leftEncoderCount < amount){
      forward();
      delay(moveTime);
      stop();
    }
    stop();
    leftEncoderCount = 0;
  }else if (action.indexOf("Backward") > 0){
    while (leftEncoderCount < amount){
      backward();
      delay(moveTime);
      stop();
    }
    stop();
    leftEncoderCount = 0;
  }else if (action.indexOf("Left") > 0){
    if (amount == 0 || amount == 360){
      stop();
    }else if (amount == 90){
      left();
      delay(turnTime);
    }else if (amount == 180){
      left();
      delay(turnTime*2);
    }else if (amount == 270){
      left();
      delay(turnTime*3);
    }else{
      stop();
    }
    stop();
    leftEncoderCount = 0;
  }else if (action.indexOf("Right") > 0){
    if (amount == 0 || amount == 360){
      stop();
    }else if (amount == 90){
      right();
      delay(turnTime);
    }else if (amount == 180){
      right();
      delay(turnTime*2);
    }else if (amount == 270){
      right();
      delay(turnTime*3);
    }else{
      stop();
    }
    stop();
  }else{
    stop();
  }
  leftEncoderCount = 0;
}

//adds one to the left encoder count whenever a pulse is recieved from it
void encoderLeftISR(){
  leftEncoderCount ++;
}