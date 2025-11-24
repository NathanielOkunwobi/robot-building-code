//servoMotorTest - testing the servo motor to see what different angles correspond
//to positions on the robot itself
//Written by Nathaniel Okunwobi (240349422)

//puts the servo library into the program
#include <Servo.h>

//defining constants for pin number
#define servoPin 9
//
Servo head;
//setup() function - setting the name for the servo pin that will be controlled
void setup(){
  head.attach(servoPin); 
  //head.write(90); #check it still works without the head
  delay(2000);
  Serial.begin(9600);
}
//loop() functions - repeatedly sets the position of the head to 30, 60, 90, 
//120, 150 and 180 degrees
void loop(){
  head.write(30);
  delay(2000);
  head.write(60);
  delay(2000);
  head.write(90);
  delay(2000);
  head.write(120);
  delay(2000);
  head.write(150);
  delay(2000);
  head.write(180);
  delay(2000);
}