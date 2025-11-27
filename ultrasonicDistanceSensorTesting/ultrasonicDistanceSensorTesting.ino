//ultrasonicDistanceSensor - outputs the distance from the ultrasonic distance sensor
//based on a calculation performed in centimetres
//Written by Nathaniel Okunwobi (240349422)

//definitions for ultrasonic pin
#define echoPin 2
#define triggerPin 10

//definition for pwm pin for voltmeter
#define ledOut 5

//global variable
int distance;

//watch() function - causes a sound pulse and based on the time taken to recieve
//it again it performs a calculation to get the distance and then round it, with the
//distance also being printed to the terminal
int watch(){
  long echoDistance;
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(5);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(15);
  digitalWrite(triggerPin, LOW);
  echoDistance = pulseIn(echoPin, HIGH);
  echoDistance = echoDistance * 0.01657;
  Serial.begin(9600);
  Serial.println(round(echoDistance));
  return round(echoDistance);
}

//distanceOutput() function - based on the distance recieved from the watch() function
//a pwm output is produced for the voltmeter display based on a mapping for the distance
//and the pwm value that is output
void distanceOutput(){
  distance = watch();
  int pwmValue = map(distance, 500, 0, 255, 0); // 0cm → 0V, 500cm → 5V
  pwmValue = constrain(pwmValue, 0, 255);
  analogWrite(ledOut, pwmValue);
}

//setup() function - sets the mode of each of the pins that is used
void setup(){
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledOut, OUTPUT);
  analogWrite(ledOut, 0);
}

//loop() function - continuously run the distanceOutput() function
void loop(){
  distanceOutput();
}
