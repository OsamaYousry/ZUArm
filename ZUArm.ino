//Servo.h is the library required to directly deal with servo motors
#include <Servo.h>
/*
 Pinout from base to grip:
 Motor 0 => 3
 Motor 1 & 2 => 5
 Motor 3 => 6
 Motor 4 => 9
 Motor 5 => 10
 Motor 6 => 11
 */
int thetaPin[] = {3,5,6,9,10,11};
// Introduce lower and upper limits for each motor due to mechanical
// limits
int lowerLimit[] = {0,0,0,0,0,16};
int upperLimit[] = {180,180,120,120,60};
// Number of motor pins
const int numberOfMotors = 6;
//We define a servo instance for each motor
Servo thetaServo[numberOfMotors];

void setup() {
  //Initialize Serial Com
  Serial.begin(9600);
  //Attach each instance with the corresponding pin
  for (int i = 0; i < numberOfMotors; i++) {
    thetaServo[i].attach(thetaPin[i]);
  }
  //Go to Home position
   for (int i = 0; i < numberOfMotors; i++) {
    thetaServo[i].write(lowerLimit[i]);
  } 
}
int Angle;
void loop() {
  /*
   Serial sends values in this format:
   xx yy:
   Where xx is the motor number, yy is the angle to be sent
   if xx is larger than the number of motors Arduino responds with
   Servo values
   */
  while (Serial.available()) {
   int servoNo = Serial.readStringUntil(' ').toInt();
   if (servoNo < numberOfMotors) {
     Angle = Serial.readStringUntil(':').toInt();
     // We check if the value of the Angle fits the limits
     if (Angle < lowerLimit[servoNo]) {
      thetaServo[servoNo].write(lowerLimit[servoNo]);
     } else if (Angle > upperLimit[servoNo]) {
      thetaServo[servoNo].write(upperLimit[servoNo]);
     } else {
      thetaServo[servoNo].write(Angle);
     }
     //For debugging purposes
//     Serial.print("Theta ");
//   Serial.print(servoNo);
//   Serial.print(" Was sent an angle of ");
//   Serial.println(Angle);
   } else {
    for (int i = 0; i < numberOfMotors; i++) {
      int val = thetaServo[i].read();
      Serial.println(val);
    }
   }
  }
}
