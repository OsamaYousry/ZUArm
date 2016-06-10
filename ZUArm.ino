//Servo.h is the library required to directly deal with servo motors
#include <Servo.h>

/* Pinout is as follows, Arm from base to tool-tip
theta0 => 9
theta2 => 11
theta2 Second Motor is connected to same pin => 11
theta3 => 3
theta4 => 5
*/
int thetaPin[] = {3,5,6,9,10,11};
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
  for (int i = 0; i < numberOfMotors; i++) {
    thetaServo[i].write(0);
  }
}
int Angle;
void loop() {
  //When the serial buffer contains the data we parse the data to send to the
  //motors, data is sent as follows xx yy zz w
  //Where xx is theta1 and yy is theta2 and zz is theta3 and w is the grip condition
  while (Serial.available()) {
   int servoNo = Serial.readStringUntil(' ').toInt();
   if (servoNo < numberOfMotors) {
     Angle = Serial.readStringUntil(':').toInt();
     thetaServo[servoNo].write(Angle);
     Serial.print("Theta ");
   Serial.print(servoNo);
   Serial.print(" Was sent an angle of ");
   Serial.println(Angle);
   } else {
    Serial.println("HERE");
    for (int i = 0; i < numberOfMotors; i++) {
      int val = thetaServo[i].read();
      Serial.println(val);
    }
   }
  }
}
