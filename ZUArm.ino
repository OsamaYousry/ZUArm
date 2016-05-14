//Servo.h is the library required to directly deal with servo motors
#include <Servo.h>

/* Pinout is as follows, Arm from base to tool-tip
theta0 => 9
theta2 => 11
theta2 Second Motor is connected to same pin => 11
theta3 => 3
theta4 => 5
*/
int theta1Pin = 9;
int theta2Pin = 11;
int theta3Pin = 3;
int theta5Pin = 5;
int openGrip;

//We define a servo instance for each motor
Servo theta1Servo;
Servo theta2Servo;
Servo theta3Servo;
Servo theta4Servo;

void setup() {
  //Initialize Serial Com
  Serial.begin(9600);
  //Attach each instance with the corresponding pin
  theta1Servo.attach(theta1Pin);
  theta2Servo.attach(theta2Pin);
  theta3Servo.attach(theta3Pin);
  theta4Servo.attach(theta4Pin);
}
//Variables that will be sent by user
int theta1,theta2,theta3,theta4;
void loop() {
  //When the serial buffer contains the data we parse the data to send to the
  //motors, data is sent as follows xx yy zz w
  //Where xx is theta1 and yy is theta2 and zz is theta3 and w is the grip condition
  if (Serial.available()) {
   theta1 = Serial.readStringUntil(' ').toInt();
   theta2 = Serial.readStringUntil(' ').toInt();
   theta3 = Serial.readStringUntil(' ').toInt();
   //This was to fix a bug from matlab
   while (!Serial.available());
   openGrip = Serial.parseInt();
  }
  //We send the angles to the motors
  theta1Servo.write(theta1);
  theta2Servo.write(theta2);
  theta3Servo.write(theta3);
  //When the arm reaches its final destiantion we change the grip mode
  if (theta1Servo.read() == theta1 && theta2Servo.read() == theta2 && theta3Servo.read() == theta3) {
   if (openGrip == 1) {theta4Servo.write(180); }
   else { theta4Servo.write(0); }
  }
}
