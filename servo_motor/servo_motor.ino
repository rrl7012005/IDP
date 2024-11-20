#include <Servo.h> //servo motor is a library builtin

Servo myServo; //declare servo object


void setup() {
  // put your setup code here, to run once:
  myServo.attach(10); //attach the servo to a pin, pin 10

}

void loop() {
  // put your main code here, to run repeatedly:
  myServo.write(0); //.write rotates servo to a degree
  delay(1000);

  myServo.write(90); //rotate to 90 degrees (not rotate 90 degrees)
  delay(1000);

  myServo.write(180);
  delay(1000);

  myServo.write(270);
  delay(1000);

  myServo.write(360);
  delay(1000);
}
