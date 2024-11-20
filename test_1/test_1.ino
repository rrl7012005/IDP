
int leftLineSensorPin = 2;  // Connect left sensor to analog pin A0
int rightLineSensorPin = 5; // Connect right sensor to analog pin A1
int centerleftLineSensorPin = 3;
int centerrightLineSensorPin = 4;
int motorspeed = 255;
int halfspeed = 191;
int turn_time = 600;
int turn_delay = 500;


#include <Adafruit_MotorShield.h>
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *Motor1 = AFMS.getMotor(3);
Adafruit_DCMotor *Motor2 = AFMS.getMotor(4);


void setup() {
  Serial.begin(9600); // Initialize the serial port

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Running this program");
  Serial.println("Motor Shield found.");

  Motor1->setSpeed(255);
  Motor2->setSpeed(255);
}

void go_forward() {
  Motor1->setSpeed(255);
  Motor2->setSpeed(255);
  Motor1->run(BACKWARD);
  Motor2->run(BACKWARD);
}

void slow_right() {
  Motor1->setSpeed(255);
  Motor2->setSpeed(191);
  Motor1->run(BACKWARD);
  Motor2->run(BACKWARD);
}

void slow_left() {
  Motor1->setSpeed(191);
  Motor2->setSpeed(255);
  Motor1->run(BACKWARD);
  Motor2->run(BACKWARD);
}

void turn_right() {
  Motor1->setSpeed(255);
  Motor2->setSpeed(255);
  Motor1->run(BACKWARD);
  Motor2->run(FORWARD);
}

void turn_left() {
  Motor1->setSpeed(255);
  Motor2->setSpeed(255);
  // Motor1->run(FORWARD);
  Motor2->run(BACKWARD);
  Motor1->run(FORWARD);
}

void loop(){

  int L = digitalRead(leftLineSensorPin);
  int R = digitalRead(rightLineSensorPin);
  int CL = digitalRead(centerleftLineSensorPin);
  int CR = digitalRead(centerrightLineSensorPin);

  String line = String(L) + String(CL) + String(CR) + String(R);

  Serial.print("LINE VALUE");
  Serial.println(line);

  if (line == "0000") {
    go_forward();
  } else if (line == "0010") {
    slow_right();
  } else if (line == "0100") {
    slow_left();
  } else if (line == "0001" || line == "0101" || line == "0111" || line == "0011") {
    delay(500);
    turn_right();
    delay(740);
  } else if (line == "1000" || line == "1010" || line == "1000" || line == "1100") {
    delay(500);
    turn_left();
    delay(740);
  } else if (line == "1111") {
    delay(500);
    turn_right();
    delay(740);
  } else if (line == "0110" || line == "1001" || line == "1011" || line == "1101") {
    go_forward();
  }

  delay(10);
}
