
int leftLineSensorPin = 2;  // Connect left sensor to analog pin A0
int rightLineSensorPin = 5; // Connect right sensor to analog pin A1
int centerleftLineSensorPin = 3;
int centerrightLineSensorPin = 4;
int motorspeed = 255;
int halfspeed = 191;
int turn_time = 600;
int turn_delay = 450;
int i = 0;
int stop_turning = 0;
int junc_num = 0;
int angle = 0;

enum Mode { STRAIGHT, TURN };
enum Turn_dir {LEFT, RIGHT};

Mode currentMode = STRAIGHT;
Turn_dir turn_dir = LEFT;

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
  Motor2->setSpeed(0);
  Motor1->run(BACKWARD);
  Motor2->run(FORWARD);
}

void turn_left() {
  Motor1->setSpeed(0);
  Motor2->setSpeed(255);
  // Motor1->run(FORWARD);
  Motor2->run(BACKWARD);
  Motor1->run(FORWARD);
}

void exit_func() {
  while (true) {
    Motor1->run(RELEASE);
    Motor2->run(RELEASE);
  }
}

void loop(){

  int L = digitalRead(leftLineSensorPin);
  int R = digitalRead(rightLineSensorPin);
  int CL = digitalRead(centerleftLineSensorPin);
  int CR = digitalRead(centerrightLineSensorPin);

  String line = String(L) + String(CL) + String(CR) + String(R);

  Serial.println(line);

  if (currentMode == STRAIGHT) {
    // In Straight Mode
    if (line == "0000") {
      go_forward();
    } else if (line == "0010") {
      slow_right();
    } else if (line == "0100") {
      slow_left();

    } else if (R == 1) { 
      unsigned long startTime = millis();
      while (millis() - startTime < 100) {
        int L = digitalRead(leftLineSensorPin);
        if (L == 1) {
          junc_num += 1;
          break;
        }
      }

      if (junc_num != 1) {
        delay(turn_delay);
        turn_right();
        angle += 90;
        turn_dir = RIGHT;
        currentMode = TURN;
      }
      

    } else if (L == 1) { 
        // Left sensor detects a line - Start turn mode

      unsigned long startTime = millis();
      while (millis() - startTime < 100) {
        int R = digitalRead(rightLineSensorPin);
        if (R == 1) {
          junc_num += 1;
          break;
        }
      }

      if (junc_num != 1) {
        delay(turn_delay);
        turn_left();
        angle -= 90;
        turn_dir = LEFT;
        currentMode = TURN;
      }
      
    }

  } else if (currentMode == TURN) {

    if (turn_dir == LEFT) {
      if (L == 1) {
      stop_turning = 1;
      }
      if (line == "0010" && stop_turning == 1) {
        // Line detected in center sensors - Turn is complete
        currentMode = STRAIGHT;
        stop_turning = 0;
      }
    } else if (turn_dir == RIGHT) {
      if (R == 1) {
      stop_turning = 1;
      }
      if (line == "0100" && stop_turning == 1) {
        // Line detected in center sensors - Turn is complete
        currentMode = STRAIGHT;
        stop_turning = 0;
      }
    }
    
  }

  // if (i % 50 == 0) {
  //   Serial.println(currentMode);
  // }
  

  i ++;

  delay(10);
}
