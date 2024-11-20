#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *Motor1 = AFMS.getMotor(3);
Adafruit_DCMotor *Motor2 = AFMS.getMotor(4);

int leftLineSensorPin = 2;  // Connect left sensor to analog pin A0
int rightLineSensorPin = 5; // Connect right sensor to analog pin A1
int centerleftLineSensorPin = 3;
int centerrightLineSensorPin = 4;

int motorspeed = 255;
int halfspeed = 191;
int turn_delay = 450;

int i = 0;
int stop_turning = 0;
int junction_detected = 0;
int path_num = -1;
int pause_mode = 0;
int np = -1;
int rubbish = 0;
int override = 0;

enum Mode {ADVANCE, TURN};
enum Turn_dir {LEF, RITE};
enum Direction {STRAIGHT, LEFT, RIGHT, PAUSEL, PAUSER, BACKTRACK};
Direction path[] = {STRAIGHT, LEFT, RIGHT, STRAIGHT, RIGHT, PAUSEL, RIGHT, RIGHT, RIGHT, PAUSEL, LEFT, LEFT, LEFT, STRAIGHT, PAUSER, LEFT, RIGHT, RIGHT, STRAIGHT, RIGHT, STRAIGHT, RIGHT, PAUSEL, LEFT, RIGHT, RIGHT, LEFT, STRAIGHT};
Direction new_path[] = {RIGHT, RIGHT, BACKTRACK, LEFT};

Mode currentMode = ADVANCE;
Turn_dir turn_dir = LEF;

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
  Motor2->run(BACKWARD);
  Motor1->run(FORWARD);
}

void exit_func() {
  while (true) {
    Motor1->run(RELEASE);
    Motor2->run(RELEASE);
  }
}

void cross_junction(Direction path){
  if (path == STRAIGHT) {
    Serial.println("STRAIGHT");
    currentMode = ADVANCE;
    delay(turn_delay);

  } else if (path == LEFT) {
    Serial.println("LEFT");
    currentMode = TURN;
    turn_dir = LEF;
    delay(turn_delay);
    turn_left();

  } else if (path == RIGHT) {
    Serial.println("RIGHT");
    currentMode = TURN;
    turn_dir = RITE;
    delay(turn_delay);
    turn_right();

  } else if (path == BACKTRACK) {

    Motor1->setSpeed(255);
    Motor2->setSpeed(255);

    Motor1->run(FORWARD);
    Motor2->run(FORWARD);

    delay(2000);

  } else if (path == PAUSEL) {

    np = -1;
    pause_mode = 1;

    if (rubbish == 0) {
      new_path[0] = RIGHT;
      new_path[1] = RIGHT;
      new_path[2] = BACKTRACK;
      new_path[3] = LEFT;
    } else if (rubbish == 1) {
      new_path[0] = RIGHT;
      new_path[1] = RIGHT;
      new_path[2] = BACKTRACK;
      new_path[3] = LEFT;
    }

  } else if (path == PAUSER) {
    
    np = -1;
    pause_mode = 1;
    if (rubbish == 0) {
      new_path[0] = LEFT;
      new_path[1] = RIGHT;
      new_path[2] = BACKTRACK;
      new_path[3] = LEFT;
    } else if (rubbish == 1) {
      new_path[0] = LEFT;
      new_path[1] = RIGHT;
      new_path[2] = BACKTRACK;
      new_path[3] = LEFT;
    }
  }
}


void loop() {
  // put your main code here, to run repeatedly:
  int L = digitalRead(leftLineSensorPin);
  int R = digitalRead(rightLineSensorPin);
  int CL = digitalRead(centerleftLineSensorPin);
  int CR = digitalRead(centerrightLineSensorPin);

  String line = String(L) + String(CL) + String(CR) + String(R);

  // Serial.println(line);

  if (currentMode == ADVANCE) {
    Serial.println("HERE 1");
    if (R == 1 || L == 1 || override == 1) {

      Serial.println("HERE 2");

      if (pause_mode == 1) {
        np += 1;
        cross_junction(new_path[np]);
        if (np == 1) {
          ;
        } else if (np == 2) {
          ;
        } else if (np == 3) {
          override = 0;
          pause_mode = 0;
        }
      } else {
        path_num += 1;
        cross_junction(path[path_num]);
      }
    } else if (line == "0000") {
      Serial.print("HERE 3");
      go_forward();
    } else if (line == "0010") {
      Serial.print("HERE 4");
      slow_right();
    } else if (line == "0100") {
      Serial.print("HERE 5");
      slow_left();
    }
    // } else if (R == 1) {
    //   unsigned long startTime = millis();
    //   while (millis() - startTime < 100) {
    //     int L = digitalRead(leftLineSensorPin);
    //     if (L == 1) {
    //       junction_detected = 1;
    //       path_num += 1;
    //       break;
    //     }
    //   }

    //   if (junction_detected == 1) {
    //     cross_junction(path[path_num]);
    //     junction_detected = 0;
    //   } else {
    //     delay(turn_delay);
    //     turn_right();
    //     turn_dir = RITE;
    //     currentMode = TURN;
    //   }

    // } else if (L == 1) {
    //   unsigned long startTime = millis();
    //   while (millis() - startTime < 100) {
    //     int R = digitalRead(rightLineSensorPin);
    //     if (R == 1) {
    //       junction_detected = 1;
    //       path_num += 1;
    //       break;
    //     }
    //   }
    
    //   if (junction_detected == 1) {
    //     cross_junction(path[path_num]);
    //     junction_detected = 0;
    //   } else {
    //     delay(turn_delay);
    //     turn_left();
    //     turn_dir = LEF;
    //     currentMode = TURN;
    //   }
    // }
  } else if (currentMode == TURN) {

    if (turn_dir == LEF) {

      if (L == 1) {
        stop_turning = 1;
      }

      if (line == "0010" && stop_turning == 1) {
        currentMode = ADVANCE;
        stop_turning = 0;
        if (np == 1) {
          override = 1;
          Motor1->setSpeed(255);
          Motor2->setSpeed(255);
          Motor1->run(BACKWARD);
          Motor2->run(BACKWARD);
          delay(1000);
        }
      }

    } else if (turn_dir == RITE) {

      if (R == 1) {
        stop_turning = 1;
      }

      if (line == "0100" && stop_turning == 1) {
        // Line detected in center sensors - Turn is complete
        currentMode = ADVANCE;
        stop_turning = 0;
        if (np == 1) {
          override = 1;
          Motor1->setSpeed(255);
          Motor2->setSpeed(255);
          Motor1->run(BACKWARD);
          Motor2->run(BACKWARD);
          delay(1000);
        }
      }
    }
  }
}
