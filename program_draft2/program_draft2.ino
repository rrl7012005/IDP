#include <Adafruit_MotorShield.h>
#include "Servo.h"
#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"

DFRobot_VL53L0X sensor;


//TODO LIST
/*
6. Off-path algorithm (in between 3rd and 4th and after all)
7. Integrating everything together (build cover for line sensor)
8. Robustify the code, so not prone to noise changes, random sensor detections
9. Final fine tuning
*/

//Declare motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *MotorL = AFMS.getMotor(1);
Adafruit_DCMotor *MotorR = AFMS.getMotor(2);
Servo myservo;
int servoPin = 10;
int open = 50;    // variable to store the servo position
int closed = 90;
int servo_close_delay = 3;
int servo_open_delay = 15;

//Declare line sensors and set readings to default
int leftLineSensorPin = 2;
int centerleftLineSensorPin = 3;
int centerrightLineSensorPin = 4;
int rightLineSensorPin = 5;

int L = 0;
int R = 0;
int CL = 0;
int CR = 0;

String line = "";

//Declare magnetic sensors
int magnetPin = 11;
int distance = 0;
int magnetVal = 0;
int threshold = 100;

//Declare LEDs and buttons
int blueLED = 7;
int redLED = 8;
int greenLED = 6;
int buttonPin = 9;


//Declare driving constants
int MOTOR_SPEED = 255;
int SLOW_SPEED = 191;

//Declare state variables
bool get_ready_to_stop_turning = false;
bool dropoff_mode = false;
bool override = false;
bool recycling = false;
bool backward_mode = false;
bool box_on = false;
int main_path_num = -1; //index for the nth direction of main path
int dropoff_path_num = -1; //index for nth direction of dropoff path
int dropoff_index = 0; //index of which the next index corresponds to backtracking

unsigned long startTime = 0;
unsigned long dropoffTimer = 0;
unsigned long ledTimer = 0;
int turn_delay = 170;
int dropoff_time = 800;
int blue_led_period = 500;
int pickup_time = 400;

//Define possible modes

enum Mode {ADVANCE, TURN};
enum Turn_dir {TURNING_LEFT, TURNING_RIGHT, NOTURN};
enum Direction {STRAIGHT, LEFT, RIGHT, DROPOFFL, DROPOFFR, BACKTRACK, FULL180, NOTHING};

//Declare path arrays

Direction main_path[] = {STRAIGHT, LEFT, RIGHT, STRAIGHT, RIGHT, DROPOFFL, RIGHT, RIGHT, RIGHT, DROPOFFL, LEFT, LEFT, LEFT, DROPOFFR, LEFT, RIGHT, RIGHT, STRAIGHT, RIGHT, STRAIGHT, RIGHT, DROPOFFL, LEFT, RIGHT, RIGHT, LEFT, FULL180};
Direction dropoff_path[] = {RIGHT, RIGHT, BACKTRACK, LEFT, NOTHING, NOTHING}; //pad array with nothings as dropoff paths have variable lengths
int paths_with_box[] = {0, 5, 10, 16};
int paths_with_box_size = 4;
int tof_distances[10] = {2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047};
const int size = 10;
int dropoffpathlength = 0;

//Note: boxes are on paths where main_path_num = 0, 5, 10, 17
//once path_num=27 (the end) we stop the robot after a bit

//Set default modes
Mode currentMode = ADVANCE;
Turn_dir turn_dir = NOTURN;


void go_forward() {
  MotorL->setSpeed(MOTOR_SPEED);
  MotorR->setSpeed(MOTOR_SPEED);
  MotorL->run(BACKWARD);
  MotorR->run(BACKWARD);
}

void go_backward() {
  MotorL->setSpeed(MOTOR_SPEED);
  MotorR->setSpeed(MOTOR_SPEED);
  MotorL->run(FORWARD);
  MotorR->run(FORWARD);
}

void slight_forward_right() {
  MotorL->setSpeed(MOTOR_SPEED);
  MotorR->setSpeed(SLOW_SPEED);
  MotorL->run(BACKWARD);
  MotorR->run(BACKWARD);
}

void slight_forward_left() {
  MotorL->setSpeed(SLOW_SPEED);
  MotorR->setSpeed(MOTOR_SPEED);
  MotorL->run(BACKWARD);
  MotorR->run(BACKWARD);
}

void slight_backward_right() {
  MotorL->setSpeed(MOTOR_SPEED);
  MotorR->setSpeed(SLOW_SPEED);
  MotorL->run(FORWARD);
  MotorR->run(FORWARD);
}

void slight_backward_left() {
  MotorL->setSpeed(SLOW_SPEED);
  MotorR->setSpeed(MOTOR_SPEED);
  MotorL->run(FORWARD);
  MotorR->run(FORWARD);
}

void forward_right() {
  MotorL->setSpeed(MOTOR_SPEED);
  MotorL->run(BACKWARD);
  MotorR->run(RELEASE);
}

void forward_left() {
  MotorR->setSpeed(MOTOR_SPEED);
  MotorL->run(RELEASE);
  MotorR->run(BACKWARD);
}

void backward_left() {
  MotorR->setSpeed(MOTOR_SPEED);
  MotorL->run(RELEASE);
  MotorR->run(FORWARD);
}

void backward_right() {
  MotorL->setSpeed(MOTOR_SPEED);
  MotorL->run(FORWARD);
  MotorR->run(RELEASE);
}

void stop_motors() {
  MotorL->run(RELEASE);
  MotorR->run(RELEASE);
}

bool in_array(int value, int array[], int array_size) {
  for (int i = 0; i < array_size; i++) {
    if (array[i] == value) {
      return true;
    }
  }
  return false;
}

int measure_distance() {
  return sensor.getDistance();
}

void navigate_junction(Direction direction) {
  Serial.println(direction);
  dropoff_time = 800;
  switch (direction) {
    case STRAIGHT:
      startTime = millis();
      currentMode = ADVANCE;
      break;
    case LEFT:
      currentMode = TURN;
      turn_dir = TURNING_LEFT;
      startTime = millis();
      break;
    case RIGHT:
      currentMode = TURN;
      turn_dir = TURNING_RIGHT;
      startTime = millis();
      break;
    case BACKTRACK:
      dropoff();
      backward_mode = true;
      dropoffTimer = millis();
      dropoff_time = 800;
      break;

    case FULL180:
      delay(2000);
      MotorL->run(FORWARD);
      MotorR->run(BACKWARD);
      delay(1800);
      MotorL->run(FORWARD);
      MotorR->run(FORWARD);
      delay(1000);
      MotorL->run(RELEASE);
      MotorR->run(RELEASE);
      
      while (true) {
        ;
      }
      break;

    case DROPOFFL:
      
      dropoff_path_num = -1;
      dropoff_mode = true;
      if (recycling) {

        digitalWrite(redLED, HIGH);
        dropoff_index = 1;
        dropoffpathlength = 3;
        dropoff_path[0] = RIGHT;
        dropoff_path[1] = RIGHT;
        dropoff_path[2] = BACKTRACK;
        dropoff_path[3] = RIGHT;
        dropoff_path[4] = NOTHING;
        dropoff_path[5] = NOTHING;

      } else {

        digitalWrite(greenLED, HIGH);
        dropoff_index = 1;
        dropoffpathlength = 5;
        dropoff_path[0] = STRAIGHT;
        dropoff_path[1] = RIGHT;
        dropoff_path[2] = BACKTRACK;
        dropoff_path[3] = LEFT;
        dropoff_path[4] = LEFT;
        dropoff_path[5] = STRAIGHT;
      }
      break;
    case DROPOFFR:

      dropoff_path_num = -1;
      dropoff_mode = true;
      if (recycling) {

        digitalWrite(redLED, HIGH);
        dropoff_index = 2;
        dropoffpathlength = 4;
        dropoff_path[0] = STRAIGHT;
        dropoff_path[1] = LEFT;
        dropoff_path[2] = RIGHT;
        dropoff_path[3] = BACKTRACK;
        dropoff_path[4] = RIGHT;
        dropoff_path[5] = NOTHING;
      } else {

        digitalWrite(greenLED, HIGH);
        dropoff_index = 0;
        dropoffpathlength = 4;
        dropoff_path[0] = LEFT;
        dropoff_path[1] = BACKTRACK;
        dropoff_path[2] = LEFT;
        dropoff_path[3] = LEFT;
        dropoff_path[4] = STRAIGHT;
        dropoff_path[5] = NOTHING;
      }
      break;
  }
}


void read_line_sensor() {
  L = digitalRead(leftLineSensorPin);
  R = digitalRead(rightLineSensorPin);
  CL = digitalRead(centerleftLineSensorPin);
  CR = digitalRead(centerrightLineSensorPin);

  line = String(L) + String(CL) + String(CR) + String(R);

  // Serial.println(line);

}

void follow_line() {

  if (line == "0000") {
        
    if (backward_mode) {
      //go backward like normal
      go_backward();
      
    } else {
      //go forward like normal
      go_forward();

    }

  } else if (line == "0010") {
    //Perform a slight right correction depending on mode

    if (backward_mode) {
      // slight_backward_right();
      go_backward();
    } else {
      slight_forward_right();

    }

  } else if (line == "0100") {
    //Perfom a slight left correction

    if (backward_mode) {
      // slight_backward_left();
      go_backward();

    } else {
      slight_forward_left();

    }

  }
}

void pickup() {
  stop_motors();
  box_on = true;

  for (int pos = closed; pos >= open; pos--) {
    myservo.write(pos);
    delay(servo_open_delay);
  }
  
  go_forward();
  delay(pickup_time);
  stop_motors();

  for (int pos = open; pos <= closed; pos++) {
    myservo.write(pos);
    delay(servo_close_delay);
  }

  delay(100);

  for (int i = 0; i < size; i++) {
    tof_distances[i] = 2047;
  }
}

void dropoff() {
  stop_motors();
  box_on = false;
  
  for (int pos = closed; pos >= open; pos--) {
    myservo.write(pos);
    delay(servo_open_delay);
  } 

  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, LOW);
  recycling = false;
  magnetVal = 0;
}

bool is_there_box() {
  int counter = 0;

  for (int i = 0; i < size; i++) {
    if (tof_distances[i] <= threshold) {
      counter += 1;
    }
  }

  if (counter >= 7) {
    return true;
  } else {
    return false;
  }
  
}

void update_distances() {

  distance = measure_distance();

  for (int i = 0; i < size - 1; i++) {
    tof_distances[i] = tof_distances[i + 1];
  }

  tof_distances[size - 1] = distance;
}

void switch_recycling() {
  if (recycling) {
    recycling = false;
  } else {
    recycling = true;
  }
}

void detect_magnetic() {
  magnetVal = digitalRead(magnetPin);
  if (magnetVal == 1) {
    recycling = true;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);

  myservo.attach(servoPin);

  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }

  while (true) {
    int buttonState = digitalRead(buttonPin);

    if (buttonState == HIGH) {
      break;
    }
  }

  Serial.println("Running this program");
  Serial.println("Motor Shield found.");
  delay(2000); //Wait 2 seconds at start

  Wire.begin();
  sensor.begin(0x50);
  sensor.setMode(sensor.eContinuous, sensor.eHigh);
  sensor.start();

  ledTimer = millis();
}

void loop() {
  read_line_sensor();

  Serial.print("DISTANCE: ");
  Serial.print(distance);
  Serial.print(" PAth_NUM: ");
  Serial.println(main_path_num);
  Serial.print(" LINE SENSOR: ");
  Serial.println(line);

  if (box_on) {
    detect_magnetic();
  }

  if ((millis() - ledTimer) > blue_led_period) {
    digitalWrite(blueLED, HIGH);
    delay(10);
    ledTimer = millis();
  } else {
    digitalWrite(blueLED, LOW);
  }

  if ((millis() - dropoffTimer) < dropoff_time) {
    // Serial.println("HERE");
    follow_line();
    return;
  }
  
  if ((millis() - startTime) < turn_delay) {
    follow_line();
    return; //skip current iteration
  }

  if ((box_on == false) && (dropoff_mode == false) && (currentMode == ADVANCE)) {
    if (in_array(main_path_num, paths_with_box, paths_with_box_size)) {
      Serial.println("HERE");
      update_distances();
      if (is_there_box()) {
        pickup();
      }
    }
  }

  switch (currentMode) {
    case ADVANCE:

      //Detect a junction/turn or override sensor information
      if (R == 1 || L == 1 || override) {

        Serial.println("JUNCTION DETECTED");

        //if ready to drop box off
        if (dropoff_mode) {
          dropoff_path_num += 1;
          navigate_junction(dropoff_path[dropoff_path_num]);

          if (dropoff_path_num == dropoffpathlength) {
            //reached end of dropping off
            override = false;
            dropoff_mode = false;
          }

          if (dropoff_path_num == dropoff_index + 2 ) {
            override = false;

            stop_motors();

            for (int pos = open; pos <= closed; pos++) {
              myservo.write(pos);
              delay(servo_close_delay);
            }
          }

        } else {
          //Navigate normal path, not ready to dropoff yet
          main_path_num += 1;
          navigate_junction(main_path[main_path_num]);
        }

      } else {
        follow_line();

      }

      break;

    case TURN:

      switch (turn_dir) {
        case TURNING_LEFT:

          if (backward_mode) {
            backward_left();

            if (R == 1) {
              get_ready_to_stop_turning = true;
            }

            if (line == "0100" && get_ready_to_stop_turning) {
              backward_mode = false;
              currentMode = ADVANCE;
              get_ready_to_stop_turning = false;

              if (dropoff_path_num == dropoff_index) { //next mode is backtracking
                override = true;
                dropoffTimer = millis();
                //INSERT CODE
              }

              if (backward_mode) {
                go_backward();
              } else {
                go_forward();
              }
            }
          } else {
            forward_left();

            if (L == 1) {
              get_ready_to_stop_turning = true;
            }


            if (line == "0100" && get_ready_to_stop_turning) {
              backward_mode = false;
              currentMode = ADVANCE;
              get_ready_to_stop_turning = false;

              if (dropoff_path_num == dropoff_index) {
                override = true;
                dropoffTimer = millis();
                //INSERT CODE
              }

              if (backward_mode) {
                go_backward();
              } else {
                go_forward();
              }
            }
          }

          break;

        case TURNING_RIGHT:


          if (backward_mode) {
            backward_right();

            if (L == 1) {
              get_ready_to_stop_turning = true;
            }


            if (line == "0010" && get_ready_to_stop_turning) {
              backward_mode = false;
              currentMode = ADVANCE;
              get_ready_to_stop_turning = false;

              if (dropoff_path_num == dropoff_index) {
                override = true;
                dropoffTimer = millis();
                //INSERT CODE
              }

              if (backward_mode) {
                go_backward();
              } else {
                go_forward();
              }
            }
          } else {
            forward_right();

            if (R == 1) {
              get_ready_to_stop_turning = true;
            }

            if (line == "0010" && get_ready_to_stop_turning) {
              backward_mode = false;
              currentMode = ADVANCE;
              get_ready_to_stop_turning = false;

              if (dropoff_path_num == dropoff_index) { //next mode is backtracking
                override = true;
                dropoffTimer = millis();
                //INSERT CODE
              }

              if (backward_mode) {
                go_backward();
              } else {
                go_forward();
              }
            }
          }


          break;
      }

      break;

  }
}
