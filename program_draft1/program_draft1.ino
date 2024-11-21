#include <Adafruit_MotorShield.h>

//Declare motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *MotorL = AFMS.getMotor(1);
Adafruit_DCMotor *MotorR = AFMS.getMotor(2);

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

//Declare LEDs


//Declare driving constants
int MOTOR_SPEED = 255;
int SLOW_SPEED = 191;

//Declare state variables
bool get_ready_to_stop_turning = false;
bool dropoff_mode = false;
bool turn_detection = true;
bool override = false;
bool recycling = false;
bool backward_mode = false;
int main_path_num = -1; //index for the nth direction of main path
int dropoff_path_num = -1; //index for nth direction of dropoff path

unsigned long startTime = 0;
unsigned long dropoffTimer = 0;
int turn_delay = 300;
int dropoff_time = 1000;

enum Mode {ADVANCE, TURN};
enum Turn_dir {TURNING_LEFT, TURNING_RIGHT, NOTHING};
enum Direction {STRAIGHT, LEFT, RIGHT, DROPOFFL, DROPOFFR, BACKTRACK};

//Declare path arrays

Direction main_path[] = {STRAIGHT, LEFT, RIGHT, STRAIGHT, RIGHT, DROPOFFL, RIGHT, RIGHT, RIGHT, DROPOFFL, LEFT, LEFT, LEFT, STRAIGHT, DROPOFFR, LEFT, RIGHT, RIGHT, STRAIGHT, RIGHT, STRAIGHT, RIGHT, DROPOFFL, LEFT, RIGHT, RIGHT, LEFT, STRAIGHT};
Direction dropoff_path[] = {RIGHT, RIGHT, BACKTRACK, LEFT};

//Note: boxes are on paths where main_path_num = 0, 5, 10, 17
//once path_num=27 (the end) we stop the robot after a bit

//Set default modes
Mode currentMode = ADVANCE;
Turn_dir turn_dir = NOTHING;


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

void navigate_junction(Direction direction) {
  Serial.println(direction);
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
      backward_mode = true;
      dropoffTimer = millis();
      break;
    case DROPOFFL:
      dropoff_path_num = -1;
      dropoff_mode = true;
      if (recycling) {
        dropoff_path[0] = RIGHT;
        dropoff_path[1] = RIGHT;
        dropoff_path[2] = BACKTRACK;
        dropoff_path[3] = RIGHT;
      } else {
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
        dropoff_path[0] = STRAIGHT;
        dropoff_path[1] = LEFT;
        dropoff_path[2] = RIGHT;
        dropoff_path[3] = BACKTRACK;
        dropoff_path[4] = RIGHT;
      } else {
        dropoff_path[0] = RIGHT;
        dropoff_path[1] = BACKTRACK;
        dropoff_path[2] = LEFT;
        dropoff_path[3] = LEFT;
        dropoff_path[4] = STRAIGHT;
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
      ;
    } else {
      slight_forward_right();

    }

  } else if (line == "0100") {
    //Perfom a slight left correction

    if (backward_mode) {
      // slight_backward_left();
      ;

    } else {
      slight_forward_left();

    }

  }
}

void pickup() {
  stop_motors();
  //INSERT CODE
}

void dropoff() {
  stop_motors();
  //INSERT CODE
}

bool is_there_box() {
  ;
}

void setup() {
  Serial.begin(9600);
  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }

  Serial.println("Running this program");
  Serial.println("Motor Shield found.");
  delay(2000); //Wait 2 seconds at start
}

void loop() {
  read_line_sensor();

  if (main_path_num == 0) {
    //INSERT CODE
    //SORT THIS OUT FURTHER, MAKE SURE ONCE IT PICKS, IT NO LOOK
    if (is_there_box()) {
      pickup();
    }
  }

  if ((millis() - dropoffTimer) < dropoff_time) {
    Serial.println("HERE");
    follow_line();
    return;
  }
  
  if ((millis() - startTime) < turn_delay) {
    follow_line();
    return; //skip current iteration
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

          if (dropoff_path_num == 3) {
            //reached end of dropping off
            override = false;
            dropoff_mode = false;
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

              if (dropoff_path_num == 1) { //next mode is backtracking
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


            if (line == "0010" && get_ready_to_stop_turning) {
              backward_mode = false;
              currentMode = ADVANCE;
              get_ready_to_stop_turning = false;

              if (dropoff_path_num == 1) {
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

              if (dropoff_path_num == 1) {
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

            if (line == "0100" && get_ready_to_stop_turning) {
              backward_mode = false;
              currentMode = ADVANCE;
              get_ready_to_stop_turning = false;

              if (dropoff_path_num == 1) { //next mode is backtracking
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
