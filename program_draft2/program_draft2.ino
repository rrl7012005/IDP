#include <Adafruit_MotorShield.h>
#include "Servo.h"
#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"

//Declare motors and motor parameters
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *MotorL = AFMS.getMotor(1);
Adafruit_DCMotor *MotorR = AFMS.getMotor(2);
Servo myservo;
int servoPin = 10;
int open = 50;    // variable to store the servo open position
int closed = 90; //store the servo closed position
int servo_close_delay = 2; //servo closing speed
int servo_open_delay = 5; //servo opening speed

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

//Declare magnetic and TOF sensor
DFRobot_VL53L0X sensor;
int magnetPin = 11;
int distance = 0; //distance measured by tof sensor
int magnetVal = 0;
int threshold = 100; //distance threshold for box detection
int tof_distances[10] = {2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047}; //continuous distance measurements
const int size = 10; //size of above arrays

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
bool dropoff_mode = false; //special mode for dropping box off
bool override = false; //trigger next path step without detecting junction
bool recycling = false;
bool backward_mode = false;
bool box_on = false;
int main_path_num = -1; //index for the nth element of main path
int dropoff_path_num = -1; //index for nth element of dropoff path
int dropoff_index = 0; //index of which the next index corresponds to backtracking

//Declare timers
unsigned long startTime = 0; //a timer used for turning and to start the program
unsigned long dropoffTimer = 0; //timer for traversing the recycling/landfill junction
unsigned long ledTimer = 0; //timer to flash LEDs at 2Hz
int turn_delay = 220; //time for going forward before turning
int dropoff_time = 800; //time to go forward and backward in the recycling/landfill area
int blue_led_period = 500;
int pickup_time = 460; //time to go forward after detecting box and before picking up

//Define possible modes

enum Mode {ADVANCE, TURN};
enum Turn_dir {TURNING_LEFT, TURNING_RIGHT, NOTURN};
enum Direction {STRAIGHT, LEFT, RIGHT, DROPOFFL, DROPOFFR, BACKTRACK, NOTHING}; //dropoff L and R indicate respective sides the dropoff junction is reached.

//Declare path arrays

Direction main_path[] = {STRAIGHT, LEFT, RIGHT, STRAIGHT, RIGHT, DROPOFFL, RIGHT, RIGHT, RIGHT, DROPOFFL, LEFT, LEFT, LEFT, DROPOFFR, LEFT, RIGHT, RIGHT, STRAIGHT, RIGHT, STRAIGHT, RIGHT, DROPOFFL, LEFT, RIGHT, RIGHT, LEFT, STRAIGHT};
Direction dropoff_path[] = {RIGHT, RIGHT, BACKTRACK, LEFT, NOTHING, NOTHING}; //pad array with nothings as dropoff paths have variable lengths
int paths_with_box[] = {0, 5, 10, 16}; //path elements with a box on it
int paths_with_box_size = 4; //length of above array
int dropoffpathlength = 0; //number of elements in drop off path

//Note: boxes are on paths where main_path_num = 0, 5, 10, 16

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
  /* Function to check whether an element is in an array.
  It also modifies the detection threshold and pickup time for each box
  as required */

  if (value == 5) {
    threshold = 100;
    pickup_time = 430;
  } else if (value == 10) {
    threshold = 100;
    pickup_time = 450;
  } else if (value == 16) {
    threshold = 120;
    pickup_time = 400;
  }

  for (int i = 0; i < array_size; i++) {
    if (array[i] == value) {
      return true;
    }
  }
  return false;
}

int measure_distance() {
  //Measure TOF sensor distance reading, use int to save storage
  return sensor.getDistance();
}

void navigate_junction(Direction direction) {
  /*Function to decide what to do at each junction (or if override variable is true).
  Depending on the next direction of the traversed path, make the robot ready.*/

  switch (direction) {
    case STRAIGHT:
      startTime = millis(); //timer to ignore junction readings (prevents double detection)
      currentMode = ADVANCE;
      break;
    case LEFT:
      currentMode = TURN;
      turn_dir = TURNING_LEFT;
      startTime = millis(); //same as above but to also allow for further movement forward to a better position for turning
      break;
    case RIGHT:
      currentMode = TURN;
      turn_dir = TURNING_RIGHT;
      startTime = millis(); //same as above
      break;
    case BACKTRACK:
      dropoff(); //drop box off
      backward_mode = true;
      dropoff_time *= 0.8; //motors are more powerful while backing up
      dropoffTimer = millis(); //backtrack for same time as going forward as line following does not work well backwards
      break;

    case DROPOFFL:
      
      dropoff_path_num = -1;
      dropoff_mode = true;
      if (recycling) { 

        digitalWrite(redLED, HIGH);
        dropoff_index = 1; //after the 1st index theres backtrack
        dropoffpathlength = 3; //there are 4 elements (index up to 3)

        //Set dropoff path
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
    //Perform a slight right correction

    if (backward_mode) {
      //do not do line following backwards
      go_backward();
    } else {
      slight_forward_right();

    }

  } else if (line == "0100") {
    //Perfom a slight left correction

    if (backward_mode) {
      go_backward();

    } else {
      slight_forward_left();

    }

  }
}

void pickup() {

  //PICK BOX UP

  if (main_path_num == 0) {
    delay(200); //knock the first box over
  }
  
  stop_motors();
  box_on = true;

  //sweep servo open
  for (int pos = closed; pos >= open; pos--) {
    myservo.write(pos);
    delay(servo_open_delay);
  }
  
  go_forward();
  delay(pickup_time);
  stop_motors();

  //sweep servo close
  for (int pos = open; pos <= closed; pos++) {
    myservo.write(pos);
    delay(servo_close_delay);
  }

  delay(100);

  for (int i = 0; i < size; i++) {
    tof_distances[i] = 2047; //reset distance measurements 
  }
}

void dropoff() {
  //DROP BOX OFF AT AREA

  stop_motors();
  box_on = false;
  
  //sweep servo open
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
  /*Detect whether a box is present. If the last k values
  are below a threshold, then detect the object. k depends on
  the box */

  int counter = 0;

  for (int i = 0; i < size; i++) {
    if (tof_distances[i] <= threshold) {
      counter += 1;
    }
  }

  if (main_path_num == 5) {
    if (counter >= 4) {
      return true;
    } else {
      return false;
    }
  } else if (main_path_num == 10) {
    if (counter >= 4) {
      return true;
    } else {
      return false;
    }
  }

  if (counter >= 7) {
    return true;
  } else {
    return false;
  }
  
}

void update_distances() {
  /*Update continuous measurement of distances*/

  distance = measure_distance();

  for (int i = 0; i < size - 1; i++) {
    tof_distances[i] = tof_distances[i + 1];
  }

  tof_distances[size - 1] = distance;
}


void detect_magnetic() {
  //Detect magnet
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

    if (buttonState == HIGH) { //initiate program when button is pressed
      delay(200);
      startTime = millis();
      while ((millis() - startTime) < 300) { //if button is pressed twice or held, execute the emergency path to save time
        buttonState = digitalRead(buttonPin);
        if (buttonState == HIGH) {
          Direction temp_path[27] = {STRAIGHT, LEFT, RIGHT, STRAIGHT, RIGHT, DROPOFFL, RIGHT, RIGHT, RIGHT, DROPOFFL, RIGHT, LEFT, LEFT, RIGHT, STRAIGHT, STRAIGHT, STRAIGHT, STRAIGHT, STRAIGHT, STRAIGHT, STRAIGHT, STRAIGHT, STRAIGHT, STRAIGHT, STRAIGHT, STRAIGHT, STRAIGHT};
          for (int i = 0; i < 27; i++) {
            main_path[i] = temp_path[i];
          }
          Serial.println("CHANGING PATH DIR");
          break;
        }
      }
      startTime = 0;
      break;
    }
  }

  Serial.println("Running this program");
  Serial.println("Motor Shield found.");
  delay(2000); //Wait 2 seconds at start

  //I2C communication
  Wire.begin();
  sensor.begin(0x50);
  sensor.setMode(sensor.eContinuous, sensor.eHigh);
  sensor.start();

  ledTimer = millis();
}

void loop() { //main loop
  read_line_sensor(); //read sensor readings

  //Continuosly measure magnet sensor to maximize chance of positive result
  if (box_on) {
    detect_magnetic();
  }

  //Flash LED
  if ((millis() - ledTimer) > blue_led_period) {
    digitalWrite(blueLED, HIGH);
    delay(10);
    ledTimer = millis();
  } else {
    digitalWrite(blueLED, LOW);
  }

  //Go forward/backward for dropoff_time millisecs in dropoff area, doing line following if forward
  if ((millis() - dropoffTimer) < dropoff_time) {
    follow_line();
    return; //skip following code
  }
  
  //go forward for turn_delay millisecs before turning
  if ((millis() - startTime) < turn_delay) {
    follow_line();
    return;
  }

  if ((box_on == false) && (dropoff_mode == false) && (currentMode == ADVANCE)) {
    //if in advance mode (going straight) and there is no box and not in dropoff mode, then activate distance sensor (to prevent noisy readings)
    if (in_array(main_path_num, paths_with_box, paths_with_box_size)) {
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

        //if ready to drop box off
        if (dropoff_mode) {
          dropoff_path_num += 1;
          navigate_junction(dropoff_path[dropoff_path_num]); //traverse next element

          if (dropoff_path_num == dropoffpathlength) {
            //reached end of dropping off
            override = false;
            dropoff_mode = false;
          }

          if (dropoff_path_num == dropoff_index + 2 ) { //the index after backtracking (for backwards side turn to come out of recyling/landfill area)
            override = false; //override only activate for backtracking and backwards side turn as no junction there

            stop_motors();

            //close servo after backtracking
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

            if (R == 1) { //when outer sensor hits line, then start detecting end of turn
              get_ready_to_stop_turning = true;
            }

            if (line == "0010" && get_ready_to_stop_turning) { //need outer sensor to hit line first as other scenarios may trigger the desired reading
              delay(50); //prevent underturning
              backward_mode = false;
              currentMode = ADVANCE;
              get_ready_to_stop_turning = false;

              if (dropoff_path_num == dropoff_index) { //next mode is backtracking
                override = true;
                dropoffTimer = millis();
              }

              if (backward_mode) {
                go_backward();
              } else {
                go_forward();
              }
            }
          } else { //similar to above
            forward_left();

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
              }

              if (backward_mode) {
                go_backward();
              } else {
                go_forward();
              }
            }
          }

          break;

        case TURNING_RIGHT: //similar to above


          if (backward_mode) {
            backward_right();

            if (L == 1) {
              get_ready_to_stop_turning = true;
            }


            if (line == "0010" && get_ready_to_stop_turning) {
              delay(50);
              backward_mode = false;
              currentMode = ADVANCE;
              get_ready_to_stop_turning = false;

              if (dropoff_path_num == dropoff_index) {
                override = true;
                dropoffTimer = millis();
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
