// int leftlinesensorPin = 2;
// int rightlinesensorPin = 3; // Connect sensor to input pin 3

// void setup() {
//   Serial.begin(9600); // Init the serial port
//   pinMode(leftlinesensorPin, INPUT); // declare LED as output
//   pinMode(rightlinesensorPin, INPUT); // declare Micro switch as input
// }
// void loop(){
//   int valLeft = digitalRead(leftlinesensorPin); // read left input value
//   Serial.print(valLeft);
//   int valRight = digitalRead(rightlinesensorPin); // read right input value
//   Serial.println(valRight);
//   delay(100);
// }

int leftLineSensorPin = A0;  // Connect left sensor to analog pin A0

void setup() {
  Serial.begin(9600); // Initialize the serial port
}

void loop(){
  int valLeft = analogRead(leftLineSensorPin); // Read left sensor value
  Serial.println("Left: ");
  Serial.println(valLeft); // Print left sensor value
  
  // int valRight = analogRead(rightLineSensorPin); // Read right sensor value
  // Serial.print(" | Right: ");
  // Serial.println(valRight); // Print right sensor value
  
  delay(500); // Delay for readability in serial output
}
