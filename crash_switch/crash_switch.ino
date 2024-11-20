int ledPin = 2; // choose the pin for the LED
int crashswitchPin = 4; // Connect sensor to input pin 3

void setup() {
  Serial.begin(9600); // Init the serial port
  pinMode(ledPin, OUTPUT); // declare LED as output
  pinMode(crashswitchPin, INPUT); // declare Micro switch as input
}

void loop(){
  int val = digitalRead(crashswitchPin); // read input value
  if (val == HIGH) { // check if the input is HIGH
    digitalWrite(ledPin, LOW); // turn LED OFF
    Serial.println("Switch Reading High");
  } else {
    digitalWrite(ledPin, HIGH); // turn LED ON
    Serial.println("Switch Reading Low");
    Serial.println("Switch Pressed!");
  }
  delay(1000);
}