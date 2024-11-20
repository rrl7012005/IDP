int ledPin = 2; // choose the pin for the LED
int buttonPin = 4; // Connect sensor to input pin 3

void setup() {
  pinMode(ledPin, OUTPUT); // declare LED as output
  pinMode(buttonPin, INPUT); // declare pushbutton as input
  Serial.begin(9600);
}

void loop(){
  int val = digitalRead(buttonPin); // read input value

  if (val == HIGH) { // check if the input is HIGH
    digitalWrite(ledPin, LOW); // turn LED OFF
    Serial.println("BUTTON IS HIGH, LED SHOULD BE LOW");
  } else {
    digitalWrite(ledPin, HIGH); // turn LED ON
    Serial.println("BUTTON IS LOW, LED SHOULD BE HIGH");
  }
  delay(1000);
}
