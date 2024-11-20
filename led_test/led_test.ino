
int led = 2;

void setup() {
  // put your setup code here, to run once:
  pinMode(led, OUTPUT); 
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(led, HIGH);
  Serial.println("HIGH");
  delay(2000);
  digitalWrite(led, LOW);
  Serial.println("LOW");
  delay(2000);
}
