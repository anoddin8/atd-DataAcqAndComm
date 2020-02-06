

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  Serial.begin(115200);
  int sensorPin = A0;
  pinMode(9,OUTPUT);

}

// the loop function runs over and over again forever
void loop() {
  int sensorValue;
  sensorValue = analogRead(A0);   // turn the LED on (HIGH is the voltage level)
  Serial.print(sensorValue);
  Serial.print(" ");
  delay(2000);                       // wait for a second
}
