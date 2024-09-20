const int limitSwitchPin = 3; // Pin connected to the limit switch
bool limitSwitchState = false;

void setup() {
  pinMode(limitSwitchPin, INPUT_PULLUP); // Enable internal pull-up resistor
  Serial.begin(9600);
}

void loop() {
  limitSwitchState = digitalRead(limitSwitchPin);
  
  if (limitSwitchState == LOW) {
    Serial.println("Limit switch open."); // When switch is not pressed
  } else {
    Serial.println("Limit switch closed!"); // When switch is pressed
  }
  
  delay(100); // Simple debouncing
}
