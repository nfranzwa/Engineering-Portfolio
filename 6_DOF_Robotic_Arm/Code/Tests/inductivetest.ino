//#define NPN4mm 9
const int sensorPin = 4; // Pin connected to the output of the sensor

void setup() {
  pinMode(sensorPin, INPUT);  // Set the sensor pin as an input
  //pinMode(NPN4mm, OUTPUT);    // Set the pin controlling the sensor power as an output
  //digitalWrite(NPN4mm, HIGH); // Keep the sensor powered throughout
  Serial.begin(9600);         // Start serial communication for debugging
}

void loop() {
  int sensorValue = digitalRead(sensorPin);  // Read the sensor output

  if (sensorValue == LOW) { // Object detected
    Serial.println("Object detected!");  // Print to serial monitor
  } else { // No object detected
    Serial.println("No object detected.");  // Print to serial monitor
  }

  delay(100);  // Wait for 100 milliseconds before checking again
}
