#include <AccelStepper.h>
#include "pins.h"
#include "motorcst.h"
#include <SimpleFOC.h>

// Pin definitions for EE
BLDCMotor motor = BLDCMotor(11, 15, 80);
BLDCDriver3PWM driver = BLDCDriver3PWM(2, 45, 6, 52);

// Initialize AccelStepper for J1, J2, J3, J4, J5, and J6
AccelStepper stepperJ1(AccelStepper::DRIVER, J1_STEP_PIN, J1_DIR_PIN);
AccelStepper stepperJ2(AccelStepper::DRIVER, J2_STEP_PIN, J2_DIR_PIN);
AccelStepper stepperJ3(AccelStepper::DRIVER, J3_STEP_PIN, J3_DIR_PIN);
AccelStepper stepperJ4(AccelStepper::DRIVER, J4_STEP_PIN, J4_DIR_PIN);
AccelStepper stepperJ5(AccelStepper::DRIVER, J5_STEP_PIN, J5_DIR_PIN);
AccelStepper stepperJ6(AccelStepper::DRIVER, J6_STEP_PIN, J6_DIR_PIN);

bool homingInProgress = false;
bool shutdown = false;
bool resetting = false;
bool bldcOpen = true;

void setup() {
   // Pin configurations for J2
  pinMode(J2_ENABLE_PIN, OUTPUT);
  pinMode(J2limitswitch, INPUT_PULLUP); // Enable internal pull-up resistor
  
  // Pin configurations for J3
  pinMode(J3_ENABLE_PIN, OUTPUT);
  pinMode(J3limitswitch, INPUT_PULLUP); // Enable internal pull-up resistor

  // Pin configurations for J4
  pinMode(J4_ENABLE_PIN, OUTPUT);
  pinMode(J4limitswitch, INPUT);  // Set the sensor pin as an input
  pinMode(J4pwr, OUTPUT);    // Set the pin controlling the sensor power as an output
  digitalWrite(J4pwr, HIGH); // Keep the sensor powered throughout

  // Pin configurations for J1
  pinMode(J1_ENABLE_PIN, OUTPUT);
  pinMode(J1limitswitch, INPUT);  // Set the sensor pin as an input
  pinMode(J1pwr, OUTPUT);    // Set the pin controlling the sensor power as an output
  digitalWrite(J1pwr, HIGH); // Keep the sensor powered throughout

  // Pin configurations for J6
  pinMode(J6_ENABLE_PIN, OUTPUT);
  pinMode(J6limitswitch, INPUT);  // Set the sensor pin as an input

  // Pin configurations for J5
  pinMode(J5_ENABLE_PIN, OUTPUT);
  pinMode(J5limitswitch, INPUT_PULLUP); // Enable internal pull-up resistor

  Serial.begin(115200);
  
  // Enable motors
  digitalWrite(J1_ENABLE_PIN, LOW);
  digitalWrite(J2_ENABLE_PIN, LOW);
  digitalWrite(J3_ENABLE_PIN, LOW);
  digitalWrite(J4_ENABLE_PIN, LOW);
  digitalWrite(J5_ENABLE_PIN, LOW);
  digitalWrite(J6_ENABLE_PIN, LOW);

  // Set AccelStepper parameters for J1
  stepperJ1.setMaxSpeed(5000); // Maximum speed (steps per second)
  stepperJ1.setAcceleration(8000); // Acceleration (steps per second squared)

  // Set AccelStepper parameters for J2
  stepperJ2.setMaxSpeed(3500); // Maximum speed (steps per second)
  stepperJ2.setAcceleration(8000); // Acceleration (steps per second squared)
  
  // Set AccelStepper parameters for J3
  stepperJ3.setMaxSpeed(4000); // Maximum speed (steps per second)
  stepperJ3.setAcceleration(6000); // Acceleration (steps per second squared)
  
  // Set AccelStepper parameters for J4
  stepperJ4.setMaxSpeed(4000); // Maximum speed (steps per second)
  stepperJ4.setAcceleration(6000); // Acceleration (steps per second squared)

  // Set AccelStepper parameters for J5
  stepperJ5.setMaxSpeed(2000); // Maximum speed (steps per second)
  stepperJ5.setAcceleration(13000); // Acceleration (steps per second squared)

  // Set AccelStepper parameters for J6
  stepperJ6.setMaxSpeed(6000); // Maximum speed (steps per second)
  stepperJ6.setAcceleration(6000); // Acceleration (steps per second squared)    

    // BLDC motor setup
  driver.voltage_power_supply = 24;
  driver.init();
  motor.linkDriver(&driver);

  motor.voltage_limit = NORMAL_VOLTAGE;   // [V]
  motor.velocity_limit = 10; // [rad/s]
  motor.controller = MotionControlType::angle_openloop;

  motor.init();
  motor.initFOC();

  openBLDC();
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "H" && !homingInProgress) {
      homingInProgress = true;
      startHomingSequence();
    }
    else if (command == "S" && !shutdown) {
      shutdown = true;
      startshutdown();
    }
    else if (command == "R" && !resetting) {
      resetting = true;
      resetAllJoints();
    }
    else if (command.startsWith("M")) {
      processJointAngles(command.substring(1));
    }
    else if (command == "OPEN") {
      openBLDC();
    }
    else if (command == "CLOSE") {
      closeBLDC();
    }
  }
  
  // Run all steppers
  stepperJ1.run();
  stepperJ2.run();
  stepperJ3.run();
  stepperJ4.run();
  stepperJ5.run();
  stepperJ6.run();
   
  // BLDC motor control
    motor.loopFOC();
    motor.move();
}

void openBLDC() {
  motor.voltage_limit = NORMAL_VOLTAGE;  // Set normal voltage for movement
  motor.move(0);  // Move to open position
  delay(500);  // Wait for movement to complete
  motor.voltage_limit = LOW_VOLTAGE;  // Set low voltage for holding
  bldcOpen = true;
  Serial.println("BLDC motor opened and set to low power mode");
}

void closeBLDC() {
  motor.voltage_limit = NORMAL_VOLTAGE;  // Set normal voltage for movement
  motor.move(2.65);  // Move to closed position
  bldcOpen = false;
  Serial.println("BLDC motor closed");
}

void processJointAngles(String angleString) {
  int commaIndex = 0;
  int nextCommaIndex = 0;
  float angles[6];
  
  for (int i = 0; i < 6; i++) {
    nextCommaIndex = angleString.indexOf(',', commaIndex);
    if (nextCommaIndex == -1) {
      nextCommaIndex = angleString.length();
    }
    
    angles[i] = angleString.substring(commaIndex, nextCommaIndex).toFloat();
    commaIndex = nextCommaIndex + 1;
  }
  
  moveJoint(1, angles[0]);
  moveJoint(2, angles[1]);
  moveJoint(3, angles[2]);
  moveJoint(4, angles[3]);
  moveJoint(5, angles[4]);
  moveJoint(6, angles[5]);
}

void resetAllJoints() {
  Serial.println("Resetting all joints to 0...");
  stepperJ1.moveTo(calculateStepsJ1(0));
  stepperJ2.moveTo(calculateStepsJ2(0));
  stepperJ3.moveTo(calculateStepsJ3(0));
  stepperJ4.moveTo(calculateStepsJ4(0));
  stepperJ5.moveTo(calculateStepsJ5(0));
  stepperJ6.moveTo(calculateStepsJ6(0));
}

bool runToZero(AccelStepper &stepper) {
  if (stepper.distanceToGo() != 0) {
    stepper.run();
    return false;
  }
  return true;
}

void moveJoint(int jointNumber, float targetAngle) {
  switch (jointNumber) {
    case 1:
      stepperJ1.moveTo(calculateStepsJ1(targetAngle));
      break;
    case 2:
      stepperJ2.moveTo(calculateStepsJ2(targetAngle));
      break;
    case 3:
      stepperJ3.moveTo(calculateStepsJ3(targetAngle));
      break;
    case 4:
      stepperJ4.moveTo(calculateStepsJ4(targetAngle));
      break;
    case 5:
      stepperJ5.moveTo(calculateStepsJ5(targetAngle));
      break;
    case 6:
      stepperJ6.moveTo(calculateStepsJ6(targetAngle));
      break;
  }
}

void startHomingSequence() {
  Serial.println("Starting homing sequence...");
  
  homeAxesJ1J2J3Simultaneously();
  homeAxesJ4J6Simultaneously();
  homeAxisJ5();

  stepperJ1.setCurrentPosition(0);
  stepperJ2.setCurrentPosition(0);
  stepperJ3.setCurrentPosition(0);
  stepperJ4.setCurrentPosition(0);
  stepperJ5.setCurrentPosition(0);
  stepperJ6.setCurrentPosition(0);

  Serial.println("Homing sequence completed.");
  homingInProgress = false;
}

void startshutdown() {
  Serial.println("Shutdown sequence initiated...");

  // Calculate steps for 10 degree movement for each joint from home position
  int stepsJ1 = calculateStepsJ1(15);
  int stepsJ2 = calculateStepsJ2(59);
  int stepsJ3 = calculateStepsJ3(25);
  int stepsJ4 = calculateStepsJ4(0);
  int stepsJ5 = calculateStepsJ5(-87);
  int stepsJ6 = calculateStepsJ6(-140);

  // Set target positions (absolute movement)
  stepperJ1.moveTo(stepsJ1);
  stepperJ2.moveTo(stepsJ2);
  stepperJ3.moveTo(stepsJ3);
  stepperJ4.moveTo(stepsJ4);
  stepperJ5.moveTo(stepsJ5);
  stepperJ6.moveTo(stepsJ6);

  // Move all motors simultaneously
  while (stepperJ1.distanceToGo() != 0 || stepperJ2.distanceToGo() != 0 || 
         stepperJ3.distanceToGo() != 0 || stepperJ4.distanceToGo() != 0 || 
         stepperJ5.distanceToGo() != 0 || stepperJ6.distanceToGo() != 0) {
    stepperJ1.run();
    stepperJ2.run();
    stepperJ3.run();
    stepperJ4.run();
    stepperJ5.run();
    stepperJ6.run();
  }

  Serial.println("All joints moved to +10 degrees from home position.");
  Serial.println("Shutdown sequence completed.");
  shutdown = false;
}

void homeAxesJ1J2J3Simultaneously() {
  Serial.println("Homing J1, J2, and J3 simultaneously...");
  
  // Set initial directions and positions
  digitalWrite(J1_DIR_PIN, HIGH); // Set J1 direction to move towards the sensor
  stepperJ1.setCurrentPosition(0);
  stepperJ1.moveTo(-1000000); // Move J1 towards its limit switch
  
  stepperJ2.moveTo(-1000000); // Move J2 towards its limit switch
  
  digitalWrite(J3_DIR_PIN, LOW);
  stepperJ3.setCurrentPosition(0);
  stepperJ3.moveTo(1000000); // Move J3 towards its limit switch
  
  // Phase 1: Move all motors until their limit switches are triggered
  bool j1Homed = false;
  bool j2Homed = false;
  bool j3Homed = false;
  
  while (!j1Homed || !j2Homed || !j3Homed) {
    if (!j1Homed && digitalRead(J1limitswitch) == HIGH) {
      stepperJ1.run();
    } else if (!j1Homed) {
      j1Homed = true;
      stepperJ1.stop();
      Serial.println("J1 limit switch triggered");
    }

    if (!j2Homed && digitalRead(J2limitswitch) == LOW) {
      stepperJ2.run();
    } else if (!j2Homed) {
      j2Homed = true;
      stepperJ2.stop();
      Serial.println("J2 limit switch triggered");
    }
    
    if (!j3Homed && digitalRead(J3limitswitch) == LOW) {
      stepperJ3.run();
    } else if (!j3Homed) {
      j3Homed = true;
      stepperJ3.stop();
      Serial.println("J3 limit switch triggered");
    }
  }
  
  // Phase 2: Back off all motors
  int backOffStepsJ1 = calculateStepsJ1(backOffAngleJ1);
  int backOffStepsJ2 = calculateStepsJ2(backOffAngleJ2);
  int backOffStepsJ3 = calculateStepsJ3(backOffAngleJ3);
  
  stepperJ1.moveTo(stepperJ1.currentPosition() + backOffStepsJ1);
  stepperJ2.moveTo(stepperJ2.currentPosition() + backOffStepsJ2);
  stepperJ3.moveTo(stepperJ3.currentPosition() - backOffStepsJ3);
  
  while (stepperJ1.distanceToGo() != 0 || stepperJ2.distanceToGo() != 0 || stepperJ3.distanceToGo() != 0) {
    stepperJ1.run();
    stepperJ2.run();
    stepperJ3.run();
  }
  
  // Phase 3: Approach limit switches slowly
  stepperJ1.setMaxSpeed(400);
  stepperJ2.setMaxSpeed(500);
  stepperJ3.setMaxSpeed(350);
  
  stepperJ1.moveTo(stepperJ1.currentPosition() - 2 * backOffStepsJ1);
  stepperJ2.moveTo(stepperJ2.currentPosition() - 2 * backOffStepsJ2);
  stepperJ3.moveTo(stepperJ3.currentPosition() + 2 * backOffStepsJ3);
  
  j1Homed = j2Homed = j3Homed = false;
  
  while (!j1Homed || !j2Homed || !j3Homed) {
    if (!j1Homed && digitalRead(J1limitswitch) == HIGH) {
      stepperJ1.run();
    } else if (!j1Homed) {
      j1Homed = true;
      stepperJ1.stop();
      Serial.println("J1 homed accurately");
    }

    if (!j2Homed && digitalRead(J2limitswitch) == LOW) {
      stepperJ2.run();
    } else if (!j2Homed) {
      j2Homed = true;
      stepperJ2.stop();
      Serial.println("J2 homed accurately");
    }
    
    if (!j3Homed && digitalRead(J3limitswitch) == LOW) {
      stepperJ3.run();
    } else if (!j3Homed) {
      j3Homed = true;
      stepperJ3.stop();
      Serial.println("J3 homed accurately");
    }
  }
  
  // Set home positions
  stepperJ1.setCurrentPosition(calculateStepsJ1(homeAngleJ1));
  stepperJ2.setCurrentPosition(calculateStepsJ2(homeAngleJ2));
  stepperJ3.setCurrentPosition(calculateStepsJ3(homeAngleJ3));
  
  // Move to target angles simultaneously
  Serial.println("Moving J1, J2, and J3 to target angles simultaneously...");
  int stepsToMoveJ1 = calculateStepsJ1(targetAngleJ1);
  int stepsToMoveJ2 = calculateStepsJ2(targetAngleJ2);
  int stepsToMoveJ3 = calculateStepsJ3(targetAngleJ3);
  
  stepperJ1.setMaxSpeed(6000);
  stepperJ1.setAcceleration(8000);
  stepperJ2.setMaxSpeed(12000);
  stepperJ2.setAcceleration(8000);
  stepperJ3.setMaxSpeed(14000);
  stepperJ3.setAcceleration(8000);
  
  stepperJ1.moveTo(stepsToMoveJ1);
  stepperJ2.moveTo(stepsToMoveJ2);
  stepperJ3.moveTo(stepsToMoveJ3);
  
  while (stepperJ1.distanceToGo() != 0 || stepperJ2.distanceToGo() != 0 || stepperJ3.distanceToGo() != 0) {
    stepperJ1.run();
    stepperJ2.run();
    stepperJ3.run();
  }
  
  Serial.println("J1, J2, and J3 homing and movement to target angles complete.");
}

void homeAxesJ4J6Simultaneously() {
  Serial.println("Homing J4 and J6 simultaneously...");
  
  // Set initial directions and positions
  digitalWrite(J4_DIR_PIN, HIGH); // Set J4 direction to move towards the sensor
  stepperJ4.setCurrentPosition(0);
  stepperJ4.moveTo(1000000); // Move J4 towards its limit switch
  
  digitalWrite(J6_DIR_PIN, HIGH); // Set J6 direction to move towards the sensor
  stepperJ6.setCurrentPosition(0);
  stepperJ6.moveTo(1000000); // Move J6 towards its limit switch
  
  // Phase 1: Move both motors until their limit switches are triggered
  bool j4Homed = false;
  bool j6Homed = false;
  
  while (!j4Homed || !j6Homed) {
    if (!j4Homed && digitalRead(J4limitswitch) == HIGH) {
      stepperJ4.run();
    } else if (!j4Homed) {
      j4Homed = true;
      stepperJ4.stop();
      Serial.println("J4 limit switch triggered");
    }

    if (!j6Homed && digitalRead(J6limitswitch) == HIGH) {
      stepperJ6.run();
    } else if (!j6Homed) {
      j6Homed = true;
      stepperJ6.stop();
      Serial.println("J6 limit switch triggered");
    }
  }
  
  // Phase 2: Back off both motors
  int backOffStepsJ4 = calculateStepsJ4(backOffAngleJ4);
  int backOffStepsJ6 = calculateStepsJ6(backOffAngleJ6);
  
  stepperJ4.moveTo(stepperJ4.currentPosition() - backOffStepsJ4);
  stepperJ6.moveTo(stepperJ6.currentPosition() - backOffStepsJ6);
  
  while (stepperJ4.distanceToGo() != 0 || stepperJ6.distanceToGo() != 0) {
    stepperJ4.run();
    stepperJ6.run();
  }
  
  // Phase 3: Approach limit switches slowly
  stepperJ4.setMaxSpeed(300);
  stepperJ6.setMaxSpeed(300);
  
  stepperJ4.moveTo(stepperJ4.currentPosition() + 2 * backOffStepsJ4);
  stepperJ6.moveTo(stepperJ6.currentPosition() + 2 * backOffStepsJ6);
  
  j4Homed = j6Homed = false;
  
  while (!j4Homed || !j6Homed) {
    if (!j4Homed && digitalRead(J4limitswitch) == LOW) {
      stepperJ4.run();
    } else if (!j4Homed) {
      j4Homed = true;
      stepperJ4.stop();
      Serial.println("J4 homed accurately");
    }

    if (!j6Homed && digitalRead(J6limitswitch) == LOW) {
      stepperJ6.run();
    } else if (!j6Homed) {
      j6Homed = true;
      stepperJ6.stop();
      Serial.println("J6 homed accurately");
    }
  }
  
  // Set home positions
  stepperJ4.setCurrentPosition(calculateStepsJ4(homeAngleJ4));
  stepperJ6.setCurrentPosition(calculateStepsJ6(homeAngleJ6));
  
  // Move to target angles simultaneously
  Serial.println("Moving J4 and J6 to target angles simultaneously...");
  int stepsToMoveJ4 = calculateStepsJ4(targetAngleJ4);
  int stepsToMoveJ6 = calculateStepsJ6(targetAngleJ6);
  
  stepperJ4.setMaxSpeed(8000);
  stepperJ4.setAcceleration(8000);
  stepperJ6.setMaxSpeed(12000);
  stepperJ6.setAcceleration(8000);
  
  stepperJ4.moveTo(stepsToMoveJ4);
  stepperJ6.moveTo(stepsToMoveJ6);
  
  while (stepperJ4.distanceToGo() != 0 || stepperJ6.distanceToGo() != 0) {
    stepperJ4.run();
    stepperJ6.run();
  }
  
  Serial.println("J4 and J6 homing and movement to target angles complete.");
}

void homeAxisJ5() {
  // Move in positive direction until limit switch is pressed
  Serial.println("Homing J5 in progress...");
  stepperJ5.moveTo(1000000); // Move a large number of steps to ensure movement
  
  // Move until limit switch is triggered
  while (digitalRead(J5limitswitch) == LOW) {
    stepperJ5.run(); // Run the motor
  }
  
  // Back off by 5 degrees
  Serial.println("Limit switch triggered. Backing off by 5 degrees...");
  int backOffSteps = calculateStepsJ5(backOffAngleJ2);
  stepperJ5.moveTo(stepperJ5.currentPosition() - backOffSteps); // Move a set number of steps to back off
  while (stepperJ5.distanceToGo() != 0) {
    stepperJ5.run(); // Run the motor
  }
  
  // Move slowly to accurately trigger the switch
  Serial.println("Re-approaching limit switch slowly...");
  stepperJ5.setMaxSpeed(300); // Set a slower speed for fine adjustment
  stepperJ5.moveTo(stepperJ5.currentPosition() + 2 * backOffSteps); // Move to re-trigger the switch
  while (digitalRead(J5limitswitch) == LOW) {
    stepperJ5.run(); // Run the motor
  }
  
  // Set the current position as the home position
  Serial.println("Homing complete. Assigning home position...");
  stepperJ5.setCurrentPosition(calculateStepsJ5(homeAngleJ5)); // Set the home position to the home angle
  stepperJ5.setMaxSpeed(10000);
  stepperJ5.setAcceleration(8000);
  // Move to the target angle
  moveToAngleJ5(targetAngleJ5);
}

void moveToAngleJ2(float targetAngle) {
  Serial.print("Moving J2 to angle: ");
  Serial.println(targetAngle);
  float currentAngle = homeAngleJ2; // Assume starting position is homeAngle after homing
  int stepsToMove = calculateStepsJ2(targetAngle);
  
  stepperJ2.moveTo(stepsToMove); // Move to the target angle
  
  while (stepperJ2.distanceToGo() != 0) {
    stepperJ2.run(); // Run the motor
  }
}

void moveToAngleJ3(float targetAngle) {
  Serial.print("Moving J3 to angle: ");
  Serial.println(targetAngle);
  float currentAngle = homeAngleJ3; // Assume starting position is homeAngle after homing
  int stepsToMove = calculateStepsJ3(targetAngle);

  stepperJ3.moveTo(stepsToMove); // Move to the target angle
  
  while (stepperJ3.distanceToGo() != 0) {
    stepperJ3.run(); // Run the motor
  }
}

void moveToAngleJ4(float targetAngle) {
  Serial.print("Moving J4 to angle: ");
  Serial.println(targetAngle);
  float currentAngle = homeAngleJ4; // Assume starting position is homeAngle after homing
  int stepsToMove = calculateStepsJ4(targetAngle);
 
  stepperJ4.moveTo(stepsToMove); // Move to the target angle
  
  while (stepperJ4.distanceToGo() != 0) {
    stepperJ4.run(); // Run the motor
  }
}

void moveToAngleJ1(float targetAngle) {
  Serial.print("Moving J1 to angle: ");
  Serial.println(targetAngle);
  float currentAngle = homeAngleJ1; // Assume starting position is homeAngle after homing
  int stepsToMove = calculateStepsJ1(targetAngle);

  stepperJ1.moveTo(stepsToMove); // Move to the target angle
  
  while (stepperJ1.distanceToGo() != 0) {
    stepperJ1.run(); // Run the motor
  }
}

void moveToAngleJ6(float targetAngle) {
  Serial.print("Moving J6 to angle: ");
  Serial.println(targetAngle);
  float currentAngle = homeAngleJ6; // Assume starting position is homeAngle after homing
  int stepsToMove = calculateStepsJ6(targetAngle);
  
  stepperJ6.moveTo(stepsToMove); // Move to the target angle
  
  while (stepperJ6.distanceToGo() != 0) {
    stepperJ6.run(); // Run the motor
  }
}

void moveToAngleJ5(float targetAngle) {
  Serial.print("Moving J5 to angle: ");
  Serial.println(targetAngle);
  float currentAngle = homeAngleJ5; // Assume starting position is homeAngle after homing
  int stepsToMove = calculateStepsJ5(targetAngle);
  
  stepperJ5.setMaxSpeed(3000);
  stepperJ5.setAcceleration(3000);
  stepperJ5.moveTo(stepsToMove); // Move to the target angle
  
  while (stepperJ5.distanceToGo() != 0) {
    stepperJ5.run(); // Run the motor
  }
}

int calculateStepsJ2(float angleDifference) {
  float stepsPerRevolution = 360.0 / stepAngle * microstepping;
  float stepsPerDegree = stepsPerRevolution / 360.0;
  return angleDifference * stepsPerDegree * gearRatioJ2;
}

int calculateStepsJ3(float angleDifference) {
  float stepsPerRevolution = 360.0 / stepAngle * microstepping;
  float stepsPerDegree = stepsPerRevolution / 360.0;
  return angleDifference * stepsPerDegree * gearRatioJ3;
}

int calculateStepsJ4(float angleDifference) {
  float stepsPerRevolution = 360.0 / stepAngle * microstepping;
  float stepsPerDegree = stepsPerRevolution / 360.0;
  return angleDifference * stepsPerDegree * gearRatioJ4;
}

int calculateStepsJ1(float angleDifference) {
  float stepsPerRevolution = 360.0 / stepAngle * microstepping;
  float stepsPerDegree = stepsPerRevolution / 360.0;
  return angleDifference * stepsPerDegree * gearRatioJ1;
}

int calculateStepsJ6(float angleDifference) {
  float stepsPerRevolution = 360.0 / stepAngle * microstepping;
  float stepsPerDegree = stepsPerRevolution / 360.0;
  return angleDifference * stepsPerDegree * gearRatioJ6;
}

int calculateStepsJ5(float angleDifference) {
  float stepsPerRevolution = 360.0 / stepAngle * microstepping;
  float stepsPerDegree = stepsPerRevolution / 360.0;
  return angleDifference * stepsPerDegree * gearRatioJ5;
}
