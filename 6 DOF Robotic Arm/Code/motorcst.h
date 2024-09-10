#ifndef MOTORCST_H
#define MOTORCST_H

// General constants
const int microstepping = 16; // 1/16 microstepping
const float stepAngle = 1.8; // Step angle of the motor in degrees

// Constants for J1 axis
const float gearRatioJ1 = 6.4; // Gear ratio for J1
const float homeAngleJ1 = 0; // Angle at home position for J1
const float targetAngleJ1 = 117.0; // Target angle after homing for J1
const float backOffAngleJ1 = 5.0; // Back off angle in degrees for J1

// Constants for J2 axis
const float gearRatioJ2 = 20.0; // Gear ratio for J2
const float homeAngleJ2 = 35.40812255; // Angle at home position for J2
const float targetAngleJ2 = 90.0; // Target angle after homing for J2
const float backOffAngleJ2 = 5.0; // Back off angle in degrees for J2

// Constants for J3 axis
const float gearRatioJ3 = 18.0952381; // Gear ratio for J3
const float homeAngleJ3 = 0; // Angle at home position for J3
const float targetAngleJ3 = -76; // Target angle after homing for J3
const float backOffAngleJ3 = 5.0; // Back off angle in degrees for J3

// Constants for J4 axis
const float gearRatioJ4 = 4.0; // Gear ratio for J4
const float homeAngleJ4 = 0; // Angle at home position for J4
const float targetAngleJ4 = -107; // Target angle after homing for J4
const float backOffAngleJ4 = 5.0; // Back off angle in degrees for J4

// Constants for J5 axis
const float gearRatioJ5 = 4.0; // Gear ratio for J5
const float homeAngleJ5 = 0; // Angle at home position for J5
const float targetAngleJ5 = -114; // Target angle after homing for J5
const float backOffAngleJ5 = 5.0; // Back off angle in degrees for J5

// Constants for J6 axis
const float gearRatioJ6 = 10.0; // Gear ratio for J6
const float homeAngleJ6 = 0; // Angle at home position for J6
const float targetAngleJ6 = 100; // Target angle after homing for J6
const float backOffAngleJ6 = 5.0; // Back off angle in degrees for J6

const float NORMAL_VOLTAGE = 20.0;  // Normal operating voltage
const float LOW_VOLTAGE = 10.0;     // Low voltage for holding position

#endif // MOTORCST_H