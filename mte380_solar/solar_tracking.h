#include "mbed.h"
#include "Servo.h"

#define NUM_SERVOS          6       // Number of servos in system
#define SERVO_FULL_RANGE    90      // full servo range from centre

int SolarTrackingControl(Serial* pc, Servo a_servo[], AnalogIn a_sensor[], AnalogIn* panel);