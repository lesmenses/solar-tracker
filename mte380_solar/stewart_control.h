#include "mbed.h"
#include "Servo.h"

#define NUM_SERVOS          6       // Number of servos in system
#define SERVO_FULL_RANGE    90      // full servo range from centre
#define PI                  (double)(3.14159)

// Calculate the home position of the servos
void HomeCalc();

// Go to home position
void GoToHome(double a_posn[], Servo a_servo[]);

// Perform calibration for each of the servos
void ServoCalibration(Servo a_servo[]);

// Calculate the servo angles based on input
void AngleCalc(double x, double y, double z, double psi, double theta, double phi, double a_posn[], bool home);

// Move servos to specified positions
int GoToPosition(double a_posn[], Servo a_servo[]);

// Main control loop for stewart platform
int StewartControl(Serial* pc, Servo a_servo[]);
