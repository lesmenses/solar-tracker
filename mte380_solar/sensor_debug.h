#include "mbed.h"

#define NUM_SENSORS     4

// Main control loop for sensor debug
int SensorDebug(Serial* pc, AnalogIn a_sensor[]);