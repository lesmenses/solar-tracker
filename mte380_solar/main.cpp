// Calibration of individual servo motors
#include "mbed.h"
#include "Servo.h"
#include "stewart_control.h"
#include "servo_debug.h"
#include "sensor_debug.h"
#include "panel_debug.h"
#include "solar_tracking.h"

int main() {
    Serial pc(SERIAL_TX, SERIAL_RX); // tx, rx
    pc.baud(9600);
     
    // Initialize servo control (servos are labelled)
    Servo servoA(D2);
    Servo servoB(D4);
    Servo servoC(D6);
    Servo servoD(D8);
    Servo servoE(D10);
    Servo servoF(D12);
    Servo a_servo[NUM_SERVOS] = {servoA, servoB, servoC, servoD, servoE, servoF};
    ServoCalibration(a_servo);
    
    // Initialize the analog inputs for photosensors
    AnalogIn sensorA(A0);   // Up
    AnalogIn sensorB(A1);   // Right
    AnalogIn sensorC(A2);   // Down
    AnalogIn sensorD(A3);   // Left
    AnalogIn a_sensor[NUM_SENSORS] = {sensorA, sensorB, sensorC, sensorD};
    
    // Initialize the analog input for solar panel output logging
    AnalogIn panel(A5);
    
     // Select program type
     while(1) {
        if(pc.readable()) {
            char c = pc.getc();
            switch(c) {
                case '1':
                    // Stewart Platform control
                    StewartControl(&pc, a_servo);
                    break;
                case '2':
                    // Solar Tracking
                    SolarTrackingControl(&pc, a_servo, a_sensor, &panel);
                    break;
                case '3':
                    // Servo Debug
                    ServoDebug(&pc, a_servo);
                    break;
                case '4':
                    // Solar Panel Debug
                    PanelDebug(&pc, &panel);
                    break;
                case '5':
                    // Sensor Debug
                    SensorDebug(&pc, a_sensor);
                    break;
                case 'q':
                    // Quit
                    goto exit;
                default:
                    // Display help menu
                    pc.printf("HELP MENU\n\r");
                    pc.printf("1 - Stewart Control Mode\n\r");
                    pc.printf("2 - Solar Tracking Mode\n\r");
                    pc.printf("3 - Servo Debug\n\r");
                    pc.printf("4 - Solar Panel Debug\n\r");
                    pc.printf("5 - Sensor Debug\n\r");
                    pc.printf("q - Quit\n\r");
                    break;
            }
        }
    }
    exit:
    printf("Donezo\n");
}