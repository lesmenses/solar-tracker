// Calibration of individual servo motors
#include "mbed.h"
#include "Servo.h"
#include "main_stewart.h"
/*
extern Servo servoA;
extern Servo servoB;
extern Servo servoC;
extern Servo servoD;
extern Servo servoE;
extern Servo servoF;
extern Servo a_servo[NUM_SERVOS];

int ManualControl(Serial* pc)
{
        int idx = 0;
        while(1) {
            // Output servo current position
            double position = a_servo[idx].read();
            
            // Get the character key pressed and move the servo
            // e - emergency stop
            // a - CCW rotation
            // d - CW rotation
            // p - output current position
            // m - go to middle position
            if(pc->readable()) {
                char c = pc->getc();
                if(c == 'e') {
                    break;
                } else if (c == 'a') {
                    position += 0.05;
                    if(position > 1)
                        position = 1;
                    a_servo[idx].write(position);                      
                } else if (c == 'd') {
                    position -= 0.05;
                    if(position < 0.05)
                        position = 0;
                    a_servo[idx].write(position);
                } else if(c == 'p') {
                    double angle = (position-0.5)*180;
                    pc->printf("Current posn (from centre) = %f, idx = %d\n", angle, idx);     
                } else if(c == 'm') {
                    position = 0.5;
                    a_servo[idx].write(position);
                } else if(c >= '0' && c <= '5') {
                    idx = c - '0';
                } else {
                    pc->printf("Invalid input\n");
                    pc->printf("HELP MENU\n");
                    pc->printf("e - emergency stop\n");
                    pc->printf("a - CCW rotation\n");
                    pc->printf("d - CW rotation\n");
                    pc->printf("p - current position and idx\n");
                    pc->printf("m - go to middle position\n");
                    pc->printf("(1,5) - switch servo being controlled\n");
                }
            }            
        }
        
        return 0;
}*/