#include "mbed.h"
#include "Servo.h"
#include "stewart_control.h"

// Menu to control individual servo
int ControlServo(Serial* pc, Servo* servo)
{
    // Info
    pc->printf("Servo Control start\n\r");
    
    // Control loop
    while(1) {
        double position = servo->read();
        if(pc->readable()) {
            char c = pc->getc();
            switch(c) {
                case 'a':
                    position += 0.05;
                    if(position > 1)
                        position = 1;
                    servo->write(position);
                    break;
                case 'd':
                    position -= 0.05;
                    if(position < 0.05)
                        position = 0;
                    servo->write(position);
                    break;
                case 'p':
                    printf("Servo at posn %f\n\r", servo->read());
                    break;
                case 'm':
                    position = 0.5;
                    servo->write(position);
                    break;
                case 'q':
                    goto servo_ctrl_exit;
                default:
                    // Display help menu
                    pc->printf("HELP MENU\n\r");
                    pc->printf("a - CCW rotation\n\r");
                    pc->printf("d - CW rotation\n\r");
                    pc->printf("p - current position and idx\n\r");
                    pc->printf("m - go to middle position\n\r");
                    pc->printf("q - go back to servo selection\n\r");
                    break;
            }
        }
    }
    
    servo_ctrl_exit:
    pc->printf("Servo Control exit\n\r");
    return 0;
}

// Main control loop for servo debug
int ServoDebug(Serial* pc, Servo a_servo[])
{
    // Info
    pc->printf("Servo Debug start\n\r");
    
    // Initialization
    ServoCalibration(a_servo);
    int servo_index = 1;
    // Select servo
    while(1){
        if(pc->readable()) {
            char c = pc->getc();
            switch(c) {
                case '1':
                case '2':
                case '3':
                case '4':
                case '5':
                case '6':
                    servo_index = c - '0';
                     pc->printf("Servo %d chosen\n\r", servo_index);
                    ControlServo(pc, &a_servo[servo_index-1]);
                    break;
                case 'q':
                    goto servo_select_exit;
                default:
                    // Display help menu
                    pc->printf("SERVO DEBUG HELP MENU\n\r");
                    pc->printf("Select Servo to Control (1-6)\n\r");
                    pc->printf("q - Quit\n\r");
                    break;
                
            }
        }
    }
    
    servo_select_exit:
    pc->printf("Servo Debug exit\n\r");
    return 0; 
}