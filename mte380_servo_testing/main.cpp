// Calibration of individual servo motors
#include "mbed.h"
#include "Servo.h"
#include "main_stewart.h"
#include "main_manual_control.h"

/*int ServoControl(Serial* pc)
{
    Servo myservo(D11);

     pc->printf("Program Start\n");
     printf("HELP MENU\n");
     printf("e - emergency stop\n");
     printf("a - CCW rotation\n");
     printf("d - CW rotation\n");
     printf("p - current position\n");
     printf("m - go to middle position\n");
     myservo.calibrate(0.001150, 90);
     
      while(1) {
            // Output servo current position
            double position = myservo.read();
            
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
                    position += 0.01;
                    if(position > 1)
                        position = 1;
                    myservo.write(position);                      
                } else if (c == 'd') {
                    position -= 0.01;
                    if(position < 0.01)
                        position = 0;
                    myservo.write(position);
                } else if(c == 'p') {
                    double angle = (position-0.5)*180;
                    pc->printf("Current posn (from centre) = %f\n", angle);     
                } else if(c == 'm') {
                    position = 0.5;
                    myservo.write(position);
                } else if(c == 'h') {
                    pc->printf("HELP MENU\n");
                    pc->printf("e - emergency stop\n");
                    pc->printf("a - CCW rotation\n");
                    pc->printf("d - CW rotation\n");
                    pc->printf("p - current position\n");
                    pc->printf("m - go to middle position\n");
                } 
                else {
                    pc->printf("Invalid input\n");
                }
            }            
        }
}*/


int main() {
     Serial pc(SERIAL_TX, SERIAL_RX); // tx, rx
     pc.baud(9600);
     // Select program type
     while(1) {
        if(pc.readable()) {
            char c = pc.getc();
            switch(c) {
                case '1':
                    // Stewart Platform control
                    StewartControl(&pc);
                    break;
                case '2':
                    // Manual Control
                    //ManualControl(&pc);
                    break;
                case '3':
                    //ServoControl(&pc);
                    break;
                case '4':
                    // Quit
                    goto exit;
                default:
                    // Display help menu
                    pc.printf("HELP MENU\n");
                    pc.printf("1 - Stewart Control Mode\n\r");
                    pc.printf("2 - Manual Control Mode\n\r");
                    pc.printf("3 - Manual Single Servo\n\r");
                    pc.printf("4 - Quit\n");
            }
        }
    }
    exit:
    printf("Donezo\n");        
}