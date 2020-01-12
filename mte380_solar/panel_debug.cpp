#include "mbed.h"

// Main control loop for sensor debug
int PanelDebug(Serial* pc, AnalogIn* panel)
{
    // Info
    pc->printf("Panel Debug Start\n\r");
    
    // Select servo
    while(1){
        if(pc->readable()) {
            char c = pc->getc();
            switch(c) {
                case 'r':
                    printf("Panel output %f\n\r", panel->read()*100.0f);   
                    break;
                case 'q':
                    goto sensor_debug_exit;
                default:
                    // Display help menu
                    pc->printf("PANEL DEBUG HELP MENU\n\r");
                    pc->printf("r - Read solar panel output\n\r");
                    pc->printf("q - Quit\n\r");
                    break;
            }
        }
    }
    
    sensor_debug_exit:
    pc->printf("Panel Debug Exit\n\r");
    return 0;
}