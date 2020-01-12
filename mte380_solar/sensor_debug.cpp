#include "mbed.h"
#include "sensor_debug.h"

// Main control loop for sensor debug
int SensorDebug(Serial* pc, AnalogIn a_sensor[])
{
    // Info
    pc->printf("Sensor Debug Start\n\r");
    
    // Select servo
    while(1){
        if(pc->readable()) {
            char c = pc->getc();
            switch(c) {
                case 'r':
                    for(int i = 0; i < NUM_SENSORS; i++) {
                        printf("Sensor %d reading %f\n\r", i, a_sensor[i].read()*100.0f);   
                    }
                    printf("\n\r");
                    break;
                case 'q':
                    goto sensor_debug_exit;
                default:
                    // Display help menu
                    pc->printf("SENSOR DEBUG HELP MENU\n\r");
                    pc->printf("r - Read Sensor\n\r");
                    pc->printf("q - Quit\n\r");
                    break;
            }
        }
    }
    
    sensor_debug_exit:
    pc->printf("Sensor Debug Exit\n\r");
    return 0;
}