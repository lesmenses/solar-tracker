#include "mbed.h"
#include "rtos.h"
#include "Servo.h"
#include "solar_tracking.h"
#include "stewart_control.h"
#include "sensor_debug.h"
#include <cmath>
#include <math.h>

#define LOGGING_INTERVAL        ((uint32_t)100)    // Interval between solar panel readings
#define MOTION_INTERVAL         ((uint32_t)5000)   // Interval between solar panel adjustments
#define MOTION_WAIT             ((uint32_t)50)      // Time to wait before sensor reading during motion (ms)      // 
#define MOTION_THRESHOLD        (double)(95)        // Solar panel threshold for position adjustment
#define SENSOR_THRESHOLD        (double)(5*0.01)    // Threshold for "equal" sensor values
#define SIGNAL_MOTION           0x01                // Signal flag to indicate position adjustment
#define SENSOR_A                0                   // Index of sensor a
#define SENSOR_B                1                   // Index of sensor b
#define SENSOR_C                2                   // Index of sensor c
#define SENSOR_D                3                   // Index of sensor d
#define ANGLE_DISP              (2*PI/180)          // angle displacement on each loop

typedef struct {
    AnalogIn* a_sensor;
    Servo* a_servo;
    Serial* pc;
} tracking_input_t;

typedef struct {
    AnalogIn* panel;
    Serial* pc;
} logging_input_t;

// Voltage output of solar cell
double outputAvg = 0;

// Mutexes
Mutex outputAvg_mutex;
Mutex pc_mutex;
Mutex sensor_mutex;
Mutex panel_mutex;
Mutex sensorCalib_mutex;

// Sensor "ambient" values
double a_sensorCalib[NUM_SENSORS]= {0};
// Create thread objects
Thread* tracking_thread = NULL;
Thread* powerLog_thread = NULL;

void Tracking_Thread(void const *args)
{
    // Set priority
    
    // Arrays holding the hardware
    AnalogIn* a_sensor = ((tracking_input_t*)(args))->a_sensor;
    Servo* a_servo = ((tracking_input_t*)(args))->a_servo;
    //Serial* pc = ((tracking_input_t*)(args))->pc;
    
    // Initialization
    ServoCalibration(a_servo);
        
    // Home Calc
    HomeCalc();
    
    // Go to home position
    double currX = 0, currY = 0, currZ = 0, currTheta = 0, currPsi = 0, currPhi = 0; 
    
    // Initialize servo home
    double a_servoPosn[NUM_SERVOS] = {0};
    for(int i = 0; i < NUM_SERVOS; i++) {
        a_servoPosn[i] = 0.5;
    }
    
    while(1) {
        Thread::signal_wait(SIGNAL_MOTION);
        
        // Solar tracking
        // Adjust the pitch axis
        // Find the direction
        sensor_mutex.lock();
        double bVal = a_sensor[SENSOR_B].read() - a_sensorCalib[SENSOR_B];
        double dVal = a_sensor[SENSOR_D].read() - a_sensorCalib[SENSOR_D];
        sensor_mutex.unlock();
        // Rotate in pitch until sensors equal
        while(abs(bVal-dVal) > SENSOR_THRESHOLD) {
            double disp = 0;
            if(dVal > bVal) {
                disp = -1*ANGLE_DISP;
            } else {
                disp = ANGLE_DISP;
            }
                
            currTheta += disp;
            AngleCalc(currX, currY, currZ, currPsi, currTheta, currPhi, a_servoPosn, false);
            if(GoToPosition(a_servoPosn, a_servo) < 0) {
                //pc_mutex.lock();
                //pc->printf("Pitch adjustment failed \n\r");
                //pc_mutex.unlock();
                currTheta -= disp;
                break;
            }
            // Wait some time before sensor reading
            Thread::wait(MOTION_WAIT);     
            sensor_mutex.lock();
            bVal = a_sensor[SENSOR_B].read() - a_sensorCalib[SENSOR_B];
            dVal = a_sensor[SENSOR_D].read() - a_sensorCalib[SENSOR_D];
            sensor_mutex.unlock();
        }
        
        // Adjust the roll axis
        // Find the direction
        sensor_mutex.lock();
        double cVal = a_sensor[SENSOR_C].read() - a_sensorCalib[SENSOR_C];
        double aVal = a_sensor[SENSOR_A].read() - a_sensorCalib[SENSOR_A];
        sensor_mutex.unlock();
        // Rotate in roll until sensors equal
        while(abs(cVal - aVal) > SENSOR_THRESHOLD) {
            double disp = 0;
            if(cVal > aVal) {
                disp = ANGLE_DISP;
            } else {
                disp = ANGLE_DISP*-1;
            }
            currPhi += disp;
            AngleCalc(currX, currY, currZ, currPsi, currTheta, currPhi, a_servoPosn, false);
            if(GoToPosition(a_servoPosn, a_servo) < 0) {
                //pc_mutex.lock();
                //pc->printf("Roll adjustment failed \n\r");
                //pc_mutex.unlock();
                currPhi -= disp;
                break;
            }
            
            // Wait some time before sensor reading
            Thread::wait(MOTION_WAIT);
            sensor_mutex.lock();
            cVal = a_sensor[SENSOR_C].read() - a_sensorCalib[SENSOR_C];
            aVal = a_sensor[SENSOR_A].read() - a_sensorCalib[SENSOR_A];
            sensor_mutex.unlock();
        }
        Thread::wait(MOTION_INTERVAL);
    }
    
    
}

void PowerLogging_Thread(void const *args)
{
    double sum = 0;
    long numSamples = 0;
    AnalogIn* panel = ((logging_input_t*)(args))->panel;
    //Serial* pc = ((logging_input_t*)(args))->pc;
    while(1) {
        // Read the solar panel output (output in percentage)
        double currReading = panel->read()*100.0f;
        panel_mutex.lock();
        sum += panel->read()*100.0f;
        panel_mutex.unlock();
        numSamples++;
        outputAvg_mutex.lock();
        outputAvg = sum/numSamples;
        outputAvg_mutex.unlock();
        
        // Determine if threshold for motion is reached
        if(currReading < MOTION_THRESHOLD) {
            // Signal tracking thread or semaphore
            tracking_thread->signal_set(SIGNAL_MOTION);   
        }
        
        Thread::wait(LOGGING_INTERVAL);
    }    
}

int SolarTrackingControl(Serial* pc, Servo a_servo[], AnalogIn a_sensor[], AnalogIn* panel)
{
    // Info
    pc->printf("Solar Tracking Control start\n\r");
    
    // Thread Input args
    tracking_input_t trackingInputArgs = {a_sensor, a_servo, pc};
    logging_input_t loggingInputArgs = {panel, pc};
    
    // Control loop
    while(1) {
        if(pc->readable()) {
            char c = pc->getc();
            switch(c) {
                case '1':
                    // Start autonomous solar tracking
                    if(tracking_thread == NULL && powerLog_thread == NULL) {
                        tracking_thread = new Thread(Tracking_Thread, &trackingInputArgs, osPriorityHigh);
                        powerLog_thread = new Thread(PowerLogging_Thread, &loggingInputArgs);
                    }
                    break;
                case '2':
                    // Stop autonomous solar tracking
                    outputAvg_mutex.lock();
                    //pc_mutex.lock();
                    pc->printf("Final Avg Output: %f\n\r", outputAvg);
                    //pc_mutex.unlock();
                    outputAvg = 0;
                    outputAvg_mutex.unlock();
                    if(tracking_thread != NULL && powerLog_thread != NULL) {
                        tracking_thread->terminate();
                        powerLog_thread->terminate();
                        delete tracking_thread;
                        tracking_thread = NULL;
                        delete powerLog_thread;
                        powerLog_thread = NULL;
                    }
                    break;
                case '3':
                    outputAvg_mutex.lock();
                    //pc_mutex.lock();
                    pc->printf("Avg Output: %f\n\r", outputAvg);
                    //pc_mutex.unlock();
                    outputAvg_mutex.unlock();
                    break;
                case '4':
                    for(int i = 0; i < NUM_SENSORS; i++) {
                        sensor_mutex.lock();
                        //pc_mutex.lock();
                        printf("Sensor %d reading %f\n\r", i, a_sensor[i].read()*100.0f);
                        //pc_mutex.unlock();
                        sensor_mutex.unlock();
                    }
                    printf("\n\r");
                    break;
                case '5':
                    panel_mutex.lock();
                    //pc_mutex.lock();
                    pc->printf("Panel output %f\n\r", panel->read()*100.0f);
                    //pc_mutex.unlock();
                    panel_mutex.unlock();
                    break;
                case 6:
                    sensorCalib_mutex.lock();
                    for(int i = 0; i < NUM_SENSORS; i++) {
                        a_sensorCalib[i] = a_sensor[i].read();
                    }
                    sensorCalib_mutex.unlock();
                    break;
                case 7:
                    if(powerLog_thread == NULL) {
                        powerLog_thread = new Thread(PowerLogging_Thread, &loggingInputArgs);
                    }
                    break;
                case 8:
                    outputAvg_mutex.lock();
                    //pc_mutex.lock();
                    pc->printf("Final Avg Output: %f\n\r", outputAvg);
                    //pc_mutex.unlock();
                    outputAvg = 0;
                    outputAvg_mutex.unlock();
                    if(powerLog_thread != NULL) {
                        powerLog_thread->terminate();
                        delete powerLog_thread;
                        powerLog_thread = NULL;
                    }
                case 'q':
                    goto solar_track_exit;
                default:
                    // Display help menu
                    //pc_mutex.lock();
                    pc->printf("SOLAR TRACKING HELP MENU\n\r");
                    pc->printf("1 - Turn on autonomous solar panel control\n\r");
                    pc->printf("2 - Turn off autonomous solar panel control\n\r");
                    pc->printf("3 - Output solar average\n\r");
                    pc->printf("4 - Output sensor readings\n\r");
                    pc->printf("5 - Output panel voltage\n\r");
                    pc->printf("6 - Photosensor calibration in ambient light\n\r");
                    pc->printf("7 - Turn on power logging only\n\r");
                    pc->printf("8 - Turn off power logging only\n\r");
                    pc->printf("q - go back to main menu\n\r");
                    //pc_mutex.unlock();
                    break;
            }
        }
    }
    solar_track_exit:
    pc_mutex.lock();
    pc->printf("Solar tracking exit\n\r");
    pc_mutex.unlock();
    if(tracking_thread != NULL && powerLog_thread != NULL) {
        tracking_thread->terminate();
        powerLog_thread->terminate();
        delete tracking_thread;
        tracking_thread = NULL;
        delete powerLog_thread;
        powerLog_thread = NULL;
    }
    return 0;
}