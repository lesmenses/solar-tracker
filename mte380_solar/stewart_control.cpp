// Control of the Stewart Platform
#include "mbed.h"
#include "Servo.h"
#include "stewart_control.h"
#include <cmath>
#include <math.h>


#define ANGLE_DISP_FINE      (2*PI/180)       // angle displacement on each loop
#define ANGLE_DISP_COARSE      (6*PI/180)       // angle displacement on each loop


// Servo positions to go to home position
double a_servoHome[NUM_SERVOS] = {0, 0, 0, 0, 0, 0};

// Hard-coded Servo Positions
/*
const double a_rightPosn[NUM_SERVOS] = {0, 0, 0, 0, 0, 0};
const double a_leftPosn[NUM_SERVOS] = {0, 0, 0, 0, 0, 0};
const double a_upPosn[NUM_SERVOS] = {0, 0, 0, 0, 0, 0};
const double a_downPosn[NUM_SERVOS] = {0, 0, 0, 0, 0, 0};
*/

// Build constants for the angle calculations
typedef struct {
    double x;
    double y;
    double z;
} vector_t;


const vector_t a_b[NUM_SERVOS] = {{0,-71.25,0},{61.70,-35.625,0},{61.70,35.625,0},{0,71.25,0},{-61.70,35.625,0},{-61.70,-35.625,0}};
const vector_t a_p[NUM_SERVOS] = {{0,-30,-9.5},{25.98,-15,-9.5},{25.98,15,-9.5},{0,30,-9.5},{-25.98,15,-9.5},{-25.98,-15,-9.5}};
const double a_beta[NUM_SERVOS] = {3*PI/2,11*PI/6,PI/6,PI/2,5*PI/6,7*PI/6};
const double s = 145;
const double a = 24.78;
void AngleCalc(double x, double y, double z, double psi, double theta, double phi, double a_posn[], bool home);

// Calculate the home position of the servos
void HomeCalc()
{
     AngleCalc(0,0,0,0,0,0,a_servoHome, true);
     for(int i = 0; i < NUM_SERVOS; i++) {
        printf("home posn %d %f \n\r", i, a_servoHome[i]);
    }
}

// Go to home position
void GoToHome(double a_posn[], Servo a_servo[]) {
    for(int i = 0; i < NUM_SERVOS; i++) {
        a_servo[i].write(0.5);
        a_posn[i] = 0.5;
    }   
}

// Perform calibration for each of the servos
void ServoCalibration(Servo a_servo[])
{
    double a_calib[NUM_SERVOS] = {0.000975, 0.00095, 0.000825, 0.000925, 0.000975, 0.00115};
    for(int i = 0; i < NUM_SERVOS; i++) {
        a_servo[i].calibrate(a_calib[i], SERVO_FULL_RANGE);
    }
}

// Calculate the servo angles based on input
void AngleCalc(double x, double y, double z, double psi, double theta, double phi, double a_posn[], bool home)
{    
    z += 146;
    // Calculate rotation matrix with pitch, roll, yaw
    double a_rotMatrix[3][3];
    a_rotMatrix[0][0] = cos(psi)*cos(theta);
    a_rotMatrix[0][1] = -1*sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi);
    a_rotMatrix[0][2] = sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
    a_rotMatrix[1][0] = sin(psi)*cos(theta);
    a_rotMatrix[1][1] = cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi);
    a_rotMatrix[1][2] = -1*cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
    a_rotMatrix[2][0] = -1*sin(theta);
    a_rotMatrix[2][1] = cos(theta)*sin(phi);
    a_rotMatrix[2][2] = cos(theta)*cos(phi);

    //for(int i = 0; i < 3; i++) {
    //  for(int j = 0; j < 3; j++) {
        //cout << a_rotMatrix[i][j] << endl;
      //}
  //  }
    
    // Calculate servo angle to achieve desired position for each servo
    for(int i = 0; i < NUM_SERVOS; i++) { 
        // Calculate q vector
        vector_t q = {0,0,0};
        q.x = a_rotMatrix[0][0]*a_p[i].x + a_rotMatrix[0][1]*a_p[i].y + a_rotMatrix[0][2]*a_p[i].z + x;
        q.y = a_rotMatrix[1][0]*a_p[i].x + a_rotMatrix[1][1]*a_p[i].y + a_rotMatrix[1][2]*a_p[i].z + y;
        q.z = a_rotMatrix[2][0]*a_p[i].x + a_rotMatrix[2][1]*a_p[i].y + a_rotMatrix[2][2]*a_p[i].z + z;

        //cout << "q(" << q.x << "," << q.y << "," << q.z << ")\n";
        
        // Calculate the leg length vector; and its magnitude
        vector_t length = {q.x-(a_b[i]).x, q.y-(a_b[i]).y, q.z-(a_b[i]).z};        
        double lengthMag = sqrt((length.x)*(length.x) + (length.y)*(length.y) + (length.z)*(length.z));
        //cout << "length mag" << lengthMag << "\n";
        
        // Intermediate values calculation
        double l = lengthMag*lengthMag - (s*s - a*a);
        double m = 2*a*(q.z - a_b[i].z);
        double n = 2*a*(cos(a_beta[i])*(q.x-a_b[i].x) + sin(a_beta[i])*(q.y-a_b[i].y));

        //cout << "l: " << l << "\n";
        //cout << "m: " << m << "\n";
        //cout << "n: " << n << "\n";
        
        // Calculate final angle
        double alpha = (asin(l/sqrt(m*m + n*n)) - atan(n/m));
        
        // If odd motor angle use (pi-angle)
        if((i+1)%2 == 0) {
            a_posn[i] = alpha;
            //cout << "Angle " << i << ": " << a_posn[i] << "\n";
        } else {
            a_posn[i] = PI-alpha;
            //cout << "Angle " << i << ": " << a_posn[i] << "\n";
        }
        
        if(!home) {
            // Change to percent of full range
            double ang = a_posn[i];
            a_posn[i] = (a_posn[i] - (a_servoHome[i] - (PI/2)))/((a_servoHome[i]+(PI/2)) - (a_servoHome[i]-(PI/2)));
            //printf("Servo %d angle %f set %f\n\r", i, ang, a_posn[i]);
        }
    }    
    
    return;
}

// Move servos to specified positions
int GoToPosition(double a_posn[], Servo a_servo[])
{
    // Check limits
    for(int i = 0; i < NUM_SERVOS; i++) {
        double currPosn = a_servo[i].read();
        //printf("Servo %d posn %f new_posn %f \n\r", i, currPosn, a_posn[i]);
        if(abs(a_posn[i]-currPosn) > 0.2) {
            printf("Too drastic a move \n\r");
            return -1;
        } else if(a_posn[i] < 0.1) {
            printf("Servo posn request out-of-range low \n\r");
            a_posn[i] = 0.1;
            return -1;
        } else if (a_posn[i] > 0.9) {
            printf("Servo posn request out-of-range high \n\r");
            a_posn[i] = 0.9;
            return -1;   
        } else if (isnan(a_posn[i])) {
            printf("Servo posn is nan\n\r");
            return -1;
        }
    }
    for(int i = 0; i < NUM_SERVOS; i++) {
        a_servo[i].write(a_posn[i]);
        a_posn[i] = a_servo[i].read();
    }
    return 0;
    
}

// Main control loop for stewart platform
int StewartControl(Serial* pc, Servo a_servo[])
{
    // Help Menu Info
    pc->printf("Stewart Control Program Start\n\r");
    
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
    //bool reset = false;
    
    // Control Loop
    while(1) {
        // Get the next movement key
        if(pc->readable()) {
            char c = pc->getc();
            // Latency timer
            Timer t;
            switch(c) {
                case 'a':
                    // -ve pitch
                    currTheta -= ANGLE_DISP_FINE;
                    t.start();
                    AngleCalc(currX, currY, currZ, currPsi, currTheta, currPhi, a_servoPosn, false);
                    t.stop();
                    //printf("Latency=%f\n\r", t.read());
                    if(GoToPosition(a_servoPosn, a_servo) < 0) {
                        pc->printf("-ve pitch movement failed \n\r");
                        currTheta += ANGLE_DISP_FINE;   
                    }
                    break;
                case 'd':
                    // +ve pitch
                    currTheta += ANGLE_DISP_FINE;
                    t.start();
                    AngleCalc(currX, currY, currZ, currPsi, currTheta, currPhi, a_servoPosn, false);
                    t.stop();
                    //printf("Latency=%f\n\r", t.read());
                    if(GoToPosition(a_servoPosn, a_servo) < 0) {
                        pc->printf("+ve pitch movement failed \n\r");
                        currTheta -= ANGLE_DISP_FINE;       
                    }
                    break;
                case 'w':
                    // -ve roll
                    currPhi -= ANGLE_DISP_FINE;
                    t.start();
                    AngleCalc(currX, currY, currZ, currPsi, currTheta, currPhi, a_servoPosn, false);
                    t.stop();
                    //printf("Latency=%f\n\r", t.read());
                    if(GoToPosition(a_servoPosn, a_servo) < 0) {
                        pc->printf("-ve roll movement failed \n\r");
                        currPhi += ANGLE_DISP_FINE;
                    }
                    break;
                case 's':
                    // +ve roll
                    currPhi += ANGLE_DISP_FINE;
                    t.start();
                    AngleCalc(currX, currY, currZ, currPsi, currTheta, currPhi, a_servoPosn, false);
                    t.stop();
                    //printf("Latency=%f\n\r", t.read());
                    if(GoToPosition(a_servoPosn, a_servo) < 0) {
                        pc->printf("+ve roll movement failed\n\r");
                        currPhi -= ANGLE_DISP_FINE;    
                    }
                    break;
                case 'j':
                    // -ve pitch
                    currTheta -= ANGLE_DISP_COARSE;
                    t.start();
                    AngleCalc(currX, currY, currZ, currPsi, currTheta, currPhi, a_servoPosn, false);
                    t.stop();
                    //printf("Latency=%f\n\r", t.read());
                    if(GoToPosition(a_servoPosn, a_servo) < 0) {
                        pc->printf("-ve pitch movement failed \n\r");
                        currTheta += ANGLE_DISP_COARSE;   
                    }
                    break;
                case 'l':
                    // +ve pitch
                    currTheta += ANGLE_DISP_COARSE;
                    t.start();
                    AngleCalc(currX, currY, currZ, currPsi, currTheta, currPhi, a_servoPosn, false);
                    t.stop();
                    //printf("Latency=%f\n\r", t.read());
                    if(GoToPosition(a_servoPosn, a_servo) < 0) {
                        pc->printf("+ve pitch movement failed \n\r");
                        currTheta -= ANGLE_DISP_COARSE;       
                    }
                    break;
                case 'i':
                    // +ve roll
                    currPhi -= ANGLE_DISP_COARSE;
                    t.start();
                    AngleCalc(currX, currY, currZ, currPsi, currTheta, currPhi, a_servoPosn, false);
                    t.stop();
                    //printf("Latency=%f\n\r", t.read());
                    if(GoToPosition(a_servoPosn, a_servo) < 0) {
                        pc->printf("-ve roll movement failed \n\r");
                        currPhi += ANGLE_DISP_COARSE;
                    }
                    break;
                case 'k':
                    // -ve roll
                    currPhi += ANGLE_DISP_COARSE;
                    t.start();
                    AngleCalc(currX, currY, currZ, currPsi, currTheta, currPhi, a_servoPosn, false);
                    t.stop();
                    //printf("Latency=%f\n\r", t.read());
                    if(GoToPosition(a_servoPosn, a_servo) < 0) {
                        pc->printf("+ve roll movement failed\n\r");
                        currPhi -= ANGLE_DISP_COARSE;    
                    }
                    break;
                case 'm':
                    // Go to home position
                    GoToHome(a_servoPosn, a_servo);
                    currTheta = 0;
                    currPhi = 0;
                    currPsi = 0;
                    currX = 0;
                    currY = 0;
                    currZ = 0;
                    break;
                case 'p':
                    for(int j = 0; j < NUM_SERVOS; j++) {
                        printf("Servo %d at posn %f\n\r", j, a_servo[j].read());
                    }
                    break;
                case 'q':
                    // Quit
                    t.stop();
                    goto done;
                default:
                    // Display help menu
                    pc->printf("HELP MENU\n\r");
                    pc->printf("w - +ve roll small\n\r");
                    pc->printf("s - -ve roll small\n\r");
                    pc->printf("d - +ve pitch small\n\r");
                    pc->printf("a - -ve pitch small\n\r");
                    pc->printf("i - +ve roll big\n\r");
                    pc->printf("k - -ve roll big\n\r");
                    pc->printf("l - +ve pitch big\n\r");
                    pc->printf("j - -ve pitch big\n\r");
                    pc->printf("m - go to middle position\n\r");
                    pc->printf("p - display servo positions\n\r");
                    pc->printf("q - end program\n\r");
                    break;
            }
        }        
    }
    
    done:
    pc->printf("Stewart Control done\n");
    return 0;
}