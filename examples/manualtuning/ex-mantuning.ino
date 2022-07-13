/*
 * Manual Tuning Example
 * Configure 2 PIDs
 */
#include "Arduino.h"

/* PID Library Include Files */
#include <ModernPIDControlSS.h>

/* Global Declarations */
/* Tuning Parameters */

/* Timing Information */
double dt[2] = {0.01, 0.01};
double t = 0.0;

const double r[2] = {150.0, 150.0};
double y[2] = {0, 0};

int countseq = 0;

const int umin[2] = {-255, -255};
const int umax[2] = {255, 255};
const int dead_max[2] = {35, 35};
const int dead_min[2] = {0, 0}; // zero tolerance limits

/* Parameters PID Structure Config. */
// PID I
PIDNet PIDobj_I(r[0], y[0], dt[0],
        umax[0], umin[0], dead_max[0], dead_min[0]);
// PID II        
PIDNet PIDobj_II(r[1], y[1], dt[1],
        umax[1], umin[1], dead_max[1], dead_min[1]);

cplmfc cplmfc_I;
cplmfc cplmfc_II;

void setup() {
    // Turn on Serial Comms.
    // 9600 baud is a compromise setting between easy to kill a runaway process
    // and very slow process
    
    Serial.begin(9600); // Can Go Up to 2M

    /* optional Parameters PID Structure Config. */
    PIDobj_I.set_bc_follow(1, 0, 1);
    PIDobj_II.set_bc_follow(1, 1, 1);
}

/* Simulated Plant */
double xsI = 0;
double xsII = 0;
double tstart = 0;
double dts = 0.01;
double t_prev = -dts;
double ref_id;
double compute_time = 0.0;
char flag1 = 0;
char flag2 = 0;

void loop() {
    // Ensure Notion of fixed sampling
    //tstart = 0;
    tstart = micros()/1E6;
    while (countseq >= 0) {
        t = micros()/1E6;
        t = t-tstart;
        // Serial.print("Time: ");Serial.print(t);Serial.print(" | ");Serial.println(countseq);

        /*! PID CONTROL LOOP for PLANT I  */
        ///*!
        // @brief Starts the control loop evolution, while passing in manual tuinng params: Kp, Ki, Kd
        // @param Knet The PID controller instance in the loop
        // @param t The current time
        // @param Kp proportional gain
        // @param Ki integral gain
        // @param Kd derivative gain
        // @param flag dummy argument: set as 0
        // @returns flag character.
        //*/
        flag1 = PID_kernelOS(PIDobj_I, t, 5.0, 8.0, 1.0, 0); // Manual

        /*! PID CONTROL LOOP for PLANT II */
        // /*!
        // @brief Starts the control loop evolution, while passing in manual tuinng params: Kp, Ti, Td
        // @param Knet The PID controller instance in the loop
        // @param t The current time
        // @param Kp proportional gain
        // @param Ti integral time constant
        // @param Td derivative time constant
        // @returns flag character.
        // 
        flag2 = PID_kernelOS(PIDobj_II, t, 1.0, 0.3, 0.1);

        if (flag1 & flag2) {
            flag1 = 0;
            flag2 = 0;
             //Serial.print("upwm_I: "); Serial.println(PIDobj_I.u);
            //Serial.print("upwm_II: "); Serial.println(PIDobj_II.u);
            countseq++;
        }
        // get total time taken by control loop: to check control computation time taken
        compute_time = (micros()/1000000.0)-t;
        

        if (t>=((t_prev+dts)-(0.5*dts))) {
            /* Simulate Control Input to Plant I */
            testsys_ss(PIDobj_I.y, &xsI, dt[0], PIDobj_I.uo, 0, 0);

            /* Simulate Control Input to Plant II*/
            testsys_ss(PIDobj_II.y, &xsII, dt[1], PIDobj_II.uo, 0, 0);

            // Log Data
            Serial.println((String) PIDobj_I.y+","+PIDobj_I.ym+","
                    +PIDobj_II.y+","+PIDobj_II.ym+","+countseq+","+compute_time);

            //Serial.println((String) PIDobj_I.Kp + "," + PIDobj_I.Ti + "," + PIDobj_I.Td +
            //                    "," + PIDobj_II.Kp+ "," + PIDobj_II.Ti + "," + PIDobj_II.Td);

            t_prev = t;
        }

        if (countseq>=1E3) {
            countseq = -1;
            break;
        }

    }

}
