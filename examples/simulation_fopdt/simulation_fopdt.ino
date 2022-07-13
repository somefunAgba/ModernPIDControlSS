/*
 *
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
PIDNet PIDobj_I(r[0], y[0], dt[0],
        umax[0], umin[0], dead_max[0], dead_min[0]);
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

    /* CPLMFC Tuning HyperParameter Config. */
    /* Hyper-Parameter for Kp and Critic Weights */ // for P, I, D
    cplmfc_I.set_alpha_critics(PIDobj_I, 5.0);
    cplmfc_II.set_alpha_critics(PIDobj_II, 20.0);

    /* PID Commonsense Model Config. */
    // TODO: if the sampling-time is changed
    //  rerun the closed-loop settling-time identification
    //  to get the correct N_ts and N_tau_l
    // identified settling-time count
    int N_ts = 228; // 228 at 0.01secs;
    // identified delay-time count = 0, assumed
    cplmfc_I.begin(PIDobj_I, N_ts, 0);
    cplmfc_II.begin(PIDobj_II, N_ts, 0);
}

/* Simulated Plant */
double xsI = 0;
double xsII = 0;
double tstart = 0;
double dts = 0.01;
double t_prev = -dts;
double ref_id;
int sys_id = 1;
double compute_time = 0.0;
char flag = 0;

void loop() {
    // Ensure Notion of fixed sampling
    //tstart = 0;
    tstart = micros()/1E6;
    while (countseq>=0) {
        t = micros()/1E6;
        t = t-tstart;
        // Serial.print("Time: ");Serial.print(t);Serial.print(" | ");Serial.println(countseq);

        /* PLANT I PID CONTROL LOOP */
        if (sys_id==0) {
            flag = PID_kernelOS(PIDobj_I, cplmfc_I, t); // Automatic
            //flag = PID_kernelOS(PIDobj_I, t,5,8,1); // Manual
            if (flag) {
                flag = 0;
                //Serial.print("upwm_I: "); Serial.println(PIDobj_I.u);
            }
            /* PLANT II PID CONTROL LOOP */
            flag = PID_kernelOS(PIDobj_II, cplmfc_II, t);
            if (flag) {
                flag = 0;
                //Serial.print("upwm_II: "); Serial.println(PIDobj_II.u);
                countseq++;
            }
            // get total time taken by control loop: to check control computation time taken
            compute_time = (micros()/1000000.0)-t;
        }

        /* SYS ID- START */
        if (sys_id==1) {
            if (countseq==0) {
                ref_id = 1*255;
                PIDobj_I.y = 1*255;
                PIDobj_II.y = 1*255;
            }
            PIDobj_I.uo = ref_id-PIDobj_I.y; // SYS ID- END
            PIDobj_II.uo = ref_id-PIDobj_II.y; // SYS ID- END
            countseq++;
        }

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
