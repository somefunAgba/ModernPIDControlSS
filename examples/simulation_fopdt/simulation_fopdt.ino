/*
 *
 */
#include "Arduino.h"

/* PID Library Include Files */
#include "ModernPIDControlSS.h"

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
const int dead_max[2] = {0, 0};
const int dead_min[2] = {0, 0}; // zero tolerance limits

/* Parameters PID Structure Config. */
PIDNet PIDobj_I(r[0], y[0], dt[0],
        umax[0], umin[0]);
PIDNet PIDobj_II(r[1], y[1], dt[1],
       umax[1], umin[1]);

cplmfc cplmfc_I;
cplmfc cplmfc_II;



void setup() {
    // Turn on Serial Comms.
    // 9600 baud is a compromise setting between easy to kill a runaway process
    // and very slow process
    Serial.begin(9600); // Can Go Up to 2M

    /* Parameters PID Structure Config. */
    PIDobj_I.set_bc_follow(0,0,0);
    PIDobj_II.set_bc_follow(1,1,1);

    /* CPLMFC Tuning HyperParameter Config. */
    /* Hyper-Parameter for Kp and Critic Weights */ // for P, I, D
    cplmfc_I.set_alpha_critics(PIDobj_I,5.0,0.5,0.1);
    cplmfc_II.set_alpha_critics(PIDobj_II,5.0,0.5,0.1);

    /* PID Commonsense Model Config. */
    // TODO: if the sampling-time is changed
    //  rerun the closed-loop settling-time identification
    //  to get the correct N_ts and N_tau_l
    // identified settling-time count
    int N_ts = 228; // 228 at 0.01secs;
    // identified delay-time count
    int N_tau_l[2] = {0, 0};
    cplmfc_I.begin(PIDobj_I, N_ts, N_tau_l[0]);
    cplmfc_II.begin(PIDobj_II, N_ts, N_tau_l[1]);
}

/* Simulated Plant */
double xsI = 0;
double xsII = 0;
double uin[2];
double tstart = 0;
double ref_id;
int sys_id = 0;
double compute_time_delay = 0.0;
//
void loop() {
 // Ensure Notion of fixed sampling
    //tstart = 0;
    tstart = micros()/1000000.0;
    t = tstart;
    while (countseq >= 0) {
        t = t - tstart;
        // Serial.print("Time: ");Serial.print(t);Serial.print(" | ");Serial.println(countseq);

        /* PLANT I PID CONTROL LOOP */
        if (t >=( (PIDobj_I.T_prev+PIDobj_I.Ts)-(0.5*PIDobj_I.Ts) )) {
            // tprev[0] = t;// done inside the PID
            PID_kernelOS(PIDobj_I, cplmfc_I, t);
            //Serial.print("upwm_I: "); Serial.println(PIDobj_I.u);
        }

        // t = t + PIDobj_I.Ts; // sysid
        // t = micros()/double(1000000);

        /* PLANT II PID CONTROL LOOP */
        if (t >=( (PIDobj_II.T_prev+PIDobj_II.Ts)-(0.5*PIDobj_II.Ts) )) {
            // tprev[1] = t; done inside the PID
            PID_kernelOS(PIDobj_II, cplmfc_II, t);
            //Serial.print("upwm_II: "); Serial.println(PIDobj_II.u);
        }

        // get total time taken by control loop: to check control computation time taken
        compute_time_delay = (micros()/1000000.0) - t;

        /* SYS ID- START */
        if (sys_id == 1) {
            if (countseq==0) {
                ref_id = 1*255;
                PIDobj_I.y = 1*255;
                PIDobj_II.y = 1*255;
            }
            PIDobj_I.u = ref_id-PIDobj_I.y; // SYS ID- END
            PIDobj_II.u = ref_id-PIDobj_II.y; // SYS ID- END
        }

        /* Simulate Control Input to Plant I */
        // dead-zone disturbance
        uin[0] = dead_zone<double>(PIDobj_I.u, dead_max[0], dead_min[0]);
        testsys_ss(PIDobj_I.y, &xsI, dt[0], uin[0], 0, 0);
        //Serial.println(PIDobj_I.y);

        /* Simulate Control Input to Plant II*/
        // dead-zone disturbance
        uin[1] = dead_zone<double>(PIDobj_II.u, dead_max[1], dead_min[1]);
        testsys_ss(PIDobj_II.y, &xsII, dt[1], uin[1], 0, 0);
        //Serial.println(PIDobj_II.y);

        //Serial.print(F("Y: "));
        Serial.print((String) PIDobj_I.y + "," + PIDobj_I.ym + ","
          + PIDobj_II.y + "," + PIDobj_II.ym + "," + countseq + "," + compute_time_delay);
        //Serial.print((String) PIDobj_I.Kp + "," + PIDobj_II.Kp);
        //Serial.print((String) PIDobj_I.Ki + "," + PIDobj_II.Ki);
        //Serial.print((String) PIDobj_I.Kd + "," + PIDobj_II.Kd);
        Serial.println();
        //Serial.println(F("---\n"));

        countseq++;
        if (countseq >= 10000) {
            countseq = -1;
            break;
        }
        // t = t + PIDobj_I.Ts; // sysid
        t = micros()/1000000.0;
    }

}
