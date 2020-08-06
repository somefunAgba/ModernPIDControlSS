/*
 *
 */
#include "Arduino.h"
/* PID Library Include Files */
#include "ModernPIDControlSS.h"

/* Global Declarations */
/* Tuning Parameters */
double wn[2];
float alpha[2];

/* Timing Information */
double L[2];
double ts;
double dt[2] = {0.01, 0.01};
double t = 0.0;

const double r[2] = {150.0, 150.0};
double y[2] = {0, 0};

int countseq = 0;

const int umin[2] = {-255, -255};
const int umax[2] = {255, 255};
const int dead_max[2] = {0, 100};
const int dead_min[2] = {10, 10}; // zero tolerance limits

/* Parameters PID Structure Config. */
PIDNet PIDobj_I(r[0], y[0], dt[0],
        umax[0], umin[0]);
PIDNet PIDobj_II(r[1], y[1], dt[1],
       umax[1], umin[1]);
//int loopnum = 2;

void setup() {
    // Turn on Serial Comms.
    // 9600 baud is a compromise setting between easy to kill a runaway process
    // and very slow process
    Serial.begin(9600); // Can Go Up to 2M

    /* Parameters PID Structure Config. */
    PIDobj_I.b = 1;
    PIDobj_I.c = 0;
    PIDobj_II.b = 1;
    PIDobj_II.c = 1;
    PIDobj_I.follow = 0;
    PIDobj_II.follow = 0;
    PIDobj_I.Ts = dt[0];
    PIDobj_II.Ts = dt[1];
    /* CPLMFC Tuning HyperParameter Config. */
    int Nts = 228; // 228 FOR 0.01; // this value changes if the sampling time changes
    ts = Nts*dt[0]; // settling time (including delay time)
    L[0] = 0*dt[0];
    L[1] = 0*dt[1]; // estimated transport or delay time
    /* Hyper-Parameter for Kp and Critic Weights */
    alpha[0] = 20.0; //  for P
    alpha[1] = 20.0;
    // for D
    PIDobj_I.lambdad = 0.1;
    PIDobj_II.lambdad = 0.1;
    // I
    PIDobj_I.lambdai = 0.5;
    PIDobj_II.lambdai = 0.5;
    /* PID Commonsense Model Config. */
    /* Natural Frequency for PID Commonsense */
    tuneWn(PIDobj_I, ts, wn[0]);
    //Serial.println(wn[0]);// debug
    tuneWn(PIDobj_II, ts, wn[1]);
    //Serial.println(wn[1]);// debug
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
    while (countseq >= 0) {
        t = t - tstart;
        // Serial.print("Time: ");Serial.print(t);Serial.print(" | ");Serial.println(countseq);

        if (t >=( (PIDobj_I.T_prev+PIDobj_I.Ts)-(0.5*PIDobj_I.Ts) )) {
            // tprev[0] = t;// done inside the PID

            /* CPLM Evolution */
            cplm_kernel(PIDobj_I.ym, PIDobj_I.xm, PIDobj_I.r,
                   PIDobj_I.Ts, 0, 0, wn[0]);
            // Serial.print("ym: "); Serial.println(PIDobj_I.ym);
            /* CPLMFC Tuning Computation */
            tuneKp(PIDobj_I, t, L[0], ts, alpha[0], 2);
            //Serial.print("Kp: ");Serial.println(PIDobj_I.Kp);

            tuneKi(PIDobj_I, wn[0]);
            //Serial.print(" Ki: ");Serial.print(PIDobj_I.Ki);

            tuneKd(PIDobj_I, wn[0]);
            //Serial.print(" Kd: ");Serial.println(PIDobj_I.Kd);

            /* PID Control State Evolution: Architecture */
            sfunPID_kernel(PIDobj_I, t);
            //Serial.print("upwm_I: "); Serial.println(PIDobj_I.u);
        }


        // t = t + PIDobj_I.Ts; // sysid
        // t = micros()/double(1000000);
        /* PLANT II PID CONTROL LOOP */
        if (t >=( (PIDobj_II.T_prev+PIDobj_II.Ts)-(0.5*PIDobj_II.Ts) )) {
            // tprev[1] = t; done inside the PID

            /* CPLM Evolution */
            cplm_kernel(PIDobj_II.ym, PIDobj_II.xm, PIDobj_II.r,
                   PIDobj_II.Ts, 0, 0, wn[1]);
            // Serial.print("ym: "); Serial.println(PIDobj_II.ym);
            /* CPLMFC Tuning Computation */
            tuneKp(PIDobj_II, t, L[1], ts, alpha[1], 2);
            //Serial.print("Kp: ");Serial.println(PIDobj_II.Kp);

            tuneKi(PIDobj_II, wn[1]);
            //Serial.print(" Ki: ");Serial.print(PIDobj_II.Ki);

            tuneKd(PIDobj_II, wn[1]);
            //Serial.print(" Kd: ");Serial.println(PIDobj_II.Kd);

            /* PID Control State Evolution: Architecture */
            sfunPID_kernel(PIDobj_II, t);
            //Serial.print("upwm_II: "); Serial.println(PIDobj_II.u);
        }

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
//        Serial.print((String) PIDobj_I.y + "," + PIDobj_I.ym + ","
//          + PIDobj_II.y + "," + PIDobj_II.ym + "," + countseq + "," +
//          compute_time_delay);
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
