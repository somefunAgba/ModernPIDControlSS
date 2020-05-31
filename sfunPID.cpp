
/*
 * File: sfunPID_kernel.c
 * SOMEFUN'S Modern PID Controller Architecture
 * oasomefun@futa.edu.ng            : 2019, 2020
 */

/* Include Files */
#include "sfunPID.h"
#include "nlsig.h"

/* Constructor Init */
PIDNet::PIDNet(double ref, double yout, double Tsample,
        int umax_lim, int umin_lim) {
    follow = 0;
    Ts = Tsample;
    T_prev = -Tsample;
    countseq = 0;

    r = ref;
    y = yout;
    ym = 0;

    e = 0;
    ei = 0;
    ed = 0;

    up = 0;
    ui = 0;
    ud = 0;
    upd = 0;
    ua = 0;
    v = 0;
    u = 0;
    e_t = 0;

    umax = umax_lim;
    umin = umin_lim;

    Kp = 0.001;
    Ki = 0.0;
    Kd = 0.0;
    lambdai = 1;
    lambdad = 1;
    Ti = 100;
    Td = 0;
    Tf = 0.5;
    b = 1;
    c = 0;

}

/* Function Definitions */
/*
 *  This function implements the 2-DOF PID control algorithm in bilinear realization.
 * Arguments    : paramsPID_T *Knet
 * Return Type  : void
 */
void sfunPID_kernel(PIDNet& Knet, const double& t) {

    double ym, yfict, e_u, Keu, ep, cut_freq, kpi, kui;

    Knet.T_prev = t;
    /*  Discretization Scheme */
    /*  Bilinear fractional parametrization */
    /*  constrain the discretization tuner to be within the unit */
    /*  circle limits of 0 and 1. */

    /* bilinear constant */
    // kpi = 2.0/Knet.Ts;
    cut_freq = (2*PI)/(10.0*2.0*Knet.Ts);
    kpi = cut_freq;
    /* first-order lpf time-constant for derivative */
    cut_freq = (PI)/(10.0*2.0);
    // pre-warp
    kpi = kpi/tan(cut_freq);
    Knet.Tf = Knet.Ts/(2.0F*tan(cut_freq));

    /*  Inputs */
    if (Knet.follow==1) {
        ym = Knet.ym;
    }
    else {
        ym = Knet.r;
        Knet.ym = ym;
    }

    /*  output recalculation AWU, anti-windup */
    e_u = (Knet.u-Knet.v);
    //  This covers a decoupled PID structure instead of the
    //  error recalculation that covers a 1-DoF structure of error only.
    /*  AWUP output recalculation coefficient */
    Keu = Knet.Kp + (0.5*Knet.Ts*Knet.Ki)+ ((2/Knet.Ts)*Knet.Kd);
    kui = 1.5F/Knet.Ki;
    yfict = Knet.y;
    Knet.ua = e_u/Keu;
    yfict += (Knet.ua);

    /* Previous Pass*/

    /*  D */
    Knet.ud *= (kpi*Knet.Tf-1); // previous ud
    Knet.ud -= kpi*Knet.Td*(Knet.ed); // previous ed
    /*  I */
    Knet.ui += (1/(Knet.Ti*kpi))*(Knet.ei); // previous ui and ei
    Knet.ui -= kui*(Knet.upd);

    /* Current Pass */

    /*  Errors */
    ep = (Knet.b*ym)-yfict;
    Knet.e = Knet.r-Knet.y;
    // integral input recalculation
    Knet.ei = (ym-yfict)+e_u;
    Knet.ed = (Knet.c*ym)-yfict;

    /*  Individual Output Terms */
    /*  P */
    Knet.up = (ep);

    /*  D */
    Knet.ud += kpi*Knet.Td*(Knet.ed);
    Knet.ud = Knet.ud/(kpi*Knet.Tf+1);

    /* PD */
    /*  change in P, D contribution, bumpless P, D . stores current to previous */
    // error of previous PD contribution if bigger than output
    Knet.upd = (Knet.up+Knet.ud);

    /*  I */
    Knet.ui += (1/(Knet.Ti*kpi))*(Knet.ei);
    /*  integral output recalculation */
    Knet.ui -= Knet.ua;
    Knet.ui += kui*(Knet.upd);

    /*  Output Sum of Contributing Terms */
    /* Criticize*/
    Knet.e_t = (Knet.up + Knet.lambdai*(Knet.ui)+ Knet.lambdad*Knet.ud);
    Knet.v = Knet.Kp * Knet.e_t;
    /* combined output recalculation */
    Knet.u = Knet.v - Knet.ua;

//    /*  Actual Control Input Constraints for u */
//    /* SATURATION EQUIVALENCE OF SATURATION, DEAD-ZONE AND COULOMB FRICTION */
//    // NL(.) 1-2 . DEAD-ZONE, min AND INVERSE DEAD-ZONE, max
//    if ((fabs(Knet.u) <= fabs(Knet.zerotol))) {
//        // 1. less or at dead-zone (minimum limit)
//        Knet.u = 0;
//    } else if ((fabs(Knet.u) > fabs(Knet.zerotol)) && (fabs(Knet.u) <= fabs(Knet.deadmax))) {
//        // 2. at inverse dead-zone (maximum limit)
//        Knet.u = copysign(Knet.deadmax, Knet.u); // if u < 0, u = -deadmax
//    } else {
//        // 3. out of inverse dead-zone (max limit)
//        // added deadmax as disturbance, effect of coulomb friction in a sense.
//        Knet.u = copysign(fabs(Knet.u + Knet.deadmax), Knet.u);
//    }
    // NL(.)3 SATURATION
    //Knet.u = maxim((double) Knet.umin, (minim(Knet.u, (double) Knet.umax)));
    Knet.u = fmax((double) Knet.umin,
            fmin(Knet.u, (double) Knet.umax
            ));
/* Logistic Saturation*/
//    Knet.u = nlsig(Knet.v, (double) Knet.umax, (double) Knet.umin,
//            (double) Knet.umax, (double) Knet.umin,
//            33, 24.0/(33+2.0), 0);


    /* Floating Point Error Prevention */
    if (fabs(Knet.u) < 0.000001) {
        Knet.u = 0;
    }
    // Serial.print("UPWM: ");Serial.println(Knet.u); // debug

    /* Misc. House Keeping */
    // increment internal sample count for the PID.
    Knet.countseq += 1;

}

/*
 * File trailer for sfunPID.cpp
 *
 * [EOF]
 */
