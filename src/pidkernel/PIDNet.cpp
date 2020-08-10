
/*
 * File: sfunPID_kernel.c
 * SOMEFUN'S Modern PID Controller Architecture
 * oasomefun@futa.edu.ng            : 2019, 2020
 */

/* Include Files */
#include "PIDNet.h"
#include "helpers/norm++kernel.h"

/* Instance Init */
PIDNet::PIDNet(double ref, double yout, double dt,
        int umax_lim, int umin_lim) {
    follow = 0;
    Ts = dt;
    T_prev = -dt;
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

    filter_u;
    uf = 0;
}

/* Function Definitions */
/*
 *  This function implements the 2-DOF PID control algorithm in bilinear realization.
 * Arguments    : paramsPID_T *Knet
 * Return Type  : void
 */
void PIDNet::compute(const double& t) {

    double ym, yfict, e_u, Keu, ep, cut_freq, kpi, kui, st, du;

    T_prev = t;
    /*  Discretization Scheme */
    /*  Bilinear fractional parametrization */
    /*  constrain the discretization tuner to be within the unit */
    /*  circle limits of 0 and 1. */

    /* bilinear constant */
    // kpi = 2.0/ Ts;
    cut_freq = (PI)/(10.0*Ts);
    /* first-order lpf time-constant for derivative */
    // pre-warped bilinear constant, and filter time-constant
    st = tan(PI/20.0);
    kpi = cut_freq/st;
    Tf = Ts/(2.0*st);

    /*  Inputs */
    if (follow==1) {
        ym = this->ym;
    }
    else {
        this->ym = r;
        ym = this->ym;
    }

    /*  output recalculation AWU, anti-windup */
    e_u = (u-v);
    //  This covers a decoupled PID structure instead of the
    //  error recalculation that covers a 1-DoF structure of error only.
    /*  AWUP output recalculation coefficient */
    Keu = Kp+(0.5*Ts*Ki)+((2/Ts)*Kd);
    kui = 1.5F/Ki;
    yfict = y;
    ua = e_u/Keu;
    yfict += (ua);

    /* Previous Pass*/

    /*  D */
    ud *= (kpi*Tf-1); // previous ud
    ud -= kpi*Td*(ed); // previous ed
    /*  I */
    ui += (1/(Ti*kpi))*(ei); // previous ui and ei
    ui -= kui*(upd);

    /* Current Pass */

    /*  Errors */
    ep = (b*ym)-yfict;
    e = r-y;
    // integral input recalculation
    ei = (ym-yfict)+e_u;
    ed = (c*ym)-yfict;

    /*  Individual Output Terms */
    /*  P */
    up = (ep);

    /*  D */
    ud += kpi*Td*(ed);
    ud = ud/(kpi*Tf+1);

    /* PD */
    /*  change in P, D contribution, bumpless P, D . stores current to previous */
    // error of previous PD contribution if bigger than output
    upd = (up+ud);

    /*  I */
    ui += (1/(Ti*kpi))*(ei);
    /*  integral output recalculation */
    ui -= ua;
    ui += kui*(upd);

    /*  Output Sum of Contributing Terms */
    /* Criticize*/
    e_t = (up+lambdai*(ui)+lambdad*ud);
    v = Kp*e_t;
    /* combined output recalculation */
    uf = v-ua;

    /*  Actual Control Input Constraints for u */
    //Serial.print("bef_ u="); Serial.println( u);

    // SATURATION
    // u = maxim((double)  umin, (minim( u, (double)  umax)));

    /* Hard Saturation */
    filter_u.run(u,uf); // filter
    u = fmax((double) umin,
            fmin(u, (double) umax ));

    /* Logistic Saturation*/
    // nlsig( u, du,  u, (double) umax, (double) umin,
    //        (double) umax, (double) umin,
    //        33, 6, 0, 0);
    //Serial.print("aft_ u="); Serial.println( u);

//    double u_norm[1] = {1.0};
//    double u_act[1] = { u};
//
//    //Serial.print("prior: "); Serial.println( u);
//    normalize<double>(u_act, u_norm,  umax,  umin);
//    // Serial.print("norm: "); Serial.println(u_norm[0]);
//    u_norm[0] = nlsig(u_norm[0], 1.0, -1.0,
//            1.0, -1.0,
//            33, 6, 0);
//    //Serial.print("out_norm: "); Serial.println(u_norm[0]);
//    denormalize<double>(u_norm, u_act,  umax,  umin);
//     u = u_act[0];
//    //Serial.print("after: "); Serial.println( u);

    // Serial.print("UPWM: ");Serial.println( u); // debug

    /* Misc. House Keeping */
    // increment internal sample count for the PID.
    countseq += 1;

}
void PIDNet::set_bc_follow(const int& b, const int& c, const char& follow) {
    this->b = b;
    this->c = c;
    this->follow = follow;
}




//    /* SATURATION EQUIVALENCE OF SATURATION, DEAD-ZONE AND COULOMB FRICTION */
//    // NL(.) 1-2 . DEAD-ZONE, min AND INVERSE DEAD-ZONE, max
//    if ((fabs( u) <= fabs( zerotol))) {
//        // 1. less or at dead-zone (minimum limit)
//         u = 0;
//    } else if ((fabs( u) > fabs( zerotol)) && (fabs( u) <= fabs( deadmax))) {
//        // 2. at inverse dead-zone (maximum limit)
//         u = copysign( deadmax,  u); // if u < 0, u = -deadmax
//    } else {
//        // 3. out of inverse dead-zone (max limit)
//        // added deadmax as disturbance, effect of coulomb friction in a sense.
//         u = copysign(fabs( u +  deadmax),  u);
//    }

/*
 * File trailer for sfunPID.cpp
 *
 * [EOF]
 */
