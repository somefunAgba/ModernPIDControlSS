/*
 * File: CPLMFC.cpp
 *
 * <CLOSED PID-LOOP MODEL> <FOLLOWING CONTROL> <METHOD> : 2020
 *
 * oasomefun@futa.edu.ng. Copyright.2020
 */

#include "cplmfc.h"
/**************************************************************************/
/*!
    @brief Sets manually the hyper-parameters for the tuning algorithm
    @param Knet The PID controller instance in the loop
    @param alpha controls the proportional gain
    @param lambda_i controls the integral-error output contribution
    @param lambda_d controls the derivative-error output contribution
    @returns void.
*/
/**************************************************************************/
void cplmfc::set_alpha_critics(PIDNet& Knet, const float& alpha, const float& lambda_i, const float& lambda_d) {
    this->alpha = alpha;
    Knet.lambdai = lambda_i;
    Knet.lambdad = lambda_d;
}
/**************************************************************************/
/*!
    @brief Sets the closed-loop settling-time and bandwidth
    @param Knet The PID controller instance in the loop
    @param N_ts The discrete-time count for the settling-time of the output to be controlled
    @param N_taul The discrete-time count for the input-output delay in the loop
    @returns void.
*/
/**************************************************************************/
void cplmfc::begin(PIDNet& Knet, const int& N_ts, const int& N_taul) {
    ts = N_ts*Knet.Ts;
    tau_l = float(N_taul*Knet.Ts);
    // Natural frequency = bandwidth
    tuneWn(Knet);
}
/**************************************************************************/
/*!
    @brief Runs the tuning algorithm
    @param Knet The PID controller instance in the loop
    @param t The current time
    @returns void.
*/
/**************************************************************************/
void cplmfc::run(PIDNet& Knet, const double& t) {
    filter_r.run(Knet.ym, Knet.r);
    // Serial.print("ym: "); Serial.println(PIDobj_I.ym);
    /* CPLMFC Tuning Computation */
    tuneKp(Knet, t, 2);
    //Serial.print("Kp: ");Serial.println(PIDobj_I.Kp);
    tuneKiKd(Knet);
    //Serial.print(" Ki: ");Serial.print(PIDobj_I.Ki);
    //Serial.print(" Kd: ");Serial.println(PIDobj_I.Kd);
}

void cplmfc::tuneWn(PIDNet& Knet) {
    /*  tuning algorithm functions */
    float xtsn;
    int xx = Knet.b+Knet.c;
    if (xx==0) {
        xtsn = 9.98;
//        Serial.println(xtsn);
    }
    else if (xx==1) {
        xtsn = 7.74;
//        Serial.println(xtsn);
    }
    else if (xx==2) {
        xtsn = 11.07;
//        Serial.println(xtsn);
    }
    else {
        xtsn = 8;
    }

    /*  normalized w_n */
    wn = float(xtsn/(0.7071*ts));
    //Serial.println(xtsn);
    //printf("wn: %lf\n", wn); //debug

}

void cplmfc::tuneKp(PIDNet& Knet, const double& t, const int& mode) {
    /* n-logistic update*/
    // curve fitted rational function
    double k_sig1, k_sig2, e, xe, xu, x_lim, LL, kg, kp_lim, e_t;
    // double dkp;
    LL = (tau_l-5.936F)/(8.771F); // L is normalized by mean 5.936 and std 8.771
    kg = (0.05132*LL*LL+0.2041*LL+0.1214)/(LL*LL+1.538*LL+0.5864);
    kp_lim = alpha*kg*(tau_l+ts)/(ts);
    //Serial.print("kp_lim: "); Serial.println(kp_lim);

    // e = Knet.ym-Knet.y;
    xe = kp_lim+Knet.e;
    xu = kp_lim+Knet.u;
    nlsig(k_sig1, Knet.e, xe, -xe, kp_lim, -kp_lim, 1, 0.1, 0, 0);
    nlsig(k_sig2, Knet.u, xu, -xu, kp_lim, -kp_lim, 1, 0.1, 0, 0);
    //Serial.print("kp_not:"); Serial.println(k_sig1+k_sig2);

    x_lim = kp_lim+(k_sig1+k_sig2);
    if (mode==0) {
        nlsig(Knet.Kp, (k_sig1+k_sig2), x_lim, 0.0, kp_lim,
                0.0, (int) fmax(1.0, alpha), 0.1, 0, 0);
    }
    if (mode==1) {
        nlsig(Knet.Kp, (k_sig1+k_sig2), x_lim, 0.0,
                kp_lim, 0.0, 16, 0.1, 0, 0);
        // Serial.print("Kpfit: "); Serial.println(Knet.Kp);
    }
    if (mode==2) {
        nlsig(Knet.Kp, (k_sig1+k_sig2), x_lim, 0.0,
                kp_lim, 0.0, 1, 0.1, 0, 0);
    }
    //Serial.print("Kpfit: "); Serial.println(Knet.Kp);

    //Knet.Kp = 16;
    //adaptive option: lyapunov update rule
    if ( fabs(t) > fabs(double (tau_l)) ) {
        e_t = fabs(Knet.e_t);
        e_t = fmin((double) Knet.umax, e_t);
        // Serial.print("e_t: ");Serial.println(e_t);
        Knet.Kp = Knet.Kp+(alpha*0.001*e)*e_t;
        //Serial.print("Kplya: "); Serial.println(Knet.Kp);
    }

}

void cplmfc::tuneKiKd(PIDNet& Knet) const {
    /* integral time constant*/
    Knet.Ti = (2.0F*0.7071F)/wn;
    /* integral gain*/
    Knet.Ki = Knet.Kp/Knet.Ti;

    /* derivative time constant*/
    Knet.Td = 1.0F/(2.0F*0.7071F*wn);
    /* derivative gain*/
    Knet.Kd = Knet.Kp*Knet.Td;
}