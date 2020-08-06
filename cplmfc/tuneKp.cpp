//
// Created by SomefunAgba on 5/28/2020.
//
#include "cpidlmfc.h"
#include "nlsig/nlsig.h"


void tuneKp(PIDNet& Knet, double t, double L, double ts, double alpha, unsigned int mode) {

/* n-logistic update*/
    // curve fitted rational function
    double ksig1, ksig2, e, xe, xu, xlim, LL, kg, kplim, e_t, dkp;
    LL = (L-5.936F)/(8.771F); // L is normalized by mean 5.936 and std 8.771
    kg = (0.05132*LL*LL+0.2041*LL+0.1214)/(LL*LL+1.538*LL+0.5864);
    kplim = alpha*kg*(L+ts)/(ts);
    //Serial.print("kplim: "); Serial.println(kplim);

    e = Knet.ym-Knet.y;
    xe = kplim+e;
    xu = kplim+Knet.u;
    nlsig(ksig1, dkp, Knet.e, xe, -xe, kplim, -kplim, 1, 0.1, 0, 0);
    nlsig(ksig2, dkp, Knet.u, xu, -xu, kplim, -kplim, 1, 0.1, 0, 0);
    //Serial.print("kp_not:"); Serial.println(ksig1+ksig2);

    xlim = kplim+(ksig1+ksig2);
    if (mode==0) {
       nlsig(Knet.Kp, dkp, (ksig1+ksig2), xlim, 0.0, kplim,
                0.0, (int) fmax(1.0, alpha), 0.1, 0, 0);
    }
    if (mode==1) {
        nlsig(Knet.Kp, dkp, (ksig1+ksig2), xlim, 0.0,
                kplim, 0.0, 16, 0.1, 0, 0);
        // Serial.print("Kpfit: "); Serial.println(Knet.Kp);
    }
    if (mode==2) {
       nlsig( Knet.Kp, dkp, (ksig1+ksig2), xlim, 0.0,
                kplim, 0.0, 1, 0.1, 0, 0);
    }
    //Serial.print("Kpfit: "); Serial.println(Knet.Kp);

    //Knet.Kp = 16;
    //adaptive option: lyapunov update rule
    if (fabs(t)>fabs(L)) {
        e_t = fabs(Knet.e_t);
        e_t = fmin((double) Knet.umax, e_t);
        // Serial.print("e_t: ");Serial.println(e_t);
        Knet.Kp = Knet.Kp+(alpha*0.001*e)*e_t;
        //Serial.print("Kplya: "); Serial.println(Knet.Kp);
    }

}