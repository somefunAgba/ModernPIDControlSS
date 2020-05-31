/*
 * File: cplm_kernel.c
 * oasomefun@futa.edu.ng
 * PID INTERNAL COMMON SENSE MODEL
 * C/C++ source code generated on  : 16-Jan-2020 06:00:56
 */

/* Include Files */
#include "cplm_kernel.h"
/* Function Definitions */
/*

 * Return Type  : double
 */
void cplm_kernel(double& ym, double* xm, double r, double Ts, int b, int c,
                 double wn) {

    double xmdot[2] = {0.0, 0.0};
    ym = ( xm[1] + (c*r) ); // output

    // OCF
    // normal implementation
    //xmdot[0] = (wn*wn)*(-xm[1] + (1-c)*r);
    // short-sampling modification
    xmdot[0] = xm[1] + (-wn*wn - 1)*(xm[1]) + (wn*wn*(1-c)*r);

    // normal implementation
    //xmdot[1] = xm[0] + (2*0.7071*wn)*(-xm[1] + (b-c)*r);
    // short-sampling modification
    xmdot[1] = xm[0] + xm[1] + (-2*0.7071*wn - 1)*(xm[1]) + (2*0.7071*wn*(b-c)*r);

    xm[0] = xm[0] + xmdot[0]*Ts;
    xm[1] = xm[1] + xmdot[1]*Ts;

    // debug
    // printf("r:%lf\tym\n", r, ym);
    // printf("x1d:%lf\tx2d:%lf\tx1:%lf\tx2:%lf\n", xmdot[0], xmdot[1], xm[0], xm[1]);

}

/*
 * File trailer for cplm_kernel.c
 *
 * [EOF]
 */
