// FIRST-ORDER APPROXIMATION OF A DC MOTOR PWM->RAD/S
// Created by oasomefun@futa.edu.ng on 1/16/2020.
//


void testsys_ss(double& y, double* x, double dt, double u, double xnoise, double ynoise) {

    double dxdt;

    /*  1s sampling */
    /* K =0.238 */
    /* T = 0.624 */

    double b0 = 1.0; //0.5F * (0.238F/0.624F); // no-load constant -> 1.0F or full-load -> 0.5F
    double a0 = 1.0; //1.0F/0.624F;

    double A[1] = {-a0};
    double B[1] = {1.0};
    double C[1] = {b0};

    // FIRST ORDER ZOH APPROXIMATION
    // OUTPUT


    // STATE
    dxdt = (*A + xnoise) * (*x) + (*B * u);
    *x += (dxdt * dt);

    y = *C * (*x) + ynoise;
}
