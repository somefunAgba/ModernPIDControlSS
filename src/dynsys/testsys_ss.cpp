// APROXIMACIÓN DE PRIMER ORDEN DE UN MOTOR DC PWM->RAD/S
// Creado por oasomefun@futa.edu.ng el 16/1/2020.
//


void testsys_ss(double& y, double* x, double dt, double u, double xnoise, double ynoise) {

    double dxdt;

    /*  Muestreo de 1s */
    /* K =0.238 */
    /* T = 0.624 */

    double b0 = 1.0; //0.5F * (0.238F/0.624F); // constante sin carga -> 1.0F o carga completa -> 0.5F
    double a0 = 1.0; //1.0F/0.624F;

    double A[1] = {-a0};
    double B[1] = {1.0};
    double C[1] = {b0};

    // APROXIMACIÓN ZOH (RETENEDOR DE ORDEN CERO) DE PRIMER ORDEN
    // SALIDA


    // ESTADO
    dxdt = (*A + xnoise) * (*x) + (*B * u);
    *x += (dxdt * dt);

    y = *C * (*x) + ynoise;
}
