/*
 * Archivo: cplm_kernel.c
 * oasomefun@futa.edu.ng
 * MODELO INTERNO DE SENTIDO COMÚN PID
 * Código fuente C/C++ generado el  : 16-Ene-2020 06:00:56
 */

/* Archivos de Inclusión */
#include "filterFO_pass.h"

/* Definiciones de Funciones */
/*
 * Tipo de Retorno  : double
 */

void cplm_kernel(double& ym, double& xm, double rin) {

    //double st = tan(PI/20.0); // Código comentado
    double Tf_kpi = PI/(20.0*TAN_ST*TAN_ST);

    xm = xm + ((Tf_kpi-1)*ym);
    ym = (xm+rin)/(Tf_kpi+1);
    xm = rin;
}


//void cplm_kernel(double& ym, double* xm, double r, double Ts, int b, int c,
//                 double wn) {


//    double xmdot[2] = {0.0, 0.0};
//    ym = ( xm[1] + (c*r) ); // salida

    // OCF
    // implementación normal
    //xmdot[0] = (wn*wn)*(-xm[1] + (1-c)*r);
    // modificación para muestreo corto

//    xmdot[0] = xm[1] + (-wn*wn - 1)*(xm[1]) + (wn*wn*(1-c)*r);

    // implementación normal
    //xmdot[1] = xm[0] + (2*0.7071*wn)*(-xm[1] + (b-c)*r);
    // modificación para muestreo corto
//
//    xmdot[1] = xm[0] + xm[1] + (-2*0.7071*wn - 1)*(xm[1]) + (2*0.7071*wn*(b-c)*r);
//
//    xm[0] = xm[0] + xmdot[0]*Ts;
//    xm[1] = xm[1] + xmdot[1]*Ts;

    // depuración
    // printf("r:%lf\tym\n", r, ym);
    // printf("x1d:%lf\tx2d:%lf\tx1:%lf\tx2:%lf\n", xmdot[0], xmdot[1], xm[0], xm[1]);
//}


void filterFO_pass::run(double& y, double u) {
    x = x + ((Tf_kpi-1)*y);
    y = (x+u)/(Tf_kpi+1);
    x = u;
}

filterFO_pass::filterFO_pass() {
    //double st = tan(PI/20.0); // Código comentado
    Tf_kpi = PI/(20.0*TAN_ST*TAN_ST);
    x = 0;
}

/*
 * Trailer de archivo para cplm_kernel.cpp
 *
 * [EOF]
 */