
/*
 * Archivo: sfunPID_kernel.c
 * Arquitectura Moderna del Controlador PID de SOMEFUN
 * oasomefun@futa.edu.ng            : 2019, 2020
 */

/* Archivos de Inclusión */
#include "PIDNet.h"
#include "helpers/norm++kernel.h"

/* Inicialización de Instancia */
PIDNet::PIDNet(double ref, double yout, double dt,
        int umax_lim, int umin_lim, int dead_max, int dead_min) {
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
    uo = 0;
    e_t = 0;

    umax = umax_lim;
    umin = umin_lim;

    this->dead_max = dead_max;
    this->dead_min = dead_min;

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

    /* discretización */
    double cut_freq = (PI)/(10.0*dt);
    /* constante de tiempo de lpf (filtro paso bajo) de primer orden para la derivada */
    // constante bilineal pre-distorsionada y constante de tiempo del filtro
    //double st = tan(PI/20.0); // Código comentado
    kpi = cut_freq/TAN_ST;
    Tf = Ts/(2.0*TAN_ST);

    filter_u;
    uf = 0;
}

/* Definiciones de Funciones */
/*
 *  Esta función implementa el algoritmo de control PID 2-DOF (Dos Grados de Libertad) en realización bilineal.
 * Argumentos    : paramsPID_T *Knet
 * Tipo de Retorno  : void
 */
void PIDNet::compute(const double& t) {

    double ym, yfict, e_u, Keu, ep, kui;

    T_prev = t;
    /*  Esquema de Discretización */
    /*  Parametrización fraccional bilineal */
    /*  restringir el sintonizador de discretización para que esté dentro de la unidad */
    /*  límites del círculo de 0 y 1. */

    /* constante bilineal */
//    // kpi = 2.0/ Ts; // Código comentado
//    cut_freq = (PI)/(10.0*Ts); // Código comentado
//    /* constante de tiempo de lpf (filtro paso bajo) de primer orden para la derivada */
//    // constante bilineal pre-distorsionada y constante de tiempo del filtro
//    st = tan(PI/20.0); // Código comentado
//    kpi = cut_freq/st; // Código comentado
//    Tf = Ts/(2.0*st); // Código comentado

    /*  Entradas */
    if (follow==1) {
        ym = this->ym;
    }
    else {
        this->ym = r;
        ym = this->ym;
    }

    /*  recálculo de salida AWU, anti-windup */
    e_u = (u-v);
    //  Esto cubre una estructura PID desacoplada en lugar del
    //  recálculo de error que cubre una estructura de 1-DoF (Un Grado de Libertad) solo de error.
    /*  Coeficiente de recálculo de salida AWUP */
    Keu = Kp+(0.5*Ts*Ki)+((2/Ts)*Kd);
    kui = 1.5F/Ki;
    yfict = y;
    ua = e_u/Keu;
    yfict += (ua);

    /* Paso Anterior*/

    /*  D */ // Término Derivativo
    ud *= (kpi*Tf-1); // ud anterior
    ud -= kpi*Td*(ed); // ed anterior
    
    /*  I */ // Término Integral
    ui += (1/(Ti*kpi))*(ei); // ui y ei anteriores
    ui -= kui*(upd);

    /* Paso Actual */

    /*  Errores */
    ep = (b*ym)-yfict;
    e = r-y;
    // recálculo de entrada integral
    ei = (ym-yfict)+e_u;
    ed = (c*ym)-yfict;

    /*  Términos de Salida Individuales */
    /*  P */ // Término Proporcional
    up = (ep);

    /*  D */ // Término Derivativo
    ud += kpi*Td*(ed);
    ud = ud/(kpi*Tf+1);

    /* PD */
    /*  cambio en la contribución P, D, P, D sin saltos. almacena actual a anterior */
    // error de la contribución PD anterior si es mayor que la salida
    upd = (up+ud);

    /*  I */ // Término Integral
    ui += (1/(Ti*kpi))*(ei);
    /*  recálculo de salida integral */
    ui -= ua;
    ui += kui*(upd);

    /*  Suma de Salida de Términos Contribuyentes */
    /* Criticar (Evaluar)*/
    e_t = (up+lambdai*(ui)+lambdad*ud);
    v = Kp*e_t;
    /* recálculo de salida combinada */
    uf = v-ua;

    /*  Restricciones Reales de Entrada de Control para u */
    //Serial.print("bef_ u="); Serial.println( u); // Código comentado

    /* Saturación Dura */
    //filter_u.run(u,uf); // filtro
    u = uf;
    u = fmax((double) umin,fmin(u, (double) umax ));
    uo = u;
    //dead_zone<double>(uo, dead_max, dead_min); // Código comentado
    // uo = fmax((double) umin, fmin(uo, (double) umax )); // Código comentado

    /* Mantenimiento Varios. */
    // incrementar contador interno de muestras para el PID.
    countseq += 1;

}
/**************************************************************************/
/*!
    @brief  Establece tres parámetros en la estructura de control PID
    @param b Constante de ponderación del punto de ajuste PID-2DOF: 0 o 1
    @param c Constante de ponderación derivativa PID-2DOF: 0 o 1
    @param follow Lógica para habilitar el filtrado del punto de ajuste: 0 o 1
    @returns void (nada).
*/
/**************************************************************************/
void PIDNet::set_bc_follow(const int& b, const int& c, const char& follow) {
    this->b = b;
    this->c = c;
    this->follow = follow;
}


// SATURACIÓN
// u = maxim((double)  umin, (minim( u, (double)  umax))); // Código comentado

/* Saturación Logística*/
// nlsig( u, du,  u, (double) umax, (double) umin, // Código comentado
//        (double) umax, (double) umin, // Código comentado
//        33, 6, 0, 0); // Código comentado
//Serial.print("aft_ u="); Serial.println( u); // Código comentado

//    double u_norm[1] = {1.0}; // Código comentado
//    double u_act[1] = { u}; // Código comentado
//
//    //Serial.print("prior: "); Serial.println( u); // Código comentado
//    normalize<double>(u_act, u_norm,  umax,  umin); // Código comentado
//    // Serial.print("norm: "); Serial.println(u_norm[0]); // Código comentado
//    u_norm[0] = nlsig(u_norm[0], 1.0, -1.0, // Código comentado
//            1.0, -1.0, // Código comentado
//            33, 6, 0); // Código comentado
//    //Serial.print("out_norm: "); Serial.println(u_norm[0]); // Código comentado
//    denormalize<double>(u_norm, u_act,  umax,  umin); // Código comentado
//     u = u_act[0]; // Código comentado
//    //Serial.print("after: "); Serial.println( u); // Código comentado

// Serial.print("UPWM: ");Serial.println( u); // depuración


//    /* EQUIVALENCIA DE SATURACIÓN, ZONA MUERTA Y FRICCIÓN DE COULOMB */
//    // NL(.) 1-2 . ZONA MUERTA, mín Y ZONA MUERTA INVERSA, máx
//    if ((fabs( u) <= fabs( zerotol))) { // Código comentado
//        // 1. menor o en la zona muerta (límite mínimo)
//         u = 0; // Código comentado
//    } else if ((fabs( u) > fabs( zerotol)) && (fabs( u) <= fabs( deadmax))) { // Código comentado
//        // 2. en la zona muerta inversa (límite máximo)
//         u = copysign( deadmax,  u); // si u < 0, u = -deadmax
//    } else { // Código comentado
//        // 3. fuera de la zona muerta inversa (límite máx)
//        // deadmax añadido como perturbación, efecto de la fricción de Coulomb en cierto sentido.
//         u = copysign(fabs( u +  deadmax),  u); // Código comentado
//    }

/*
 * Trailer de archivo para sfunPID.cpp
 *
 * [EOF]
 */
