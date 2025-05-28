/*
 * Archivo: CPLMFC.cpp
 *
 * <MODELO DE BUCLE PID CERRADO> <CONTROL DE SEGUIMIENTO> <MÉTODO> : 2020
 *
 * oasomefun@futa.edu.ng. Copyright.2020
 */

#include "cplmfc.h"
/**************************************************************************/
/*!
    @brief Establece manualmente los hiperparámetros para el algoritmo de sintonización
    @param Knet La instancia del controlador PID en el bucle
    @param alpha controla la ganancia proporcional
    @param lambda_i controla la contribución de salida del error integral
    @param lambda_d controla la contribución de salida del error derivativo
    @returns void (nada).
*/
/**************************************************************************/
void cplmfc::set_alpha_critics(PIDNet& Knet, const float& alpha, const float& lambda_i, const float& lambda_d) {
    this->alpha = alpha;
    Knet.lambdai = lambda_i;
    Knet.lambdad = lambda_d;
}
/**************************************************************************/
/*!
    @brief Establece el tiempo de estabilización y el ancho de banda del bucle cerrado
    @param Knet La instancia del controlador PID en el bucle
    @param N_ts El conteo de tiempo discreto para el tiempo de estabilización de la salida a controlar
    @param N_taul El conteo de tiempo discreto para el retardo de entrada-salida en el bucle
    @returns void (nada).
*/
/**************************************************************************/
void cplmfc::begin(PIDNet& Knet, const int& N_ts, const int& N_taul) {
    ts = N_ts*Knet.Ts;
    tau_l = float(N_taul*Knet.Ts);
    // Frecuencia natural = ancho de banda
    tuneWn(Knet);
}
/**************************************************************************/
/*!
    @brief Ejecuta el algoritmo de sintonización
    @param Knet La instancia del controlador PID en el bucle
    @param t El tiempo actual
    @returns void (nada).
*/
/**************************************************************************/
void cplmfc::run(PIDNet& Knet, const double& t) {
    filter_r.run(Knet.ym, Knet.r);
    // Serial.print("ym: "); Serial.println(PIDobj_I.ym); // Código comentado, no se traduce el contenido del código
    /* Cálculo de Sintonización CPLMFC */
    tuneKp(Knet, t, 2);
    //Serial.print("Kp: ");Serial.println(PIDobj_I.Kp); // Código comentado
    tuneKiKd(Knet);
    //Serial.print(" Ki: ");Serial.print(PIDobj_I.Ki); // Código comentado
    //Serial.print(" Kd: ");Serial.println(PIDobj_I.Kd); // Código comentado
}

void cplmfc::tuneWn(PIDNet& Knet) {
    /*  funciones del algoritmo de sintonización */
    float xtsn;
    int xx = Knet.b+Knet.c;
    if (xx==0) {
        xtsn = 9.98;
//        Serial.println(xtsn); // Código comentado
    }
    else if (xx==1) {
        xtsn = 7.74;
//        Serial.println(xtsn); // Código comentado
    }
    else if (xx==2) {
        xtsn = 11.07;
//        Serial.println(xtsn); // Código comentado
    }
    else {
        xtsn = 8;
    }

    /*  w_n normalizado */
    wn = float(xtsn/(0.7071*ts));
    //Serial.println(xtsn); // Código comentado
    //printf("wn: %lf\n", wn); //depuración

}

void cplmfc::tuneKp(PIDNet& Knet, const double& t, const int& mode) {
    /* actualización n-logística*/
    // función racional ajustada a la curva
    double k_sig1, k_sig2, e, xe, xu, x_lim, LL, kg, kp_lim, e_t;
    // double dkp; // Código comentado
    LL = (tau_l-5.936F)/(8.771F); // L está normalizado por la media 5.936 y la desviación estándar 8.771
    kg = (0.05132*LL*LL+0.2041*LL+0.1214)/(LL*LL+1.538*LL+0.5864);
    kp_lim = alpha*kg*(tau_l+ts)/(ts);
    //Serial.print("kp_lim: "); Serial.println(kp_lim); // Código comentado

    // e = Knet.ym-Knet.y; // Código comentado
    xe = kp_lim+Knet.e;
    xu = kp_lim+Knet.u;
    nlsig(k_sig1, Knet.e, xe, -xe, kp_lim, -kp_lim, 1, 0.1, 0, 0);
    nlsig(k_sig2, Knet.u, xu, -xu, kp_lim, -kp_lim, 1, 0.1, 0, 0);
    //Serial.print("kp_not:"); Serial.println(k_sig1+k_sig2); // Código comentado

    x_lim = kp_lim+(k_sig1+k_sig2);
    if (mode==0) {
        nlsig(Knet.Kp, (k_sig1+k_sig2), x_lim, 0.0, kp_lim,
                0.0, (int) fmax(1.0, alpha), 0.1, 0, 0);
    }
    if (mode==1) {
        nlsig(Knet.Kp, (k_sig1+k_sig2), x_lim, 0.0,
                kp_lim, 0.0, 16, 0.1, 0, 0);
        // Serial.print("Kpfit: "); Serial.println(Knet.Kp); // Código comentado
    }
    if (mode==2) {
        nlsig(Knet.Kp, (k_sig1+k_sig2), x_lim, 0.0,
                kp_lim, 0.0, 1, 0.1, 0, 0);
    }
    //Serial.print("Kpfit: "); Serial.println(Knet.Kp); // Código comentado

    //Knet.Kp = 16; // Código comentado
    //opción adaptativa: regla de actualización de Lyapunov
    if ( fabs(t) > fabs(double (tau_l)) ) {
        e_t = fabs(Knet.e_t);
        e_t = fmin((double) Knet.umax, e_t);
        // Serial.print("e_t: ");Serial.println(e_t); // Código comentado
        Knet.Kp = Knet.Kp+(alpha*0.001*e)*e_t;
        //Serial.print("Kplya: "); Serial.println(Knet.Kp); // Código comentado
    }

}

void cplmfc::tuneKiKd(PIDNet& Knet) const {
    /* constante de tiempo integral*/
    Knet.Ti = (2.0F*0.7071F)/wn;
    /* ganancia integral*/
    Knet.Ki = Knet.Kp/Knet.Ti;

    /* constante de tiempo derivativa*/
    Knet.Td = 1.0F/(2.0F*0.7071F*wn);
    /* ganancia derivativa*/
    Knet.Kd = Knet.Kp*Knet.Td;
}