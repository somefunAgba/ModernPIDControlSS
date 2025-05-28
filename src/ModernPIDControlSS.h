/**************************************************************************/
/*!
    @file     ModernPIDControlSS.h
    @author   Oluwasegun Somefun (oasomefun@futa.edu.ng, somefuno@oregonstate.edu)

        Una librería PID para la placa Arduino

        Esta es una librería PID optimizada específicamente para su uso con Arduino. Utiliza
        teoría moderna de control y procesamiento de señales (algoritmos)
        ----> http://github.com/somefunagba/ModernPIDControlSS

        En esta librería se ha invertido tiempo y recursos,
        ¡por favor apoya compartiendo y marcando como favorito en GitHub!

    @section  HISTORIAL

*/
/**************************************************************************/
#pragma once

#ifndef MODERNPIDCONTROLSS_MODERNPIDCONTROLSS_H
#define MODERNPIDCONTROLSS_MODERNPIDCONTROLSS_H

#include "pidkernel/PIDNet.h"
#include "cplmfc/filterFO_pass.h"
#include "cplmfc/cplmfc.h"
#include "dynsys/testsys_ss.h"
#include "helpers/norm++kernel.h"

/**************************************************************************/
/*!
    @brief  Inicia la evolución del bucle de control y sintonización
    @param Knet La instancia del controlador PID en el bucle
    @param Tune La instancia del algoritmo de sintonización CPLMFC para este controlador PID
    @param t El tiempo actual
    @returns carácter de bandera (flag).
*/
/**************************************************************************/
inline int PID_kernelOS(PIDNet&, cplmfc&, const double&);
/*!
    @brief Inicia la evolución del bucle de control, pasando parámetros de sintonización manual: Kp, Ti, Td
    @param Knet La instancia del controlador PID en el bucle
    @param t El tiempo actual
    @param Kp ganancia proporcional
    @param Ti constante de tiempo integral
    @param Td constante de tiempo derivativa
    @returns carácter de bandera (flag).
*/
inline char PID_kernelOS(PIDNet&, const double&, const float&, const float&, const float&);

/*!
    @brief Inicia la evolución del bucle de control, pasando parámetros de sintonización manual: Kp, Ki, Kd
    @param Knet La instancia del controlador PID en el bucle
    @param t El tiempo actual
    @param Kp ganancia proporcional
    @param Ki ganancia integral
    @param Kd ganancia derivativa
    @param flag argumento ficticio (dummy): establecer como 0
    @returns carácter de bandera (flag).
*/
inline char PID_kernelOS(PIDNet&, const double&, const float&, const float&, const float&, char flag);

/*
 * CPLMFC Adaptativo Automático
 */
int PID_kernelOS(PIDNet& Knet, cplmfc& Tune, const double& t) {
    int flag = 0;
    if (t >=( (Knet.T_prev+Knet.Ts)-(0.5*Knet.Ts) )) {
        /* Evolución CPLM */
        Tune.run(Knet, t);
        /* Evolución del Estado de Control PID: Arquitectura */
        Knet.compute(t);
        flag = 1;
    }
    return flag;
}

/*
 * Manual I
 */
char PID_kernelOS(PIDNet& Knet, const double& t, const float& Kp, const float& Ti, const float& Td) {
    char flag = 0;
    /* Establecer Ganancias Manualmente */
    Knet.Kp = Kp;
    Knet.Ti = Ti;
    Knet.Td = Td;
    Knet.Ki = Kp/Ti;
    Knet.Kd = Kp*Td;
    if (t >=( (Knet.T_prev+Knet.Ts)-(0.5*Knet.Ts) )) {
        /* Evolución del Estado de Control PID: Arquitectura */
        Knet.compute(t);
        flag = 1;
    }
    return flag;
}

/*
 * Manual II
 */
char PID_kernelOS(PIDNet& Knet, const double& t, const float& Kp, const float& Ki, const float& Kd, char flag) {
    // char flag = 0; // Este comentario parece ser código desactivado, lo mantengo tal cual pero traduzco "flag" si fuera un comentario explicativo.
                     // En este caso, "flag" es nombre de variable, así que no se traduce.
    /* Establecer Ganancias Manualmente */
    Knet.Kp = Kp;
    Knet.Ki = Ki;
    Knet.Kd = Kd;
    Knet.Ti = Kp/Ki;
    Knet.Td = Kd/Kp;
    if (t >=( (Knet.T_prev+Knet.Ts)-(0.5*Knet.Ts) )) {
        /* Evolución del Estado de Control PID: Arquitectura */
        Knet.compute(t);
        flag = 1;
    }
    return flag;
}

// MODERNPIDCONTROLSS_MODERNPIDCONTROLSS_H (Comentario de fin de guarda de inclusión, sin traducción)
#endif //MODERNPIDCONTROLSS_MODERNPIDCONTROLSS_H
