/**************************************************************************/
/*!
    @file     ModernPIDControlSS.h
    @author   Oluwasegun Somefun (oasomefun@futa.edu.ng, somefuno@oregonstate.edu)

        A PID Library for the Arduino Board

        This is a streamlined PID library specifically made for Arduino use. It
        uses modern control and signal processing theory (algorithms)
        ----> http://github.com/somefunagba/ModernPIDControlSS

        This library is invested time and resources,
        please support by sharing and starring on GitHub!

        @section  HISTORY

*/
/**************************************************************************/
#pragma once

#ifndef MODERNPIDCONTROLSS_MODERNPIDCONTROLSS_H
#define MODERNPIDCONTROLSS_MODERNPIDCONTROLSS_H

// Incluir componentes principales de la biblioteca PID
#include "pidkernel/PIDNet.h"       // Lógica central del controlador PID (clase PIDNet)
#include "cplmfc/filterFO_pass.h"   // Utilidades de filtro de primer orden, potencialmente usadas por PIDNet o cplmfc
#include "cplmfc/cplmfc.h"          // Algoritmo de autoajuste CPLMFC (clase cplmfc)
#include "dynsys/testsys_ss.h"      // Para simular sistemas dinámicos (usado principalmente en ejemplos de la biblioteca)
#include "helpers/norm++kernel.h"   // Utilidades de ayuda (ej., funciones de normalización, no detalladas aquí)

// Los comentarios estilo Doxygen para funciones ya están presentes sobre las declaraciones.
// Las siguientes son implementaciones en línea de las funciones PID_kernelOS.

/**************************************************************************/
/*!
    @brief  Inicia la evolución del lazo de control y ajuste usando el auto-sintonizador CPLMFC.
            Esta función se llama repetidamente en el bucle principal de un sketch de Arduino.
            Maneja tanto la ejecución del algoritmo de ajuste CPLMFC como el cómputo del PID.
            Solo se ejecuta si ha pasado suficiente tiempo desde la última ejecución,
            determinado por el tiempo de muestreo `Ts` en el objeto `Knet` (PIDNet).
    @param Knet El objeto PIDNet (instancia del controlador PID) a ser controlado y ajustado.
                Contiene todas las variables de estado, parámetros e información de temporización del PID como `Ts` y `T_prev`.
    @param Tune El objeto cplmfc (instancia del sintonizador CPLMFC) que adaptará los parámetros PID de `Knet`.
    @param t El tiempo actual en segundos. Este debe ser un valor continuamente creciente,
               típicamente de `millis()/1000.0` o `micros()/1E6`.
    @returns int flag: Devuelve 1 si el cómputo del PID y el ajuste se realizaron en esta llamada.
                       Devuelve 0 si el intervalo de tiempo de muestreo (`Knet.Ts`) aún no ha transcurrido
                       desde el último cómputo (`Knet.T_prev`).
*/
/**************************************************************************/
inline int PID_kernelOS(PIDNet&, cplmfc&, const double&); // Declaración para la versión de autoajuste CPLMFC

/*!
    @brief Inicia la evolución del lazo de control con parámetros de ajuste PID manuales (Kp, Ti, Td).
           Esta función se llama repetidamente. Establece las ganancias PID basándose en la
           ganancia proporcional (Kp), constante de tiempo integral (Ti) y constante de tiempo derivativa (Td) proporcionadas.
           Solo ejecuta el cómputo del PID si ha pasado suficiente tiempo desde la última ejecución.
    @param Knet El objeto PIDNet (instancia del controlador PID) a ser controlado.
    @param t El tiempo actual en segundos.
    @param Kp La ganancia proporcional para el controlador PID.
    @param Ti La constante de tiempo integral (en segundos) para el controlador PID.
               La ganancia integral (Ki) se calculará como Kp / Ti.
    @param Td La constante de tiempo derivativa (en segundos) para el controlador PID.
               La ganancia derivativa (Kd) se calculará como Kp * Td.
    @returns char flag: Devuelve 1 si se realizó el cómputo del PID.
                        Devuelve 0 si el intervalo de tiempo de muestreo no ha transcurrido.
*/
inline char PID_kernelOS(PIDNet&, const double&, const float&, const float&, const float&); // Declaración para ajuste manual (Kp, Ti, Td)

/*!
    @brief Inicia la evolución del lazo de control con parámetros de ajuste PID manuales (Kp, Ki, Kd).
           Esta función se llama repetidamente. Establece las ganancias PID directamente.
           Solo ejecuta el cómputo del PID si ha pasado suficiente tiempo desde la última ejecución.
    @param Knet El objeto PIDNet (instancia del controlador PID) a ser controlado.
    @param t El tiempo actual en segundos.
    @param Kp La ganancia proporcional para el controlador PID.
    @param Ki La ganancia integral para el controlador PID.
               La constante de tiempo integral (Ti) se calculará como Kp / Ki (si Ki no es cero).
    @param Kd La ganancia derivativa para el controlador PID.
               La constante de tiempo derivativa (Td) se calculará como Kd / Kp (si Kp no es cero).
    @param flag Un argumento char ficticio, aparentemente para propósitos de sobrecarga de funciones para diferenciar
                esta firma de otras. Típicamente se establece a 0 en los ejemplos.
    @returns char flag: Devuelve 1 si se realizó el cómputo del PID.
                        Devuelve 0 si el intervalo de tiempo de muestreo no ha transcurrido.
*/
inline char PID_kernelOS(PIDNet&, const double&, const float&, const float&, const float&, char flag); // Declaración para ajuste manual (Kp, Ki, Kd)

/*
 * Auto. Adaptive CPLMFC
 * Implementación de PID_kernelOS para autoajuste con CPLMFC.
 * Esta versión orquesta el algoritmo de ajuste CPLMFC y luego el cómputo del PID.
 */
int PID_kernelOS(PIDNet& Knet, cplmfc& Tune, const double& t) {
    int result_flag = 0; // Bandera para indicar si el PID y el ajuste se ejecutaron. Inicializar a 0 (no se ejecutó).
    // Comprobar si el tiempo actual 't' está en o después del próximo tiempo de ejecución programado.
    // Knet.T_prev almacena el tiempo de la última ejecución.
    // Knet.Ts es el período de muestreo.
    // (Knet.T_prev + Knet.Ts) es el próximo tiempo de ejecución nominal.
    // (0.5 * Knet.Ts) es un factor de tolerancia, permitiendo la ejecución si 't' está dentro de medio período de muestreo
    // antes del tiempo exacto siguiente. Esto puede ayudar a asegurar la ejecución incluso con ligeras variaciones de temporización
    // en el bucle principal, creando efectivamente una ventana para el disparador de ejecución.
    if (t >=( (Knet.T_prev+Knet.Ts)-(0.5*Knet.Ts) )) {
        /* Evolución CPLM */
        // Ejecutar el algoritmo de ajuste CPLMFC.
        // Este método leerá el estado actual del sistema desde Knet (como error, salida)
        // y actualizará los parámetros PID de Knet (Kp, Ki, Kd) basándose en su lógica interna.
        Tune.run(Knet, t);

        /* Evolución del Estado de Control PID: Arquitectura */
        // Ejecutar el cómputo central del PID usando las ganancias (potencialmente actualizadas) en Knet.
        // Este método calcula la nueva salida de control (Knet.u) y actualiza Knet.T_prev a 't'.
        Knet.compute(t);

        result_flag = 1; // Establecer bandera a 1, indicando que el PID y el ajuste se realizaron.
    }
    return result_flag; // Devolver la bandera.
}

/*
 * Manual I
 * Implementación de PID_kernelOS para ajuste manual usando Kp, Ti (tiempo integral) y Td (tiempo derivativo).
 */
char PID_kernelOS(PIDNet& Knet, const double& t, const float& Kp, const float& Ti, const float& Td) {
    char result_flag = 0; // Bandera para indicar si el PID se ejecutó.
    /* Establecer Ganancias Manualmente */
    // Asignar directamente Kp, Ti, Td proporcionados al objeto Knet.
    Knet.Kp = Kp; // Establecer ganancia proporcional.
    Knet.Ti = Ti; // Establecer constante de tiempo integral.
    Knet.Td = Td; // Establecer constante de tiempo derivativa.

    // Calcular ganancia integral (Ki) y ganancia derivativa (Kd) a partir de Kp, Ti, Td.
    // Estas ganancias derivadas serán usadas por Knet.compute().
    // Manejar posible división por cero si Ti es cero para evitar NaN/infinito.
    Knet.Ki = (Ti != 0.0f) ? (Kp / Ti) : 0.0f;
    Knet.Kd = Kp * Td;

    // Comprobar condición de tiempo de muestreo, similar a la versión CPLMFC.
    if (t >=( (Knet.T_prev+Knet.Ts)-(0.5*Knet.Ts) )) {
        /* Evolución del Estado de Control PID: Arquitectura */
        // Ejecutar el cómputo del PID con las ganancias establecidas manualmente.
        // Actualiza Knet.u y Knet.T_prev.
        Knet.compute(t);
        result_flag = 1; // Establecer bandera a 1, indicando que se realizó el PID.
    }
    return result_flag;
}

/*
 * Manual II
 * Implementación de PID_kernelOS para ajuste manual usando Kp, Ki (ganancia integral) y Kd (ganancia derivativa) directamente.
 * El último parámetro 'flag' es un argumento ficticio usado para diferenciación de firma de función (sobrecarga).
 */
char PID_kernelOS(PIDNet& Knet, const double& t, const float& Kp, const float& Ki, const float& Kd, char d_flag) { // d_flag es el ficticio
    // char flag = 0; // Línea original comentada, se usa result_flag en su lugar.
    char result_flag = 0; // Bandera para indicar si el PID se ejecutó.
    /* Establecer Ganancias Manualmente */
    // Asignar directamente las ganancias Kp, Ki, Kd proporcionadas al objeto Knet.
    Knet.Kp = Kp; // Establecer ganancia proporcional.
    Knet.Ki = Ki; // Establecer ganancia integral.
    Knet.Kd = Kd; // Establecer ganancia derivativa.

    // Calcular tiempo integral (Ti) y tiempo derivativo (Td) a partir de Kp, Ki, Kd por completitud o referencia.
    // Estos podrían no ser usados directamente por Knet.compute() si este usa primariamente Kp, Ki, Kd.
    // Manejar posible división por cero.
    Knet.Ti = (Ki != 0.0f) ? (Kp / Ki) : 0.0f;
    Knet.Td = (Kp != 0.0f) ? (Kd / Kp) : 0.0f;

    // Comprobar condición de tiempo de muestreo.
    if (t >=( (Knet.T_prev+Knet.Ts)-(0.5*Knet.Ts) )) {
        /* Evolución del Estado de Control PID: Arquitectura */
        // Ejecutar el cómputo del PID con las ganancias establecidas manualmente.
        // Actualiza Knet.u y Knet.T_prev.
        Knet.compute(t);
        result_flag = 1; // Establecer bandera a 1, indicando que se realizó el PID.
    }
    return result_flag;
}



#endif //MODERNPIDCONTROLSS_MODERNPIDCONTROLSS_H
