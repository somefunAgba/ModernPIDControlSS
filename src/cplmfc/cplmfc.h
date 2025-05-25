
/*
 * File: CPLMFC.h
 *
 * <CLOSED PID-LOOP MODEL> <FOLLOWING CONTROL> <METHOD> : 2019-2020
 *
 * oasomefun@futa.edu.ng. Copyright.2020
 */

#ifndef CPLMFC_H
#define CPLMFC_H

/* Include Files */
#include <Arduino.h>
#include "pidkernel/PIDNet.h"       // Dependencia: Clase PIDNet para acceder a los estados y parámetros del PID.
#include "cplmfc/filterFO_pass.h"   // Dependencia: Filtro de primer orden, usado para filtrar la señal de referencia.
#include "nlsig/nlsig.h"            // Dependencia: Generación de señales no lineales (detalles no especificados aquí, pero probablemente para señales de prueba o perfiles de ajuste específicos).

/*!
 *  @class cplmfc
 *  @brief Implementa el algoritmo de ajuste CPLMFC (Control por Seguimiento de Modelo en Lazo PID Cerrado).
 *
 *  Esta clase provee métodos para ajustar automáticamente las ganancias (Kp, Ki, Kd) de un controlador PIDNet.
 *  Funciona comparando la respuesta en lazo cerrado del PID con un modelo de referencia y ajustando
 *  las ganancias para minimizar la diferencia, haciendo efectivamente que el comportamiento del controlador PID
 *  siga al de un modelo deseado. El proceso de ajuste implica identificar características del sistema
 *  (como tiempo de establecimiento y tiempo muerto) y usar adaptación basada en críticos
 *  para los parámetros PID.
 */
class cplmfc{

public:
    /*!
     *  @brief Inicializa el sintonizador CPLMFC con las características del sistema.
     *  @param Knet Referencia al objeto PIDNet sobre el cual operará este sintonizador.
     *              Usado para inicializar parámetros internos como el tiempo de muestreo.
     *  @param N_ts Tiempo de establecimiento del sistema identificado, en número de intervalos de muestreo (Ts).
     *              Este es un parámetro crucial obtenido de la identificación del sistema (ej. respuesta al escalón).
     *  @param N_tau_l Tiempo muerto (retraso) del sistema identificado, en número de intervalos de muestreo (Ts).
     *                 Por defecto es 0 si no se especifica. También obtenido de la identificación del sistema.
     */
    void begin(PIDNet& Knet, const int& N_ts, const int& N_tau_l = 0);

    /*!
     *  @brief Establece las tasas de aprendizaje (valores alfa) para los críticos de ganancia PID.
     *  @param Knet Referencia al objeto PIDNet (no usado directamente en el cuerpo del método actual, pero bueno para el contexto).
     *  @param alpha_ Tasa de aprendizaje del crítico Kp. Este es el principal parámetro de ajuste para la agresividad de CPLMFC.
     *  @param lambda_i Tasa de aprendizaje del crítico Ki (referida como alpha_i en .cpp). Por defecto 0.5.
     *  @param lambda_d Tasa de aprendizaje del crítico Kd (referida como alpha_d en .cpp). Por defecto 0.1.
     */
    void set_alpha_critics(PIDNet& Knet, const float& alpha_, const float& lambda_i = 0.5F, const float& lambda_d = 0.1F);

    /*!
     *  @brief Ejecuta un paso del algoritmo de ajuste CPLMFC.
     *         Este método debe ser llamado periódicamente a la tasa de muestreo del sistema de control.
     *         Internamente llama a métodos para ajustar Kp, Ki y Kd.
     *  @param Knet Referencia al objeto PIDNet cuyas ganancias se están ajustando.
     *              El sintonizador leerá variables de proceso de Knet y actualizará Knet.Kp, Knet.Ki, Knet.Kd.
     *  @param t Tiempo actual en segundos.
     */
    void run(PIDNet& Knet, const double& t);

    // Métodos auxiliares de ajuste interno, potencialmente podrían ser privados o protegidos
    // si no están destinados para uso externo directo, pero aquí son públicos.

    /*!
     *  @brief Ajusta el parámetro de frecuencia natural (Wn), que influye en Kp.
     *         (Nota: La implementación real en cplmfc.cpp parece ajustar Knet.Kp directamente
     *         basado en criterios de error, en lugar de ajustar explícitamente un 'Wn' que luego establece Kp.
     *         El nombre podría ser un poco engañoso o referirse a un concepto teórico subyacente.)
     *  @param Knet Referencia al objeto PIDNet.
     */
    void tuneWn(PIDNet& Knet); // Wn está relacionado con el ajuste de Kp

    /*!
     *  @brief Ajusta la ganancia proporcional (Kp) del controlador PID.
     *  @param Knet Referencia al objeto PIDNet.
     *  @param t Tiempo actual en segundos.
     *  @param mode Modo de ajuste (0 o 1), influye en cómo se ajusta Kp basado en señales de error.
     */
    void tuneKp(PIDNet& Knet, const double& t, const int& mode);

    /*!
     *  @brief Ajusta las ganancias integral (Ki) y derivativa (Kd) del controlador PID.
     *         Estas se ajustan típicamente en relación con el valor actual de Kp.
     *  @param Knet Referencia al objeto PIDNet.
     */
    void tuneKiKd(PIDNet& Knet) const; // Marcado como const, implica que podría no cambiar miembros de cplmfc directamente pero modifica Knet.

    // --- Variables Miembro ---

    float wn = 1.0F;      //!< Frecuencia natural objetivo (rad/s) para el sistema en lazo cerrado.
                          //!< Usada como base para calcular los parámetros del modelo de referencia. Inicializada a 1.0.
                          //!< Este 'wn' parece ser más una configuración para el comportamiento del modelo de referencia
                          //!< que un parámetro directamente ajustado del PID en sí mismo en `tuneWn`.

    float alpha = 1.0F;   //!< Tasa de aprendizaje primaria para el crítico Kp (mecanismo de adaptación).
                          //!< Valores más altos llevan a una adaptación más rápida pero pueden causar inestabilidad.

    double ts = 1.0F;     //!< Tiempo de muestreo (segundos) del sistema de control.
                          //!< Debería estar sincronizado con el Ts de PIDNet. Inicializado a 1.0 pero establecido en `begin()`.

    float tau_l = 0.0F;   //!< Tiempo muerto del sistema (segundos). Calculado a partir de N_tau_l y ts en `begin()`.

    filterFO_pass filter_r; //!< Instancia de filtro de primer orden. Usada para filtrar la señal de referencia (setpoint `r`)
                            //!< para generar una referencia suavizada `ym` para la lógica de seguimiento de modelo.
                            //!< Esto ayuda a lograr un control más suave y a reducir los ajustes de ganancia de alta frecuencia.
};

#endif
/*
 * File trailer for cplmfc.h
 *
 * [EOF]
 */