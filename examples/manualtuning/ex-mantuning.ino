/*
 * Ejemplo de Sintonización Manual
 * Configurar 2 PIDs
 */
#include "Arduino.h"

/* Archivos de Inclusión de la Biblioteca PID */
#include <ModernPIDControlSS.h>

/* Declaraciones Globales */
/* Parámetros de Sintonización */

/* Información de Temporización */
double dt[2] = {0.01, 0.01}; // Intervalo de tiempo para cada PID (delta t)
double t = 0.0; // Tiempo actual

const double r[2] = {150.0, 150.0}; // Setpoints (referencias) para cada PID
double y[2] = {0, 0}; // Variables de proceso (salidas medidas) para cada PID

int countseq = 0; // Contador de secuencia

// Límites de la salida de control y zona muerta para cada PID
const int umin[2] = {-255, -255}; // Límite inferior de la salida del actuador
const int umax[2] = {255, 255};  // Límite superior de la salida del actuador
const int dead_max[2] = {35, 35}; // Límite superior de la zona muerta
const int dead_min[2] = {0, 0}; // Límites de tolerancia cero (sin zona muerta inferior explícita si es igual a 0)

/* Config. de Parámetros de la Estructura PID */
// PID I
PIDNet PIDobj_I(r[0], y[0], dt[0],
        umax[0], umin[0], dead_max[0], dead_min[0]);
// PID II        
PIDNet PIDobj_II(r[1], y[1], dt[1],
        umax[1], umin[1], dead_max[1], dead_min[1]);

// Instancias CPLMFC (no usadas en este ejemplo de sintonización manual, pero declaradas)
cplmfc cplmfc_I;
cplmfc cplmfc_II;

void setup() {
    // Activar Comunicaciones Seriales.
    // 9600 baudios es una configuración de compromiso entre fácil de detener un proceso fuera de control
    // y un proceso muy lento
    
    Serial.begin(9600); // Puede ir hasta 2M (2,000,000 baudios)

    /* Config. opcional de Parámetros de la Estructura PID */
    // Configurar ponderación de setpoint y modo de seguimiento (anti-windup)
    PIDobj_I.set_bc_follow(1, 0, 1); // b=1 (P sobre error), c=0 (D sobre -PV), follow=1 (anti-windup habilitado)
    PIDobj_II.set_bc_follow(1, 1, 1); // b=1 (P sobre error), c=1 (D sobre error), follow=1 (anti-windup habilitado)
}

/* Planta Simulada */
double xsI = 0;  // Variable de estado para la planta simulada I
double xsII = 0; // Variable de estado para la planta simulada II
double tstart = 0; // Tiempo de inicio para la temporización del bucle
double dts = 0.01; // Intervalo de tiempo de muestreo para la simulación (debe coincidir con dt de los PID)
double t_prev = -dts; // Tiempo previo para la simulación de la planta
double ref_id; // Variable para identificación de referencia (no usada activamente aquí)
double compute_time = 0.0; // Tiempo tomado por el cómputo del lazo de control
char flag1 = 0; // Bandera de estado para PID I
char flag2 = 0; // Bandera de estado para PID II

void loop() {
    // Asegurar Noción de muestreo fijo
    //tstart = 0; // Reiniciar tstart en cada iteración de loop() si se desea un tiempo relativo al inicio de loop()
    tstart = micros()/1E6; // Tiempo de inicio absoluto para este ciclo de while
    while (countseq >= 0) {
        t = micros()/1E6; // Tiempo actual en segundos
        t = t-tstart;     // Tiempo relativo al inicio del ciclo while
        // Serial.print("Time: ");Serial.print(t);Serial.print(" | ");Serial.println(countseq); // Descomentar para depuración de tiempo

        /*! LAZO DE CONTROL PID para PLANTA I  */
        ///*!
        // @brief Inicia la evolución del lazo de control, pasando parámetros de sintonización manual: Kp, Ki, Kd
        // @param Knet La instancia del controlador PID en el lazo
        // @param t El tiempo actual
        // @param Kp ganancia proporcional
        // @param Ki ganancia integral
        // @param Kd ganancia derivativa
        // @param flag argumento ficticio: establecer como 0
        // @returns carácter de bandera.
        //*/
        flag1 = PID_kernelOS(PIDobj_I, t, 5.0, 8.0, 1.0, 0); // Manual: Kp=5.0, Ki=8.0, Kd=1.0

        /*! LAZO DE CONTROL PID para PLANTA II */
        // /*!
        // @brief Inicia la evolución del lazo de control, pasando parámetros de sintonización manual: Kp, Ti, Td
        // @param Knet La instancia del controlador PID en el lazo
        // @param t El tiempo actual
        // @param Kp ganancia proporcional
        // @param Ti constante de tiempo integral
        // @param Td constante de tiempo derivativa
        // @returns carácter de bandera.
        // 
        flag2 = PID_kernelOS(PIDobj_II, t, 1.0, 0.3, 0.1); // Manual: Kp=1.0, Ti=0.3, Td=0.1

        if (flag1 & flag2) { // Si ambos PIDs se ejecutaron
            flag1 = 0; // Reiniciar banderas
            flag2 = 0;
             //Serial.print("upwm_I: "); Serial.println(PIDobj_I.u); // Descomentar para depurar salida PID I
            //Serial.print("upwm_II: "); Serial.println(PIDobj_II.u); // Descomentar para depurar salida PID II
            countseq++; // Incrementar contador de secuencia
        }
        // obtener el tiempo total tomado por el lazo de control: para verificar el tiempo de cómputo del control tomado
        compute_time = (micros()/1000000.0)-t; 
        

        if (t>=((t_prev+dts)-(0.5*dts))) { // Condición de temporización para simular la planta
            /* Simular Entrada de Control a Planta I */
            // testsys_ss actualiza PIDobj_I.y (salida de la planta) basado en PIDobj_I.uo (salida del controlador)
            testsys_ss(PIDobj_I.y, &xsI, dt[0], PIDobj_I.uo, 0, 0);

            /* Simular Entrada de Control a Planta II*/
            testsys_ss(PIDobj_II.y, &xsII, dt[1], PIDobj_II.uo, 0, 0);

            // Registrar Datos
            Serial.println((String) PIDobj_I.y+","+PIDobj_I.ym+","
                    +PIDobj_II.y+","+PIDobj_II.ym+","+countseq+","+compute_time);

            // Descomentar para registrar ganancias PID (serán las manuales establecidas)
            //Serial.println((String) PIDobj_I.Kp + "," + PIDobj_I.Ti + "," + PIDobj_I.Td +
            //                    "," + PIDobj_II.Kp+ "," + PIDobj_II.Ti + "," + PIDobj_II.Td);

            t_prev = t; // Actualizar tiempo previo de simulación
        }

        if (countseq>=1E3) { // Detener después de 1000 secuencias
            countseq = -1; // Condición para salir del bucle while
            break;
        }

    }

}
