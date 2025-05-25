/*
 * Ejemplo de Simulación de un Sistema de Primer Orden con Retardo (FOPDT)
 * y Control PID con Auto-Sintonización CPLMFC.
 * Configura 2 PIDs para controlar dos sistemas simulados idénticos o diferentes.
 */
#include "Arduino.h"

/* Archivos de Inclusión de la Biblioteca PID */
#include <ModernPIDControlSS.h>

/* Declaraciones Globales */
/* Parámetros de Sintonización (Nota: en este ejemplo, la sintonización es automática vía CPLMFC) */

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
// PID I: Controla la planta simulada I
PIDNet PIDobj_I(r[0], y[0], dt[0],
        umax[0], umin[0], dead_max[0], dead_min[0]);
// PID II: Controla la planta simulada II
PIDNet PIDobj_II(r[1], y[1], dt[1],
        umax[1], umin[1], dead_max[1], dead_min[1]);

// Instancias CPLMFC para cada PID, para sintonización automática
cplmfc cplmfc_I;
cplmfc cplmfc_II;

void setup() {
    // Activar Comunicaciones Seriales.
    // 9600 baudios es una configuración de compromiso entre fácil de detener un proceso fuera de control
    // y un proceso muy lento.
    
    Serial.begin(9600); // Puede ir hasta 2M (2,000,000 baudios)

    /* Config. opcional de Parámetros de la Estructura PID */
    // Configurar ponderación de setpoint y modo de seguimiento (anti-windup)
    PIDobj_I.set_bc_follow(1, 0, 1); // b=1 (P sobre error), c=0 (D sobre -PV), follow=1 (anti-windup habilitado)
    PIDobj_II.set_bc_follow(1, 1, 1); // b=1 (P sobre error), c=1 (D sobre error), follow=1 (anti-windup habilitado)

    /* Config. de Hiperparámetros de Ajuste CPLMFC */
    /* Hiperparámetro para Kp y Pesos Críticos */ // para P, I, D
    // Establecer tasas de aprendizaje para los sintonizadores CPLMFC
    cplmfc_I.set_alpha_critics(PIDobj_I, 5.0);  // Alpha para el sintonizador del PID I
    cplmfc_II.set_alpha_critics(PIDobj_II, 20.0); // Alpha para el sintonizador del PID II (puede ser diferente)

    /* Config. del Modelo de Sentido Común PID (para CPLMFC) */
    // TODO: si se cambia el tiempo de muestreo
    //  volver a ejecutar la identificación del tiempo de establecimiento en lazo cerrado
    //  para obtener los N_ts y N_tau_l correctos
    // conteo de tiempo de establecimiento identificado
    int N_ts = 228; // Ejemplo: 228 conteos a 0.01seg de tiempo de muestreo; ESTE VALOR DEBE SER IDENTIFICADO PARA SU SISTEMA
    // conteo de tiempo de retardo identificado = 0, asumido
    // Inicializar los sintonizadores CPLMFC con los parámetros del sistema (N_ts, N_tau_l)
    cplmfc_I.begin(PIDobj_I, N_ts, 0); // N_tau_l = 0 (sin retardo asumido)
    cplmfc_II.begin(PIDobj_II, N_ts, 0); // Usando los mismos N_ts, N_tau_l para la planta II como ejemplo
}

/* Planta Simulada */
double xsI = 0;  // Variable de estado para la planta simulada I
double xsII = 0; // Variable de estado para la planta simulada II
double tstart = 0; // Tiempo de inicio para la temporización del bucle
double dts = 0.01; // Intervalo de tiempo de muestreo para la simulación (debe coincidir con dt de los PID)
double t_prev = -dts; // Tiempo previo para la simulación de la planta
double ref_id; // Variable para identificación de referencia (usada en modo sys_id)
int sys_id = 1; // Bandera para modo de Identificación de Sistema (1) o Control PID (0)
double compute_time = 0.0; // Tiempo tomado por el cómputo del lazo de control
char flag = 0; // Bandera de estado genérica para las llamadas a PID_kernelOS

void loop() {
    // Asegurar Noción de muestreo fijo
    //tstart = 0; // Reiniciar tstart en cada iteración de loop() si se desea un tiempo relativo al inicio de loop()
    tstart = micros()/1E6; // Tiempo de inicio absoluto para este ciclo de while
    while (countseq>=0) {
        t = micros()/1E6; // Tiempo actual en segundos
        t = t-tstart;     // Tiempo relativo al inicio del ciclo while
        // Serial.print("Time: ");Serial.print(t);Serial.print(" | ");Serial.println(countseq); // Descomentar para depuración de tiempo

        /* LAZO DE CONTROL PID PLANTA I */
        if (sys_id==0) { // Modo de Control PID
            flag = PID_kernelOS(PIDobj_I, cplmfc_I, t); // Automático (CPLMFC)
            //flag = PID_kernelOS(PIDobj_I, t,5,8,1); // Manual (comentado)
            if (flag) { // Si el PID I se ejecutó
                flag = 0; // Reiniciar bandera
                //Serial.print("upwm_I: "); Serial.println(PIDobj_I.u); // Descomentar para depurar salida PID I
            }
            /* LAZO DE CONTROL PID PLANTA II */
            flag = PID_kernelOS(PIDobj_II, cplmfc_II, t); // Automático (CPLMFC) para PID II
            if (flag) { // Si el PID II se ejecutó
                flag = 0; // Reiniciar bandera
                //Serial.print("upwm_II: "); Serial.println(PIDobj_II.u); // Descomentar para depurar salida PID II
                countseq++; // Incrementar contador de secuencia (solo si ambos PIDs se ejecutan, o ajustar lógica)
            }
            // obtener el tiempo total tomado por el lazo de control: para verificar el tiempo de cómputo del control tomado
            compute_time = (micros()/1000000.0)-t;
        }

        /* ID SISTEMA - INICIO */
        if (sys_id==1) { // Modo de Identificación de Sistema
            if (countseq==0) { // En el primer paso de identificación
                ref_id = 1*255; // Establecer una entrada de referencia/escalón (ej. valor PWM máximo)
                // Inicializar salidas de planta (opcional, depende de la estrategia de ID)
                PIDobj_I.y = 1*255; 
                PIDobj_II.y = 1*255;
            }
            // Aplicar una entrada de prueba a las plantas (ej. bucle abierto o P simple)
            PIDobj_I.uo = ref_id-PIDobj_I.y; // ID SISTEMA - FIN (Ejemplo simple, podría ser un escalón fijo)
            PIDobj_II.uo = ref_id-PIDobj_II.y; // ID SISTEMA - FIN
            countseq++;
        }

        if (t>=((t_prev+dts)-(0.5*dts))) { // Condición de temporización para simular la planta
            /* Simular Entrada de Control a Planta I */
            // testsys_ss actualiza PIDobj_I.y (salida de la planta) basado en PIDobj_I.uo (salida del controlador)
            testsys_ss(PIDobj_I.y, &xsI, dt[0], PIDobj_I.uo, 0, 0);

            /* Simular Entrada de Control a Planta II*/
            testsys_ss(PIDobj_II.y, &xsII, dt[1], PIDobj_II.uo, 0, 0);

            // Registrar Datos
            Serial.println((String) PIDobj_I.y+","+PIDobj_I.ym+","
                    +PIDobj_II.y+","+PIDobj_II.ym+","+countseq+","+compute_time);

            // Descomentar para registrar ganancias PID (serán las sintonizadas por CPLMFC si sys_id=0)
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
