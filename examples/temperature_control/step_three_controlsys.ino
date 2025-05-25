/*
 *  PASO3: CONTROL EN LAZO CERRADO DE LA TEMPERATURA DE SALIDA
 *         USANDO UNA ENTRADA DE CONTROL PID AL SISTEMA
 */
#include "Arduino.h"

#include "Wire.h"
#include "Adafruit_MCP4725.h"
#include "Adafruit_MAX31865_library/Adafruit_MAX31865.h"

/*Biblioteca PID*/
#include "ModernPIDControlSS.h"

/* Declaraciones Globales */

/* temporización */
int countseq = 0; // tomar nota de esta variable de paso de tiempo
//int max_discrete_time_count = 5000; // variar según se desee

double t = 0;
double dt_control = 0.1; // tiempo de muestreo del control; puedes jugar con esto como desees
double dt_sys = 1.0; // equivalente a 1 segundo
double t_prev_sys = -dt_sys;
double t_start = 0;


// establecer salida de referencia en celsius
double ref_celsius = 0;
// variables de entrada y salida
int in_pwm;
double in_val;
double out_celsius = 0;

// TODO: inicializar otras bibliotecas para el DAC y ADC
Adafruit_MCP4725 dac; // DAC Adafruit MCP4725
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10,11,12,13); // ADC Adafruit MAX31865 PT100


/* Config. de Parámetros de la Estructura PID */
// 10 -> entrada de control máx., 0 -> entrada de control mín. en voltaje
PIDNet  PID_I(ref_celsius, out_celsius, dt_control,10, 0, 0, 0);
cplmfc cplmfc_tuner;


void setup() {
    // Activar Comunicaciones Seriales.
    // 9600 baudios es una configuración de compromiso entre fácil de detener un proceso fuera de control
    // y un proceso muy lento
    Serial.begin(9600); // Puede ir hasta 2M

    //TODO: configurar DAC y ADC
    dac.begin(0x62); // Biblioteca Adafruit MCP4725A1
    thermo.readRTD(); // Biblioteca Adafruit MAX31865

    /* Config. de Parámetros de la Estructura PID */
     PID_I.set_bc_follow(0,0,0);

    /* Config. de Hiperparámetros de Ajuste CPLMFC */
    /* Hiperparámetro para Kp y Pesos Críticos */ // para P, I, D
    //TODO: incrementar alfa gradualmente para aumentar la respuesta rápida;
    //cplmfc_tuner.set_alpha_critics( PID_I, alpha, lambda_i, lambda_d)
    cplmfc_tuner.set_alpha_critics( PID_I,10.0,0.5,0.1);

    /* Config. del Modelo de Sentido Común PID */
    // nota: si se cambia el tiempo de muestreo
    // volver a ejecutar la identificación del tiempo de establecimiento en lazo cerrado
    // para obtener los N_ts y N_tau_l correctos
    // TODO: reemplazar con el conteo de tiempo de establecimiento identificado del paso 2
    int N_ts = 228; // N_ts a 0.1seg;
    // TODO: reemplazar con el conteo de tiempo de retardo identificado
    //  (es decir: paso de tiempo máximo en el que la salida = 0)
    int N_tau_l = 0;
    //cplmfc_tuner.begin( PID_I, N_ts, N_tau_l);
    cplmfc_tuner.begin( PID_I, N_ts, N_tau_l);

}

char flag;

void loop() {
    // asegurar noción de muestreo fijo
    t_start = millis()/1E3;
    t = t_start;
    while (countseq >= 0) {
        t = millis()/1E3;
        t = t - t_start;
        /* INICIO LAZO CERRADO */
        // Ejecutar el PID y su algoritmo de ajuste
        flag = PID_kernelOS( PID_I, cplmfc_tuner, t);
        if (flag) {
            countseq++;
            flag = 0;
            in_val = PID_I.u;
            in_pwm = int((4095/10.0)*in_val); // convertir voltaje de entrada a pwm
        }
        dac.setVoltage(in_pwm,"false"); // pasar valor de entrada máx. al dac

        /* MUESTREO DE SALIDA DEL SISTEMA-: CADA DT_SYS SEGUNDOS*/
        if ( t >= (t_prev_sys + dt_sys)) {
            out_celsius = thermo.temperature(100,430.0); // obtener la temperatura de salida
            PID_I.y = out_celsius;
            t_prev_sys = t;

            // mostrar input_pwm, output_celsius y paso de tiempo discreto actual.
            // observar los valores
            Serial.print((String) in_pwm + ", celsius: " + out_celsius +
                    ", countseq: " + countseq);
            Serial.println(F("---\n"));

        }

    }

}
