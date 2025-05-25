/*
 *  PASO2: IDENTIFICAR EL TIEMPO DE ESTABLECIMIENTO EN LAZO CERRADO DE LA SALIDA
 *         DADA UNA ENTRADA DE CONTROL DE ERROR AL SISTEMA
 */
#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_MCP4725.h"
#include <Adafruit_MAX31865_library/Adafruit_MAX31865.h>

/* Declaraciones Globales */

/* temporización */
int countseq = 0; // tomar nota de esta variable de paso de tiempo
int max_discrete_time_count = 5000; // variar según se desee

double t = 0;
double dt_control = 0.1; // tiempo de muestreo del control; puedes jugar con esto como desees
double t_prev_control = -dt_control;
double dt_sys = 1.0; // equivalente to 1 segundo
double t_prev_sys = -dt_sys;
double t_start = 0;

// establecer entrada de control máx.
const int in_max = 10; // 10V, equivalente a 4095 PWM
// establecer salida de referencia en celsius
double ref_celsius = 0;

// variables de entrada y salida
int in_pwm;
double in_val;
double out_celsius = 0;
//TODO: rellenar con el valor máximo de celsius de salida identificado en el paso 1.
double out_celsius_max = 100; // cambiar esto al valor correcto
// TODO: inicializar bibliotecas para el DAC y ADC, si no son dac y adc de Adafruit
Adafruit_MCP4725 dac; // DAC Adafruit MCP4725
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10,11,12,13); // ADC Adafruit MAX31865 PT100

void setup() {
    // Activar Comunicaciones Seriales.
    // 9600 baudios es una configuración de compromiso entre fácil de detener un proceso fuera de control
    // y un proceso muy lento
    Serial.begin(9600); // Puede ir hasta 2M

    //TODO: configurar DAC y ADC
    dac.begin(0x62); // Biblioteca Adafruit MCP4725A1
    thermo.readRTD(); // Biblioteca Adafruit MAX31865

}


void loop() {
    // asegurar noción de muestreo fijo
    t_start = millis()/1000.0;
    t = t_start;
    while (countseq >= 0) {

        t = t - t_start;
        /* ID SISTEMA N_TS EN LAZO CERRADO - INICIO */
        if( t >= (t_prev_control + dt_control) - 0.5*(dt_control)) {
            if (countseq==0) {
                double k = 1; // puede variar entre 0.5 y 1
                ref_celsius = k*out_celsius_max;
                out_celsius = out_celsius_max;
            }
            in_val = (in_max/out_celsius_max)*(ref_celsius-out_celsius);
            in_pwm = int((4095/10.0)*in_val); // convertir voltaje de entrada a pwm
            t_prev_control = t;
        }

        dac.setVoltage(in_pwm,"false"); // pasar valor de entrada máx. al dac

        /* MUESTREO DE SALIDA DEL SISTEMA-: CADA DT_SYS SEGUNDOS*/
        if ( t >= (t_prev_sys + dt_sys)) {
            out_celsius = thermo.temperature(100,430.0); // obtener la temperatura de salida
            t_prev_sys = t;
        }

        // mostrar input_pwm, output_celsius y paso de tiempo discreto actual.
        //TODO: anotar el valor de countseq,
        //  donde out_celsius se vuelve estable en promedio.
        //  Este valor se convierte en el valor N_ts para el algoritmo de ajuste del control PID
        Serial.print((String) in_pwm + ", celsius: " + out_celsius +
                ", countseq: " + countseq);
        Serial.println(F("---\n"));

        if (countseq >= max_discrete_time_count) {
            countseq = -1;
            break;
        }
        countseq++;
        t = millis()/1000.0;

    }

}
