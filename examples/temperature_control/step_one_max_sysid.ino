/*
 *  PASO1: IDENTIFICAR LA SALIDA MÁXIMA EN ESTADO ESTACIONARIO DEL SISTEMA
 *         DADA LA ENTRADA DE CONTROL MÁXIMA AL SISTEMA
 */
#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_MCP4725.h"
#include "Adafruit_MAX31865_library/Adafruit_MAX31865.h"

/* Declaraciones Globales */

/* temporización */
int countseq = 0; // tomar nota de esta variable de paso de tiempo
double t_current = 0;
int max_discrete_time_count = 5000; // variar según se desee
int dt = 1000; // equivalente a 1 segundo

// establecer entrada de control máx.
const int max_in = 10;

// TODO: inicializar otras bibliotecas para el DAC y ADC
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

// variables de entrada y salida
double in_pwm = 0;
double out_celsius = 0;

void loop() {
    // asegurar noción de muestreo fijo
    t_current = 0;

    while (countseq >= 0) {
        /* ID SISTEMA SALIDA-MÁX - INICIO */
        /* registrar la salida cada dt milisegundos  */
        if ( millis() - t_current >= dt) {
            in_pwm = (4095/10.0)*max_in;
            dac.setVoltage(in_pwm,"false"); // pasar valor de entrada máx. al dac
            out_celsius = thermo.temperature(100,430.0); // obtener la temperatura de salida
            t_current = millis();
        }

        // mostrar input_pwm, output_celsius y paso de tiempo discreto actual.
        // TODO: anotar el valor máximo
        //  donde out_celsius se vuelve estable en promedio
        Serial.print((String) in_pwm + ", celsius: " + out_celsius +
                    ", countseq: " + countseq);
        Serial.println(F("---\n"));

        if (countseq >= max_discrete_time_count) {
            countseq = -1;
            break;
        }
        countseq++;

    }

}
