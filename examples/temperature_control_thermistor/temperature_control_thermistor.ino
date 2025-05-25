/*
 *  CONTROL DE TEMPERATURA CON TERMISTOR NTC
 *  ========================================
 *
 *  Propósito: Este ejemplo demuestra el control de temperatura en lazo cerrado
 *           utilizando la biblioteca ModernPIDControlSS con un termistor NTC común
 *           como sensor de temperatura y un calentador simulado (por ejemplo, un LED
 *           controlado por PWM).
 *
 *  Este ejemplo está adaptado de 'examples/temperature_control/step_three_controlsys.ino'
 *  y se centra en reemplazar el sensor PT100 MAX31865 por un termistor NTC
 *  y un divisor de tensión.
 *
 *  Configuración del Hardware:
 *  -------------------------
 *  1. Termistor NTC (ej., 10k Ohm a 25°C):
 *     Conectar una pata del termistor a un Pin Analógico (ej., A0).
 *     Conectar la otra pata del termistor a GND.
 *
 *  2. Resistencia del Divisor de Tensión (ej., resistencia fija de 10k Ohm):
 *     Conectar una pata de esta resistencia al mismo Pin Analógico (A0) que el termistor.
 *     Conectar la otra pata de esta resistencia a 5V (o 3.3V, dependiendo del VCC de tu Arduino).
 *
 *     Diagrama:
 *       5V (o 3.3V) --- [Resistencia Fija (R_fixed)] --- Pin A0 --- [Termistor NTC (R_ntc)] --- GND
 *
 *  3. Calentador Simulado (LED):
 *     Conectar el ánodo (pata más larga) de un LED a un pin digital PWM (ej., D9).
 *     Conectar el cátodo (pata más corta) del LED a una resistencia limitadora de corriente (ej., 220-330 Ohm).
 *     Conectar el otro extremo de la resistencia a GND.
 *
 *  4. (Opcional) DAC para Control del Calentador:
 *     Si tienes un DAC MCP4725, puedes usarlo para un control más fino del voltaje del calentador.
 *     Conectar SDA a A4, SCL a A5, VCC a 5V, GND a GND.
 *     La salida del DAC entonces manejaría la base de un transistor que controla el elemento calentador real.
 *     Este ejemplo usa PWM directamente por simplicidad, asumiendo un simple LED como proxy del calentador.
 *
 *  Medición de Temperatura:
 *  ------------------------
 *  El código utiliza la ecuación del Parámetro Beta para termistores NTC, que es más simple
 *  que Steinhart-Hart pero a menudo suficiente.
 *
 *  Fórmula: 1/T = 1/T0 + (1/B) * ln(R/R0)
 *    T: Temperatura en Kelvin
 *    T0: Temperatura nominal en Kelvin (ej., 25°C = 298.15 K)
 *    B: Coeficiente Beta del termistor (ej., 3950 K) - revisa la hoja de datos de tu termistor.
 *    R: Resistencia medida del termistor.
 *    R0: Resistencia nominal del termistor a T0 (ej., 10k Ohms).
 *
 *  Para calibrar:
 *  1. Encuentra R0 y T0 en la hoja de datos de tu termistor (ej., 10kOhm a 25°C).
 *  2. Encuentra el coeficiente Beta (B) en la hoja de datos.
 *  3. Si los valores de la hoja de datos no son suficientemente precisos, puede que necesites calibrar midiendo
 *     la resistencia del termistor a unas pocas temperaturas conocidas y ajustando el valor de B.
 *
 *  Adaptación de N_ts (Tiempo de Establecimiento para el Sintonizador CPLMFC):
 *  --------------------------------------------------------------------
 *  El parámetro `N_ts` para `cplmfc_tuner.begin()` es el tiempo de establecimiento en lazo abierto del sistema
 *  en términos del número de muestras de control (`dt_control`).
 *  Si cambias el sensor, el calentador o el sistema físico que se está calentando, probablemente
 *  necesitarás re-determinar `N_ts`.
 *  Esto se puede hacer mediante:
 *  1. Aplicando una entrada escalón al calentador (ej., establecer PWM a un valor fijo).
 *  2. Registrando la temperatura hasta que se estabilice.
 *  3. Contando cuántos períodos `dt_control` tomó para establecerse.
 *  Un proceso similar se aplica para `N_tau_l` (tiempo muerto).
 *
 *  El concepto `out_celsius_max` del ejemplo original (aunque no explícitamente una variable allí)
 *  se relaciona con la temperatura máxima esperada o el rango operativo. Esto ayuda a escalar
 *  los esfuerzos de control o a establecer setpoints realistas. Con un nuevo sensor/sistema, este rango podría diferir.
 */

#include "Arduino.h"
// #include "Wire.h" // Solo si se usan dispositivos I2C como MCP4725
// #include "Adafruit_MCP4725.h" // Solo si se usa el DAC MCP4725

/* Archivos de Inclusión de la Biblioteca PID */
#include <ModernPIDControlSS.h>

/* Declaraciones Globales */

/* ----- Parámetros del Termistor (MODIFICA ESTOS PARA TU TERMISTOR) ----- */
#define THERMISTOR_PIN A0     // Pin analógico para el termistor
#define FIXED_RESISTOR 10000.0 // Resistencia de la resistencia fija en el divisor de tensión (Ohms)
#define NOMINAL_RESISTANCE 10000.0 // Resistencia nominal del termistor NTC a temperatura nominal (Ohms)
#define NOMINAL_TEMPERATURE_C 25.0 // Temperatura nominal para el termistor NTC (Celsius)
#define BETA_COEFFICIENT 3950.0    // Coeficiente Beta del termistor NTC (Kelvin)
#define ADC_MAX 1023.0        // Resolución del ADC (1023 para ADC de 10 bits en Arduino Uno)
#define VOLTAGE_SUPPLY 5.0    // VCC del Arduino (5V o 3.3V)
/* -------------------------------------------------------------------- */

/* ----- Parámetros del Calentador (Simulado por LED) ----- */
#define HEATER_PWM_PIN 9     // Pin PWM para controlar el LED (proxy del calentador)
// Si se usa DAC:
// Adafruit_MCP4725 dac;
/* ------------------------------------------------ */

/* Información de Temporización */
int countseq = 0; // Variable de paso de tiempo
double t = 0;
double dt_control = 0.2; // Tiempo de muestreo del control (segundos). Se puede ajustar.
                         // Un poco más largo que el ejemplo MAX31865 debido a la estabilidad de analogRead.
double dt_sys_sense = 0.2; // Intervalo de detección de la salida del sistema (segundos)
double t_prev_sys_sense = -dt_sys_sense;
double t_start = 0;

/* Configuración del Controlador PID */
double ref_celsius = 35.0; // Temperatura de setpoint deseada (Celsius)
double out_celsius = 20.0; // Temperatura inicial/medida (Celsius)

// Para salida PWM, umax puede ser 255. Para DAC, podría ser voltaje (ej., 5.0V) o valor DAC (4095).
// Usaremos valores PWM directamente para el rango de salida del PID por simplicidad con el LED.
// umax: Valor PWM máximo (calentador completamente encendido)
// umin: Valor PWM mínimo (calentador completamente apagado)
// dead_max, dead_min: Zona muerta para la salida de control (puede ser 0 si no se necesita)
PIDNet PID_thermistor(ref_celsius, out_celsius, dt_control,
                      255.0, 0.0, 0, 0); // PWM Máx es 255, Mín es 0
cplmfc cplmfc_tuner;

void setup() {
    Serial.begin(115200); // Usar una tasa de baudios más rápida para un registro de datos más veloz
    pinMode(THERMISTOR_PIN, INPUT);
    pinMode(HEATER_PWM_PIN, OUTPUT);

    // Si se usa DAC:
    // dac.begin(0x62); // Dirección I2C por defecto para MCP4725

    Serial.println(F("Ejemplo de Control de Temperatura con Termistor NTC"));
    Serial.println(F("--------------------------------------------------"));

    /* Config. de Parámetros de la Estructura PID */
    // b: ponderación del setpoint para el término proporcional (0 o 1)
    // c: ponderación del setpoint para el término derivativo (0 o 1)
    // follow: modo anti-windup (0 o 1)
    PID_thermistor.set_bc_follow(1, 0, 1); // Configuraciones comunes: b=1, c=0 o 1

    /* Config. de Hiperparámetros de Ajuste CPLMFC */
    // alpha: tasa de aprendizaje para el crítico Kp (ajustar para responsividad)
    // lambda_i, lambda_d: tasas de aprendizaje para los críticos Ki, Kd (típicamente 0.1-0.5)
    // Aumentar alpha para una respuesta más rápida, pero puede llevar a inestabilidad.
    // Estos valores son puntos de partida y pueden necesitar ajuste.
    cplmfc_tuner.set_alpha_critics(PID_thermistor, 15.0, 0.5, 0.1); // Alpha ajustado

    /* Config. del Modelo de Sentido Común PID para el Sintonizador CPLMFC */
    // N_ts: Tiempo de establecimiento en lazo abierto en número de muestras dt_control.
    //       Esto es CRÍTICO y dependiente del sistema.
    //       DEBES estimar esto para tu configuración específica (potencia del calentador, masa térmica).
    //       Ejemplo: Si el sistema se establece en 30 segundos, y dt_control es 0.2s,
    //       N_ts = 30 / 0.2 = 150.
    // N_tau_l: Tiempo muerto en lazo abierto (retraso) en número de muestras dt_control.
    //          Tiempo desde que se aplica entrada al calentador hasta que la temperatura *comienza* a cambiar.
    int N_ts = 150;    // Valor provisional - ¡ESTIMA PARA TU SISTEMA!
    int N_tau_l = 5;   // Valor provisional - ¡ESTIMA PARA TU SISTEMA!
    cplmfc_tuner.begin(PID_thermistor, N_ts, N_tau_l);

    Serial.println(F("Configuración completa. Iniciando lazo de control..."));
    Serial.println(F("Setpoint, Temp Real (C), Salida PID (PWM), Kp, Ki, Kd, Tiempo (s)"));

    // Inicializar temperatura medida
    PID_thermistor.y = readTemperatureNTC();
    out_celsius = PID_thermistor.y;
}

char pid_flag; // Bandera de PID_kernelOS

void loop() {
    t_start = millis() / 1000.0; // Tiempo actual en segundos
                                 // Este uso de t_start es un poco diferente al original,
                                 // buscando un tiempo absoluto para t.
    
    // El ejemplo original tenía un bucle `while(countseq >=0)` aquí.
    // Para sketches típicos de Arduino, la función `loop()` en sí misma es el bucle continuo.
    // Usaremos `millis()` para la temporización para asegurar que `dt_control` se respete aproximadamente.

    // Calcular tiempo para PID
    // Nota: La estructura original del ejemplo para 't' basada en sustracciones dentro del bucle
    // puede ser complicada. Usar tiempo absoluto de millis() y verificar intervalos es a menudo más claro.
    // Sin embargo, PID_kernelOS espera que 't' sea un tiempo evolutivo, a menudo comenzando desde 0 para una ejecución.
    // Por simplicidad en la adaptación, reiniciaremos 't' conceptualmente para cada ciclo PID,
    // pero el PID mismo usa t para su estado interno relativo a su llamada anterior.
    // Un enfoque común es t = (millis() - tiempo_inicio_inicial) / 1000.0;
    // Usemos un t relativo más simple para PID_kernelOS como estaba en el ejemplo original,
    // asegurando que PID_kernelOS se llame a intervalos dt_control.

    // Esta verificación asegura que PID_kernelOS se llame aproximadamente cada dt_control segundos
    if ( (millis()/1000.0 - PID_thermistor.T_prev) >= PID_thermistor.Ts ) {
        // El 't' pasado a PID_kernelOS idealmente debería ser el tiempo total transcurrido
        // para el proceso de control.
        t = millis()/1000.0; // Pasar tiempo actual en segundos

        /* --- Ejecutar Controlador PID y Sintonizador CPLMFC --- */
        pid_flag = PID_kernelOS(PID_thermistor, cplmfc_tuner, t);

        if (pid_flag) { // Si el PID calculó (lo cual debería basándose en la verificación de tiempo externa)
            // PID_thermistor.u contiene la nueva salida de control (0-255 para PWM)
            analogWrite(HEATER_PWM_PIN, (int)PID_thermistor.u);

            // El ejemplo original incrementaba countseq aquí.
            // Podemos usarlo como un contador general para los cómputos PID.
            countseq++;
            pid_flag = 0; // Reiniciar bandera
        }
    }


    /* --- Detección de Salida del Sistema (Lectura del Termistor) --- */
    // Esto también debería ocurrir a intervalos regulares, no necesariamente los mismos que el PID.
    // Por simplicidad, usamos dt_sys_sense, que puede ser igual a dt_control.
    if ( (millis()/1000.0 - t_prev_sys_sense) >= dt_sys_sense ) {
        out_celsius = readTemperatureNTC();
        PID_thermistor.y = out_celsius; // Actualizar PID con nueva variable de proceso
        t_prev_sys_sense = millis()/1000.0;

        /* --- Registro Serial --- */
        Serial.print(PID_thermistor.r, 2); // Setpoint
        Serial.print(F(", "));
        Serial.print(PID_thermistor.y, 2); // Temperatura Actual
        Serial.print(F(", "));
        Serial.print(PID_thermistor.u, 2); // Salida PID (PWM)
        Serial.print(F(", "));
        Serial.print(PID_thermistor.Kp, 3);
        Serial.print(F(", "));
        Serial.print(PID_thermistor.Ki, 3);
        Serial.print(F(", "));
        Serial.print(PID_thermistor.Kd, 3);
        Serial.print(F(", "));
        Serial.print(t, 2); // Tiempo actual
        Serial.println();
    }

    // Opcional: Añadir un retardo para evitar que el bucle se ejecute demasiado rápido si no se usan acciones temporizadas
    // delay(10); // Pequeño retardo
}

/*
 * Función para leer la temperatura de un termistor NTC usando la ecuación del Parámetro Beta.
 */
double readTemperatureNTC() {
    int adc_val = analogRead(THERMISTOR_PIN);

    // Calcular la resistencia del termistor
    // Vout = Vin * R_ntc / (R_fixed + R_ntc)
    // adc_val / ADC_MAX = R_ntc / (R_fixed + R_ntc)  (asumiendo que Vin es el voltaje de referencia del ADC)
    // Sea ratio = adc_val / ADC_MAX
    // ratio * (R_fixed + R_ntc) = R_ntc
    // ratio * R_fixed + ratio * R_ntc = R_ntc
    // ratio * R_fixed = R_ntc * (1 - ratio)
    // R_ntc = (ratio * R_fixed) / (1 - ratio)
    // R_ntc = R_fixed * ( (ADC_MAX / adc_val) - 1 )  -- Esto es para un divisor donde el termistor está entre A0 y GND
    // Si el termistor está entre VCC y A0, y R_fixed está de A0 a GND:
    // Vout = Vin * R_fixed / (R_ntc + R_fixed)
    // R_ntc = R_fixed * (Vin / Vout - 1)
    // R_ntc = R_fixed * ( (VOLTAGE_SUPPLY / (adc_val * VOLTAGE_SUPPLY / ADC_MAX)) - 1)
    // R_ntc = R_fixed * (ADC_MAX / adc_val - 1) -- Esta fórmula es correcta para la configuración descrita:
    // 5V --- [Resistencia Fija (R_fixed)] --- Pin A0 --- [Termistor NTC (R_ntc)] --- GND
    // Entonces V_A0 = VCC * R_ntc / (R_fixed + R_ntc)
    // adc_val / ADC_MAX = R_ntc / (R_fixed + R_ntc)
    // (adc_val / ADC_MAX) * (R_fixed + R_ntc) = R_ntc
    // (adc_val / ADC_MAX) * R_fixed = R_ntc * (1 - adc_val / ADC_MAX)
    // R_ntc = R_fixed * (adc_val / ADC_MAX) / (1 - adc_val / ADC_MAX)
    // R_ntc = R_fixed * adc_val / (ADC_MAX - adc_val)

    // Verificar si adc_val = ADC_MAX para prevenir división por cero
    if (adc_val >= ADC_MAX -1) { // La resistencia del termistor es muy alta (frío) o está abierto
      return -100; // O algún indicador de error/temperatura muy baja
    }
    if (adc_val <= 1) { // La resistencia del termistor es muy baja (caliente) o está en cortocircuito
        return 500; // O algún indicador de error/temperatura muy alta
    }

    double R_ntc = FIXED_RESISTOR * adc_val / (ADC_MAX - adc_val);

    // Ecuación de Steinhart-Hart: 1/T = A + B*ln(R) + C*(ln(R))^3
    // La ecuación del Parámetro Beta es más simple: 1/T = 1/T0 + (1/B) * ln(R/R0)
    double steinhart;
    steinhart = R_ntc / NOMINAL_RESISTANCE;     // (R/R0)
    steinhart = log(steinhart);                // ln(R/R0)
    steinhart /= BETA_COEFFICIENT;             // (1/B) * ln(R/R0)
    steinhart += 1.0 / (NOMINAL_TEMPERATURE_C + 273.15); // + (1/T0)
    steinhart = 1.0 / steinhart;               // T en Kelvin
    steinhart -= 273.15;                       // Convertir a Celsius

    // Suavizado básico (opcional, se puede mejorar con filtros más avanzados)
    static double smoothed_temp = -1000; // Inicializar a una temperatura improbable
    if (smoothed_temp == -1000) {
      smoothed_temp = steinhart;
    } else {
      smoothed_temp = smoothed_temp * 0.8 + steinhart * 0.2; // Filtro IIR simple
    }
    return smoothed_temp;
}

// TODO: Añadir funciones para identificación del sistema (respuesta al escalón) si se desea,
// similar a cómo step_one y step_two podrían haber sido estructurados en la suite de ejemplos original.
// Para este ejemplo, asumimos que N_ts y N_tau_l están predeterminados o estimados.
```
