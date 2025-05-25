/*
 *  CONTROL DE VELOCIDAD DE MOTOR DC CON ENCODER ROTATIVO
 *  =====================================================
 *
 *  Propósito: Este ejemplo demuestra el control PID de la velocidad (RPM) de un motor DC
 *           utilizando la biblioteca ModernPIDControlSS. La retroalimentación es proporcionada por un
 *           encoder rotativo de cuadratura, y el motor es accionado por una señal PWM
 *           a través de un controlador de motor como el L298N.
 *
 *  Configuración del Hardware:
 *  --------------------------
 *  1. Motor DC con Encoder de Cuadratura:
 *     - Conectar los cables de alimentación del motor a la salida del controlador de motor.
 *     - Encoder VCC a Arduino 5V (o 3.3V, verificar especificaciones del encoder).
 *     - Encoder GND a Arduino GND.
 *     - Encoder Canal A (CHA) a un pin digital con capacidad de interrupción en Arduino (ej., D2).
 *     - Encoder Canal B (CHB) a otro pin digital (ej., D3, interrupción opcional pero útil para dirección).
 *
 *  2. Controlador de Motor (ej., L298N):
 *     - Conectar VCC/Vs del controlador de motor a la fuente de alimentación del motor.
 *     - Conectar GND del controlador de motor a Arduino GND y GND de la fuente de alimentación.
 *     - Conectar VCC Lógica (Vss/5V) del controlador de motor a Arduino 5V.
 *     - Conectar Entrada 1 (IN1) del controlador de motor a un pin digital (ej., D7 para dirección).
 *     - Conectar Entrada 2 (IN2) del controlador de motor a un pin digital (ej., D8 para dirección).
 *     - Conectar Habilitar A (ENA o EN1) del controlador de motor a un pin digital con capacidad PWM (ej., D9 para velocidad).
 *       (Asegurarse que el módulo L298N tenga el jumper removido de ENA si se quiere controlar velocidad vía PWM).
 *
 *  Resumen de Cableado (Pines de Ejemplo para Arduino Uno):
 *  - Motor + : L298N OUT1
 *  - Motor - : L298N OUT2
 *  - Encoder CHA: Arduino D2 (Interrupción 0)
 *  - Encoder CHB: Arduino D3
 *  - Encoder VCC: Arduino 5V
 *  - Encoder GND: Arduino GND
 *
 *  - L298N IN1: Arduino D7 (Control de Dirección del Motor)
 *  - L298N IN2: Arduino D8 (Control de Dirección del Motor)
 *  - L298N ENA (PWM): Arduino D9 (Control de Velocidad del Motor vía PWM)
 *  - L298N Suministro Lógico (Vss/5V): Arduino 5V
 *  - L298N Suministro del Motor (Vs): Fuente de Alimentación Externa del Motor (ej., 6-12V)
 *  - L298N GND: Arduino GND & GND de Fuente de Alimentación del Motor
 *
 *  Lectura del Encoder:
 *  --------------------
 *  Este ejemplo usa una interrupción simple en el Canal A del Encoder para contar pulsos.
 *  Para una detección de velocidad y dirección más robusta, ambos canales A y B serían monitoreados,
 *  típicamente con dos interrupciones o leyendo el canal B dentro de la ISR del canal A.
 *  La implementación actual asume rotación unidireccional o conteo simple de pulsos para velocidad.
 *
 *  Cálculo de RPM:
 *  RPM = (Pulsos por Intervalo / Pulsos por Revolución) * (60 / Segundos del Intervalo)
 *
 *  Guía de Sintonización Manual PID:
 *  -------------------------------
 *  1. Establecer Ki y Kd a 0 inicialmente.
 *  2. Comenzar con un Kp pequeño. Incrementar Kp gradualmente hasta que la velocidad del motor comience a oscilar
 *     alrededor del setpoint o se vuelva agresivamente responsiva. Esta es la "ganancia crítica" (Kc).
 *     Reducir Kp a aproximadamente 0.5 * Kc.
 *  3. Con Kp establecido, comenzar a incrementar Ki para reducir el error de estado estacionario (la diferencia entre
 *     setpoint y RPM real cuando está estable). Si las RPM sobrepasan demasiado o se vuelven lentas,
 *     Ki podría ser demasiado alto.
 *  4. Si es necesario, introducir Kd para amortiguar oscilaciones y mejorar el tiempo de respuesta (reducir sobreimpulso).
 *     Comenzar con un Kd pequeño. Demasiado Kd puede hacer el sistema ruidoso o lento para responder.
 *  5. Iterar: Ajustar Kp, Ki y Kd en pequeños pasos. Es un proceso iterativo.
 *     - Si la respuesta es demasiado lenta: Incrementar Kp.
 *     - Si hay error de estado estacionario: Incrementar Ki.
 *     - Si hay demasiado sobreimpulso u oscilación: Incrementar Kd, o disminuir ligeramente Kp/Ki.
 *
 *  La salida PID en este ejemplo se usa directamente como el ciclo de trabajo PWM (0-255).
 */

#include "Arduino.h"
#include <ModernPIDControlSS.h> // Asumiendo que esta es la ruta correcta de la biblioteca

/* ----- Definiciones de Motor y Encoder (MODIFICAR SEGÚN SEA NECESARIO) ----- */
#define MOTOR_PWM_PIN 9    // Pin PWM para L298N ENA (Control de velocidad)
#define MOTOR_DIR_PIN1 7   // L298N IN1
#define MOTOR_DIR_PIN2 8   // L298N IN2

#define ENCODER_PIN_A 2    // Canal A del Encoder (Debe ser capaz de interrupción, ej., D2 o D3 en Uno)
#define ENCODER_PIN_B 3    // Canal B del Encoder (Puede ser cualquier pin digital si solo se usa CHA para velocidad)

// Pulsos Por Revolución para tu encoder específico
// Esto es crucial para el cálculo preciso de RPM. Revisa la hoja de datos de tu encoder.
#define ENCODER_PPR 300.0 // Ejemplo: 300 pulsos por revolución

/* ----- Configuración del Controlador PID ----- */
double setpoint_rpm = 100.0; // Velocidad deseada del motor (RPM)
double current_rpm = 0.0;   // Velocidad medida del motor (RPM)
double pid_output_pwm = 0.0; // Salida PID, se mapeará al ciclo de trabajo PWM

// Tiempo de muestreo PID (segundos). Ajustar según la dinámica del sistema y capacidad de procesamiento.
// Un muestreo más rápido puede llevar a un mejor control pero requiere más procesamiento.
double dt_control = 0.05; // Intervalo de control de 50 ms

// Ganancias PID - ¡Estas requieren sintonización manual! Ver guía arriba.
// Comenzar con Kp pequeño, Ki=0, Kd=0.
double Kp_manual = 0.8; // Ganancia proporcional
double Ki_manual = 0.5; // Ganancia integral
double Kd_manual = 0.1; // Ganancia derivativa

// Objeto PIDNet: (setpoint, pv_inicial, tiempo_muestreo, umax, umin, dead_max, dead_min)
// umax = 255 (PWM máx), umin = 0 (PWM mín)
// dead_max/min pueden usarse si tu motor tiene una zona de parada, ej., no se mueve bajo cierto PWM.
PIDNet pid_motor(setpoint_rpm, current_rpm, dt_control, 255.0, 0.0, 20, 0); // ejemplo dead_max=20

/* ----- Conteo de Pulsos del Encoder ----- */
volatile long encoder_pulses = 0; // Usar volatile para variables modificadas en ISRs
unsigned long last_rpm_calc_time_ms = 0;
double rpm_calc_interval_ms = 100.0; // Calcular RPM cada 100ms (ajustar según sea necesario)

/* ----- Variables de Temporización y Control ----- */
double t = 0; // Variable de tiempo para PID_kernelOS
unsigned long last_pid_time_ms = 0;

void setup() {
    Serial.begin(115200);
    Serial.println(F("Ejemplo de Control de Velocidad PID de Motor"));
    Serial.println(F("------------------------------------------"));

    // Pines de control del motor
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    pinMode(MOTOR_DIR_PIN1, OUTPUT);
    pinMode(MOTOR_DIR_PIN2, OUTPUT);

    // Establecer dirección inicial del motor (ej., adelante)
    // Para cambiar dirección, intercambiar HIGH/LOW en DIR_PIN1 y DIR_PIN2
    digitalWrite(MOTOR_DIR_PIN1, HIGH);
    digitalWrite(MOTOR_DIR_PIN2, LOW);

    // Pines del encoder
    pinMode(ENCODER_PIN_A, INPUT_PULLUP); // Habilitar pull-up interno para estabilidad
    pinMode(ENCODER_PIN_B, INPUT_PULLUP); // Habilitar pull-up interno

    // Adjuntar interrupción para Canal A del Encoder
    // Llamar a la función 'countPulseA' en flancos ASCENDENTES de ENCODER_PIN_A
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), countPulseA, RISING);

    // Configurar objeto PID (opcional set_bc_follow si es necesario, por defecto usualmente bien para control de velocidad)
    // pid_motor.set_bc_follow(1, 0, 1); // Ejemplo: b=1 (ponderación setpoint P), c=0 (ponderación setpoint D)

    Serial.println(F("Configuración completa. Iniciando lazo de control..."));
    Serial.println(F("Setpoint RPM, RPM Actual, Salida PID (PWM), Kp, Ki, Kd, Tiempo (s)"));

    last_rpm_calc_time_ms = millis();
    last_pid_time_ms = millis();
}

void loop() {
    unsigned long current_time_ms = millis();

    // --- Cálculo de RPM ---
    if (current_time_ms - last_rpm_calc_time_ms >= rpm_calc_interval_ms) {
        long pulses_read;
        // Leer y reiniciar atomicamente encoder_pulses
        noInterrupts(); // Deshabilitar interrupciones mientras se accede a variable compartida
        pulses_read = encoder_pulses;
        encoder_pulses = 0;
        interrupts();   // Rehabilitar interrupciones

        double interval_seconds = (current_time_ms - last_rpm_calc_time_ms) / 1000.0;
        last_rpm_calc_time_ms = current_time_ms;

        // Calcular RPM
        // RPM = (Pulsos / Pulsos Por Revolución) * (Segundos Por Minuto / Segundos del Intervalo)
        current_rpm = ( (double)pulses_read / ENCODER_PPR ) * ( 60.0 / interval_seconds );

        // Actualizar variable de proceso PID
        pid_motor.y = current_rpm;
    }

    // --- Lazo de Control PID ---
    // Ejecutar controlador PID en el intervalo dt_control definido
    if (current_time_ms - last_pid_time_ms >= (unsigned long)(dt_control * 1000.0)) {
        t = current_time_ms / 1000.0; // Tiempo actual en segundos para el kernel PID

        // Actualizar setpoint si es necesario (ej., desde entrada Serial, potenciómetro)
        // Para este ejemplo, setpoint_rpm es fijo pero puede hacerse dinámico.
        pid_motor.r = setpoint_rpm;

        // Llamar al kernel PID con ganancias manuales
        // PID_kernelOS(PIDNet& Knet, const double& t, Kp, Ki, Kd, char flag_dummy)
        char pid_status_flag = PID_kernelOS(pid_motor, t, Kp_manual, Ki_manual, Kd_manual, 0);

        if (pid_status_flag) {
            pid_output_pwm = pid_motor.u;

            // Aplicar salida PID al PWM del motor
            // Asegurar que PWM esté en el rango 0-255 (PIDNet umax/umin debería manejar esto)
            analogWrite(MOTOR_PWM_PIN, (int)pid_output_pwm);

            // Registrar datos
            Serial.print(pid_motor.r, 2); // Setpoint RPM
            Serial.print(F(", "));
            Serial.print(pid_motor.y, 2); // RPM Actual
            Serial.print(F(", "));
            Serial.print(pid_motor.u, 2); // Salida PID (PWM)
            Serial.print(F(", "));
            Serial.print(pid_motor.Kp, 3); // Kp Actual (será Kp_manual)
            Serial.print(F(", "));
            Serial.print(pid_motor.Ki, 3); // Ki Actual
            Serial.print(F(", "));
            Serial.print(pid_motor.Kd, 3); // Kd Actual
            Serial.print(F(", "));
            Serial.print(t, 2);           // Tiempo
            Serial.println();
        }
        last_pid_time_ms = current_time_ms;
    }
}

/*
 * Rutina de Servicio de Interrupción para Canal A del Encoder.
 * Esta función se llama cada vez que se detecta un flanco ASCENDENTE en ENCODER_PIN_A.
 */
void countPulseA() {
    // Conteo básico de pulsos.
    // Para detección de dirección, leerías ENCODER_PIN_B aquí:
    // if (digitalRead(ENCODER_PIN_B) == HIGH) {
    //   encoder_pulses++; // Sentido horario
    // } else {
    //   encoder_pulses--; // Sentido antihorario
    // }
    // Para este ejemplo, incremento simple para cálculo de velocidad.
    encoder_pulses++;
}

/*
 *  Ideas para Desarrollo Futuro:
 *  - Implementar control de dirección basado en señales A y B del Encoder.
 *  - Añadir comandos seriales para cambiar setpoint_rpm o ganancias PID dinámicamente.
 *  - Implementar cálculo de RPM más sofisticado (ej., filtro de media móvil).
 *  - Usar el auto-sintonizador CPLMFC si la sintonización manual resulta difícil, aunque requiere
 *    caracterizar primero el sistema del motor (N_ts, N_tau_l).
 */
```
