// Incluir la librería principal
#include <ModernPIDControlSS.h>

// Definición de variables globales
PIDNet pid_controller; // Objeto del controlador PID, se inicializará en setup()

double setpoint = 100.0; // Valor deseado o referencia para el controlador
double proceso_y = 0.0;  // Salida actual del proceso que se está controlando
double control_u = 0.0; // Salida del controlador PID (señal de control)

unsigned long tiempo_anterior = 0; // Variable para manejar el tiempo de muestreo
const double dt = 0.01; // Tiempo de muestreo en segundos (10 ms)

// Parámetros del PID (estos valores se deben ajustar/sintonizar para cada proceso)
double Kp = 2.0; // Ganancia Proporcional
double Ki = 0.5; // Ganancia Integral
double Kd = 0.1; // Ganancia Derivativa

// Parámetros del proceso simulado (Sistema de Primer Orden más Tiempo Muerto - FOPDT)
double K_proceso = 1.0;    // Ganancia estática del proceso simulado
double Tau_proceso = 1.0;  // Constante de tiempo del proceso simulado (en segundos)
const int pasos_delay = 5; // Retardo de tiempo muerto (en número de pasos de muestreo dt)
                           // Un retardo de 5 pasos con dt=0.01s equivale a 50ms de tiempo muerto
double buffer_u[pasos_delay]; // Buffer para simular el retardo de la señal de control
int indice_buffer = 0;     // Índice para manejar el buffer circular del retardo

// Configuración inicial del sistema
void setup() {
  // Inicializar la comunicación serial a 9600 baudios (para depuración y Serial Plotter)
  Serial.begin(9600);

  // Inicializar el objeto del controlador PID
  // PIDNet(r, y, Ts, umax, umin, dead_max, dead_min)
  // r: setpoint inicial (se puede actualizar en loop)
  // y: variable de proceso inicial (se actualiza en loop)
  // Ts: tiempo de muestreo (dt)
  // umax: límite superior de la salida del control (ej. 255 para PWM de Arduino)
  // umin: límite inferior de la salida del control (ej. 0 para PWM de Arduino)
  // dead_max: límite superior de la banda muerta (anti-windup integral)
  // dead_min: límite inferior de la banda muerta (anti-windup integral)
  pid_controller = PIDNet(setpoint, proceso_y, dt, 255.0, 0.0, 0.0, 0.0);

  // Configurar parámetros 2-DOF (Two Degrees of Freedom)
  // set_bc_follow(b, c, follow_mode)
  // b: ponderación del setpoint en la acción proporcional (0 a 1). 1 para P sobre error (SP-y).
  // c: ponderación del setpoint en la acción derivativa (0 a 1). 0 para D sobre -y (evita "derivative kick").
  // follow_mode: 0 para filtro SP desactivado, 1 para filtro SP tipo PT1, 2 para filtro SP tipo PI-PD.
  pid_controller.set_bc_follow(1.0, 0.0, 0); // P sobre error (b=1), D sobre -y (c=0), sin filtro SP

  // Inicializar el buffer de retardo a ceros
  for (int i = 0; i < pasos_delay; i++) {
    buffer_u[i] = 0.0;
  }

  // Imprimir encabezados para el Serial Plotter de Arduino IDE
  // Esto permite visualizar las variables en tiempo real
  Serial.println("Setpoint,Proceso_Y,Control_U");
}

// Bucle principal de control
void loop() {
  // Controlar el tiempo de muestreo para asegurar una ejecución periódica
  if (millis() - tiempo_anterior >= (unsigned long)(dt * 1000)) {
    tiempo_anterior = millis(); // Actualizar el tiempo anterior para el próximo ciclo

    // Actualizar el setpoint y la variable de proceso en el objeto PID
    // (Asegurar que el setpoint esté actualizado si puede cambiar dinámicamente)
    pid_controller.r = setpoint;
    pid_controller.y = proceso_y; // Retroalimentación de la salida del proceso

    // Calculamos la salida del PID
    // PID_kernelOS(controlador, tiempo_actual_sec, Kp, Ki, Kd, Kc_antiwindup)
    // Kc_antiwindup: Ganancia para el anti-windup (0 si se usa dead_max/min en constructor)
    PID_kernelOS(pid_controller, millis() / 1000.0, Kp, Ki, Kd, 0);
    control_u = pid_controller.u; // Obtener la señal de control calculada

    // --- Simulación del Proceso ---
    // Esta sección simula cómo un sistema físico podría responder a la señal de control.

    // 1. Simular el retardo de tiempo muerto (Dead Time)
    // La señal de control 'control_u' no afecta inmediatamente al proceso,
    // sino después de 'pasos_delay' instantes de muestreo.
    double u_retrasado = buffer_u[indice_buffer]; // Obtener la señal de control que realmente afecta al proceso
    buffer_u[indice_buffer] = control_u;         // Almacenar la señal de control actual en el buffer
    indice_buffer = (indice_buffer + 1) % pasos_delay; // Avanzar el índice del buffer circularmente

    // 2. Simular un proceso de primer orden (First-Order System)
    // dy/dt = (K_proceso * u_retrasado - y) / Tau_proceso
    // Usamos el método de Euler para integrar la ecuación diferencial:
    // y_nuevo = y_anterior + dt * (dy/dt)
    double derivada_y = (K_proceso * u_retrasado - proceso_y) / Tau_proceso;
    proceso_y += derivada_y * dt;

    // --- Fin de la Simulación del Proceso ---

    // Enviamos datos al Serial Plotter para visualización
    Serial.print(setpoint);
    Serial.print(",");
    Serial.print(proceso_y);
    Serial.print(",");
    Serial.println(control_u);
  }
}
