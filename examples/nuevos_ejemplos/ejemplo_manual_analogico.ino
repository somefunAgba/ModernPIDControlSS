// Incluir la librería principal
#include <ModernPIDControlSS.h>

// Definición de pines
const int pin_sensor = A0;       // Pin para la entrada analógica del sensor
const int pin_actuador_pwm = 3;  // Pin PWM para la salida del actuador

// Definición de variables globales
PIDNet pid_controller; // Objeto del controlador PID, se inicializará en setup()

double setpoint = 100.0; // Valor deseado o referencia para el controlador (ej. 0-255 si se mapea desde sensor)
double proceso_y = 0.0;  // Salida actual del proceso (leída desde el sensor, ej. 0-255)
double control_u = 0.0; // Salida del controlador PID (señal de control, ej. 0-255 para PWM)

unsigned long tiempo_anterior = 0; // Variable para manejar el tiempo de muestreo
const double dt = 0.01; // Tiempo de muestreo en segundos (10 ms)

// Parámetros del PID (estos valores se deben ajustar/sintonizar para cada proceso)
double Kp = 2.0; // Ganancia Proporcional
double Ki = 0.5; // Ganancia Integral
double Kd = 0.1; // Ganancia Derivativa

// Configuración inicial del sistema
void setup() {
  // Inicializar la comunicación serial a 9600 baudios (para depuración y Serial Plotter)
  Serial.begin(9600);

  // Configurar pines
  pinMode(pin_sensor, INPUT);
  pinMode(pin_actuador_pwm, OUTPUT);

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

  // Imprimir encabezados para el Serial Plotter de Arduino IDE
  // Esto permite visualizar las variables en tiempo real
  Serial.println("Setpoint,Proceso_Y,Control_U");
}

// Bucle principal de control
void loop() {
  // Controlar el tiempo de muestreo para asegurar una ejecución periódica
  if (millis() - tiempo_anterior >= (unsigned long)(dt * 1000)) {
    tiempo_anterior = millis(); // Actualizar el tiempo anterior para el próximo ciclo

    // --- Adquisición de la Variable de Proceso (Entrada Analógica) ---
    // Leer la variable de proceso desde un sensor analógico (ej. A0)
    int valor_sensor = analogRead(pin_sensor);
    // Mapear el valor del sensor (0-1023) a un rango esperado para 'proceso_y' (ej. 0-255)
    // Ajustar este mapeo según el sensor y el rango deseado para el proceso.
    proceso_y = map(valor_sensor, 0, 1023, 0, 255);

    // --- INICIO: Simulación para pruebas sin hardware ---
    // Si no hay hardware conectado, descomenta la siguiente línea para una simulación simple
    // donde la salida del proceso sigue directamente a la señal de control (no realista, solo para pruebas).
    // Es útil para verificar el comportamiento básico del PID sin un sistema físico.
    // proceso_y = control_u; 
    // --- FIN: Simulación para pruebas sin hardware ---

    // Actualizar el setpoint y la variable de proceso en el objeto PID
    // (Asegurar que el setpoint esté actualizado si puede cambiar dinámicamente)
    pid_controller.r = setpoint;
    pid_controller.y = proceso_y; // Retroalimentación de la salida del proceso (leída del sensor)

    // Calculamos la salida del PID
    // PID_kernelOS(controlador, tiempo_actual_sec, Kp, Ki, Kd, Kc_antiwindup)
    // Kc_antiwindup: Ganancia para el anti-windup (0 si se usa dead_max/min en constructor)
    PID_kernelOS(pid_controller, millis() / 1000.0, Kp, Ki, Kd, 0);
    control_u = pid_controller.u; // Obtener la señal de control calculada

    // --- Aplicación de la Señal de Control (Salida PWM) ---
    // Escribir la señal de control a un pin PWM (ej. pin 3)
    // Asegurarse que control_u está en el rango PWM (0-255)
    // La función 'constrain' limita el valor de 'control_u' al rango [0, 255]
    analogWrite(pin_actuador_pwm, (int)constrain(control_u, 0, 255));

    // Enviamos datos al Serial Plotter para visualización
    Serial.print(setpoint);
    Serial.print(",");
    Serial.print(proceso_y);
    Serial.print(",");
    Serial.println(control_u);
  }
}
