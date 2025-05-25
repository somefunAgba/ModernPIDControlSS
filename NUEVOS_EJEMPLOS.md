# Nuevos Ejemplos y Actualizaciones de Documentación

Este documento proporciona explicaciones para los nuevos ejemplos añadidos a la biblioteca `ModernPIDControlSS` y aclara algunos de los comentarios TODO encontrados en los ejemplos existentes.

## Nuevo Ejemplo: Control de Temperatura con Termistor

**Archivo:** `examples/temperature_control_thermistor/temperature_control_thermistor.ino`

**Propósito:** Este ejemplo demuestra cómo usar la biblioteca `ModernPIDControlSS` para el control de temperatura utilizando un termistor NTC común como sensor de temperatura. Es una adaptación del ejemplo original `temperature_control` que usaba un sensor MAX31865.

**Configuración del Hardware:**

*   **Termistor:** Se asume un termistor NTC de 10k. Conecte una pata del termistor a un pin de entrada analógica (p. ej., A0) en su Arduino.
*   **Divisor de Tensión:** Conecte la otra pata del termistor a una resistencia de 10k Ohm. Conecte el otro extremo de esta resistencia a VCC (5V o 3.3V, dependiendo de su Arduino). La unión entre el termistor y la resistencia (que está conectada al pin analógico) forma el divisor de tensión. Conecte la pata del termistor que también está conectada a la resistencia a GND.
    *   `GND --- [Termistor] --- A0 --- [Resistencia 10k] --- VCC`
*   **Calentador Simulado:** Un LED controlado por un pin PWM (p. ej., pin 9) puede simular el calentador. Conecte el LED en serie con una resistencia limitadora de corriente (p. ej., 220 Ohm) al pin PWM y a GND.
*   **DAC (Opcional, como en el ejemplo original):** Si está utilizando un DAC MCP4725 para la salida de control, conéctelo según su hoja de datos. El código de ejemplo usa `dac.setVoltage()`, lo que implica dicho DAC. Si está controlando directamente el calentador simulado con PWM, necesitará modificar la línea `dac.setVoltage(in_pwm,"false");` para usar `analogWrite(HEATER_PIN, pwm_value);` y escalar `in_val` apropiadamente al rango PWM de 0-255.

**Configuración del Software y Explicación:**

1.  **Detección de Temperatura:**
    *   El ejemplo lee el voltaje analógico del circuito del termistor.
    *   Utiliza la ecuación del parámetro Beta para convertir la lectura analógica a una temperatura en Celsius:
        ```cpp
        // Para Termistor NTC
        #define BETA 3950 // Debería coincidir con su termistor
        #define R_KNOWN 10000 // Resistencia de 10k
        #define T_NOMINAL 25 // Temperatura nominal en C para R_NOMINAL
        #define R_NOMINAL_RESISTANCE 10000 // Resistencia del termistor a T_NOMINAL

        //... en loop() o una función
        int analog_val = analogRead(THERMISTOR_PIN);
        double r_thermistor = R_KNOWN * (1023.0 / (double)analog_val - 1.0);
        double steinhart;
        steinhart = r_thermistor / R_NOMINAL_RESISTANCE;     // (R/Ro)
        steinhart = log(steinhart);                  // ln(R/Ro)
        steinhart /= BETA;                           // 1/B * ln(R/Ro)
        steinhart += 1.0 / (T_NOMINAL + 273.15); // + (1/To)
        steinhart = 1.0 / steinhart;                 // Invertir
        out_celsius = steinhart - 273.15;            // convertir a C
        ```
    *   **Calibración:** El coeficiente `BETA` es específico de su termistor. Es posible que necesite encontrar este valor en su hoja de datos o calibrarlo para una mejor precisión.

2.  **Control PID:**
    *   El ejemplo utiliza el mecanismo de autoajuste CPLMFC (`PID_kernelOS(PID_I, cplmfc_tuner, t);`).
    *   **Adaptación de Parámetros:**
        *   `out_celsius_max`: Este valor (temperatura máxima de salida identificada en el paso 1 de la guía original) probablemente será diferente con una configuración de termistor y su calentador específico. Necesitará volver a ejecutar un paso de identificación del sistema (como un `step_one_max_sysid.ino` adaptado de los ejemplos originales, para su termistor) para encontrar la temperatura máxima estable que su configuración puede alcanzar con la potencia total del calentador.
        *   `N_ts`: El conteo del tiempo de establecimiento en lazo cerrado (identificado en el paso 2 de la guía original) también cambiará. Necesitará volver a ejecutar un experimento similar a `step_two_closedloop_settlingtimeID.ino` (adaptado para su termistor) para determinar el nuevo `N_ts` para su sistema específico basado en termistor. El ejemplo usa un valor provisional; **es crucial actualizar esto para un buen rendimiento.**
        *   `dt_control`: El tiempo de muestreo del control. Asegúrese de que sea apropiado para la respuesta térmica de su sistema.

**Interpretación de Resultados:**
Monitoree la salida serial. Mostrará la entrada PWM al calentador, la temperatura medida (`out_celsius`), y un contador de pasos de tiempo. Observe cómo el controlador PID ajusta la señal PWM para llevar la temperatura al setpoint `ref_celsius` y mantenerla allí.

## Nuevo Ejemplo: Control de Velocidad de Motor DC

**Archivo:** `examples/motor_speed_control/motor_speed_control.ino`

**Propósito:** Este ejemplo demuestra cómo usar la biblioteca `ModernPIDControlSS` para controlar la velocidad (RPM) de un motor DC. Utiliza un codificador rotatorio para la retroalimentación de velocidad y un controlador de motor controlado por PWM (como un L298N).

**Configuración del Hardware:**

*   **Motor DC y Controlador:** Conecte su motor DC a un controlador de motor (p. ej., L298N).
    *   Conecte las entradas de alimentación del controlador de motor (VCC, GND) apropiadamente.
    *   Conecte las entradas de control del controlador de motor a los pines de Arduino. Típicamente, necesitará un pin PWM para el control de velocidad (p. ej., `MOTOR_PWM_PIN`) y uno o dos pines digitales para el control de dirección (p. ej., `MOTOR_DIR_PIN_1`, `MOTOR_DIR_PIN_2`). Este ejemplo asume una dirección o que la dirección se maneja por separado.
*   **Codificador Rotatorio:**
    *   Conecte las salidas de fase A y B del codificador a dos pines digitales con capacidad de interrupción en el Arduino (p. ej., pines 2 y 3 para `ENCODER_PIN_A` y `ENCODER_PIN_B`).
    *   Conecte los pines VCC y GND del codificador.
*   **Ejemplo de Cableado (Conceptual para L298N):**
    *   Pin Arduino 9 (PWM) -> L298N ENA (Habilitar A / Control de Velocidad)
    *   Pin Arduino 8 -> L298N IN1
    *   Pin Arduino 7 -> L298N IN2
    *   Motor conectado a L298N OUT1 y OUT2.
    *   Pin A del Codificador -> Pin Arduino 2 (Pin de Interrupción)
    *   Pin B del Codificador -> Pin Arduino 3 (Pin de Interrupción)

**Configuración del Software y Explicación:**

1.  **Lectura del Codificador:**
    *   El ejemplo utiliza interrupciones (`attachInterrupt`) en los pines del codificador para detectar pulsos.
    *   Una función como `readEncoder()` se llama en cada pulso para incrementar o decrementar un contador de pulsos basado en la dirección de rotación (derivada del estado del pin B cuando el pin A se activa, o viceversa).
    *   Las RPM se calculan periódicamente basándose en el cambio en el conteo de pulsos durante un intervalo de tiempo:
        ```cpp
        // En loop() o una función temporizada
        long current_micros = micros();
        double delta_time_s = (current_micros - prev_micros_rpm) / 1.0e6;
        prev_micros_rpm = current_micros;

        long pulse_diff = encoder_pulses - prev_encoder_pulses;
        prev_encoder_pulses = encoder_pulses;

        current_rpm = (pulse_diff / PULSES_PER_REVOLUTION) / delta_time_s * 60.0;
        ```
    *   `PULSES_PER_REVOLUTION`: Esta constante debe establecerse de acuerdo con la especificación de su codificador.

2.  **Control PID (Ajuste Manual):**
    *   El ejemplo utiliza ajuste manual de PID: `PID_kernelOS(PID_motor, t, Kp, Ki, Kd, flag);`
    *   `PID_motor` es una instancia de `PIDNet`.
    *   `Kp`, `Ki`, `Kd` son las ganancias proporcional, integral y derivativa, respectivamente.
    *   **Guía de Ajuste:**
        *   Comience con `Ki` y `Kd` en 0.
        *   Aumente gradualmente `Kp` hasta que la velocidad del motor comience a oscilar alrededor del setpoint o responda razonablemente rápido.
        *   Si hay error de estado estacionario (las RPM reales no alcanzan las RPM del setpoint), aumente lentamente `Ki` para reducirlo. Demasiado `Ki` puede causar sobreimpulso o inestabilidad.
        *   Si la respuesta es demasiado oscilatoria o tiene demasiado sobreimpulso, intente añadir algo de `Kd` para amortiguarla. `Kd` puede hacer que el sistema sea sensible al ruido, así que úselo con cautela.
        *   Este proceso es iterativo. Ajuste una ganancia a la vez y observe el efecto.

3.  **Control del Motor:**
    *   La salida PID (`PID_motor.u`) se escala y se utiliza para establecer el ciclo de trabajo PWM para el controlador del motor mediante `analogWrite(MOTOR_PWM_PIN, motor_pwm_signal);`.
    *   El control de dirección se simplifica en este ejemplo (se asume fijo o se maneja estableciendo `MOTOR_DIR_PIN_1` y `MOTOR_DIR_PIN_2` apropiadamente antes de iniciar el PID).

**Interpretación de Resultados:**
Monitoree la salida serial, que registra las RPM del setpoint, las RPM reales medidas y la señal PWM aplicada al motor. Experimente con diferentes valores de setpoint y ganancias PID para ver cómo responde el sistema y para lograr el rendimiento de control de velocidad deseado (p. ej., respuesta rápida, mínimo sobreimpulso, buen rechazo de perturbaciones).

## Aclaraciones para TODOs en Ejemplos Existentes

Los ejemplos existentes (especialmente en `examples/temperature_control/` y `examples/simulation_fopdt/`) contienen varios comentarios `TODO`. Aquí se explica cómo abordarlos:

1.  **`//TODO: initialize other libraries for the DAC and ADC`**
    *   **Explicación:** Estos comentarios (p. ej., en `step_one_max_sysid.ino`) le recuerdan que incluya e inicialice las bibliotecas Arduino específicas para cualquier DAC (Convertidor Digital-Analógico, como Adafruit_MCP4725) o componentes relacionados con ADC (Convertidor Analógico-Digital, como el de MAX31865) que esté utilizando su proyecto.
    *   **Acción:** Si está utilizando el hardware exacto mencionado (Adafruit MCP4725, Adafruit MAX31865), el código de ejemplo ya los incluye e inicializa. Si está utilizando hardware diferente, debe reemplazar estos con las inclusiones de biblioteca y llamadas de inicialización correctas para los componentes elegidos.

2.  **`//TODO: fill with max output celsius value identified in step 1.` (p. ej., en `step_two_closedloop_settlingtimeID.ino`)**
    *   **Explicación:** La variable `out_celsius_max` es crucial para algunos escalados internos o cálculos en la configuración del PID, particularmente para el sintonizador CPLMFC.
    *   **Acción:** Antes de ejecutar `step_two_closedloop_settlingtimeID.ino` o `step_three_controlsys.ino`, primero necesita determinar la temperatura máxima de estado estacionario que su sistema de calefacción específico puede alcanzar cuando el calentador funciona a máxima potencia. Esto se hace ejecutando un experimento similar a `step_one_max_sysid.ino`. Anote la temperatura estable más alta y actualice `out_celsius_max` en los archivos de ejemplo subsiguientes con este valor.

3.  **`//TODO: note down the countseq value, where the out_celsius becomes averagely steady. This value becomes the N_ts value for the PID control tuning algorithm` (p. ej., en `step_two_closedloop_settlingtimeID.ino`)**
    *   **Explicación:** `N_ts` representa el tiempo de establecimiento en lazo abierto o cerrado de su sistema, expresado como un número de pasos de tiempo discretos (`countseq`). Este parámetro es crítico para el algoritmo de autoajuste CPLMFC.
    *   **Acción:** Cuando ejecute `step_two_closedloop_settlingtimeID.ino`, observe el trazador serial o los valores registrados de `out_celsius` y `countseq`. Identifique el valor de `countseq` en el que `out_celsius` se establece por primera vez y permanece relativamente estable alrededor del nuevo setpoint después de un cambio de escalón. Este `countseq` es su `N_ts`. Luego debe actualizar la variable `N_ts` en `step_three_controlsys.ino` (u otros ejemplos basados en CPLMFC) con este valor determinado experimentalmente.
    *   **Nota:** `dt_control` (tiempo de muestreo del control) afecta directamente a `N_ts`. Si cambia `dt_control`, *debe* re-determinar `N_ts`.

4.  **`//TODO: increase alpha steadily to increase fast response;` (p. ej., en `step_three_controlsys.ino` para `cplmfc_tuner.set_alpha_critics`)**
    *   **Explicación:** El parámetro `alpha` en `cplmfc_tuner.set_alpha_critics()` influye en la velocidad y agresividad del auto-sintonizador CPLMFC.
    *   **Acción:** Comience con el valor predeterminado o un valor `alpha` pequeño. Si la respuesta del sistema es demasiado lenta, aumente gradualmente `alpha`. Si el sistema se vuelve demasiado oscilatorio o inestable, disminuya `alpha`. Encontrar un buen valor de `alpha` es un proceso de ajuste experimental para el propio algoritmo CPLMFC.

5.  **`// TODO: replace with identified delay-time count (that is: maximum time-step at which output = 0)` (p. ej., en `step_three_controlsys.ino` para `N_tau_l`)**
    *   **Explicación:** `N_tau_l` representa el tiempo muerto o retardo de su sistema, expresado como un número de pasos de tiempo discretos antes de que la salida comience a reaccionar a una entrada. Para muchos sistemas, esto podría ser 0 o muy pequeño.
    *   **Acción:** Durante la identificación de su sistema (como en el paso uno o dos), observe si hay un retardo antes de que la temperatura (u otra variable de proceso) comience a cambiar después de que se aplica la entrada (calentador). Si lo hay, estime cuántos pasos `countseq` dura este retardo. Este es su `N_tau_l`. Si no hay un retardo notable, 0 es apropiado. Este parámetro también lo utiliza el sintonizador CPLMFC.

Al abordar estos TODOs mediante la experimentación y caracterización del sistema, puede mejorar significativamente el rendimiento del controlador PID, especialmente cuando se utiliza la función de autoajuste CPLMFC.
