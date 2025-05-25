# New Examples and Documentation Updates

This document provides explanations for the new examples added to the `ModernPIDControlSS` library and clarifies some of the TODO comments found in the existing examples.

## New Example: Temperature Control with Thermistor

**File:** `examples/temperature_control_thermistor/temperature_control_thermistor.ino`

**Purpose:** This example demonstrates how to use the `ModernPIDControlSS` library for temperature control using a common NTC thermistor as the temperature sensor. It is an adaptation of the original `temperature_control` example that used a MAX31865 sensor.

**Hardware Setup:**

*   **Thermistor:** A 10k NTC thermistor is assumed. Connect one leg of the thermistor to an analog input pin (e.g., A0) on your Arduino.
*   **Voltage Divider:** Connect the other leg of the thermistor to a 10k Ohm resistor. Connect the other end of this resistor to VCC (5V or 3.3V, depending on your Arduino). The junction between the thermistor and the resistor (which is connected to the analog pin) forms the voltage divider. Connect the thermistor's leg that's also connected to the resistor to GND.
    *   `GND --- [Thermistor] --- A0 --- [10k Resistor] --- VCC`
*   **Heater Proxy:** An LED controlled by a PWM pin (e.g., pin 9) can simulate the heater. Connect the LED in series with a current-limiting resistor (e.g., 220 Ohm) to the PWM pin and GND.
*   **DAC (Optional, as in original example):** If you are using an MCP4725 DAC for the control output, connect it as per its datasheet. The example code uses `dac.setVoltage()` which implies such a DAC. If you are directly driving the heater proxy with PWM, you'll need to modify the line `dac.setVoltage(in_pwm,"false");` to use `analogWrite(HEATER_PIN, pwm_value);` and scale `in_val` appropriately to the 0-255 PWM range.

**Software Setup & Explanation:**

1.  **Temperature Sensing:**
    *   The example reads the analog voltage from the thermistor circuit.
    *   It uses the Beta-parameter equation to convert the analog reading to a temperature in Celsius:
        ```cpp
        // For NTC Thermistor
        #define BETA 3950 // Should match your thermistor
        #define R_KNOWN 10000 // 10k resistor
        #define T_NOMINAL 25 // Nominal temperature in C for R_NOMINAL
        #define R_NOMINAL_RESISTANCE 10000 // Resistance of thermistor at T_NOMINAL

        //... in loop() or a function
        int analog_val = analogRead(THERMISTOR_PIN);
        double r_thermistor = R_KNOWN * (1023.0 / (double)analog_val - 1.0);
        double steinhart;
        steinhart = r_thermistor / R_NOMINAL_RESISTANCE;     // (R/Ro)
        steinhart = log(steinhart);                  // ln(R/Ro)
        steinhart /= BETA;                           // 1/B * ln(R/Ro)
        steinhart += 1.0 / (T_NOMINAL + 273.15); // + (1/To)
        steinhart = 1.0 / steinhart;                 // Invert
        out_celsius = steinhart - 273.15;            // convert to C
        ```
    *   **Calibration:** The `BETA` coefficient is specific to your thermistor. You might need to find this value in its datasheet or calibrate it for better accuracy.

2.  **PID Control:**
    *   The example uses the CPLMFC auto-tuning mechanism (`PID_kernelOS(PID_I, cplmfc_tuner, t);`).
    *   **Adapting Parameters:**
        *   `out_celsius_max`: This value (max output temperature identified in step 1 of the original guide) will likely be different with a thermistor setup and your specific heater. You'll need to re-run a system identification step (like `step_one_max_sysid.ino` from the original examples, adapted for your thermistor) to find the maximum stable temperature your setup can achieve with full heater power.
        *   `N_ts`: The closed-loop settling time count (identified in step 2 of the original guide) will also change. You'll need to re-run an experiment similar to `step_two_closedloop_settlingtimeID.ino` (adapted for your thermistor) to determine the new `N_ts` for your specific thermistor-based system. The example uses a placeholder value; **it's crucial to update this for good performance.**
        *   `dt_control`: The control sampling time. Ensure this is appropriate for your system's thermal response.

**Interpreting Results:**
Monitor the serial output. It will show the PWM input to the heater, the measured temperature (`out_celsius`), and a time-step counter. Observe how the PID controller adjusts the PWM signal to bring the temperature to the `ref_celsius` setpoint and maintain it there.

## New Example: DC Motor Speed Control

**File:** `examples/motor_speed_control/motor_speed_control.ino`

**Purpose:** This example demonstrates how to use the `ModernPIDControlSS` library to control the speed (RPM) of a DC motor. It uses a rotary encoder for speed feedback and a PWM-controlled motor driver (like an L298N).

**Hardware Setup:**

*   **DC Motor & Driver:** Connect your DC motor to a motor driver (e.g., L298N).
    *   Connect the motor driver's power inputs (VCC, GND) appropriately.
    *   Connect the motor driver's control inputs to Arduino pins. Typically, you'll need one PWM pin for speed control (e.g., `MOTOR_PWM_PIN`) and one or two digital pins for direction control (e.g., `MOTOR_DIR_PIN_1`, `MOTOR_DIR_PIN_2`). This example assumes one direction or that direction is handled separately.
*   **Rotary Encoder:**
    *   Connect the encoder's A and B phase outputs to two interrupt-capable digital pins on the Arduino (e.g., pins 2 and 3 for `ENCODER_PIN_A` and `ENCODER_PIN_B`).
    *   Connect the encoder's VCC and GND pins.
*   **Wiring Example (Conceptual for L298N):**
    *   Arduino Pin 9 (PWM) -> L298N ENA (Enable A / Speed Control)
    *   Arduino Pin 8 -> L298N IN1
    *   Arduino Pin 7 -> L298N IN2
    *   Motor connected to L298N OUT1 and OUT2.
    *   Encoder Pin A -> Arduino Pin 2 (Interrupt Pin)
    *   Encoder Pin B -> Arduino Pin 3 (Interrupt Pin)

**Software Setup & Explanation:**

1.  **Encoder Reading:**
    *   The example uses interrupts (`attachInterrupt`) on the encoder pins to detect pulses.
    *   A function like `readEncoder()` is called on each pulse to increment or decrement a pulse counter based on the rotation direction (derived from the state of pin B when pin A triggers, or vice-versa).
    *   RPM is calculated periodically based on the change in pulse count over a time interval:
        ```cpp
        // In loop() or a timed function
        long current_micros = micros();
        double delta_time_s = (current_micros - prev_micros_rpm) / 1.0e6;
        prev_micros_rpm = current_micros;

        long pulse_diff = encoder_pulses - prev_encoder_pulses;
        prev_encoder_pulses = encoder_pulses;

        current_rpm = (pulse_diff / PULSES_PER_REVOLUTION) / delta_time_s * 60.0;
        ```
    *   `PULSES_PER_REVOLUTION`: This constant must be set according to your encoder's specification.

2.  **PID Control (Manual Tuning):**
    *   The example uses manual PID tuning: `PID_kernelOS(PID_motor, t, Kp, Ki, Kd, flag);`
    *   `PID_motor` is an instance of `PIDNet`.
    *   `Kp`, `Ki`, `Kd` are the proportional, integral, and derivative gains, respectively.
    *   **Tuning Guidance:**
        *   Start with `Ki` and `Kd` set to 0.
        *   Gradually increase `Kp` until the motor speed starts to oscillate around the setpoint or responds reasonably quickly.
        *   If there's steady-state error (actual RPM doesn't reach setpoint RPM), slowly increase `Ki` to reduce it. Too much `Ki` can cause overshoot or instability.
        *   If the response is too oscillatory or overshoots too much, try adding some `Kd` to dampen it. `Kd` can make the system sensitive to noise, so use it cautiously.
        *   This process is iterative. Adjust one gain at a time and observe the effect.

3.  **Motor Control:**
    *   The PID output (`PID_motor.u`) is scaled and used to set the PWM duty cycle for the motor driver via `analogWrite(MOTOR_PWM_PIN, motor_pwm_signal);`.
    *   Direction control is simplified in this example (assumed fixed or handled by setting `MOTOR_DIR_PIN_1` and `MOTOR_DIR_PIN_2` appropriately before starting the PID).

**Interpreting Results:**
Monitor the serial output, which logs the setpoint RPM, actual measured RPM, and the PWM signal applied to the motor. Experiment with different setpoints and PID gain values to see how the system responds and to achieve desired performance (e.g., fast response, minimal overshoot, good disturbance rejection).

## Clarifications for TODOs in Existing Examples

The existing examples (especially in `examples/temperature_control/` and `examples/simulation_fopdt/`) contain several `TODO` comments. Here's how to address them:

1.  **`//TODO: initialize other libraries for the DAC and ADC`**
    *   **Explanation:** These comments (e.g., in `step_one_max_sysid.ino`) remind you to include and initialize the specific Arduino libraries for the DAC (Digital-to-Analog Converter, like Adafruit_MCP4725) and ADC (Analog-to-Digital Converter, like the one for MAX31865) you are using.
    *   **Action:** If you are using the exact hardware mentioned (Adafruit MCP4725, Adafruit MAX31865), the example code already includes and initializes them. If you are using different hardware, you must replace these with the correct library includes and initialization calls for your chosen components.

2.  **`//TODO: fill with max output celsius value identified in step 1.` (e.g., in `step_two_closedloop_settlingtimeID.ino`)**
    *   **Explanation:** The variable `out_celsius_max` is crucial for some internal scaling or calculations in the PID setup, particularly for the CPLMFC tuner.
    *   **Action:** Before running `step_two_closedloop_settlingtimeID.ino` or `step_three_controlsys.ino`, you first need to determine the maximum steady-state temperature your specific heating system can achieve when the heater is run at maximum power. This is done by running an experiment similar to `step_one_max_sysid.ino`. Note the highest stable temperature and update `out_celsius_max` in the subsequent example files with this value.

3.  **`//TODO: note down the countseq value, where the out_celsius becomes averagely steady. This value becomes the N_ts value for the PID control tuning algorithm` (e.g., in `step_two_closedloop_settlingtimeID.ino`)**
    *   **Explanation:** `N_ts` represents the open-loop or closed-loop settling time of your system, expressed as a number of discrete time steps (`countseq`). This parameter is critical for the CPLMFC auto-tuning algorithm.
    *   **Action:** When running `step_two_closedloop_settlingtimeID.ino`, observe the serial plotter or logged `out_celsius` and `countseq`. Identify the `countseq` value at which the `out_celsius` first settles and remains relatively stable around the new setpoint after a step change. This `countseq` is your `N_ts`. You must then update the `N_ts` variable in `step_three_controlsys.ino` (or other CPLMFC-based examples) with this experimentally determined value.
    *   **Note:** `dt_control` (control sampling time) directly affects `N_ts`. If you change `dt_control`, you *must* re-determine `N_ts`.

4.  **`//TODO: increase alpha steadily to increase fast response;` (e.g., in `step_three_controlsys.ino` for `cplmfc_tuner.set_alpha_critics`)**
    *   **Explanation:** The `alpha` parameter in `cplmfc_tuner.set_alpha_critics()` influences the speed and aggressiveness of the CPLMFC auto-tuner.
    *   **Action:** Start with the default or a small `alpha` value. If the system response is too slow, gradually increase `alpha`. If the system becomes too oscillatory or unstable, decrease `alpha`. Finding a good `alpha` value is an experimental tuning process for the CPLMFC algorithm itself.

5.  **`// TODO: replace with identified delay-time count (that is: maximum time-step at which output = 0)` (e.g., in `step_three_controlsys.ino` for `N_tau_l`)**
    *   **Explanation:** `N_tau_l` represents the system's dead time or delay, expressed as a number of discrete time steps before the output starts to react to an input. For many systems, this might be 0 or very small.
    *   **Action:** During your system identification (like in step one or two), observe if there's a delay before the temperature (or other process variable) starts changing after the input (heater) is applied. If there is, estimate how many `countseq` steps this delay lasts. This is your `N_tau_l`. If there's no noticeable delay, 0 is appropriate. This parameter is also used by the CPLMFC tuner.

By addressing these TODOs through experimentation and system characterization, you can significantly improve the performance of the PID controller, especially when using the CPLMFC auto-tuning feature.
