/*
 *  STEP2: IDENTIFY THE CLOSED-LOOP SETTLING-TIME OF THE OUTPUT
 *         GIVEN AN ERROR CONTROL INPUT TO THE SYSTEM
 */
#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_MCP4725.h"
#include <Adafruit_MAX31865_library/Adafruit_MAX31865.h>

/* Global Declarations */

/* timings */
int countseq = 0; // take note of this time-step variable
int max_discrete_time_count = 5000; // vary as desired

double t = 0;
double dt_control = 0.1; // control sampling-time; you can play with this as desired
double t_prev_control = -dt_control;
double dt_sys = 1.0; // equivalent to 1 seconds
double t_prev_sys = -dt_sys;
double t_start = 0;

// set max. control input
const int in_max = 10; // 10V, equivalent to 4095 PWM
// set reference output celsius
double ref_celsius = 0;

// input and output variables
int in_pwm;
double in_val;
double out_celsius = 0;
//TODO: fill with max output celsius value identified in step 1.
double out_celsius_max = 100; // change this to the correct value
// TODO: initialize libraries for the DAC and ADC, if not Adafruit dac and adc
Adafruit_MCP4725 dac; // Adafruit MCP4725 dac
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10,11,12,13); // Adafruit MAX31865 PT100 adc

void setup() {
    // Turn on Serial Comms.
    // 9600 baud is a compromise setting between easy to kill a runaway process
    // and very slow process
    Serial.begin(9600); // Can Go Up to 2M

    //TODO: setup DAC and ADC
    dac.begin(0x62); // Adafruit MCP4725A1 library
    thermo.readRTD(); // Adafruit MAX31865 library

}


void loop() {
    // ensure notion of fixed sampling
    t_start = millis()/1000.0;
    t = t_start;
    while (countseq >= 0) {

        t = t - t_start;
        /* CLOSED-LOOP N_TS SYS ID- START */
        if( t >= (t_prev_control + dt_control) - 0.5*(dt_control)) {
            if (countseq==0) {
                double k = 1; // can vary between 0.5 and 1
                ref_celsius = k*out_celsius_max;
                out_celsius = out_celsius_max;
            }
            in_val = (in_max/out_celsius_max)*(ref_celsius-out_celsius);
            in_pwm = int((4095/10.0)*in_val); // convert input voltage to pwm
            t_prev_control = t;
        }

        dac.setVoltage(in_pwm,"false"); // pass in max-input value to dac

        /* SYSTEM OUTPUT SAMPLING-: EVERY DT_SYS SECONDS*/
        if ( t >= (t_prev_sys + dt_sys)) {
            out_celsius = thermo.temperature(100,430.0); // get the output temperature
            t_prev_sys = t;
        }

        // display input_pwm, output_celsius and current discrete-time step.
        //TODO: note down the countseq value,
        //  where the out_celsius becomes averagely steady.
        //  This value becomes the N_ts value for the PID control tuning algorithm
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
