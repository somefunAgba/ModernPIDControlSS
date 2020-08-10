/*
 *  STEP3: CLOSED-LOOP CONTROL OF THE OUTPUT-TEMPERATURE
 *         USING A PID-CONTROL INPUT TO THE SYSTEM
 */
#include "Arduino.h"

#include "Wire.h"
#include "Adafruit_MCP4725.h"
#include "Adafruit_MAX31865_library/Adafruit_MAX31865.h"

/*PID Library*/
#include "ModernPIDControlSS.h"

/* Global Declarations */

/* timings */
int countseq = 0; // take note of this time-step variable
int max_discrete_time_count = 5000; // vary as desired

double t = 0;
double dt_control = 0.1; // control sampling-time; you can play with this as desired
double dt_sys = 1.0; // equivalent to 1 seconds
double t_prev_sys = -dt_sys;
double t_start = 0;


// set reference output celsius
double ref_celsius = 0;
// input and output variables
int in_pwm;
double in_val;
double out_celsius = 0;

// TODO: initialize other libraries for the DAC and ADC
Adafruit_MCP4725 dac; // Adafruit MCP4725 dac
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10,11,12,13); // Adafruit MAX31865 PT100 adc


/* Parameters PID Structure Config. */
// 10 -> max control input, 0 -> min control input in voltage
PIDNet  PID_I(ref_celsius, out_celsius, dt_control,10, 0);
cplmfc cplmfc_tuner;


void setup() {
    // Turn on Serial Comms.
    // 9600 baud is a compromise setting between easy to kill a runaway process
    // and very slow process
    Serial.begin(9600); // Can Go Up to 2M

    //TODO: setup DAC and ADC
    dac.begin(0x62); // Adafruit MCP4725A1 library
    thermo.readRTD(); // Adafruit MAX31865 library

    /* Parameters PID Structure Config. */
     PID_I.set_bc_follow(0,0,0);

    /* CPLMFC Tuning HyperParameter Config. */
    /* Hyper-Parameter for Kp and Critic Weights */ // for P, I, D
    //TODO: increase alpha steadily to increase fast response;
    //cplmfc_tuner.set_alpha_critics( PID_I, alpha, lambda_i, lambda_d)
    cplmfc_tuner.set_alpha_critics( PID_I,10.0,0.5,0.1);

    /* PID Commonsense Model Config. */
    // note: if the sampling-time is changed
    // rerun the closed-loop settling-time identification
    // to get the correct N_ts and N_tau_l
    // TODO: replace with identified settling-time count from step 2
    int N_ts = 228; // N_ts at 0.1secs;
    // TODO: replace with identified delay-time count
    //  (that is: maximum time-step at which output = 0)
    int N_tau_l = 0;
    //cplmfc_tuner.begin( PID_I, N_ts, N_tau_l);
    cplmfc_tuner.begin( PID_I, N_ts, N_tau_l);

}


void loop() {
    // ensure notion of fixed sampling
    t_start = millis()/1000.0;
    t = t_start;
    while (countseq >= 0) {
        t = t - t_start;
        /* CLOSED-LOOP START */
        if( t >= ( PID_I.T_prev +  PID_I.Ts) - 0.5*( PID_I.Ts)) {
            // Run the PID and its tuning algorithm
            PID_kernelOS( PID_I, cplmfc_tuner, t);
            in_val =  PID_I.u;
            in_pwm = int((4095/10.0)*in_val); // convert input voltage to pwm
        }

        dac.setVoltage(in_pwm,"false"); // pass in max-input value to dac

        /* SYSTEM OUTPUT SAMPLING-: EVERY DT_SYS SECONDS*/
        if ( t >= (t_prev_sys + dt_sys)) {
            out_celsius = thermo.temperature(100,430.0); // get the output temperature
             PID_I.y = out_celsius;
            t_prev_sys = t;
        }


        // display input_pwm, output_celsius and current discrete-time step.
        // observe the values
        Serial.print((String) in_pwm + ", celsius: " + out_celsius +
                ", countseq: " + countseq);
        Serial.println(F("---\n"));

        countseq++;
        t = millis()/1000.0;

    }

}
