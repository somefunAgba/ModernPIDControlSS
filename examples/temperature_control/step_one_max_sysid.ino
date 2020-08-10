/*
 *  STEP1: IDENTIFY THE MAXIMUM STEADY STATE OUTPUT OF THE SYSTEM
 *         GIVEN THE MAXIMUM CONTROL INPUT TO THE SYSTEM
 */
#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_MCP4725.h"
#include "Adafruit_MAX31865.h"

/* Global Declarations */

/* timings */
int countseq = 0; // take note of this time-step variable
double t_current = 0;
int max_discrete_time_count = 5000; // vary as desired
int dt = 1000; // equivalent to 1 seconds

// set max. control input
const int umax = 4095;

// TODO: initialize other libraries for the DAC and ADC
Adafruit_MCP4725 dac; // Adafruit MCP4725 dac
thermo = Adafruit_MAX31865(10,11,12,13); // Adafruit MAX31865 PT100 adc

void setup() {
    // Turn on Serial Comms.
    // 9600 baud is a compromise setting between easy to kill a runaway process
    // and very slow process
    Serial.begin(9600); // Can Go Up to 2M

    //TODO: setup DAC and ADC
    dac.begin(0x62); // Adafruit MCP4725A1 library
    thermo.readRTD(); // Adafruit MAX31865 library
}

// input and output variables
double in_pwm = 0;
double out_celsius = 0;

void loop() {
    // ensure notion of fixed sampling
    t_current = 0;

    while (countseq >= 0) {
        /* MAX-OUTPUT SYS ID- START */
        /* log the output every dt milli-secs  */
        if ( millis() - t_current >= dt) {
            in_pwm = (umax/10.0)*10;
            dac.setvoltage(in_pwm,"false"); // pass in max-input value to dac
            out_celsius = thermo.temperature(100,430.0); // get the output temperature
            t_current = millis();
        }

        // display input_pwm, output_celsius and current discrete-time step.
        // TODO: note down the maximum value
        //  where the out_celsius becomes averagely steady
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
