
/*
 * File: CPIDLMFC.h
 *
 * <CLOSED PID-LOOP MODEL> <FOLLOWING CONTROL> <METHOD> : 2019-2020
 *
 * oasomefun@futa.edu.ng. Copyright.2020
 */

#ifndef CPIDLMFC_H
#define CPIDLMFC_H

/* Include Files */
#include <Arduino.h>
#include "pidkernel/sfunPID.h"

/* Function Declarations */
void tuneWn(PIDNet& Knet, double ts, double& wn);

void tuneKp(PIDNet& Knet, double t, double L, double ts, double alpha, unsigned int mode);

void tuneKi(PIDNet& Knet, double wn);
					
void tuneKd(PIDNet& Knet, double wn);


#endif

/*
 * File trailer for cpidlmfc.h
 *
 * [EOF]
 */

#pragma clang diagnostic pop