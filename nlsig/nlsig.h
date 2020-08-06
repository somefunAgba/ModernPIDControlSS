/*
 * File: nlsig.h
 *
 * C/C++ source code
 */
#ifndef NLSIG_H
#define NLSIG_H

/* Include Files */
//#include <stdint.h>
//#include <stddef.h>
//#include <stdlib.h>
#include <Arduino.h>
#include "helpers/fasta_exp.h"

/* Function Declarations */

void nlsig(double& y, double& dy_dx, double x,
		double xmax, double xmin, double ymax, double ymin,
		int n=1, double lambda=6, int safety=0, unsigned char isreverse=0);

#endif

/*
 * File trailer for nlsig.h
 *
 * [EOF]
 */
