//
// Created by oasomefun@futa.edu.ng on 1/16/2020.
//
#pragma once

#ifndef CPLM_KERNEL_H
#define CPLM_KERNEL_H

#include "Arduino.h"

/* Function Declarations */
void cplm_kernel(double& ym, double* xm, double r, double Ts, int b, int c,
                 double wn);

#endif //
