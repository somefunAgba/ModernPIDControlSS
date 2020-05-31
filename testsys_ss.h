//
// Created by oaosmefun@futa.edu.ng on 1/16/2020.
//

#pragma once

#ifndef TESTSYS_SS_H
#define TESTSYS_SS_H

//#include <stddef.h>
//#include <stdlib.h>
//#include <stdint.h>

#include <Arduino.h>

/* Func Decls. */
void testsys_ss(double& y, double* x, double dt, double u, double xnoise, double ynoise);
template<class T>
T dead_zone(const double& uin, const int& dead_max, const int& dead_min);

template<class T>
T dead_zone(const double& uin, const int& dead_max, const int& dead_min) {
    T act_in;
/*  Actual Control Input Constraints for u */
/* saturation equivalence of saturation, dead-zone and coulomb friction */
// nl(.) 1-2 . dead-zone, min and inverse dead-zone, max
    if ((fabs(uin)<=fabs(dead_min))) {
// 1. less or at dead-zone (minimum limit)
        act_in = 0;
    }
    else if ((fabs(uin)>fabs(dead_min)) && (fabs(uin)<=fabs(dead_max))) {
// 2. at inverse dead-zone (maximum limit)
        act_in = copysign(dead_max, uin); // if u < 0, u = -deadmax
    }
    else {
// 3. out of inverse dead-zone (max limit)
// added deadmax as disturbance, effect of coulomb friction in a sense.
        act_in = copysign(fabs(uin+dead_max), uin);
    }

    return act_in;
}


#endif //TESTSYS_SS_H
/*
 * File trailer for testsys_ss.h
 *
 * [EOF]
 */

