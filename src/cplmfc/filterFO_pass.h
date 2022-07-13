//
// Created by oasomefun@futa.edu.ng on 1/16/2020.
//
#ifndef FILTERFO_PASS_H
#define FILTERFO_PASS_H

#include "Arduino.h"

#ifndef TAN_ST_C
#define TAN_ST_C
/**
 * Shanon's discrete-time
 * Bilinear pre-warping tan constant
 */
inline constexpr double TAN_ST = 0.1583844403;
#endif

/* Class Declarations */
class filterFO_pass{
public:
    explicit filterFO_pass();
    double x;
    double Tf_kpi;
    void run(double&, double);
};

#endif //
