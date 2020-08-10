//
// Created by oasomefun@futa.edu.ng on 1/16/2020.
//
#ifndef FILTERFO_PASS_H
#define FILTERFO_PASS_H

#include "Arduino.h"

/* Class Declarations */
class filterFO_pass{
public:
    explicit filterFO_pass();
    double x;
    double Tf_kpi;
    void run(double&, double);
};

#endif //
