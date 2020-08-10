//
// Created by oasomefun@futa.edu.ng on 1/23/2020. Updated: August, 2020
//
#pragma once

#ifndef MODERNPIDCONTROLSS_MODERNPIDCONTROLSS_H
#define MODERNPIDCONTROLSS_MODERNPIDCONTROLSS_H

#include "pidkernel/PIDNet.h"
#include "cplmfc/filterFO_pass.h"
#include "cplmfc/cplmfc.h"
#include "dynsys/testsys_ss.h"
#include "helpers/norm++kernel.h"


inline void PID_kernelOS(PIDNet&, cplmfc&, const double&);

inline void PID_kernelOS(PIDNet& Knet, cplmfc& Tune, const double& t) {
    /* CPLM Evolution */
    Tune.run(Knet, t);
    /* PID Control State Evolution: Architecture */
    Knet.compute(t);
}

#endif //MODERNPIDCONTROLSS_MODERNPIDCONTROLSS_H
