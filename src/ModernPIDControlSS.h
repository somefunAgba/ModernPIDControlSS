/**************************************************************************/
/*!
    @file     ModernPIDControlSS.h
    @author   Oluwasegun Somefun (oasomefun@futa.edu.ng)

        A PID Library for the Arduino Board

        This is a PID library specifically made for Arduino use. It
        uses modern control and signal processing algorithms
        ----> http://github.com/somefunagba/ModernPIDControlSS

        This library is invested time and resources,
        please support by sharing and starring on GitHub!

        @section  HISTORY

*/
/**************************************************************************/
#pragma once

#ifndef MODERNPIDCONTROLSS_MODERNPIDCONTROLSS_H
#define MODERNPIDCONTROLSS_MODERNPIDCONTROLSS_H

#include "pidkernel/PIDNet.h"
#include "cplmfc/filterFO_pass.h"
#include "cplmfc/cplmfc.h"
#include "dynsys/testsys_ss.h"
#include "helpers/norm++kernel.h"

/**************************************************************************/
/*!
    @brief  Starts the control and tuning loop evolution
    @param Knet The PID controller instance in the loop
    @param Tune The CPLMFC Tuning Algorithm Instance for this PID Controller
    @param t The current time
    @returns void.
*/
/**************************************************************************/
inline void PID_kernelOS(PIDNet&, cplmfc&, const double&);

inline void PID_kernelOS(PIDNet& Knet, cplmfc& Tune, const double& t) {
    /* CPLM Evolution */
    Tune.run(Knet, t);
    /* PID Control State Evolution: Architecture */
    Knet.compute(t);
}

#endif //MODERNPIDCONTROLSS_MODERNPIDCONTROLSS_H
