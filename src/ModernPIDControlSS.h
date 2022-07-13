/**************************************************************************/
/*!
    @file     ModernPIDControlSS.h
    @author   Oluwasegun Somefun (oasomefun@futa.edu.ng, somefuno@oregonstate.edu)

        A PID Library for the Arduino Board

        This is a streamlined PID library specifically made for Arduino use. It
        uses modern control and signal processing theory (algorithms)
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
    @returns flag character.
*/
/**************************************************************************/
inline int PID_kernelOS(PIDNet&, cplmfc&, const double&);
/*!
    @brief Starts the control loop evolution, while passing in manual tuinng params: Kp, Ti, Td
    @param Knet The PID controller instance in the loop
    @param t The current time
    @param Kp proportional gain
    @param Ti integral time constant
    @param Td derivative time constant
    @returns flag character.
*/
inline char PID_kernelOS(PIDNet&, const double&, const float&, const float&, const float&);

/*!
    @brief Starts the control loop evolution, while passing in manual tuinng params: Kp, Ki, Kd
    @param Knet The PID controller instance in the loop
    @param t The current time
    @param Kp proportional gain
    @param Ki integral gain
    @param Kd derivative gain
    @param flag dummy argument: set as 0
    @returns flag character.
*/
inline char PID_kernelOS(PIDNet&, const double&, const float&, const float&, const float&, char flag);

/*
 * Auto. Adaptive CPLMFC
 */
int PID_kernelOS(PIDNet& Knet, cplmfc& Tune, const double& t) {
    int flag = 0;
    if (t >=( (Knet.T_prev+Knet.Ts)-(0.5*Knet.Ts) )) {
        /* CPLM Evolution */
        Tune.run(Knet, t);
        /* PID Control State Evolution: Architecture */
        Knet.compute(t);
        flag = 1;
    }
    return flag;
}

/*
 * Manual I
 */
char PID_kernelOS(PIDNet& Knet, const double& t, const float& Kp, const float& Ti, const float& Td) {
    char flag = 0;
    /* Set Gains Manually */
    Knet.Kp = Kp;
    Knet.Ti = Ti;
    Knet.Td = Td;
    Knet.Ki = Kp/Ti;
    Knet.Kd = Kp*Td;
    if (t >=( (Knet.T_prev+Knet.Ts)-(0.5*Knet.Ts) )) {
        /* PID Control State Evolution: Architecture */
        Knet.compute(t);
        flag = 1;
    }
    return flag;
}

/*
 * Manual II
 */
char PID_kernelOS(PIDNet& Knet, const double& t, const float& Kp, const float& Ki, const float& Kd, char flag) {
    // char flag = 0;
    /* Set Gains Manually */
    Knet.Kp = Kp;
    Knet.Ki = Ki;
    Knet.Kd = Kd;
    Knet.Ti = Kp/Ki;
    Knet.Td = Kd/Kp;
    if (t >=( (Knet.T_prev+Knet.Ts)-(0.5*Knet.Ts) )) {
        /* PID Control State Evolution: Architecture */
        Knet.compute(t);
        flag = 1;
    }
    return flag;
}



#endif //MODERNPIDCONTROLSS_MODERNPIDCONTROLSS_H
