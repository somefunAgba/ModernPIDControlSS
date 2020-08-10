
/*
 * File: CPLMFC.h
 *
 * <CLOSED PID-LOOP MODEL> <FOLLOWING CONTROL> <METHOD> : 2019-2020
 *
 * oasomefun@futa.edu.ng. Copyright.2020
 */

#ifndef CPLMFC_H
#define CPLMFC_H

/* Include Files */
#include <Arduino.h>
#include "pidkernel/PIDNet.h"
#include "cplmfc/filterFO_pass.h"
#include "nlsig/nlsig.h"

class cplmfc{

public:
    void begin(PIDNet&, const int&, const int& = 0);
    void set_alpha_critics(PIDNet&, const float&, const float& = 0.5F, const float& = 0.1F);
    void run(PIDNet&, const double&);

    void tuneWn(PIDNet&);
    void tuneKp(PIDNet&, const double& t, const int& mode);
    void tuneKiKd(PIDNet& Knet) const;

    float wn = 1.0F;
    float alpha = 1.0F;
    double ts = 1.0F;
    float tau_l = 0.0F;
    filterFO_pass filter_r;

};

#endif
/*
 * File trailer for cplmfc.h
 *
 * [EOF]
 */