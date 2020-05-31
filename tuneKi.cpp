//
// Created by SomefunAgba on 5/28/2020.
//

#include "cpidlmfc.h"
#include "nlsig.h"

void tuneKi(PIDNet& Knet, double wn) {
    /* integral time constant*/
    Knet.Ti = (2.0*0.7071)/wn;
    /* integral gain*/
    Knet.Ki = Knet.Kp/Knet.Ti;
}