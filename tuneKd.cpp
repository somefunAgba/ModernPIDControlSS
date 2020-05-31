//
// Created by SomefunAgba on 5/28/2020.
//

#include "cpidlmfc.h"
#include "nlsig.h"

void tuneKd(PIDNet& Knet, double wn) {
    /* derivative time constant*/
    Knet.Td = 2.0F/(0.7071F*wn);
    /* derivative gain*/
    Knet.Kd = Knet.Kp*Knet.Td;
}