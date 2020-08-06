//
// Created by SomefunAgba on 5/28/2020.
//
#include "cpidlmfc.h"

void tuneWn(PIDNet& Knet, double ts, double& wn) {
    /*  tuning algorithm functions */
    double xtsn;
    int xx = Knet.b+Knet.c;
    if (xx==0) {
        xtsn = 9.98;
//        Serial.println(xtsn);
    }
    else if (xx==1) {
        xtsn = 7.74;
//        Serial.println(xtsn);
    }
    else if (xx==2) {
        xtsn = 11.07;
//        Serial.println(xtsn);
    }
    else {
        xtsn = 8;
    }

    /*  normalized w_n */
    wn = xtsn/(0.7071*ts);
//    Serial.println(xtsn);
    //printf("wn: %lf\n", wn); //debug

}