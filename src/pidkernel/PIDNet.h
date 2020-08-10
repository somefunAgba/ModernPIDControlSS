/*
  File: sfunPID.h
 
//  Author: oasomefun@futa.edu.ng            : 2019
 
 */


//#include <stddef.h>
//#include <stdlib.h>
//#include <stdint.h>

#ifndef SFUNPID_H
#define SFUNPID_H

#include <Arduino.h>
#include "cplmfc/filterFO_pass.h"

/* Data Structure-> Class Decl.*/
/* class PIDNet*/
class PIDNet {
public:
    explicit PIDNet(double, double, double, int, int);

    // friend void sfunPID_kernel(PIDNet& Knet, const double& t);
    void compute(const double&);
    void set_bc_follow(const int&, const int&, const char&);

    char follow;
    double Ts;
    double T_prev;
    int countseq;

    double r;
    double y;
    double ym;
    double e;
    double ei;
    double ed;
    double e_t;

    double up;
    double ui;
    double ud;
    double upd;
    double ua;
    double v;
    double u;

    signed int umax;
    signed int umin;

    double Kp;
    double Ki;
    double Kd;
    double lambdai;
    double lambdad;
    double Ti;
    double Td;
    int b = 1;
    int c = 0;
    double Tf;

    filterFO_pass filter_u;
    double uf;

};

//PIDNet::~PIDNet() = default;
// void sfunPID_kernel(PIDNet& Knet, const double& t);
//#define maxim(a,b)	(((a) > (b)) ? (a) : (b))
//#define minim(a,b)	(((a) < (b)) ? (a) : (b))

#endif // SFUNPID_H

/*
  File trailer for sfunPID.h
  [EOF]
*/