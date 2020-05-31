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

/* Data Structure-> Class Decl.*/
/* class PIDNet*/
class PIDNet {
public:
    char follow;
    double Ts;
    double T_prev;
    int countseq;

    double r;
    double y;
    double ym;
    double xm[2]= {0.0, 0.0};
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

    PIDNet(double ref, double yout, double Tsample,
                int umax_lim, int umin_lim);

    //virtual ~PIDNet();

    friend void sfunPID_kernel(PIDNet& Knet, const double& t);
};

//PIDNet::~PIDNet() = default;
void sfunPID_kernel(PIDNet& Knet, const double& t);
//#define maxim(a,b)	(((a) > (b)) ? (a) : (b))
//#define minim(a,b)	(((a) < (b)) ? (a) : (b))

#endif // SFUNPID_H

/*
  File trailer for sfunPID.h
  [EOF]
*/