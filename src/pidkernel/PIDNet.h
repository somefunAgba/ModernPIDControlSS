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
    explicit PIDNet(double, double, double, int, int, int, int);

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
    double uo;

    int umax;
    int umin;
    int dead_max;
    int dead_min;

    double Kp;
    double Ki;
    double Kd;
    double lambdai;
    double lambdad;
    double Ti;
    double Td;
    int b = 1;
    int c = 0;

    double kpi; // bilinear constant
    double Tf; // first-order filter

    filterFO_pass filter_u;
    double uf;

};

//PIDNet::~PIDNet() = default;
// void sfunPID_kernel(PIDNet& Knet, const double& t);
//#define maxim(a,b)	(((a) > (b)) ? (a) : (b))
//#define minim(a,b)	(((a) < (b)) ? (a) : (b))

/*
 * Dead-zone disturbance simulation
 */
template<class T>
void dead_zone(T&, const int&, const int&);

// expected type is floating-point
template<class T>
void dead_zone(T& uin, const int& dead_max, const int& dead_min) {
/*  Actual Control Input Constraints for u */
/* saturation equivalence of saturation, dead-zone and coulomb friction */
// nl(.) 1-2 . dead-zone, min and inverse dead-zone, max
    if ((fabs(uin)<=fabs(dead_min))) {
// 1. less or at dead-zone (minimum limit)
        uin = 0;
    }
    else if ((fabs(uin)>fabs(dead_min)) && (fabs(uin)<=fabs(dead_max))) {
// 2. at inverse dead-zone (maximum limit)
        uin = copysign(dead_max, uin); // if u < 0, u = -deadmax
    }
    else {
// 3. out of inverse dead-zone (max limit)
// added deadmax as disturbance, effect of coulomb friction in a sense.
        uin = copysign(fabs(uin+dead_max), uin);
    }

}

#endif // SFUNPID_H

/*
  File trailer for sfunPID.h
  [EOF]
*/