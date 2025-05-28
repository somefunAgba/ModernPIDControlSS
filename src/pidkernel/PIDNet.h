/*
  Archivo: sfunPID.h
 
//  Autor: oasomefun@futa.edu.ng            : 2019
 
 */


//#include <stddef.h> // Código comentado
//#include <stdlib.h> // Código comentado
//#include <stdint.h> // Código comentado

#ifndef SFUNPID_H
#define SFUNPID_H

#include <Arduino.h>
#include "cplmfc/filterFO_pass.h"

/* Estructura de Datos-> Declaración de Clase.*/
/* clase PIDNet*/
class PIDNet {
public:
    explicit PIDNet(double, double, double, int, int, int, int);

    // friend void sfunPID_kernel(PIDNet& Knet, const double& t); // Código comentado
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

    double kpi; // constante bilineal
    double Tf; // filtro de primer orden

    filterFO_pass filter_u;
    double uf;

};

//PIDNet::~PIDNet() = default; // Código comentado
// void sfunPID_kernel(PIDNet& Knet, const double& t); // Código comentado
//#define maxim(a,b)	(((a) > (b)) ? (a) : (b)) // Código comentado
//#define minim(a,b)	(((a) < (b)) ? (a) : (b)) // Código comentado

/*
 * Simulación de perturbación de zona muerta
 */
template<class T>
void dead_zone(T&, const int&, const int&);

// el tipo esperado es punto flotante
template<class T>
void dead_zone(T& uin, const int& dead_max, const int& dead_min) {
/*  Restricciones Reales de Entrada de Control para u */
/* equivalencia de saturación, zona muerta y fricción de Coulomb */
// nl(.) 1-2 . zona muerta, mín y zona muerta inversa, máx
    if ((fabs(uin)<=fabs(dead_min))) {
// 1. menor o en la zona muerta (límite mínimo)
        uin = 0;
    }
    else if ((fabs(uin)>fabs(dead_min)) && (fabs(uin)<=fabs(dead_max))) {
// 2. en la zona muerta inversa (límite máximo)
        uin = copysign(dead_max, uin); // si u < 0, u = -deadmax
    }
    else {
// 3. fuera de la zona muerta inversa (límite máx)
// deadmax añadido como perturbación, efecto de la fricción de Coulomb en cierto sentido.
        uin = copysign(fabs(uin+dead_max), uin);
    }

}

#endif // SFUNPID_H

/*
  Trailer de archivo para sfunPID.h
  [EOF]
*/