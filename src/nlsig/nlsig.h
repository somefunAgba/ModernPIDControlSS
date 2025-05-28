/*
 * Archivo: nlsig.h
 *
 * Código fuente C/C++
 */
#ifndef NLSIG_H
#define NLSIG_H

/* Archivos de Inclusión */
//#include <stdint.h> // Código comentado
//#include <stddef.h> // Código comentado
//#include <stdlib.h> // Código comentado
#include <Arduino.h>
#include "helpers/fast_exps.h"

/* Declaraciones de Funciones */

//void nlsig(double& y, double& dy_dx, double x, // Código comentado
//		double xmax, double xmin, double ymax, double ymin, // Código comentado
//		int n=1, double lambda=6, int safety=0, unsigned char isreverse=0); // Código comentado

void nlsig(double& y, const double& x,
        double xmax, double xmin, double ymax, double ymin,
        int n=1, double lambda=6, int safety=0, unsigned char isreverse=0);
#endif

/*
 * Trailer de archivo para nlsig.h
 *
 * [EOF]
 */
