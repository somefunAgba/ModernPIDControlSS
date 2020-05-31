
/*
*	nlsig.c
*	Author: oasomefun@futa.edu.ng : 2019
* 	Version: 1.0 Production
*/

/* Include Files */
#include "nlsig.h"

/* Function Definitions */
/*
 * n-logistic sigmoid function min->max
 *
 *  ARGS
 *  INPUTS:  x, inlen
 *			double xmax, ymax -> i/o maximum limits
 *          double xmin, ymin -> i/o minimum limits
 *          int n -> logistic sigmoid type,
 *			double exp_tuner -> hyper-parameter, controls rate of growth
 *			int reverse -> default: 0 logic for forward (0) 
 						   or reverse (1) sigmoid
 * RETURN OUTPUT: double y
 */
double nlsig(double x, double xmax, double xmin, double ymax, double ymin,
			unsigned int n, double exp_tuner, unsigned char reverse) {

double y, safety = 0; // 0.005; % 5% safety
// set constraints, max and min
ymin = (1.0-safety)*ymin;
ymax = (1.0-safety)*ymax;

// Obtain length of input:
// size_t insize = (int) (sizeof x - 1) * (sizeof *x);
// int inlen =  insize / (sizeof *x);
// printf("%d\n",inlen); // debug

/* N Logistic Sigmoid */
    double partydiff; partydiff = (ymax-ymin)/((double)n);
    double partxdiff; partxdiff = (xmax-xmin)/((double)n);
    
	// unsigned int q; = n + 1; // length of parts et. al.
    double ky[n+1];
    double kx[n+1];
    
    ky[0] = ymin;
    kx[0] = xmin;
    for (int id = 0; id < n; id++) {
    	ky[id] = ky[id] + partydiff;
        kx[id] = kx[id] + partxdiff;
	}
       
    double alpha = exp_tuner * (2.0 / partxdiff);
    double delta_i[n];
    for (int id = 0; id < n; id++) {
        delta_i[id]  = 0.5 * (kx[id+1]+kx[id]);
    }

    // v_i
    double v_i[n];
    for (int id = 0; id < n; id++) {
        if (!reverse){ 
        	v_i[id] =  partydiff / (1.0 + exp(-alpha*(x - delta_i[id])));
        } else {
        	v_i[id] =  partydiff / (1.0 + exp(alpha*(x - delta_i[id])));
        }
    }

    y = ky[0];
	for (int id = 0; id < n; id++) {
		y = y + v_i[id];
	}
    
    return y;

	// END
}
