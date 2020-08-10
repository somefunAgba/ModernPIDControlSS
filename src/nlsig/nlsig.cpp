
/*
*	nlsig.c
*	Author: Oluwasegun Somefun. oasomefun@futa.edu.ng : 2020
* 	Version: 1.0 Production
*/

/* Include Files */
#include "nlsig.h"

/* Function Definitions */
/*
 * NLOGISTIC-SIGMOID FUNCTION
 *
 *  ARGUMENTS:
 *			y, dy_dx
 *			x,  %inlen
 *			double xmax, ymax -> in max-min limits
 *          double xmin, ymin -> out max-min limits
 *          int n -> logistic sigmoid type,
 *			double lambda -> hyper-parameter, controls rate of growth
 *			char isreverse -> default: 0 logic for forward (0) 
 *						   or reverse (1) sigmoid
 *
 */
void nlsig(double& y, const double& x, double xmax, double xmin, double ymax, double ymin,
			const int n, const double lambda, int safety, const unsigned char isreverse) {

// Obtain length of input:
// size_t insize = (int) (sizeof x - 1) * (sizeof *x);
// int inlen =  insize / (sizeof *x);
// printf("%d\n",inlen); // debug

double e, dy, dx, N, alpha, tau, u;
double delta_i[n], v_i[n];
int c;
	
	N = n;
	if (safety!=0) {
		safety = min(100,max(-100,safety));
		e = safety/100.0;
		// set constraints, max and min
		ymin = (1-e)*ymin;
		ymax = (1-e)*ymax;
		xmin = (1-e)*xmin;
		xmax = (1-e)*xmax;
	}
	
	c = -1;
	if (isreverse){
		c = 1;
	}

	// quantize or partition the input-output space by n.
	// A greater n is a more finer space
	// The most sparse or coarse space is n = 1

	// input-output interval spacing
	dy =(ymax-ymin)/N;
	dx =(xmax-xmin)/N;
    
	y = ymin;
	//dy_dx =0;
    
	// logistic rate
    alpha = lambda * (2/dx);
	// derivative constant
	tau = alpha/dy;

    for (int id = 0; id < n; id++) {
		// inflections
        delta_i[id] = (xmin) + (dx*(id + 0.5)); // id+1-0.5 = id+0.5
		
		// partial output
		u = c*alpha*(x-delta_i[id]);
		v_i[id] = dy/(1+exp(u));
		//v_i[id] = dy/(1+exp_by_ones<double>(u)); // fast approximation
		
		// output and derivative
		y = y + v_i[id];
		//dy_dx = dy_dx + ( v_i[id]*(dy - v_i[id]) );
    }
	
	y = y + 0.0;
	//dy_dx = tau * dy_dx;

    // Serial.print("y: "); Serial.println(y);
	// Serial.print("dydx: "); Serial.println(dy_dx);


	// END
}
