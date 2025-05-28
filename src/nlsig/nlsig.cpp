
/*
*	nlsig.c
*	Autor: Oluwasegun Somefun. oasomefun@futa.edu.ng : 2020
* 	Versión: 1.0 Producción
*/

/* Archivos de Inclusión */
#include "nlsig.h"

/* Definiciones de Funciones */
/*
 * FUNCIÓN SIGMOIDE N-LOGÍSTICA
 *
 *  ARGUMENTOS:
 *			y, dy_dx
 *			x,  %inlen
 *			double xmax, ymax -> límites max-min de entrada
 *          double xmin, ymin -> límites max-min de salida
 *          int n -> tipo de sigmoide logística,
 *			double lambda -> hiperparámetro, controla la tasa de crecimiento
 *			char isreverse -> por defecto: lógica 0 para sigmoide directa (0) 
 *						   o inversa (1)
 *
 */
void nlsig(double& y, const double& x, double xmax, double xmin, double ymax, double ymin,
			const int n, const double lambda, int safety, const unsigned char isreverse) {

// Obtener longitud de la entrada:
// size_t insize = (int) (sizeof x - 1) * (sizeof *x); // Código comentado
// int inlen =  insize / (sizeof *x); // Código comentado
// printf("%d\n",inlen); // depuración

double e, dy, dx, N, alpha, tau, u;
double delta_i[n], v_i[n];
int c;
	
	N = n;
	if (safety!=0) {
		safety = min(100,max(-100,safety));
		e = safety/100.0;
		// establecer restricciones, max y min
		ymin = (1-e)*ymin;
		ymax = (1-e)*ymax;
		xmin = (1-e)*xmin;
		xmax = (1-e)*xmax;
	}
	
	c = -1;
	if (isreverse){
		c = 1;
	}

	// cuantizar o particionar el espacio de entrada-salida por n.
	// Un n mayor es un espacio más fino
	// El espacio más disperso o grueso es n = 1

	// espaciado del intervalo de entrada-salida
	dy =(ymax-ymin)/N;
	dx =(xmax-xmin)/N;
    
	y = ymin;
	//dy_dx =0; // Código comentado
    
	// tasa logística
    alpha = lambda * (2/dx);
	// constante derivativa
	tau = alpha/dy;

    for (int id = 0; id < n; id++) {
		// inflexiones
        delta_i[id] = (xmin) + (dx*(id + 0.5)); // id+1-0.5 = id+0.5
		
		// salida parcial
		u = c*alpha*(x-delta_i[id]);
		v_i[id] = dy/(1+exp(u));
		//v_i[id] = dy/(1+exp_by_ones<double>(u)); // aproximación rápida
		
		// salida y derivada
		y = y + v_i[id];
		//dy_dx = dy_dx + ( v_i[id]*(dy - v_i[id]) ); // Código comentado
    }
	
	y = y + 0.0;
	//dy_dx = tau * dy_dx; // Código comentado

    // Serial.print("y: "); Serial.println(y); // Código comentado
	// Serial.print("dydx: "); Serial.println(dy_dx); // Código comentado


	// FIN
}
