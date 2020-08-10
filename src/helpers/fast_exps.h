//
// Created by SomefunAgba on 6/12/2020.
//
#pragma once

#ifndef MODERNPIDCONTROLSS_FAST_EXPS_H
#define MODERNPIDCONTROLSS_FAST_EXPS_H

#include <Arduino.h>



template<class T>
T expbysq(int, int);

template<class T>
T exp_by_ones(T);

template<class T>
T exp_fast(const T&);

inline int is_int_even(const int&);

static inline double exp_fast32(const double& x);
static inline double exp_fast64(const double& x);



/* EVEN INTEGER CHECK*/
inline int is_int_even(const int& x) {
// tests if an integer x is even or odd
// returns: 1 for even x; 0 for odd x.
    return int( ~(x&1) );
}

/* EXP_BY ONES NORMALIZATION OF INPUT */
template<class T>
T exp_by_ones(T x){
	T y =1;

	if (x > 1.0) {
		while( x > 1.0){
			y = y * exp_fast<double>(1.0);
			//y = y * exp_fast32(1.0);
			//y = y * exp_fast64(1.0);
			x = x - 1.0;
		}
	}else if (x < -1.0) {
		while( x > 1.0){
			y = y * exp_fast<double>(-1.0);
			//y = y * exp_fast32(-1.0);
			//y = y * exp_fast64(-1.0);
			x = x + 1.0;
		}
	}
	return ( y * exp_fast<double>(x) );
}

/* EXP_BY SQUARING*/
template<class T>
T expbysq(int x, int n){
	int y = 1;

	if (n < 0) {
		x = 1/x;
		n = -n;
	}else if (n == 0) {
		y = 1;
	} else if (n == 1){
		y = x;
	}
	while (n > 1) {
		if (is_int_even(n)){
			x = x * x;
			n = int( n << 1);
		} else{
			y = x * y;
			x = x * x;
			n = int( (n-1) << 1);
		}
	}
	return T(x * y);
}

/* EXP_APPROXIMATION 1*/

/* LARGE-LIMIT APPROXIMATION*/
template<class T>
T exp_fast(const T& x) {

	int xx;
	xx = 1 + (int(x) >> 16);
	return ( expbysq<double>(xx, (1 << 16 ) ) );

}


/* DOUBLE PRECISION- UNION STRUCTURE APPROXIMATION */
double exp_fast32(const double& x){

    union{
      double _d;
      int32_t _i[2];
    } u{};

//    const int32_t a = 1512775395;
//    const int32_t b_c = 1072693248 - 60801;
//    const double x_scaled = (x)/1000;
    u._i[0] = 0;
    u._i[1] = (1072693248 - 60801) + (1512775)*x;
    return u._d;
}

double exp_fast64(const double& x) {

    union{
      double _d;
      int64_t _i;
    } u{};

    const int64_t a = (int64_t(1) << 52)/0.6931417806;
    const int64_t b_c = (int64_t(1) << 52)*1023 - 60801;
    u._i = b_c + (a)*x;
    return u._d;
}



#endif //MODERNPIDCONTROLSS_FAST_EXPS_H
