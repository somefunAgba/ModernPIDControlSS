//
// Created by SomefunAgba on 6/13/2020.
//

#ifndef MODERNPIDCONTROLSS_NORMS++_KERNEL_H
#define MODERNPIDCONTROLSS_NORMS++_KERNEL_H

//
// File: norms++kernel.h
// C/C++ source code created on  : 13-Jun-2020 09:37:47
//

// Include Files
// change headers for other projects, if not Arduino.
#include <Arduino.h>

// MIN-MAX MEDIAN (DE)-NORMALIZATION INTERVAL FUNCTIONS
//  referred to the min-max median point.
// <oasomefun@futa.edu.ng> c. 2020

/* DECLARE */
template<class T>
T mid_interval(const T& max, const T& min);

template<class T, size_t N>
void normalize(const T (&x)[N], T* x_n, const T& max, const T& min) noexcept;

template<class T, size_t N>
void denormalize(const T (&x_n)[N], T* x, const T& max, const T& min) noexcept;

/* DEFINE */

// MEDIAN OF MIN-MAX INTERVAL
// computes middle point in the closed interval [min max]
template<class T>
T mid_interval(const T& max, const T& min) {
    return (max+min)/2.0;
}

// NORMALIZATION referred to the min-max median point.
template<class T, size_t N>
void normalize(const T (&x)[N], T* x_n, const T& max, const T& min) noexcept {
// median point in the interval
    const T mid = mid_interval(max, min);
// sweep: normalize interval
    for (int id = 0; id<N; id++) {
        x_n[id] = (x[id]-mid)/(max-mid);
        // std::cout << x_n[id] << std::endl; //debug
    }

// normalized limits: always 1 and -1
}

// DE-NORMALIZATION referred to the min-max median point.
template<class T, size_t N>
void denormalize(const T (&x_n)[N], T* x, const T& max, const T& min) noexcept {
// median point in the interval
    const T mid = mid_interval(max, min);
// sweep: denormalize interval
    for (int id = 0; id<N; id++) {
        x[id] = (x_n[id]*(max-mid))+mid;
    }
}

#endif //MODERNPIDCONTROLSS_NORM++_KERNEL_H
