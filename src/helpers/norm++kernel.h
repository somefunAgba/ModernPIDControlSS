//
// Creado por SomefunAgba el 13/6/2020.
//

#ifndef MODERNPIDCONTROLSS_NORMS++_KERNEL_H
#define MODERNPIDCONTROLSS_NORMS++_KERNEL_H

//
// Archivo: norms++kernel.h
// Código fuente C/C++ creado el  : 13-Jun-2020 09:37:47
//

// Archivos de Inclusión
// cambiar encabezados para otros proyectos, si no es Arduino.
#include <Arduino.h>

// FUNCIONES DE INTERVALO DE (DES)NORMALIZACIÓN MEDIANA MIN-MAX
//  referido al punto medio min-max.
// <oasomefun@futa.edu.ng> c. 2020

/* DECLARAR */
template<class T>
T mid_interval(const T& max, const T& min);

template<class T, size_t N>
void normalize(const T (&x)[N], T* x_n, const T& max, const T& min) noexcept;

template<class T, size_t N>
void denormalize(const T (&x_n)[N], T* x, const T& max, const T& min) noexcept;

/* DEFINIR */

// MEDIANA DEL INTERVALO MIN-MAX
// calcula el punto medio en el intervalo cerrado [min max]
template<class T>
T mid_interval(const T& max, const T& min) {
    return (max+min)/2.0;
}

// NORMALIZACIÓN referida al punto medio min-max.
template<class T, size_t N>
void normalize(const T (&x)[N], T* x_n, const T& max, const T& min) noexcept {
// punto medio en el intervalo
    const T mid = mid_interval(max, min);
// barrido: normalizar intervalo
    for (int id = 0; id<N; id++) {
        x_n[id] = (x[id]-mid)/(max-mid);
        // std::cout << x_n[id] << std::endl; //depuración
    }

// límites normalizados: siempre 1 y -1
}

// DESNORMALIZACIÓN referida al punto medio min-max.
template<class T, size_t N>
void denormalize(const T (&x_n)[N], T* x, const T& max, const T& min) noexcept {
// punto medio en el intervalo
    const T mid = mid_interval(max, min);
// barrido: desnormalizar intervalo
    for (int id = 0; id<N; id++) {
        x[id] = (x_n[id]*(max-mid))+mid;
    }
}

#endif //MODERNPIDCONTROLSS_NORM++_KERNEL_H
