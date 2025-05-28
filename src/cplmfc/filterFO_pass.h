//
// Creado por oasomefun@futa.edu.ng el 16/1/2020.
//
#ifndef FILTERFO_PASS_H
#define FILTERFO_PASS_H

#include "Arduino.h"

#ifndef TAN_ST_C
#define TAN_ST_C
/**
 * Constante tan de pre-distorsión bilineal
 * en tiempo discreto de Shanon
 */
inline constexpr double TAN_ST = 0.1583844403;
#endif

/* Declaraciones de Clase */
class filterFO_pass{
public:
    explicit filterFO_pass();
    double x;
    double Tf_kpi;
    void run(double&, double);
};

#endif // FILTERFO_PASS_H (El comentario original no tenía texto después de //, así que se mantiene igual)
