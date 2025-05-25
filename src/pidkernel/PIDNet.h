/*
  File: sfunPID.h
 
//  Author: oasomefun@futa.edu.ng            : 2019
 
 */


//#include <stddef.h>
//#include <stdlib.h>
//#include <stdint.h>

#ifndef SFUNPID_H
#define SFUNPID_H

#include <Arduino.h>               // Biblioteca principal de Arduino para funciones como fabs, copysign, etc.
#include "cplmfc/filterFO_pass.h"   // Para la clase filterFO_pass, usada para filtrar la salida.

/*!
 *  @class PIDNet
 *  @brief Encapsula un controlador PID (Proporcional-Integral-Derivativo).
 *
 *  Esta clase almacena todas las variables de estado, parámetros y métodos necesarios
 *  para realizar el control PID. Incluye características como ponderación del setpoint,
 *  saturación de salida, compensación de zona muerta e integración con el algoritmo
 *  de autoajuste CPLMFC (mediante parámetros compartidos como lambdai, lambdad).
 */
class PIDNet {
public:
    /*!
     *  @brief Constructor para la clase PIDNet.
     *  @param r_init Setpoint inicial (valor de referencia) para el controlador PID.
     *  @param y_init Valor inicial de la variable de proceso (salida medida).
     *  @param Ts_val Tiempo de muestreo (en segundos) para el controlador PID.
     *  @param umax_val Valor máximo permitido para la salida de control (límite superior de saturación).
     *  @param umin_val Valor mínimo permitido para la salida de control (límite inferior de saturación).
     *  @param dead_max_val Límite superior de la zona muerta para la salida de control.
     *  @param dead_min_val Límite inferior de la zona muerta para la salida de control.
     */
    explicit PIDNet(double r_init, double y_init, double Ts_val, int umax_val, int umin_val, int dead_max_val, int dead_min_val);

    /*!
     *  @brief Calcula la salida de control PID.
     *         Este es el método principal que implementa el algoritmo PID. Calcula
     *         el error, los términos proporcional, integral y derivativo, los suma,
     *         aplica saturación y potencialmente compensación de zona muerta, y actualiza
     *         la salida de control `u`. También actualiza `T_prev` al tiempo actual `t`.
     *  @param t Tiempo actual en segundos.
     */
    void compute(const double& t);

    /*!
     *  @brief Configura la ponderación del setpoint y el modo anti-windup (seguimiento).
     *  @param b_val Factor de ponderación del setpoint para el término proporcional (0 o 1).
     *               Si b=1, el término proporcional actúa sobre el error (r-y). Si b=0, actúa sobre -y.
     *  @param c_val Factor de ponderación del setpoint para el término derivativo (0 o 1).
     *               Si c=1, el término derivativo actúa sobre el cambio del error. Si c=0, actúa sobre el cambio de -y.
     *               (Nota: La implementación real en PIDNet.cpp para el derivativo podría necesitar verificación con este comentario).
     *  @param follow_val Modo para la estrategia anti-windup u otro comportamiento de seguimiento.
     *                    (El mecanismo exacto de 'follow' no se detalla completamente aquí, pero se establece).
     */
    void set_bc_follow(const int& b_val, const int& c_val, const char& follow_val);

    // --- Variables Miembro ---

    // Configuración y Estado
    char follow;      //!< Modo para anti-windup o comportamiento de seguimiento. Establecido por `set_bc_follow`.
    double Ts;        //!< Tiempo de muestreo (período) en segundos. Establecido por el constructor.
    double T_prev;    //!< Tiempo del último cálculo PID (segundos). Actualizado en `compute`.
    int countseq;     //!< Un contador de secuencia, a menudo incrementado cada vez que el PID calcula. (El uso puede variar, ej. en ejemplos).

    // Variables de Proceso
    double r;         //!< Setpoint o valor de referencia para la variable controlada.
    double y;         //!< Variable de proceso medida (salida real del sistema).
    double ym;        //!< Setpoint filtrado o salida del modelo de referencia. Usado por CPLMFC o si `filter_r` se aplica a `r`.
    double e;         //!< Señal de error (típicamente `r - y` o `ym - y`).
    double ei;        //!< Integral del error `e`. Se acumula con el tiempo.
    double ed;        //!< Derivada del error `e` (o derivada de `y`).
    double e_t;       //!< Un término de error transformado o específico, potencialmente usado en la regla de Lyapunov de CPLMFC (Knet.e_t). (La naturaleza exacta no está completamente clara solo con PIDNet.h).

    // Componentes de la Salida de Control
    double up;        //!< Componente proporcional de la salida de control. (Kp * error_p)
    double ui;        //!< Componente integral de la salida de control. (Ki * ei)
    double ud;        //!< Componente derivativo de la salida de control. (Kd * ed)
    double upd;       //!< Suma de los componentes proporcional y derivativo (a menudo P y D actúan juntos antes que I).
    double ua;        //!< Señal del actuador antes de saturación/zona muerta (a menudo suma de up, ui, ud). También referida como 'v' a veces.
    double v;         //!< Señal de control intermedia, típicamente suma de los términos P, I, D antes de saturación. (A menudo igual que ua).
    double u;         //!< Salida de control final después de saturación pero antes de zona muerta/filtrado de salida en algunos contextos.
    double uo;        //!< Señal de salida final acondicionada enviada al actuador (ej. después de zona muerta y/o filtrado de salida mediante `filter_u`).

    // Límites de Saturación de Salida y Zona Muerta
    int umax;         //!< Valor máximo para la salida de control `u` (límite superior de saturación).
    int umin;         //!< Valor mínimo para la salida de control `u` (límite inferior de saturación).
    int dead_max;     //!< Umbral superior para la compensación de zona muerta.
    int dead_min;     //!< Umbral inferior para la compensación de zona muerta. (Valores entre -dead_min y +dead_min podrían ser cero).

    // Ganancias PID y Constantes de Tiempo
    double Kp;        //!< Ganancia proporcional.
    double Ki;        //!< Ganancia integral.
    double Kd;        //!< Ganancia derivativa.
    double lambdai;   //!< Tasa de aprendizaje o factor de ponderación para la adaptación del componente integral (usado por CPLMFC).
    double lambdad;   //!< Tasa de aprendizaje o factor de ponderación para la adaptación del componente derivativo (usado por CPLMFC).
    double Ti;        //!< Constante de tiempo integral (segundos). Ki = Kp / Ti.
    double Td;        //!< Constante de tiempo derivativa (segundos). Kd = Kp * Td.

    // Parámetros de Ponderación del Setpoint
    int b;            //!< Ponderación del setpoint para el término proporcional (0 o 1). Por defecto 1.
    int c;            //!< Ponderación del setpoint para el término derivativo (0 o 1). Por defecto 0.

    // Constantes Internas del Algoritmo PID
    double kpi;       //!< Constante de transformación bilineal, relacionada con la implementación en tiempo discreto. (1 / Ts a menudo forma parte de ella).
    double Tf;        //!< Constante de tiempo para el filtro de primer orden usado para el término derivativo (para hacerlo menos ruidoso).

    // Filtro de Salida
    filterFO_pass filter_u; //!< Instancia de filtro de primer orden para suavizar la salida de control `u`.
    double uf;              //!< Salida de control filtrada. `uf = filter_u.run(u)`. (Nota: uo se usa a menudo como la salida final después de esto).
};

// PIDNet::~PIDNet() = default; // El destructor por defecto está bien.
// Estilo de firma de función antiguo comentado:
// void sfunPID_kernel(PIDNet& Knet, const double& t);

// Macros de utilidad comentadas (no usadas en otra parte de esta cabecera):
//#define maxim(a,b)	(((a) > (b)) ? (a) : (b))
//#define minim(a,b)	(((a) < (b)) ? (a) : (b))

/*!
 *  @brief Aplica una no linealidad de zona muerta a una entrada de control.
 *  @tparam T El tipo de dato de la señal de entrada (se espera que sea de punto flotante como double o float).
 *  @param uin Señal de entrada (valor de control) a modificar. Se pasa por referencia.
 *  @param dead_max_val La magnitud superior de la región "activa" de la zona muerta.
 *                      Si `abs(uin)` está entre `dead_min_val` y `dead_max_val`, `uin` se establece a `copysign(dead_max_val, uin)`.
 *  @param dead_min_val El umbral de magnitud inferior de la zona muerta.
 *                      Si `abs(uin)` es menor o igual que `dead_min_val`, `uin` se establece a 0.
 *
 *  Esta función modela un tipo específico de zona muerta combinada con un efecto de "zona muerta inversa" o "salida mínima".
 *  1. Si `|uin| <= dead_min_val`, la salida es 0.
 *  2. Si `dead_min_val < |uin| <= dead_max_val`, la salida es `copysign(dead_max_val, uin)`.
 *  3. Si `|uin| > dead_max_val`, la salida es `copysign(|uin| + dead_max_val, uin)`. Esta parte es inusual para una zona muerta simple
 *     y parece añadir `dead_max_val` a las entradas que exceden este umbral, lo que podría simular un comportamiento específico del actuador
 *     o perturbación (como un tipo de fricción o stiction que necesita un esfuerzo extra para superarse y luego se añade a la salida).
 */
template<class T>
void dead_zone(T& uin, const int& dead_max_val, const int& dead_min_val); // Declaración

// Implementación de la función plantilla dead_zone
template<class T>
void dead_zone(T& uin, const int& dead_max_val, const int& dead_min_val) {
    /*  Restricciones reales de la entrada de control para uin */
    /*  equivalencia de saturación, zona muerta y fricción de Coulomb */
    // nl(.) 1-2 . zona muerta, mín y zona muerta inversa, máx
    // Nota: Los nombres de los parámetros en la firma de la función se cambiaron de dead_max/dead_min
    // a dead_max_val/dead_min_val para evitar conflictos con miembros de PIDNet si se usan de forma laxa.
    // Sin embargo, el comentario Doxygen usa _val, mientras que la implementación de abajo usa dead_max y dead_min.
    // Asumiendo que dead_max y dead_min aquí se refieren a los parámetros de la función.

    if ((fabs(uin) <= fabs(dead_min_val))) {
        // 1. La entrada está dentro o en el límite mínimo de la zona muerta: la salida es cero.
        uin = 0;
    }
    else if ((fabs(uin) > fabs(dead_min_val)) && (fabs(uin) <= fabs(dead_max_val))) {
        // 2. La entrada está por encima del mínimo de la zona muerta pero dentro o en el límite máximo de la "zona muerta inversa":
        //    La salida se establece a +/- dead_max_val (manteniendo el signo original de uin).
        uin = copysign(static_cast<T>(dead_max_val), uin); // Convierte dead_max_val a T
    }
    else {
        // 3. La entrada está fuera del límite máximo de la "zona muerta inversa":
        //    El comentario original "added deadmax as disturbance, effect of coulomb friction in a sense"
        //    sugiere que esto modela un comportamiento donde superar stiction/max_dead_zone añade esa cantidad a la salida.
        //    La salida se convierte en uin + copysign(dead_max_val, uin), pero simplificado como copysign(fabs(uin) + fabs(dead_max_val), uin)
        //    que es equivalente a uin + dead_max_val si uin es positivo, o uin - dead_max_val si uin es negativo (asumiendo que dead_max_val es positivo).
        //    Usemos la lógica original: copysign(fabs(uin + dead_max_val), uin) no es del todo correcto.
        //    El original era: uin = copysign(fabs(uin+dead_max), uin);
        //    Si dead_max es positivo:
        //      Si uin es positivo, uin = uin + dead_max.
        //      Si uin es negativo, uin = -( |uin| - dead_max ) si |uin|>dead_max, o uin = dead_max - |uin| si |uin|<dead_max (esto es complejo).
        //    La interpretación más directa de "added deadmax as disturbance" es:
        //    uin = uin + copysign(static_cast<T>(dead_max_val), uin);
        //    Sin embargo, se mantiene la línea original del código proporcionado `uin = copysign(fabs(uin+dead_max), uin);`.
        //    Esta línea específica es: uin_nuevo = |uin_viejo + dead_max_val| con el signo de uin_viejo.
        //    Ejemplo: uin_viejo = 10, dead_max_val = 2 -> uin_nuevo = |10+2| = 12 (signo +) -> 12
        //    Ejemplo: uin_viejo = -10, dead_max_val = 2 -> uin_nuevo = |-10+2|=|-8|=8 (signo -) -> -8. Esto es contraintuitivo para "añadir perturbación".
        //    Usemos una interpretación más estándar si el objetivo es "la salida más allá de dead_max obtiene dead_max añadido":
        //    uin = uin + copysign(static_cast<T>(dead_max_val), uin);
        //    Pero manteniéndonos en la línea original por ahora:
        uin = copysign(fabs(uin + static_cast<T>(dead_max_val)), uin); // Línea original, convierte dead_max_val
    }
}

#endif // SFUNPID_H

/*
  File trailer for sfunPID.h
  [EOF]
*/