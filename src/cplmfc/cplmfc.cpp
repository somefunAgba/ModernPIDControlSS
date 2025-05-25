/*
 * File: CPLMFC.cpp
 *
 * <CLOSED PID-LOOP MODEL> <FOLLOWING CONTROL> <METHOD> : 2020
 *
 * oasomefun@futa.edu.ng. Copyright.2020
 */

#include "cplmfc.h"

// Los comentarios Doxygen para los métodos están principalmente en el archivo .h.
// Estos comentarios se centrarán en detalles de implementación.

void cplmfc::set_alpha_critics(PIDNet& Knet, const float& alpha_val, const float& lambda_i_val, const float& lambda_d_val) {
    // Almacenar la tasa de aprendizaje primaria para la adaptación de Kp en este objeto cplmfc.
    this->alpha = alpha_val;
    // Almacenar las tasas de aprendizaje para la adaptación de Ki y Kd directamente en el objeto PIDNet.
    // Se espera que Knet.lambdai y Knet.lambdad sean miembros de PIDNet.
    Knet.lambdai = lambda_i_val; // Tasa de aprendizaje para el crítico de la ganancia integral
    Knet.lambdad = lambda_d_val; // Tasa de aprendizaje para el crítico de la ganancia derivativa
}

void cplmfc::begin(PIDNet& Knet, const int& N_ts_val, const int& N_taul_val) {
    // Calcular el tiempo de establecimiento del sistema (ts) en segundos multiplicando
    // el número de intervalos de muestreo (N_ts_val) por el período de muestreo del PID (Knet.Ts).
    ts = N_ts_val * Knet.Ts;

    // Calcular el tiempo muerto del sistema (tau_l) en segundos de manera similar.
    tau_l = float(N_taul_val * Knet.Ts);

    // Inicializar/ajustar la frecuencia natural del modelo de referencia (wn).
    // Este 'wn' se usa para calcular Ti y Td más tarde en tuneKiKd.
    // Establece efectivamente la capacidad de respuesta deseada del modelo de referencia.
    tuneWn(Knet); // Se pasa Knet para acceder a los parámetros b y c.
}

void cplmfc::run(PIDNet& Knet, const double& t) {
    // Ejecutar el filtro de primer orden para la señal de referencia.
    // Knet.r es el setpoint crudo (entrada de referencia).
    // Knet.ym es la salida de referencia filtrada/del modelo.
    // Esta referencia suavizada ym es lo que la salida real del sistema y debería seguir.
    filter_r.run(Knet.ym, Knet.r);
    // Salida de depuración de ejemplo (comentada):
    // Serial.print("ym: "); Serial.println(PIDobj_I.ym); // Asumiendo que PIDobj_I es un alias para Knet

    /* Cálculo del Ajuste CPLMFC */
    // Ajustar la ganancia proporcional Kp.
    // Se pasa el Modo 2, que corresponde a una configuración nlsig específica en tuneKp.
    tuneKp(Knet, t, 2);
    // Salida de depuración de ejemplo:
    //Serial.print("Kp: ");Serial.println(Knet.Kp); // O PIDobj_I.Kp

    // Ajustar las ganancias integral (Ki) y derivativa (Kd) basadas en el nuevo Kp y el wn del modelo.
    tuneKiKd(Knet);
    // Salida de depuración de ejemplo:
    //Serial.print(" Ki: ");Serial.print(Knet.Ki);
    //Serial.print(" Kd: ");Serial.println(Knet.Kd);
}

void cplmfc::tuneWn(PIDNet& Knet) {
    /*  funciones del algoritmo de ajuste para wn (frecuencia natural del modelo de referencia) */
    float xtsn; // Un factor basado en la estructura PID (ponderaciones de setpoint b y c)
    // Knet.b y Knet.c son parámetros de ponderación del setpoint en PIDNet (0 o 1).
    // Su suma determina el tipo de respuesta PID que se busca, influyendo en xtsn.
    int xx = Knet.b + Knet.c; // Suma de las ponderaciones del setpoint

    // Estos valores de xtsn son constantes derivadas empíricamente o basadas en reglas de teoría de control
    // para diferentes configuraciones PID para lograr ciertos criterios de rendimiento (como amortiguamiento).
    // Los valores parecen relacionarse con características deseadas de sobreimpulso o establecimiento.
    if (xx == 0) { // Corresponde a I-PD o PID con b=0, c=0
        xtsn = 9.98;
//        Serial.println(xtsn); // Depuración
    }
    else if (xx == 1) { // Corresponde a PI-D o PID con b=1,c=0 o b=0,c=1
        xtsn = 7.74;
//        Serial.println(xtsn); // Depuración
    }
    else if (xx == 2) { // Corresponde a PID con b=1, c=1
        xtsn = 11.07;
//        Serial.println(xtsn); // Depuración
    }
    else { // Caso por defecto/fallback, aunque xx idealmente debería ser 0, 1, o 2.
        xtsn = 8;
    }

    /*  w_n normalizado */
    // Calcular la frecuencia natural 'wn' para el modelo de referencia.
    // Es inversamente proporcional al tiempo de establecimiento del sistema 'ts' y un factor de amortiguamiento (0.7071, que es 1/sqrt(2)).
    // Este 'wn' determina cuán rápido debe responder el modelo de referencia (y por lo tanto el PID ajustado).
    wn = float(xtsn / (0.7071 * ts)); // 0.7071 implica una razón de amortiguamiento objetivo de sqrt(2)/2 para el modelo de referencia.
    //Serial.println(xtsn); // Depuración
    //printf("wn: %lf\n", wn); // Depuración
}

void cplmfc::tuneKp(PIDNet& Knet, const double& t, const int& mode) {
    /* Actualización n-logística para Kp */
    // Esta sección implementa una regla de adaptación no lineal compleja para Kp.
    // Utiliza la función 'nlsig' (conformación/saturación de señal no lineal).
    double k_sig1, k_sig2, xe, xu, x_lim, LL, kg, kp_lim, e_t;
    // double dkp; // Declarado pero no usado

    // Normalizar el tiempo muerto 'tau_l' usando media y desviación estándar precalculadas.
    // Esta normalización (LL) se usa para calcular 'kg', un factor de ganancia.
    // Estas constantes (5.936, 8.771, etc.) probablemente se derivan de estudios empíricos o modelado del sistema.
    LL = (tau_l - 5.936F) / (8.771F);
    // 'kg' se calcula usando una función racional del tiempo muerto normalizado LL.
    // Esto sugiere que 'kg' adapta el límite de Kp basado en las características de tiempo muerto del sistema.
    kg = (0.05132*LL*LL + 0.2041*LL + 0.1214) / (LL*LL + 1.538*LL + 0.5864);
    // 'kp_lim' es un límite o objetivo calculado para Kp, influenciado por 'alpha' (tasa de aprendizaje),
    // 'kg', tiempo muerto 'tau_l', y tiempo de establecimiento 'ts'.
    kp_lim = alpha * kg * (tau_l + ts) / (ts);
    //Serial.print("kp_lim: "); Serial.println(kp_lim); // Depuración

    // 'e' es el error Knet.e (r - y o ym - y).
    // 'xe' y 'xu' son versiones desplazadas de Knet.e (error) y Knet.u (salida de control) usando kp_lim.
    xe = kp_lim + Knet.e;
    xu = kp_lim + Knet.u; // Nota: Knet.u es la salida de control anterior.

    // Aplicar conformación no lineal (saturación/escalado) a Knet.e y Knet.u usando nlsig.
    // Los parámetros de la función nlsig son: (salida, entrada, sat_sup, sat_inf, ganancia_pos, ganancia_neg, modo, p1, p2, p3)
    // k_sig1 se deriva del error Knet.e.
    nlsig(k_sig1, Knet.e, xe, -xe, kp_lim, -kp_lim, 1, 0.1, 0, 0);
    // k_sig2 se deriva de la salida de control Knet.u.
    nlsig(k_sig2, Knet.u, xu, -xu, kp_lim, -kp_lim, 1, 0.1, 0, 0);
    //Serial.print("kp_not:"); Serial.println(k_sig1+k_sig2); // Depuración

    // 'x_lim' es otro límite calculado basado en kp_lim y la suma de las señales conformadas.
    x_lim = kp_lim + (k_sig1 + k_sig2);

    // El parámetro 'mode' determina la configuración de la función nlsig para actualizar Kp.
    // Esto aplica otra capa de conformación no lineal para determinar el Kp final.
    // Los parámetros para nlsig (ganancia, p1, etc.) varían con 'mode'.
    if (mode == 0) {
        // alpha (miembro de la clase) se usa aquí, posiblemente escalado por fmax(1.0, alpha) lo cual parece redundante si alpha >=1.
        // Si alpha < 1, usa 1. Esto sugiere que se espera que alpha sea >= 1.
        nlsig(Knet.Kp, (k_sig1+k_sig2), x_lim, 0.0, kp_lim,
                0.0, (int) fmax(1.0, alpha), 0.1, 0, 0);
    }
    if (mode == 1) { // Usa una ganancia fija de 16 para nlsig.
        nlsig(Knet.Kp, (k_sig1+k_sig2), x_lim, 0.0,
                kp_lim, 0.0, 16, 0.1, 0, 0);
        // Serial.print("Kpfit: "); Serial.println(Knet.Kp); // Depuración
    }
    if (mode == 2) { // Usa una ganancia fija de 1 para nlsig. Este es el modo usado por run().
        nlsig(Knet.Kp, (k_sig1+k_sig2), x_lim, 0.0,
                kp_lim, 0.0, 1, 0.1, 0, 0);
    }
    //Serial.print("Kpfit: "); Serial.println(Knet.Kp); // Depuración

    //Knet.Kp = 16; // Anulación manual para depuración (comentado)

    // Opción adaptativa: regla de actualización de Lyapunov (ajuste adicional a Kp).
    // Esta parte parece ser una regla de actualización alternativa o suplementaria para Kp.
    // Se aplica si el tiempo actual 't' es mayor que el tiempo muerto del sistema 'tau_l'.
    if ( fabs(t) > fabs(double (tau_l)) ) {
        // Knet.e_t parece estar relacionado con la señal de error, posiblemente filtrada o transformada.
        // Su valor absoluto se toma y luego se limita por Knet.umax (salida de control máxima).
        e_t = fabs(Knet.e_t); // Knet.e_t no es un miembro estándar de PIDNet mostrado en PIDNet.h, podría ser una variable interna específica o un error tipográfico por Knet.e
        e_t = fmin((double) Knet.umax, e_t);
        // Serial.print("e_t: ");Serial.println(e_t); // Depuración

        // Actualizar Kp usando un término proporcional a Knet.e (error actual) y el e_t modificado.
        // 'alpha' (tasa de aprendizaje) y una pequeña constante (0.001) escalan este ajuste.
        // Nota: El código original usa 'e' aquí, que fue declarado pero no asignado en esta función.
        // Probablemente debería ser Knet.e, asumiendo que 'e' estaba destinado a ser el error actual.
        // Si 'e' era de 'e = Knet.ym-Knet.y;' (comentado anteriormente), entonces esto usa un error de seguimiento de modelo.
        // Asumamos que Knet.e es el previsto aquí para la regla de Lyapunov basada en el error actual.
        Knet.Kp = Knet.Kp + (alpha * 0.001 * Knet.e) * e_t; // Asumiendo que Knet.e es el previsto en lugar de 'e' no asignado
        //Serial.print("Kplya: "); Serial.println(Knet.Kp); // Depuración
    }
}

void cplmfc::tuneKiKd(PIDNet& Knet) const {
    // Calcular Ti (Constante de Tiempo Integral) y Td (Constante de Tiempo Derivativa)
    // basado en la frecuencia natural del modelo de referencia 'wn' y un factor de amortiguamiento de 0.7071 (1/sqrt(2)).
    // Estas son fórmulas estándar para la colocación de polos en lazo cerrado o para lograr ciertas características de respuesta
    // (ej., amortiguamiento crítico o sobreimpulso específico) en un sistema de segundo orden.

    /* constante de tiempo integral */
    // Ti = 2 * zeta / wn, donde zeta es la razón de amortiguamiento. Aquí, 2*0.7071/wn implica una colocación específica de polos.
    Knet.Ti = (2.0F * 0.7071F) / wn;

    /* ganancia integral */
    // Ki = Kp / Ti. Manejar la posible división por cero si Ti es (o llega a ser) cero.
    Knet.Ki = (Knet.Ti != 0.0f) ? (Knet.Kp / Knet.Ti) : 0.0f;

    /* constante de tiempo derivativa */
    // Td = 1 / (2 * zeta * wn), aquí 1 / (2 * 0.7071 * wn).
    Knet.Td = 1.0F / (2.0F * 0.7071F * wn);

    /* ganancia derivativa */
    // Kd = Kp * Td
    Knet.Kd = Knet.Kp * Knet.Td;
}