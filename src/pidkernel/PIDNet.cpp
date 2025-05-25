
/*
 * File: sfunPID_kernel.c
 * SOMEFUN'S Modern PID Controller Architecture
 * oasomefun@futa.edu.ng            : 2019, 2020
 */

/* Include Files */
#include "PIDNet.h"               // Cabecera para este archivo de implementación.
#include "helpers/norm++kernel.h" // Para TAN_ST (constante de pre-distorsión tangente) y potencialmente otras ayudas matemáticas.
                                  // PI también es probable que esté definido aquí o mediante Arduino.h.

// Los comentarios Doxygen para los métodos están principalmente en PIDNet.h.
// Estos comentarios se centrarán en detalles de implementación.

/* Instance Init */
// Constructor para la clase PIDNet.
PIDNet::PIDNet(double ref, double yout, double dt,
        int umax_lim, int umin_lim, int dead_max_val, int dead_min_val) {
    // Inicializar variables de temporización y estado
    follow = 0;       // Por defecto: sin filtrado de setpoint mediante ym inicialmente.
    Ts = dt;          // Tiempo de muestreo (período) en segundos.
    T_prev = -dt;     // Tiempo de cómputo anterior, inicializado para asegurar que la primera llamada a compute se ejecute.
    countseq = 0;     // Contador de secuencia para cómputos PID.

    // Inicializar variables de proceso
    r = ref;          // Setpoint.
    y = yout;         // Variable de proceso inicial.
    ym = 0;           // Salida de referencia del modelo inicial (usada si follow=1).

    // Inicializar términos de error
    e = 0;            // Error (r - y).
    ei = 0;           // Integral del error.
    ed = 0;           // Derivada del error.

    // Inicializar componentes PID y señales de salida
    up = 0;           // Componente proporcional.
    ui = 0;           // Componente integral.
    ud = 0;           // Componente derivativo.
    upd = 0;          // Suma de los componentes P y D.
    ua = 0;           // Señal del actuador antes de saturación/zona muerta (suma intermedia).
    v = 0;            // Suma de los términos P, I, D antes de saturación (a menudo Kp * (Término_P + Término_I_ponderado + Término_D_ponderado)).
    u = 0;            // Salida de control después de saturación.
    uo = 0;           // Salida final después de zona muerta (si se aplica, este ejemplo lo comenta en compute).
    e_t = 0;          // Error transformado, usado en el contexto CPLMFC (Kp * e_t es parte de 'v').

    // Establecer límites de saturación de salida
    umax = umax_lim;  // Valor máximo de salida.
    umin = umin_lim;  // Valor mínimo de salida.

    // Establecer parámetros de zona muerta (usando 'this->' para distinguir de parámetros si los nombres fueran idénticos)
    this->dead_max = dead_max_val;
    this->dead_min = dead_min_val;

    // Inicializar ganancias PID y constantes de tiempo a valores por defecto/seguros.
    // Estos son típicamente sobrescritos por configuraciones manuales o ajuste CPLMFC.
    Kp = 0.001;       // Ganancia proporcional.
    Ki = 0.0;         // Ganancia integral.
    Kd = 0.0;         // Ganancia derivativa.
    lambdai = 1;      // Ponderación para el término integral en el cálculo de 'e_t' (para CPLMFC).
    lambdad = 1;      // Ponderación para el término derivativo en el cálculo de 'e_t' (para CPLMFC).
    Ti = 100;         // Constante de tiempo integral (Ti grande significa Ki pequeño inicialmente).
    Td = 0;           // Constante de tiempo derivativa.
    // Tf: Constante de tiempo para el filtro derivativo. Inicializada abajo basada en Ts.
    b = 1;            // Ponderación del setpoint para el término P (por defecto: P actúa sobre el error r-y).
    c = 0;            // Ponderación del setpoint para el término D (por defecto: D actúa sobre el cambio de -y).

    /* Inicialización de Discretización y Filtro */
    // Calcular una frecuencia de corte para el filtro derivativo.
    // Esto es 1/10 de la frecuencia de Nyquist (PI/Ts es Nyquist en rad/s).
    double cut_freq = (PI) / (10.0 * Ts); // Ts es dt del constructor.

    // kpi: Una constante relacionada con la transformación bilineal, usada para discretizar los términos I y D.
    // TAN_ST es probablemente tan(PI * Ts_ref_interna / Ts_real) para pre-distorsión, o simplemente tan(frec_corte_normalizada * Ts / 2).
    // Aquí, parece ser una constante pre-calculada de norm++kernel.h, posiblemente tan(PI/20.0) como se comentó.
    // kpi = cut_freq / TAN_ST; // Esto se parece a 2*fc/tan(wc*Ts/2), relacionado con Tustin pre-distorsionado.
    // Tf: Constante de tiempo para el filtro paso bajo de primer orden para el término derivativo.
    // Esto ayuda a reducir la amplificación de ruido por la acción derivativa.
    // Tf = Ts / (2.0 * TAN_ST); // Fórmula para el coeficiente del filtro en el dominio discreto.
    // La derivación exacta de kpi y Tf depende de las elecciones específicas de implementación discreta
    // (ej., qué filtro de tiempo continuo se está discretizando y cómo).
    // Asumamos que TAN_ST es una constante de pre-distorsión como tan(omega_c * Ts / 2) donde omega_c está relacionada con frec_corte.
    // Si TAN_ST = tan( (PI/10.0*Ts) * Ts / 2) = tan(PI*Ts/20), entonces...
    // kpi = (PI/10.0*Ts) / tan(PI*Ts/20.0)
    // Tf = Ts / (2.0*tan(PI*Ts/20.0))
    // El código proporcionado tiene Ts en Tf, pero dt (que es Ts) en cut_freq. Consistente.
    kpi = cut_freq / TAN_ST; // TAN_ST es probablemente tan( (PI/Fs_ref)/(2*N) ) o similar, pre-calculado.
    Tf = Ts / (2.0 * TAN_ST); // Constante de tiempo efectiva para el filtro derivativo.


    // filter_u se construye por defecto. uf es la salida filtrada.
    uf = 0;           // Salida filtrada inicial.
}

/* Function Definitions */
/*
 *  Esta función implementa el algoritmo de control PID 2-DOF (Dos Grados de Libertad) en realización bilineal.
 *  La estructura parece ser una forma específica de PID, posiblemente interactuante o paralelo,
 *  con anti-windup y ponderación del setpoint.
 */
void PIDNet::compute(const double& t) {

    double ym_local, yfict, e_u, Keu, ep_local, kui; // Variables locales para el cómputo.

    T_prev = t; // Actualizar el tiempo del último cómputo.

    /*  Los comentarios del Esquema de Discretización son generales, la discretización real está embebida en las fórmulas. */
    /*  Parametrización fraccional bilineal */
    /*  restringir el sintonizador de discretización para que esté dentro de la unidad */
    /*  límites del círculo de 0 y 1. */

    /* Cálculos relacionados con la constante bilineal (kpi, Tf) se hacen en el constructor. */
    // Líneas comentadas sugieren que estos fueron potencialmente re-calculados aquí antes.
    // kpi = 2.0/ Ts;
    // cut_freq = (PI)/(10.0*Ts);
    // st = tan(PI/20.0);
    // kpi = cut_freq/st;
    // Tf = Ts/(2.0*st);

    /*  Procesamiento de Entradas */
    // Determinar la señal de referencia para el cálculo del error.
    // Si el modo 'follow' está habilitado, usar 'this->ym' (que podría ser 'r' filtrado desde cplmfc).
    // De lo contrario, 'this->ym' se establece directamente al setpoint crudo 'r'.
    if (follow == 1) {
        ym_local = this->ym; // Usar setpoint pre-filtrado si está disponible vía CPLMFC.
    }
    else {
        this->ym = r;       // Si no se sigue un ym externo, el ym interno es solo r.
        ym_local = this->ym;
    }

    /*  Anti-Windup (AWU) y Cálculo de Variable de Proceso Ficticia */
    // e_u es la diferencia entre la salida de control saturada 'u' del paso anterior
    // y 'v' (suma de los términos P,I,D antes de saturación del paso anterior).
    // Esta diferencia representa la cantidad de saturación ("error_windup").
    e_u = (u - v);

    // Keu es un "coeficiente de recálculo de salida" para anti-windup.
    // Parece ser una suma de ganancias, posiblemente relacionada con la ganancia general del controlador.
    // El término (2/Ts)*Kd sugiere una transformación de Euler hacia atrás o Tustin para la parte Kd.
    // Esta formulación específica de Keu no es estándar y su derivación requeriría más contexto.
    Keu = Kp + (0.5 * Ts * Ki) + ((2.0 / Ts) * Kd); // Nota: 2.0 para división de punto flotante.

    // kui es una constante relacionada con la ganancia integral, posiblemente para escalado en AWU.
    // 1.5F es una constante flotante. La división por Ki implica que Ki no debería ser cero.
    kui = (Ki != 0.0f) ? (1.5F / Ki) : 0.0f; // Evitar división por cero.

    yfict = y; // Comenzar con la variable de proceso real.
    // 'ua' aquí parece ser un término de ajuste basado en el error de saturación e_u y Keu.
    // Este ajuste se aplica luego a yfict.
    // Si Keu no es cero, ua escala el error_windup.
    ua = (Keu != 0.0f) ? (e_u / Keu) : 0.0f; // Evitar división por cero.
    yfict += ua; // yfict es ahora una variable de proceso "ficticia" o "corregida".
                 // Esto es una forma de anti-windup por retroalimentación (back-calculation).

    /* Actualización de Valores del Paso Anterior (Integrador y Diferenciador Discretos) */
    // Estas líneas actualizan los términos integral (ui) y derivativo (ud) usando sus valores anteriores
    // y términos de error anteriores. Esto es una implementación discreta.
    // La estructura específica (ej., `ud *= (kpi*Tf-1)`) proviene de la transformación de Tustin (bilineal)
    // de un integrador o diferenciador de tiempo continuo con filtrado.

    /*  Actualización del Diferenciador (Discreto con Filtro) */
    // ud_nuevo = c1*ud_viejo + c2*ed_nuevo + c3*ed_viejo
    // Esta parte calcula la porción `c1*ud_viejo - c3*ed_viejo`.
    ud *= (kpi * Tf - 1.0); // Factor de la transformación de Tustin de s/(Tf*s+1)
    ud -= kpi * Td * (ed);  // Contribución del error derivativo anterior 'ed'. 'ed' aquí es ed_viejo.
                            // Nota: ed se actualizará a ed_actual más tarde.

    /*  Actualización del Integrador (Discreto) */
    // ui_nuevo = ui_viejo + c_factor * (ei_nuevo + ei_viejo) o similar.
    // Esta línea añade la contribución de la integral del error anterior 'ei'.
    // (1/(Ti*kpi)) está relacionado con Ts / (2 * Ti) de Tustin de 1/s.
    ui += (Ti * kpi != 0.0f) ? (1.0 / (Ti * kpi)) * (ei) : 0.0; // ei aquí es ei_viejo.
                                                               // Evitar división por cero.
    // Este término `kui*(upd)` parece ser parte de un anti-windup o una interacción específica de la estructura PID.
    // 'upd' es la suma de los términos P y D del paso anterior.
    ui -= kui * (upd);

    /* Cálculos del Paso Actual */

    /*  Cálculos de Error para el Paso Actual */
    // ep_local: Error proporcional. Usa la ponderación del setpoint 'b' y la PV ficticia 'yfict'.
    ep_local = (b * ym_local) - yfict;
    // e: Error estándar basado en el setpoint crudo 'r' y la PV real 'y'.
    e = r - y;
    // ei: Error para el término integral. Usa ym_local, PV ficticia 'yfict' y error de saturación 'e_u'.
    // Esta es una formulación específica para el error integral, incorporando anti-windup.
    ei = (ym_local - yfict) + e_u;
    // ed: Error para el término derivativo. Usa la ponderación del setpoint 'c' y la PV ficticia 'yfict'.
    // Esto será ed_actual.
    ed = (c * ym_local) - yfict;


    /*  Cálculo de Términos de Salida Individuales para el Paso Actual */
    /*  P (Término Proporcional) */
    up = ep_local; // El componente proporcional se basa en ep_local. La ganancia Kp se aplica más tarde.

    /*  D (Término Derivativo) */
    // Completa el cálculo del término derivativo usando 'ed' actual.
    ud += kpi * Td * (ed); // Añadir contribución derivativa actual.
    // Aplicar el denominador del filtro derivativo (transformación de Tustin de 1/(Tf*s+1)).
    ud = (kpi * Tf + 1.0 != 0.0f) ? ud / (kpi * Tf + 1.0) : 0.0; // Evitar división por cero.


    /* Suma PD */
    // upd: Suma de los componentes proporcional y derivativo actuales (antes del escalado Kp).
    // Se usa en la actualización del término integral y potencialmente en otros lugares.
    upd = (up + ud);

    /*  I (Término Integral) */
    // Completa el cálculo del término integral usando 'ei' actual.
    ui += (Ti * kpi != 0.0f) ? (1.0 / (Ti * kpi)) * (ei) : 0.0; // Añadir contribución integral actual.
                                                                // Evitar división por cero.
    // Recálculo adicional de la salida integral, posiblemente por anti-windup o razones estructurales.
    ui -= ua;         // Restar el término 'ua' (relacionado con el error de saturación).
    ui += kui * (upd); // Añadir suma P+D actual escalada.

    /*  Suma de Salida de Términos Contribuyentes (antes del escalado Kp principal) */
    /* Criticar / Combinar términos para 'v' */
    // e_t: Esta es una suma ponderada de los componentes P, I, D (up, ui, ud).
    // lambdai y lambdad son factores de ponderación, potencialmente para adaptación CPLMFC.
    // Nota: up, ui, ud aquí son los componentes, aún no escalados por Kp, Ki, Kd respectivamente
    // en el sentido tradicional (ej. up es solo error_p, no Kp*error_p).
    // La ganancia Kp principal se aplica a este e_t combinado.
    e_t = (up + lambdai * (ui) + lambdad * ud);
    v = Kp * e_t; // 'v' es la salida PID no saturada. Kp actúa como una ganancia general aquí.

    /* Recálculo de salida combinada para filtrado */
    // uf: Esta es la señal que se filtrará. Es 'v' ajustado por 'ua'.
    // Esto parece ser otra capa de anti-windup o acondicionamiento de salida.
    uf = v - ua;

    /*  Restricciones Reales de la Entrada de Control para u */
    //Serial.print("bef_ u="); Serial.println( u); // Depuración

    /* Saturación Dura y Asignación de Salida */
    // La intención de filter_u.run(u, uf) no está clara.
    // Típicamente, filter_u.run(uf, variable_filtrada_u) o u = filter_u.run(uf).
    // Asumiendo que uf es la entrada al filtro, y el resultado debería almacenarse en u.
    // Si filter_u está activo, debería ser: u = filter_u.run(uf); (o similar basado en la API de filterFO_pass)
    // Por ahora, asignación directa:
    u = uf; // Si no se aplica filtro o filter_u es un paso directo (pass-through).

    // Aplicar límites de saturación.
    u = fmax((double) umin, fmin(u, (double) umax ));

    // uo es la salida final que iría a un actuador.
    uo = u;
    // La función dead_zone está comentada. Si estuviera activa, modificaría uo.
    // dead_zone<double>(uo, dead_max, dead_min);
    // Podría ser necesaria una re-saturación después de dead_zone si la lógica de dead_zone puede exceder umax/umin.
    // uo = fmax((double) umin, fmin(uo, (double) umax ));


    /* Tareas Domésticas Varias. */
    // Incrementar el contador de muestras interno para el PID.
    countseq += 1;
}

void PIDNet::set_bc_follow(const int& b_val, const int& c_val, const char& follow_val) {
    // Asignar parámetros de ponderación del setpoint para los términos P y D.
    this->b = b_val; // b=1: P actúa sobre el error; b=0: P actúa sobre -y.
    this->c = c_val; // c=1: D actúa sobre el cambio del error; c=0: D actúa sobre el cambio de -y (requiere formulación cuidadosa de ed).
    // Establecer el modo 'follow', que influye en cómo se usa ym (setpoint modelo/filtrado).
    this->follow = follow_val;
}


// SATURACIÓN (Implementaciones alternativas, comentadas)
// u = maxim((double)  umin, (minim( u, (double)  umax)));

/* Saturación Logística (Alternativa, comentada)*/
// nlsig( u, du,  u, (double) umax, (double) umin,
//        (double) umax, (double) umin,
//        33, 6, 0, 0);
//Serial.print("aft_ u="); Serial.println( u);

//    double u_norm[1] = {1.0};
//    double u_act[1] = { u};
//
//    //Serial.print("prior: "); Serial.println( u);
//    normalize<double>(u_act, u_norm,  umax,  umin);
//    // Serial.print("norm: "); Serial.println(u_norm[0]);
//    u_norm[0] = nlsig(u_norm[0], 1.0, -1.0,
//            1.0, -1.0,
//            33, 6, 0);
//    //Serial.print("out_norm: "); Serial.println(u_norm[0]);
//    denormalize<double>(u_norm, u_act,  umax,  umin);
//     u = u_act[0];
//    //Serial.print("after: "); Serial.println( u);

// Serial.print("UPWM: ");Serial.println( u); // Depuración


//    /* EQUIVALENCIA DE SATURACIÓN, ZONA MUERTA Y FRICCIÓN DE COULOMB (Alternativa, comentada) */
//    // NL(.) 1-2 . ZONA MUERTA, mín Y ZONA MUERTA INVERSA, máx
//    if ((fabs( u) <= fabs( zerotol))) {
//        // 1. menor o en la zona muerta (límite mínimo)
//         u = 0;
//    } else if ((fabs( u) > fabs( zerotol)) && (fabs( u) <= fabs( deadmax))) {
//        // 2. en la zona muerta inversa (límite máximo)
//         u = copysign( deadmax,  u); // si u < 0, u = -deadmax
//    } else {
//        // 3. fuera de la zona muerta inversa (límite máximo)
//        // añadido deadmax como perturbación, efecto de fricción de Coulomb en cierto sentido.
//         u = copysign(fabs( u +  deadmax),  u);
//    }

/*
 * File trailer for sfunPID.cpp
 *
 * [EOF]
 */
