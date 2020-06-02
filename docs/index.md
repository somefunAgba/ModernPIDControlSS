# Simplified Modern Recipe for PID Control
<!-- summary, tldr, info, todo, tip , hint, important, success, check, done, faq, help, question-->
<!-- warning, attention, failure, error, bug, missing, example, quote, cite-->

!!!abstract
    The PID law is still largely misunderstood.
    The PID is the simplest representation of the complex prediction or optimization actions in the world.
    In this library, we provide a simple but practically robust realization of the PID law.

For a more technical understanding of the automatic tuning design visit (https://arxiv.org/pdf/2006.00314).

## PID Law
The PID Law is a function: $u = f(r,y)}$
$$
u=f(r,y)=\lambda_p\,u_p + \lambda_i\,u_i + \lambda_d\,u_d
$$

## Controller Usecase

<!--* `mkdocs new [dir-name]` - Create a new project.-->
<!--* `mkdocs serve` - Start the live-reloading docs server.-->
<!--* `mkdocs build` - Build the documentation site.-->
<!--* `mkdocs -h` - Print help message and exit.-->

## Tuning Usecase
=== "C++"
``` c++
    if (t >=( (PIDobj_II.T_prev+PIDobj_II.Ts)-(0.5*PIDobj_II.Ts) )) {
        // tprev[1] = t; done inside the PID

        /* CPLM Evolution */
        cplm_kernel(PIDobj_II.ym, PIDobj_II.xm, PIDobj_II.r,
               PIDobj_II.Ts, 0, 0, wn[1]);
        // Serial.print("ym: "); Serial.println(PIDobj_II.ym);
        /* CPLMFC Tuning Computation */
        tuneKp(PIDobj_II, t, L[1], ts, alpha[1], 2);
        //Serial.print("Kp: ");Serial.println(PIDobj_II.Kp);

        tuneKi(PIDobj_II, wn[1]);
        //Serial.print(" Ki: ");Serial.print(PIDobj_II.Ki);

        tuneKd(PIDobj_II, wn[1]);
        //Serial.print(" Kd: ");Serial.println(PIDobj_II.Kd);

        /* PID Control State Evolution: Architecture */
        sfunPID_kernel(PIDobj_II, t);
        //Serial.print("upwm_II: "); Serial.println(PIDobj_II.u);
    }
```

<!--    mkdocs.yml    # The configuration file.-->
<!--    docs/-->
<!--        index.md  # The documentation homepage.-->
<!--        ...       # Other markdown pages, images and other files.-->
