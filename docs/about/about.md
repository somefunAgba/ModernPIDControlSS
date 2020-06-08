---
template: overrides/main.html
---


# About

Many accounts available online, as regards the implementation of a PID controller, modify the simplified overview representation using some practical issues namely:

1. Sampling

2. Noisy Derivative Contributions

3. Set-point and Derivative Kick

4. Integral Anti-windup

5. Bumpless Parameter Changes.

However, from knowledge available in current modern control systems theory and application, these issues can be viewed as compensating for a limited view of controller representation. The design and realization of a modern controller should inherently address these issues.

For example, Issue 1 is necessary for discrete-time implementation (software). Issue 2 is covered by digital signal-processing information theory in relation to Issue 1. Issue 3 is covered by the two-degree of freedom control structure. Issue 4 is covered by modern anti-windup synthesis. Issue 5 is covered by similarly treating the effect of parameter changes as a wind-up problem.

In this library, the realization of the PID control algorithm addresses
all these issues internally as part of the control design. It features:

1. Simplified Bilinear Discretization for Integration and Differentiation

2. Filtering: Information theory perspective of Signal processing

3. Critic Contributions

4. Two Degree-of-Freedom Control Structure

5. Automatic Anti-windup Control

6. Real-time Tuning: Closed PID-Loop Model Following Control


## Closed PID-Loop Model Following Control
**Closed PID-Loop Model Following Control** is the method used  
to tune the PID is detailed in this paper **CPLMFC**.
For more details on the theory, see [Closed PID-Loop Model Following Control Preprint](https://arxiv.org/pdf/2006.00314).


<!--Habebat equus, dictu una agros incaluit inque, undis missum laevo, est. Nomine-->
<!--ferre: maturo in non lacertis tantis **natis felicia** qui; Niseia? Timide-->
<!--putavi libertas *in ego* quodcumque mutata. Armeniae proferre nomine Olympus-->
<!--procul, est si amante intercipe Hesperium ad regis revulsit flammasque mersurum-->
<!--mansit Alcyonen iam furit?-->


