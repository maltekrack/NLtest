## State-space model of the advanced base-excited virtual experiment

This section describes the mathematical model underlying the virtual experiment in `'EXAMPLES/BT03_virtualClampedBeam'`. 

### Model of the structure under test
The structure under test (SUT) in this example is a doubly-clamped beam, inspired by the test rig form [Mueller.2022]. Several responses are measaured (via sensors) along the beam's length, which results in a generalised response vector $\boldsymbol{q}$. The mmathematical model of the structure is described using the (linear) modal coordinates $\boldsymbol{\eta}$, such that $\boldsymbol{q} = \boldsymbol{\Phi\boldsymbol{\eta}}$, with $\boldsymbol{\Phi}$ containing the first $n_\mathrm{m}$ linear mass-normalized mode shapes of the beam evaluated at the response sensor locations. The equation of motion of the SUT under base excitation is

```math
\ddot{\boldsymbol{\eta}} + \boldsymbol{D}\dot{\boldsymbol{\eta}} + \boldsymbol{K}\boldsymbol{\eta} + \boldsymbol{f}_\mathrm{nl}\left( \boldsymbol{\eta} \right) = -\boldsymbol{\gamma}\ddot{q}_\mathrm{b}\,. \hspace{1cm}\mathrm{(1)}
````

$\boldsymbol{D}$ and $\boldsymbol{K}$ are the linear modal damping and stiffness matrices of the beam, respectively. The nonlinear restoring forces $\boldsymbol{f}_\mathrm{nl}$ arising from the bending-stretching coupling are expressed directly in the modal coordinates. The one-directional base acceleration is denoted $\ddot{q}_\mathrm{b}$, and $\boldsymbol{\gamma}$ describes the distribution of the inertia forces to the individual modes. 

### Model of the exciter
As in the force-excited example, the exciter is modelled as a spring-mass-damper system with electric circuit (see e.g. [Hippold.2024]). The moving part of the exciter is assumed to be rigidly connected to the base of the structure under test, such that the coordinate $q_\mathrm{b}$ describes the motion of the exciter. The exciter coupled with the SUT is described by the ordinary differential equation

```math
\frac{m_\mathrm{SUT} + m_\mathrm{ex}}{m_\mathrm{ex}}\ddot{q}_\mathrm{b} + 2 \zeta_\mathrm{ex} \omega_\mathrm{ex} \dot{q}_\mathrm{b} + \omega_\mathrm{ex}^2 q_\mathrm{b} = \frac{1}{m_\mathrm{ex}} \left( \frac{G}{R} u(t) - \boldsymbol{\gamma}^\top \ddot{\boldsymbol{\eta}} \right) \hspace{1cm}\mathrm{(2)}
```

with the moving mass of the exciter $`m_\mathrm{ex}`$, the mass of the SUT $m_\mathrm{SUT}$, the exciter natural frequency $`\omega_\mathrm{ex}`$, and the exciter damping ratio $`\zeta_\mathrm{ex}`$. $`G`$ is the exciter's electromotive force constant and $`R`$ the resistance of the coil. The input voltage is denoted $`u(t)`$ and is explicitly time dependent. The parameters in the example are chosen to match a Brüel & Kjaer exciter type 4809 with maximum forcing of 45 N. Note that $m_\mathrm{ex}$ includes the exciter armature and the fixture (frame) connecting the two ends of the beam to it.

### State-space model of the plant
Concatenation Eqs. (1) and (2) into matrix form yields the equation of motion of the coupled system,

````math
\underbrace{\begin{bmatrix} \boldsymbol{I} & \boldsymbol{\gamma} \\ \frac{1}{m_\mathrm{ex}}\boldsymbol{\gamma}^\top & \frac{m}{m_\mathrm{ex}} \end{bmatrix}}_{\boldsymbol{M}_\mathrm{c}} \begin{bmatrix} \ddot{\boldsymbol{\eta}} \\ \ddot{q}_\mathrm{b} \end{bmatrix} +
\underbrace{\begin{bmatrix} \boldsymbol{D} & \boldsymbol{0} \\ \boldsymbol{0} & 2\zeta_\mathrm{ex}\omega_\mathrm{ex} \end{bmatrix}}_{\boldsymbol{D}_\mathrm{c}} \begin{bmatrix} \dot{\boldsymbol{\eta}} \\ \dot{q}_\mathrm{b} \end{bmatrix}+
\underbrace{\begin{bmatrix} \boldsymbol{K} & \boldsymbol{0} \\ \boldsymbol{0} & \omega_\mathrm{ex}^2 \end{bmatrix}}_{\boldsymbol{K}_\mathrm{c}} \begin{bmatrix} \boldsymbol{\eta} \\ q_\mathrm{b} \end{bmatrix} + 
\begin{bmatrix} \boldsymbol{f}_\mathrm{nl}\left( \boldsymbol{\eta} \right) \\ 0 \end{bmatrix} = 
\begin{bmatrix} \boldsymbol{0} \\ \frac{G}{R m_\mathrm{ex}} u(t) \end{bmatrix}\,, \hspace{1cm}\mathrm{(3)}
````
with $m = m_\mathrm{ex} + m_\mathrm{SUT}$.

Eq. (3) is implemented in the virtual experiment by transforming the coupled second-order system into state-space representation. It takes on the form

````math
\dot{\boldsymbol{z}} = \boldsymbol{A} \boldsymbol{z} + \boldsymbol{B}_\mathrm{nl}\boldsymbol{f}_\mathrm{nl}(\boldsymbol{T}_\mathrm{nl}\boldsymbol{z}) + \boldsymbol{B}u(t)\,, \hspace{1cm}\mathrm{(4)}
````
with 
````math
\begin{align*}
\boldsymbol{z} &= \begin{bmatrix} \boldsymbol{\eta} & q_\mathrm{b} & \dot{\boldsymbol{\eta}} & \dot{q}_\mathrm{b} \end{bmatrix}^\top \\
%
A &= \begin{bmatrix} \boldsymbol{0} & \boldsymbol{I} \\ -\boldsymbol{M}\mathrm{c}^{-1}\boldsymbol{K}_\mathrm{c} & -\boldsymbol{M}\mathrm{c}^{-1}\boldsymbol{D}_\mathrm{c} \end{bmatrix} \\
%
\boldsymbol{B}_\mathrm{nl} &= \begin{bmatrix} \boldsymbol{0} \\ \boldsymbol{0} \\ -\boldsymbol{I} \\ \boldsymbol{0} \end{bmatrix} \\
%
\boldsymbol{T_\mathrm{nl}} &= \begin{bmatrix} \boldsymbol{I} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \end{bmatrix}\\
%
\boldsymbol{B} &= \begin{bmatrix} \boldsymbol{0} \\ 0 \\ \boldsymbol{M}_\mathrm{c}^{-1} \begin{bmatrix} \boldsymbol{0} \\ \frac{G}{R m_\mathrm{ex}} \end{bmatrix} \end{bmatrix} \,.
\end{align*}
````
### References
[Mueller.2022] F. Mueller, L. Woiwode, J. Gross, M. Scheel, M. Krack (2022): Nonlinear damping quantification from phase-resonant tests under base excitation, Mechanical Systems and Signal Processing. https://doi.org/10.1016/j.ymssp.2022.109170

[Hippold.2024]	P. Hippold, M. Scheel, L. Renson, M. Krack (2024): Robust and fast backbone tracking via phase-locked loops, Mechanical Systems and Signal Processing. http://doi.org/10.1016/j.ymssp.2024.111670