## State-space model of the advanced force-excited virtual experiment

This section describes the mathematical model underlying the virtual experiment in `'EXAMPLES/BT01_virtualRubBeR'` (advanced version).

### Model of the structure under test
 The structure under test is modelled as a finite element model of an Euler-Bernoulli beam with specified length, cross section, mass density and Young's modulus. The nonlinearity takes the form of an elastic Coulomb dry friction element with specified stiffness and friction limit force, which is attached to one node in the translational (bending) direction. As is common practice in nonlinear vibration analysis, the model order is reduced using the Hurty/Craig-Bampton method. A number of fixed-interface normal modes is retained, along with the static constraint mode associated with the translational degree of freedom of the node to which the friction element is attached. Linear viscous modal damping is specified. The equations governing the dynamics of the reduced-order model can be expressed as

```math
\boldsymbol{M}_\mathrm{red} \ddot{\boldsymbol{q}}_\mathrm{red} + \boldsymbol{D}_\mathrm{red} \dot{\boldsymbol{q}}_\mathrm{red} + \boldsymbol{K}_\mathrm{red} \boldsymbol{q}_\mathrm{red} + \boldsymbol{w}_\mathrm{nl,red} f_\mathrm{nl} = \boldsymbol{w}_\mathrm{ex,red} f_\mathrm{ex}
\hspace{1cm}\mathrm{(1)}
```
with $`\boldsymbol{M}_\mathrm{red}`$, $`\boldsymbol{D}_\mathrm{red}`$, $`\boldsymbol{K}_\mathrm{red}`$ being the linear mass, damping, and stiffness matrices in reduced-order coordinates, respectively. $`\boldsymbol{w}_\mathrm{nl,red}`$ is the localization vector in reduced-order coordinates for the nonlinear force $`f_\mathrm{nl}`$. $`\boldsymbol{w}_\mathrm{ex,red}`$ is the localization vector in reduced-order coordinates for the external force $`f_\mathrm{ex}`$.

To set up the Euler-Bernoulli beam model and to perform the model order reduction, functionalities of NLvib [GitHub.NLvib, Krack.2019] are used in the advanced version of the virtual experiment. If the required NLvib files are not present on the Matlab path, pre-computed structural matrices of the reduced-order model are loaded from a file.

### Model of the exciter

In the advanced version of the virtual experiment, the exciter is modelled as a spring-mass-damper system with electric circuit (see e.g. [Hippold.2024]). The exciter is described by the differential equation

```math
\ddot{q}_\mathrm{ex} + 2 \zeta_\mathrm{ex} \omega_\mathrm{ex} \dot{q}_\mathrm{ex} + \omega_\mathrm{ex}^2 q_\mathrm{ex} = \frac{1}{m_\mathrm{ex}} \left( \frac{G}{R} u(t) - f_\mathrm{ex} \right) \hspace{1cm}\mathrm{(2)}
```

with the moving mass $`m_\mathrm{ex}`$, the exciter natural frequency $`\omega_\mathrm{ex}`$, and the exciter damping ratio $`\zeta_\mathrm{ex}`$. $`G`$ is the exciter's electromotive force constant and $`R`$ the resistance of the coil. The input voltage is denoted $`u(t)`$ and is explicitly time dependent. The parameters in the example are chosen to match a Brüel & Kjaer exciter type 4809 with maximum forcing of 45 N.
The coupling between exciter and structure is assumed to be rigid. Therefore, the armature displacement is

```math
q_\mathrm{ex} = \boldsymbol{w}_\mathrm{ex,red}^\top \boldsymbol{q}_\mathrm{red} \,. \hspace{1cm}\mathrm{(3)}
```

### State-space model of the plant

The model is transformed to first-order state-space form. The states are the displacements and velocities in reduced-order coordinates,

```math
\boldsymbol{x} = \begin{bmatrix} \boldsymbol{q}_\mathrm{red} \\ \dot{\boldsymbol{q}}_\mathrm{red} \end{bmatrix} \,. \hspace{1cm}\mathrm{(4)}
```

The input is the shaker input voltage $`u(t)`$. The outputs are the translational responses at the sensor nodes (in terms of displacements, velocities or accelerations) and the excitation force resulting from the coupled shaker-structure system.

 To obtain the differential equation governing the states, the excitation force in Eq. (1) is eliminated by substituting Eqs. (2) and (3). This results in

```math
\boldsymbol{M}_\mathrm{c} \ddot{\boldsymbol{q}}_\mathrm{red} + \boldsymbol{D}_\mathrm{c} \dot{\boldsymbol{q}}_\mathrm{red} + \boldsymbol{K}_\mathrm{c} \boldsymbol{q}_\mathrm{red} + \boldsymbol{w}_\mathrm{nl,red} f_\mathrm{nl} = \boldsymbol{w}_\mathrm{ex,red} \frac{G}{R} u(t) \hspace{1cm}\mathrm{(5)}
```

with

```math
\begin{align*}
\boldsymbol{M}_\mathrm{c} &= \boldsymbol{M}_\mathrm{red} + \boldsymbol{w}_\mathrm{ex,red} m_\mathrm{ex} \boldsymbol{w}_\mathrm{ex,red}^\top \\
\boldsymbol{D}_\mathrm{c} &= \boldsymbol{D}_\mathrm{red} + \boldsymbol{w}_\mathrm{ex,red} 2 \zeta_\mathrm{ex} \omega_\mathrm{ex} m_\mathrm{ex} \boldsymbol{w}_\mathrm{ex,red}^\top \\
\boldsymbol{K}_\mathrm{c} &= \boldsymbol{K}_\mathrm{red} + \boldsymbol{w}_\mathrm{ex,red} \omega_\mathrm{ex}^2 m_\mathrm{ex} \boldsymbol{w}_\mathrm{ex,red}^\top \,.
\end{align*}
```

Eq. (5) can be brought to the first-order form

```math
\dot{\boldsymbol{x}} = \boldsymbol{A}\boldsymbol{x} + \boldsymbol{B}_\mathrm{nl} f_\mathrm{nl}(\boldsymbol{x})+ \boldsymbol{B} u(t) \hspace{1cm}\mathrm{(6)}
```

with

```math
\begin{align*}
\boldsymbol{A} &=
\begin{bmatrix}
\boldsymbol{0} & \boldsymbol{I} \\
-\boldsymbol{M}_\mathrm{c}^{-1} \boldsymbol{K}_\mathrm{c} & -\boldsymbol{M}_\mathrm{c}^{-1} \boldsymbol{D}_\mathrm{c}
\end{bmatrix} \\
\boldsymbol{B}_\mathrm{nl} &=
\begin{bmatrix}
\boldsymbol{0} \\ -\boldsymbol{M}_\mathrm{c}^{-1}\boldsymbol{w}_\mathrm{nl,red}
\end{bmatrix} \\
\boldsymbol{B} &=
\begin{bmatrix}
\boldsymbol{0} \\ \boldsymbol{M}_\mathrm{c}^{-1}\boldsymbol{w}_\mathrm{ex,red} \frac{G}{R}
\end{bmatrix} \,.
\end{align*}
```

To obtain the measurement equation for the excitation force, the armature acceleration in Eq. (2) is eliminated using Eqs. (1) and (3) resulting in

```math
f_\mathrm{ex} = \boldsymbol{C}_\mathrm{f} \boldsymbol{x} + C_\mathrm{fnl} f_\mathrm{nl} + Du(t) \hspace{1cm}\mathrm{(7)}
```

with

```math
\begin{align*}
\boldsymbol{C}_\mathrm{f} &=
\begin{bmatrix}
m_\mathrm{ex} \left( \boldsymbol{w}_\mathrm{ex,red}^\top \boldsymbol{M}_\mathrm{c}^{-1} \boldsymbol{K}_\mathrm{c} - \omega_\mathrm{ex}^2 \boldsymbol{w}_\mathrm{ex,red}^\top \right) &
m_\mathrm{ex} \left( \boldsymbol{w}_\mathrm{ex,red}^\top \boldsymbol{M}_\mathrm{c}^{-1} \boldsymbol{D}_\mathrm{c} - 2 \zeta_\mathrm{ex} \omega_\mathrm{ex}^2 \boldsymbol{w}_\mathrm{ex,red}^\top \right)
\end{bmatrix} \,, \\
C_\mathrm{fnl} &= m_\mathrm{ex} \boldsymbol{w}_\mathrm{ex,red}^\top \boldsymbol{M}_\mathrm{c}^{-1} \boldsymbol{w}_\mathrm{nl,red} \,, \\
D &= \left( 1 - m_\mathrm{ex} \boldsymbol{w}_\mathrm{ex,red}^\top \boldsymbol{M}_\mathrm{c}^{-1} \boldsymbol{w}_\mathrm{ex,red} \right) \frac{G}{R} \,.
\end{align*}
```
