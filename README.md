# NLtest
NLtest is a Matlab/Simulink-based toolbox for nonlinear vibration testing.

Vibration testing relying on linear theory is standard practice in both industry and academia. Computational methods for model-based nonlinear vibration analysis are readily applicable. Vibration testing methods for their experimental validation, on the other hand, are still under highly active research. With NLtest, we aim to provide academic research groups consolidated methods for nonlinear vibration testing.

At present, NLtest includes the tool *Backbone Tracker* and post-processing methods for Open Data from our lab, as described below. We intend to extend the publicly available version of NLtest towards experimental continuation of frequency response curves, and selected methods for other forms of nonlinear vibration testing in the future.

## Backbone Tracker

![BackboneTracker](DOC/backbone_tracker.svg)

Backbone tracking is useful for extracting amplitude-dependent modal parameters (modal frequency, damping ratio, deflection shape) of well-separated modes [Hippold.2024], and uncovering isolated frequency response branches [Woiwode.2024].

The *Backbone Tracker* is a state-of-the-art implementation of phase-locked-loop-based experimental continuation of phase-resonant backbone curves.
The phase lag between response and excitation, and the response amplitude are estimated using adaptive filters based on Widrow’s least-mean-squares algorithm.
Simple proportional-integral controllers are used, the gains of which are selected in a theory-driven way relying only on standard vibration tests.
Once phase resonance is achieved at the (first) backbone point on, a stepwise in-/decrease of the excitation level is automatically triggered to reach the next point along the backbone curve.
As a variant, the tool features stepping the response amplitude (in contrast to stepping the input voltage) using an additional amplitude control loop.
The tool largely relies on the theory described in [Hippold.2024].

The tool is limited to primary resonances driven by force excitation. An extension towards base excitation, and a combination with the iteration-free excitation harmonization from [Hippold.2025] are planned.
To run real tests on physical experiments, the tool relies on dSPACE hardware in its present form. But a transfer to other environments seems feasible.

To get started with the *Backbone Tracker*, please see `'DOC/BackboneTracking.md'`.

## Post-processing of Open Data

For a description of the Open Data, post-processing methods, and instructions how to run these, please see *README* file in folders `'EXAMPLES/POST01_TRChallenge'` and `'EXAMPLES/POST02_VIbeams'`, respectively.

## Contact Information and Developer Team

Do not hesitate to contact us if you have any questions:

Maren Scheel (maren.scheel@ila.uni-stuttgart.de); 
Malte Krack (malte.krack@ila.uni-stuttgart.de)

Besides us, the following people contributed to the development of *NLtest*: Patrick Hippold, Seigan Hayashi.

## Copyright and Licensing

The copyright rests with the developers named above.

The tool comes with absolutely no warranty. It is a free software, and can be redistributed and/or modified under the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version. For details, see http://www.gnu.org/licenses or the file LICENSE provided with the tool package.

## References

[Hippold.2024]	P. Hippold, M. Scheel, L. Renson, M. Krack (2024): Robust and fast backbone tracking via phase-locked loops, Mechanical Systems and Signal Processing. http://doi.org/10.1016/j.ymssp.2024.111670

[Woiwode.2024]	L. Woiwode, M. Krack (2024): Experimentally uncovering isolas via backbone tracking, Journal of Structural Dynamics. http://doi.org./10.25518/2684-6500.180