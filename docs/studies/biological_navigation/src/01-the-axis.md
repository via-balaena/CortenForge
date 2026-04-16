# The Axis: Reynolds Number

## Why Reynolds Number Is the Right Axis

The Reynolds number (Re) is a dimensionless ratio of inertial to viscous forces in a fluid:

$$
\text{Re} = \frac{\rho U L}{\mu}
$$

where ρ is fluid density, U is velocity, L is characteristic length, and μ is dynamic viscosity.

It is the natural organizing axis for biological locomotion strategies because it determines what physics dominates at a given scale and speed. At low Re, viscosity dominates — the fluid has no memory of past motion, and every perturbation is immediately damped. At high Re, inertia dominates — perturbations persist as vortices, wakes, and turbulent structures that interact with the swimmer on timescales longer than the swimmer's own motion.

Crucially for our purposes: the ratio of signal timescale to noise timescale changes across the Re spectrum in a way that maps directly onto the thermodynamic X-encoding problem. At low Re, signal and noise are on the same timescale — you cannot separate them and must encode in the topology of motion. At intermediate Re, both are relevant simultaneously — you need multimodal switching. At high Re, noise generates coherent structures that can be exploited — speed itself becomes a source of control authority.

The Re spectrum spans roughly 13 orders of magnitude in biology, from bacteria at Re ~ 10⁻⁵ to blue whales at Re ~ 10⁸. The locomotion strategies are not a continuum — they are discrete regimes separated by qualitative phase transitions in the physics.

## Why Three Is Not the Golden Number

The initial framing identified three exemplars: octopus (inertial, distributed computation), dragonfly (predictive internal models), peregrine (vortex-noise coupling). The full spectrum contains at least five qualitatively distinct regimes, each with a different dominant strategy. Three of the five have been experimentally validated on Ising chain models (see the Validated Results section); the other two are hypothesized but untested.

The transition zones between regimes may be as important as the regimes themselves — real thermodynamic circuits will operate across throughput ranges, and failure modes will likely occur at the transitions.
