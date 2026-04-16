Biological Navigation Spectrum as a Design Framework for X-Encoding in Thermodynamic Computing
Via Balaena LLC / CortenForge — Foundational Research Document v1

Status: Pre-experimental hypothesis generation
Date: April 2026
Classification: Open research — intended for public release


Preface
This document records the intellectual origin of a research program connecting biological locomotion control theory to the unsolved x-encoding problem in thermodynamic computing. It is written as a living document: a record of where the reasoning started, what has been established, and what remains open. It is not a paper. It is the foundation from which papers will be written.
The central claim is this: the full spectrum of biological strategies for navigating chaotic physical media — from bacteria to peregrine falcons — constitutes a map of design principles for injecting logical inputs into nonequilibrium stochastic physical systems. Each regime of the biological spectrum corresponds to a distinct engineering strategy. The map is not complete. But enough of it has been identified to constitute a serious research program.

Part I: The Problem
1.1 Thermodynamic Computing and the Y Problem
Thermodynamic computing is a paradigm in which computation is performed not by enforcing deterministic logic states, but by allowing a physical system to relax toward thermodynamic equilibrium, reading the resulting probability distribution as the output. The key entities are:

The energy function F: defines the shape of the target probability distribution over output states
Y: the target distribution itself — the answer the system is supposed to produce
The physical substrate: a stochastic system (resistor-inductor-capacitor networks, analog probabilistic circuits, or future purpose-built thermodynamic chips) whose natural dynamics under Langevin noise are governed by F

The state of the art as of 2025–2026 includes Normal Computing's CN101 — the world's first taped-out thermodynamic semiconductor chip — and Extropic's XTR-0 development platform, both of which demonstrate that thermodynamic sampling units (TSUs) can perform AI inference tasks including matrix inversion and Gaussian sampling at energy efficiencies orders of magnitude beyond conventional GPUs.
The Y problem, to first approximation, is solved. Energy-based models provide a formal language for defining Y. Boltzmann statistics guarantee that a system in thermal equilibrium will sample from Y. The physics of the output is understood.
1.2 The X Problem — The Unsolved Root
The unsolved problem is X: the input encoding.
In a classical digital computer, encoding an input X is trivial — you set a voltage high or low. The physical act of encoding is decoupled from the physics of computation. In a thermodynamic computer, there is no such decoupling. The input X must be encoded as the initial conditions or boundary conditions of a physical stochastic system. The system then relaxes — through a nonequilibrium trajectory governed by Langevin dynamics — toward (hopefully) the correct distribution Y.
The problem is that this relaxation trajectory is sensitive to how X is injected. An imperfect encoding perturbs the energy landscape, potentially steering the system toward the wrong basin of attraction. At small scale, this can be managed empirically. At scale — as the circuit grows in size and the noise profile shifts — there is no formal theory telling an engineer how to inject X such that the relaxation remains correct. Every scaling step requires empirical re-tuning.
This is not a materials problem or a fabrication problem. It is a theory problem: we lack a formal design language for X-encoding in nonequilibrium stochastic systems. Until that language exists, thermodynamic computing cannot be engineered the way digital computing is engineered — from first principles, with predictable scaling behavior.
1.3 The Deeper Root: The Gap in Nonequilibrium Statistical Physics
The theory problem traces to a gap in physics. Equilibrium statistical physics is well-understood. Landauer's bound gives the minimum energy cost of erasing a bit. Boltzmann statistics describe the equilibrium distribution. But thermodynamic computers operate far from equilibrium — they must complete a computation quickly, which means they cannot afford to wait for true equilibrium to be reached.
The stochastic thermodynamics of far-from-equilibrium systems is a field in active development. Thermodynamic uncertainty relations now provide bounds on computation speed, noise level, and energy cost. But a constructive theory — one that tells you how to build a nonequilibrium system that reliably encodes X and converges to Y — does not yet exist.
The analogy: we know the speed limit (Landauer's bound, uncertainty relations), but we have no road map.
1.4 The Core Hypothesis of This Research Program
The hypothesis is that biological evolution has already solved versions of the X-encoding problem across a continuous spectrum of operating regimes, and that the biological solutions can be formally mapped to engineering principles for thermodynamic circuit design.
Specifically: every organism that navigates a chaotic physical medium — fluid, air, a stochastic chemical gradient — is solving a version of the same problem. It must inject "intent" (a direction, a target, a behavioral goal) into a noisy physical system (its own body in a turbulent medium) and achieve reliable convergence to the desired output state, at some throughput level, with finite energy. The physics of the problem is structurally isomorphic to X-encoding in thermodynamic computing.
The biological world has explored this design space for hundreds of millions of years. We should read the solutions it found.

Part II: The Axis
2.1 Why Reynolds Number Is the Right Axis
The Reynolds number (Re) is a dimensionless ratio of inertial to viscous forces in a fluid:

Re = ρUL / μ

where ρ is fluid density, U is velocity, L is characteristic length, and μ is dynamic viscosity.
It is the natural organizing axis for biological locomotion strategies because it determines what physics dominates at a given scale and speed. At low Re, viscosity dominates — the fluid has no memory of past motion, and every perturbation is immediately damped. At high Re, inertia dominates — perturbations persist as vortices, wakes, and turbulent structures that interact with the swimmer on timescales longer than the swimmer's own motion.
Crucially for our purposes: the ratio of signal timescale to noise timescale changes across the Re spectrum in a way that maps directly onto the thermodynamic X-encoding problem. At low Re, signal and noise are on the same timescale — you cannot separate them and must encode in the topology of motion. At intermediate Re, both are relevant simultaneously — you need multimodal switching. At high Re, noise generates coherent structures that can be exploited — speed itself becomes a source of control authority.
The Re spectrum spans roughly 13 orders of magnitude in biology, from bacteria at Re ~ 10⁻⁵ to blue whales at Re ~ 10⁸. The locomotion strategies are not a continuum — they are discrete regimes separated by qualitative phase transitions in the physics.
2.2 Why Three Is Not the Golden Number
The initial framing of this research identified three biological exemplars: the octopus (low speed, distributed local computation), the dragonfly (medium speed, predictive internal models), and the peregrine falcon (high speed, vortex-noise coupling). These three are real and important. But three is not the complete picture.
The full spectrum contains at least five qualitatively distinct regimes, each with a different dominant strategy, and the transition zones between regimes may be as important as the regimes themselves. This is because real thermodynamic circuits will not sit cleanly in one regime — they will operate across throughput ranges, and the failure modes will likely occur at the transitions.

Part III: The Five Regimes
Regime 1: Re < 1 — The Viscosity-Dominated Regime
Exemplar: Escherichia coli
The Physics
At Re < 1, the Navier-Stokes equations simplify to the Stokes equations — linear, time-reversible, with no inertial terms. The fluid has no memory. A swimmer moving forward and then backward through the exact same sequence of shapes returns to exactly its starting position. This is Purcell's scallop theorem (1977): in a time-reversible fluid, any reciprocal motion (one that looks the same played forward and backward) produces zero net displacement. It does not matter how fast or slow the motion is performed — speed is irrelevant.
The implication is profound: at low Re, you cannot encode information in the amplitude or timing of a perturbation. Amplitude scales out. Timing scales out. The only thing that survives is the topology of the motion sequence — whether it traces a closed loop in configuration space that encloses a nonzero area. This is geometric phase, formalized by Shapere and Wilczek (1989) using the language of gauge theory and fiber bundles.
What E. coli Does
E. coli navigates chemical gradients (chemotaxis) at Re ~ 10⁻⁵ using the run-and-tumble strategy. Multiple flagellar motors rotate counter-clockwise to bundle the flagella into a helical propeller (a "run" — straight motion). When one or more motors switch to clockwise rotation, the bundle unbundles and the cell reorients randomly (a "tumble"). By extending runs when moving up a chemical gradient and shortening them when moving down, the cell executes a biased random walk toward attractants.
The critical insight: the CheY-P signaling molecule that controls motor switching follows Langevin dynamics — it has intrinsic stochastic fluctuations described by:

dY/dt = -(Y - Y₀)/τ_Y + η(t)

where η(t) is Gaussian white noise. Crucially, signaling noise enhances chemotactic drift. An intermediate level of noise in the slow methylation dynamics improves gradient-climbing performance — not despite the noise, but because of it. The noise is not a contaminant to be suppressed; it is a functional component of the control strategy.
Furthermore, E. coli achieves signal amplification of more than 50-fold: a 2% change in receptor occupancy produces a 100% change in motor output. This nonlinear amplification emerges from the cooperative structure of the receptor-kinase complex operating near a phase transition — a bifurcation point at which sensitivity is maximized.
The X-Encoding Principle: Topological Encoding
Principle 1: In the viscosity-dominated regime, amplitude and rate are irrelevant. Information must be encoded in the sequence structure — the topology — of the injection. A closed loop in the injection parameter space that encloses nonzero area produces net displacement in the output state; a reciprocal sequence produces nothing.
Applied to thermodynamic circuits: At very low throughput rates where the circuit is effectively at quasi-static equilibrium, X-encoding cannot rely on amplitude modulation. The injection must trace a non-reciprocal path in parameter space. The geometric phase associated with that path determines the encoding fidelity. This is a formal statement that connects X-encoding theory to the mathematics of holonomy and gauge theory.
Principle 2: Stochastic resonance is available. An optimal noise level enhances encoding fidelity by improving sensitivity near the bifurcation point. The circuit should be tuned to operate in this regime rather than attempting to minimize noise.
Key References:

Purcell, E.M. "Life at Low Reynolds Number." American Journal of Physics 45 (1977)
Shapere, A. & Wilczek, F. "Geometry of Self-Propulsion at Low Reynolds Number." Journal of Fluid Mechanics 198 (1989)
Flores, M. et al. "Signalling Noise Enhances Chemotactic Drift of E. coli." Physical Review Letters 109 (2012)
Mattingly, H.H. et al. "Escherichia coli Chemotaxis is Information Limited." Nature Physics 17 (2021)


Regime 2: Re ~ 1–1000 — The Intermediate Regime
Exemplar: Ctenophores (Bolinopsis vitrea) and Water Boatmen (Corixidae)
The Physics
This is the most neglected and arguably most important regime. Both inertia and viscosity play significant and comparable roles simultaneously. There is no clean simplification. The governing equations are the full nonlinear Navier-Stokes, but at scales where both viscous and inertial terms contribute non-negligibly to thrust and drag. Crucially: thrust and drag are governed by different forces — at Re ~ 10², thrust is generated primarily by inertial forces while drag is still significantly viscous. No single strategy dominates.
The scallop theorem still technically applies (reciprocal motion produces no net displacement), but the breakdown of the theorem begins in this regime as inertial effects introduce memory into the fluid. A swimmer leaving a wake disturbs the flow it will encounter on its return, breaking exact time-reversibility.
What Ctenophores Do
Ctenophores are among the oldest animals on Earth and the largest animals that use cilia to swim. Their ctene rows (arrays of fused cilia) beat in a metachronal wave — sequential, coordinated beating with a phase lag between adjacent appendages, creating the appearance of a traveling wave moving down each row. This strategy is notably distinct from either the purely viscous strategy of small ciliates or the inertial undulation of fish.
The metachronal wave achieves several things simultaneously: it creates non-reciprocal motion (satisfying the kinematic constraint of the scallop theorem while inertia begins to provide memory), it generates hydrodynamic interactions between adjacent paddles that increase efficiency beyond what any single paddle could achieve alone, and it provides omnidirectional maneuverability by independently modulating different ctene rows.
Critically: ctenophores can perform tight turns while maintaining forward swimming speeds close to 70% of their maximum — a performance metric comparable to or exceeding vertebrates with far more complex locomotor systems. This is multimodal switching in action: the system does not stop and reorient, it redistributes effort across different control surfaces in real time.
What Water Boatmen Do
Water boatmen (Corixidae, Re ~ 10–200) operate in the most confused part of the intermediate regime. They use drag-based paddling with asymmetric power and recovery strokes — a direct application of temporal asymmetry to break time-reversibility. Their energetic efficiency is lower than fish of comparable size, but they are trimodal: they can swim, walk, and fly, transitioning rapidly between locomotor modes as the physical environment demands. This mode switching is the characteristic strategy of the intermediate regime.
The X-Encoding Principle: Multimodal Switching and Temporal Asymmetry
Principle 3: In the intermediate regime, no single encoding strategy dominates. The optimal approach combines temporal asymmetry (power and recovery phases at different rates) with distributed spatial coordination (metachronal waves) and mode switching (dynamically selecting between encoding strategies as local conditions shift).
Applied to thermodynamic circuits: At intermediate throughput rates — fast enough that purely topological encoding is insufficient, slow enough that inertial coupling effects haven't emerged — the X-encoder must operate as a multimodal adaptive system. It should maintain multiple encoding strategies simultaneously and switch between them based on local observables of the circuit's stochastic state. This is the regime where a Kalman filter architecture is most directly applicable: continuously estimating which encoding mode is appropriate given the observed noise floor and relaxation rate.
Principle 4: Distributed coordination across multiple control points (metachronal wave) produces emergent efficiency that no single control point could achieve. Applied to multi-cell thermodynamic circuits: coordinating injection timing across circuit nodes with a phase lag (analogous to a metachronal wave) may produce higher encoding fidelity than simultaneous or independent injection.
Key References:

Byron, M.L. "Moving in the In-Between: Locomotion Strategies at Intermediate Reynolds Numbers." Princeton MAE Seminar (2022)
McHenry, M.J. et al. "The Hydrodynamics of Locomotion at Intermediate Reynolds Numbers." Journal of Experimental Biology 206 (2003)
Daniels, J. et al. "The Hydrodynamics of Swimming at Intermediate Reynolds Numbers in the Water Boatman." Journal of Experimental Biology 217 (2014)
Hoover, A.P. et al. "Omnidirectional Propulsion in a Metachronal Swimmer." PLOS Computational Biology (2023)


Regime 3: Re ~ 10³–10⁵ — The Inertial Distributed Regime
Exemplar: Octopus (Octopus vulgaris)
The Physics
Inertia dominates. Viscosity still contributes to drag but does not govern the locomotion strategy. The fluid has memory — perturbations persist and propagate. The swimmer generates wakes that interact with the environment on relevant timescales. But this is also the regime of high degrees of freedom: the control problem is not underdetermined (as at low Re where everything scales out) but overdetermined — there are more controllable degrees of freedom than there are dimensions of the desired output, and the control architecture must manage this redundancy efficiently.
What the Octopus Does
The octopus has eight arms with virtually infinite degrees of freedom. Each arm is a muscular hydrostat — no rigid skeleton, with muscles providing both structural support and actuation. The arm contains approximately 380,000 motor neurons distributed along its length, yet the brain controls all of them via only ~4,000 efferent nerve fibers — a compression ratio of roughly 100:1.
This is the key insight. The central brain does not specify each muscle's activation individually. It sends a compressed command that specifies the goal (reach toward target at location X), and the arm's own distributed neural circuitry handles the decomposition of that command into the specific muscle activations required to execute the motion in the specific stochastic fluid environment the arm is currently in.
The primary locomotion primitive is bend propagation: the brain initiates a bend at the base of the arm, which travels as a wave from base to tip. Different reaching movements vary in speed and distance but maintain a basic invariant velocity profile scaled appropriately. The same motor program, scaled, generates all reaches. This scale-invariance is fundamental — it means the encoding strategy does not need to be re-designed for every target; one parametric program covers the full range.
Sensory information flows back in the opposite direction: ~2.3 million receptors distributed along the arm send information to the brain via only ~17,500 afferent fibers. Most sensory processing happens locally in the arm's peripheral nervous system. The brain receives a summary, not raw data.
The architecture: compressed high-level command → distributed local decoding → precise physical execution → compressed sensory summary → updated command.
The X-Encoding Principle: Compressed Command and Distributed Local Decoding
Principle 5: In the inertial distributed regime, the optimal architecture separates the encoding problem into two layers: a compressed high-level specification of the desired output (what state to reach) and a distributed local computation layer that translates that specification into physical actions appropriate to local noise conditions. The central controller does not need to know the noise floor at every circuit node; the local layer adapts.
Applied to thermodynamic circuits: The X-encoder broadcasts a compressed description of the target energy basin to all circuit nodes. Each node runs a local computation (analogous to the arm's ganglion) that determines how to modulate its local Langevin dynamics to steer toward that basin, given the local noise statistics it observes. The circuit's distributed neural computation — its local stochastic dynamics — handles the translation. This is the thermodynamic analog of bend propagation.
Principle 6: Scale-invariance of the encoding program. A parametric injection wave that can be scaled in amplitude and velocity to cover a range of target states (analogous to the octopus's invariant velocity profile) is more powerful than a lookup table of specific injection sequences for each target. Scale-invariance is also a prerequisite for scaling the circuit itself — an encoding strategy that must be re-tuned for each circuit size is not an engineering tool.
Key References:

Gutfreund, Y. et al. "Organization of Octopus Arm Movements: A Model System for Studying the Control of Flexible Arms." Journal of Neuroscience 16 (1996)
Sumbre, G. et al. "Octopuses Use a Human-like Strategy to Control Precise Point-to-Point Arm Movements." Current Biology 16 (2006)
Levy, G. et al. "Motor Control in Soft-Bodied Animals." Current Biology 25 (2015)
Davenport, J.S. et al. "Lessons for Robotics from the Control Architecture of the Octopus." Frontiers in Robotics and AI (2022)
Mischiati, M. et al. "Neural Models and Algorithms for Sensorimotor Control of an Octopus Arm." arXiv (2024)


Regime 4: Re ~ 10⁵–10⁷ — The Predictive Inertial Regime
Exemplar: Dragonfly (Plathemis lydia)
The Physics
Full inertial regime. Viscosity contributes to drag but not meaningfully to the locomotion control problem. The fluid generates persistent wakes and vortices. The key physical constraint is latency: at these speeds, the time for a reactive signal to propagate through the nervous system and execute a motor command is comparable to or longer than the time in which the target moves a significant fraction of the control distance. A pure reactive strategy fails — by the time the correction executes, it is already wrong.
This is the latency gap regime: you are moving fast enough to lose the distributed local computation advantage (the arm can't adapt fast enough), but not fast enough to generate the vortex-lift control authority that defines the next regime up. It is the most computationally demanding regime for a biological or engineered controller.
What the Dragonfly Does
Dragonflies achieve a prey capture success rate of 90–95% in the wild — the highest of any known predator. They do this while intercepting aerobatic prey in mid-air at speeds up to several meters per second, in turbulent air, with a nervous system containing roughly 1 million neurons total.
The mechanism was established by Mischiati et al. (Nature, 2015): dragonfly interception steering is driven by forward and inverse internal models of the dragonfly's own body dynamics and of the prey's predicted trajectory. Predictive rotations of the head continuously track the prey's angular position. The head-body angle thereby established guides systematic rotations of the body to align with the prey's predicted flight path. Model-driven control underlies the bulk of maneuvers; reactive control is reserved specifically for unexpected prey movements.
The guidance law was characterized by Brighton et al. (PNAS, 2017, working with peregrine falcons but confirmed analogously for dragonflies): proportional navigation (PN). Under PN, turning rate is commanded proportional to the angular rate of the line-of-sight to target:

ω_commanded = N × (dλ/dt)

where λ is the line-of-sight angle and N is the navigation constant (feedback gain). This is the same guidance law used by most visually guided missiles. For dragonflies, N ≈ 3, which coincides with the classical linear-quadratic optimal guidance result: PN with N = 3 minimizes control effort to intercept a non-maneuvering target.
Before takeoff, the dragonfly performs a critical pre-selection step: it assesses whether the prey's angular size and velocity co-vary within a privileged range, and times its takeoff to predict when the prey will cross its zenith. The behavioral decision embeds the computational constraints — the dragonfly only pursues prey it has pre-verified it can intercept given its own body dynamics.
The minimum sensory requirement is also remarkable: two local observables — vertical wind acceleration and torque (body rotation rate) — are sufficient to implement the PN guidance law. Global knowledge of the flow field is not required.
The X-Encoding Principle: Predictive Internal Model with Reactive Fallback
Principle 7: In the latency-gap regime, the X-encoder must operate predictively. It cannot react to observed drift in the circuit's state because the signal propagation latency is too long for reactive correction to be useful. Instead, it maintains a forward model of the circuit's own relaxation dynamics — predicting where the system will be at time t+Δ and injecting X at the predicted future state rather than the current observed state. The injection leads the relaxation rather than chasing it.
Applied to thermodynamic circuits: A forward model of the Langevin dynamics, calibrated from observed circuit statistics, predicts the evolution of the energy landscape for the next several time steps. X-injection is computed against the predicted state. When prediction error exceeds a threshold (unexpected stochastic events), the system falls back to reactive injection temporarily, then returns to model-driven control.
Principle 8: Pre-selection and operating regime commitment. Before entering high-throughput operation, the X-encoder should verify that the circuit's current noise profile and relaxation rate are within the regime for which the encoding strategy was designed (analogous to the dragonfly's pre-takeoff assessment). Attempting to encode outside the designed regime is the primary failure mode.
Principle 9: Minimum observables sufficiency. Two local scalar cues are sufficient for full trajectory guidance. The X-encoder instrumentation can therefore be minimal: local energy gradient and local curvature (or equivalent observable pair) fed into a PN-style feedback law. This is a tractable instrumentation problem, not a global state estimation problem.
Key References:

Mischiati, M. et al. "Internal Models Direct Dragonfly Interception Steering." Nature 517 (2015)
Brighton, C.H. et al. "Terminal Attack Trajectories of Peregrine Falcons are Described by the Proportional Navigation Guidance Law of Missiles." PNAS 114 (2017)
Mills, R. et al. "Physics-Based Simulations of Aerial Attacks by Peregrine Falcons Reveal that Stooping at High Speed Maximizes Catch Success." PLOS Computational Biology 14 (2018)
Combes, S.A. "Neuroscience: Dragonflies Predict and Plan Their Hunts." Nature 517 (2015)
Gonzalez-Bellido, P.T. et al. "Eight Pairs of Descending Visual Neurons in the Dragonfly Give Wing Motor Centers Accurate Population Vector of Prey Direction." PNAS 110 (2013)


Regime 5: Re > 10⁷ — The High-Inertial Turbulent Regime
Exemplar: Peregrine Falcon (Falco peregrinus) during stoop
The Physics
At very high Re, the flow is turbulent. Vortices are generated spontaneously and persistently. The swimmer is not avoiding or suppressing turbulence — it is immersed in a chaotic vortex field of its own and the environment's making. The conventional engineering assumption — that noise degrades control authority at high speed — breaks down entirely. A new physics emerges: vortex-induced lift. At high enough speeds, deliberately shaped vortices generate aerodynamic forces larger than those available from conventional attached flow. Speed generates control authority rather than destroying it.
This is the most counterintuitive regime for engineers trained in digital systems, where noise is always the enemy. It is also where the most powerful and unexpected biological principle lives.
What the Peregrine Falcon Does
The peregrine is the fastest animal on Earth — exceeding 380 km/h in a stoop (hunting dive). At these speeds, it maintains not merely stable flight but precise, active maneuvering sufficient to intercept aerobatic prey. The mechanism was established by Gowree et al. (Communications Biology, 2018) and extended by Brücker and Gowree (AIAA Journal, 2021).
The stoop is a four-phase morphological sequence:
Phase I (Teardrop — T-shape): Wings folded completely, feathers tucked, legs retracted. Drag minimized. The falcon converts gravitational potential energy to kinetic energy with near-zero energy expenditure. Angle of attack maintained at ~5° — the equilibrium point where aerodynamic and gravitational forces balance. This phase is passive.
Phase II (Cupped wing — C-shape): Wings open slightly with primary feathers aligned vertically. Substantial lateral (side) forces generated — up to 3× body weight — enabling pure yaw control. Asymmetric morphing allows roll and heading correction. The strong vortices produced are aligned laterally, providing steering authority without significant deceleration.
Phase III (M-shape — terminal phase): The defining configuration. Wings deploy into a forward-swept M-shape. This is where the core physics lives.
The M-shape vortex field: Wind tunnel experiments and Large Eddy Simulations (LES) revealed a rich set of interacting vortex structures:

Horn / Werlé-Legendre vortices emanating from the frontal region due to strong spanwise flow promoted by the forward sweep of the radiale (wrist bone)
Dorsal vortex (DV) interacting with the horseshoe vortex (HSV) of the body
Wing vortex (WV) and tail vortex (TV) enhanced by M-shape geometry
Primary feather vortex (PFV) at the wingtip primaries

The critical discovery: a counter-rotating vortex pair interacts with the main wing vortex to reduce induced drag, which would otherwise decelerate the bird significantly during pullout. The vortices do not merely provide lift — they actively cancel each other's drag penalty. The chaos is not fought; it is structured so that one layer cancels another.
Deliberate pitch instability: LES analysis confirmed that the falcon is flying unstably in pitch during the M-shape phase — positive pitching moment slope at trim angle of attack ~5°. This is a feature, not a flaw. Pitch instability maximizes responsiveness: a small input produces a large output change. The hand wings (primaries) act as "elevons" — stabilizing the intentionally unstable configuration while preserving its high-sensitivity property.
The guidance law: Brighton et al. (PNAS, 2017) confirmed using GPS loggers and onboard cameras that terminal attack trajectories follow the proportional navigation guidance law (shared with the dragonfly) but with a lower navigation constant N < 3, appropriate to the lower flight speed relative to missiles and accounting for higher biological sensor latency. Monte Carlo simulation confirmed N ≈ 3 as the optimum for high-speed stoops against agile prey.
Physiological substrate supporting high-throughput precision:

Nasal tubercles regulate respiratory pressure at >200 mph
Visual acuity ~4× human density; 150 fps processing rate; dual fovea (forward shallow + lateral deep)
Nictitating membrane clears debris without interrupting vision
Reinforced arm skeleton and shoulder girdle (~2–3× bone mass of comparable raptors) to sustain 3g+ load factors
Talon reflex arc bypassing conscious processing: impact → grip in ~15ms (vs. 200ms human reaction time)

The X-Encoding Principle: Constructive Vortex-Noise Coupling
Principle 10: In the high-inertial turbulent regime, speed itself generates control authority. The appropriate strategy is not to suppress noise but to deliberately inject structured perturbations that generate counter-rotating vortex pairs whose mutual interaction cancels drag while preserving lift (control force). Higher throughput produces more vortex-induced force available for steering — the precision-throughput relationship inverts.
Applied to thermodynamic circuits: At high injection rates, the X-encoder should deliberately introduce paired perturbation structures into the circuit's Langevin noise field — perturbations designed so that their stochastic cross-correlation produces net drift toward the target energy basin, while their self-canceling structure minimizes the energy dissipation (entropy production) associated with the injection. This is the thermodynamic analog of the counter-rotating vortex pair.
Principle 11: Deliberate instability as a sensitivity amplifier. A circuit operating near a phase transition (bifurcation) in its energy landscape is pitch-unstable in the analogy — highly sensitive to perturbations. The X-encoder should target this operating point and use a minimal stabilization mechanism (the circuit analog of elevon primaries) to prevent divergence while preserving the high-sensitivity regime.
Principle 12: The logarithmic spiral as a scale-invariant approach geometry. Peregrine falcons resolve the conflict between aerodynamic streamlining (head straight) and maximum visual acuity (head turned 40°) by flying a logarithmic spiral path — a constant-angle curve that is self-similar at every scale. A scale-invariant X-injection trajectory would not require re-tuning as the circuit scales, which is the central engineering requirement for thermodynamic computing to become a manufacturable technology.
Key References:

Gowree, E.R. et al. "Vortices Enable the Complex Aerobatics of Peregrine Falcons." Communications Biology 1 (2018)
Brücker, C. & Gowree, E.R. "Peregrine Falcon's Dive: Pullout Maneuver and Flight Control Through Wing Morphing." AIAA Journal 59 (2021)
Brighton, C.H. et al. "Terminal Attack Trajectories of Peregrine Falcons are Described by the Proportional Navigation Guidance Law of Missiles." PNAS 114 (2017)
Tucker, V.A. "Curved Flight Paths and Sideways Vision in Peregrine Falcons." Journal of Experimental Biology 203 (2000)
Mills, R. et al. "Physics-Based Simulations of Aerial Attacks by Peregrine Falcons." PLOS Computational Biology 14 (2018)


Part IV: The Transition Zones
The five regimes and their principles are the main result. But the transition zones between them are arguably the most important research target, because:

Real thermodynamic circuits will operate across a range of throughput levels, necessarily crossing multiple regime boundaries.
The biological exemplars suggest that transitions between regimes are discontinuous — there is no smooth interpolation of strategies, and performance degrades sharply in the transition zone before a new strategy takes over.
The intermediate regime (Regime 2) exists precisely because neither low-Re nor high-Re strategies work there, and the organisms that inhabit it (ctenophores, water boatmen) are notably less efficient than those that specialize in either adjacent regime.

The transition zone hypothesis: In thermodynamic computing, as injection throughput increases from the regime of one strategy to the next, there will be a characteristic throughput range where neither strategy works well — analogous to the intermediate Reynolds number regime. Identifying these critical throughput values for a given circuit architecture, and designing the mode-switching logic that handles the transitions, is a specific and tractable engineering problem that this research program can address through simulation.
Additionally, there is a sixth organism that deserves dedicated study for its unique role at the boundary between Regimes 3 and 4: the fish with lateral line. The lateral line is an array of mechanosensory organs distributed along the fish's body that detects local pressure gradients and vortex shedding frequencies with remarkable precision. Fish use the lateral line to perform Kármán gaiting — holding station in a turbulent vortex street behind a cylinder by passively synchronizing their body kinematics to the oscillating flow. This is energy harvesting from chaos: exploiting the environmental noise field as a source of free locomotion rather than fighting it. The lateral line provides the sensing architecture that makes this possible. The engineering analog is an in-situ noise characterization system embedded in the circuit itself, providing real-time observables to the X-encoder without interrupting computation.

Part V: Synthesis — The Unified Design Framework
5.1 The Complete Spectrum
RegimeRe RangeBiological ExemplarDominant StrategyX-Encoding AnalogViscosity-dominated< 1E. coliTopological encoding; geometric phase; stochastic resonanceEncode in injection sequence topology, not amplitude; exploit optimal noise levelIntermediate1–1,000Ctenophores, water boatmenTemporal asymmetry; metachronal coordination; mode switchingAsymmetric injection phases; distributed phase-lagged coordination; adaptive mode switchingInertial distributed10³–10⁵OctopusCompressed central command; distributed local decoding; propagating waveCompressed X-specification; local circuit-level decoding; scale-invariant bend propagationPredictive inertial10⁵–10⁷DragonflyPredictive internal model; PN guidance; minimum observable sufficiencyForward-model-guided injection; PN-style feedback; two local cues sufficientHigh-inertial turbulent> 10⁷Peregrine falconVortex-noise coupling; constructive counter-rotation; deliberate instabilityPaired perturbation structures; counter-rotating noise coupling; bifurcation-point operation
5.2 What Varies Across the Spectrum
What actually changes as one moves from low to high Re is not simply "speed." The underlying variable is the ratio of signal timescale to noise timescale (τ_signal / τ_noise):

Low Re: τ_signal ≈ τ_noise. They are indistinguishable. Encode in topology.
Intermediate Re: τ_signal and τ_noise are both relevant but different. Exploit both via multimodal switching.
Inertial distributed Re: τ_signal > τ_noise. You have time for local computation. Distribute the decoding.
Predictive inertial Re: τ_signal < τ_noise for reactive control but > τ_noise for predictive control. Precompute.
High inertial turbulent Re: τ_noise generates coherent structures on timescales accessible to the controller. Couple constructively.

This framing suggests that the correct axis for thermodynamic circuit design is not "throughput" but the dimensionless ratio τ_circuit / τ_noise — a circuit-level analog of the Reynolds number. Computing this ratio for a given circuit architecture and operating condition determines which encoding regime applies and which strategy should be deployed.
5.3 The Common Thread
Despite their apparent diversity, all five strategies share a single underlying principle: the environment's stochastic character is not an obstacle to be overcome but a resource to be exploited. Every biological exemplar either uses noise to amplify sensitivity (E. coli), harvests noise as a phase-coordination mechanism (ctenophores), treats noise as the medium through which a compressed command propagates (octopus), uses noise statistics to calibrate a forward model (dragonfly), or couples constructively with noise to generate control force (peregrine).
This is the deepest lesson for thermodynamic computing: the designers of thermodynamic circuits should not be trying to suppress Langevin noise. They should be designing X-encoders that make the noise work for them.

Part VI: Research Program and CortenForge Simulation Targets
6.1 Why CortenForge
CortenForge's simulation infrastructure — Langevin-capable physics simulation, Hill-type muscles, XPBD deformables, RL training loops, sensor fusion — provides the tools to simulate the biological exemplars directly and extract quantitative design parameters. The organisms are not analogies to be reasoned about abstractly; they are simulatable physical systems from which specific numbers can be extracted.
6.2 Priority Simulation Experiments
Experiment 1 — E. coli topological encoding validation
Simulate the run-and-tumble chemotaxis of E. coli in a Langevin noise field with a parameterized chemical gradient. Extract the relationship between injection sequence topology (area enclosed in configuration space) and convergence fidelity to the target distribution. Verify the stochastic resonance optimum — identify the noise level that maximizes encoding fidelity and characterize how it scales with gradient steepness.
Experiment 2 — Ctenophore metachronal phase-lag optimization
Simulate a row of N ctenes beating with a variable phase lag δ in both the viscous and intermediate Re regime. Measure thrust and encoding efficiency as a function of δ and Re. Identify whether there is a characteristic phase lag that maximizes efficiency at the viscous-inertial crossover. This translates directly to a design parameter for multi-node injection timing in thermodynamic circuits.
Experiment 3 — Octopus bend propagation compression ratio
Simulate octopus arm reaching using CortenForge's Hill-type muscle model with the 4,000:380,000 efferent compression architecture. Measure the minimum central command dimensionality required to achieve a given reaching accuracy in a stochastic fluid environment. Characterize how this compression ratio scales with arm length and noise level.
Experiment 4 — Dragonfly PN guidance in Langevin noise
Implement proportional navigation with N as a free parameter in a simulated Langevin particle navigating a 2D energy landscape toward a target basin. Vary N across [1, 5] and measure convergence speed and convergence fidelity as a function of noise level. Verify the N ≈ 3 optimum and characterize its robustness to noise floor variations. Extend to the forward-model variant: precompute the target basin's future position using the Langevin drift term, and measure the improvement in convergence.
Experiment 5 — Peregrine M-shape vortex coupling simulation
Simulate the M-shape vortex field in CortenForge's fluid simulation domain. Quantify the counter-rotating vortex pair's drag-cancellation ratio as a function of injection speed (analogous to dive speed). Verify the predicted monotonic increase in drag cancellation with speed. Translate the vortex geometry into a parameterized description of structured perturbation pairs for circuit injection.
Experiment 6 — Regime transition characterization
For each pair of adjacent regimes, simulate a Langevin circuit operating at the strategy of the lower regime and systematically increase the injection rate. Measure when performance begins to degrade and what the transition regime looks like. Identify the critical τ_circuit / τ_noise ratio at each transition. Map these to concrete circuit parameters (injection rate, noise floor, relaxation time) to provide actionable design guidance for where to switch encoding strategies.
Experiment 7 — Fish lateral line in-situ sensing
Implement a lateral-line-style sensor array in the simulated circuit: distributed local pressure sensors (equivalent to neuromasts) providing real-time vortex shedding frequency and local gradient information. Test whether this in-situ sensing reduces the number of pre-measurement calibration steps required before the X-encoder can operate effectively. Quantify the steady-state sensing accuracy as a function of array density and sensor placement.
6.3 Connections to Existing CortenForge Infrastructure
Simulation NeedCortenForge ModuleLangevin dynamicssim-core with Euler/Verlet/RK4 integratorsHill-type muscle for octopussim-physics Hill-type muscle modelsFluid-structure interactionsim-contact + sim-constraintRL for guidance law optimizationml-training (Burn)Sensor array (lateral line)sensor-fusion hardware-agnostic sensor types3D mesh reconstructionmesh-io, mesh-repairParametric geometry (M-shape wing)geometry/curve-types NURBSGPU-batched simulationssim-physics GPU batching

Part VII: Strategic and Positioning Notes
7.1 What Is Novel About This Program
The individual biological phenomena described in this document are established science. The engineering applications to specific domains (aeronautics, robotics) are well-explored. What is novel is:

The formal mapping from Reynolds number regimes to thermodynamic circuit encoding regimes. This connection has not been made in the literature.
The identification of five distinct biological strategies — not three, not a continuum — as a design basis for X-encoding. The prior art in bio-inspired computing tends to pick one or two biological exemplars; a systematic spectrum has not been proposed.
The specific hypothesis that constructive vortex-noise coupling (Regime 5) produces an inversion of the precision-throughput tradeoff. This is a falsifiable prediction with direct engineering implications.
The formalization of CortenForge as the simulation infrastructure for extracting quantitative design parameters from these biological systems. The biological exemplars are not metaphors here — they are simulatable.

7.2 Relationship to Current Thermodynamic Computing Research
The companies currently leading thermodynamic computing development (Extropic, Normal Computing) are focused on chip fabrication and algorithm development. The X-encoding problem is acknowledged but not yet formally addressed in the published literature. The biological spectrum framework is therefore complementary rather than competitive — it addresses the foundational theory question that enables the next generation of chip and algorithm design.
The research partnership programs of both companies (Extropic's algorithmic partnership form; Normal Computing's academic partnerships) are explicitly seeking exactly this kind of cross-domain theoretical contribution.
7.3 Open Source Philosophy
This research is released openly. The reasoning is straightforward: the value created by becoming the foundational reference for biological-inspired X-encoding theory exceeds the value of any IP protection that could be applied to the theoretical framework. The simulation infrastructure (CortenForge) is the durable asset. The research program generates citations, partnerships, and positioning that makes CortenForge indispensable to the thermodynamic computing ecosystem.
The precedent is clear: PyTorch, TensorFlow, LLVM. Open infrastructure that became the standard captured far more value than closed alternatives.

Appendix A: Glossary
Chemotaxis: Navigation by a chemical gradient, characteristic of bacteria including E. coli.
CheY-P: Phosphorylated CheY protein in E. coli; the signaling molecule that controls flagellar motor switching between run and tumble states. Follows Langevin dynamics with intrinsic noise that enhances gradient sensitivity.
Energy-based model (EBM): A machine learning model that defines the shape of a probability distribution via an energy function. The foundational mathematical object in thermodynamic computing.
Geometric phase / holonomy: In the context of low-Re locomotion, the net displacement achieved by a closed loop in configuration space. Formally equivalent to the holonomy of a connection on a fiber bundle (Shapere-Wilczek formalism).
Kármán gait: A swimming behavior in fish involving synchronization to the oscillating flow of a Kármán vortex street behind a cylinder; allows passive energy harvesting from environmental turbulence.
Kármán vortex street: Alternating, periodic vortices shed downstream of a bluff body in steady flow.
Langevin dynamics: A stochastic differential equation describing the evolution of a system subject to deterministic drift plus Gaussian noise:  dX = -∇U(X)dt + √(2kT)dW where U is the energy function and dW is Wiener process noise.
Lateral line: An array of mechanosensory organs distributed along the body of fish and aquatic amphibians, detecting local pressure gradients and flow velocity. Enables vortex detection and Kármán gaiting.
Metachronal wave: Sequential, coordinated beating of an array of appendages (cilia, pleopods, etc.) with a phase lag between adjacent elements, producing a traveling wave appearance. Characteristic of ctenophores, krill, and many other invertebrates.
Muscular hydrostat: A muscular structure with no rigid skeleton (e.g., octopus arm, elephant trunk, human tongue) in which muscles provide both structural support and actuation, enabling infinite degrees of freedom.
Proportional navigation (PN): A guidance law in which turning rate is commanded proportional to the angular rate of the line-of-sight to target. Used by most guided missiles; proven in peregrine falcon and dragonfly prey capture. Optimal navigation constant N ≈ 3 minimizes control effort to intercept a non-maneuvering target.
Purcell's scallop theorem: At low Reynolds number (Re < 1), any reciprocal body motion (time-reversible) produces zero net displacement. Net locomotion requires non-reciprocal motion — sequences that trace a closed loop enclosing nonzero area in configuration space.
Reynolds number (Re): Dimensionless ratio of inertial to viscous forces in fluid flow: Re = ρUL/μ. The fundamental organizing parameter for fluid dynamics and biological locomotion strategies.
Run-and-tumble: E. coli's primary locomotion strategy. "Run" = all flagella rotating CCW, forming a bundle → straight motion. "Tumble" = one or more flagella switching to CW rotation → random reorientation.
Stochastic Processing Unit (SPU): Normal Computing's first prototype thermodynamic computer; an 8-cell stochastic circuit on a PCB using RLC elements.
Stochastic resonance: The phenomenon by which an intermediate level of noise improves signal detection or transmission performance in a nonlinear system, beyond what is possible with either no noise or excessive noise.
Thermodynamic Sampling Unit (TSU): Extropic's hardware unit; a probabilistic circuit that produces samples from a programmable energy-based distribution. The thermodynamic computing analog of the GPU.
X-encoding: The problem of translating a logical input X into the initial or boundary conditions of a physical stochastic system such that the system's relaxation toward equilibrium produces the correct output distribution Y. The primary unsolved problem in thermodynamic computing hardware design.

Appendix B: Open Questions
The following are the highest-priority open questions identified by this research program, ordered by tractability:

What is the correct dimensionless ratio for thermodynamic circuits? We proposed τ_circuit / τ_noise as the analog of the Reynolds number. Is this the right formulation? What are τ_circuit and τ_noise precisely in terms of circuit parameters (resistance, inductance, capacitance, temperature)?
Are the five regimes discrete or continuous? The biological spectrum suggests discrete transitions, but this may be because biological organisms specialize rather than operate across regimes. A thermodynamic circuit might be engineered to have a smooth transition — or the transitions might be genuinely discontinuous (phase transitions in the physics sense).
What is the minimum encoding sequence length for topological encoding? The scallop theorem says you need at least two degrees of freedom sweeping a nonzero area. What is the minimum area required for a given encoding fidelity? How does this scale with circuit noise temperature?
Does the counter-rotating vortex pair principle have a direct electrical/stochastic analog? The peregrine's mechanism relies on specific vortex geometry. What is the equivalent concept in Langevin dynamics — two perturbation modes with opposite "rotation" in the energy landscape that cancel each other's dissipation while preserving their net drift?
How does the optimal noise level (stochastic resonance optimum) scale across the spectrum? In Regime 1 (E. coli), there is an optimal CheY-P noise level. Is there an analogous optimal noise temperature for each regime? Does it increase or decrease as throughput increases?
What are the failure modes at regime transitions? When a circuit running a Regime 3 strategy is pushed into the Regime 4 throughput range, what specifically breaks? Does the encoding fail gradually or catastrophically?
Can the fish lateral line be implemented as an X-encoder feedback system? Specifically: can a distributed array of local circuit observables (the lateral line analog) provide sufficient information for real-time X-encoder adjustment without requiring a global measurement step?


End of document v1
This document is a founding artifact of the Via Balaena / CortenForge research program on biological navigation principles for thermodynamic computing. It records the state of understanding as of April 2026 and will be updated as simulation results and theoretical developments warrant.