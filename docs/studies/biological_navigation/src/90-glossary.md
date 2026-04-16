# Glossary

**Chemotaxis:** Navigation by a chemical gradient, characteristic of bacteria including E. coli.

**CheY-P:** Phosphorylated CheY protein in E. coli; the signaling molecule that controls flagellar motor switching between run and tumble states. Follows Langevin dynamics with intrinsic noise that enhances gradient sensitivity.

**Energy-based model (EBM):** A machine learning model that defines the shape of a probability distribution via an energy function. The foundational mathematical object in thermodynamic computing.

**Geometric phase / holonomy:** In the context of low-Re locomotion, the net displacement achieved by a closed loop in configuration space. Formally equivalent to the holonomy of a connection on a fiber bundle (Shapere-Wilczek formalism).

**Karman gait:** A swimming behavior in fish involving synchronization to the oscillating flow of a Karman vortex street behind a cylinder; allows passive energy harvesting from environmental turbulence.

**Karman vortex street:** Alternating, periodic vortices shed downstream of a bluff body in steady flow.

**Langevin dynamics:** A stochastic differential equation describing the evolution of a system subject to deterministic drift plus Gaussian noise:

$$
dX = -\nabla U(X)\,dt + \sqrt{2kT}\,dW
$$

where U is the energy function and dW is Wiener process noise.

**Lateral line:** An array of mechanosensory organs distributed along the body of fish and aquatic amphibians, detecting local pressure gradients and flow velocity. Enables vortex detection and Karman gaiting.

**Metachronal wave:** Sequential, coordinated beating of an array of appendages (cilia, pleopods, etc.) with a phase lag between adjacent elements, producing a traveling wave appearance. Characteristic of ctenophores, krill, and many other invertebrates.

**Muscular hydrostat:** A muscular structure with no rigid skeleton (e.g., octopus arm, elephant trunk, human tongue) in which muscles provide both structural support and actuation, enabling infinite degrees of freedom.

**Proportional navigation (PN):** A guidance law in which turning rate is commanded proportional to the angular rate of the line-of-sight to target. Used by most guided missiles; proven in peregrine falcon and dragonfly prey capture. Optimal navigation constant N ~ 3 minimizes control effort to intercept a non-maneuvering target.

**Purcell's scallop theorem:** At low Reynolds number (Re < 1), any reciprocal body motion (time-reversible) produces zero net displacement. Net locomotion requires non-reciprocal motion — sequences that trace a closed loop enclosing nonzero area in configuration space.

**Reynolds number (Re):** Dimensionless ratio of inertial to viscous forces in fluid flow: Re = ρUL/μ. The fundamental organizing parameter for fluid dynamics and biological locomotion strategies.

**Run-and-tumble:** E. coli's primary locomotion strategy. "Run" = all flagella rotating CCW, forming a bundle → straight motion. "Tumble" = one or more flagella switching to CW rotation → random reorientation.

**Stochastic Processing Unit (SPU):** Normal Computing's first prototype thermodynamic computer; an 8-cell stochastic circuit on a PCB using RLC elements.

**Stochastic resonance:** The phenomenon by which an intermediate level of noise improves signal detection or transmission performance in a nonlinear system, beyond what is possible with either no noise or excessive noise.

**Thermodynamic Sampling Unit (TSU):** Extropic's hardware unit; a probabilistic circuit that produces samples from a programmable energy-based distribution. The thermodynamic computing analog of the GPU.

**X-encoding:** The problem of translating a logical input X into the initial or boundary conditions of a physical stochastic system such that the system's relaxation toward equilibrium produces the correct output distribution Y. The primary unsolved problem in thermodynamic computing hardware design.
