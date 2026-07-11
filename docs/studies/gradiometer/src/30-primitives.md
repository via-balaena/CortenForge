# SDK primitives for gradiometer co-design

This is the implementer's chapter. It answers two questions precisely: **what does the SDK
already provide**, and **what must be built** — each new piece named, tiered, and pinned to the
existing symbol it stands on. The rule is the same one the routing arc used: *extract/consume the
primitive, consumer-gated* — build a rung only when a real caller needs it.

## Inventory — what exists vs. what is missing

| Capability | Symbol / location | Status | Role in the gradiometer |
|---|---|---|---|
| Geometry kernel (implicit) | `Sdf`, `Solid` (`cf-geometry`, `cf-design`) | ✅ exists | the parametric proof-mass + housing geometry |
| Differentiable SDF field/gradient | `cf-design` `gradient.rs` / `param.rs` | ✅ exists | geometry parameters carried differentiably |
| **Mass from geometry** | `cf_design::mechanism::mass::mass_properties(solid, density, cell_size)` → mass, COM, inertia | ✅ exists | **foundation for self-gravity + matching** |
| Rigid multibody + accel sensors | `sim-core` forward dynamics + sensor pipeline | ✅ exists | the differential / platform-motion framing |
| Co-design optimizer | `cf-codesign` — `CoDesignProblem` trait, `optimize(problem, x0, cfg)`, `OptConfig`, `Normalized` | ✅ exists | **the loop the gradiometer objective plugs into** |
| **∇g forward model** | — | ❌ missing | geometry + mass → gradient tensor + self-gravity (Tier 1, crown jewel) |
| Modal analysis (eigen-freq $f_0$) | — (not in `sim-soft`) | ❌ missing | the resonant frequency the noise budget needs |
| Dissipation / $Q$ model | — (not in `sim-soft`) | ❌ missing | sets the noise floor (Tier 2, the hard part) |
| Electromechanical transduction | — | ⛔ out of scope | displacement→capacitance→voltage; the readout is not ours |

The shape of the work is now concrete: the geometry, mass, differentiability, and the co-design
loop are **already here**. The gradiometer-specific additions are a small, in-character set —
dominated by one crown-jewel primitive and one genuinely hard one.

## The primitive ladder

Each rung below is a consumer-gated addition. They are ordered by dependency, not urgency; the
build sequence and its gates are in [Chapter 4](40-program.md).

### Rung 1 — the ∇g forward model (Tier 1, crown jewel) — *build first*

**What:** given a source mass distribution (from CAD or an anomaly model) and the proof-mass
positions, compute the gravitational field $\mathbf{g}$ and the **gradient tensor**

$$\Gamma_{ij} = \frac{\partial^2 \Phi}{\partial x_i \partial x_j}, \qquad
\Phi(\mathbf{p}) = -G \int \frac{\rho(\mathbf{r})}{|\mathbf{p}-\mathbf{r}|}\, dV,$$

at each proof mass, **differentiable with respect to the geometry parameters**. Includes the
instrument's own self-gravity (Rung 3 is the same operator applied to the housing).

**Builds on:** `mass_properties` already integrates $\rho\,dV$ over an implicit field on a grid;
the ∇g model reuses that integration, replacing the mass/inertia accumulators with the field-and-
gradient accumulators. Same voxel walk, new kernel. Newtonian, well-posed, no new solver universe.

**Why first:** without it you are only designing a resonator. Everything downstream (self-gravity,
sensitivity, the objective) consumes it. This is the smallest in-character rung that unblocks the
program — the direct analog of the routing arc's `CatmullRomCurve`.

**Differentiability:** the field is a smooth function of geometry parameters (grid-integrated
$1/r$ kernels); gradients flow through the same `param` machinery the SDF gradient uses.

### Rung 2 — the differential / CMRR framing

**What:** the gradiometer *reads* the difference of two accelerometer channels. Model the two
proof masses as a differential pair, express the channel scale-factor mismatch $\varepsilon$ and
the common-mode rejection ratio, and expose $\varepsilon$ as a differentiable function of geometry.

**Builds on:** `sim-core` already provides rigid multibody dynamics and acceleration sensors. This
rung is a **gradiometer error-model wrapper** on top — not new physics, a framing layer: given the
two channels' responses, produce the differential output and its common-mode leakage.

**Consumer-gate:** built when the matching objective (Rung 5) needs a differentiable $\varepsilon$.

### Rung 3 — self-gravity-from-CAD

**What:** apply Rung 1's operator to the *instrument's own* mass distribution to get the
self-gravity gradient at the proof masses (~1000 E per [Chapter 1](10-feasibility.md)), and its
**sensitivity to thermal expansion** — so the structure can be optimized to null the *drift*, not
just the DC offset.

**Builds on:** Rung 1 + `mass_properties` (the housing is a `Solid` with density). The thermal-drift
term needs the geometry's response to a temperature field — a small coupling to the thermo track,
or a linear expansion model as a first cut.

### Rung 4 — the thermomechanical noise budget (Tier 2, the hard part)

**What:** the Eötvös/√Hz floor from Brownian noise via fluctuation-dissipation
($a_n = \sqrt{4 k_B T \omega_0 / (m Q)}$, [Chapter 1, Gate 2](10-feasibility.md#gate-2--is-the-signal-above-the-noise)). Requires the
proof-mass **resonant frequency $f_0$** (a modal/eigen-analysis of the flexure structure) and the
**quality factor $Q$** (the dissipation model: squeeze-film gas damping, thermoelastic loss, anchor
loss).

**Builds on:** no existing primitive — but the two pieces differ in difficulty. $f_0$ is a modal
analysis (an eigenproblem on the stiffness/mass matrices `sim-soft`'s tet FEM already assembles;
the eigenproblem itself is new, and a linear-elastic material path is needed for the stiff silicon
flexure — `sim-soft` is a NeoHookean *hyperelastic* soft-body solver). $Q$ is the genuinely missing
physics and the hard middle: some mechanisms are separate solvers (squeeze-film needs a thin-film
fluid model). **This rung, not the forward model, is what could cap the program at "pessimistic."** It is
gated accordingly ([Chapter 4, Gate 2](40-program.md#gate-2--the-q--dissipation-floor---the-genuinely-hard-part)).

### Rung 5 — the differentiable objective

**What:** combine sensitivity (Rung 1), matching $\varepsilon$ (Rung 2), self-gravity residual
(Rung 3), and the noise floor (Rung 4) into one scalar loss, and implement it as a
`cf_codesign::CoDesignProblem` so the existing `optimize(...)` driver co-designs the geometry.

**Builds on:** `cf-codesign` — the loop, `OptConfig`, and `Normalized` (loss scaling / log-space)
already exist. This rung is the objective, not the optimizer. It is the [payoff](50-payoff.md).

## What deliberately stays out

Per the mission scope and the gradiometer gap-map: the **analog readout electronics** (amplifier
and DAQ noise, feedback-loop *electronics*) and the **silicon micromachining process** (DRIE
sidewall angles, anisotropic single-crystal-Si material models) are different engines, not sensing
crates. CortenForge designs the mechanical transducer; it does not design the chip fab or the
amplifier. Keep that boundary sharp.
