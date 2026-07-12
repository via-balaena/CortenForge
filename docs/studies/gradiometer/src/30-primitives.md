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
| **Anisotropic single-crystal-Si elasticity** | — (extends `sim-soft` material path) | ❌ missing | the flexure's true constitutive law — sets $f_0$, the Brownian floor, and orientation-driven pair mismatch |
| **Fab-geometry tolerance model** | — (parametric deviations on `Solid`) | ❌ missing | DRIE sidewall taper + scalloping + mask undercut as differentiable geometry params — the Gate 1 mismatch machinery |
| **Readout noise (referred-input)** | — | ❌ missing | capacitive-pickoff + amplifier noise as a differentiable budget term (gap, $C$, bias, $e_n$) — co-optimizable |
| Circuit synthesis / DAQ firmware / etch-process physics | — | ⛔ different engine | SPICE/EDA + Sentaurus's job, not a sensing crate — never grown here |

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

### Rung 4 — the noise budget: thermomechanical + readout (Tier 2, the hard part)

**What:** the Eötvös/√Hz floor, as the **quadrature sum of two terms**. (1) Brownian noise via
fluctuation-dissipation ($a_n = \sqrt{4 k_B T \omega_0 / (m Q)}$, [Chapter 1, Gate 2](10-feasibility.md#gate-2--is-the-signal-above-the-noise)),
requiring the proof-mass **resonant frequency $f_0$** (a modal/eigen-analysis of the flexure) and
the **quality factor $Q$** (dissipation: squeeze-film gas damping, thermoelastic loss, anchor loss).
(2) **Readout noise referred to the mechanical input** — a differentiable capacitive-pickoff model
(gap, overlap capacitance, bias, amplifier input-referred noise density $e_n$). The two add in
quadrature, and for room-temp capacitive MEMS the readout term is frequently *dominant*, so a
mechanical-only floor is an optimistic proxy, not the floor.

**Builds on:** no existing primitive — the pieces differ in difficulty. $f_0$ is a modal analysis
(an eigenproblem on the stiffness/mass matrices `sim-soft`'s tet FEM already assembles; the
eigenproblem itself is new, and a **linear-elastic, anisotropic single-crystal-Si** material path is
needed for the stiff silicon flexure — `sim-soft` is a NeoHookean *hyperelastic* soft-body solver).
$Q$ is the genuinely missing physics and the hard middle: some mechanisms are separate solvers
(squeeze-film needs a thin-film fluid model). The readout term is a physical *noise model*, not
circuit design — a few parameters, differentiable, co-optimized with the geometry. **This rung, not
the forward model, is what could cap the program at "pessimistic."** It is gated accordingly
([Chapter 4, Gate 2](40-program.md#gate-2--the-q--dissipation-floor---the-genuinely-hard-part)).

### Rung 5 — the differentiable objective

**What:** combine sensitivity (Rung 1), matching $\varepsilon$ (Rung 2), self-gravity residual
(Rung 3), and the noise floor (Rung 4) into one scalar loss, and implement it as a
`cf_codesign::CoDesignProblem` so the existing `optimize(...)` driver co-designs the geometry.

**Builds on:** `cf-codesign` — the loop, `OptConfig`, and `Normalized` (loss scaling / log-space)
already exist. This rung is the objective, not the optimizer. It is the [payoff](50-payoff.md).

## Where the scope line actually falls

The boundary is drawn by **coupling to the figure of merit**, not by organizational category
("that's fab," "that's the amplifier"). The test for every term is: *does the feasibility / matching
answer move if we model it?* If yes it comes in — at the cheapest fidelity that captures the
sensitivity, and **differentiable**, so it co-optimizes on the same tape. Only what would require
standing up a mature external engine's whole job stays out. Three buckets, not two:

**In — coupled inputs, modeled cheap and differentiable:**

- **Anisotropic single-crystal-Si elasticity.** Not fab — the *constitutive law of the transducer*.
  Si runs ~130 GPa ⟨100⟩ to ~169 GPa ⟨110⟩; flexure stiffness sets $f_0$, the Brownian floor (via
  $k$), and — decisively — the pair mismatch's sensitivity to crystal-axis misalignment. Model it
  isotropic and Gate 1 (matching-under-tolerance) is unanswerable.
- **Fab-geometry tolerances** — DRIE sidewall taper (~89–90.5°), scalloping, mask undercut — as
  *parametric deviations on the geometry*, not a process simulation. This is the Gate 1 mismatch
  machinery, first-class, not a proxy for something out of scope.
- **Readout noise** as a differentiable *referred-input* budget term (pickoff gap, overlap
  capacitance, bias, amplifier input-referred noise density $e_n$). The device's floor is
  thermomechanical **plus** readout in quadrature, and for room-temp capacitive MEMS the readout
  term is frequently *dominant* — so a mechanical-only floor is an optimistic proxy, not the floor.
  It enters as a physical noise model, never as circuit design.

**Deferrable for the feasibility pass, needed later:** closed-loop force-rebalance electromechanics
and electronics thermal drift. Open-loop is the right first cut.

**Out — genuinely separate engines, never grown here:** plasma-etch / diffusion *process-physics
simulation*, SPICE-level *circuit synthesis*, and DAQ firmware. These are Sentaurus's and SPICE's
job; reimplementing them is not ambition, it is a detour that starves the moat. The moat is the
**differentiable co-design loop** — and "all the way" means every term that touches the figure of
merit lives *on that tape*, which is a bigger goal than a mechanical-only floor, not a smaller one.
