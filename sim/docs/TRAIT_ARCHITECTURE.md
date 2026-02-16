# Simulation Domain — Trait Architecture

## The Problem

MuJoCo is a monolith. One bending model (cotangent Laplacian). One elasticity
formulation (Saint Venant-Kirchhoff). One actuator gain function per type. One
integrator per simulation. The algorithms are excellent — Todorov made
world-class choices — but they are choices. Hardcoded at compile time, baked
into every line of `engine_passive.c` and `engine_forward.c`.

This is fine for MuJoCo's purpose: a single high-quality simulator for robotics
research. It is not fine for ours.

CortenForge targets a wider envelope. Robotics training (small deformation,
stiff contacts, implicit integration) and animation (large deformation,
compliant contacts, explicit integration). Sim-to-real transfer (MuJoCo-exact
conformance) and sim-to-sim diversity (domain randomization over physics
models). The same engine needs to be both: exactly MuJoCo when you need
MuJoCo, and something else when you don't.

The question is: where do the seams go?

## The Principle

**Trait the modeling choices. Enum the solver variants. Hardcode the math.**

Three categories of algorithm in the pipeline:

| Category | Criterion | Mechanism | Example |
|----------|-----------|-----------|---------|
| **Modeling choice** | Different physics for different applications | `trait` (static dispatch) | Bending discretization, membrane elasticity, actuator gain model |
| **Solver variant** | Same physics, different numerical strategy | `enum` (match dispatch) | PGS vs CG vs Newton, Euler vs RK4 vs implicit |
| **Core math** | One correct algorithm, no alternatives | Direct implementation | CRBA, RNE, sparse LDL, quaternion integration |

The trait boundaries are where *physics diverges* — where one formulation serves
robotics and another serves animation and both are correct for their domain.
The enum boundaries are where *numerics diverge* — where the same physical
model can be solved with different accuracy-performance tradeoffs. The hardcoded
paths are where the math is the math — there is one way to compute composite
rigid body inertia, one way to factor a sparse matrix, one way to integrate a
quaternion on SO(3).

This is not an abstraction exercise. Each trait boundary exists because we have
identified (or will identify) at least two concrete implementations that serve
different use cases. No speculative traits. No "maybe we'll need this."

## Tier Classification

### Tier 1 — Modeling Choices (Traits)

These change what the simulation *is*. Different implementations produce
different physics. Choosing one over another is a statement about what
physical regime the simulation targets.

#### 1a. Flex Bending Model

**Why it's a trait:** Bridson dihedral angle springs (geometrically nonlinear,
large deformation, animation) vs Wardetzky/Garg cotangent Laplacian (linear
coefficients, small deformation, robotics). MuJoCo uses cotangent exclusively.
Both are correct for their domain.

**Current state:** Bridson implemented in `mj_fwd_passive()` with per-vertex
force magnitude clamp. Cotangent specified in §42B (future_work_10.md).

```rust
pub trait FlexBendingModel {
    /// Per-flex precomputed data (opaque to the pipeline).
    type Precomputed;

    /// Precompute coefficients at model build time.
    /// Called once per flex body during `Model` construction.
    fn precompute(
        flex_id: usize,
        model: &Model,
    ) -> Self::Precomputed;

    /// Accumulate bending forces into qfrc_passive.
    /// Called each timestep in `mj_fwd_passive()`.
    fn apply_forces(
        flex_id: usize,
        pre: &Self::Precomputed,
        model: &Model,
        data: &mut Data,
    );
}
```

| Implementation | Precomputed | Force Computation | Stability | Domain |
|---------------|-------------|-------------------|-----------|--------|
| `BridsonBending` | Rest dihedral angles (1 per hinge) | Nonlinear: `atan2`-based dihedral, gradient depends on current geometry | Per-vertex force magnitude clamp: `fm_max = 1/(dt^2 * \|grad\| * invmass)` | Cloth simulation, large deformation, animation |
| `CotangentBending` | 17 f64 per edge (4x4 coupling matrix + curved ref coefficient) | Linear: `f = -k * B * (x - x0)`, constant matrix `B` | No clamp needed (constant coefficients) | Robotics, small deformation, MuJoCo conformance |

**Spec:** §42B in future_work_10.md.

#### 1b. Membrane / Edge Elasticity Model

**Why it's a trait:** The edge constraint currently uses a soft equality
constraint with solref/solimp (matching MuJoCo's `mjEQ_FLEX` mechanism). This
works for linear elastic response. But nonlinear hyperelastic materials
(Neo-Hookean, Mooney-Rivlin, Ogden) require strain-energy-based forces that
cannot be expressed as constraint rows — they need direct force computation
from deformation gradients.

**The divergence:**

| Regime | Model | Mechanism | Use Case |
|--------|-------|-----------|----------|
| Small strain | Linear elastic (current) | Constraint row (`FlexEdge`) | Robotics, stiff bodies, MuJoCo conformance |
| Large strain | Neo-Hookean / SVK | Passive force from strain energy | Soft tissue, surgical simulation, rubber |
| Hyperelastic | Mooney-Rivlin, Ogden | Passive force from strain energy invariants | Biomechanics, material characterization |

```rust
pub trait FlexElasticityModel {
    type Precomputed;

    fn precompute(
        flex_id: usize,
        model: &Model,
    ) -> Self::Precomputed;

    /// How this elasticity model participates in the solve.
    /// Constraint-based models return rows for the Jacobian.
    /// Force-based models accumulate into qfrc_passive.
    fn apply(
        flex_id: usize,
        pre: &Self::Precomputed,
        model: &Model,
        data: &mut Data,
    );
}
```

**Current:** `LinearElastic` (constraint-based, matches MuJoCo). Future:
`NeoHookean`, `StVenantKirchhoff`, `MooneyRivlin`.

**Status:** Not yet specified. Priority: after §42B (bending trait proves the
pattern).

#### 1c. Actuator Gain Model

**Why it's a trait:** MuJoCo already has three gain types (`Fixed`, `Affine`,
`Muscle`) dispatched by enum. The pattern is correct but the set is closed.
Real actuator modeling needs: series elastic actuators (SEA compliance),
pneumatic actuators (nonlinear pressure-volume), hydraulic actuators (valve
dynamics), cable-driven actuators (cable elasticity + friction). Each of these
computes `force = f(length, velocity, activation, params)` but with radically
different internal models.

```rust
pub trait ActuatorGainModel {
    type Params;

    /// Compute the actuator force contribution.
    fn gain(
        length: f64,
        velocity: f64,
        activation: f64,
        ctrl: f64,
        params: &Self::Params,
    ) -> f64;

    /// Compute ∂force/∂velocity for implicit integration.
    fn dgain_dvel(
        length: f64,
        velocity: f64,
        activation: f64,
        ctrl: f64,
        params: &Self::Params,
    ) -> f64;
}
```

**Current:** Enum dispatch (`GainType::Fixed | Affine | Muscle`) in
`mj_fwd_actuation()`. The enum stays for MuJoCo-compatible types; the trait
extends it for custom models.

**Status:** Not yet specified. Priority: medium (current enum dispatch works
for most RL models).

#### 1d. Contact Solver

**Why it's a trait (eventually):** The current architecture already has enum
dispatch (`SolverType::PGS | CG | Newton`). This works because all three
solve the same LCP formulation — they differ in numerics, not physics. But
future solvers may differ in formulation: position-based dynamics (PBD/XPBD)
for real-time applications, compliant contact models for soft robotics,
impulse-based solvers for event-driven simulation. These aren't just different
numerical strategies for the same problem — they're different problem
formulations entirely.

**Current:** Enum dispatch. The existing architecture is correct for PGS/CG/Newton.
Trait boundary only needed if/when we add solvers with fundamentally different
contact formulations. Not yet specified.

### Tier 2 — Performance/Accuracy Knobs (Enums)

These don't change the physics — they change how accurately or quickly the same
physics is computed. The user trades speed for precision. All variants converge
to the same answer in the limit.

| Knob | Current Implementation | Possible Additions | Mechanism |
|------|----------------------|-------------------|-----------|
| **Integrator** | `Euler`, `ImplicitSpringDamper`, `ImplicitFast`, `Implicit`, `RungeKutta4` | Symplectic Euler, Verlet | `Integrator` enum + match in `step()` |
| **Contact solver** | PGS, CG, Newton | (see Tier 1d for formulation changes) | `SolverType` enum + match |
| **Broad-phase** | Sweep-and-prune | Spatial hashing, BVH | `BroadPhase` enum + match |
| **Friction cone** | Elliptic (linearized projection) | Pyramidal (#32) | `ConeType` enum + match |

These stay as enums. They're dispatch points in a hot loop. Match-based
dispatch is zero-cost, exhaustive at compile time, and trivially inlined. A
trait here would add indirection for no architectural benefit.

### Tier 3 — Core Math (Hardcoded)

These have one correct implementation. Traiting them is abstraction for
abstraction's sake.

| Algorithm | Why No Trait |
|-----------|-------------|
| CRBA (Composite Rigid Body Algorithm) | One algorithm. Featherstone proved optimality. |
| RNE (Recursive Newton-Euler) | Dual of CRBA. Same argument. |
| Sparse LDL factorization | Numerical linear algebra. One correct algorithm for symmetric positive semi-definite. |
| Quaternion integration on SO(3) | Differential geometry. The exponential map is the exponential map. |
| Gravity | `F = mg`. |
| Forward kinematics | Tree traversal with pose composition. No alternative. |
| Sensors | Read from data buffers. The physics is upstream. |

If someone says "but what about a different FK algorithm," the answer is: there
isn't one. Joint-to-joint pose composition is the definition of forward
kinematics. The only variation is traversal order (topological sort vs
depth-first), which is a micro-optimization, not a modeling choice.

## Composition Pattern

Traits compose at model construction time. The `SimBuilder` assembles the
trait implementations into a concrete simulation type with fully static
dispatch — no vtable overhead in the inner loop.

```rust
// Default: MuJoCo-conformant configuration
let model = load_model(mjcf)?;
let mut data = model.make_data();
data.step(&model)?;

// Custom: compose trait implementations at build time
let sim = SimBuilder::new()
    .bending(CotangentBending)              // MuJoCo-exact bending
    .elasticity(NeoHookean::new(mu, lambda)) // Hyperelastic membranes
    .actuators(DefaultGain)                  // Standard MuJoCo gain types
    .build(model)?;

sim.step()?;

// Animation preset: large-deformation cloth
let sim = SimBuilder::new()
    .bending(BridsonBending)
    .elasticity(LinearElastic)
    .actuators(DefaultGain)
    .build(model)?;
```

**Implementation strategy:** Generic `Sim<B: FlexBendingModel, E: FlexElasticityModel, ...>`
with type aliases for common configurations:

```rust
pub type MujocoSim = Sim<CotangentBending, LinearElastic, DefaultGain>;
pub type AnimationSim = Sim<BridsonBending, LinearElastic, DefaultGain>;
pub type SoftBodySim = Sim<CotangentBending, NeoHookean, DefaultGain>;
```

The `Model`/`Data` API remains the default entry point. `SimBuilder` is the
power-user API for when you need to deviate from MuJoCo defaults. The two
coexist — `Model::make_data()` + `Data::step()` always uses the MuJoCo-conformant
configuration.

## What This Unlocks

### For Robotics (MuJoCo Conformance Path)

The default configuration matches MuJoCo exactly. Cotangent bending, linear
edge constraints, standard actuator gains. Every conformance test (#45) passes
with the default trait implementations. Users who need MuJoCo compatibility
change nothing.

### For Domain Randomization

Trait selection becomes a randomization axis. Instead of just randomizing
`Young's modulus` and `damping`, you can randomize the *physics model itself*:

```rust
// Train a policy that's robust to physics model uncertainty
for episode in 0..num_episodes {
    let bending = if rng.gen_bool(0.5) {
        BendingChoice::Cotangent
    } else {
        BendingChoice::Bridson
    };
    let sim = SimBuilder::new()
        .bending(bending)
        .build(model.clone())?;
    // ... train episode ...
}
```

This is a fundamentally different kind of domain randomization — not just
"what if friction is 10% higher" but "what if the physics is discretized
differently." Policies trained this way transfer better because they learn
the invariant structure of the task, not the artifacts of a particular
discretization.

### For Soft Robotics and Biomechanics

Neo-Hookean and Mooney-Rivlin elasticity models unlock simulation of
biological tissue, elastomers, and pneumatic actuators. These materials
exhibit large-strain nonlinear behavior that linear edge constraints cannot
capture. The trait boundary makes this a configuration choice, not a fork.

### For Real-Time and Animation

Bridson bending + explicit integration + compliant contacts = fast, visually
plausible simulation for interactive applications. The same engine serves
both the training loop (implicit, stiff, conformant) and the visualization
loop (explicit, compliant, fast).

## Implementation Roadmap

The trait architecture rolls out incrementally. Each step proves the pattern
on one boundary before expanding to the next.

| Phase | Trait Boundary | Spec | Status |
|-------|---------------|------|--------|
| **Phase A** | `FlexBendingModel` | [§42B](todo/future_work_10.md) | Specified, not implemented |
| **Phase B** | `FlexElasticityModel` | [§42C](todo/future_work_10.md) | Specified, not implemented |
| **Phase C** | `ActuatorGainModel` | [§42D](todo/future_work_10.md) | Specified, not implemented |
| **Phase D** | `SimBuilder` composition | [§42F](todo/future_work_10.md) | Specified, not implemented (depends on A-C, E) |
| **Phase E** | `ContactSolver` | [§42E](todo/future_work_10.md) | Specified, not implemented |

Phase A is the proof-of-concept. If the bending trait works cleanly — static
dispatch, no performance regression, clean precomputation API — then B and C
follow the same pattern. Phase D assembles them into the builder.

The existing enum-based dispatch points (integrator, solver type, cone type)
remain as-is. They work. No refactor needed.

## Design Constraints

### No Dynamic Dispatch in the Inner Loop

All trait dispatch is monomorphized. `Sim<B, E, A>` is a concrete type with
concrete method calls. The hot path — `mj_fwd_passive()`, constraint assembly,
`mj_fwd_actuation()` — sees no vtables.

The one exception: if a model mixes bending types across flex bodies (e.g.,
flex 0 uses cotangent, flex 1 uses Bridson), we use a per-flex enum wrapper
with match dispatch. This is a cold branch (one per flex per timestep), not
a hot-loop indirection.

### Precomputation Separates Build Time from Runtime

Each trait has a `Precomputed` associated type. Heavy setup (cotangent weight
computation, strain energy Hessian assembly) happens once at model construction.
The runtime path reads from precomputed arrays. This matches MuJoCo's pattern:
`user_mesh.cc::ComputeBending()` runs at load time, `engine_passive.c` reads
the precomputed `flex_bending[17*e + ...]` at runtime.

### Backward Compatibility is Non-Negotiable

The `Model`/`Data` API does not change. `load_model()` + `make_data()` +
`step()` continues to work exactly as before. The trait architecture is
additive — new capability, not new requirements.

MJCF models that don't specify a bending model get cotangent (MuJoCo default).
Models that specify `bending_model="bridson"` in `<flex>` get Bridson.
The configuration surface is the MJCF file, not the Rust API.

### Each Trait Must Have At Least Two Implementations Before It Ships

No speculative abstractions. The bending trait ships when both `CotangentBending`
and `BridsonBending` pass tests. The elasticity trait ships when both
`LinearElastic` and at least one hyperelastic model pass tests. This prevents
trait boundaries that look good on paper but don't work in practice.

## Competitive Position

| Feature | MuJoCo | CortenForge (with traits) |
|---------|--------|--------------------------|
| Bending models | Cotangent only | Cotangent + Bridson (extensible) |
| Elasticity | SVK only | Linear + Neo-Hookean + extensible |
| Actuator models | Fixed/Affine/Muscle (closed set) | Same + user-defined (open set) |
| Contact solvers | PGS/CG/Newton | Same (enum) |
| Domain randomization | Parameters only | Parameters + physics models |
| Conformance | Self-referential | Exact match (default config) |
| Extension model | `<plugin>` C callbacks | Rust traits (type-safe, zero-cost) |

MuJoCo's monolithic design is its strength for reproducibility and its
weakness for extensibility. CortenForge's trait architecture provides the
same reproducibility (default config = MuJoCo-exact) with composable
extensibility where it matters.

## References

- Wardetzky, M. et al. (2007). "Discrete Quadratic Curvature Energies" — cotangent Laplacian bending
- Garg, A. et al. (2007). "Cubic Shells" — curved reference surface correction
- Bridson, R. et al. (2003). "Simulation of Clothing with Folds and Wrinkles" — dihedral angle bending
- Todorov, E. (2014). "Convex and analytically-invertible dynamics" — MuJoCo's contact formulation
- Bonet, J. & Wood, R.D. (2008). "Nonlinear Continuum Mechanics for Finite Element Analysis" — hyperelastic models
- Hill, A.V. (1938). "The heat of shortening and the dynamic constants of muscle" — muscle gain model
