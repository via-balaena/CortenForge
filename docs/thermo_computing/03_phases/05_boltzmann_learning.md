# Phase 5 — Boltzmann Learning on a Physical Ising Sampler

> **Status**: Spec draft — awaiting approval.
> **Owner**: Jon
> **Parents**:
> - [`04_coupled_bistable_array.md`](./04_coupled_bistable_array.md) (Phase 4 — the gate this phase inherits)
> - Phase 4 §11.3 (bilinear coupling maps to Ising, not general EBMs)
> - Phase 4 §11.4 (full joint-distribution comparison deferred to Phase 6)
> - [`../02_foundations/open_questions.md`](../02_foundations/open_questions.md) Q5 (cf-design NOT end-to-end differentiable)
> - [`../04_recon_log/2026-04-09_part_13_q5_cf_design.md`](../04_recon_log/2026-04-09_part_13_q5_cf_design.md) (Q5 resolution — three breaks)
> - [`../01_vision/research_directions.md`](../01_vision/research_directions.md) D4 (sim-to-real — Phase 5 training algorithm is D4's EBM trainer)

Phase 5's original framing — "differentiable EBM via cf-design" — is dead.
Q5 resolved NO: cf-design is not end-to-end differentiable (three named
breaks: discrete mesh extraction, unwired mass properties, opaque sim-core
forward step). No autograd tape, no finite-difference workaround, and no
cf-design dependency are needed. What replaces it is cleaner and more
fundamental.

**The replacement**: train the coupling and field parameters of a physical
Ising-like system to match a target distribution using the **Boltzmann
machine learning rule** (Hinton & Sejnowski 1983). The gradient of the KL
divergence between the target and model distributions has a closed-form
expression that requires only sample correlations from the physical sampler
— no differentiation through the simulation, no surrogate model, no neural
network. The physical Langevin system IS the generative model; the training
signal is the gap between what the system naturally samples and what we want
it to sample.

---

## 1. Goal

Demonstrate that the coupled bistable array from Phase 4, extended with
per-edge coupling constants and per-element external fields, can be
**trained** as a generative model via the Boltzmann machine learning rule.
Specifically:

1. **Parameter recovery**: Starting from zero parameters (uniform
   distribution), the learner recovers the coupling constants `J_{ij}` and
   external fields `h_i` of a known target Ising model to within a tight
   tolerance.
2. **KL convergence**: The KL divergence between the target distribution
   and the learned model's distribution decreases monotonically (in
   expectation) over training iterations, converging below a threshold.
3. **Physical sampler as model**: The training loop uses the Langevin
   simulation as its *only* source of model statistics — no software
   sampler, no exact computation of model statistics during training. The
   physical system is the generative model.

This is the first phase where the thermodynamic computing stack produces a
**learned** artifact: a set of parameters that encode a target distribution
in the energy landscape of a mechanical system.

---

## 2. Scope

**In scope for Phase 5:**

- **Generalize `PairwiseCoupling`** to per-edge coupling constants (§5.1).
  Breaking change to `new()` signature; `chain()`, `ring()` unchanged.
- **New `ExternalField` component** (§5.2): linear bias `V = −h_i · x_i`
  as a `PassiveComponent`.
- **New `ising` module** (§5.3): exact Ising distribution, statistics, and
  KL divergence for arbitrary topology, per-edge couplings, and per-site
  fields. Production code (used by `IsingLearner` and by Phase 6).
- **New `IsingLearner` struct** (§6): the Boltzmann learning loop.
  Production code (used by Phase 5 tests and by D4).
- **`fully_connected` factory** on `PairwiseCoupling` (§5.4).
- **Validation tests** (§8): KL convergence gate (must-pass), parameter
  recovery gate (`#[ignore]`), learning curve, symmetry breaking,
  reproducibility.

**Explicitly out of scope:**

- **cf-design integration** — Q5 killed this. No SDF, no mesh, no design
  parameters.
- **Autograd tape** — the Boltzmann learning rule is analytic; no
  `sim-ml-bridge` dependency.
- **Finite differences** — wasteful when the gradient is closed-form.
- **General EBMs / higher-order coupling** — Phase 5 stays within the
  pairwise Ising family. §10.3 discusses the upgrade path.
- **N > 4 arrays** — components and learner are parameterized in N, but
  only N=4 is validated.
- **Continuous-distribution targets** — the "2D Gaussian mixture" from the
  original sketch is ill-posed for this model class (§4.3).
- **CPU Gibbs sampler** — that's Phase 6. Phase 5 uses exact enumeration
  for reference statistics.

---

## 3. Physics: Boltzmann Machine Learning

### 3.1 Ising energy with external fields

The continuous system's energy (inherited from Phase 4, extended with
external fields):

```
E(x) = Σ_i V_well(x_i) − Σ_{(i,j) ∈ edges} J_{ij} · x_i · x_j − Σ_i h_i · x_i
```

where `V_well(x) = a(x² − x₀²)²` is the quartic double-well from Phase 3,
`J_{ij}` are per-edge coupling constants, and `h_i` are per-element
external fields.

After coarse-graining to binary spins `σ_i = sign(x_i) ∈ {−1, +1}`, the
effective Ising Hamiltonian is:

```
H(σ) = −Σ_{(i,j)} J_eff_{ij} · σ_i · σ_j − Σ_i h_eff_i · σ_i
```

with the continuous-to-Ising mapping (inherited from Phase 4 §3.2):

```
J_eff_{ij} ≈ J_{ij} · ⟨x⟩²_well / kT ≈ J_{ij} · x₀² / kT
h_eff_i    ≈ h_i   · ⟨x⟩_well  / kT ≈ h_i   · x₀  / kT
```

At the central parameters (`x₀ = 1.0`, `kT = 1.0`), the mapping is
approximately 1:1: `J_eff ≈ J`, `h_eff ≈ h`. The ~5% anharmonic correction
from Phase 4 §3.2 applies to both couplings and fields.

The Boltzmann distribution:

```
P(σ; J, h) = exp(−H(σ) / kT) / Z(J, h)
           = exp((Σ J_{ij} σ_i σ_j + Σ h_i σ_i) / kT) / Z
```

For N=4 with 2⁴ = 16 configurations, `Z` and all expectations are
computable by exact enumeration.

### 3.2 The Boltzmann learning rule

The KL divergence between the target distribution `P*` and the model
distribution `P_model`:

```
KL(P* ‖ P_model) = Σ_σ P*(σ) · log(P*(σ) / P_model(σ; J, h))
```

The gradient with respect to the coupling and field parameters:

```
∂KL/∂J_{ij} = −(1/kT) · [⟨σ_i σ_j⟩_target − ⟨σ_i σ_j⟩_model]
∂KL/∂h_i    = −(1/kT) · [⟨σ_i⟩_target     − ⟨σ_i⟩_model    ]
```

**Derivation**: `log P_model(σ) = (Σ J σσ + Σ h σ)/kT − log Z`. The
gradient of `−log P_model` w.r.t. `J_{ij}` is `−σ_i σ_j / kT + ⟨σ_i σ_j⟩_model / kT`.
Averaging over `P*` gives the result. This is exactly the Boltzmann machine
learning rule, and it is the natural gradient of maximum-likelihood
estimation for exponential families.

The gradient descent update (absorbing `1/kT` into the learning rate):

```
J_{ij} ← J_{ij} + η · [⟨σ_i σ_j⟩_target − ⟨σ_i σ_j⟩_model]
h_i    ← h_i    + η · [⟨σ_i⟩_target     − ⟨σ_i⟩_model    ]
```

**Key property**: `⟨σ_i σ_j⟩_model` is estimated by running the physical
Langevin sampler and measuring spin correlations — exactly the Phase 4
measurement protocol. The physical system IS the model. No surrogate, no
neural network, no differentiation through simulation.

**Convergence guarantee**: For exponential families, the KL divergence is
convex in the natural parameters `(J, h)`. If the target is in the model
family (which it is — we choose an Ising target), gradient descent converges
to the unique global minimum `KL = 0` (up to the ~5% continuous-to-Ising
mapping correction). With noisy gradients (finite simulation), convergence
is to a neighborhood of the optimum whose radius depends on the learning
rate and sample noise.

### 3.3 Continuous-to-Ising mapping corrections (inherited)

Phase 4 §3.2 documented ~5% mapping deviation from the naive Ising
prediction due to within-well anharmonic shifts. The same corrections
apply here:

1. **Anharmonic shift**: `⟨x⟩ ≈ 0.979 · x₀`, shifting `J_eff` by ~4%.
2. **Backreaction**: Second-order in `J/V″(x₀)` — negligible.
3. **Barrier-region exclusion**: ~34% data reduction for N=4 (same as
   Phase 4).

The learner does not correct for these — it learns the *continuous*
parameters `{J, h}` that best match the target statistics. The mapping
error means the learned parameters will differ from the target Ising
parameters by ~5%. This is expected and accounted for in the validation
tolerances.

### 3.4 Expressiveness of the N=4 fully-connected Ising model

With N=4 on a fully-connected graph:

- **Edges**: `(0,1), (0,2), (0,3), (1,2), (1,3), (2,3)` — 6 edges.
- **Learnable parameters**: 6 couplings `J_{ij}` + 4 fields `h_i` = **10
  parameters**.
- **Distribution**: 2⁴ = 16 configurations with **15 free probabilities**
  (sum-to-one constraint).

The pairwise Ising model is an exponential family with 10 sufficient
statistics `{σ_i, σ_i σ_j}`. It can represent any distribution in this
exponential family, but NOT arbitrary distributions over 16 states (which
would require 15 free parameters — 5 more than the model has). Specifically,
the model cannot represent distributions that require **three-body or
higher-order interactions** (e.g., `σ_1 σ_2 σ_3`).

For Phase 5's validation, the target is chosen FROM the Ising family, so
it is exactly representable by the model (up to the continuous-to-Ising
mapping correction).

---

## 4. Design Decisions

### 4.1 Boltzmann learning rule, not FD or autograd

Three candidate framings were considered:

| Framing | Mechanism | Why rejected |
|---------|-----------|-------------|
| 5A: Finite differences on J | Perturb J, re-run sim, estimate ∂KL/∂J | Wasteful — the gradient is closed-form (`−x_i x_j`). Burns 2× simulation budget per parameter per iteration. |
| 5B: Autograd tape | Wire J into `sim-ml-bridge`'s tape | The tape can't differentiate through `mj_step`. Model statistics still require simulation; autograd adds nothing. |
| 5C: Defer entirely | Jump to D1 or D2 | Phase 5 has real, achievable content that unlocks D4. Deferring wastes the opportunity. |

**5D (chosen): Boltzmann learning rule**. The gradient is exact and
analytic: `∂KL/∂J_{ij} ∝ ⟨σ_i σ_j⟩_target − ⟨σ_i σ_j⟩_model`. No
perturbation, no tape, no cf-design. The training signal is the difference
between what the physical system samples and what we want it to sample.
This is the original Boltzmann machine learning rule (Hinton & Sejnowski
1983) — the most fundamental training algorithm for energy-based models.

### 4.2 Fully connected N=4 topology

**Choice**: Fully connected 4-node graph (6 edges, 10 parameters).

**Why not chain** (Phase 4's topology): The chain has only 3 edges with
uniform J — too few degrees of freedom for interesting learning. With
uniform J, the "learned" model has 1 free coupling parameter; the learning
problem is trivially solved by matching a single correlation. Fully
connected provides 10 parameters with non-trivial interactions between
them.

**Why not N=8**: 2⁸ = 256 configurations and N(N−1)/2 = 28 edges — the
exact Ising reference is still cheap, but the simulation budget for
convergent statistics scales linearly with the number of parameters. N=4
keeps Phase 5 focused on validating the *algorithm*, not on scaling.

### 4.3 Known Ising target, not "2D Gaussian mixture"

The overview's original Phase 5 sketch said "train the array as an EBM to
match a 2D Gaussian mixture target." This is ill-posed:

1. A Gaussian mixture is a continuous distribution; the coarse-grained
   system has 16 discrete configurations. "Matching" requires a mapping
   between these spaces that is not well-defined.
2. The pairwise Ising model cannot represent arbitrary distributions over
   16 states (§3.4). A 2D Gaussian mixture, discretized to 4 binary
   variables, may not be in the Ising family.
3. The learning algorithm's convergence guarantee (§3.2) requires the
   target to be in the model family.

**The right target**: a known Ising model with specific `{J*, h*}`
parameters, whose exact statistics are computable by enumeration. This
lets us validate:
- That the learning algorithm converges (KL → 0)
- That the learned parameters match the target (J → J*, h → h*)
- That the physical sampler's statistics match the exact model

### 4.4 ExternalField as a separate component

**Choice**: A new `ExternalField` struct implementing `PassiveComponent`,
separate from `DoubleWellPotential`.

**Why not modify DoubleWellPotential**: The double-well is a nonlinear
conservative potential; the external field is a linear bias. They have
different physical roles (confining vs. symmetry-breaking) and different
parameter semantics (barrier height vs. field strength). Composing them
via the `PassiveStack` is cleaner than overloading the well with a tilt
parameter.

**Why constant force is correct**: `V_field = −h_i · x_i` gives
`F_i = −dV/dx_i = h_i` — a position-independent force. This is the
natural continuous analogue of the Ising external field, which biases the
spin uniformly regardless of its current value.

### 4.5 IsingLearner as production code, not test-only

The Boltzmann learning loop lives in `sim-thermostat/src/ising_learner.rs`,
not in the test file. Reason: D4 (sim-to-real on a printed device) needs
the same training algorithm to train the EBM before printing. Keeping it
in production code makes it available to D4 and any future consumer.

The `ising` module (exact distribution, statistics, KL) is similarly
production code — Phase 6's Gibbs sampler comparison needs these utilities.

### 4.6 Model ownership, not MJCF loading

`sim-mjcf` is a **dev-dependency** of `sim-thermostat` — it is not
available to production code. The `IsingLearner` therefore takes ownership
of a pre-loaded `Model` rather than loading from XML internally. The test
file (which CAN use `sim-mjcf` as a dev-dependency) handles model loading:

```rust
// In tests/boltzmann_learning.rs:
let model = sim_mjcf::load_model(CHAIN_XML).expect("load");
let mut learner = IsingLearner::new(config, target, model);
```

This keeps the dependency boundary clean. The learner re-installs the
`PassiveStack` on its owned model each iteration (`stack.install()` safely
replaces `model.cb_passive`) and creates fresh `Data` via
`model.make_data()`. Verified safe: `install()` is an unconditional
overwrite (`stack.rs:114-115`), `make_data()` returns independent
instances, `data.forward()` recomputes all derived quantities from
`qpos`/`qvel` with no hidden state leakage.

---

## 5. Infrastructure Changes

### 5.1 `PairwiseCoupling`: per-edge coupling constants

**Current** (Phase 4): one `coupling_j: f64` shared across all edges.

**New**: per-edge `coupling_j: Vec<f64>`, one value per edge. The internal
representation changes from scalar to vector.

```rust
pub struct PairwiseCoupling {
    /// Per-edge coupling constants. `coupling_j[k]` is the coupling
    /// constant for `edges[k]`.
    coupling_j: Vec<f64>,
    /// List of DOF pairs that are coupled.
    edges: Vec<(usize, usize)>,
}
```

**API changes**:

```rust
impl PairwiseCoupling {
    /// Create a coupling with per-edge coupling constants.
    ///
    /// # Panics
    /// - If `coupling_j.len() != edges.len()`.
    /// - If any edge has `i == j` (self-coupling).
    pub fn new(coupling_j: Vec<f64>, edges: Vec<(usize, usize)>) -> Self;

    /// Create with uniform coupling constant across all edges.
    pub fn uniform(coupling_j: f64, edges: Vec<(usize, usize)>) -> Self;

    /// Nearest-neighbor open chain with uniform J.
    /// Unchanged signature — calls `uniform` internally.
    pub fn chain(n: usize, coupling_j: f64) -> Self;

    /// Nearest-neighbor ring with uniform J.
    /// Unchanged signature — calls `uniform` internally.
    pub fn ring(n: usize, coupling_j: f64) -> Self;

    /// Fully connected graph with uniform J.
    /// New factory for Phase 5.
    pub fn fully_connected(n: usize, coupling_j: f64) -> Self;

    /// Per-edge coupling constants (read-only).
    pub fn coupling_j(&self) -> &[f64];

    /// Edge list (read-only). Unchanged.
    pub fn edges(&self) -> &[(usize, usize)];

    /// Total coupling energy with per-edge J.
    pub fn coupling_energy(&self, qpos: &DVector<f64>) -> f64;
}
```

**Breaking change**: `new()` signature changes from `(f64, Vec<...>)` to
`(Vec<f64>, Vec<...>)`. The old single-J usage is recovered by `uniform()`.
Phase 4's test code uses `chain()`, which is unaffected. Unit tests in
`pairwise_coupling.rs` that call `new()` directly need updating.

**`coupling_j()` return type change**: returns `&[f64]` (slice of per-edge
values) instead of `f64`. Phase 4's unit test `chain_produces_correct_edges`
asserts `c.coupling_j() == 0.5` — needs updating to check the slice.

**`Diagnose::diagnostic_summary` update**: The current format
`PairwiseCoupling(J={:.4}, edges={})` formats a single `f64` — won't
compile with `Vec<f64>`. New format for per-edge couplings:

```rust
impl Diagnose for PairwiseCoupling {
    fn diagnostic_summary(&self) -> String {
        let j_min = self.coupling_j.iter().cloned().fold(f64::INFINITY, f64::min);
        let j_max = self.coupling_j.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        if (j_min - j_max).abs() < 1e-15 {
            // Uniform coupling — compact display
            format!("PairwiseCoupling(J={j_min:.4}, edges={})", self.edges.len())
        } else {
            format!(
                "PairwiseCoupling(J=[{j_min:.4}, {j_max:.4}], edges={})",
                self.edges.len()
            )
        }
    }
}
```

Uniform-J case preserves the old format exactly (Phase 4 test
`diagnostic_summary_format` passes unchanged for `chain(4, 0.5)`).
Per-edge case shows `[min, max]` range.

**`PassiveComponent::apply` update**:

```rust
fn apply(&self, _model: &Model, data: &Data, qfrc_out: &mut DVector<f64>) {
    for (&(i, j), &j_k) in self.edges.iter().zip(&self.coupling_j) {
        let xi = data.qpos[i];
        let xj = data.qpos[j];
        qfrc_out[i] += j_k * xj;
        qfrc_out[j] += j_k * xi;
    }
}
```

### 5.2 `ExternalField` component

New file: `sim-thermostat/src/external_field.rs`.

```rust
/// Linear external field: `V = −Σ h_i · x_i`.
///
/// Contributes a constant force `F_i = h_i` to each DOF. This breaks the
/// symmetry of the double-well potential, biasing elements toward the
/// right well (h > 0) or left well (h < 0). The continuous analogue of
/// the Ising external field.
///
/// Not stochastic — this is a deterministic conservative force.
///
/// # Joint type constraint
///
/// Same as `DoubleWellPotential` and `PairwiseCoupling`: only correct for
/// slide and hinge joints where `nq = nv = 1`.
pub struct ExternalField {
    /// Per-DOF field strengths.
    field_h: Vec<f64>,
}

impl ExternalField {
    /// Create an external field with per-DOF field strengths.
    pub fn new(field_h: Vec<f64>) -> Self;

    /// Field strengths (read-only).
    pub fn field_h(&self) -> &[f64];

    /// Field energy: `V = −Σ h_i · x_i`.
    pub fn field_energy(&self, qpos: &DVector<f64>) -> f64;
}

impl PassiveComponent for ExternalField {
    fn apply(&self, _model: &Model, _data: &Data, qfrc_out: &mut DVector<f64>) {
        for (i, &h) in self.field_h.iter().enumerate() {
            qfrc_out[i] += h;
        }
    }
}

impl Diagnose for ExternalField {
    fn diagnostic_summary(&self) -> String {
        format!("ExternalField(n={}, h_range=[{:.4}, {:.4}])",
            self.field_h.len(),
            self.field_h.iter().cloned().fold(f64::INFINITY, f64::min),
            self.field_h.iter().cloned().fold(f64::NEG_INFINITY, f64::max),
        )
    }
}
```

**Unit tests** (in-module):
- `new_creates_field`
- `field_energy_computation`
- `apply_adds_constant_force` (via finite-difference check on `field_energy`)
- `diagnostic_summary_format`

### 5.3 `ising` module — exact Ising reference

New file: `sim-thermostat/src/ising.rs`. Production code, exported from
`lib.rs`.

```rust
/// Exact Ising distribution by enumeration over 2^N configurations.
///
/// Supports arbitrary topology (via edge list), per-edge coupling constants,
/// and per-site external fields. Returns a vector of (config_bitmask,
/// probability) pairs sorted by config index.
///
/// # Parameters
/// - `n`: number of spins
/// - `edges`: coupling topology
/// - `coupling_j`: per-edge coupling constants (length = edges.len())
/// - `field_h`: per-site external fields (length = n)
/// - `k_b_t`: thermal energy
///
/// # Panics
/// Panics if `n > 20` (2^20 = 1M configurations — safety limit).
pub fn exact_distribution(
    n: usize,
    edges: &[(usize, usize)],
    coupling_j: &[f64],
    field_h: &[f64],
    k_b_t: f64,
) -> Vec<(u32, f64)>;

/// Extract magnetizations ⟨σ_i⟩ and pairwise correlations ⟨σ_i σ_j⟩
/// from an exact distribution.
pub fn ising_statistics(
    dist: &[(u32, f64)],
    n: usize,
    edges: &[(usize, usize)],
) -> IsingStats;

/// KL divergence KL(P ‖ Q) between two distributions over the same
/// configuration space.
///
/// Returns `f64::INFINITY` if any configuration has P > 0 and Q = 0.
pub fn kl_divergence(p: &[(u32, f64)], q: &[(u32, f64)]) -> f64;

/// Summary statistics from an Ising distribution.
pub struct IsingStats {
    /// Per-site magnetizations ⟨σ_i⟩.
    pub magnetizations: Vec<f64>,
    /// Per-edge correlations ⟨σ_i σ_j⟩ (same order as edge list).
    pub correlations: Vec<f64>,
}
```

**Unit tests**:
- `uniform_distribution_at_zero_coupling_and_field` — all 16 configs
  equally probable.
- `magnetization_with_field_only` — no coupling, field biases marginals.
- `nn_chain_matches_phase4` — reproduce Phase 4's `tanh(βJ)` result for
  the 4-chain with uniform J.
- `kl_divergence_is_zero_for_identical_distributions`
- `kl_divergence_is_positive_for_different_distributions`
- `fully_connected_partition_function` — verify `Z` by explicit summation.

### 5.4 `fully_connected` factory

```rust
impl PairwiseCoupling {
    /// Fully connected graph: all N(N−1)/2 edges, uniform J.
    ///
    /// Edge ordering: `(0,1), (0,2), ..., (0,N−1), (1,2), ..., (N−2,N−1)`.
    /// Lexicographic — matches the natural enumeration order.
    ///
    /// # Panics
    /// Panics if `n < 2`.
    pub fn fully_connected(n: usize, coupling_j: f64) -> Self {
        assert!(n >= 2, "fully_connected requires at least 2 elements, got {n}");
        let mut edges = Vec::with_capacity(n * (n - 1) / 2);
        for i in 0..n {
            for j in (i + 1)..n {
                edges.push((i, j));
            }
        }
        Self::uniform(coupling_j, edges)
    }
}
```

---

## 6. `IsingLearner`

New file: `sim-thermostat/src/ising_learner.rs`. Exported from `lib.rs`.

### 6.1 API

```rust
/// Configuration for the Boltzmann learning loop.
pub struct LearnerConfig {
    /// Number of elements.
    pub n: usize,
    /// Coupling topology (edge list).
    pub edges: Vec<(usize, usize)>,
    /// Double-well barrier height.
    pub delta_v: f64,
    /// Well half-separation.
    pub x_0: f64,
    /// Thermostat damping coefficient (per DOF).
    pub gamma: f64,
    /// Thermal energy kT.
    pub k_b_t: f64,
    /// Learning rate η.
    pub learning_rate: f64,
    /// Simulation steps per trajectory (measurement window).
    pub n_steps: usize,
    /// Burn-in steps per trajectory.
    pub n_burn_in: usize,
    /// Independent trajectories per measurement.
    pub n_trajectories: usize,
    /// Spin classification threshold.
    pub x_thresh: f64,
    /// RNG seed base.
    pub seed_base: u64,
}

/// Target distribution for the learner.
pub struct IsingTarget {
    /// Per-site target magnetizations ⟨σ_i⟩.
    pub magnetizations: Vec<f64>,
    /// Per-edge target correlations ⟨σ_i σ_j⟩ (same order as edge list).
    pub correlations: Vec<f64>,
}

/// Record of a single learning iteration.
pub struct LearningRecord {
    /// Iteration index (0-based).
    pub iteration: usize,
    /// Current per-edge coupling constants.
    pub coupling_j: Vec<f64>,
    /// Current per-site external fields.
    pub field_h: Vec<f64>,
    /// Measured per-site magnetizations from the physical sampler.
    pub measured_magnetizations: Vec<f64>,
    /// Measured per-edge correlations from the physical sampler.
    pub measured_correlations: Vec<f64>,
    /// KL divergence between exact Ising at current params and target.
    pub kl_divergence: f64,
}

/// Boltzmann machine learning loop on a physical Ising sampler.
///
/// Trains per-edge coupling constants `J_{ij}` and per-site external
/// fields `h_i` to match a target distribution using the Boltzmann
/// learning rule. The physical Langevin simulation is the generative
/// model — no software sampler or autograd tape.
///
/// # Model ownership
///
/// The learner owns the `Model`. At each iteration it rebuilds and
/// re-installs the `PassiveStack` with updated parameters, then creates
/// fresh `Data` via `model.make_data()`. The caller is responsible for
/// loading the model (e.g., via `sim_mjcf::load_model`) before
/// constructing the learner. This keeps `sim-mjcf` out of
/// `sim-thermostat`'s production dependencies (it is a dev-dependency
/// only).
pub struct IsingLearner {
    config: LearnerConfig,
    target: IsingTarget,
    model: Model,
    coupling_j: Vec<f64>,
    field_h: Vec<f64>,
}

impl IsingLearner {
    /// Create a new learner with the given config, target, and model.
    /// Initial parameters: all zeros (uniform distribution).
    ///
    /// The model must have `nv >= config.n` DOFs (slide joints).
    /// The learner takes ownership and mutates `model.cb_passive`
    /// each iteration.
    pub fn new(config: LearnerConfig, target: IsingTarget, model: Model) -> Self;

    /// Create from explicit initial parameters.
    pub fn with_initial_params(
        config: LearnerConfig,
        target: IsingTarget,
        model: Model,
        initial_j: Vec<f64>,
        initial_h: Vec<f64>,
    ) -> Self;

    /// Run one learning iteration. Returns the iteration record.
    ///
    /// Each iteration:
    /// 1. Build a PassiveStack with current {J, h} and install on model.
    /// 2. Create fresh Data from the model.
    /// 3. Run `n_trajectories` independent simulations.
    /// 4. Measure ⟨σ_i⟩ and ⟨σ_i σ_j⟩ from the physical sampler.
    /// 5. Compute KL divergence (exact Ising at current params vs target).
    /// 6. Update: J += η · (⟨σσ⟩_target − ⟨σσ⟩_model).
    /// 7. Update: h += η · (⟨σ⟩_target − ⟨σ⟩_model).
    pub fn step(&mut self) -> LearningRecord;

    /// Run multiple iterations. Returns the full learning curve.
    pub fn train(&mut self, n_iterations: usize) -> Vec<LearningRecord>;

    /// Current coupling constants.
    pub fn coupling_j(&self) -> &[f64];

    /// Current external fields.
    pub fn field_h(&self) -> &[f64];
}
```

### 6.2 Training loop detail

Each call to `step()`:

1. **Rebuild stack and install**: Build a new `PassiveStack` with N
   `DoubleWellPotential` instances + 1 `PairwiseCoupling` with current
   `coupling_j` + 1 `ExternalField` with current `field_h` + 1
   `LangevinThermostat`. Install on the owned `model` (replaces the
   previous `cb_passive` unconditionally — verified safe; see
   `stack.rs:114-115`).

2. **Run trajectories**: For each trajectory `t ∈ 0..n_trajectories`:
   - Create fresh `Data` via `model.make_data()`.
   - Set initial condition: `qpos[i] = x₀`, `qvel[i] = 0` for all i.
   - Call `data.forward(&model)` to initialize derived quantities.
   - Run `n_burn_in` steps (discard).
   - Run `n_steps − n_burn_in` measurement steps.
   - At each measurement step: classify states, accumulate per-site spin
     and per-edge spin products (same protocol as Phase 4 §9.1–9.2).

3. **Aggregate statistics**: Two-level Welford: per-trajectory means →
   ensemble means. Extract `⟨σ_i⟩_model` (N values) and
   `⟨σ_i σ_j⟩_model` (|edges| values).

4. **Compute KL**: Use the `ising` module to compute the exact Ising
   distribution at the current `{J, h}` and at the target's `{J*, h*}`.
   Compute `KL(P_target ‖ P_current)`.

5. **Update parameters**:
   ```
   coupling_j[k] += η · (target.correlations[k] − measured_correlations[k])
   field_h[i]    += η · (target.magnetizations[i] − measured_magnetizations[i])
   ```

6. **Return** the `LearningRecord` with all iteration data.

### 6.3 Measurement protocol

Identical to Phase 4 §9.1–9.2, generalized to fully-connected edges:

- **State classification**: `WellState::from_position(qpos[i], x_thresh)`.
  Barrier samples excluded from statistics (same as Phase 4).
- **Per-site magnetization**: `⟨σ_i⟩ = (count_right − count_left) /
  (count_right + count_left)` for each element.
- **Per-edge correlation**: `⟨σ_i σ_j⟩ = mean(spin_i × spin_j)` over
  timesteps where BOTH elements are in wells.

---

## 7. Target Parameter Set

### 7.1 Target parameters

Fully connected N=4 graph. Mixed ferromagnetic and anti-ferromagnetic
couplings, non-zero external fields. All dimensionless at `kT = 1.0`.

```
Edges (lexicographic):  (0,1)  (0,2)  (0,3)  (1,2)  (1,3)  (2,3)
Target J*:               0.8   -0.3    0.1    0.5   -0.2    0.6
Target h*:               0.3   -0.2    0.0    0.15
```

**Rationale for these values**:
- **Mixed couplings**: Tests the learner's ability to recover both signs.
  `J*_01 = 0.8` (strong ferro), `J*_02 = −0.3` (anti-ferro),
  `J*_03 = 0.1` (weak ferro) — spanning the range.
- **Non-zero fields**: Tests symmetry-breaking learning. `h*_0 = 0.3`
  (right preference), `h*_1 = −0.2` (left preference), `h*_2 = 0` (no
  preference), `h*_3 = 0.15` (slight right).
- **Moderate magnitudes**: All |J| ≤ 0.8 and |h| ≤ 0.3, ensuring
  bistability is preserved (`|h · x₀| ≤ 0.3 ≪ ΔV = 3.0`; effective
  barrier asymmetry ≤ 10% of barrier height).

### 7.2 Target statistics (exact, from enumeration)

Computed by `ising::exact_distribution` + `ising::ising_statistics` at the
target parameters with `kT = 1.0`. The exact values will be verified in
the implementation; the qualitative structure:

- `⟨σ_0⟩ > 0` (field pushes right)
- `⟨σ_1⟩ < 0` (field pushes left)
- `⟨σ_2⟩ ≈ 0` (no field, but coupling effects may shift slightly)
- `⟨σ_3⟩ > 0` (field pushes right)
- `⟨σ_0 σ_1⟩ > 0` (strong ferro coupling dominates opposing fields)
- `⟨σ_0 σ_2⟩ < 0` (anti-ferro)

The most probable configurations should be `(+,+,+,+)` (all aligned,
favored by strong ferro couplings and net positive field) and `(−,−,−,−)`
(all anti-aligned, still favored by ferro couplings but penalized by field).
The alternating configurations `(+,−,+,−)` should be least probable.

### 7.3 Initial conditions

All parameters start at zero:
- `J_{ij} = 0` for all edges → no coupling → independent elements.
- `h_i = 0` for all sites → symmetric wells → uniform Ising distribution.

At initialization, the model distribution is uniform: `P(σ) = 1/16` for
all 16 configurations. The initial KL divergence is:

```
KL_init = log(16) − H(P_target)
```

where `H(P_target)` is the entropy of the target distribution. For the
chosen target (strongly non-uniform), `KL_init ≈ 1.0–1.5 nats`.

### 7.4 Learning rate and simulation budget

**Gate A (smoke test, default `#[test]`)**:
- Learning rate: `η = 0.5`
- Steps per trajectory: 1,000,000 (1M)
- Burn-in: 20,000
- Trajectories per iteration: 3
- Iterations: 30
- Total simulation: 90M steps (~0.5× Phase 4 Gate A budget)
- Seed base: `20_260_410_05` (Phase 5 seed space)

**Gate B (convergence, `#[ignore]`)**:
- Learning rate: `η = 0.3`
- Steps per trajectory: 2,000,000 (2M)
- Burn-in: 20,000
- Trajectories per iteration: 5
- Iterations: 60
- Total simulation: 600M steps (~3× Phase 4 Gate A budget)
- Seed base: `20_260_410_05 + 100_000`

The lower learning rate in Gate B reduces oscillation near the optimum,
allowing tighter convergence. The increased trajectories per iteration
reduce gradient noise.

**Seed strategy**: Each iteration `k` uses seed base `seed_base + k * 1000
+ traj_index`. This ensures:
- Different seeds per trajectory within an iteration.
- Different seeds across iterations.
- Reproducibility given the same seed base.

---

## 8. Validation Tests

Test file: `sim-thermostat/tests/boltzmann_learning.rs`.

### 8.1 Gate A — KL convergence (must-pass)

The primary gate. Runs the Boltzmann learning loop at Gate A parameters
(§7.4) and asserts that learning happened:

```rust
#[test]
fn gate_a_kl_convergence() {
    let model = sim_mjcf::load_model(CHAIN_XML).expect("load");
    let config = /* Gate A parameters from §7.4 */;
    let target = /* Target from §7.1, computed via ising module */;
    let mut learner = IsingLearner::new(config, target, model);
    let curve = learner.train(30);

    // Learning happened: final KL < initial KL
    let kl_init = curve[0].kl_divergence;
    let kl_final = curve.last().unwrap().kl_divergence;
    assert!(
        kl_final < kl_init * 0.3,
        "Gate A FAILED: KL did not decrease sufficiently.\n  \
         KL_init = {kl_init:.4}, KL_final = {kl_final:.4}",
    );

    // Final KL is in a reasonable range
    assert!(
        kl_final < 0.15,
        "Gate A FAILED: KL_final = {kl_final:.4} > 0.15",
    );
}
```

**Tolerances**:
- `kl_final < kl_init * 0.3`: learning reduced KL by at least 70%. This
  is loose — the convexity guarantee means the true optimum is KL=0 (up to
  mapping correction), so 70% reduction from ~1.0–1.5 nats is achievable in
  30 iterations.
- `kl_final < 0.15 nats`: absolute ceiling. At 0.15 nats the learned
  distribution is close to the target in total variation distance
  (`TV ≤ √(KL/2) ≈ 0.27`).

### 8.2 Gate B — Parameter recovery (`#[ignore]`)

Tighter test with more iterations and simulation budget. Asserts that the
learned parameters converge close to the target parameters.

```rust
#[test]
#[ignore = "600M steps — run with `cargo test -p sim-thermostat -- --ignored gate_b`"]
fn gate_b_parameter_recovery() {
    let config = /* Gate B parameters from §7.4 */;
    let target_j = [0.8, -0.3, 0.1, 0.5, -0.2, 0.6];
    let target_h = [0.3, -0.2, 0.0, 0.15];
    let target = /* computed from target_j, target_h */;
    let model = sim_mjcf::load_model(CHAIN_XML).expect("load");
    let mut learner = IsingLearner::new(config, target, model);
    let curve = learner.train(60);

    // KL convergence
    let kl_final = curve.last().unwrap().kl_divergence;
    assert!(
        kl_final < 0.05,
        "Gate B KL FAILED: {kl_final:.4} > 0.05",
    );

    // Parameter recovery (allowing ~15% relative error for mapping correction)
    let learned_j = learner.coupling_j();
    let learned_h = learner.field_h();
    for (k, (&learned, &target)) in learned_j.iter().zip(&target_j).enumerate() {
        let abs_error = (learned - target).abs();
        assert!(
            abs_error < 0.20,
            "Gate B J recovery FAILED on edge {k}: |J_learned − J*| = {abs_error:.4} > 0.20\n  \
             learned = {learned:.4}, target = {target:.4}",
        );
    }
    for (i, (&learned, &target)) in learned_h.iter().zip(&target_h).enumerate() {
        let abs_error = (learned - target).abs();
        assert!(
            abs_error < 0.15,
            "Gate B h recovery FAILED on site {i}: |h_learned − h*| = {abs_error:.4} > 0.15\n  \
             learned = {learned:.4}, target = {target:.4}",
        );
    }
}
```

**Tolerances**:
- KL < 0.05 nats: much tighter than Gate A. Achievable with 60 iterations
  at η=0.3 and 10M steps per iteration.
- |J_learned − J*| < 0.20: ~25% of the target range [−0.3, 0.8]. The ~5%
  mapping correction shifts the optimum by ~0.04; the remaining budget is
  for gradient noise and learning-rate-induced oscillation.
- |h_learned − h*| < 0.15: similar reasoning, tighter because fields are
  smaller magnitude.

### 8.3 Supporting — Learning curve monotonicity

Verify that the learning curve is monotonically decreasing in a smoothed
sense (the raw curve will have noise from finite sampling):

```rust
// Smooth KL over a 5-iteration window, check that each window's mean
// is lower than the previous window's mean.
let window = 5;
let smoothed: Vec<f64> = curve.windows(window)
    .map(|w| w.iter().map(|r| r.kl_divergence).sum::<f64>() / window as f64)
    .collect();
for i in 1..smoothed.len() {
    assert!(
        smoothed[i] < smoothed[i - 1] * 1.05, // 5% tolerance for noise
        "Smoothed KL not decreasing: window {i} = {:.4} >= window {} = {:.4}",
        smoothed[i], i - 1, smoothed[i - 1],
    );
}
```

### 8.4 Supporting — ExternalField symmetry breaking

A dedicated test that the `ExternalField` component correctly breaks
well symmetry:

```rust
#[test]
fn external_field_breaks_symmetry() {
    // Single element, double well, external field h=0.5.
    // Run thermostat. Measure P(right) > P(left).
    // ...
    assert!(p_right > 0.6, "field h=0.5 should bias to right well");
}
```

### 8.5 Supporting — Reproducibility

Same-seed runs of the full learning loop produce bit-identical parameter
trajectories:

```rust
#[test]
fn learning_reproducibility() {
    // Run 5 iterations with seed S. Record final J, h.
    // Run 5 iterations with seed S again. Assert bit-equal.
}
```

---

## 9. Acceptance Criteria

Phase 5 passes when **all** of the following are green:

1. `cargo test -p sim-thermostat` passes (all default tests, including
   Gate A).
2. `cargo test -p sim-thermostat -- --ignored gate_b_parameter_recovery`
   passes.
3. `cargo clippy -p sim-thermostat -- -D warnings` clean.
4. `cargo fmt -p sim-thermostat -- --check` clean.
5. No regressions in Phase 1–4 tests.
6. All existing `PairwiseCoupling` unit tests updated for per-edge API and
   passing.
7. Spec (this document) updated with "Done" status and commit hash.

---

## 10. Implications for Future Phases

### 10.1 D4 readiness

Phase 5's `IsingLearner` is the EBM training algorithm D4 needs. D4's
workflow: choose a target distribution → train with `IsingLearner` → freeze
parameters → print the device → measure. The training step is exactly
Phase 5's deliverable.

### 10.2 Phase 6 boundary

Phase 6 introduces a CPU Gibbs sampler that draws from the same energy
function. The `ising` module's exact distribution and KL divergence
utilities are directly reusable: Phase 6 compares Gibbs samples against
`exact_distribution`, just as Phase 5 compares Langevin samples.

The full joint-distribution comparison deferred in Phase 4 §11.4 is now
enabled by the `ising` module: total variation distance between the
Langevin-measured configuration histogram and the exact Ising reference,
with per-configuration frequency matching.

### 10.3 Higher-order coupling

The pairwise Ising model cannot represent arbitrary distributions (§3.4).
To increase expressiveness, future phases could:

1. **Three-body coupling**: `V₃ = −K_{ijk} · x_i · x_j · x_k`. Adds
   `N choose 3` parameters (4 for N=4). The Boltzmann learning rule
   extends naturally: `∂KL/∂K_{ijk} ∝ ⟨σ_i σ_j σ_k⟩_target − ⟨σ_i σ_j σ_k⟩_model`.
2. **Non-bilinear coupling**: Replace `J · x_i · x_j` with a learned
   nonlinear function `f(x_i, x_j; θ)`. Requires differentiating through
   the coupling function — a natural entry point for the autograd engine
   if/when it becomes relevant.
3. **Larger N**: Scale to N=8 or N=16. The exact Ising reference becomes
   expensive (2^16 = 65K configs), but the Boltzmann learning rule itself
   scales linearly with N and |edges|.

None of these are in Phase 5's scope. The pairwise model is sufficient to
demonstrate the core capability: **a physical system trained as a generative
model**.

### 10.4 Phase 5 does NOT require cf-design

Worth stating explicitly: Phase 5 has zero dependency on cf-design. The
well geometry is prescribed (quartic double-well), the coupling is bilinear,
and the training algorithm operates entirely on `sim-thermostat` components.
This is consistent with the Q5 resolution (D3 blocked, but D1/D2/D4/Phase 5
are all unblocked).

---

## 11. Module Layout Summary

After Phase 5, `sim-thermostat/src/` contains:

```
src/
├── component.rs          # PassiveComponent, Stochastic traits (unchanged)
├── diagnose.rs           # Diagnose trait (unchanged)
├── double_well.rs        # DoubleWellPotential (unchanged)
├── external_field.rs     # ExternalField (NEW)
├── ising.rs              # Exact Ising distribution, stats, KL (NEW)
├── ising_learner.rs      # IsingLearner, LearnerConfig, etc. (NEW)
├── langevin.rs           # LangevinThermostat (unchanged)
├── lib.rs                # Re-exports (updated)
├── pairwise_coupling.rs  # PairwiseCoupling (MODIFIED: per-edge J)
├── stack.rs              # PassiveStack, EnvBatch, etc. (unchanged)
└── test_utils.rs         # WelfordOnline, WellState, etc. (unchanged)

tests/
├── boltzmann_learning.rs # Phase 5 validation (NEW)
├── coupled_bistable_array.rs # Phase 4 (unchanged)
├── kramers_escape_rate.rs    # Phase 3 (unchanged)
└── langevin_thermostat.rs    # Phase 1-2 (unchanged)
```

New production code: ~3 files, ~400–500 LOC.
New test code: ~1 file, ~250–350 LOC.
Modified: `pairwise_coupling.rs` (~50 LOC diff), `lib.rs` (~5 LOC).
