# Phase 6 — Native Gibbs Sampler + Three-Way Distribution Comparison

> **Status**: Spec draft.
> **Owner**: Jon
> **Parents**:
> - [`05_boltzmann_learning.md`](./05_boltzmann_learning.md) (Phase 5 — the gate this phase inherits)
> - Phase 4 §11.4 (full joint-distribution comparison — the explicit deferral this phase closes)
> - Phase 5 §10.2 (Phase 6 boundary — `ising` module reusable, Gibbs vs exact comparison)
> - [`../02_foundations/open_questions.md`](../02_foundations/open_questions.md) Q3 (RESOLVED: option B — native block-Gibbs in Rust)
> - [`overview.md`](./overview.md) Phase 6 row

Phase 6 introduces a CPU Gibbs sampler that draws from the same Ising energy
function as the physical Langevin sampler. This enables a three-way
comparison — Gibbs vs exact, Langevin vs exact, Langevin vs Gibbs — that
isolates "does the Langevin thermostat sample the Ising distribution
correctly?" from "does the continuous-to-Ising mapping hold?"

Phases 4 and 5 could only compare the Langevin sampler against the exact
Ising distribution, conflating two error sources: (1) the Langevin
thermostat's sampling accuracy, and (2) the ~5% anharmonic correction in
the continuous-to-Ising mapping (Phase 4 §3.2). Phase 6 decouples them.

---

## 1. Goal

Demonstrate that:

1. **Gibbs sampler correctness**: A single-site Gibbs sampler on the Ising
   energy function produces a configuration histogram that matches the exact
   distribution (from `exact_distribution`) to within statistical tolerance.
   TV distance < 0.02.
2. **Three-way isolation**: TV(Gibbs, exact) << TV(Langevin, exact). The
   Gibbs sampler's error is purely statistical (vanishes with more samples).
   The Langevin sampler's error includes the irreducible mapping error from
   coarse-graining continuous positions to binary spins.
3. **Phase 4 §11.4 closure**: Full 2^N joint-distribution comparison for
   the Langevin sampler. TV distance between the Langevin-measured
   configuration histogram and the exact Ising reference, with
   per-configuration frequency data reported. This closes the explicit
   deferral from Phase 4.

---

## 2. Scope

**In scope for Phase 6:**

- **New `GibbsSampler` struct** (§5): single-site systematic-scan Gibbs
  sampler for pairwise Ising models with external fields. Production code
  in `sim-thermostat`.
- **New `tv_distance` function** (§5.3): total variation distance between
  two distributions, added to the `ising` module alongside `kl_divergence`.
- **Validation tests** (§8): Gibbs correctness gate (Gate A), three-way
  comparison gate (Gate B), Phase 4 §11.4 closure gate (Gate C),
  reproducibility.
- **`gibbs` module** in `sim-thermostat`: public module exporting
  `GibbsSampler`.

**Explicitly out of scope:**

- **Block-Gibbs**: Single-site systematic scan suffices for pairwise Ising
  at N=4. The conditional `P(σ_i | σ_{-i})` is closed-form; block updates
  add complexity with no accuracy benefit. The `gibbs` module name leaves
  the upgrade path open (§10.1).
- **GPU Gibbs**: CPU-only. N ≤ 20 (inherited from `exact_distribution`'s
  safety limit). GPU is unnecessary at this scale.
- **Continuous-energy Gibbs**: The Gibbs sampler operates on discrete
  spins `σ ∈ {−1, +1}^N`, not on continuous positions. Comparing against
  the Langevin sampler requires the same spin-classification
  (`WellState::from_position`) used in Phases 4–5. The comparison is
  "both samplers → spin histogram → TV distance", not "Gibbs samples
  continuous positions."
- **Non-Ising models**: Pairwise Ising with external fields only. Higher-
  order coupling (Phase 5 §10.3) is future work.
- **Modifications to `IsingLearner`**: Phase 6 is a validation/comparison
  phase, not a training phase. The Gibbs sampler is a standalone reference
  tool.

---

## 3. Physics: Gibbs Sampling on the Ising Model

### 3.1 The Ising energy (inherited)

The Ising Hamiltonian (same as `ising::exact_distribution`):

```
H(σ) = −Σ_{(i,j) ∈ edges} J_{ij} · σ_i · σ_j − Σ_i h_i · σ_i
```

Boltzmann distribution: `P(σ) = exp(−H(σ) / kT) / Z`.

### 3.2 Single-site Gibbs conditional

For site `i`, given all other spins `σ_{-i}`, the conditional probability:

```
local_field_i = Σ_{j: (i,j) ∈ edges} J_{ij} · σ_j + h_i

P(σ_i = +1 | σ_{-i}) = 1 / (1 + exp(−2 · local_field_i / kT))
```

**Derivation**: The energy difference between `σ_i = +1` and `σ_i = −1`
(all other spins fixed) is:

```
ΔH = H(σ_i = −1) − H(σ_i = +1) = 2 · local_field_i
```

So `P(+1) / P(−1) = exp(ΔH / kT) = exp(2 · local_field_i / kT)`, giving
the sigmoid form above.

### 3.3 Systematic scan

One Gibbs **sweep** updates all N sites in index order `i = 0, 1, …, N−1`.
Each site is updated by:

1. Compute `local_field_i` from the *current* spins of all neighbors.
2. Draw `u ~ Uniform(0, 1)`.
3. Set `σ_i = +1` if `u < P(σ_i = +1 | σ_{-i})`, else `σ_i = −1`.

Systematic scan (deterministic order) is simpler and has the same
stationary distribution as random-scan Gibbs for the Ising model. At N=4,
mixing is fast regardless of scan order.

### 3.4 Convergence properties

For pairwise Ising at N=4 with fully-connected topology:

- **Mixing time**: O(1) sweeps at moderate coupling. The Gibbs sampler
  mixes in ~10–50 sweeps for the parameter ranges used in Phases 4–5
  (`|J| ≤ 1.0`, `|h| ≤ 0.5`, `kT = 1.0`).
- **Burn-in**: Conservative 1000 sweeps (far exceeds mixing time). After
  burn-in, every sweep produces a valid sample.
- **Autocorrelation**: At N=4, adjacent sweeps produce nearly independent
  samples. No thinning needed for the histogram construction (we sum counts,
  not compute time-series statistics).

### 3.5 Error structure

The three-way comparison disentangles two error sources:

```
TV(Gibbs, exact)    ≈ statistical noise only (→ 0 with more sweeps)
TV(Langevin, exact) = mapping error + statistical noise
TV(Langevin, Gibbs) ≈ mapping error + statistical noise (from both)
```

**Mapping error** is the irreducible ~5% anharmonic correction from
Phase 4 §3.2: the continuous double-well's occupation probabilities don't
exactly match the Ising model's probabilities because the well shape is
quartic (not infinitely steep). This error is a property of the
continuous-to-discrete coarse-graining, not of the Langevin thermostat.

**Expected ordering**: TV(Gibbs, exact) << TV(Langevin, exact) ≈
TV(Langevin, Gibbs). If this ordering fails, something is wrong with
either the Gibbs sampler or the Langevin thermostat.

### 3.6 Langevin effective sample count

The Gibbs sampler produces nearly independent samples every sweep (~N
site updates). The Langevin sampler does not — consecutive simulation
steps are highly autocorrelated. The effective number of independent
joint-configuration observations from a Langevin trajectory is set by
the Kramers switching rate, not the step count.

At the Phase 4/5 central parameters (`ΔV = 3.0`, `kT = 1.0`, `γ = 10.0`,
`x₀ = 1.0`, `h = 0.001`), the overdamped Kramers escape rate per element
is:

```
r = (ω_well · ω_barrier) / (2π · γ) · exp(−ΔV / kT)
  ≈ (√24 · √12) / (2π · 10) · exp(−3)
  ≈ 0.013 per time unit
  ≈ 1.3 × 10⁻⁵ per step
```

One switching event every ~75,000 steps per element. The integrated
autocorrelation time for the joint spin configuration is on the same
order: `τ_int ≈ 75,000` steps. The effective independent sample count
for a trajectory of `N_measure` steps is:

```
N_eff ≈ N_measure / (2 · τ_int) ≈ N_measure / 150,000
```

For a 16-bin histogram with `K` independent observations, the expected
TV distance from statistical noise alone is approximately:

```
TV_noise ≈ √(B / (2π · K))    [B = 2^N = 16 bins]
```

**Calibration against tolerance**: To achieve TV(Langevin, exact) < 0.15
with margin, we need TV_noise < 0.08 (leaving ~0.07 for mapping error).
This requires `K ≥ 400` independent observations. At 150K steps per
independent observation, that's ~60M measurement steps total. Using 10
independent trajectories of 5M steps each: `N_eff = 10 × (5M / 150K)
≈ 330`, giving `TV_noise ≈ 0.062`. Combined with ~5% mapping error
(TV ≈ 0.05–0.08), total TV ≈ 0.08–0.12 — passes with margin.

This is why Phase 4 used 20 × 10M = 200M steps for its (less demanding)
qualitative histogram check, and why Gates B and C below use 10 × 5M
rather than Phase 5's 3 × 1M (which was sufficient for correlations but
not for full 2^N histogram accuracy).

---

## 4. Design Decision: Single-Site Gibbs (Q3 Option B)

Q3 in `open_questions.md` resolved with three options; this spec commits
to **Option B: native single-site Gibbs sampler in Rust**.

**Why not block-Gibbs**: For pairwise Ising, the single-site conditional
is already exact (§3.2). Block-Gibbs computes the conditional distribution
over a subset of sites jointly — useful for models with higher-order
interactions where single-site conditionals don't capture the full local
structure, but unnecessary for pairwise models. At N=4, every block is at
most 4 sites, and the single-site sweep already visits all sites per sweep.

**Why not Option A (community port dependency)**: Neither `thrml-rs` port
is on crates.io or officially maintained. "Compatibility NOT guaranteed"
(Pingasmaster) is a red flag for correctness-critical validation. We need
exactly one thing — a single-site Gibbs sampler for pairwise Ising — which
is ~100 LOC. The dependency cost far exceeds the implementation cost.

**Why not Option C (vendor/fork)**: Same reasoning as Option A, plus
maintenance burden of a forked crate for ~100 LOC of useful code.

**Implementation cost**: ~100 LOC for `GibbsSampler`, ~15 LOC for
`tv_distance`, ~200 LOC for tests. Consistent with the "narrow crates,
full control" reasoning that drove the `sim-thermostat` sibling-crate
decision.

---

## 5. API Surface

### 5.1 `GibbsSampler` struct

```rust
// sim-thermostat/src/gibbs.rs

use rand::Rng;
use rand_chacha::ChaCha8Rng;

/// Single-site systematic-scan Gibbs sampler for pairwise Ising models.
///
/// Operates on discrete spins σ ∈ {−1, +1}^N with the same Hamiltonian
/// as [`exact_distribution`](crate::ising::exact_distribution):
///
/// ```text
/// H(σ) = −Σ J_{ij} σ_i σ_j − Σ h_i σ_i
/// ```
///
/// Each call to [`sweep`] updates all N sites in index order using the
/// exact conditional P(σ_i | σ_{-i}) = sigmoid(2·local_field_i / kT).
pub struct GibbsSampler {
    n: usize,
    edges: Vec<(usize, usize)>,
    coupling_j: Vec<f64>,
    field_h: Vec<f64>,
    k_b_t: f64,
    spins: Vec<f64>,          // +1.0 or −1.0
    rng: ChaCha8Rng,
    /// Precomputed adjacency: for each site i, a list of (j, J_{ij}).
    neighbors: Vec<Vec<(usize, f64)>>,
}
```

**Constructor**:

```rust
impl GibbsSampler {
    /// Create a new Gibbs sampler with all spins initialized to +1.
    ///
    /// # Panics
    /// - If `n > 20` (inherited safety limit).
    /// - If `coupling_j.len() != edges.len()`.
    /// - If `field_h.len() != n`.
    /// - If `k_b_t <= 0`.
    pub fn new(
        n: usize,
        edges: &[(usize, usize)],
        coupling_j: &[f64],
        field_h: &[f64],
        k_b_t: f64,
        seed: u64,
    ) -> Self;
}
```

**Methods**:

```rust
impl GibbsSampler {
    /// Perform one full sweep: update each site once in index order.
    pub fn sweep(&mut self);

    /// Current spin configuration as a bitmask (matching
    /// `exact_distribution`'s format: bit i set = σ_i = +1).
    pub fn config_bitmask(&self) -> u32;

    /// Run `n_burn_in` sweeps (discarded) then `n_samples` sweeps,
    /// collecting a configuration histogram. Returns the empirical
    /// distribution in the same `Vec<(u32, f64)>` format as
    /// `exact_distribution` — all 2^N configs present, sorted by
    /// config index, probabilities summing to 1.
    pub fn sample(&mut self, n_burn_in: usize, n_samples: usize) -> Vec<(u32, f64)>;
}
```

The `sample` method returns the same format as `exact_distribution`,
enabling direct use with `kl_divergence` and the new `tv_distance`.

### 5.2 Precomputed adjacency

To avoid scanning all edges on every site update, the constructor builds a
`neighbors` adjacency list: `neighbors[i]` contains all `(j, J_{ij})`
pairs where `(i, j)` or `(j, i)` appears in the edge list. This makes
`local_field_i` computation O(degree) instead of O(|edges|).

At N=4 fully-connected (6 edges), this is a micro-optimization. It
matters more if Phase 6+ ever tests N=8 or N=16 topologies.

### 5.3 `tv_distance` function

Added to `ising.rs` alongside `kl_divergence`:

```rust
/// Total variation distance between two distributions over the same
/// configuration space.
///
/// TV(P, Q) = ½ Σ_c |P(c) − Q(c)|
///
/// Returns a value in [0, 1]. Unlike KL divergence, TV distance is
/// symmetric and bounded, making it more suitable for validation gates.
///
/// Both distributions must have the same length and be sorted by config
/// index (as returned by `exact_distribution`).
///
/// # Panics
/// Panics if `p.len() != q.len()`.
pub fn tv_distance(p: &[(u32, f64)], q: &[(u32, f64)]) -> f64;
```

### 5.4 Module structure

```
sim-thermostat/src/
├── gibbs.rs          ← NEW: GibbsSampler
├── ising.rs          ← MODIFIED: add tv_distance
├── lib.rs            ← MODIFIED: add `pub mod gibbs;`, re-export GibbsSampler
├── ...               (existing modules unchanged)

sim-thermostat/tests/
├── gibbs_sampler.rs  ← NEW: Phase 6 validation tests
├── ...               (existing tests unchanged)
```

**Export change in `lib.rs`**:

```rust
pub mod gibbs;
pub mod ising;
// ...
pub use gibbs::GibbsSampler;
```

### 5.5 No new dependencies

`GibbsSampler` uses `rand::Rng` (for `gen::<f64>()`) and
`rand_chacha::ChaCha8Rng` (deterministic PRNG). Both are already in
`sim-thermostat`'s dependency set. No new crate dependencies.

---

## 6. Implementation Plan

Three implementation steps, each producing a testable increment:

### Step 1: `tv_distance` in `ising.rs`

- Add `tv_distance` function (§5.3).
- Add unit tests: TV(P, P) = 0; TV(P, Q) > 0 for different P, Q;
  TV(P, Q) = TV(Q, P) (symmetry); TV is in [0, 1]; TV of uniform vs
  delta ≈ 1 − 1/2^N.

### Step 2: `GibbsSampler` in `gibbs.rs`

- Implement `GibbsSampler` (§5.1).
- Add `pub mod gibbs` and `pub use gibbs::GibbsSampler` to `lib.rs`.
- Unit tests within `gibbs.rs`:
  - Uniform distribution at J=0, h=0 (all configs equiprobable).
  - Deterministic reproducibility (same seed → same histogram).
  - `config_bitmask` matches spin state.

### Step 3: Integration tests in `gibbs_sampler.rs`

- Gate A: Gibbs correctness (§8.1).
- Gate B: Three-way comparison (§8.2).
- Gate C: Phase 4 §11.4 closure (§8.3).
- Reproducibility (§8.4).

---

## 7. MJCF Model

Gate B and Gate C reuse the Phase 4/5 MJCF model (4 independent slide-joint
bodies). Reproduced here for completeness:

```xml
<mujoco model="bistable_chain_4">
  <option timestep="0.001" gravity="0 0 0" integrator="Euler"/>
  <worldbody>
    <body name="p0" pos="0 0 0">
      <joint name="x0" type="slide" axis="1 0 0" stiffness="0" damping="0"/>
      <geom type="sphere" size="0.05" mass="1" contype="0" conaffinity="0"/>
    </body>
    <body name="p1" pos="0 1 0">
      <joint name="x1" type="slide" axis="1 0 0" stiffness="0" damping="0"/>
      <geom type="sphere" size="0.05" mass="1" contype="0" conaffinity="0"/>
    </body>
    <body name="p2" pos="0 2 0">
      <joint name="x2" type="slide" axis="1 0 0" stiffness="0" damping="0"/>
      <geom type="sphere" size="0.05" mass="1" contype="0" conaffinity="0"/>
    </body>
    <body name="p3" pos="0 3 0">
      <joint name="x3" type="slide" axis="1 0 0" stiffness="0" damping="0"/>
      <geom type="sphere" size="0.05" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
```

Gate A uses no MJCF model — it's a pure software test of the Gibbs sampler
against `exact_distribution`.

---

## 8. Validation Tests

### 8.1 Gate A — Gibbs sampler correctness (must-pass)

**What**: Run the Gibbs sampler on a fully-connected N=4 Ising model with
known parameters, collect a configuration histogram, and compare against
`exact_distribution` via TV distance.

**Parameters**: Reuse Phase 5's target parameters for familiarity:
- `J = [0.8, −0.3, 0.1, 0.5, −0.2, 0.6]` (fully-connected, 6 edges)
- `h = [0.3, −0.2, 0.0, 0.15]`
- `kT = 1.0`
- `n_burn_in = 1_000` sweeps
- `n_samples = 100_000` sweeps

**Gate criterion**: `TV(Gibbs, exact) < 0.02`.

**Tolerance justification**: With 100K independent samples over 16 bins,
the expected per-bin count for the least-likely configuration is ~1000
(even for the rarest config, which has probability ~1/60 at these
parameters). The standard error of the empirical probability for a bin
with probability `p` over `N` samples is `sqrt(p(1−p)/N)`. For the
worst case (`p ≈ 0.01`, `N = 100_000`): `SE ≈ 0.0003`. The TV distance
accumulates errors across bins, but with 16 bins the expected TV under
pure sampling noise is `~√(16) · 0.0003 ≈ 0.001`. The threshold of 0.02
gives ~20× headroom.

**Also test**: `KL(exact ‖ Gibbs) < 0.01` — confirms KL and TV tell the
same story.

### 8.2 Gate B — Three-way comparison (must-pass)

**What**: Run both the Gibbs sampler and the Langevin sampler on the same
Ising parameters, collect configuration histograms from both, compute all
three TV distances.

**Parameters**:
- Ising parameters: same as Gate A.
- Langevin parameters: inherited from Phase 4/5 (`ΔV = 3.0`, `x₀ = 1.0`,
  `γ = 10.0`, `kT = 1.0`, `x_thresh = x₀/2`).
- Gibbs: 1_000 burn-in, 100_000 sample sweeps.
- Langevin: 10 trajectories × 5_000_000 steps (20_000 burn-in each).

**Sample-size justification** (§3.6): At these parameters the Kramers
switching time is ~75K steps per element, giving `τ_int ≈ 75K`. With
10 × 5M measurement steps: `N_eff ≈ 10 × (5M / 150K) ≈ 330` independent
joint-config observations, `TV_noise ≈ √(16 / (2π × 330)) ≈ 0.062`.
Combined with mapping error (~0.05–0.08), total TV ≈ 0.08–0.12, passing
the 0.15 threshold with comfortable margin. (Phase 5's 3 × 1M was
sufficient for correlations but gives only ~20 effective config
observations — far too few for a 16-bin histogram gate.)

**Gate criteria**:

1. `TV(Gibbs, exact) < 0.02` — Gibbs is a good sampler (redundant with
   Gate A, but confirms it in the three-way context).
2. `TV(Langevin, exact) < 0.15` — the Langevin sampler produces a
   reasonable approximation. The ~5% mapping error (Phase 4 §3.2) plus
   statistical noise should stay well below 0.15 at 330 effective samples.
3. `TV(Gibbs, exact) < TV(Langevin, exact)` — Gibbs is strictly closer to
   exact than Langevin. This is the load-bearing assertion: it proves the
   Langevin-to-exact gap is dominated by mapping error, not sampler error.
4. `TV(Langevin, Gibbs) > 0.01` — there IS a nonzero mapping error (the
   Langevin sampler is not secretly equivalent to Gibbs). This catches the
   degenerate case where the mapping is accidentally perfect and the test
   is vacuous.

**Diagnostic output** (eprintln, not assertions):
- Per-configuration probabilities from all three distributions (exact,
  Gibbs, Langevin).
- All three TV distances.
- Per-configuration |P_exact − P_langevin| to identify which configs
  contribute most to the mapping error.

### 8.3 Gate C — Phase 4 §11.4 closure (must-pass)

**What**: Full 2^N joint-distribution comparison for the Langevin sampler
on the **Phase 4 nearest-neighbor chain** topology (not the fully-connected
graph). This directly closes the deferral.

**Parameters**:
- Topology: NN chain `[(0,1), (1,2), (2,3)]` with uniform `J = 0.5`.
- Fields: `h = [0; 4]` (no external field — Phase 4 had none).
- Langevin: `ΔV = 3.0`, `x₀ = 1.0`, `γ = 10.0`, `kT = 1.0`,
  10 trajectories × 5_000_000 steps (20_000 burn-in each) — same as
  Gate B. (Phase 4 used 20 × 10M on this topology; 10 × 5M gives
  `N_eff ≈ 330`, sufficient for quantitative TV.)
- Gibbs: 1_000 burn-in, 100_000 sample sweeps on the same chain topology.

**Gate criteria**:

1. `TV(Gibbs, exact) < 0.02` — Gibbs works on the chain topology.
2. `TV(Langevin, exact) < 0.10` — tighter than Gate B because the chain
   topology has weaker coupling (3 edges vs 6), so the mapping error
   should be smaller.
3. `TV(Gibbs, exact) < TV(Langevin, exact)` — same isolation check.

**Diagnostic output**: Full 16-row table of `(config, P_exact, P_gibbs,
P_langevin)` printed to stderr.

This gate closes Phase 4 §11.4. The `overview.md` Phase 4 row and
Phase 4 §11.4 will be updated with a forward reference to this gate.

### 8.4 Reproducibility

Same-seed runs produce bit-identical histograms.

---

## 9. Acceptance Criteria

Phase 6 is complete when:

1. `GibbsSampler` is implemented and exported from `sim-thermostat`.
2. `tv_distance` is implemented in `ising.rs` with unit tests.
3. Gates A, B, C all pass: `cargo test -p sim-thermostat gibbs`.
4. Reproducibility test passes.
5. `overview.md` Phase 6 row updated to **done** with spec link.
6. Phase 4 §11.4 updated with "CLOSED by Phase 6 Gate C" annotation.
7. `open_questions.md` Q3 updated to reference this spec as the
   commitment (currently says "Phase 6 spec will decide").

---

## 10. Implications for Future Phases

### 10.1 Block-Gibbs upgrade path

If future phases need higher-order coupling (Phase 5 §10.3) or larger N,
block-Gibbs can be added to the `gibbs` module as a second struct
(`BlockGibbsSampler`) without breaking the single-site API. The `sample`
return type is already topology-agnostic.

### 10.2 Phase 7 readiness

Phase 7 (RL on the sampler) needs a reward signal. The Gibbs sampler +
TV distance infrastructure directly enables two reward candidates:
- **TV(model_samples, target)** — measures how well the current Ising
  parameters match the target.
- **Effective sample size (ESS)** — measures how efficiently the Langevin
  sampler explores the state space.

Phase 6's deliverables are prerequisites for Phase 7's reward design
(Q6 in `open_questions.md`).

### 10.3 D4 readiness

D4 (sim-to-real) uses the Gibbs sampler as the "expected" distribution
for the printed device's measured samples. The `GibbsSampler::sample`
output can be directly compared against physical-device histograms via
`tv_distance`.

---

## 11. Explicit Deferrals

### 11.1 Langevin configuration histogram infrastructure as reusable function

The Langevin configuration histogram measurement (run trajectories,
classify spins, build histogram) is currently inlined in the Phase 4
test and will be partially duplicated in Gates B and C. Extracting a
reusable `measure_langevin_histogram(model, config, ...) -> Vec<(u32, f64)>`
helper is useful but not Phase 6's job. Defer to if/when Phase 7 or D4
also needs it.

### 11.2 Mixing time analysis

Phase 6 uses a conservative burn-in (1000 sweeps) without formally
measuring the mixing time or autocorrelation. For N=4 pairwise Ising at
moderate coupling, this is overkill. If N scales up (Phase 5 §10.3),
formal mixing diagnostics become necessary. Defer to that phase.
