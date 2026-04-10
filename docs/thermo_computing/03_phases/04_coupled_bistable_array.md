# Phase 4 — Coupled Bistable Array + Ising Comparison

> **Status**: Spec'd. Pending implementation.
> **Owner**: Jon
> **Parents**:
> - [`03_bistable_kramers.md`](./03_bistable_kramers.md) (Phase 3 — the gate this phase inherits)
> - Phase 3 §9.1 (hysteresis-based state classification — inherited for joint state measurement)
> - Phase 3 §11 (Euler finding — slide joints sidestep it; same reasoning applies here)
> - [`overview.md`](./overview.md) Phase 4 sketch
> - [`../01_vision/research_directions.md`](../01_vision/research_directions.md) D2 (gates on Phase 4)

This spec defines the first phase with *coupling* between bistable elements.
The deliverable is a `PairwiseCoupling` component in `sim-thermostat` and
validation tests proving that an array of 4 coupled double-well elements,
under the Langevin thermostat, produces pairwise correlations consistent
with the exact Ising model on the same coupling topology. Passing these
gates is the entrance condition for Phase 5 (differentiable EBM) and
unlocks D2 (stochastic resonance).

---

## 1. Goal

Demonstrate that an array of N=4 bistable elements (quartic double-well
potentials), coupled via bilinear pairwise interactions and driven by the
Phase 1 `LangevinThermostat`, produces:

1. **Ising-consistent correlations**: Nearest-neighbor pairwise correlations
   `⟨σᵢσᵢ₊₁⟩` match the exact Ising prediction `tanh(βJ_eff)` to within
   ±25%.
2. **Marginal symmetry**: Each element's marginal state distribution is
   symmetric: `P(σᵢ = +1) ≈ 0.5` (zero effective external field).
3. **Coupling-strength scaling**: Correlations increase with coupling
   strength `J` in the direction predicted by the Ising model.
4. **Per-DOF equipartition**: The thermostat maintains correct kinetic
   temperature `⟨½Mv²⟩ = ½kT` on each DOF despite coupling.

The physics test shifts from single-element dynamics (Phase 3: escape rate)
to **correlated equilibrium**: the first test that the thermostat +
`PassiveComponent` composition produces correct *joint* statistics across
coupled DOFs.

---

## 2. Scope

**In scope for Phase 4:**
- One new `PassiveComponent`: `PairwiseCoupling` (§6), implementing bilinear
  coupling `V = −J · xᵢ · xⱼ` with `PassiveComponent + Diagnose`.
- One MJCF model: 4 independent slide-joint bodies (§5).
- Six validation tests: NN correlation gate (§8.1), coupling sweep (§8.2),
  marginal symmetry (§8.3), NNN/long-range correlations (§8.4),
  configuration frequency ratio (§8.5), reproducibility (§8.6).
- Joint state classification infrastructure (§9.1), extending Phase 3's
  hysteresis threshold to N elements.
- Promotion of `WellState` from Phase 3's test-local definition to
  `sim_thermostat::test_utils` (§9.1) — shared by Phase 4+ tests.

**Explicitly out of scope:**
- **Ring / fully-connected topologies** — trivial to enable via
  `PairwiseCoupling`'s edge-list API, but the validation tests only cover
  the 1D chain.
- **N > 4 arrays** — the component and test infrastructure are parameterized
  in N, but only N=4 is validated.
- **cf-design bistable elements** — Phase 4 uses prescribed quartic
  potentials (same as Phase 3). cf-design integration is a D2 concern.
- **Contact constraints** — the 4 slide-joint bodies have no contacts. Q1
  constraint-projection testing remains deferred.
- **Anti-ferromagnetic validation** — `J > 0` only (ferromagnetic). The
  `PairwiseCoupling` component supports `J < 0` by construction, but the
  Ising comparison is only validated for `J > 0`.
- **Changes to existing `sim-thermostat/src/` files** other than adding the
  new module, re-exporting from `lib.rs`, and promoting `WellState` to
  `test_utils.rs` (§9.1).

---

## 3. Physics: The Ising Mapping

### 3.1 The bilinear coupling

Each pair of coupled elements `(i, j)` interacts through a bilinear
potential:

```
V_pair(xᵢ, xⱼ) = −J · xᵢ · xⱼ
```

where `J > 0` is the coupling constant (ferromagnetic: aligned positions
are energetically favored). The force on DOF i from coupling to DOF j:

```
F_i = −∂V_pair/∂xᵢ = +J · xⱼ
```

For a chain topology with edges `{(0,1), (1,2), (2,3)}`, the total coupling
energy is:

```
V_coupling = −J · (x₀x₁ + x₁x₂ + x₂x₃)
```

The total system energy:

```
E(x₀, ..., x₃) = Σᵢ V(xᵢ) − J · Σ_{⟨ij⟩} xᵢxⱼ
```

where `V(x) = a(x² − x₀²)²` is the quartic double-well from Phase 3.

**Why bilinear and not spring**: A spring coupling `½K(xᵢ − xⱼ)²` adds a
harmonic restoring term `K·xᵢ²` to each element that modifies the well
shape and shifts the minima. The bilinear coupling does not modify the
individual well shapes — it only creates an energetic preference for
alignment. This gives a cleaner Ising mapping.

### 3.2 Continuous-to-Ising mapping

Coarse-grain the continuous position to a binary spin: `σᵢ = +1` if
`xᵢ > x_thresh`, `σᵢ = −1` if `xᵢ < −x_thresh` (hysteresis-based,
inherited from Phase 3 §9.1). The effective Ising Hamiltonian is:

```
H_Ising = −J_eff · Σ_{⟨ij⟩} σᵢσⱼ
```

where `βJ_eff = J · ⟨x⟩²_well / kT ≈ J · x₀² / kT`.

At the central parameters (§7): `βJ_eff = 0.5 · 1.0² / 1.0 = 0.5`.

**Mapping corrections** (`O(kT/ΔV) ≈ O(1/3)`):

1. **Within-well anharmonic shift**: The mean position in a well is not
   exactly `±x₀`. The cubic anharmonicity of the quartic well gives
   `⟨x⟩ ≈ x₀ − V‴(x₀)·kT/(6·V″(x₀)²) ≈ 0.979·x₀`. This shifts `βJ_eff`
   from 0.500 to ~0.479, a 4% correction. `tanh(0.479) = 0.445` vs.
   `tanh(0.500) = 0.462` — a 3.7% deviation.
2. **Backreaction**: The coupling modifies the within-well distribution.
   Second-order in `J/V″(x₀) ≈ 0.5/24 ≈ 0.02` — negligible.
3. **Barrier-region exclusion**: Samples where any element is in the barrier
   region are excluded from the Ising comparison. This is a
   ~`(1 − 0.9^N) ≈ 34%` data reduction for N=4 but does not introduce bias
   (barrier visits are symmetric).

Combined expected mapping deviation from the naive Ising prediction: **~5%**.

### 3.3 Exact Ising solution for the 1D chain

For N spins on an open chain with nearest-neighbor coupling `βJ`, the
partition function (via transfer matrix) is:

```
Z = 2 · (2·cosh(βJ))^{N−1}
```

Pairwise correlations (exact for open NN chain):

```
⟨σᵢσⱼ⟩ = tanh(βJ)^|i−j|
```

At `βJ = 0.5`:

| Pair | Distance | ⟨σᵢσⱼ⟩ |
|------|----------|---------|
| (0,1), (1,2), (2,3) | 1 | `tanh(0.5) = 0.4621` |
| (0,2), (1,3) | 2 | `tanh(0.5)² = 0.2135` |
| (0,3) | 3 | `tanh(0.5)³ = 0.0987` |

The Ising model has `2^4 = 16` configurations. The most probable (`++++`
and `−−−−`) each have probability ~0.195. The least probable (`+−+−` and
`−+−+`) each have probability ~0.0097 — a 20:1 ratio.

**Verification**: The exact NN correlation `⟨σ₀σ₁⟩` has been verified
against explicit enumeration of all 16 configurations at `βJ = 0.5`:
numerator = 10.602, `Z = 22.944`, ratio = 0.4621 = `tanh(0.5)`. ✓
Similarly verified for distances 2 and 3.

### 3.4 Coupling effect on well geometry

The coupling tilts the individual double-well. For element i with
`n_aligned` aligned neighbors:

- **Well-minimum shift**:
  `δx ≈ J · x₀ · n / V″(x₀) = 0.5 · n / 24 ≈ 0.021 · n`.
  At most 4.2% of `x₀` for n=2.
- **Effective barrier**: `ΔV_eff = ΔV ± J · x₀ · n_neighbors`:
  - 1 aligned neighbor: `ΔV_eff = 3.5` (hardened)
  - 1 anti-aligned neighbor: `ΔV_eff = 2.5` (softened)
  - 2 aligned neighbors: `ΔV_eff = 4.0` (hardened)
  - 2 anti-aligned neighbors: `ΔV_eff = 2.0` (softened)

The barrier never disappears (min `ΔV_eff = 2.0 > 0`), and the well shape
remains qualitatively bistable. The coupling is in the perturbative regime:
`J · x₀ / (V″(x₀) · x₀) = 0.5 / 24 = 0.021 ≪ 1`.

### 3.5 Discretization and temperature bias

Identical to Phase 3 §3.5: `O(h·γ/M) = 1%` temperature bias from the
semi-implicit Euler scheme. The coupling is a deterministic force and does
not affect the FDT pairing. Euler stability at `h = 0.001`:
`h² · max|F′| / M ≈ 10⁻⁶ · 25 = 2.5 × 10⁻⁵ ≪ 1`.

---

## 4. Design Decisions

### 4.1 Composable PassiveComponents (not a monolithic array)

**Choice**: N independent `DoubleWellPotential` instances + 1
`PairwiseCoupling` instance + 1 `LangevinThermostat`, all stacked in a
`PassiveStack`.

**Why not a monolithic `CoupledDoubleWellArray`**:

- **Reuse**: `DoubleWellPotential` is already validated (Phase 3). A
  monolithic component would duplicate that logic.
- **Composability**: The `PassiveStack` pattern exists precisely for this —
  independent force contributions accumulate via `qfrc_out[i] +=`. A
  monolithic component fights the architecture.
- **Testability**: `PairwiseCoupling` can be unit-tested independently of
  the double-well (correct forces, correct energy) before integration
  testing.

**How it works in the stack**: Each `apply()` call accumulates into the
shared `qfrc_out` buffer. The 4 `DoubleWellPotential` instances each
contribute `F_well(xᵢ)` to their respective DOF. The `PairwiseCoupling`
contributes `+J·xⱼ` to each coupled DOF. The `LangevinThermostat`
contributes `−γ·vᵢ + noise` to all DOFs. The stack folds everything into
`qfrc_passive` once per step.

```rust
let mut builder = PassiveStack::builder();
for i in 0..N {
    builder = builder.with(DoubleWellPotential::new(DELTA_V, X_0, i));
}
builder = builder.with(PairwiseCoupling::chain(N, J));
builder = builder.with(LangevinThermostat::new(
    DVector::from_element(N, GAMMA), K_B_T, seed,
));
builder.build().install(&mut model);
```

### 4.2 1D chain topology

**Choice**: Open chain (nearest-neighbor, 3 edges for N=4).

**Why chain over ring**: The open chain has an exact closed-form solution
`⟨σᵢσⱼ⟩ = tanh(βJ)^|i−j|` that is simple, well-known, and does not
require edge-effect corrections. A ring adds one more bond (periodic
boundary), eliminates edge effects, but does not add validation power for
N=4.

**Why not fully connected**: `N(N−1)/2 = 6` bonds for N=4. The mean-field
Ising solution is less precise at small N, and the mapping corrections are
harder to analyze. The chain is the minimal topology that tests coupling;
fully connected is a future extension.

### 4.3 N=4 elements

**Choice**: 4 elements (16 configurations).

**Why 4 and not 8**: `2^4 = 16` configurations are manageable for exact
enumeration and statistical convergence. `2^8 = 256` configurations would
require ~16× more simulation time for the same per-configuration statistics.
N=4 is the minimal size that produces non-trivial multi-body correlations
(a chain of 3 has only NN and NNN correlations; 4 adds a three-bond
distance).

**Parameterization**: The `PairwiseCoupling` component and test
infrastructure accept arbitrary N and edge lists. N=8 is a trivial
extension for a future `#[ignore]` test.

### 4.4 Semi-implicit Euler integrator

Same reasoning as Phase 3 §4.2. All DOFs are slide joints with constant
scalar mass `M = 1`. The Phase 2 Euler finding (configuration-dependent
`M(q)` breaks equipartition) does not apply.

### 4.5 Pairwise correlations as the primary validation metric

**Choice**: Compare measured NN pairwise correlations `⟨σᵢσᵢ₊₁⟩` against
exact Ising predictions. Not a full 2^N joint-distribution comparison.

**Why correlations over joint distribution**: For the 1D NN chain, the
pairwise correlations `⟨σᵢσⱼ⟩ = tanh(βJ)^|i−j|` fully determine the
entire equilibrium distribution (the Ising model's sufficient statistics
are the pairwise products). Correlations converge much faster than
per-configuration frequencies: the rarest configuration (`+−+−`, probability
~0.97%) would need ~100× more data for meaningful frequency estimates than
the NN correlation (~46%) needs for its mean estimate.

**What Phase 3 anticipated vs. what Phase 4 does**: Phase 3 §12 point 4
states "the transition-counting measurement infrastructure exists for
Phase 4's joint-distribution tests." Phase 4 reuses the *state
classification* infrastructure (hysteresis threshold, `WellState` enum)
but not the transition-counting protocol. Phase 4 measures *correlations*
(time-averaged products of spin indicators), not *transition rates*.

---

## 5. MJCF Model

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

**Design choices** (identical to Phase 3 §5 where applicable):
- `stiffness="0"`, `damping="0"` — all potential and damping from
  `PassiveStack` components.
- `mass="1"` — unit mass per element.
- `gravity="0 0 0"` — no gravitational tilt.
- `contype="0" conaffinity="0"` — contacts explicitly disabled. This is the
  load-bearing fix: without it, overlapping geoms generate spurious contact
  forces that corrupt the Langevin dynamics, producing an asymmetric
  correlation gradient across the chain (**Implementation finding 1**,
  2026-04-10 — root cause isolated: `contype=0` alone fixes it; geometric
  separation alone also fixes it; both prevent the broadphase from
  generating contact pairs).
- Bodies offset in Y (`pos="0 N 0"`) so geoms never overlap geometrically,
  even without the contact flags. Slide joints act along X, so Y offset
  does not affect qpos or the coupling physics.
- Each body is a direct child of worldbody → independent DOFs → diagonal
  mass matrix `M = I₄`. Euler safe.
- `nv = 4`, `nq = 4`. DOF index = qpos index = joint index (slide joints,
  `nq = nv = 1` each).

### 5.1 Model invariants

```rust
#[test]
fn test_chain_model_invariants() {
    let model = sim_mjcf::load_model(CHAIN_XML).expect("load");
    assert_eq!(model.nv, N, "chain model must have {N} velocity DOFs");
    assert_eq!(model.nq, N, "slide joints: nq = nv = {N}");
    assert_eq!(model.timestep, 0.001);
    assert!(matches!(model.integrator, Integrator::Euler));
    for i in 0..N {
        assert_eq!(model.dof_damping[i], 0.0,
                   "thermostat owns damping on DOF {i}");
        assert_eq!(model.jnt_stiffness[i], 0.0,
                   "potential comes from cb_passive on joint {i}");
    }
}
```

---

## 6. `PairwiseCoupling` Component

### 6.1 API

```rust
/// Bilinear pairwise coupling: `V = −J · Σ_{(i,j) ∈ edges} xᵢ · xⱼ`.
///
/// Contributes force `F_i = +J · Σ_{j: (i,j) ∈ edges} xⱼ` to the per-DOF
/// force accumulator. Ferromagnetic for `J > 0` (aligned positions are
/// energetically favored). This is the natural continuous analogue of the
/// Ising model's `σᵢσⱼ` coupling.
///
/// Not stochastic — this is a deterministic conservative force.
///
/// # Joint type constraint
///
/// Uses DOF indices to access both `data.qpos` and `qfrc_out`. Same
/// restriction as `DoubleWellPotential`: only correct for slide and hinge
/// joints where `nq = nv = 1`.
pub struct PairwiseCoupling {
    coupling_j: f64,
    edges: Vec<(usize, usize)>,
}

impl PairwiseCoupling {
    /// Create a coupling with explicit edge list.
    ///
    /// # Panics
    /// Panics if any edge has `i == j` (self-coupling).
    pub fn new(coupling_j: f64, edges: Vec<(usize, usize)>) -> Self;

    /// Create a nearest-neighbor open chain:
    /// edges `[(0,1), (1,2), ..., (n−2, n−1)]`.
    ///
    /// # Panics
    /// Panics if `n < 2`.
    pub fn chain(n: usize, coupling_j: f64) -> Self;

    /// Create a nearest-neighbor ring: chain + closing edge `(n−1, 0)`.
    ///
    /// # Panics
    /// Panics if `n < 3`.
    pub fn ring(n: usize, coupling_j: f64) -> Self;

    /// Coupling constant `J`.
    pub fn coupling_j(&self) -> f64;

    /// Edge list (read-only).
    pub fn edges(&self) -> &[(usize, usize)];

    /// Total coupling energy: `V = −J · Σ xᵢ · xⱼ`.
    pub fn coupling_energy(&self, qpos: &DVector<f64>) -> f64;
}
```

### 6.2 `PassiveComponent` implementation

```rust
impl PassiveComponent for PairwiseCoupling {
    fn apply(&self, _model: &Model, data: &Data, qfrc_out: &mut DVector<f64>) {
        for &(i, j) in &self.edges {
            let xi = data.qpos[i];
            let xj = data.qpos[j];
            // V = −J · xi · xj  →  F_i = +J · xj,  F_j = +J · xi
            qfrc_out[i] += self.coupling_j * xj;
            qfrc_out[j] += self.coupling_j * xi;
        }
    }
}

impl Diagnose for PairwiseCoupling {
    fn diagnostic_summary(&self) -> String {
        format!("PairwiseCoupling(J={:.4}, edges={})",
                self.coupling_j, self.edges.len())
    }
}
```

### 6.3 Factory methods

```rust
pub fn chain(n: usize, coupling_j: f64) -> Self {
    assert!(n >= 2, "chain requires at least 2 elements, got {n}");
    let edges = (0..n - 1).map(|i| (i, i + 1)).collect();
    Self::new(coupling_j, edges)
}

pub fn ring(n: usize, coupling_j: f64) -> Self {
    assert!(n >= 3, "ring requires at least 3 elements, got {n}");
    let mut edges: Vec<(usize, usize)> = (0..n - 1).map(|i| (i, i + 1)).collect();
    edges.push((n - 1, 0));
    Self::new(coupling_j, edges)
}
```

### 6.4 `coupling_energy`

```rust
pub fn coupling_energy(&self, qpos: &DVector<f64>) -> f64 {
    self.edges.iter()
        .map(|&(i, j)| -self.coupling_j * qpos[i] * qpos[j])
        .sum()
}
```

### 6.5 Module location

`sim-thermostat/src/pairwise_coupling.rs`, exported via `lib.rs`.

### 6.6 Unit tests (in-module)

The module should include unit tests for:
1. `new` rejects self-coupling edges `(i, i)`.
2. `chain(4, J)` produces edges `[(0,1), (1,2), (2,3)]`.
3. `ring(4, J)` produces edges `[(0,1), (1,2), (2,3), (3,0)]`.
4. `coupling_energy` at `x = [+1, +1, +1, +1]` equals `−3J` for a 4-chain.
5. `coupling_energy` at `x = [+1, −1, +1, −1]` equals `+3J` for a 4-chain.
6. Force direction: for `J > 0`, coupling to a positive neighbor produces a
   positive force (ferromagnetic pull toward alignment).
7. `diagnostic_summary` contains coupling constant and edge count.

---

## 7. Central Parameter Set

### 7.1 Parameters

| Parameter | Symbol | Value | Rationale |
|---|---|---|---|
| Number of elements | N | 4 | Minimal non-trivial array (§4.3) |
| Well half-separation | x₀ | 1.0 | Same as Phase 3 |
| Barrier height | ΔV | 3.0 | Same as Phase 3 |
| Mass | M | 1.0 | Unit mass per element |
| Friction | γ | 10.0 | Same as Phase 3 (moderate-to-overdamped) |
| Temperature | kT | 1.0 | Same as Phase 3 |
| Coupling constant | J | 0.5 | Moderate coupling: `βJ_eff = 0.5` (§7.2) |
| Timestep | h | 0.001 | Same as Phase 3 |
| Topology | — | chain | Edges: `{(0,1), (1,2), (2,3)}` |

### 7.2 Derived quantities

| Quantity | Formula | Value |
|---|---|---|
| Effective Ising coupling | `βJ_eff = J·x₀²/kT` | 0.500 |
| NN correlation (Ising) | `tanh(βJ_eff)` | 0.4621 |
| NNN correlation | `tanh(βJ_eff)²` | 0.2135 |
| 3rd-neighbor correlation | `tanh(βJ_eff)³` | 0.0987 |
| Partition function | `2·(2cosh(βJ))³` | 22.94 |
| Most probable config | `p(++++) = p(−−−−)` | 0.1953 each |
| Least probable config | `p(+−+−) = p(−+−+)` | 0.0097 each |
| Max effective barrier | `ΔV + 2J·x₀` | 4.0 |
| Min effective barrier | `ΔV − 2J·x₀` | 2.0 |
| Uncoupled Kramers rate | `k₀` (Phase 3) | 0.01214 per time unit |
| Well-minimum shift per neighbor | `J·x₀/V″(x₀)` | 0.021 (2.1%) |
| Mapping correction to `βJ_eff` | from anharmonic `⟨x⟩` shift | ~4% |
| Discretization bias on kT | `O(h·γ/M)` | 1% |
| Euler stability | `h²·max\|F′\|/M` | 2.5×10⁻⁵ |

### 7.3 Ergodicity check

**Slowest process**: Flip of an interior element with 2 aligned neighbors.
`ΔV_eff = 4.0`. Rate ≈ `C·exp(−4.0) = 0.2438 · 0.0183 = 0.00446` per
time unit. Mean dwell: ~224 time units.

**Fastest process**: Flip of an element with anti-aligned neighbor(s).
`ΔV_eff = 2.0`. Rate ≈ 0.033 per time unit. Mean dwell: ~30 time units.

**Exit rate from most stable config** (`++++`): 2 edge flips
(`ΔV_eff = 3.5`) + 2 interior flips (`ΔV_eff = 4.0`) = 0.0237 per time
unit. Mean dwell: ~42 time units.

**Mixing time** (conservative): ~`4 × 224 ≈ 900` time units. Over 10,000
time units per trajectory: ~11 mixing times. Adequate ergodicity.

---

## 8. Validation Tests

### 8.1 Gate A — Pairwise NN correlations at central parameters (must-pass)

**Setup**: 20 independent trajectories (distinct seeds), each 10M steps
(10,000 time units). Per trajectory: set all `data.qpos[i] = x₀` +
`data.forward(&model)`, 20,000-step burn-in (20 time units — slightly
longer than Phase 3's 10-time-unit burn-in to account for coupling
equilibration), then measure over 9,980,000 steps.

**Measurement**: At each step, classify each element into `{Left, Right,
Barrier}` using the Phase 3 hysteresis threshold `x_thresh = x₀/2`. For
each nearest-neighbor pair `(i, i+1)`: if both elements are in a well
(not Barrier), record `σᵢ · σᵢ₊₁` where `σ = +1` for Right and `σ = −1`
for Left. Compute the per-trajectory time-average `⟨σᵢσᵢ₊₁⟩` for each
pair, then compute the ensemble mean and SEM across 20 trajectories (same
two-level Welford pattern as Phase 1/2). Average across the 3 NN pairs
(by chain symmetry, they should be equal within statistical noise).

**Expected**: `⟨σᵢσᵢ₊₁⟩_Ising = tanh(βJ_eff) = 0.4621`.

**Statistical error**: Autocorrelation time of `σᵢσⱼ` ≈ 50K steps
(single-flip time at moderate coupling; both elements must flip or
their product must change). Usable samples per trajectory ≈ `10M × 0.81`
(both in wells) = 8.1M. `N_eff` per trajectory ≈ `8.1M / 100K = 81`.
Across 20 trajectories: `N_eff ≈ 1620`. SE ≈ `√(0.79/1620) ≈ 0.022`.
Relative SE ≈ 4.8%.

**Tolerance**: `|⟨σσ⟩_measured − tanh(βJ)| / tanh(βJ) < 0.25` (25%).
Budget: ~5% mapping correction (§3.2) + ~5% statistical error = ~10%
expected deviation. 25% tolerance gives ~3σ margin.

**Equipartition sanity check**: Track `⟨½Mv²⟩` per DOF using Welford.
Assert each equals `½kT` within ±5%. Failure indicates a thermostat bug,
not a coupling issue.

**Cost**: 20 × 10M = 200M steps.

### 8.2 Gate B — Coupling-strength sweep (must-pass, `#[ignore]`)

**Setup**: Repeat Gate A's protocol at three coupling strengths:

| J | βJ_eff | Expected `⟨σσ⟩_NN` | Min `ΔV_eff` |
|---|--------|---------------------|--------------|
| 0.25 | 0.25 | `tanh(0.25) = 0.2449` | 2.5 |
| 0.50 | 0.50 | `tanh(0.50) = 0.4621` | 2.0 |
| 0.75 | 0.75 | `tanh(0.75) = 0.6351` | 1.5 |

At `J = 0.75`, the minimum effective barrier is `ΔV − 2J·x₀ = 1.5`. This
is still in the activated regime (`ΔV_eff/kT = 1.5 > 1`), and the Kramers
framework remains qualitatively valid. The mapping correction increases
slightly (~6-8% at `J = 0.75`) but stays within the 25% tolerance.

**Measurement**: For each `J`, compute the mean NN correlation across pairs
and trajectories.

**Test**:
1. Each correlation matches the Ising prediction within ±25%.
2. Monotonicity: `⟨σσ⟩(J=0.25) < ⟨σσ⟩(J=0.50) < ⟨σσ⟩(J=0.75)` (strict).

**Cost**: 3 × 20 × 10M = 600M steps. Mark `#[ignore]` — run with
`cargo test -p sim-thermostat -- --ignored`.

### 8.3 Supporting — Marginal symmetry

**Setup**: Same data as Gate A (20 trajectories, central parameters).

**Measurement**: For each element i, compute `P(σᵢ = +1)` = fraction of
usable steps (element i in a well, not Barrier) where element i is in the
Right well.

**Test**: Each `|P(σᵢ = +1) − 0.5| < 0.05` (5% absolute tolerance). The
symmetric quartic + symmetric chain has zero effective external field — any
marginal asymmetry beyond statistical noise is a bug.

**Cost**: No additional simulation (reuses Gate A data).

### 8.4 Supporting — NNN and long-range correlations

**Setup**: Same data as Gate A.

**Measurement**: Compute `⟨σᵢσⱼ⟩` for distances `|i−j| = 2` and
`|i−j| = 3` using the same estimation procedure as Gate A but for
non-nearest pairs.

**Expected**: `tanh(0.5)² = 0.2135` (distance 2),
`tanh(0.5)³ = 0.0987` (distance 3).

**Tolerance**: Distance 2: `|measured − expected| / expected < 0.30` (30%
— looser than Gate A because the signal is weaker). Distance 3:
`|measured − expected| < 0.08` (absolute tolerance — the expected value
0.099 is small enough that relative tolerance is unreliable at moderate
`N_eff`).

**Cost**: No additional simulation.

### 8.5 Supporting — Configuration frequency ratio

**Setup**: Same data as Gate A. At each valid step (all N elements in
wells), record the full joint configuration as a bitmask (bit i = 1 if
`σᵢ = +1`, 0 if `σᵢ = −1`). Accumulate a histogram over all 2^N = 16
configurations across all trajectories.

**Measurement**: Compute the ratio of the most frequent to least frequent
configuration. At βJ = 0.5, the Ising prediction is
`p(++++)/p(+−+−) = exp(6βJ) = exp(3) ≈ 20.1`.

**Test**: The measured ratio is within a factor of 2 of the expected ratio:
`10 < ratio < 40` (i.e., ±100% on a log scale). This is deliberately
generous — the rare configurations (`+−+−`, `−+−+`) have probability
~0.97% each, so even at 200M steps with ~66% valid-step yield, the
expected raw count per rare configuration is ~1.3M samples but only
~13 effective independent observations (given the ~100K autocorrelation
time of the joint state). The ratio's sampling noise is dominated by the
rare-configuration count.

**Why this test exists**: The correlation tests (Gates A/B) are sufficient
statistics for the NN chain Ising model, but they test *marginal* pair
properties. This test verifies that the *joint* distribution has the right
qualitative structure — the coupling creates a 20:1 preference for
fully-aligned over fully-alternating configurations. A bug that preserves
pairwise correlations but breaks the joint distribution is theoretically
impossible for the Ising model, but this test catches implementation errors
that the correlation test might miss (e.g., force double-counting on shared
edges).

**Note on the full 2^N joint distribution**: A precise per-configuration
frequency comparison (total variation distance or KL divergence against
Ising) is deferred to Phase 6, where a CPU Gibbs sampler on the same
continuous energy function provides an exact discrete reference —
eliminating the continuous→discrete mapping error that would otherwise
dominate the tolerance budget. See §11.5.

**Cost**: No additional simulation (reuses Gate A data).

### 8.6 Supporting — Reproducibility

**Setup**: Two trajectories with the same seed, same model, same initial
conditions.

**Test**: Bit-for-bit identical positions and velocities at every step.
Inherits Phase 1's `ChaCha8Rng` determinism guarantee. Confirms that the
`PairwiseCoupling` force computation is deterministic (no hidden state, no
floating-point order dependence across runs).

**Cost**: 2 × 100K = 200K steps (short trajectories suffice).

---

## 9. Measurement Protocol

### 9.1 Joint state classification

**Extension of Phase 3's hysteresis**: Each element is independently
classified as Left, Right, or Barrier using the same threshold
`x_thresh = x₀/2`.

**`WellState` promotion to `test_utils`**: Phase 3 defined `WellState` and
`WellState::from_position()` locally inside `kramers_escape_rate.rs`
(lines 76-94). Phase 4 needs the same classification for N elements, and
Phases 5-7 will too. Rather than copying ~15 lines into each test file,
Phase 4 promotes `WellState` to `sim_thermostat::test_utils` alongside
`WelfordOnline` and `assert_within_n_sigma` — the same "shared test
infrastructure" pattern that chassis Decision 6 established. The
Phase 3 test file is then updated to import from `test_utils` instead of
defining locally.

```rust
// In sim_thermostat::test_utils:

/// Hysteresis-based well-state classification for bistable elements.
///
/// Used by Phase 3+ tests to classify a 1-DOF position into one of three
/// regions: left well, right well, or barrier. The threshold `x_thresh`
/// defines the boundary between well and barrier regions.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WellState {
    Left,    // x < -x_thresh
    Right,   // x > +x_thresh
    Barrier, // -x_thresh <= x <= +x_thresh
}

impl WellState {
    /// Classify a position into a well state.
    pub fn from_position(x: f64, x_thresh: f64) -> Self {
        if x > x_thresh {
            Self::Right
        } else if x < -x_thresh {
            Self::Left
        } else {
            Self::Barrier
        }
    }

    /// Convert to a spin value: +1 for Right, -1 for Left.
    /// Panics if called on Barrier.
    pub fn spin(self) -> f64 {
        match self {
            Self::Right => 1.0,
            Self::Left => -1.0,
            Self::Barrier => panic!("spin() called on Barrier state"),
        }
    }

    /// Returns true if the element is in a well (not in the barrier).
    pub fn is_in_well(self) -> bool {
        self != Self::Barrier
    }
}
```

**Note**: The Phase 3 `from_position` took a single argument (using a
module-level `X_THRESH` constant). The promoted version takes `x_thresh`
as an explicit parameter — each test defines its own threshold, and the
promoted helper doesn't couple to any specific parameter set. The Phase 3
test is updated to pass `X_THRESH` at each call site.

**Valid step**: A step where *all* N elements are in a well (not Barrier).
Only valid steps are used for pairwise correlation estimates. At the central
parameters, ~`(0.9)^4 ≈ 66%` of steps are valid — a 34% data reduction
that does not introduce bias.

**Per-pair valid step**: For measuring `⟨σᵢσⱼ⟩`, it suffices for elements
i and j to both be in wells (the other elements can be in Barrier). This
gives ~81% usable steps per pair. Gate A uses the per-pair criterion for
correlation estimation.

**Spin mapping**: `σ = +1` for Right, `σ = −1` for Left (via
`WellState::spin()`).

**Difference from Phase 3's transition counting**: Phase 3 uses a state
machine that tracks committed transitions (well-to-well switches) for rate
measurement. Phase 4 does not count transitions — it samples the joint
state at each step for correlation accumulation. The hysteresis threshold
is reused for state classification, but the measurement protocol is
different: Phase 3 measures a *rate* (Poisson statistics), Phase 4 measures
a *mean correlation* (Gaussian CLT).

### 9.2 Correlation estimation

For each pair (i, j), accumulate `σᵢ · σⱼ` using `WelfordOnline` (per
trajectory). The per-trajectory mean is the estimated correlation for that
trajectory.

Across 20 trajectories: compute ensemble mean and SEM from the 20
per-trajectory estimates (same two-level pattern as Phase 1/2). Each
trajectory is seeded independently, eliminating cross-trajectory
correlations — the SEM from the ensemble level is valid.

### 9.3 Ising reference computation (in test)

The exact Ising correlation for an open NN chain is a closed-form
expression:

```rust
fn ising_nn_correlation(beta_j: f64) -> f64 {
    beta_j.tanh()
}

fn ising_correlation_at_distance(beta_j: f64, distance: usize) -> f64 {
    beta_j.tanh().powi(distance as i32)
}
```

For future topologies (ring, fully connected), the test file includes a
brute-force enumeration helper:

```rust
/// Exact Ising distribution by enumeration over 2^N configurations.
/// Returns Vec of (config_as_bitmask, probability).
fn exact_ising_distribution(
    n: usize,
    edges: &[(usize, usize)],
    beta_j: f64,
) -> Vec<(u32, f64)> {
    let n_configs = 1u32 << n;
    let mut weights = Vec::with_capacity(n_configs as usize);
    let mut z = 0.0f64;

    for c in 0..n_configs {
        let s: f64 = edges.iter().map(|&(i, j)| {
            let si = if c & (1 << i) != 0 { 1.0 } else { -1.0 };
            let sj = if c & (1 << j) != 0 { 1.0 } else { -1.0 };
            si * sj
        }).sum();
        let w = (beta_j * s).exp();
        z += w;
        weights.push((c, w));
    }

    weights.iter_mut().for_each(|(_, w)| *w /= z);
    weights
}
```

This is ~20 LOC and handles arbitrary topologies. Only used for
verification during spec development and for potential future ring/complete
topology tests.

### 9.4 Initial condition and burn-in

All elements start in the right well:

```rust
for i in 0..N {
    data.qpos[i] = X_0;
    data.qvel[i] = 0.0;
}
data.forward(&model).expect("forward");
```

The 20,000-step burn-in (20 time units = 200 intra-well relaxation times)
is sufficient for kinetic thermalization. The initial all-aligned
configuration introduces a transient correlation bias, but this is diluted
to insignificance over the 9,980 time-unit measurement window (~11 mixing
times).

---

## 10. Acceptance Criteria

| Test | Criterion | Tolerance |
|---|---|---|
| Gate A — NN correlations | `\|⟨σσ⟩ − tanh(βJ)\| / tanh(βJ)` | < 25% |
| Gate A — KE sanity (per DOF) | `\|⟨½Mv²⟩ − ½kT\| / (½kT)` | < 5% |
| Gate B — per-J correlation | Same as Gate A, at each J | < 25% |
| Gate B — monotonicity | `⟨σσ⟩(J₁) < ⟨σσ⟩(J₂) < ⟨σσ⟩(J₃)` | strict |
| Marginal symmetry | `\|P(σᵢ=+1) − 0.5\|` | < 0.05 |
| NNN correlation | `\|⟨σσ⟩ − tanh²(βJ)\| / tanh²(βJ)` | < 30% |
| Long-range correlation | `\|⟨σσ⟩ − tanh³(βJ)\|` | < 0.08 |
| Config frequency ratio | `10 < p_max/p_min < 40` | ±100% log scale |
| Reproducibility | Bit-exact positions/velocities | exact |

**Gates A + B must pass for Phase 4 to be green.** Supporting tests
provide additional confidence but are not blocking.

---

## 11. Implications for Future Phases

### 11.1 The Euler finding remains sidestepped

Same as Phase 3 §11: all DOFs are slide joints with constant M. If future
phases couple bistable elements via hinge joints in multi-link chains, the
Phase 2 Euler limitation will apply.

### 11.2 The `PairwiseCoupling` component is topology-agnostic

The edge-list API supports arbitrary coupling graphs. Phase 4 validates
only the 1D chain, but ring topology (D2: stochastic resonance) and fully
connected graphs (D5: Brownian computer) are one factory-method call away.
The Ising comparison code must be extended for non-chain topologies
(enumeration helper in §9.3, or transfer-matrix method).

### 11.3 The bilinear coupling maps to Ising, not general EBMs

Phase 5 (differentiable EBM) requires coupling functions that can represent
arbitrary energy surfaces, not just pairwise bilinear. The
`PairwiseCoupling` component is sufficient for Phase 4's Ising validation
but will need generalization (or replacement by a learned coupling
function) for Phase 5.

### 11.4 Full joint-distribution comparison deferred to Phase 6

Phase 4 validates the coupled array via pairwise correlations and a
qualitative configuration ratio test (§8.5). The full 2^N
joint-distribution comparison — total variation distance or KL divergence
against an exact reference, with per-configuration frequency matching — is
**explicitly deferred to Phase 6**.

**Why Phase 6**: Phase 6 introduces a CPU Gibbs sampler that draws from
the same continuous energy function `E(x) = Σ V(xᵢ) − J Σ xᵢxⱼ`. This
sampler produces exact (up to mixing) reference probabilities for all 2^N
discrete configurations, *without* the continuous→discrete mapping error
that plagues the Phase 4 Ising comparison at ~5%. Comparing two samplers
on the same energy function isolates the question "does the Langevin
thermostat sample correctly?" from "does the Ising mapping hold?" — a
cleaner test with tighter achievable tolerances.

**What Phase 6 should test**: TV distance between the Langevin-measured
configuration histogram and the Gibbs-sampled reference, with tolerance
set by the statistical convergence of both samplers (no systematic mapping
error to budget for).

### 11.5 Q1 constraint-projection remains deferred

Phase 4's 4 independent slide-joint bodies have no active constraints. The
Q1 constraint-projection question (noise + contacts/equality constraints)
remains untested. The first natural test point is D2, where cf-design
buckling beams may have contact surfaces.

---

## 12. What This Enables

Passing Phase 4 proves:

1. The `PassiveComponent` composability pattern extends to **coupled
   multi-DOF systems** — independent wells + pairwise interactions +
   thermostat, all composed without code changes to existing components.
2. The Langevin thermostat produces correct **correlated equilibrium
   statistics** on coupled systems, not just single-DOF dynamics.
3. The continuous-to-Ising mapping is quantitatively validated — the
   mechanical system's joint state distribution matches the analytical
   statistical-mechanics model.
4. The `PairwiseCoupling` component provides a **topology-agnostic**
   coupling primitive for future phases.

This unlocks:
- **Phase 5** (differentiable EBM) — builds on the coupled array as the
  substrate for energy-based learning.
- **D2** (stochastic resonance) — requires coupled elements; Phase 4
  validates the coupling infrastructure.
- **D4** (sim-to-real on printed device) — the coupled array is the
  simplest printable thermodynamic computing device.
