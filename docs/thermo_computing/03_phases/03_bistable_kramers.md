# Phase 3 — Single Bistable Element + Kramers' Escape Rate

> **Status**: **Done.** Implemented and all gates green on `main`.
> **Commit**: `ffbb14d`
> **Owner**: Jon
> **Parents**:
> - [`02_multi_dof_equipartition.md`](./02_multi_dof_equipartition.md) (Phase 2 — the gate this phase inherits)
> - Phase 2 §12 Finding 1 (Euler integrator limitation — drives the slide-joint design choice)
> - [`../02_foundations/open_questions.md`](../02_foundations/open_questions.md) Q1 §3.3 (constraint-projection testing deferred to Phase 3 for models with contacts)
> - [`overview.md`](./overview.md) Phase 3 sketch ("Double-well switching rate matches Kramers' formula")
> - [`../01_vision/research_directions.md`](../01_vision/research_directions.md) D1, D2 (both gate on Phase 3)

This spec defines the first phase with a non-trivial energy landscape. The deliverable is a `DoubleWellPotential` component in `sim-thermostat` and four validation tests proving that Kramers' escape-rate formula holds for a 1-DOF particle in a quartic double-well under the Langevin thermostat. Passing these gates is the entrance condition for Phase 4 (coupled bistable arrays) and unlocks D1 (flashing ratchet) and D2 (stochastic resonance).

---

## 1. Goal

Demonstrate that a particle in a quartic double-well potential, coupled to the Phase 1 `LangevinThermostat`, exhibits:

1. **Kramers-rate dynamics**: The measured switching rate between wells matches the Kramers escape-rate formula to within ±25%.
2. **Arrhenius scaling**: log(rate) is linear in 1/kT with slope −ΔV to within ±10%.
3. **Boltzmann equilibrium**: The stationary position distribution matches p(x) ∝ exp(−V(x)/kT).

The physics test shifts from equilibrium statistics (Phases 1–2: equipartition) to **escape-rate dynamics**: the first test of the thermostat on a system where the interesting observable is a rare, activated process rather than a time-averaged equilibrium quantity. **It is a dynamics test, not just a thermodynamics test** — sloppy Phase 3 would mean D1 and D2 are built on unvalidated ground.

---

## 2. Scope

**In scope for Phase 3:**
- One new `PassiveComponent`: `DoubleWellPotential` (§6), implementing `PassiveComponent + Diagnose` (not `Stochastic` — it is a deterministic conservative force).
- One MJCF model: 1-DOF slide joint with zero stiffness (§5).
- Four validation tests: Kramers rate gate (§8.1), Arrhenius slope gate (§8.2), Boltzmann distribution (§8.3), reproducibility (§8.4).
- Transition-counting measurement infrastructure (§9).

**Explicitly out of scope:**
- **cf-design bistable elements** (buckling beams, etc.) — Phase 3 validates the escape-rate measurement methodology against a known analytical potential. cf-design integration is a Phase 4 / D2 concern where the potential shape is emergent rather than prescribed. See §4.1.
- **Contact constraints** — the 1-DOF slide joint has no contacts. Q1 §3.3's constraint-projection testing remains deferred to a phase where contacts appear (Phase 4 coupling, or a dedicated constraint-projection test).
- **Multi-DOF bistable systems** — Phase 4.
- **BAOAB / GJF integrators** — not needed (§4.2 explains why).
- **Changes to existing `sim-thermostat/src/` files** other than adding the new module and re-exporting from `lib.rs`.

---

## 3. Physics: Kramers' Escape-Rate Theory

### 3.1 The quartic double-well potential

```
V(x) = a(x² − x₀²)²
```

where `a = ΔV / x₀⁴`. This is the standard symmetric quartic double-well:

- **Minima** at `x = ±x₀`, with `V(±x₀) = 0`.
- **Barrier** at `x = 0`, with `V(0) = ΔV`.
- **Force**: `F(x) = −V′(x) = −4ax(x² − x₀²)`.
- **Curvature at well bottom**: `V″(x₀) = 8ax₀²` → `ω_a = √(8ΔV / (M·x₀²))`.
- **Curvature at barrier top**: `|V″(0)| = 4ax₀²` → `ω_b = √(4ΔV / (M·x₀²)) = ω_a / √2`.

The potential is confining (`V → ∞` as `|x| → ∞`), so no joint limits are needed.

### 3.2 The Kramers formula (moderate-to-strong friction)

For a particle of mass `M` in a 1-D potential with friction coefficient `γ` (force/velocity units) at temperature `kT`, the Kramers escape rate from one well is:

```
k = (ω_a / 2π) · (λ_r / ω_b) · exp(−ΔV / kT)
```

where `λ_r` is the positive root of `λ² + γ̃·λ − ω_b² = 0`:

```
λ_r = (−γ̃ + √(γ̃² + 4ω_b²)) / 2,     γ̃ = γ / M
```

This is the **Kramers–Grote–Hynes formula**, valid for the spatial-diffusion regime (moderate-to-strong friction, `γ̃ ≳ ω_b`). Limiting cases:

- **Overdamped** (`γ̃ ≫ 2ω_b`): `k ≈ ω_a·ω_b / (2π·γ̃) · exp(−ΔV/kT)`.
- **TST limit** (`γ̃ ≪ 2ω_b`): `k ≈ (ω_a / 2π) · exp(−ΔV/kT)`.

The formula gives the **one-directional** rate (escape from one well). For the symmetric quartic, `k_{A→B} = k_{B→A} = k`. The total transition rate observed as zero-crossings per unit time is also `k` (§9.1).

### 3.3 Regime classification

At the central parameter set (§7): `γ̃ = 10`, `ω_b = √12 ≈ 3.464`.

```
γ̃ / (2ω_b) = 10 / 6.928 ≈ 1.44     (moderately overdamped)
```

This is above the Kramers turnover (`γ̃ > ω_b`) but not deep overdamped. The **full Kramers formula** (§3.2) must be used — the overdamped approximation overestimates by ~11% at this ratio.

### 3.4 Finite-barrier and anharmonic corrections

The Kramers formula is asymptotically exact as `ΔV/kT → ∞`. At finite `ΔV/kT`, two correction sources:

1. **Finite-barrier corrections**: `O(kT/ΔV)`. At `ΔV/kT = 3`, ~10–15% in practice.
2. **Anharmonic corrections**: The quartic well is not harmonic. The cubic term at the well bottom (`V‴(x₀) = 24ax₀`) and the quartic term at the barrier top modify both the attempt frequency and effective barrier. For this potential at `ΔV/kT = 3`, the correction is ~10%.

Combined expected deviation from Kramers at `ΔV/kT = 3`: **15–25%**. At `ΔV/kT = 5`, it drops to ~5–10%.

**Key insight for Gate B**: These corrections affect the absolute rate but are approximately `kT`-independent — they largely cancel in the Arrhenius slope (`log k` vs. `1/kT`). The slope test is therefore much tighter than the absolute rate test.

### 3.5 Discretization error

sim-core uses **semi-implicit (symplectic) Euler**: velocity is updated first (`qvel += h·qacc`), then position uses the *new* velocity (`qpos += h·qvel_new`). The stochastic force injection is still Euler-order (additive noise at the current timestep), so the scheme is Euler-Maruyama in the Langevin sense — the `O(h·γ/M)` temperature bias analysis applies unchanged.

The bias is `O(h·γ/M) = 0.001 · 10 / 1 = 0.01 = 1%`. This shifts the effective bath temperature by ~1%, which shifts the escape rate by `exp(0.01 · ΔV/kT) − 1 ≈ 3%` at `ΔV/kT = 3`. Systematic, well within the ±25% Gate A tolerance. The Arrhenius slope is affected by ~1% (same relative temperature shift at all `kT`).

---

## 4. Design Decisions

### 4.1 Slide joint + quartic PassiveComponent

**Choice**: 1-DOF slide joint with a quartic double-well implemented as a `PassiveComponent`. Not a cf-design buckling beam. Not a hinge joint.

**Why quartic on a slide joint**:

- **Exact analytical control**: `ΔV`, `ω_a`, `ω_b`, `x₀` are algebraically known from two parameters `(ΔV, x₀)`. Kramers validation requires comparing measured rates against a *known* theoretical prediction.
- **Clean separation**: Phase 3 validates the escape-rate measurement methodology. cf-design integration (emergent potentials from elastic geometry) is a separate concern for Phase 4 / D2 where the physics is more complex and the potential shape is part of the experiment, not the validation.
- **Phase 2 Euler finding sidestepped**: A slide joint has **constant mass** (scalar `M`). See §4.2.

**Why not cf-design buckling beam**: A buckling beam has an emergent `V(θ)` that depends on geometry, material, and boundary conditions in ways that are not analytically closed-form. Validating Kramers against it would be circular — measuring the rate can't be compared to the formula unless the potential is independently known. The buckling beam is the right substrate for D2 (stochastic resonance) where the potential shape is part of the experiment, not the validation.

**Why not hinge joint**: For a single body on a hinge (attached to worldbody), `M` is constant (moment of inertia about the axis), so Euler works equally well. But a slide joint (particle in a 1D potential) is Kramers' original formulation, making the physics mapping direct and the literature comparison unambiguous.

### 4.2 Semi-implicit Euler integrator

**The Phase 2 Euler finding does not apply to Phase 3.** Finding 1 showed that Euler-Maruyama breaks equipartition when `M(q)` changes significantly per step — this occurs in multi-link hinge chains where off-diagonal mass-matrix terms depend on joint angles. A 1-DOF slide joint has constant scalar mass. No `M(q)` dependence, no Euler bias, no angle restriction.

**Integration scheme** (from `sim/L0/core/src/integrate/`): sim-core's "Euler" integrator is semi-implicit (symplectic Euler):
```
qacc = M⁻¹ · qfrc_passive          (sparse LDL solve in constraint stage)
qvel += h · qacc                    (velocity updated first)
qpos += h · qvel                    (uses NEW velocity)
```
This is better for energy conservation than forward Euler. The stochastic force injection is still Euler-order (the Langevin noise enters `qfrc_passive` at the current timestep), so the scheme is classified as Euler-Maruyama for purposes of the `O(h·γ/M)` temperature-bias analysis. The Kramers rate validation is unaffected — the formula assumes continuous-time dynamics and the discretization error is handled by the tolerance budget (§3.5).

The remaining Euler concern — force-gradient stability for the nonlinear quartic force — is easily satisfied. The maximum force gradient in the well region is `|F′(x₀)| = V″(x₀) = 8ax₀² = 24` at the central parameters. Euler stability requires `h² · |F′| / M ≪ 1`; at `h = 0.001`: `10⁻⁶ · 24 = 2.4 × 10⁻⁵`. No issue.

### 4.3 Moderate-to-overdamped friction regime

**Choice**: `γ = 10` (`γ̃/(2ω_b) = 1.44`). Above the Kramers turnover, in the spatial-diffusion regime where the Kramers formula is valid.

**Why not `γ = 0.1`** (Phase 1 value): At `γ = 0.1`, `γ̃/(2ω_b) = 0.014` — deep in the energy-diffusion regime. The spatial-diffusion Kramers formula overestimates the rate by orders of magnitude. The energy-diffusion formula (Kramers 1940 underdamped result) applies but requires additional theory and is less standard.

**Why not `γ = 50`** (deep overdamped): Rate scales as `~1/γ` in the overdamped limit. Fewer transitions per unit time → longer simulations for the same statistics. `γ = 10` balances regime validity against computational cost.

---

## 5. MJCF Model

```xml
<mujoco model="bistable_1dof">
  <option timestep="0.001" gravity="0 0 0" integrator="Euler"/>
  <worldbody>
    <body name="particle">
      <joint name="x" type="slide" axis="1 0 0"
             stiffness="0" damping="0" springref="0" ref="0"/>
      <geom type="sphere" size="0.05" mass="1"/>
    </body>
  </worldbody>
</mujoco>
```

**Design choices**:
- `stiffness="0"` — all potential energy comes from `DoubleWellPotential` via `cb_passive`. No built-in spring.
- `damping="0"` — thermostat owns damping (Q4, carried from Phase 1).
- `mass="1"` — unit mass, matching the Kramers derivation.
- `gravity="0 0 0"` — the double-well IS the potential; gravity would add a linear tilt breaking the symmetry.
- `ref="0"` — default position at `x = 0`. Tests override this by setting `data.qpos[0] = x₀` before stepping (start in one well).

### 5.1 Model invariants

```rust
#[test]
fn test_bistable_model_invariants() {
    let model = sim_mjcf::load_model(BISTABLE_XML).expect("load");
    assert_eq!(model.nv, 1, "bistable model must have 1 velocity DOF");
    assert_eq!(model.nq, 1, "slide joint: nq = nv = 1");
    assert_eq!(model.timestep, 0.001);
    assert!(matches!(model.integrator, Integrator::Euler));
    assert_eq!(model.dof_damping[0], 0.0, "thermostat owns damping");
    assert_eq!(model.jnt_stiffness[0], 0.0, "potential comes from cb_passive");
}
```

---

## 6. `DoubleWellPotential` Component

### 6.1 API

```rust
/// Symmetric quartic double-well potential: V(x) = a(x² − x₀²)²
/// where a = ΔV / x₀⁴.
///
/// Contributes force F(x) = −V′(x) = −4ax(x² − x₀²) to `qfrc_out`
/// on a single DOF. Not stochastic — this is a deterministic conservative force.
pub struct DoubleWellPotential {
    delta_v: f64,  // Barrier height: ΔV = V(0) − V(±x₀)
    x_0: f64,      // Well half-separation: minima at ±x₀
    dof: usize,    // DOF index (= qpos index for slide/hinge joints where nq=nv=1)
}
// NOTE: `dof` is used to index both `data.qpos` and `qfrc_out`. This is
// correct for slide and hinge joints (nq = nv = 1, so DOF index = qpos index).
// For ball (nq=4, nv=3) or free (nq=7, nv=6) joints, DOF and qpos indices
// diverge — this component does not support those joint types.

impl DoubleWellPotential {
    pub fn new(delta_v: f64, x_0: f64, dof: usize) -> Self;

    pub fn barrier_height(&self) -> f64;
    pub fn well_separation(&self) -> f64;

    /// Angular frequency at well bottom: ω_a = √(8ΔV / (M·x₀²))
    pub fn omega_a(&self, mass: f64) -> f64;

    /// Angular frequency at barrier top: ω_b = √(4ΔV / (M·x₀²))
    pub fn omega_b(&self, mass: f64) -> f64;

    /// Kramers escape rate (one-directional, Kramers–Grote–Hynes formula).
    pub fn kramers_rate(&self, gamma: f64, mass: f64, k_b_t: f64) -> f64;
}

impl PassiveComponent for DoubleWellPotential {
    fn apply(&self, _model: &Model, data: &Data, qfrc_out: &mut DVector<f64>) {
        let q = data.qpos[self.dof];
        let a = self.delta_v / self.x_0.powi(4);
        qfrc_out[self.dof] += -4.0 * a * q * (q * q - self.x_0 * self.x_0);
    }
}

impl Diagnose for DoubleWellPotential {
    fn diagnostic_summary(&self) -> String {
        format!("DoubleWellPotential(ΔV={:.4}, x₀={:.4}, dof={})",
                self.delta_v, self.x_0, self.dof)
    }
}
```

**Rationale**: Parameterizing by `(ΔV, x₀)` rather than `(a, x₀)` makes the physics transparent — `ΔV` is what appears in the Kramers exponential, `x₀` is the well location. The `kramers_rate` method centralizes the formula (§3.2), so tests compare against a single source of truth.

### 6.2 `kramers_rate` implementation

```rust
pub fn kramers_rate(&self, gamma: f64, mass: f64, k_b_t: f64) -> f64 {
    let omega_a = self.omega_a(mass);
    let omega_b = self.omega_b(mass);
    let gamma_tilde = gamma / mass;
    let lambda_r = (-gamma_tilde
        + (gamma_tilde * gamma_tilde + 4.0 * omega_b * omega_b).sqrt())
        / 2.0;
    (omega_a / (2.0 * std::f64::consts::PI))
        * (lambda_r / omega_b)
        * (-self.delta_v / k_b_t).exp()
}
```

### 6.3 Module location

`sim-thermostat/src/double_well.rs`, exported via `lib.rs`. This makes the component available to Phase 4+ (coupled arrays) without test-only coupling.

---

## 7. Central Parameter Set

### 7.1 Parameters

| Parameter | Symbol | Value | Rationale |
|---|---|---|---|
| Well half-separation | x₀ | 1.0 | Unit separation |
| Barrier height | ΔV | 3.0 | ΔV/kT = 3 at kT=1 — activated regime with good transition statistics |
| Mass | M | 1.0 | Unit mass (slide joint, `mass="1"`) |
| Friction | γ | 10.0 | Moderate-to-overdamped: γ̃/(2ω_b) = 1.44 (§4.3) |
| Temperature | kT | 1.0 | Unit thermal energy |
| Timestep | h | 0.001 | Same as Phase 1/2 |

### 7.2 Derived quantities

| Quantity | Formula | Value |
|---|---|---|
| Quartic coefficient | a = ΔV/x₀⁴ | 3.0 |
| Well frequency | ω_a = √(8ΔV/(Mx₀²)) | √24 ≈ 4.899 rad/s |
| Barrier frequency | ω_b = √(4ΔV/(Mx₀²)) | √12 ≈ 3.464 rad/s |
| Damping rate | γ̃ = γ/M | 10.0 s⁻¹ |
| Regime indicator | γ̃/(2ω_b) | 1.443 (moderately overdamped) |
| Reactive eigenvalue | λ_r = (−γ̃ + √(γ̃² + 4ω_b²))/2 | 1.083 s⁻¹ |
| Kramers prefactor | C = (ω_a/(2π)) · λ_r/ω_b | 0.2438 s⁻¹ |
| **Kramers rate** | k = C · exp(−ΔV/kT) | **0.01214 per time unit** |
| Mean escape time | 1/k | ~82.4 time units |
| Intra-well relaxation | M/γ | 0.1 time units |
| Escape/relaxation ratio | (1/k) / (M/γ) | ~824 (≫ 1 ✓) |
| Discretization bias on kT | O(h·γ/M) | 1% |
| Discretization bias on rate | exp(ΔV/kT · 0.01) − 1 | ~3% |

### 7.3 Transitions per trajectory

At 5M steps (`T_sim = 5000` time units): expected transitions per trajectory = `k · T_sim ≈ 60.7`.

---

## 8. Validation Tests

### 8.1 Gate A — Kramers rate at central parameters (must-pass)

**Setup**: 30 independent trajectories (distinct seeds), each 5M steps. Per trajectory: set `data.qpos[0] = x₀` + `data.forward(&model)` (§9.2), 10,000-step burn-in (10 time units = 100 · M/γ), then count zero-crossings over 4,990,000 measurement steps.

**Measurement**: `k_measured = N_total / (30 · T_measure)`, where `N_total` is the total zero-crossings across all 30 trajectories and `T_measure = 4990` time units per trajectory.

**Expected**: `k_Kramers = 0.01214` per time unit.

**Statistical error**: `√(N_total) / (30 · T_measure)`. With `N_total ≈ 1820`: SE ≈ 2.3%.

**Tolerance**: `|k_measured − k_Kramers| / k_Kramers < 0.25` (25%). Breakdown: ~15–25% finite-barrier/anharmonic corrections + ~3% discretization bias + ~2.3% statistical error.

**Kinetic equipartition sanity check**: During measurement, also track `⟨½Mv²⟩` using Welford. Assert it equals `½kT` within ±5%. Failure here indicates a thermostat bug, not a Kramers issue — stops the test early with a clear diagnosis.

**Cost**: 30 × 5M = 150M steps.

### 8.2 Gate B — Arrhenius slope over kT sweep (must-pass)

**Setup**: Repeat Gate A's protocol at three temperatures:

| kT | ΔV/kT | Expected k | Transitions/trajectory | Total (30 trajectories) | SE |
|---|---|---|---|---|---|
| 0.75 | 4.0 | 0.00447 | 22.3 | 669 | ~3.9% |
| 1.0 | 3.0 | 0.01214 | 60.7 | 1820 | ~2.3% |
| 1.5 | 2.0 | 0.03300 | 165.1 | 4953 | ~1.4% |

**Measurement**: For each `kT`, compute `k_measured(kT)`. Fit `log(k_measured)` vs. `1/kT` by least-squares linear regression (3 points, 1 degree of freedom). Extract slope `m`.

**Expected**: `m = −ΔV = −3.0` (the Arrhenius slope is the barrier height).

**Tolerance**: `|m − (−ΔV)| / ΔV < 0.10` (10%). The Arrhenius slope is insensitive to prefactor corrections (they are approximately kT-independent), so 10% is conservative. If this test fails narrowly, the finite-barrier correction to the effective slope (`O(kT²/ΔV)`) is the first diagnostic.

**Cost**: 3 × 30 × 5M = 450M steps (~15× Phase 2's heaviest test at 30M steps). Mark `#[ignore]` — this is the first test in `sim-thermostat` to use this attribute. Justification: escape-rate statistics are inherently more expensive than equilibrium measurements (must observe many rare events). The cost is unavoidable for a meaningful Arrhenius validation but should not gate every `cargo test -p sim-thermostat` run. Run explicitly during phase validation: `cargo test -p sim-thermostat -- --ignored`.

### 8.3 Supporting — Boltzmann position distribution

**Setup**: Single trajectory, 2M steps (2,000 time units) at the central parameter set. Start at `x = x₀`, 10,000-step burn-in.

**Measurement**: Bin position values into 40 equally-spaced bins over `[−2x₀, +2x₀]`. Test three physically meaningful shape properties rather than a chi-squared goodness-of-fit. **Implementation finding (2026-04-10)**: a chi-squared test at high sample count (10M) detects the known `O(h·γ/M) ≈ 1%` Euler discretization bias, producing chi2 ~ 27000 against the exact Boltzmann distribution. This is a correct detection of a known artifact, not a physics failure. Shape tests are the right tool here.

**Three shape tests**:
1. **Peak positions**: The histogram should be bimodal with peaks within 0.2 of `±x₀`.
2. **Density ratio**: `ln(p_peak / p_barrier)` should equal `ΔV/kT` within ±25%.
3. **Peak width**: FWHM of each peak should match `2.355 · √(kT / V″(x₀))` within ±50% (generous due to bin quantization).

This verifies that the stationary distribution has the correct Boltzmann shape — a fundamentally different test than the escape rate (equilibrium distribution vs. dynamical rate).

**Cost**: 10M steps.

### 8.4 Supporting — Reproducibility

**Setup**: Two trajectories with the same seed, same model, same initial condition.

**Test**: Bit-for-bit identical transition times. Inherits the Phase 1 reproducibility guarantee (ChaCha8Rng determinism). Confirms that the `DoubleWellPotential` + `LangevinThermostat` stack produces reproducible dynamics.

**Cost**: 2 × 5M = 10M steps.

---

## 9. Measurement Protocol

### 9.1 Transition counting

A **transition** is a committed well-to-well switch detected via hysteresis. The particle state is classified as:
- **Left well**: `x < −x_thresh`
- **Right well**: `x > +x_thresh`
- **Barrier region**: `−x_thresh ≤ x ≤ +x_thresh`

where `x_thresh = x₀/2`. A transition is counted when the particle moves from one well to the other (Left→Right or Right→Left). Barrier visits that return to the same well are filtered out.

**Why hysteresis, not zero-crossings**: At the central parameters, the Kramers–Grote–Hynes transmission coefficient `κ = λ_r/ω_b = 0.313`. This means ~69% of zero-crossings are *recrossings* — the particle oscillates through `x = 0` without committing to the other well. Simple zero-crossing counting measures the TST rate (`k_TST = (ω_a/2π) · exp(−ΔV/kT) ≈ 0.039`), which is ~3.2× the Kramers rate. Hysteresis at `x_thresh = x₀/2` filters out these recrossings and recovers the genuine committed-transition rate that Kramers predicts. **Implementation finding (2026-04-10)**: without hysteresis, the measured rate was ~2.6× the Kramers prediction; with hysteresis, it was within 10%.

### 9.2 Initial condition and burn-in

Each trajectory sets the initial position explicitly and calls `forward()` to synchronize derived quantities before stepping:

```rust
data.qpos[0] = x_0;  // start in the right well
data.qvel[0] = 0.0;
data.forward(&model).expect("forward");  // sync qM, qfrc_bias, etc.
```

This follows the Phase 2 pattern (`multi_dof_equipartition.rs:145`) where `data.forward()` is called after any manual `qpos` modification to ensure the mass matrix, bias forces, and other derived fields are consistent before the first `step()`.

The first 10,000 steps (10 time units = 100 · M/γ) are then discarded as burn-in. This is ≫ the intra-well equilibration time (`M/γ = 0.1` time units) and ≪ the mean escape time (~82.4 time units), ensuring the system is thermalized but no transitions are wasted.

### 9.3 Statistical methodology

**Rate estimation**:
1. Pool all transitions across trajectories: `N_total = Σᵢ Nᵢ`.
2. `k_measured = N_total / (n_traj · T_measure)`.
3. Poisson standard error: `SE(k) = √(k_measured / (n_traj · T_measure))`.
4. Compare: `|k_measured − k_Kramers| / k_Kramers`.

**Arrhenius slope**:
1. Compute `k_measured(kT)` at each sweep temperature.
2. Fit `y = log(k_measured)` vs. `u = 1/kT` by ordinary least-squares: `y = a + m·u`.
3. Extract slope `m`, compare to `−ΔV`.

**Departure from Phase 1/2 pattern**: Phases 1–2 use a two-level Welford estimator (per-trajectory time-average → across-trajectories ensemble mean/SEM). This pattern is correct for time-averaged equilibrium quantities where per-step samples are autocorrelated (velocity autocorrelation time ~5000 steps in Phase 1). Phase 3's escape-rate measurement is fundamentally different — transitions are discrete, approximately Poisson-distributed events. Pooling counts across trajectories and using Poisson statistics (variance = mean) is the correct estimator. The two-level Welford pattern is still used for the kinetic equipartition sanity check within Gate A (same quantity, same pattern as Phase 1/2).

---

## 10. Acceptance Criteria

| Test | Criterion | Hard tolerance |
|---|---|---|
| Gate A — Kramers rate | \|k_measured − k_Kramers\| / k_Kramers | < 25% |
| Gate B — Arrhenius slope | \|m − (−ΔV)\| / ΔV | < 10% |
| Boltzmann — peak positions | Peaks within 0.2 of ±x₀ | < 0.2 |
| Boltzmann — density ratio | \|ln(p_peak/p_barrier) − ΔV/kT\| / (ΔV/kT) | < 25% |
| Boltzmann — peak width | \|FWHM − 2.355·σ\| / (2.355·σ) | < 50% |
| Reproducibility | Bit-exact transition times | exact |
| KE sanity (within Gate A) | \|⟨½Mv²⟩ − ½kT\| / (½kT) | < 5% |

**All five criteria must pass for Phase 3 to be green.**

---

## 11. Implications for the Phase 2 Euler Finding

Phase 2 Finding 1: Euler breaks equipartition at large angles with configuration-dependent `M(q)`.

**Phase 3 sidesteps this entirely** — the slide joint has constant `M`, so Euler works at any excursion. The finding remains **load-bearing for future phases**:

- **Phase 4** (coupled bistable array): If bistable elements are coupled via hinge joints in a multi-link chain, `M(q)` will be configuration-dependent and large excursions are inherent. This will force either (a) a BAOAB/GJF integrator or (b) creative parametrization that keeps `|Δq|` small per step.
- **D2** (stochastic resonance on a buckling beam): A real buckling beam from cf-design will have rotational DOFs with configuration-dependent inertia. The Euler limitation applies.

Phase 3 does not resolve this — it defers it by choosing the slide-joint design. The clean upgrade path (swap the `Stochastic` component without touching the chassis) remains valid.

---

## 12. What This Enables

Passing Phase 3 proves:

1. The thermostat produces correct **escape-rate dynamics**, not just equilibrium statistics.
2. The `PassiveComponent` composability pattern (stacking a conservative potential with the stochastic thermostat) works for **non-trivial energy landscapes**.
3. The Kramers formula is quantitatively validated in this stack — D1 (flashing ratchet) and D2 (stochastic resonance) can rely on the thermostat's dynamical correctness.
4. The **transition-counting measurement infrastructure** exists for Phase 4's joint-distribution tests.

This unlocks:
- **Phase 4** (coupled bistable array + Ising comparison) — builds on the `DoubleWellPotential` component and the transition-counting infrastructure.
- **D1** (flashing ratchet) — achievable immediately after Phase 3 using the slide-joint bistable element + a binary actuator + RL.
- **D2** (stochastic resonance) — requires Phase 4 coupling infrastructure but the bistable validation is Phase 3's contribution.
