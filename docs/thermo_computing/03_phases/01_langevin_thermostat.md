# Phase 1 — Langevin Thermostat (1-DOF)

> **Status**: Active spec, ready for implementation.
> **Branch**: `feature/thermo-doc-review`
> **Owner**: Jon
> **Rollback target**: `git reset --hard f8d0550` returns to the post-content-follow-ons state, before this spec landed. Branch is never force-pushed.
> **Parents**:
> - [`02_foundations/chassis_design.md`](../02_foundations/chassis_design.md) (Decisions 1–7 — the bolt patterns this spec inherits)
> - [`02_foundations/working_principles.md`](../02_foundations/working_principles.md) (sharpen-the-axe discipline)
> - [`02_foundations/open_questions.md`](../02_foundations/open_questions.md) (Q1 gates Phase 2)
> - [`overview.md`](./overview.md) (the Phases-2–7 outline this spec replaces the Phase 1 sketch in)
> - Recon log parts [02](../04_recon_log/2026-04-09_part_02_forward_step.md), [04](../04_recon_log/2026-04-09_part_04_cb_passive.md), [06](../04_recon_log/2026-04-09_part_06_prng.md), [08](../04_recon_log/2026-04-09_part_08_timestep_variability.md), [10](../04_recon_log/2026-04-09_part_10_module_location.md)

This spec is a **planning document for an immediate implementation**. It does not re-litigate any chassis decision. It does not generate its own rubric, doc-review, or follow-ons artifacts. Its job is to be precise enough that the implementation it drives can be reviewed against it. The chassis owns the *why*; this spec owns the *how* for one phase.

---

## 1. Goal

Build a `LangevinThermostat` that, when bolted onto a 1-DOF damped harmonic oscillator via the Phase 1 chassis (`PassiveStack` + `cb_passive`), produces a stationary velocity distribution whose kinetic energy satisfies the equipartition theorem:

$$\left\langle \tfrac{1}{2} M v^2 \right\rangle = \tfrac{1}{2} k_B T$$

to within a sampling-error tolerance the spec locks in §7. Passing this gate is the entrance condition for every phase above. **It is the foundation, not a warm-up** — sloppy Phase 1 undermines every later validation gate. The expected level of margin is "the central parameter set passes by ~3σ on the first run, repeatedly."

The deliverable is the `sim/L0/thermostat/` crate as inventoried in chassis Decision 6, with the four validation tests in §7–§10 all green.

---

## 2. Scope

**In scope for Phase 1:**
- One physics setup: 1-DOF damped harmonic oscillator (`M=1`, `k_spring=1`, `dof_damping=0`, gravity off, `Integrator::Euler`).
- One component: `LangevinThermostat`, implementing `PassiveComponent + Stochastic + Diagnose`.
- One mandatory gate: equipartition on the central parameter set (§7).
- Three supporting tests: callback firing-count (§8), reproducibility from seed (§9), Stochastic gating sanity (§10).
- The full chassis API surface (`PassiveComponent`, `PassiveStack`, `install_per_env`, `Diagnose`, `Stochastic`, `WelfordOnline`, `assert_within_n_sigma`, `sample_stats`).

**Explicitly out of scope** (each blocked or deferred elsewhere):
- Free / articulated bodies — Phase 2, gated on Q1 (constrained Langevin).
- Bistable elements, Kramers escape rates — Phase 3+.
- Differentiable contexts (FD perturbation under `disable_stochastic()`) — full validation at Phase 5; Phase 1 ships only the smoke test in §10.
- BatchSim parallel-env runs — `install_per_env` ships with the chassis (chassis Decision 3) but Phase 1 validates one env per trajectory; per-env independence is a Phase 2+ concern.
- Plugin passive forces firing after `cb_passive` (`forward/passive.rs:723-735`) — no plugins in the Phase 1 model.
- BAOAB / higher-accuracy integrators — clear upgrade path (the chassis is integrator-agnostic), not Phase 1 work.

---

## 3. Algorithm

Explicit Langevin via Euler-Maruyama. Per-DOF, every step:

```
qfrc_out[i] += −γ_i · qvel[i]  +  sqrt(2 · γ_i · k_B · T / h) · z_i,    z_i ~ N(0, 1)
                └── damping ──┘   └────── FDT-paired noise ──────┘
```

The damping coefficient `γ_i` and the bath temperature `k_B·T` are owned by the `LangevinThermostat` instance — **not** by `model.dof_damping`, which stays at zero (Q4 resolution, recon log part 2). The fluctuation–dissipation relation `σ² = 2γkT/h` is the only physics statement the implementation makes; everything else is bookkeeping. Both the damping and the noise are **passive thermal forces**, which is exactly what `qfrc_passive` is for (recon log part 4 ontology).

The discretization-bias temperature error is `O(h·γ/M)`. At the central parameter set (`h=0.001`, `γ=0.1`, `M=1`) that is ≈ `10⁻⁴` of `½kT` — well below the §7 sampling-error tolerance of `4.5%`. The gate passes with margin, not at threshold (sharpen-the-axe). Higher-order schemes (BAOAB, GJF) reduce this further but are not needed; the upgrade path is to swap the `Stochastic` component without touching the chassis.

The thermostat is integrator-agnostic — `Implicit` and `ImplicitFast` will both work via the same `qfrc_passive` path (recon log part 2) — but Phase 1 only validates against `Euler` to keep the test surface minimal.

---

## 4. API surface (chassis-inherited)

The chassis (Decisions 1–7) defines every API the user interacts with. This section walks the surface verbatim so an implementer can read the spec without flipping to the chassis doc. Nothing here is new design.

### 4.1 The `PassiveComponent` trait (Decision 1 + M5)

```rust
pub trait PassiveComponent: Send + Sync + 'static {
    fn apply(&self, model: &Model, data: &Data, qfrc_out: &mut DVector<f64>);
    fn as_stochastic(&self) -> Option<&dyn Stochastic> { None }
}
```

`apply` is the only required method. The component reads `model` and `data` immutably and accumulates its per-DOF contribution into `qfrc_out`. **No mutable access to `Data`** — the M5 contract makes this uncompilable, not just discouraged. The `as_stochastic` hook is the gating introspection from Decision 7 (default `None`).

### 4.2 The `Stochastic` trait (Decision 7)

```rust
pub trait Stochastic: Send + Sync {
    fn set_stochastic_active(&self, active: bool);
    fn is_stochastic_active(&self) -> bool;
}
```

When inactive, `apply` produces only the deterministic part (`-γ·v` for the Langevin thermostat). The chassis exposes both `PassiveStack::set_all_stochastic(bool)` (one-shot) and `PassiveStack::disable_stochastic() -> StochasticGuard<'_>` (RAII; the FD/autograd entry point).

### 4.3 The `Diagnose` trait (Decision 4)

```rust
pub trait Diagnose {
    fn diagnostic_summary(&self) -> String;
}
```

`LangevinThermostat` implements it as:

```rust
format!("LangevinThermostat(kT={:.6}, n_dofs={}, seed={})",
        self.k_b_t, self.gamma.len(), self.seed)
```

### 4.4 The `LangevinThermostat` struct

```rust
pub struct LangevinThermostat {
    gamma: DVector<f64>,
    k_b_t: f64,
    seed: u64,                    // retained for diagnostic_summary; not consumed at apply-time
    rng: Mutex<ChaCha8Rng>,
    stochastic_active: AtomicBool, // defaults to true
}

impl LangevinThermostat {
    pub fn new(gamma: DVector<f64>, k_b_t: f64, seed: u64) -> Self;
}
```

Constructor signature is fixed by recon log parts 6 + 8: three parameters, no `h` (`h = model.timestep` is read fresh inside `apply` every step — robust to legal-but-unusual mid-simulation timestep mutation, cost is one `f64` read + one `sqrt` per DOF per step, negligible at Phase 1's `nv=1`).

`Mutex<ChaCha8Rng>` is forced by `cb_passive` being `Fn` not `FnMut` (recon log part 4). `ChaCha8Rng` is the chassis-mandated PRNG choice from recon log part 6 (Scheme B): bit-stable across `rand` versions, no silent reproducibility hazard. Gaussian draws via `rand_distr::StandardNormal` with the `(2γkT/h)^{1/2}` scale applied outside the sample.

### 4.5 User code — single-env Phase 1 setup

```rust
use sim_core::DVector;
use sim_thermostat::{LangevinThermostat, PassiveStack};

let mut model = sim_mjcf::load_model(SHO_XML)?;
let mut data  = model.make_data();

PassiveStack::builder()
    .with(LangevinThermostat::new(
        DVector::from_element(model.nv, 0.1),
        1.0,
        42,
    ))
    .build()
    .install(&mut model);

for _ in 0..n_steps {
    data.step(&model)?;
}
```

`Data` construction is via `model.make_data()` (`sim/L0/core/src/types/model_init.rs:445`), the workspace's canonical idiom — every existing `cb_passive` consumer in `tests/integration/callbacks.rs` constructs its `Data` this way. There is no `Data::new(&model)` constructor; `make_data` is the only path. `DVector` is re-exported from `nalgebra` via `sim_core::DVector` at `sim/L0/core/src/lib.rs:257`, so the user code does not need a direct `nalgebra` import.

**No split-step.** Plain `data.step(&model)?`, joining the dominant convention catalogued in recon log part 5 (both `ml-bridge::ActionSpace::apply` and `coupled_pendulums.rs` use plain `step()`). The `step` method is at `sim/L0/core/src/forward/mod.rs:220` with signature `pub fn step(&mut self, model: &Model) -> Result<(), StepError>`.

### 4.6 The chassis test utilities (Decision 5 + M4)

```rust
// sim_thermostat::test_utils

pub fn assert_within_n_sigma(
    measured: f64,
    expected: f64,
    standard_error: f64,
    n_sigma: f64,
    description: &str,
);

#[derive(Clone)]
pub struct WelfordOnline { /* count, mean, m2 — private */ }
impl WelfordOnline {
    pub fn new() -> Self;
    pub fn push(&mut self, x: f64);
    pub fn reset(&mut self);                // M4 — burn-in support
    pub fn merge(&mut self, other: &Self);  // M4 — Chan/Pébay parallel merge
    pub fn count(&self) -> usize;
    pub fn mean(&self) -> f64;
    pub fn variance(&self) -> f64;          // unbiased (n-1)
    pub fn std_error_of_mean(&self) -> f64; // sqrt(variance / count)
}

pub fn sample_stats(data: &[f64]) -> (f64, f64); // (mean, variance)
```

Default `n_sigma = 3.0` per chassis sub-decision N2. The Phase 1 tests use the default.

---

## 5. Crate layout

Per chassis Decision 6 (Scheme A — flat ml-bridge style). New directory at `sim/L0/thermostat/`, package name `sim-thermostat`.

```
sim/L0/thermostat/                       (NEW directory)
├── Cargo.toml                           (~20 LOC)
├── src/
│   ├── lib.rs                           (~50 LOC: crate docs + re-exports only)
│   ├── component.rs                     (~70 LOC: PassiveComponent + Stochastic traits + unit tests)
│   ├── stack.rs                         (~170 LOC: PassiveStack + builder + install + install_per_env + EnvBatch + StochasticGuard + unit tests)
│   ├── diagnose.rs                      (~20 LOC: Diagnose trait + unit test)
│   ├── langevin.rs                      (~150 LOC: LangevinThermostat struct + 3 trait impls + unit tests)
│   └── test_utils.rs                    (~170 LOC: WelfordOnline + assert_within_n_sigma + sample_stats + unit tests)
└── tests/
    └── langevin_thermostat.rs           (~250 LOC: 4 integration tests from §7–§10)
```

Total Phase 1 footprint: **~900 LOC across 8 files**.

**Cargo.toml dependencies** (from recon log parts 6 + 10):

```toml
[dependencies]
sim-core    = { workspace = true }
nalgebra    = { workspace = true }
rand        = { workspace = true }
rand_chacha = { workspace = true }       # NEW workspace dep — see ordering note below
rand_distr  = { workspace = true }
```

**Workspace `Cargo.toml` ordering — operational note**: the workspace root `Cargo.toml` (lines 375-376 verified during the stress-test pass) currently declares `rand = "0.9"` and `rand_distr = "0.5"` but **not** `rand_chacha`. `rand_chacha 0.9.0` is already in `Cargo.lock` as a transitive dep of `rand`, so adding it as a workspace dep is a zero-compile-cost organizational change. **The implementation must edit the workspace `Cargo.toml` first** (add `rand_chacha = "0.9"` to the `[workspace.dependencies]` block alongside `rand` and `rand_distr`), **then** create `sim/L0/thermostat/Cargo.toml`. Reverse order produces a workspace-resolution failure on the new crate's first build.

`lib.rs` is a door: crate-level rustdoc + `pub use` re-exports only, no type definitions, no impl blocks, no functions. `pub mod test_utils;` is the one `pub` module declaration; everything else is `mod` (private) with selective re-exports of `PassiveComponent`, `PassiveStack`, `PassiveStackBuilder`, `EnvBatch`, `StochasticGuard`, `Stochastic`, `Diagnose`, `LangevinThermostat`.

**sim-core stays rand-free in production.** The thermostat lives in the sibling crate exactly so this property is observable from the dep graph alone (recon log part 10, item 8 constraint). The implementation must not weaken this — no `rand`/`rand_chacha`/`rand_distr` additions to `sim/L0/core/Cargo.toml` are permitted as part of Phase 1.

**Implementation order** (bottom-up by dependency):
`Cargo.toml` → `lib.rs` (door scaffold + module declarations) → `component.rs` (`PassiveComponent` + `Stochastic` traits — depended on by everything below) → `stack.rs` (`PassiveStack` + builder + `install_per_env` + `EnvBatch` + `StochasticGuard` — depends on `component`) → `diagnose.rs` (independent) → `test_utils.rs` (independent of component code, depended on by tests) → `langevin.rs` (`LangevinThermostat` — implements all three traits, depends on everything else in the crate) → `tests/langevin_thermostat.rs` (depends on the full surface). Each natural commit boundary: scaffold, traits, stack, langevin, tests.

---

## 6. The MJCF model

The 1-DOF damped harmonic oscillator is constructed via MJCF XML (test-convention 1 from recon log part 9 — every callback test in `tests/integration/callbacks.rs` and almost every passive test in `tests/integration/passive_forces.rs` builds models this way). The canonical idiom for a slide joint with a linear spring is the `stiffness` + `springref` + `ref` attribute set directly on the `<joint>` element, verified by `tests/integration/passive_forces.rs::test_springref_shifts_equilibrium` (line 35). The Phase 1 model:

```xml
<mujoco model="sho_1d">
  <option timestep="0.001" gravity="0 0 0" integrator="Euler"/>
  <worldbody>
    <body name="particle">
      <joint name="x" type="slide" axis="1 0 0"
             stiffness="1" damping="0" springref="0" ref="0"/>
      <geom type="sphere" size="0.05" mass="1"/>
    </body>
  </worldbody>
</mujoco>
```

**The implementation must verify the loaded model satisfies all of these properties** before any thermostat is installed:

| Property | Required value | Verification |
|---|---|---|
| `model.nv` | `1` | direct read |
| `model.nq` | `1` | direct read |
| `model.timestep` | `0.001` | direct read |
| `model.integrator` | `Integrator::Euler` | direct read |
| Mass on the moving body | `1.0` | direct read of `model.body_mass[1]` (`pub body_mass: Vec<f64>` at `sim/L0/core/src/types/model.rs:139`; body 0 is the world body, body 1 is the user body) |
| `model.dof_damping[0]` | `0.0` | direct read — **critical**; thermostat owns damping (Q4 option (a) from recon log part 2). Model damping would compound with thermostat damping and silently shift the equilibrium temperature. |
| Linear restoring force | `F = −k·x` with `k = 1.0` | step the un-thermostatted model from `qpos[0]=0.5, qvel[0]=0`; assert `qacc[0] ≈ −0.5` on the first forward solve (spring force `-k·(x − springref) = −1·0.5 = −0.5`, mass `1.0`). This is a sanity-check fixture inside the test module, run once before the equipartition gate. |

The verification pass is a `#[test]` named `test_sho_model_invariants` that runs first; if it fails, the spec is no longer satisfiable on this model and the rest of the suite is meaningless. Crash-loud rather than silently miscompare downstream.

---

## 7. Validation test #1 — equipartition (THE Phase 1 gate)

This is the test the entire Phase 1 effort exists to pass. It is the gate to Phases 2–7. It must pass with margin, not at threshold.

### 7.1 The Decision 5 validation parameter pick: **option β**

The chassis flagged three options for fixing the recon-log-part-2 sampling-error miscalculation (Decision 5 side finding, M1 correction):

| | Trajectory shape | Per-test cost | Helper requirement | Combined std error |
|---|---|---|---|---|
| α | 1 trajectory of ~10⁷ steps | ~10⁷ sim steps | needs `integrated_autocorrelation_time` (deferred per Decision 5) | ~4.5% of ½kT |
| **β** | **100 trajectories of 10⁵ steps each** | **~10⁷ sim steps** | **only `WelfordOnline` + `merge` (M4 — already shipping)** | **~4.5% of ½kT** |
| γ | 1 trajectory of 10⁵ steps, loose tolerance | ~10⁵ sim steps | none extra | ~5–10% of ½kT, no discretization-bias detection |

**Pick: option β.** Five reasons in priority order:

1. **It is the only option compatible with the chassis test utilities Phase 1 actually ships.** Decision 5 explicitly defers `integrated_autocorrelation_time` to whichever later phase first needs it (a Sokal-style automatic-windowing implementation is its own ~100 LOC of statistical code with its own correctness validation needs). Option α requires it; option β does not. Option γ technically does not require it either but ships with no discretization-bias-detection capability, which fails the "pass with margin, not at threshold" rule.
2. **It uses `WelfordOnline::merge` (chassis M4) end-to-end.** Per-trajectory accumulators get folded into a global accumulator via the Chan/Pébay formula. The chassis added `merge` specifically for this option; using it for the Phase 1 gate validates `merge` itself as a side effect.
3. **It exercises the seed/RNG path under normal use.** Each of the 100 trajectories constructs a fresh `LangevinThermostat::new(gamma, k_b_t, seed_base + i)`, which exercises the same code path the eventual `install_per_env` BatchSim case will use. The reproducibility test in §10 then locks the property that this is bit-stable.
4. **It avoids autocorrelation analysis entirely** — each trajectory contributes one *aggregated* `(mean, variance)` pair via its own per-trajectory `WelfordOnline`, and the final assertion runs on the merged 100-trajectory accumulator. The autocorrelation between consecutive samples *within* a trajectory is folded into the per-trajectory variance, where it belongs.
5. **Total wall-clock cost is seconds.** ~10⁷ sim steps at `nv=1` with the constraint solver running on a 1-DOF system is order-of-seconds on any developer machine. Acceptable for an integration test that runs in CI on every thermo-touching PR.

### 7.2 Parameter set (locked)

| Parameter | Value | Source |
|---|---|---|
| `M` (mass) | `1.0` | central baseline |
| `k_spring` | `1.0` | central baseline; gives natural frequency `ω = 1` |
| `γ_thermostat` | `DVector::from_element(1, 0.1)` | central baseline; underdamped ratio `ζ ≈ 0.05` |
| `k_B·T` | `1.0` | central baseline |
| `h` (timestep) | `0.001` | `h·ω = 10⁻³`, `h·γ/M = 10⁻⁴` (discretization-bias floor) |
| `Integrator` | `Euler` | Phase 1 only |
| `dof_damping` | `0` (model) | thermostat owns damping (Q4 option (a)) |
| `n_burn_in` | `25_000` steps (= `5·τ_int`) | `τ_int ≈ M/(2γh) = 5000` (recon log part 2) |
| `n_measure` | `100_000` steps | per trajectory |
| `n_traj` | `100` | independent trajectories |
| `seed_base` | `0xC0FFEE_u64` | arbitrary fixed; reproducibility-from-seed locks bit-stability |
| `n_sigma` | `3.0` | chassis Decision 5 sub-decision N2 default |

**Statistical accounting** (per the M1 correction):
- Per-trajectory effective sample count `N_eff ≈ n_measure / (1 + 2·τ_int) ≈ 10`.
- Per-trajectory std error of `⟨½v²⟩` ≈ `(½kT)·√2/√10 ≈ 0.45·(½kT)`.
- 100-trajectory aggregated std error ≈ per-traj std error / `√100` ≈ `0.045·(½kT)` ≈ **±4.5% of ½kT**.
- At `n_sigma=3.0`, the gate's fail boundary is at `±13.5% of ½kT`. Discretization bias `~10⁻⁴ · ½kT` is two orders of magnitude below the gate. **Margin is real.**

### 7.3 Test pseudocode

```rust
#[test]
fn test_equipartition_central_parameter_set() {
    let xml = SHO_1D_XML;
    let n_burn_in   = 25_000;
    let n_measure   = 100_000;
    let n_traj      = 100;
    let seed_base   = 0xC0FFEE_u64;
    let k_b_t       = 1.0;
    let gamma_value = 0.1;

    let mut global = WelfordOnline::new();

    for i in 0..n_traj {
        let mut model = sim_mjcf::load_model(xml).expect("load");
        let mut data  = model.make_data();

        PassiveStack::builder()
            .with(LangevinThermostat::new(
                DVector::from_element(model.nv, gamma_value),
                k_b_t,
                seed_base + i as u64,
            ))
            .build()
            .install(&mut model);

        // Burn in (no measurement).
        for _ in 0..n_burn_in { data.step(&model).expect("burn-in step"); }

        // Measure ½ M v² into a per-trajectory accumulator.
        let mut traj = WelfordOnline::new();
        for _ in 0..n_measure {
            data.step(&model).expect("measure step");
            traj.push(0.5 * 1.0 * data.qvel[0] * data.qvel[0]);
        }

        // Merge into the global accumulator (Chan/Pébay, M4).
        global.merge(&traj);
    }

    let measured     = global.mean();
    let expected     = 0.5 * k_b_t;
    let std_error    = global.std_error_of_mean();
    assert_within_n_sigma(
        measured, expected, std_error, 3.0,
        "equipartition central: 1-DOF damped harmonic oscillator, 100 traj × 10⁵ steps",
    );
}
```

### 7.4 The γ + T sweep (separate test, not the gate)

A 3×3 sweep over `γ ∈ {0.01, 0.1, 1.0}` and `k_B·T ∈ {0.5, 1.0, 2.0}` verifies T-linearity (`⟨½v²⟩ ∝ T`) and γ-independence (the stationary temperature must not depend on γ — γ controls relaxation time, not equilibrium). Smaller per-combination trajectory count (`n_traj = 30`) keeps total cost bounded; the central combination is already proven by §7.3 so the sweep is verification, not the gate. Run as a separate `#[test]` that may be marked `#[ignore]` if total runtime becomes a CI concern.

Combinations where `h·γ/M ≥ 10⁻³` (i.e. the `γ=1.0` row) push the discretization-bias closer to the sampling tolerance and are the most informative — if the test ever fails on the `γ=1.0` rows but passes on the `γ=0.01`/`γ=0.1` rows, that is a discretization-bias signal, and the spec's response is to drop `h` rather than loosen the tolerance.

---

## 8. Validation test #2 — callback firing-count

Per recon log part 9: the thermostat is the workspace's first **stateful** `cb_passive` consumer, so its tests need to establish the precedent for callback-RNG reproducibility. The existing `tests/integration/callbacks.rs::passive_callback_adds_force` (line 11) only proves the callback fires; it does not exercise the stateful + `Mutex<RNG>` + per-step-RNG-advance path the thermostat introduces.

The test:

1. Construct an `Arc<AtomicUsize>` counter; install a thin diagnostic `PassiveComponent` wrapper around the `LangevinThermostat` that increments the counter inside `apply` before delegating to the inner thermostat.
2. Run `data.step(&model)` exactly `K = 1000` times.
3. Assert the counter is exactly `K` — `cb_passive` fired once per `step()`, hard equality.

This test is small (~25 LOC) and is the workspace's first hard-equality check on `cb_passive` invocation count. Future stateful `cb_passive` consumers can copy the pattern.

**RNG-advancement-by-construction note**: a separate test asserting "the RNG advanced by exactly `K · n_dofs` draws over `K` steps" is **deliberately not part of Phase 1**. The combination of this test (callback fires `K` times) and §9 (bit-stable seed reproducibility) is already a complete proof of RNG advancement: the `apply` method's per-call RNG-draw count is structural source code, not behavior that needs separate runtime verification, and adding a public test-only `draw_for_test()` accessor on `LangevinThermostat` would pollute the chassis's deliberately minimal Decision 1 surface. If a future phase needs in-test RNG inspection for some other reason, it adds the accessor then with a real motivating use case.

---

## 9. Validation test #3 — reproducibility from seed

`ChaCha8Rng` is bit-stable across `rand`/`rand_chacha` versions (recon log part 6, the entire reason Scheme B was chosen over Scheme A `StdRng`). The test locks this property:

1. Construct two independent simulations with identical MJCF, identical parameters, identical seed.
2. Step each forward `N = 10_000` steps.
3. Assert `data1.qvel[0] == data2.qvel[0]` (and `qpos[0]`) as **hard equality**, no `assert_relative_eq!`. f64 byte-for-byte.

A failure of this test means either the RNG implementation drifted (catastrophic — the entire reproducibility-as-foundation argument from recon log part 6 breaks) or the chassis introduced nondeterminism somewhere it shouldn't have (e.g., a `HashMap` iteration in `PassiveStack::install`). Either is a stop-the-line bug.

---

## 10. Validation test #4 — Stochastic gating sanity

Per chassis Decision 7 sub-decision "Phase 1 test additions": one small test that proves `disable_stochastic()` zeroes the noise contribution end-to-end, before the FD/autograd code paths that depend on this property exist in Phases 5+.

The test pattern (note the Arc-clone-before-install dance — `PassiveStack::install` consumes the `Arc<Self>`, so a handle for the guard call must be retained separately):

```rust
#[test]
fn test_stochastic_gating_sanity() {
    let mut model = sim_mjcf::load_model(SHO_1D_XML).expect("load");
    let mut data  = model.make_data();

    // Build the stack and KEEP a clone before install consumes the original.
    let stack: Arc<PassiveStack> = PassiveStack::builder()
        .with(LangevinThermostat::new(
            DVector::from_element(model.nv, 0.1),
            1.0,
            0xC0FFEE_u64,
        ))
        .build();
    Arc::clone(&stack).install(&mut model);

    // Initial state: stretched spring at rest.
    data.qpos[0] = 1.0;
    data.qvel[0] = 0.0;

    // Disable noise for the decay phase.
    {
        let _guard = stack.disable_stochastic();
        // Damping time constant M/γ = 10 time units = 10_000 steps.
        // Run 5× that to land in numerical-floor territory.
        for _ in 0..50_000 { data.step(&model).expect("step"); }
        assert!(
            data.qvel[0].abs() < 1e-3 && data.qpos[0].abs() < 1e-3,
            "decayed state should be near rest, got qpos={}, qvel={}",
            data.qpos[0], data.qvel[0],
        );
    } // guard drops here, stochastic re-enabled

    // One step with noise re-enabled. Expected kick magnitude ≈ σ·h
    // = sqrt(2γkT/h)·h = sqrt(2·0.1·1·1000)·0.001 ≈ 0.014. We assert
    // a threshold an order of magnitude below the kick to remain robust
    // against single-sample variance while still catching "noise didn't
    // re-engage" failures.
    data.step(&model).expect("post-guard step");
    assert!(
        data.qvel[0].abs() > 1e-3,
        "guard drop should re-energize the system; got qvel={}",
        data.qvel[0],
    );
}
```

The post-decay assertion uses `< 1e-3` rather than `< 1e-6` because Euler integration of a damped harmonic oscillator at `h=0.001` after 50k steps lands in the `O(1e-4)`-to-`O(1e-3)` range, not at the f64 floor — `1e-6` would be too tight and risk a false negative on a correct implementation.

This is a smoke test, not a full FD validation. Phase 5 will exercise `disable_stochastic` under actual perturbation loops with derivative checks. Phase 1's job is to prove the chassis gating mechanism wires through `LangevinThermostat::apply` correctly.

---

## 11. Open contracts (what Phase 1 does NOT test)

Each of these is named explicitly so the spec's scope is unambiguous. None are blockers for Phase 1; each has a forward owner.

- **Q1 — constrained Langevin.** Forces written into `qfrc_passive` and projected by the constraint solver have a known-but-not-yet-quantified effective temperature on constrained DOFs. Reference: Lelièvre, Rousset, Stoltz, *Free Energy Computations*. **Gates Phase 2.** Phase 1's 1-DOF slide joint has zero constraints, so the question is invisible here. See [`open_questions.md`](../02_foundations/open_questions.md) Q1.
- **`install_per_env` correctness on parallel envs.** The chassis ships the API (Decision 3 + S3 → `EnvBatch { models, stacks }`) with a defensive `clear_passive_callback` + `debug_assert!` (N4) inside the loop. Phase 1 validates one env per trajectory in the §7.3 gate; per-env independence under parallel BatchSim execution is a Phase 2+ test (the natural place is alongside Phase 2's free-body equipartition test, which already needs multiple envs for statistics).
- **`Stochastic` gating under FD perturbation.** §10 is a smoke test only. The full validation — that an FD loop wrapped in `disable_stochastic()` recovers `∂F_det/∂qpos` to floating-point precision — is Phase 5 work. Decision 7 designed the chassis so this works by construction; Phase 5 will verify.
- **Composition with `cb_control` / `qfrc_applied` writers.** Recon log part 5 established that field disjointness makes composition automatic — the thermostat lives in `qfrc_passive`, ml-bridge `ActionSpace::apply` writes `qfrc_applied`/`xfrc_applied`/`ctrl`, and they cannot clobber each other. Phase 1 documents this in the crate-level rustdoc but does not exercise it with a test (no Phase 1 use case).
- **Plugin passive forces firing after `cb_passive`** (`forward/passive.rs:723-735`). The Phase 1 model has no plugins. If a future phase introduces a model with passive plugins, the spec for that phase is responsible for verifying the ordering interaction.
- **BAOAB / GJF / higher-accuracy Langevin schemes.** The chassis is integrator-agnostic; the upgrade path is "implement a new `PassiveComponent` (`BAOABThermostat`) and bolt it onto the same `PassiveStack`." Not Phase 1 work.
- **Q3 — `thrml-rs` / Phase 6 sampler bridge.** Resolved at the open-questions level (three options A/B/C named). Phase 6 spec will pick. No Phase 1 implication.
- **Q5 — cf-design end-to-end differentiability.** Resolved NO with three breaks (recon log part 13). Phase 1 lives entirely on the sim-core side of the dep graph and is unaffected.

---

## 12. Acceptance criteria

The Phase 1 implementation is "done" when **all** of the following hold:

1. `cargo build -p sim-thermostat` builds clean from a fresh `cargo clean` (no warnings, no `#[allow]` annotations introduced for Phase 1 code).
2. `cargo clippy -p sim-thermostat -- -D warnings` is clean.
3. `cargo fmt -p sim-thermostat -- --check` is clean.
4. `cargo test -p sim-thermostat` runs all four integration tests (§7, §8, §9, §10) plus per-module unit tests; **all green**.
   - §7 (equipartition gate, 100 trajectories) lands within `±3σ` of `½kT` on the central parameter set on the first run, with no flake on three back-to-back runs.
   - §7.4 (γ + T sweep) lands within `±3σ` on every combination, possibly under `--ignored` if total runtime exceeds the CI budget.
   - §8 (callback firing count) holds as **hard equality**.
   - §9 (reproducibility) holds as **hard f64 equality** on `qvel` and `qpos`.
   - §10 (Stochastic gating sanity) decays to `<1e-3` under the guard (per the Euler-at-h=0.001-after-50k-steps floor reasoning in §10's body — `1e-6` would risk a false negative on a correct implementation), re-energizes immediately on guard drop.
5. `cargo xtask grade sim-thermostat` reaches **A across all 7 criteria**. Anything less is stop-the-line per the project's `A-grade or it doesn't ship` rule.
6. `sim/L0/core/Cargo.toml` shows **no production deps** on `rand`, `rand_chacha`, `rand_distr`. The sim-core-stays-rand-free invariant from item 4 + item 8 must be verifiable by `grep`.
7. The crate-level `lib.rs` rustdoc renders cleanly (`cargo doc -p sim-thermostat`) and contains the four pieces from chassis Decision 6 sub-decisions: purpose paragraph, architecture summary, quick-start example, link to chassis_design.md and to this spec.

When all seven hold, the spec is satisfied. Then — and only then — the implementation review (§13) runs.

---

## 13. Implementation review (closing rigor)

After the validation gate is green, the closing step of this session is to **review the implementation against this spec end-to-end** and reconcile any drift in either direction:

- **Impl diverged from spec but spec is right** → fix the impl. Land as a final commit on the implementation branch.
- **Impl is right but spec was wrong** → fix the spec. The spec is a planning document; if implementation revealed a better path, the spec should reflect it before the branch lands. Land as a final commit on the spec.
- **Impl is right but a chassis decision broke** → **STOP**. Surface to the user. A chassis-level break is a bigger conversation than this session; the user decides whether to amend the chassis, the spec, or both.
- **Everything matched cleanly** → no commit needed; report the alignment.

Per the recon-to-iteration handoff principle (canonical statement in the auto-memory file `feedback_recon_to_iteration_handoff.md`, mirrored operationally in [`02_foundations/working_principles.md`](../02_foundations/working_principles.md)): the first compile error or test failure that contradicts a chassis decision is the *real* recon round. Be ready for it. Don't fix chassis-level issues in flight — surface them.

---

## 14. Phase 5+ caveats forwarded

Each is fully resolved at the chassis level; this section names them so the Phase 1 implementer can read forward and know what they don't have to think about.

- **FD perturbation under `cb_passive`** — RESOLVED at chassis level by Decision 7 (`Stochastic` trait + RAII `disable_stochastic()` guard). Phase 5 wraps its FD perturbation block in the guard; the perturbed and baseline runs both produce zero noise; the FD difference recovers `∂F_det/∂qpos` exactly for state-independent noise (which every roadmap component has).
- **Plugin passive forces firing after `cb_passive`** (`passive.rs:723-735`) — Phase 1 has no plugins. Documented for future phases that introduce them.
- **BatchSim parallel envs sharing `Arc<LangevinThermostat>`** — RESOLVED at chassis level by Decision 3 (`PassiveStack::install_per_env(prototype, n, build_one)` + defensive `clear_passive_callback` + `debug_assert!` per N4). Each env gets its own fresh callback by construction.

The rest of the chassis (Decisions 1, 2, 4, 5, 6) ships unchanged into Phase 1. The implementation's job is to make the chassis real.
