# D1 — Brownian Ratchet Motor: RL Harvests Thermal Noise

> **Status**: spec draft
> **Branch**: `feature/thermo-phase-2-3`
> **Depends on**: Phases 1-3 (Langevin thermostat, multi-DOF equipartition,
> bistable Kramers — all done), sim-ml-bridge (CEM, REINFORCE — all done)
> **Answers**: Q6 (partially — reward signal for the ratchet case)
> **Parent**: [`research_directions.md`](../01_vision/research_directions.md) D1

---

## 1. Goal

Build and validate the **first RL ↔ thermostat bridge**: an RL agent that
learns to harvest thermal fluctuations for directed transport using a
flashing Brownian ratchet.

Success means:
- A new `RatchetPotential` component in `sim-thermostat`
- A D1 integration test that wraps the ratchet as an `ml-bridge` `Environment`
- A trained CEM policy that produces statistically significant directed current
  from thermal noise alone
- Validation gates that separate the ratchet effect from diffusion

This is the first experiment in the thermo-computing line that produces
*both* a research result (RL discovers how to harvest thermal noise) and
a working bridge pattern that D2/D4/D5 will reuse.

## 2. Physics Background

### 2.1 The flashing ratchet

(Astumian 1994, Magnasco 1993, Reimann 2002 review)

A particle in an **asymmetric periodic potential** coupled to a thermal
bath at temperature T. When the potential is ON, the particle sits in a
local minimum. When the potential is OFF, the particle diffuses freely
(Brownian motion). Due to the spatial asymmetry of the potential, upon
re-activation the particle is statistically more likely to have diffused
into the basin of the *next* well in the preferred direction than the
*previous* well. Over many ON/OFF cycles, this rectifies thermal
fluctuations into directed transport — a Brownian motor.

This is how biology does it: kinesin walking on microtubules, ATP synthase
as a rotary ratchet, myosin in muscle contraction. The mechanism converts
chemical energy (which modulates the potential) into directed mechanical work.

### 2.2 Potential model: two-harmonic ratchet

```text
V(x) = α · [ -V₁ sin(2πx/L) - V₂ sin(4πx/L + φ) ]
```

- `V₁`: first harmonic amplitude (sets the overall potential scale)
- `V₂`: second harmonic amplitude (breaks symmetry when V₂ ≠ 0)
- `φ`: phase offset (controls asymmetry direction)
- `L`: spatial period
- `α ∈ [0, 1]`: amplitude multiplier (the RL agent's control)

When V₂ = 0, the potential is symmetric — no ratchet current possible
regardless of flashing strategy. This is the null-control baseline.

When V₂ > 0 and φ ≠ 0, nπ, the potential has asymmetric wells within each
period. The steeper side of each well acts as the "short slope" of the
sawtooth, and diffusing particles preferentially cross toward the gentle
side during the OFF phase.

Force:
```text
F(x) = α · [ V₁(2π/L) cos(2πx/L) + V₂(4π/L) cos(4πx/L + φ) ]
```

Smooth everywhere — no force discontinuities, no integrator issues.

### 2.3 Why this model

- **Standard in the literature** — enables comparison with published
  ratchet currents and Fokker-Planck solutions
- **Smooth** — avoids the piecewise-linear sawtooth's force discontinuities
  at the teeth, which would require integrator care
- **Parametrically tunable** — V₂/V₁ and φ independently control asymmetry
  magnitude and direction; L controls the spatial scale
- **Reuses existing infrastructure** — `PassiveComponent` trait,
  `PassiveStack`, `LangevinThermostat` all work unchanged

## 3. Bridge Architecture: the ctrl-channel pattern

D1 establishes the long-term interface between `sim-thermostat` and
`sim-ml-bridge`. These two crates never depend on each other. The bridge
is `sim-core::Data` — specifically, `data.ctrl`.

### 3.1 The pattern

PassiveComponents that want to be RL-controllable take an optional
`ctrl_idx` per tunable parameter. At `apply` time, they read
`data.ctrl[ctrl_idx]` as a modulation input. The MJCF model gets one
zero-gain actuator per controllable parameter, providing the ctrl slot
without applying any force.

```text
ml-bridge ActionSpace::apply(action)
        │
        ▼
   data.ctrl[0] = α        ← agent writes (standard ml-bridge path)
        │
        ▼
   data.step(&model)
        │
        ▼
   cb_passive fires
        │
        ▼
   RatchetPotential::apply  ← reads data.ctrl[0] (sim-thermostat)
```

No new traits. No new dependencies. No bridge crate. The two crates
communicate through a field that already exists for a different reason.

### 3.2 Progressive dissolution (D1 → D5)

The RL agent's control authority grows monotonically:

| Direction | ctrl slots | What the agent controls |
|-----------|-----------|-------------------------|
| D1 | `ctrl[0]` = α | One knob on one component |
| D2 | + `ctrl[1]` = kT multiplier | A physical parameter of the bath |
| D3 | + `ctrl[2..n]` = all params | Boundary between "physics" and "control" dissolves |
| D5 | `ctrl[0..N]` = energy landscape | Complete unification |

The architecture is invariant — just more ctrl slots. The distinction
between "what the physics engine controls" and "what the agent controls"
fades as more physical parameters become actions.

### 3.3 Two bridge modes

D1 uses **runtime RL** — the agent acts every decision step via ctrl.
D4/D5 will use **offline parameter optimization** — the optimizer holds
`Arc<Component>` references and calls setters between sampling rounds
(the existing `IsingLearner` pattern from Phase 5). Both modes coexist;
they are not competing architectures.

### 3.4 Forward design: `with_ctrl_temperature`

D1 does NOT implement this. But the `LangevinThermostat` API is designed
so D2 can add it without restructuring:

```rust
// D1: temperature is fixed at construction
LangevinThermostat::new(gamma, k_b_t, seed)

// D2: temperature modulated at runtime via ctrl slot
LangevinThermostat::new(gamma, k_b_t, seed)
    .with_ctrl_temperature(ctrl_idx)  // ← new builder method, backward-compatible
```

In `apply`, when `k_b_t_ctrl` is `Some(idx)`:
```rust
let k_b_t = self.k_b_t * data.ctrl[idx].clamp(0.0, 10.0);
```

The same pattern extends to `PairwiseCoupling` (`.with_ctrl_coupling`),
`ExternalField` (`.with_ctrl_field`), etc. Each component gains ctrl
modulation independently, on its own schedule, without cross-component
coordination.

### 3.5 What we explicitly defer

- **No `CtrlPort` trait or auto-discovery** — one data point (D1) isn't
  enough to abstract. Revisit after D2 validates the pattern.
- **No `sim-thermo-rl` crate** — the test-file bridge is sufficient
  until D3 forces the question.
- **No `ThermoEnv` struct** — ad-hoc environment construction in
  tests/examples is the right commitment level.

Canonical record: [`project_thermo_rl_bridge_architecture.md`](memory
file) has the full architectural reasoning.

## 4. New component: `RatchetPotential`

**Location**: `sim-thermostat/src/ratchet.rs`

A `PassiveComponent` that implements the two-harmonic ratchet potential
with ctrl-dependent amplitude modulation.

### 4.1 API

```rust
pub struct RatchetPotential {
    v1: f64,          // first harmonic amplitude
    v2: f64,          // second harmonic amplitude
    phi: f64,         // phase offset (radians)
    period: f64,      // spatial period L
    dof: usize,       // DOF index (qpos/qvel index for slide joints)
    ctrl_idx: usize,  // index into data.ctrl for amplitude modulation
}

impl RatchetPotential {
    pub fn new(v1: f64, v2: f64, phi: f64, period: f64, dof: usize, ctrl_idx: usize) -> Self;

    /// Potential energy at position x with amplitude α.
    pub fn potential(&self, x: f64, alpha: f64) -> f64;

    /// Theoretical ratchet current for square-wave flashing at frequency f.
    /// Uses the two-state Fokker-Planck solution (Reimann eq. 56).
    /// This is for validation — comparing measured current against theory.
    /// (Phase D1c, may be deferred if the analytical solution is too involved.)
    // pub fn theoretical_current(&self, freq: f64, gamma: f64, k_b_t: f64) -> f64;
}
```

### 4.2 `PassiveComponent` implementation

```rust
impl PassiveComponent for RatchetPotential {
    fn apply(&self, _model: &Model, data: &Data, qfrc_out: &mut DVector<f64>) {
        let x = data.qpos[self.dof];
        let alpha = data.ctrl[self.ctrl_idx].clamp(0.0, 1.0);

        let k1 = 2.0 * PI / self.period;
        let k2 = 4.0 * PI / self.period;

        // F(x) = -dV/dx = α · [V₁·k₁·cos(k₁·x) + V₂·k₂·cos(k₂·x + φ)]
        let force = alpha * (
            self.v1 * k1 * (k1 * x).cos()
            + self.v2 * k2 * (k2 * x + self.phi).cos()
        );
        qfrc_out[self.dof] += force;
    }
}
```

The component is **deterministic** — it does not implement `Stochastic`.
The stochastic element is the `LangevinThermostat` in the same stack.

### 4.3 `Diagnose` implementation

```rust
fn diagnostic_summary(&self) -> String {
    format!(
        "RatchetPotential(V1={:.4}, V2={:.4}, phi={:.4}, L={:.4}, dof={}, ctrl={})",
        self.v1, self.v2, self.phi, self.period, self.dof, self.ctrl_idx,
    )
}
```

### 4.4 Unit tests

1. **Force = -dV/dx** — finite-difference check against `potential()` at
   several x values, for both α=0 and α=1
2. **Zero force when α=0** — potential off produces no force regardless of x
3. **Symmetry breaking** — with V₂ > 0, φ ≠ 0: the force averaged over one
   period is zero (conservative force), but the potential minima are
   asymmetrically placed
4. **Symmetric case** — with V₂ = 0: potential is symmetric, minima at
   x = L/4 + nL
5. **Diagnostic summary format**

## 5. MJCF Model

Minimal 1-DOF particle on a frictionless slide rail with a zero-gain
actuator providing a ctrl slot:

```xml
<mujoco model="brownian-ratchet">
  <option timestep="0.001" gravity="0 0 0" integrator="Euler">
    <flag contact="disable"/>
  </option>
  <worldbody>
    <body name="particle">
      <joint name="x" type="slide" axis="1 0 0" damping="0"/>
      <geom type="sphere" size="0.05" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <general name="ratchet_ctrl" joint="x" gainprm="0" biasprm="0 0 0"
             ctrllimited="true" ctrlrange="0 1"/>
  </actuator>
</mujoco>
```

**Design notes**:
- `damping="0"`: the thermostat owns damping (Q4 resolution)
- `gravity="0 0 0"`: 1-DOF slide, gravity along x would bias motion
- `integrator="Euler"`: required for Langevin (Euler-Maruyama discretization)
- `contact="disable"`: no collision geometry needed
- `gainprm="0" biasprm="0 0 0"`: the actuator applies zero force — it exists
  solely to provide `data.ctrl[0]` as a communication channel from the RL
  agent to the `RatchetPotential`

**Ctrl flow** (verified against `forward/mod.rs:527-535`):
1. `ActionSpace::apply` writes `data.ctrl[0]` (before sub-stepping)
2. Each sub-step calls `data.step()` → `forward_acc()`:
   - `mj_fwd_actuation` reads ctrl, computes zero force (gain=0)
   - `mj_fwd_passive` → `cb_passive` → `RatchetPotential::apply` reads
     `data.ctrl[0]` (still intact — actuation reads but does not clear ctrl)
3. ctrl persists across all sub-steps until the next `ActionSpace::apply`

## 6. PassiveStack Configuration

```rust
let thermostat = LangevinThermostat::new(
    DVector::from_element(model.nv, gamma),
    k_b_t,
    seed,
);
let ratchet = RatchetPotential::new(v1, v2, phi, period, 0, 0);

PassiveStack::builder()
    .with(thermostat)
    .with(ratchet)
    .build()
    .install(&mut model);
```

The stack composes the Langevin noise + damping with the ratchet potential.
No new infrastructure needed — the Phase 1-3 chassis handles this.

## 7. RL Environment Design

### 7.1 Observation space

| Index | Field   | Description             | Scale factor       |
|-------|---------|-------------------------|---------------------|
| 0     | qpos[0] | Particle position x     | 1/L (normalize by period) |
| 1     | qvel[0] | Particle velocity v     | 1/v_th where v_th = √(kT/M) |

Observation dimension: **2**.

The agent needs position to know where the particle is in the ratchet
landscape, and velocity to read the thermal fluctuation state.

### 7.2 Action space

| Index | Field    | Range  | Description                |
|-------|----------|--------|----------------------------|
| 0     | ctrl[0]  | [0, 1] | Ratchet potential amplitude |

Action dimension: **1**.

Continuous [0, 1]. The potential interpolates smoothly between OFF (0)
and ON (1). CEM will naturally discover that near-binary switching is
optimal, but the continuous parameterization is more general.

### 7.3 Reward

```text
reward = -data.qvel[0]
```

Negated instantaneous velocity. Stateless — no tracking of previous
position needed.

**Why negated?** The ratchet drift direction is a physical property of
the potential, determined by (V₁, V₂, φ). D1b measured the drift
direction as −x for φ = π/4 (random baseline mean displacement =
−2.15 ± 1.69). CEM maximizes fitness (`cem.rs:187` sorts descending).
Without negation, CEM would converge to zero-velocity policies (always-ON
or always-OFF, fitness ≈ 0) that beat the optimal ratchet (negative
velocity, fitness < 0). Negation aligns the reward with the physics:
positive reward = stronger drift in the ratchet direction.

**Why not change φ instead?** That would rearrange the physics to fit a
coordinate convention — "physics correct first, ergonomic second"
inverted. The drift direction is not a bug; it's a measured property.

**Why not `(x_new - x_old) / Δt`?** The reward closure signature is
`Fn(&Model, &Data) -> f64` — stateless, shared across VecEnv envs.
There's no `x_old` to reference without interior-mutability hacks.

**Why velocity works**: CEM computes fitness as `Σ rewards / episode_length`
(`cem.rs:182`). With all episodes the same length (1000 steps — see §7.4),
this is proportional to the sum:

```text
fitness = -(1/N) · Σ_t qvel[0]_t ≈ -Δx / T_episode
```

CEM's fitness is the **negated mean velocity** = ratchet current
magnitude. It ranks policies by current strength, which is exactly the
optimization target.

**Load-bearing invariant**: `done = never` (§7.4) ensures all episodes
have equal length. If episodes could terminate early, CEM's
`sum / length` normalization would distort rankings (a short lucky
episode would score disproportionately high). The fixed-length design
is a correctness requirement, not just a physics choice.

**Noise**: per-step velocity is thermally noisy (σ_v = √(kT/M) = 1.0).
Over 1000 steps the noise averages out: σ_fitness ≈ σ_v / √1000 ≈ 0.03.
The ratchet signal (net current) should be detectable above this floor.

**D1d note**: REINFORCE uses per-step rewards for gradient estimation,
where the thermal noise will increase gradient variance. If D1d is
attempted, a variance-reduced reward (e.g., a running-average baseline)
may be needed. This is a D1d-specific concern, not a D1c concern.

### 7.4 Episode structure

| Parameter       | Value    | Rationale                                |
|-----------------|----------|------------------------------------------|
| Physics timestep h | 0.001 | Same as Phase 1-3                        |
| Sub-steps/decision | 100   | Decision interval 0.1 t.u. ≫ τ_relax   |
| Episode length  | 1000 decisions | 100 t.u. total ≫ τ_escape             |
| Done condition  | never    | No terminal state for a Brownian motor   |
| Truncated       | at step 1000 | Fixed-length episodes                 |

### 7.5 On-reset

Each episode resets qpos[0] = 0.0 (particle starts at origin). The
thermal state (velocity) is zeroed by `Data::reset` and the thermostat
will thermalize it within ~τ_relax ≈ 0.025 t.u. (2.5 decision steps).
No domain randomization needed for Phase D1a.

**ctrl zeroing**: `Data::reset()` calls `ctrl.fill(0.0)` (`data.rs:1073`),
so the ratchet starts in the OFF state (α = 0) at the beginning of each
episode. The first `ActionSpace::apply` (before the first sub-step) sets
ctrl to the agent's chosen α. This means one decision interval of free
diffusion at episode start — acceptable, and identical across all policies
(does not distort CEM rankings).

## 8. Central Parameter Set

| Parameter | Symbol | Value | Rationale |
|-----------|--------|-------|-----------|
| Mass      | M      | 1.0   | Unit mass |
| Damping   | γ      | 1.0   | Moderate; spatial-diffusion regime |
| Temperature | kT   | 1.0   | Unit thermal energy |
| First harmonic | V₁ | 1.0 | Sets potential scale |
| Second harmonic | V₂ | 0.25 | Moderate asymmetry (V₂/V₁ = 0.25) |
| Phase offset | φ   | π/4   | Breaks symmetry; neither maximal nor minimal |
| Period    | L      | 1.0   | Unit period |
| Timestep  | h      | 0.001 | Euler-Maruyama, same as Phase 1 |

**Derived time scales** (sanity check):

| Time scale | Formula | Value | Interpretation |
|------------|---------|-------|----------------|
| τ_relax | γ / V₁(2π/L)² | ~0.025 | In-well relaxation |
| τ_decision | sub_steps × h | 0.1 | Agent acts every 0.1 t.u. |
| τ_diffusion | L²/(2D), D=kT/(Mγ) | 0.5 | Free diffusion across one period |
| τ_escape | ~1/k_Kramers | ~12.5 | Spontaneous escape (potential ON) |
| T_episode | 1000 × 0.1 | 100 | Total episode time |

Hierarchy: τ_relax ≪ τ_decision ≪ τ_diffusion ≪ τ_escape ≪ T_episode.

This ensures:
- The particle thermalizes between decisions (agent sees equilibrated state)
- Free diffusion covers a meaningful fraction of L per decision interval
- Spontaneous escape is rare enough that the agent's flashing strategy
  matters (only ~8 escapes per episode with potential always ON)
- Episodes are long enough to measure steady-state current

## 9. Baselines

### 9.1 Always-ON (α = 1.0)

Particle in a periodic potential at thermal equilibrium. Despite the
asymmetry within each period, the steady-state current is exactly zero
by **detailed balance**: a periodic potential with time-independent
parameters satisfies detailed balance, and detailed balance implies zero
net current. The asymmetry shapes the probability distribution (the
Boltzmann weight p(x) ∝ exp(−V(x)/kT) is non-uniform) but cannot
produce directed transport without broken time-reversal symmetry — which
requires the flashing. Expected mean displacement ≈ 0.

### 9.2 Always-OFF (α = 0.0)

Free Brownian motion. Diffusion with D = kT/(Mγ) = 1.0. Expected
displacement = 0 (diffusion is unbiased). RMS displacement grows as
√(2D·t) — provides the null distribution for statistical tests.

### 9.3 Random switching

Each decision step, α ~ Uniform[0, 1]. This partially activates the
ratchet mechanism — the asymmetric potential occasionally captures
particles that diffused in the preferred direction. Expected to show
small but positive net current.

### 9.4 Optimal square-wave sweep (the "cheating" baseline)

Grid search over switching period T_cycle ∈ [0.2, 5.0] with 50% duty
cycle (ON for T_cycle/2, OFF for T_cycle/2). Run each frequency for
100 episodes, measure mean current. The peak current from this sweep
is the open-loop optimal — the trained closed-loop agent should
approach or exceed this.

## 10. Training Plan

### Phase D1a — RatchetPotential component

**Deliverable**: `sim-thermostat/src/ratchet.rs` with unit tests.

1. Implement `RatchetPotential` per §4
2. Unit tests per §4.4
3. Integration test: install ratchet + thermostat on MJCF model,
   step 10,000 times with α=1, verify particle oscillates in wells

**Gate**: all unit tests pass, integration test shows bounded motion.

### Phase D1b — Baselines

**Deliverable**: `sim-thermostat/tests/d1_brownian_ratchet.rs` integration
test with baseline measurements.

1. Add `sim-ml-bridge` as a dev-dependency of `sim-thermostat`
2. Build the RL environment per §7 (construct model, install PassiveStack,
   build `SimEnv`, wrap in `VecEnv`)
3. Run always-ON baseline (50 episodes) → measure mean displacement
4. Run always-OFF baseline (50 episodes) → measure mean displacement
5. Run random baseline (50 episodes) → measure mean displacement
6. Verify: always-ON and always-OFF have mean displacement consistent
   with zero (within 3σ of sampling error)

**Gate**: baselines produce expected behavior — zero net current for
always-ON and always-OFF; small but nonzero for random.

**D1b finding (2026-04-10)**: with φ = π/4, the ratchet drifts in −x
(random baseline mean displacement = −2.15 ± 1.69). The reward is
negated (`-qvel[0]`, see §7.3) so CEM sees positive fitness for
leftward drift and maximizes it correctly. The D1c gates below use
the negated reward convention.

### Phase D1c — CEM training

**Deliverable**: CEM trains a policy that produces directed current.

1. Build `TaskConfig` (or construct `VecEnv` directly)
2. Train CEM with `LinearPolicy(obs_dim=2, act_dim=1)`
3. Hyperparameters:
   - Population: 32 envs
   - Elite fraction: 0.2
   - Noise std: 0.5, decay: 0.99, min: 0.05
   - Max episode steps: 1000
   - Budget: 100 epochs
4. Evaluate trained policy: 50 episodes, measure mean current
5. Compare against baselines

**Gate A** (ratchet effect): trained policy mean fitness > 0 with
p < 0.01 (one-sample t-test against zero). With `reward = -qvel[0]`,
positive fitness = net displacement in the ratchet direction (−x).

**Gate B** (learning): mean fitness improves over training
(best-of-last-10 > epoch 0 fitness).

**Gate C** (baseline separation): trained policy mean fitness > 2×
random-baseline mean fitness.

### Phase D1d — REINFORCE comparison (stretch)

Same environment, train with REINFORCE. Compare learning curves and
final performance against CEM. This phase is optional for the initial
D1 deliverable but informs the D2/D4 algorithm selection.

## 11. Dependency Routing

```
sim-thermostat (library)
  ├── RatchetPotential (new, §4)
  ├── LangevinThermostat (existing)
  ├── PassiveStack (existing)
  └── [dev-dependencies]
       └── sim-ml-bridge (new dev-dep for D1 integration tests)

sim-ml-bridge (library, unchanged)
  ├── Environment trait
  ├── SimEnv / VecEnv
  ├── CEM / REINFORCE
  └── TaskConfig
```

The D1 integration test lives in `sim-thermostat/tests/d1_brownian_ratchet.rs`.
It imports both `sim_thermostat` (library) and `sim_ml_bridge` (dev-dep) to
bridge the two.

**Why this routing**: `RatchetPotential` is a `PassiveComponent` — it belongs
in `sim-thermostat`. The bridge code is test-only for now. If D2/D4 need a
reusable bridge, we either:
- (a) promote the bridge to a `sim-thermo-rl` sibling crate, or
- (b) add `sim-thermostat` as an optional dep of `sim-ml-bridge` behind a
  feature flag

That decision is deferred to D2 — D1 validates the pattern first.

## 12. Validation and Measurement

### 12.1 Statistical protocol

All current measurements use 50 independent episodes with different RNG
seeds. Report mean ± standard error. Statistical significance via
one-sample t-test (for current ≠ 0) and two-sample t-test (for
baseline comparisons).

### 12.2 Diffusion calibration check

Before training, verify the free-diffusion regime: run 50 episodes with
α = 0, measure RMS displacement at t = 100. Expected: √(2Dt) = √(200)
≈ 14.14. Accept if within ±15% (accounting for finite-episode sampling).
This confirms the Langevin thermostat is producing correct diffusion in
the ratchet context.

### 12.3 Current measurement

Net current J = ⟨Δx⟩ / T_episode, where Δx = x(T) - x(0) and T_episode =
100 t.u. Reported in units of L/t.u. (periods per time unit).

### 12.4 Release mode

All tests with multi-episode rollouts must use `cargo test --release`
(per feedback: debug mode is 100×+ slower for Langevin trajectories).
Each episode is 100,000 physics steps; 50 episodes × 32 envs × 100
epochs = 160M steps for CEM training. Release mode is non-negotiable.

## 13. Q6 Partial Resolution

Q6 asks: "What's the right reward signal for Phase 7?"

D1 answers for the **ratchet case**: reward = net displacement per
decision interval. This directly measures the quantity the Brownian motor
optimizes (directed current). It is dense (every step), physically
meaningful (velocity in preferred direction), and does not require
knowledge of the target distribution.

Q6 remains open for:
- D2 (stochastic resonance): reward = mutual information between input
  signal and observed switching
- General sampler case: reward = sample quality (ESS, KL, autocorrelation)

## 14. Forward References

- **D2 bridge pattern**: D1 establishes the `PassiveComponent reads ctrl`
  pattern for RL control of physical parameters. D2 will use the same
  pattern — the agent controls noise level (thermostat kT or coupling
  stiffness) via ctrl.
- **Reusable histogram measurement**: if D1 needs spin-state histograms
  for diagnostics, that triggers the extraction of
  `measure_langevin_histogram()` deferred from Phase 6 §11.1.
- **VecEnv shared-RNG caveat**: VecEnv wraps a single `Arc<Model>`,
  so all 32 envs share one `LangevinThermostat` and its `Mutex<ChaCha8Rng>`.
  Noise draws are interleaved across envs but each draw is still i.i.d.
  `N(0,1)` with correct `σ` — equipartition holds per-env. CEM doesn't
  need cross-env independence (each env evaluates a different policy
  perturbation). Sequential stepping (the default) has no Mutex
  contention. **Accepted for D1.** If D2+ needs independent per-env
  noise streams, the fix is to either (a) bridge `install_per_env`'s
  `EnvBatch` into VecEnv's interface, or (b) modify VecEnv to accept
  per-env Models. The `install_per_env` factory (chassis Decision 3)
  already creates independent thermostat instances — the gap is only
  in the VecEnv adapter layer.

## 15. What D1 Does NOT Do

- **No custom integrator** — Euler-Maruyama via the existing Langevin
  thermostat. No BAOAB or GJF schemes.
- **No multi-particle ratchet** — single 1-DOF particle. Coupled ratchets
  are D5 territory.
- **No energy accounting** — we measure displacement, not thermodynamic
  efficiency. Efficiency measurement requires careful heat/work
  decomposition that is out of scope (D5 territory).
- **No Bevy visualization** — the test is headless. Visual Bevy example
  of the Brownian motor is a follow-up after the gates pass.
- **No analytical Fokker-Planck solution** — the theoretical_current()
  method is commented out in §4.1. The validation is empirical (baselines
  + statistical tests), not analytical. Analytical comparison is a
  possible Phase D1e extension.
