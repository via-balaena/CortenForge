# D2 — Stochastic Resonance: RL Discovers the Optimal Noise Level

> **Status**: spec draft
> **Branch**: `feature/thermo-phase-2-3`
> **Depends on**: Phases 1-3 (Langevin thermostat, multi-DOF equipartition,
> bistable Kramers — all done), D1 (ctrl-channel bridge pattern — done),
> sim-ml-bridge (CEM — done)
> **Answers**: Q6 (partially — reward signal for the SR case)
> **Parent**: [`research_directions.md`](../01_vision/research_directions.md) D2

---

## 1. Goal

Build and validate the **second RL ↔ thermostat bridge experiment**: an RL
agent that discovers the stochastic resonance (SR) peak by learning to set
the bath temperature to maximize signal detection in a noisy bistable system.

Success means:
- A new `OscillatingField` component in `sim-thermostat`
- `LangevinThermostat::with_ctrl_temperature(ctrl_idx)` — the first time
  a *physical parameter* of the bath becomes an RL action (D1 spec §3.4
  forward design, now executed)
- A D2 integration test that wraps the SR system as an `ml-bridge`
  `Environment`
- A trained CEM policy that converges to a temperature near the analytical
  SR peak
- Validation gates that separate SR-optimal behavior from fixed-temperature
  baselines

This is the experiment where the boundary between "physics parameters" and
"agent actions" begins to dissolve. D1 controlled a *component* parameter
(ratchet amplitude). D2 controls a *bath* parameter (temperature). The ctrl
channel is the same; the conceptual shift is that the agent now modulates
the noise source itself.

## 2. Physics Background

### 2.1 Stochastic resonance

(Benzi 1981, Gammaitoni et al. 1998 review, McNamara & Wiesenfeld 1989
two-state theory)

A particle in a symmetric double-well potential driven by:
1. A **sub-threshold periodic signal** — too weak to push the particle over
   the barrier deterministically
2. **Thermal noise** from the Langevin thermostat

At low noise: switching between wells is too rare. The particle stays in one
well regardless of the signal. Poor signal detection.

At high noise: switching is so frequent that it is uncorrelated with the
signal. The signal is drowned in noise. Poor signal detection.

At the **SR peak**: the noise-assisted escape rate matches the signal
frequency. Switching events *synchronize* with the signal — the particle
hops to the "correct" well at the right time. Signal detection is maximized.

This is counter-intuitive: adding noise *improves* signal detection. Biology
uses this — mechanosensory neurons in crayfish detect sub-threshold water
currents via SR; the human auditory system exploits SR in cochlear hair cells.

### 2.2 Two-state model

The McNamara–Wiesenfeld (1989) two-state theory approximates the bistable
system as a telegraph process with transition rates modulated by the signal:

```text
k_±(t) = k₀ · exp(∓ A₀·x₀ · cos(ωt) / kT)
```

where `k₀` is the zero-field Kramers rate, `A₀` is the signal amplitude,
`x₀` is the well separation, and `±` refers to the two transition
directions. The SR peak occurs approximately where the mean residence time
in one well matches half the signal period:

```text
1/k₀(T*) ≈ T_signal / 2    ⟹    k₀(T*) ≈ 2·f_signal
```

i.e., the particle escapes each well roughly once per half-cycle, so its
state tracks the signal. Below this temperature, escapes are too rare to
track the signal. Above it, escapes are too frequent and decouple from
the signal.

**Caveat**: the exact matching factor depends on the metric. The
Gammaitoni time-domain condition gives `k₀ ≈ 2f`; the McNamara–Wiesenfeld
spectral SNR peaks closer to `k₀ ≈ πf`. For the time-domain synchrony
metric D2 uses (§8.3), the peak location is determined empirically in
D2b — the formula above is a design guide for placing the peak in the
right neighborhood, not a precision prediction.

### 2.3 Potential and signal model

**Double-well potential** (reusing Phase 3 `DoubleWellPotential`):

```text
V(x) = a(x² − x₀²)²,    a = ΔV / x₀⁴
```

Minima at `±x₀`, barrier of height `ΔV` at `x = 0`.

**Sub-threshold periodic signal** (new `OscillatingField`):

```text
F_signal(t) = A₀ · cos(ω·t)
```

The signal is a force (not a potential modulation). It tilts the double-well
energy landscape periodically, lowering one barrier and raising the other.
The critical force for deterministic barrier removal is:

```text
F_c = 8ΔV / (3√3 · x₀)
```

The signal must satisfy `A₀ ≪ F_c` (sub-threshold condition).

### 2.4 Why this model

- **Standard in the SR literature** — enables direct comparison with
  published SNR curves and the two-state theory
- **Reuses existing validated infrastructure** — `DoubleWellPotential`
  (Phase 3, Kramers-validated), `LangevinThermostat` (Phase 1,
  equipartition-validated), `PassiveStack`, CEM
- **Clean separation of signal and noise** — the signal enters as a
  deterministic force (OscillatingField); the noise enters through the
  thermostat. The agent controls only the noise level.
- **1-DOF** — same complexity as D1. The physics is rich (SR is a
  many-textbook topic) without requiring multi-body dynamics.

## 3. Bridge Architecture: `with_ctrl_temperature`

D2 uses the same ctrl-channel pattern as D1 (spec §3), now applied to the
`LangevinThermostat` itself.

### 3.1 The pattern (unchanged from D1)

```text
ml-bridge ActionSpace::apply(action)
        │
        ▼
   data.ctrl[0] = kT_mult      ← agent writes (standard ml-bridge path)
        │
        ▼
   data.step(&model)
        │
        ▼
   cb_passive fires
        │
        ▼
   LangevinThermostat::apply    ← reads data.ctrl[0], scales kT
   OscillatingField::apply      ← reads data.time, applies signal force
   DoubleWellPotential::apply   ← deterministic conservative force
```

### 3.2 `with_ctrl_temperature` API

The D1 spec (§3.4) forward-designed this interface:

```rust
// Before D2: temperature fixed at construction
LangevinThermostat::new(gamma, k_b_t, seed)

// D2: temperature modulated at runtime via ctrl slot
LangevinThermostat::new(gamma, k_b_t, seed)
    .with_ctrl_temperature(ctrl_idx)  // new builder method
```

In `apply`, when `k_b_t_ctrl` is `Some(idx)`:
```rust
let k_b_t = self.k_b_t * data.ctrl[idx].clamp(0.0, 10.0);
```

The `clamp(0.0, 10.0)` bounds the multiplier. At 0.0, the thermostat
produces pure damping (no noise). At 10.0, the effective temperature is
10× the base. The base `k_b_t` (construction parameter) sets the scale;
the ctrl value is a *multiplier*, not an absolute temperature.

**Backward compatibility**: `with_ctrl_temperature` is opt-in. Without it,
`k_b_t_ctrl` is `None` and `apply` uses `self.k_b_t` directly — identical
to the current behavior. No change to existing code paths.

### 3.3 The noise scaling

When kT varies, the FDT noise amplitude must track it:

```text
σ² = 2·γ·kT_eff / h
```

where `kT_eff = k_b_t * ctrl_multiplier`. The `apply` method already
computes `sigma = sqrt(2 * gamma_i * self.k_b_t / h)` per DOF. With
ctrl temperature, this becomes `sqrt(2 * gamma_i * k_b_t_eff / h)` — a
one-line change inside the existing noise loop.

### 3.4 What D2 adds to the progressive dissolution table

| Direction | ctrl slots | What the agent controls |
|-----------|-----------|-------------------------|
| D1 | `ctrl[0]` = α | One knob on one component |
| **D2** | **`ctrl[0]` = kT multiplier** | **A physical parameter of the bath** |
| D3 | + `ctrl[1..n]` = all params | Boundary fully dissolved |

D2 is the conceptual inflection point: the agent is no longer modulating
a synthetic parameter (ratchet amplitude) but a *thermodynamic quantity*
(the temperature of the heat bath). The same `data.ctrl` channel; a
categorically different kind of control.

## 4. New Component: `OscillatingField`

**Location**: `sim-thermostat/src/oscillating_field.rs`

A `PassiveComponent` that applies a sinusoidal force `F(t) = A₀ cos(ωt + φ₀)`
to a single DOF. Reads `data.time` for the current simulation time.

### 4.1 API

```rust
pub struct OscillatingField {
    amplitude: f64,    // A₀ — signal amplitude
    omega: f64,        // ω — angular frequency (rad / time unit)
    phase: f64,        // φ₀ — initial phase offset (radians)
    dof: usize,        // DOF index (qpos/qvel index for slide joints)
}

impl OscillatingField {
    pub fn new(amplitude: f64, omega: f64, phase: f64, dof: usize) -> Self;

    /// Angular frequency ω.
    pub fn omega(&self) -> f64;

    /// Signal period T = 2π/ω.
    pub fn period(&self) -> f64;

    /// Signal frequency f = ω/(2π).
    pub fn frequency(&self) -> f64;

    /// Signal value at time t: A₀ · cos(ωt + φ₀).
    /// Exposed for reward computation — the reward closure needs the
    /// signal phase to compute synchrony.
    pub fn signal_value(&self, t: f64) -> f64;

    /// Normalized signal at time t: cos(ωt + φ₀) (unit amplitude).
    /// For synchrony reward: reward = sign(x) · normalized_signal(t).
    pub fn normalized_signal(&self, t: f64) -> f64;
}
```

### 4.2 `PassiveComponent` implementation

```rust
impl PassiveComponent for OscillatingField {
    fn apply(&self, _model: &Model, data: &Data, qfrc_out: &mut DVector<f64>) {
        let t = data.time;
        qfrc_out[self.dof] += self.amplitude * (self.omega * t + self.phase).cos();
    }
}
```

Deterministic — does not implement `Stochastic`. The signal is external
and unaffected by the stochastic gating mechanism (Decision 7).

### 4.3 `Diagnose` implementation

```rust
fn diagnostic_summary(&self) -> String {
    format!(
        "OscillatingField(A₀={:.4}, ω={:.4}, φ₀={:.4}, T={:.4}, dof={})",
        self.amplitude, self.omega, self.phase, self.period(), self.dof,
    )
}
```

### 4.4 Unit tests

1. **Force at t=0** — `F(0) = A₀ · cos(φ₀)` for various phase offsets
2. **Force = −dV/dx** — not applicable (time-dependent force, not
   conservative). Instead: verify the force is correct at several time
   points against the analytical formula.
3. **Signal periodicity** — `signal_value(t) == signal_value(t + T)` for
   several t values
4. **Zero force when amplitude zero** — `OscillatingField::new(0.0, ...)`
   produces zero force at all times
5. **`normalized_signal` is unit amplitude** — bounded in `[-1, 1]`
6. **Diagnostic summary format**

## 5. LangevinThermostat Modification

### 5.1 Structural change

Add one field to `LangevinThermostat`:

```rust
pub struct LangevinThermostat {
    gamma: DVector<f64>,
    k_b_t: f64,
    seed: u64,
    rng: Mutex<ChaCha8Rng>,
    stochastic_active: AtomicBool,
    k_b_t_ctrl: Option<usize>,  // ← NEW: ctrl index for temperature modulation
}
```

### 5.2 Builder method

```rust
impl LangevinThermostat {
    /// Enable runtime temperature modulation via ctrl channel.
    ///
    /// When set, `apply` reads `data.ctrl[ctrl_idx]` as a multiplier
    /// on the base `k_b_t`. The effective temperature is
    /// `k_b_t * ctrl.clamp(0.0, 10.0)`.
    ///
    /// This is the D2 forward design from D1 spec §3.4.
    #[must_use]
    pub fn with_ctrl_temperature(mut self, ctrl_idx: usize) -> Self {
        self.k_b_t_ctrl = Some(ctrl_idx);
        self
    }
}
```

### 5.3 `apply` modification

```rust
fn apply(&self, model: &Model, data: &Data, qfrc_out: &mut DVector<f64>) {
    let h = model.timestep;
    let active = self.stochastic_active.load(Ordering::Relaxed);
    let n_dofs = self.gamma.len();

    // ── Effective temperature ──────────────────────────────────────────
    let k_b_t = match self.k_b_t_ctrl {
        Some(idx) => self.k_b_t * data.ctrl[idx].clamp(0.0, 10.0),
        None => self.k_b_t,
    };

    // ── Damping (unconditional) ────────────────────────────────────────
    for i in 0..n_dofs {
        qfrc_out[i] += -self.gamma[i] * data.qvel[i];
    }

    if !active {
        return;
    }

    // ── FDT-paired noise (uses k_b_t, not self.k_b_t) ─────────────────
    let mut rng = self.rng.lock().unwrap_or_else(PoisonError::into_inner);
    for i in 0..n_dofs {
        let gamma_i = self.gamma[i];
        let sigma = (2.0 * gamma_i * k_b_t / h).sqrt();
        let z: f64 = StandardNormal.sample(&mut *rng);
        qfrc_out[i] += sigma * z;
    }
}
```

**Note**: damping is NOT affected by the temperature multiplier. Damping
`−γ·v` is a property of the coupling to the bath, not the bath temperature.
Only the noise amplitude `σ = √(2γkT/h)` changes. This preserves the FDT
relation at the effective temperature.

### 5.4 Backward compatibility

- `new()` sets `k_b_t_ctrl: None`. The `match` in `apply` takes the `None`
  arm, returning `self.k_b_t` — identical to the current code path.
- `diagnostic_summary` should report the ctrl index when present.
- All existing tests pass unchanged (they don't call `with_ctrl_temperature`).

### 5.5 New unit tests for `with_ctrl_temperature`

1. **Default is None** — `LangevinThermostat::new(...)` has no ctrl
   temperature. `apply` uses `self.k_b_t`.
2. **Builder method sets ctrl index** — `with_ctrl_temperature(0)` wires
   the ctrl channel. Verify via a diagnostic summary string check.
3. **Effective kT scales with ctrl** — set `data.ctrl[0] = 2.0`, run apply
   with stochastic disabled. Damping force should be unchanged (−γ·v).
   Re-enable stochastic: noise amplitude should be √2× the base noise
   (since kT_eff = 2·kT_base, σ ∝ √kT).
4. **Ctrl clamping** — negative ctrl → effective kT = 0 (no noise).
   ctrl > 10 → capped at 10× base.
5. **Zero ctrl = zero noise** — `data.ctrl[0] = 0.0` produces pure damping,
   identical to stochastic-inactive mode (except the RNG advances).

## 6. MJCF Model

Minimal 1-DOF particle with one zero-gain actuator for the temperature
ctrl channel:

```xml
<mujoco model="stochastic-resonance">
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
    <general name="temp_ctrl" joint="x" gainprm="0" biasprm="0 0 0"
             ctrllimited="true" ctrlrange="0 10"/>
  </actuator>
</mujoco>
```

**Design notes**:
- Same structure as D1's MJCF (§5), swapping `ratchet_ctrl` for `temp_ctrl`
- `ctrlrange="0 10"` matches the `clamp(0.0, 10.0)` in the thermostat
- `damping="0"`: the thermostat owns damping (Q4 resolution)
- `gravity="0 0 0"`: 1-DOF slide, no gravitational bias
- `integrator="Euler"`: required for Langevin (Euler-Maruyama)
- `contact="disable"`: no collision geometry
- `gainprm="0" biasprm="0 0 0"`: zero force — the actuator exists solely
  to provide `data.ctrl[0]` as a communication channel

**Ctrl flow** (same as D1 §5):
1. `ActionSpace::apply` writes `data.ctrl[0]` (before sub-stepping)
2. Each sub-step: `mj_fwd_actuation` reads ctrl (zero force),
   `mj_fwd_passive` → `cb_passive` → thermostat reads `data.ctrl[0]`
3. ctrl persists across all sub-steps until the next `ActionSpace::apply`

## 7. PassiveStack Configuration

```rust
let thermostat = LangevinThermostat::new(
    DVector::from_element(model.nv, gamma),
    k_b_t_base,
    seed,
).with_ctrl_temperature(0);  // ← D2: agent controls kT via ctrl[0]

let double_well = DoubleWellPotential::new(delta_v, x_0, 0);
let signal = OscillatingField::new(a_0, omega, 0.0, 0);

PassiveStack::builder()
    .with(thermostat)
    .with(double_well)
    .with(signal)
    .build()
    .install(&mut model);
```

Three components compose naturally in the stack:
1. Thermostat provides noise (FDT pair) at agent-controlled temperature
2. Double-well provides the bistable energy landscape
3. Oscillating field provides the sub-threshold periodic signal

No new infrastructure needed. The Phase 1-3 chassis handles the composition.

## 8. RL Environment Design

### 8.1 Observation space

| Index | Field   | Description                     | Scale factor      |
|-------|---------|---------------------------------|--------------------|
| 0     | qpos[0] | Particle position x             | 1/x₀ (normalize by well separation) |
| 1     | qvel[0] | Particle velocity v             | 1/v_th where v_th = √(kT_base/M) |

Observation dimension: **2**.

The agent needs:
- **Position** — to know which well the particle occupies
- **Velocity** — to read the current thermal fluctuation state

**Why no signal phase**: Classical SR is a *static-parameter* phenomenon —
the optimal temperature depends on barrier height, damping, and signal
frequency, none of which change within an episode. The agent does not need
to know the signal phase to find the SR peak; it needs to find the right
constant temperature. A `LinearPolicy(obs_dim=2, act_dim=1)` will converge
to `kT ≈ w₁·x + w₂·v + bias`. Since position and velocity carry no
information about the optimal noise level (the SR peak is a property of the
energy landscape, not the instantaneous state), CEM will zero out `w₁` and
`w₂` and converge the bias to the SR-optimal multiplier.

**Codebase constraint**: `ObservationSpace` (verified against
`sim/L0/ml-bridge/src/space.rs`) supports fixed extractors for physics
state fields (`qpos`, `qvel`, `time`, `sensordata`, etc.) but has **no
mechanism for computed features** like `sin(ωt)` or `cos(ωt)`. There is
no custom observation closure on `SimEnv` or `VecEnv`. Including signal
phase would require extending `ObservationSpace` with a closure-based
custom extractor — a small ml-bridge API change tracked for D2d.

**D2d extension — dynamic temperature modulation**: the more interesting
question is whether modulating temperature *in sync with the signal cycle*
(boosting noise when a switch is expected, dampening it otherwise) can
outperform the static optimum. Testing this requires signal-phase
observations `(sin(ωt), cos(ωt))`, which requires the `ObservationSpace`
extension above. If D2c validates the static SR result, D2d extends the
observation to obs_dim=4 and re-trains to test for dynamic modulation.
This is a separate deliverable, not a gate for D2c.

### 8.2 Action space

| Index | Field    | Range   | Description                    |
|-------|----------|---------|--------------------------------|
| 0     | ctrl[0]  | [0, 10] | Temperature multiplier on kT_base |

Action dimension: **1**.

Continuous [0, 10]. The effective bath temperature is
`kT_eff = kT_base × ctrl[0]`. CEM will search for the optimal constant
value. (Dynamic modulation is a D2d extension — see §8.1.)

### 8.3 Reward

```text
reward = sign(qpos[0]) * cos(ω · data.time)
```

**Synchrony measure**: the product of the particle's well indicator
(`sign(x)`: +1 in right well, −1 in left well) and the normalized signal
(`cos(ωt)`: +1 when signal pushes toward right well). This is bounded
in `[-1, 1]` per step.

**Behavior at different noise levels**:
- **Low noise** (kT ≪ kT*): particle stuck in one well, sign(x) constant.
  Reward averages to zero over full signal cycles (⟨cos(ωt)⟩ = 0).
- **High noise** (kT ≫ kT*): random switching, sign(x) uncorrelated with
  signal. Reward averages to zero (⟨sign(x)·cos(ωt)⟩ = 0 for independent
  random variables).
- **SR peak** (kT ≈ kT*): switching synchronized with signal. sign(x) ≈
  sign(cos(ωt)), so the product ≈ |cos(ωt)|. Reward averages to 2/π ≈ 0.637
  (the theoretical maximum for a two-state system tracking a sinusoid).

**CEM fitness**: with fixed-length episodes, CEM's fitness = mean reward
= time-averaged synchrony. CEM maximizes this, converging to the
temperature that maximizes signal detection. The reward polarity is
naturally correct — no negation needed (unlike D1's velocity reward).

**Noise**: per-step reward is noisy (sign(x) can flip within a well during
fast oscillations near x=0). Over many steps the noise averages out;
the statistical precision is `σ_fitness ≈ 1/√N_steps` per episode.

### 8.4 Observation construction

Standard `ObservationSpace` builder — no custom features needed for D2c:

```rust
let obs_space = ObservationSpace::builder()
    .all_qpos()   // [qpos[0]] = particle position
    .all_qvel()   // [qvel[0]] = particle velocity
    .build(&model)
    .unwrap();
```

This produces a 2-element observation `[x, v]` via
`ObservationSpace::extract(&data)`, which is called internally by
`SimEnv::observe()` and `VecEnv`'s step/reset methods. No custom closure
or post-processing needed.

**Verified against codebase**: `ObservationSpace::extract` is the sole
observation path in both `SimEnv` (`env.rs:129-131`) and `VecEnv`
(`vec_env.rs:233-234`). There is no custom observation closure on either
type. The builder supports `.qpos()`, `.qvel()`, `.time()`, `.sensordata()`,
and other fixed extractors (`space.rs:284-434`), but not computed features.

**D2d note**: if the D2d extension adds signal-phase observations
`(sin(ωt), cos(ωt))`, it will require either (a) extending
`ObservationSpace` with a closure-based custom extractor (small ml-bridge
change), or (b) a custom training loop that augments observations outside
the `Cem::train()` path. Decision deferred to D2d.

### 8.5 Episode structure

| Parameter          | Value     | Rationale                               |
|--------------------|-----------|-----------------------------------------|
| Physics timestep h | 0.001     | Same as Phase 1-3, D1                   |
| Sub-steps/decision | 100       | Decision interval 0.1 t.u. ≪ τ_relax   |
| Episode length     | 5000 decisions | 500 t.u. ≈ 6 signal cycles         |
| Done condition     | never     | No terminal state for SR                |
| Truncated          | at step 5000 | Fixed-length episodes (CEM invariant) |

**Why 5000 steps**: at the SR peak, we expect ~6 signal cycles per
episode (T_signal ≈ 82.4 t.u., T_episode = 500 t.u.), each with ~1
switching event. This gives ~6 measured switches — enough for the
per-episode synchrony metric to distinguish SR from non-SR regimes, but
on the lower end for statistical precision. The D1 lesson (CEM's
`sum/length` normalization requires equal-length episodes) applies here
with the same force.

**Fallback if 6 cycles is marginal**: if D2b baselines show that
per-episode synchrony variance is too high to separate the SR peak from
the tails (i.e., the sweep curve is noisy despite averaging over 20
episodes), increase episode length to 10,000 decisions (1000 t.u. ≈ 12
signal cycles). This doubles the computational budget for CEM training
(from ~320M to ~640M physics steps) but remains tractable in release
mode. The decision point is D2b — if the sweep produces a clean peak
at 5000 steps, keep it; if not, bump to 10,000 before starting D2c.

### 8.6 On-reset

Each episode resets `qpos[0] = 0.0`. The particle starts at the barrier top
and thermalizes into one of the wells within ~τ_relax ≈ 0.6 t.u. (6
decision steps). This is negligible compared to the 500 t.u. episode.

`Data::reset()` calls `ctrl.fill(0.0)`, so the thermostat starts with
kT_eff = kT_base × 0 = 0 (no noise). The first `ActionSpace::apply` sets
ctrl to the agent's chosen multiplier. One decision interval of zero noise
at episode start — a cold start that does not distort CEM rankings (same
across all policies).

**Important**: the `data.time` field resets to 0.0 on `Data::reset`. This
means the signal phase restarts at `cos(0) = 1.0` every episode. All
episodes have the same signal phase trajectory. This is a feature, not a
bug: it removes signal-phase randomness across episodes, giving CEM clean
comparisons between policies. (Phase randomization would be a D2+ extension
for testing robustness.)

## 9. Central Parameter Set

| Parameter           | Symbol  | Value   | Rationale                           |
|---------------------|---------|---------|-------------------------------------|
| Mass                | M       | 1.0     | Unit mass                           |
| Damping             | γ       | 10.0    | Overdamped; matches Phase 3 Kramers validation |
| Base temperature    | kT_base | 1.0     | Unit thermal energy; SR peak near multiplier = 1 |
| Barrier height      | ΔV      | 3.0     | Matches Phase 3; deeply bistable    |
| Well separation     | x₀      | 1.0     | Unit separation                     |
| Signal amplitude    | A₀      | 0.3     | Sub-threshold: A₀/F_c ≈ 6.5%       |
| Signal frequency    | f_s     | k₀(kT=1) | SR peak at kT_base (Kramers matching) |
| Signal angular freq | ω       | 2π·f_s  | ≈ 0.0763 rad/t.u.                  |
| Signal period       | T_s     | 1/f_s   | ≈ 82.4 t.u.                        |
| Signal phase offset | φ₀      | 0.0     | Cosine phase; signal is positive at t=0 |
| Timestep            | h       | 0.001   | Euler-Maruyama                      |

**Derived quantities**:

| Quantity              | Formula              | Value   | Interpretation                    |
|-----------------------|----------------------|---------|-----------------------------------|
| Critical force F_c    | 8ΔV/(3√3·x₀)        | ≈ 4.62  | Deterministic switching threshold |
| Sub-threshold ratio   | A₀/F_c               | ≈ 6.5%  | Deeply sub-threshold              |
| Kramers rate k₀(kT=1)| kramers_rate(10,1,1)  | 0.01214 | Phase 3 validated value           |
| Well curvature ω_a    | √(8ΔV/(M·x₀²))      | ≈ 4.899 | Oscillation freq at well bottom   |
| Barrier curvature ω_b | √(4ΔV/(M·x₀²))      | ≈ 3.464 | Curvature at barrier top          |

**Why ΔV=3.0 and γ=10.0 (same as Phase 3)**: These parameters are
*Kramers-validated* — the Phase 3 integration test confirmed that measured
escape rates match the Kramers formula to within statistical tolerance.
Reusing them means D2 builds on a validated substrate rather than
introducing new parameters that haven't been cross-checked.

**Signal frequency design**: the signal frequency is set to `f_s = k₀(kT_base)`
so that the SR matching condition (§2.2) places the peak in the neighborhood
of `kT_base`. Depending on the exact matching criterion (factor of 2 vs π),
the peak falls at kT ≈ 1.0–1.6 (multiplier ≈ 1.0–1.6). The D2b temperature
sweep will determine the empirical peak; the agent should converge near it.
The ±50% tolerance in Gate D (§11 D2c) accommodates this analytical
uncertainty.

**Time scale hierarchy**:

| Time scale            | Formula           | Value   | Interpretation              |
|-----------------------|-------------------|---------|-----------------------------|
| τ_relax               | γ/V''(x₀) = γ/(8a·x₀²) | 0.625  | In-well relaxation          |
| τ_decision            | sub_steps × h     | 0.1     | Agent acts every 0.1 t.u.  |
| T_signal              | 1/f_s             | ≈ 82.4  | Signal period               |
| τ_escape(kT=1)        | 1/k₀              | ≈ 82.4  | Mean escape time (SR match) |
| T_episode             | 5000 × 0.1        | 500     | Total episode time          |

Hierarchy: τ_decision ≪ τ_relax ≪ T_signal ≈ τ_escape ≪ T_episode.

This ensures:
- The agent makes many decisions per relaxation time (can react quickly)
- The particle thermalizes between decisions (well occupancy is well-defined)
- The signal period matches the escape time (SR condition)
- Episodes contain ~6 signal cycles (enough for synchrony measurement)

## 10. Baselines

### 10.1 Temperature sweep (the "cheating" baseline)

Grid search over kT multipliers in [0.1, 5.0], 30 values, logarithmically
spaced. For each kT, run 20 episodes with fixed temperature (no agent),
measure mean synchrony. Plot the SR curve: synchrony vs. kT. The peak of
this curve is the analytical optimum — the trained agent should converge
to this.

### 10.2 Low noise (multiplier = 0.1)

kT_eff = 0.1. Kramers rate k(0.1) ≈ 10⁻¹³ — effectively zero switching.
Particle trapped in one well. Expected synchrony ≈ 0.

### 10.3 High noise (multiplier = 5.0)

kT_eff = 5.0. Kramers rate k(5.0) ≫ f_signal — rapid random switching.
Expected synchrony ≈ 0 (uncorrelated with signal).

### 10.4 SR-peak noise (multiplier from sweep)

kT_eff at the empirically measured SR peak from §10.1. This is the
open-loop optimal — the trained agent should approach or exceed this.

### 10.5 No signal (A₀ = 0)

Control experiment: with zero signal amplitude, switching is purely
noise-driven regardless of kT. Expected synchrony ≈ 0 at all temperatures.
(This can't happen by accident with A₀ = 0.3, but it validates the
synchrony metric.)

## 11. Training Plan

### Phase D2a — OscillatingField + with_ctrl_temperature

**Deliverable**: `sim-thermostat/src/oscillating_field.rs` with unit tests,
and modified `langevin.rs` with `with_ctrl_temperature` and new unit tests.

1. Implement `OscillatingField` per §4
2. Unit tests per §4.4
3. Add `k_b_t_ctrl` field + `with_ctrl_temperature` builder per §5
4. Modify `apply` per §5.3
5. New unit tests per §5.5
6. Integration test: install all three components (thermostat with ctrl_temp,
   double-well, oscillating field) on MJCF model, step 10,000 times with
   ctrl = 1.0, verify particle oscillates in wells and signal force is present

**Gate**: all unit tests pass, integration test shows bounded bistable motion
with periodic signal force superimposed.

### Phase D2b — Baselines

**Deliverable**: `sim-thermostat/tests/d2b_stochastic_resonance_baselines.rs`
with temperature sweep and baseline measurements.

1. Build the RL environment per §8 (construct model, install PassiveStack,
   build `SimEnv` with `ObservationSpace::builder().all_qpos().all_qvel()`)
2. Run temperature sweep (§10.1) → measure and print the SR curve
3. Run low-noise baseline (§10.2) → measure mean synchrony
4. Run high-noise baseline (§10.3) → measure mean synchrony
5. Identify the empirical SR peak temperature from the sweep

**Gate**: SR curve shows a clear peak. Low-noise and high-noise baselines
have synchrony consistent with zero. Peak synchrony is statistically
greater than zero (p < 0.01).

**D2b results (2026-04-11)**: all gates passed.

| Baseline | Synchrony (mean ± stderr) | Gate |
|----------|---------------------------|------|
| Temperature sweep peak | 0.098 ± 0.022 at kT×2.55 | |t| = 4.40, p < 0.01 PASS |
| Low noise (kT×0.1) | 0.003 ± 0.002 | |mean| < 3σ PASS |
| High noise (kT×5.0) | 0.044 ± 0.017 | |mean| < 3σ PASS |
| No signal (A₀=0, kT×1.5) | 0.019 ± 0.024 | |mean| < 3σ PASS |
| Peak > 2× tails | 0.098 > 2×0.034 | PASS |

**Findings**:
- The SR curve is broad: synchrony is elevated across kT ≈ 1.1–2.5
  (all in the 0.04–0.10 range), with the measured peak at kT×2.55. The
  analytical prediction (§2.2, §9) of kT ≈ 1.0–1.6 falls within this
  elevated band but not at the noisy maximum. With 20 episodes × 6 signal
  cycles, per-kT variance is high; the "peak" location is noise-sensitive.
- Peak synchrony (0.098) is well below the theoretical maximum 2/π ≈ 0.637.
  This is expected: at A₀/F_c ≈ 6.5%, the signal barely modulates the
  barrier. The synchrony is measurable but modest.
- CEM fitness signal for D2c: peak synchrony ~0.10, noise floor
  σ_fitness ≈ 1/√5000 ≈ 0.014, giving SNR ≈ 7. Weaker than D1c's
  SNR ≈ 12 but adequate for CEM to distinguish temperatures.
- Episode length 5000 is sufficient — the fallback to 10,000 (§8.5) is
  NOT needed.
- Gate D target for D2c: ±50% of empirical peak kT×2.55 = [1.27, 3.82].

### Phase D2c — CEM training

**Deliverable**: CEM trains a policy that discovers the SR-optimal
temperature.

1. Build `VecEnv` with 32 envs, synchrony reward (§8.3)
2. Train CEM with `LinearPolicy(obs_dim=2, act_dim=1)`
3. Hyperparameters:
   - Population: 32 envs
   - Elite fraction: 0.2
   - Noise std: 0.5, decay: 0.99, min: 0.05
   - Max episode steps: 5000
   - Budget: 100 epochs
4. Evaluate trained policy: 20 episodes, measure mean synchrony and
   effective temperature
5. Compare against baselines and SR sweep

**Gate A** (SR effect): trained policy mean synchrony > 0 with p < 0.01.

**Gate B** (learning): mean synchrony improves over training
(best-of-last-10 > epoch 0 synchrony).

**Gate C** (baseline separation): trained policy mean synchrony > 2×
the mean of uniformly random temperature baselines (random multiplier
∈ [0.1, 5.0] each step).

**Gate D** (SR peak recovery): report the trained policy's mean
effective temperature. The policy should converge to approximately
constant temperature (position/velocity weights ≈ 0, bias dominates).
The effective kT should be within ±50% of the **empirical** SR peak
from D2b (not the analytical prediction — see §2.2 caveat and §9
signal frequency design).

### Phase D2d — Dynamic modulation extension (optional, deferred)

**Deliverable**: Test whether phase-dependent temperature modulation
outperforms the static SR optimum.

**Prerequisite**: extend `ObservationSpace` with a closure-based custom
extractor so that `sin(ωt)` and `cos(ωt)` can be included in the
observation vector without modifying `SimEnv`/`VecEnv` internals. This
is a small ml-bridge API change (~50 LOC): add a `Custom` variant to the
private `Extractor` enum in `space.rs` and a `.custom()` builder method.

1. Extend `ObservationSpace` per above
2. Re-train CEM with `LinearPolicy(obs_dim=4, act_dim=1)`, obs =
   `[x/x₀, v/v_th, sin(ωt), cos(ωt)]`
3. Inspect weights: if sin/cos weights are significant, the agent
   discovered dynamic modulation → plot kT(t) over one signal cycle
4. Compare synchrony against D2c's static result
5. Compare learned effective temperature against the two-state theory
   prediction

**Gate**: no hard pass/fail — this is a research analysis phase. The
question is whether dynamic modulation exists and whether it improves
on the static optimum. Either answer is a valid result.

## 12. Dependency Routing

```
sim-thermostat (library)
  ├── OscillatingField (new, §4)
  ├── LangevinThermostat.with_ctrl_temperature (modified, §5)
  ├── DoubleWellPotential (existing, Phase 3)
  ├── PassiveStack (existing)
  └── [dev-dependencies]
       └── sim-ml-bridge (existing dev-dep from D1)

sim-ml-bridge (library, unchanged)
  ├── Environment trait
  ├── SimEnv / VecEnv
  ├── CEM
  └── LinearPolicy
```

D2 integration tests live in `sim-thermostat/tests/d2*.rs`, following the
D1 pattern. Same dependency routing — `sim-ml-bridge` is a dev-dep only.

## 13. Validation and Measurement

### 13.1 Statistical protocol

All synchrony measurements use independent episodes with different RNG seeds
(via per-env `LangevinThermostat` instances or sequential `make_eval_env`
calls). Report mean ± standard error. Statistical significance via
one-sample t-test (synchrony ≠ 0) and two-sample t-test (baseline
comparisons).

### 13.2 Synchrony metric

Per-episode synchrony S = (1/N) Σₜ sign(qpos[0]_t) · cos(ω·t)

Bounded in [-1, 1]. Theoretical maximum ≈ 2/π ≈ 0.637 (perfect two-state
tracking of a sinusoidal signal). Expected value at the SR peak: some
fraction of 2/π, determined empirically in D2b.

### 13.3 Effective temperature measurement

For policy analysis: measure the mean ctrl[0] value over an evaluation
episode. This is the time-averaged temperature multiplier. If the policy
is static (constant output), this equals the output value. If dynamic,
it's the mean modulation level.

### 13.4 Release mode

All tests with multi-episode rollouts must use `cargo test --release`.
Each episode is 500,000 physics steps (5× D1's 100,000). CEM training:
32 envs × 100 epochs × 500,000 steps = 1.6B steps (~5× D1c's 320M).
At D1c release-mode throughput (~10M steps/sec), D2c training takes
~2.5 minutes. Release mode is non-negotiable.

## 14. Q6 Partial Resolution (continued)

D1 answered Q6 for the **ratchet case**: reward = net displacement.

D2 answers Q6 for the **stochastic resonance case**: reward = synchrony
between particle state and input signal, measured as
`sign(qpos) · cos(ωt)`.

Q6 remains open for:
- General sampler case: reward = sample quality (ESS, KL, autocorrelation)

## 15. Forward References

- **D3 bridge pattern**: D2 establishes that `LangevinThermostat` can
  modulate kT at runtime via ctrl. D3 extends this to all physical
  parameters simultaneously.
- **VecEnv shared-model caveat**: same as D1 §14 — all 32 envs share one
  `Arc<Model>` and one thermostat instance. The shared RNG produces
  interleaved-but-i.i.d. noise. Accepted for D2 (same analysis as D1).
  If independent per-env noise streams are needed, the `install_per_env`
  factory path (chassis Decision 3) is the fix.
- **ObservationSpace custom extractor**: D2c uses standard `[qpos, qvel]`
  observations (no custom features needed for static SR). D2d requires
  signal-phase features `(sin(ωt), cos(ωt))`, which requires extending
  `ObservationSpace` with a closure-based custom extractor — a small
  ml-bridge API change. This same extension would benefit D3+ experiments
  that need computed observation features beyond raw physics state.

## 16. What D2 Does NOT Do

- **No multi-DOF SR** — single 1-DOF particle. Coupled SR (array of
  bistable elements with shared signal) is an extension.
- **No buckling-beam geometry** — the D2 entry in `research_directions.md`
  mentions a cf-design buckling beam. D2 uses the mathematical equivalent
  (`DoubleWellPotential`) for clean validation. The cf-design geometry is
  D4 territory (sim-to-real).
- **No SNR spectral analysis** — the canonical SR metric is the power
  spectral density at the signal frequency. D2 uses the simpler synchrony
  metric (time-domain correlation) which is directly interpretable as an
  RL reward. Spectral analysis is a possible D2e extension.
- **No Bevy visualization** — headless tests only. Visual example is a
  follow-up.
- **No REINFORCE comparison** — D1d proved CEM is preferred for switching
  tasks. D2 uses CEM only.
- **No dynamic temperature modulation** (D2c) — the agent sees `[x, v]`
  only, not signal phase. D2c validates the static SR result. Dynamic
  modulation (does phase-dependent kT beat the static optimum?) is D2d,
  which requires an `ObservationSpace` extension.
- **No phase randomization** — all episodes start with the same signal
  phase (t=0). Testing robustness to random initial phase is a D2+
  extension.
