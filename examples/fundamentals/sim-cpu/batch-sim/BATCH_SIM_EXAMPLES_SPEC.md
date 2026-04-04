# Batch-Sim Examples Spec

Parallel multi-environment simulation using `BatchSim`.

**Engine source:** `sim/L0/core/src/batch.rs` (224 LOC impl + 375 LOC tests)

**API surface exercised:**

| Method | Example |
|--------|---------|
| `BatchSim::new(model, n)` | parameter-sweep |
| `step_all()` | parameter-sweep, reset-subset |
| `envs()` / `envs_mut()` | parameter-sweep, reset-subset |
| `env(i)` / `env_mut(i)` | reset-subset |
| `reset_where(mask)` | reset-subset |
| `reset(i)` | stress-test |
| `reset_all()` | stress-test |
| `len()` / `is_empty()` | stress-test |
| `model()` | stress-test |

---

## Example 1: `parameter-sweep` (visual)

**One concept:** BatchSim construction and parallel stepping.

### Design decision: damping via ctrl

All environments in a `BatchSim` share one `Model` — per-env `dof_damping`
is impossible. Instead, each env applies a **velocity-proportional control
torque** as a damping proxy: `ctrl[0] = -D_i * qvel[0]`. The MJCF model has
one motor actuator on the hinge joint with `damping="0"` on the joint itself.

Motor actuators default to `ctrllimited=false` (unbounded ctrlrange), so
the torque is never clipped regardless of velocity magnitude.

### What you see

Eight pendulums side by side, each with a different effective damping
coefficient (0.0, 0.1, 0.2, ..., 0.7). All start tilted 90° from vertical.
They swing simultaneously — the undamped pendulum swings forever, while the
highest-damped one dies out almost immediately.

The HUD shows per-environment energy and the damping value, updating in
real-time. After 10 seconds, a validation report prints.

### Scene

- **MJCF:** Single-link pendulum (hinge joint, Y-axis) with one motor
  actuator (`<motor joint="hinge" gear="1"/>`). Joint `damping="0"`. Energy
  enabled via `<flag energy="enable"/>`.
- **BatchSim:** `BatchSim::new(Arc::new(model), 8)`.
- **Init:** `qpos[0] = π/2` (90° tilt) for all 8 envs via `envs_mut()`.
- **Per-step:** Before each `step_all()`, iterate `envs_mut()` and set
  `env.ctrl[0] = -D_i * env.qvel[0]` where `D_i = i * 0.1`.

### Visual rendering

`BatchSim` is stored as a Bevy `Resource`. Eight visual pendulums are
spawned manually at X-offsets (one rod + sphere per env). A custom sync
system runs each frame: reads `batch.env(i).qpos[0]` and updates the
corresponding visual's `Transform` rotation. This is **not** the
`MultiScenePlugin` pattern (which holds N separate Model/Data pairs) — we
use a single `BatchSim` resource and sync visuals directly.

- **Camera:** Side view, all 8 pendulums visible. X-spacing ~1.5m.
- **Labels:** Each pendulum labeled with its `D` value (text below pivot).

### Physics

Effective damping torque: `τ = -D * ω` where `ω = qvel[0]`.

With no damping (`D=0`), total mechanical energy `E = KE + PE` is conserved.
With damping, energy decays as `dE/dt = -D * ω²` — exponential envelope.

| Env | D | Expected behavior |
|-----|---|-------------------|
| 0 | 0.0 | Perpetual swing, energy ≈ initial |
| 1 | 0.1 | Slow decay, still swinging at t=10 |
| 2 | 0.2 | Moderate decay |
| 3 | 0.3 | Moderate decay |
| 4 | 0.4 | Noticeable settling |
| 5 | 0.5 | Mostly settled by t=10 |
| 6 | 0.6 | Nearly stopped |
| 7 | 0.7 | Overdamped, barely swings |

### Validation (4 checks at t=10s)

| # | Check | Criterion |
|---|-------|-----------|
| 1 | Undamped energy conserved | `\|E(t=10) - E(t=0)\| / E(t=0) < 0.01` for env 0 |
| 2 | Damped energy decays | `E_7(t=10) < 0.05 * E_7(t=0)` for env 7 (highest damping) |
| 3 | Energy monotonically ordered | `E_0 > E_1 > ... > E_7` at t=10 |
| 4 | All envs stepped | All 8 envs have `time ≈ 10.0` |

Energy is read directly from `env.energy_kinetic + env.energy_potential`
(computed automatically during `step()` when `ENABLE_ENERGY` is set).
`data.total_energy()` is the convenience method.

### Package

- **Crate name:** `example-batch-sim-parameter-sweep`
- **Dependencies:** `sim-core`, `sim-mjcf`, `sim-bevy`, `bevy`

---

## Example 2: `reset-subset` (visual)

**One concept:** Selective reset with `reset_where(mask)`.

### What you see

Sixteen pendulums in a row. Each has a different initial angular velocity
(0.0 to 4.5 rad/s). They swing freely. Every 1.0s of **sim time**, any
pendulum whose angle exceeds ±90° gets reset — it snaps back to its starting
state and resumes swinging. Pendulums that haven't crossed the threshold
continue uninterrupted.

The HUD shows per-env status ("swinging" vs "RESET") and a running count
of total resets.

### Scene

- **MJCF:** Single-link pendulum with hinge joint, no damping, no actuator.
- **BatchSim:** `BatchSim::new(Arc::new(model), 16)`.
- **Init:** For each env `i` via `envs_mut()`:
  - `qpos[0] = π/4` (45° tilt)
  - `qvel[0] = i as f64 * 0.3` (0.0 to 4.5 rad/s)
- **Per-second logic (every 1.0s sim time):**
  1. Build mask: `mask[i] = |env.qpos[0]| > π/2`
  2. Call `batch.reset_where(&mask)`
  3. For each reset env: restore initial conditions
     (`qpos[0] = π/4`, `qvel[0] = v_i`) — `reset()` restores to
     `qpos0 = 0, qvel = 0`, so we must re-apply the per-env initial state.

### Visual rendering

Same pattern as parameter-sweep: `BatchSim` as a `Resource`, 16 visual
pendulums at X-offsets, custom sync system. Pendulums that just reset
flash a highlight color for a few frames.

### Physics

An undamped pendulum's max angle depends on initial kinetic energy.
Conservation of energy: `½Iω² = mgl(1 - cos θ_max)`. With different initial
ω, some envs never reach 90° and others overshoot easily.

| Env group | Initial ω (rad/s) | Crosses ±90°? | Gets reset? |
|-----------|-------------------|---------------|-------------|
| 0–4 | 0.0–1.2 | No | Never |
| 5–10 | 1.5–3.0 | Sometimes | Occasionally |
| 11–15 | 3.3–4.5 | Frequently | Often |

### Validation (4 checks at t=10s)

| # | Check | Criterion |
|---|-------|-----------|
| 1 | Low-velocity envs never reset | Envs 0–2 have `time > 9.0` (continuous, never reset) |
| 2 | High-velocity envs reset | Envs 13–15 have been reset at least once (tracked by counter) |
| 3 | Reset restores qpos | Immediately after `reset_where`, reset envs have `qpos == qpos0` |
| 4 | Non-reset envs untouched | `reset_where` does not alter envs where `mask[i] == false` |

### Package

- **Crate name:** `example-batch-sim-reset-subset`
- **Dependencies:** `sim-core`, `sim-mjcf`, `sim-bevy`, `bevy`

---

## Example 3: `stress-test` (headless)

**Concept:** Exhaustive validation of the BatchSim API.

### Checks (10)

| # | Check | Setup | Criterion |
|---|-------|-------|-----------|
| 1 | N environments created | `BatchSim::new(model, 8)` | `batch.len() == 8` and `!batch.is_empty()` |
| 2 | Environments have independent state | Set `env_mut(0).qpos[0] = 1.0`, leave others at 0 | After `step_all()`, env 0 qpos differs from env 1 qpos |
| 3 | step_all advances all | Step 10 times | All 8 envs have `time == 10 * dt` |
| 4 | No cross-contamination | Set ctrl on env 0 only, step | Env 1 qpos unchanged vs separate single-env step |
| 5 | Batch matches sequential | 4 envs, distinct initial qpos, step 50 times | `batch.env(i).qpos == sequential[i].qpos` for all i (bitwise) |
| 6 | reset(i) resets only i | Step 10, reset(2) | `env(2).time == 0`, `env(2).qpos == qpos0`; `env(0).time > 0` |
| 7 | reset_where selective | Step 10, `reset_where([T,F,T,F,T,F,T,F])` | Even envs reset, odd envs untouched |
| 8 | reset_all resets everything | Step 10, `reset_all()` | All envs have `time == 0` and `qpos == qpos0` |
| 9 | Shared model | Check `batch.model()` | `batch.model().nq == model.nq` and `batch.model().nv == model.nv` |
| 10 | Single-env batch matches standalone | `BatchSim::new(model, 1)` vs `model.make_data()`, step 50 | `batch.env(0).qpos == standalone.qpos` (bitwise) |

### Package

- **Crate name:** `example-batch-sim-stress-test`
- **Dependencies:** `sim-core`, `sim-mjcf`

---

## Directory Layout

```
examples/fundamentals/sim-cpu/batch-sim/
├── README.md
├── parameter-sweep/
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
├── reset-subset/
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
└── stress-test/
    ├── Cargo.toml
    ├── README.md
    └── src/main.rs
```

## Estimated LOC

| Example | LOC |
|---------|-----|
| parameter-sweep | ~200 (MJCF + 8 visual pendulums + ctrl loop + HUD + validation) |
| reset-subset | ~200 (MJCF + 16 visual pendulums + reset logic + HUD + validation) |
| stress-test | ~200 (10 check functions + main) |
| **Total** | **~600** |
