# ThermCircuitEnv: Building the Experiment Platform

All 7 experiments in this research program share the same core loop: configure a Langevin particle system with an energy landscape, wire it into an RL environment, train a policy to modulate temperature or other control channels, and measure the result. The `ThermCircuitEnv` crate eliminates the ~35 lines of boilerplate that each experiment would otherwise repeat.

This chapter documents how the platform was built — what worked, what surprised us, and what the infrastructure tells us about the experiments it will support.

## Foundations: What We Built On

`ThermCircuitEnv` sits on top of two layers of infrastructure that were already in place:

**The ML chassis** ([`sim-ml-chassis`](../../../sim/L0/ml-chassis/)) provides the `Algorithm` trait, `VecEnv`, `SimEnv`, `Policy`, observation/action spaces, and the rollout machinery. This layer was the subject of a [prior study](../ml_chassis_refactor/book/) that refactored the original `sim-ml-bridge` into a clean chassis/algorithm split. The key outcome of that study: every algorithm (CEM, REINFORCE, PPO, TD3, SAC from [`sim-rl`](../../../sim/L0/rl/); SA, Richer-SA, PT from [`sim-opt`](../../../sim/L0/opt/)) bolts onto the same `Algorithm` trait and accepts a `&mut VecEnv`. This means any experiment built on `ThermCircuitEnv` automatically gets access to all 8 algorithms with zero additional wiring.

**The thermostat layer** ([`sim-thermostat`](../../../sim/L0/thermostat/)) provides `LangevinThermostat`, `PassiveComponent` (double-well potentials, oscillating fields, pairwise coupling, ratchets, external fields), and `PassiveStack`. The D2 stochastic resonance study validated this layer end-to-end with 4-algorithm comparison tests.

`ThermCircuitEnv` is the bridge: it takes a circuit description (particle count, energy landscape, control channels) and produces a ready-to-train `VecEnv` or a single `ThermCircuitEnv` for evaluation.

## The Build: Four Phases

### Phase 1 — MJCF Generation

The first problem: MuJoCo needs an XML model. Every experiment has N particles as 1D slide joints, zero-gravity Euler integration, contact disabled, and zero-gain actuators that exist only to allocate `data.ctrl` slots.

`generate_mjcf(n_particles, n_ctrl, timestep)` produces this XML programmatically. The key design decisions:

- **Slide joints** give nq = nv = 1 per particle. No quaternion headaches.
- **Zero-gain actuators** (`gainprm="0" biasprm="0 0 0"`) produce zero force regardless of ctrl value. All physics comes from the thermostat and landscape components via `cb_passive`. The actuators are ctrl-slot allocators, nothing more.
- **Mass = 1** simplifies the fluctuation-dissipation relation. Configurable later if needed.

### Phase 2 — Builder and Single Environment

The builder composes 9 steps into a fluent API:

```
ThermCircuitEnv::builder(n_particles)
    .gamma(10.0).k_b_t(1.0).timestep(0.001)
    .with(DoubleWellPotential::new(3.0, 1.0, 0))
    .with_ctrl_temperature()
    .reward(|_m, d| -(d.qpos[0] - 1.0).powi(2))
    .sub_steps(100).episode_steps(1000)
    .build()
```

**What we learned:**

- `SimEnvBuilder` requires all 5 closures (obs_space, act_space, reward, done, truncated) — no optional fields. `ThermCircuitEnvBuilder` supplies sensible defaults for `done` (always false) and `truncated` (time-based from `episode_steps`). Only `reward` is mandatory.
- The thermostat is constructed internally from `gamma` and `k_b_t`. The user never touches `LangevinThermostat` directly. Landscape components (double wells, fields, couplings) are added via `.with()` and the builder wraps them in `Arc` for the `PassiveStack`.
- We had to add `PassiveStackBuilder::with_arc()` to sim-thermostat — a 4-line method to accept pre-erased `Arc<dyn PassiveComponent>` without double-wrapping.
- No ctrl channels → `nu = 0` → dim-0 action space. This works: tests pass `Tensor::zeros(&[0])` for actions. Experiments that don't need ctrl-temperature can omit it entirely.

**Physics validation:** Damping at γ=10 over 100 sub-steps at h=0.001 produces velocity decay v ~ 0.37 (from exp(-γ·Δt) = exp(-1)), confirming the thermostat is active. Double-well restoring force at x=0.5 pushes the particle toward x=+1.0, confirming landscape components are wired through `cb_passive`.

### Phase 3 — VecEnv (Batched Training)

`build_vec(n_envs)` produces a `VecEnv` for parallel training. The implementation extracted shared setup into a private `prepare() -> PreparedCircuit` method, eliminating ~50 lines of duplication between `build()` and `build_vec()`.

**What we learned:**

- `VecEnvBuilder::on_reset` takes 3 args `(model, data, env_index)` vs the builder's 2-arg closure. Wrapped with `move |m, d, _idx| on_reset(m, d)`. Per-env seeding via index is deferred.
- VecEnvBuilder stores closures as `Arc` (vs SimEnvBuilder's `Box`), so passing `Box<dyn Fn>` produces `Arc<Box<dyn Fn>>`. Double indirection, negligible overhead.
- All 26 Phase 1-2 tests passed unchanged after the `prepare()` extraction — no regressions.

### Phase 4 — Experiment 4 Validation

The first end-to-end test: can CEM learn state-dependent temperature control in a double-well landscape?

**Setup:** 1 particle, double well (ΔV=3, x₀=1), ctrl-temperature, γ=10, kT=1, h=0.001, 100 sub-steps, 1000 episode steps. Reward: -(qpos[0] - x₀)². CEM with LinearPolicy(2, 1), 50 epochs, 16-env VecEnv.

**Baseline:** Constant temperature (ctrl=1.0 → kT=1.0) gives episode-total reward of -876.

**Result:** CEM best-epoch reward = -431. Roughly 2x improvement over baseline.

**What we learned:**

- `LinearPolicy` outputs `tanh(W·x_scaled + b)` ∈ [-1, 1]. With `ctrlrange="0 10"` and `ctrllimited="true"`, MuJoCo clamps the ctrl to [0, ~1.0]. The policy can only *reduce* temperature from baseline, never increase it. Despite this, the PN guidance strategy works: higher ctrl when in the wrong well (more noise → hopping), lower ctrl when in the right well (less noise → staying).
- CEM is noisy epoch-to-epoch. The gate uses best-epoch reward, not final-epoch. This is standard practice for evolutionary methods — the last perturbation batch can be unlucky.
- The test was tuned after initial failure: bias initialization (1.0), noise_std (1.5), and best-epoch gate were all decisions made after seeing the first run fail. This is honest but not ideal — future experiments should define gates before running.

**What this test does NOT prove:**

- It does not verify that the learned policy is genuinely state-dependent (the weights were never inspected).
- It uses a single seed.
- It tests only one algorithm (CEM) when 8 are available.
- It does not implement actual proportional navigation — there is no N parameter sweep, no noise-level sweep, no forward-model variant.

The test validates that the plumbing works end-to-end. The scientific claims of the dragonfly regime remain untested.

## The Crate

**Location:** [`sim/L0/therm-env/`](../../../sim/L0/therm-env/)

| File | Contents |
|------|----------|
| `src/lib.rs` | Re-exports |
| `src/builder.rs` | `ThermCircuitEnvBuilder`, `generate_mjcf`, `PreparedCircuit` |
| `src/env.rs` | `ThermCircuitEnv`, `Environment` impl |
| `src/error.rs` | `ThermCircuitError` |
| `tests/phase2.rs` | 26 tests (MJCF + builder + single env) |
| `tests/phase3.rs` | 5 tests (VecEnv batch) |
| `tests/experiment_4.rs` | 1 `#[ignore]` test (CEM training, ~30s release) |
| `tests/experiment_1.rs` | 6 `#[ignore]` tests (SR sweep, controls, CEM/PPO/SA training) |

38 tests total. 31 run in debug mode, 7 require `--release`.

## Open Infrastructure Questions

1. **Effective ctrl range:** The tanh → [0, 1] limitation means experiments needing kT > 1 will require either a rescaling layer, a different activation, or a wider ctrlrange in the MJCF. This is a design decision that should be made before Experiment 1.

2. **Per-env thermostat seeding:** Currently all VecEnv environments share the same thermostat seed. Per-env seeding via the `env_index` parameter in `on_reset` is deferred but will matter for statistical rigor.

3. **Multi-algorithm validation:** The platform supports all 8 algorithms via the `Algorithm` trait. Experiment 1 tests three algorithm classes (CEM, PPO, SA) on the same gradient-biased SR task — the first cross-algorithm validation on `ThermCircuitEnv`.
