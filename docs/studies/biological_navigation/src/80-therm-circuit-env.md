# Experiment Platform: ThermCircuitEnv

All experiments in this research program share the same core loop: configure a Langevin particle system with an energy landscape, wire it into a training environment, run a gradient-free optimizer or RL algorithm to modulate temperature or other control channels, and measure the result. The `ThermCircuitEnv` crate eliminates the boilerplate that each experiment would otherwise repeat.

## What It Provides

`ThermCircuitEnv` sits on top of two layers of infrastructure:

**The ML chassis** (`sim-ml-chassis`) provides the `Algorithm` trait, batched environments, policy representations, and rollout machinery. Every algorithm (CEM, REINFORCE, PPO, TD3, SAC from `sim-rl`; SA, Richer-SA, PT from `sim-opt`) bolts onto the same trait. Any experiment built on `ThermCircuitEnv` automatically gets access to all 8 algorithms with zero additional wiring.

**The thermostat layer** (`sim-thermostat`) provides `LangevinThermostat` and passive energy landscape components: double-well potentials, oscillating fields, pairwise coupling, ratchets, and external fields.

`ThermCircuitEnv` bridges these: it takes a circuit description (particle count, energy landscape, control channels) and produces a ready-to-train environment.

## Example

```rust
ThermCircuitEnv::builder(n_particles)
    .gamma(10.0).k_b_t(1.0).timestep(0.001)
    .with(DoubleWellPotential::new(3.0, 1.0, 0))
    .with_ctrl_temperature()
    .reward(|_m, d| -(d.qpos[0] - 1.0).powi(2))
    .sub_steps(100).episode_steps(1000)
    .build()
```

## Validation

The platform was validated with a CEM training test: one particle in a double well with ctrl-temperature. CEM learned state-dependent temperature control, achieving roughly 2x improvement over a constant-temperature baseline. This confirms the plumbing is correct end-to-end but does not test the scientific claims of any specific biological regime.

**Location:** `sim/L0/therm-env/` — 38 tests total (31 debug, 7 require `--release`).
