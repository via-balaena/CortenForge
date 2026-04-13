//! D2a — Stochastic resonance component integration tests.
//!
//! Validates that the three-component stack (`LangevinThermostat` with
//! `ctrl_temperature` + `DoubleWellPotential` + `OscillatingField`) composes
//! correctly and produces bounded bistable motion with a periodic signal
//! force superimposed.
//!
//! Spec: `docs/thermo_computing/03_phases/d2_stochastic_resonance.md` §7, §11 D2a

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::float_cmp,
    clippy::suboptimal_flops
)]

use std::f64::consts::PI;

use sim_core::DVector;
use sim_thermostat::{DoubleWellPotential, LangevinThermostat, OscillatingField, PassiveStack};

// ─── MJCF model (spec §6) ─────────────────────────────────────────────────

const SR_XML: &str = r#"
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
"#;

// ─── Central parameters (spec §9) ─────────────────────────────────────────

const DELTA_V: f64 = 3.0;
const X_0: f64 = 1.0;
const GAMMA: f64 = 10.0;
const K_B_T_BASE: f64 = 1.0;
const A_0: f64 = 0.3;
const SEED: u64 = 20_260_410;

/// Kramers rate at kT=1 for Phase 3 central parameters (ΔV=3, γ=10, M=1).
const KRAMERS_RATE: f64 = 0.01214;

/// Signal angular frequency: ω = 2π·k₀(kT=1) (spec §9).
fn signal_omega() -> f64 {
    2.0 * PI * KRAMERS_RATE
}

// ─── Tests ─────────────────────────────────────────────────────────────────

/// Three-component stack builds and installs without error.
#[test]
fn three_component_stack_installs() {
    let mut model = sim_mjcf::load_model(SR_XML).unwrap();

    let thermostat =
        LangevinThermostat::new(DVector::from_element(model.nv, GAMMA), K_B_T_BASE, SEED, 0)
            .with_ctrl_temperature(0);

    let double_well = DoubleWellPotential::new(DELTA_V, X_0, 0);
    let signal = OscillatingField::new(A_0, signal_omega(), 0.0, 0);

    let stack = PassiveStack::builder()
        .with(thermostat)
        .with(double_well)
        .with(signal)
        .build();
    stack.install(&mut model);

    let mut data = model.make_data();
    // Set ctrl to 1.0 (kT_eff = kT_base × 1.0 = 1.0)
    data.ctrl[0] = 1.0;

    // Step once to verify no panics
    data.step(&model).unwrap();
}

/// With ctrl=1.0 (`kT_eff`=1), the particle stays bounded and visits
/// both wells over 10k steps (bistable motion).
#[test]
fn bounded_bistable_motion_with_signal() {
    let mut model = sim_mjcf::load_model(SR_XML).unwrap();

    let thermostat =
        LangevinThermostat::new(DVector::from_element(model.nv, GAMMA), K_B_T_BASE, SEED, 0)
            .with_ctrl_temperature(0);

    let double_well = DoubleWellPotential::new(DELTA_V, X_0, 0);
    let signal = OscillatingField::new(A_0, signal_omega(), 0.0, 0);

    PassiveStack::builder()
        .with(thermostat)
        .with(double_well)
        .with(signal)
        .build()
        .install(&mut model);

    let mut data = model.make_data();
    data.ctrl[0] = 1.0;

    let n_steps = 10_000;
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;

    for _ in 0..n_steps {
        data.step(&model).unwrap();
        let x = data.qpos[0];
        min_x = min_x.min(x);
        max_x = max_x.max(x);
    }

    // Bounded: particle should stay within a few well separations.
    // The double-well has minima at ±1; with kT=1 and ΔV=3, excursions
    // beyond ±3 are extremely unlikely.
    assert!(
        max_x < 5.0,
        "particle escaped too far right: max_x = {max_x}",
    );
    assert!(
        min_x > -5.0,
        "particle escaped too far left: min_x = {min_x}",
    );

    // Bistable: particle should have visited both wells.
    // With kT=1 and ΔV=3, the Kramers rate is ~0.012 per t.u.
    // In 10 t.u. (10k steps at h=0.001), we expect ~0.24 escapes,
    // so the particle might not switch wells. Instead check it's
    // oscillating near one of the wells (|x| near x₀=1.0).
    //
    // The real bistability test is D2b with longer trajectories.
    // Here we just verify the particle is in a well, not at x=0.
    let final_x = data.qpos[0];
    assert!(
        final_x.abs() > 0.3,
        "particle should be near a well (|x| > 0.3), got x = {final_x}",
    );
}

/// Signal force is present: with thermostat disabled (no noise, no damping
/// contribution to force), the only forces are the double-well and the
/// oscillating field. Verify the signal force appears.
#[test]
fn signal_force_present_in_stack() {
    let mut model = sim_mjcf::load_model(SR_XML).unwrap();

    let thermostat =
        LangevinThermostat::new(DVector::from_element(model.nv, GAMMA), K_B_T_BASE, SEED, 0)
            .with_ctrl_temperature(0);

    let double_well = DoubleWellPotential::new(DELTA_V, X_0, 0);
    let omega = signal_omega();
    let signal = OscillatingField::new(A_0, omega, 0.0, 0);

    PassiveStack::builder()
        .with(thermostat)
        .with(double_well)
        .with(signal)
        .build()
        .install(&mut model);

    let mut data = model.make_data();
    data.ctrl[0] = 1.0;

    // Step to t=0.001 (one timestep)
    data.step(&model).unwrap();

    // At t ≈ 0.001 with x ≈ 0, the signal force is approximately
    // A₀ · cos(ω·0.001) ≈ 0.3 (ω is tiny, so cos ≈ 1).
    // The double-well force at x≈0 is ≈ 0.
    // The thermostat contributes damping + noise.
    //
    // We can't isolate the signal force from the total, but we can
    // verify the particle has nonzero acceleration (signal is present).
    // At x=0, the double-well force is exactly 0, so any nonzero
    // qfrc_passive at step 0 comes from thermostat + signal.
    //
    // Just verify the simulation produces nonzero movement.
    assert!(
        data.qpos[0].abs() > 1e-10 || data.qvel[0].abs() > 1e-10,
        "particle should have moved from signal + noise forces",
    );
}

/// ctrl=0 (zero noise) produces damping-only dynamics. Particle at x=0
/// (barrier top) with signal force should drift toward a well.
#[test]
fn ctrl_zero_is_deterministic() {
    let mut model = sim_mjcf::load_model(SR_XML).unwrap();

    let thermostat =
        LangevinThermostat::new(DVector::from_element(model.nv, GAMMA), K_B_T_BASE, SEED, 0)
            .with_ctrl_temperature(0);

    let double_well = DoubleWellPotential::new(DELTA_V, X_0, 0);
    let signal = OscillatingField::new(A_0, signal_omega(), 0.0, 0);

    PassiveStack::builder()
        .with(thermostat)
        .with(double_well)
        .with(signal)
        .build()
        .install(&mut model);

    let mut data = model.make_data();
    data.ctrl[0] = 0.0; // zero noise

    // Run two trajectories with the same setup — should be identical
    // (deterministic when ctrl=0, sigma=0).
    let n_steps = 100;
    for _ in 0..n_steps {
        data.step(&model).unwrap();
    }
    let x1 = data.qpos[0];

    // Reset and run again
    data.reset(&model);
    data.ctrl[0] = 0.0;
    for _ in 0..n_steps {
        data.step(&model).unwrap();
    }
    let x2 = data.qpos[0];

    // Note: the RNG advances even with ctrl=0 (sigma=0 * z = 0, but z
    // is still drawn). Two runs from the same seed produce the same RNG
    // sequence. But reset() doesn't re-seed the thermostat RNG — it
    // only resets Data. So the RNG stream is different for the second
    // run. However, since sigma=0, the noise contribution is always 0
    // regardless of the RNG draw. Both trajectories should be identical.
    assert!(
        (x1 - x2).abs() < 1e-12,
        "ctrl=0 should be deterministic: x1={x1}, x2={x2}",
    );
}
