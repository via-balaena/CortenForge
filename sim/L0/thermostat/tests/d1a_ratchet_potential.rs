//! D1a — `RatchetPotential` integration test.
//!
//! Verifies that a 1-DOF particle in a two-harmonic ratchet potential
//! under the Langevin thermostat exhibits bounded motion (particle
//! oscillates in wells) when the potential is ON (α = 1).
//!
//! This is the D1a gate: the ratchet potential produces the expected
//! physics before any RL is attempted.
//!
//! Spec: `docs/thermo_computing/03_phases/d1_brownian_ratchet.md` §10

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_precision_loss,
    clippy::float_cmp,
    clippy::suboptimal_flops
)]

use sim_core::DVector;
use sim_thermostat::{LangevinThermostat, PassiveStack, RatchetPotential};

// ─── MJCF model (spec §5) ─────────────────────────────────────────────────

const RATCHET_XML: &str = r#"
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
"#;

// ─── Central parameters (spec §8) ─────────────────────────────────────────

const V1: f64 = 1.0;
const V2: f64 = 0.25;
const PHI: f64 = std::f64::consts::FRAC_PI_4; // π/4
const PERIOD: f64 = 1.0;
const GAMMA: f64 = 1.0;
const K_B_T: f64 = 1.0;
const SEED: u64 = 20_260_410;

// ─── Tests ─────────────────────────────────────────────────────────────────

/// Gate: with α=1 (potential ON) and thermostat, particle stays bounded.
///
/// Run 10,000 steps. The particle should oscillate within a few periods
/// of the origin, not drift to infinity.
#[test]
fn ratchet_on_produces_bounded_motion() {
    let mut model = sim_mjcf::load_model(RATCHET_XML).unwrap();
    assert_eq!(model.nv, 1, "expected 1 DOF");
    assert_eq!(model.nu, 1, "expected 1 actuator");

    let thermostat = LangevinThermostat::new(DVector::from_element(model.nv, GAMMA), K_B_T, SEED);
    let ratchet = RatchetPotential::new(V1, V2, PHI, PERIOD, 0, 0);

    let stack = PassiveStack::builder()
        .with(thermostat)
        .with(ratchet)
        .build();
    stack.install(&mut model);

    let mut data = model.make_data();

    // Set ctrl[0] = 1.0 (potential ON)
    data.ctrl[0] = 1.0;

    let n_steps = 10_000;
    let mut max_abs_x: f64 = 0.0;

    for _ in 0..n_steps {
        data.step(&model).unwrap();
        let x = data.qpos[0];
        max_abs_x = max_abs_x.max(x.abs());
    }

    // With kT = 1 and barrier ~ 1.25, the particle should stay within
    // a few periods. 20 periods is extremely conservative — if the
    // particle reaches this far, something is wrong.
    assert!(
        max_abs_x < 20.0 * PERIOD,
        "particle drifted too far: |x|_max = {max_abs_x}, expected < {}",
        20.0 * PERIOD,
    );

    // Also verify it moved at all (not stuck at zero)
    assert!(
        max_abs_x > 0.1,
        "particle didn't move: |x|_max = {max_abs_x}"
    );
}

/// With α=0 (potential OFF), particle undergoes free diffusion.
///
/// RMS displacement should grow as √(2Dt) where D = kT/(Mγ).
/// After 10,000 steps at h=0.001, t = 10.0.
/// Expected RMS = √(2 × 1.0 × 10.0) ≈ 4.47.
/// We just check it's in a reasonable range.
#[test]
fn ratchet_off_produces_diffusion() {
    let mut model = sim_mjcf::load_model(RATCHET_XML).unwrap();

    let thermostat = LangevinThermostat::new(DVector::from_element(model.nv, GAMMA), K_B_T, SEED);
    let ratchet = RatchetPotential::new(V1, V2, PHI, PERIOD, 0, 0);

    let stack = PassiveStack::builder()
        .with(thermostat)
        .with(ratchet)
        .build();
    stack.install(&mut model);

    let mut data = model.make_data();

    // ctrl[0] = 0.0 (potential OFF) — this is the default after make_data
    assert_eq!(data.ctrl[0], 0.0);

    let n_steps = 10_000;
    for _ in 0..n_steps {
        data.step(&model).unwrap();
    }

    let final_x = data.qpos[0];
    // Free diffusion: |x| should be non-trivial.
    // Expected RMS ≈ 4.47 at t=10. A single trajectory can be anywhere,
    // but |x| < 0.01 after 10k steps would indicate the thermostat isn't
    // injecting noise.
    //
    // We can't assert a tight bound on a single trajectory, but we CAN
    // assert it moved more than a trivial amount.
    let _ = final_x; // used for manual inspection if needed

    // Run 20 independent trajectories and check the mean-squared displacement.
    let n_traj = 20;
    let mut sum_x2: f64 = 0.0;
    for seed_offset in 0..n_traj {
        let mut model_i = sim_mjcf::load_model(RATCHET_XML).unwrap();
        let thermostat_i = LangevinThermostat::new(
            DVector::from_element(model_i.nv, GAMMA),
            K_B_T,
            SEED + seed_offset,
        );
        let ratchet_i = RatchetPotential::new(V1, V2, PHI, PERIOD, 0, 0);
        let stack_i = PassiveStack::builder()
            .with(thermostat_i)
            .with(ratchet_i)
            .build();
        stack_i.install(&mut model_i);
        let mut data_i = model_i.make_data();
        // ctrl stays at 0.0 (potential OFF)
        for _ in 0..n_steps {
            data_i.step(&model_i).unwrap();
        }
        sum_x2 += data_i.qpos[0] * data_i.qpos[0];
    }
    let msd = sum_x2 / (n_traj as f64);
    let n_steps_f = f64::from(n_steps);
    let expected_msd = 2.0 * (K_B_T / GAMMA) * (n_steps_f * 0.001);
    // Expected MSD = 2Dt = 2 × 1.0 × 10.0 = 20.0
    // Accept within factor of 3 (single-digit trajectories have high variance)
    assert!(
        msd > expected_msd * 0.3,
        "MSD too low: {msd:.2}, expected ~{expected_msd:.1}"
    );
    assert!(
        msd < expected_msd * 3.0,
        "MSD too high: {msd:.2}, expected ~{expected_msd:.1}"
    );
}

/// Zero-gain actuator produces zero force.
///
/// With the ratchet at α=0 and no thermostat noise, the particle should
/// remain at rest (no actuator force, no passive force).
#[test]
fn zero_gain_actuator_produces_no_force() {
    let model = sim_mjcf::load_model(RATCHET_XML).unwrap();

    // No PassiveStack — just the bare model with the zero-gain actuator.
    let mut data = model.make_data();

    // Set ctrl to various values — should produce zero force regardless.
    for &ctrl_val in &[0.0, 0.5, 1.0] {
        data.ctrl[0] = ctrl_val;
        data.step(&model).unwrap();
    }

    // Particle should not have moved (no forces, no gravity, no noise)
    assert!(
        data.qpos[0].abs() < 1e-15,
        "particle moved without any force: qpos = {}",
        data.qpos[0],
    );
}

/// Ctrl value persists across steps.
///
/// Set ctrl[0] = 1.0, step several times, verify the `RatchetPotential`
/// receives the same ctrl value each step (indirectly, by checking that
/// forces are applied).
#[test]
fn ctrl_persists_across_steps() {
    let mut model = sim_mjcf::load_model(RATCHET_XML).unwrap();

    let ratchet = RatchetPotential::new(V1, V2, PHI, PERIOD, 0, 0);
    // No thermostat — deterministic test.
    let stack = PassiveStack::builder().with(ratchet).build();
    stack.install(&mut model);

    let mut data = model.make_data();
    // Place particle at x = 0.1 (non-equilibrium position)
    data.qpos[0] = 0.1;
    data.forward(&model).unwrap();

    // Set ctrl once
    data.ctrl[0] = 1.0;

    // Step multiple times — force should be applied each step
    for _ in 0..100 {
        data.step(&model).unwrap();
    }

    // Particle should have moved from x = 0.1 (ratchet force pushes it)
    assert!(
        (data.qpos[0] - 0.1).abs() > 1e-6,
        "particle didn't move — ctrl may not be persisting: qpos = {}",
        data.qpos[0],
    );
}
