//! Phase 2 tests: `ThermCircuitEnvBuilder` -> `ThermCircuitEnv` (single env).
//!
//! Gates per spec §7 Phase 2:
//! - Reset returns obs of correct dim
//! - Step with zero action advances time
//! - Reward closure fires correctly
//! - Truncation triggers at `episode_steps`
//! - Double-well forces are applied (particle moves toward well)
//! - Ctrl-temperature modulates noise (deterministic damping check)
//! - Domain accessors: `n_particles`, `config_k_b_t`, `effective_temperature`
//! - Builder validation: missing reward -> error, zero particles -> error
//! - Multiple landscape components stack correctly

#![allow(clippy::unwrap_used)]

use approx::assert_abs_diff_eq;
use sim_ml_chassis::Environment;
use sim_ml_chassis::Tensor;
use sim_therm_env::{ThermCircuitEnv, ThermCircuitError};
use sim_thermostat::DoubleWellPotential;

// ── Helpers ──────────────────────────────────────────────────────────────

/// Build a minimal 1-particle env with a trivial reward.
fn make_1p_env() -> ThermCircuitEnv {
    ThermCircuitEnv::builder(1)
        .reward(|_m, _d| 1.0)
        .build()
        .unwrap()
}

// ── 1. Reset returns obs of correct dim ─────────────────────────────────

#[test]
fn reset_returns_correct_obs_dim() {
    let mut env = make_1p_env();
    let obs = env.reset().unwrap();
    // 1 particle: obs = [qpos(1), qvel(1)] = dim 2
    assert_eq!(obs.shape(), &[2], "obs dim should be 2*n_particles");
}

#[test]
fn reset_returns_correct_obs_dim_4_particles() {
    let mut env = ThermCircuitEnv::builder(4)
        .reward(|_m, _d| 0.0)
        .build()
        .unwrap();
    let obs = env.reset().unwrap();
    // 4 particles: obs = [qpos(4), qvel(4)] = dim 8
    assert_eq!(obs.shape(), &[8], "obs dim should be 2*n_particles");
}

// ── 2. Step with zero action advances time ──────────────────────────────

#[test]
fn step_advances_time() {
    let mut env = make_1p_env();
    env.reset().unwrap();
    let action = Tensor::zeros(&[0]); // no actuators when ctrl_temperature is off
    env.step(&action).unwrap();
    assert!(env.data().time > 0.0, "time should advance after step");
}

// ── 3. Reward closure fires correctly ───────────────────────────────────

#[test]
fn reward_fires_correctly() {
    let mut env = ThermCircuitEnv::builder(1)
        .reward(|_m, _d| 42.0)
        .build()
        .unwrap();
    env.reset().unwrap();
    let result = env.step(&Tensor::zeros(&[0])).unwrap();
    assert_abs_diff_eq!(result.reward, 42.0, epsilon = 1e-12);
}

#[test]
fn reward_reads_state() {
    // Reward = -qpos[0]^2: should be near zero after reset
    let mut env = ThermCircuitEnv::builder(1)
        .reward(|_m, d| -(d.qpos[0] * d.qpos[0]))
        .build()
        .unwrap();
    env.reset().unwrap();
    let result = env.step(&Tensor::zeros(&[0])).unwrap();
    // qpos starts near zero → reward near zero
    assert!(result.reward.abs() < 1.0, "reward should read qpos");
}

// ── 4. Truncation triggers at episode_steps ─────────────────────────────

#[test]
fn truncation_triggers_at_episode_limit() {
    // Short episode: 5 steps, sub_steps=1, timestep=0.001
    // max_time = 5 * 1 * 0.001 = 0.005
    let mut env = ThermCircuitEnv::builder(1)
        .episode_steps(5)
        .sub_steps(1)
        .timestep(0.001)
        .reward(|_m, _d| 0.0)
        .build()
        .unwrap();
    env.reset().unwrap();
    let action = Tensor::zeros(&[0]);

    // Steps 1-5 should not be truncated (time <= 0.005)
    for i in 1..=5 {
        let result = env.step(&action).unwrap();
        assert!(
            !result.truncated,
            "step {i} should not be truncated (time={})",
            env.data().time
        );
    }

    // Step 6 pushes time past max_time
    let result = env.step(&action).unwrap();
    assert!(
        result.truncated,
        "step 6 should trigger truncation (time={})",
        env.data().time
    );
}

// ── 5. Double-well forces are applied ───────────────────────────────────

#[test]
fn double_well_forces_move_particle_toward_well() {
    // Place particle at origin in a double well (minima at ±1).
    // The barrier force at x=0 is zero, but at x=+0.5 the force
    // pushes toward x=+1. We'll use on_reset to place particle
    // at x=+0.5 and verify it moves toward x=+1.
    let mut env = ThermCircuitEnv::builder(1)
        .gamma(0.0) // no damping for cleaner force test
        .seed(42)
        .with(DoubleWellPotential::new(3.0, 1.0, 0))
        .sub_steps(100)
        .timestep(0.001)
        .reward(|_m, _d| 0.0)
        .on_reset(|_m, data| {
            data.qpos[0] = 0.5;
        })
        .build()
        .unwrap();

    env.reset().unwrap();
    let initial_x = env.data().qpos[0];
    assert_abs_diff_eq!(initial_x, 0.5, epsilon = 1e-10);

    // Step — particle at x=0.5 in V(x)=a(x²-1)² should feel
    // restoring force F = -4a·x·(x²-1) = -4·3·0.5·(0.25-1) = +4.5
    // pointing toward x=+1.
    let action = Tensor::zeros(&[0]);
    env.step(&action).unwrap();
    let final_x = env.data().qpos[0];

    assert!(
        final_x > initial_x,
        "particle should move toward +1 well: initial={initial_x}, final={final_x}"
    );
}

// ── 6. Ctrl-temperature modulates noise ─────────────────────────────────

#[test]
fn ctrl_temperature_damping_check() {
    // With ctrl_temperature enabled, ctrl[0]=0 → pure damping (kT_eff=0).
    // Place particle with qvel>0, step, verify it slows down from damping.
    let mut env = ThermCircuitEnv::builder(1)
        .gamma(10.0)
        .k_b_t(1.0)
        .with_ctrl_temperature()
        .timestep(0.001)
        .sub_steps(100)
        .reward(|_m, _d| 0.0)
        .on_reset(|_m, data| {
            data.qvel[0] = 1.0;
        })
        .build()
        .unwrap();

    env.reset().unwrap();
    assert_abs_diff_eq!(env.data().qvel[0], 1.0, epsilon = 1e-10);

    // ctrl[0] = 0 → zero effective temperature → pure damping, no noise
    let action = Tensor::from_slice(&[0.0_f32], &[1]);
    env.step(&action).unwrap();

    // With gamma=10 and 100 sub-steps at h=0.001, the particle should
    // slow significantly (exponential damping: v ≈ v₀·exp(-γ·t) ≈ exp(-1))
    let v_after = env.data().qvel[0];
    assert!(
        v_after < 0.5,
        "particle should slow from damping: initial=1.0, after={v_after}"
    );
    assert!(
        v_after > 0.0,
        "particle should still be moving forward: after={v_after}"
    );
}

#[test]
fn ctrl_temperature_env_has_action_dim_1() {
    let env = ThermCircuitEnv::builder(1)
        .with_ctrl_temperature()
        .reward(|_m, _d| 0.0)
        .build()
        .unwrap();
    assert_eq!(
        env.action_space().dim(),
        1,
        "ctrl_temperature allocates 1 ctrl"
    );
}

// ── 7. Domain accessors ─────────────────────────────────────────────────

#[test]
fn domain_accessors() {
    let env = ThermCircuitEnv::builder(3)
        .k_b_t(2.5)
        .reward(|_m, _d| 0.0)
        .build()
        .unwrap();
    assert_eq!(env.n_particles(), 3);
    assert_abs_diff_eq!(env.config_k_b_t(), 2.5, epsilon = 1e-15);
}

#[test]
fn effective_temperature_without_ctrl() {
    let env = ThermCircuitEnv::builder(1)
        .k_b_t(2.0)
        .reward(|_m, _d| 0.0)
        .build()
        .unwrap();
    assert_abs_diff_eq!(env.effective_temperature(), 2.0, epsilon = 1e-15);
}

#[test]
fn effective_temperature_with_ctrl() {
    let mut env = ThermCircuitEnv::builder(1)
        .k_b_t(1.0)
        .with_ctrl_temperature()
        .reward(|_m, _d| 0.0)
        .build()
        .unwrap();
    env.reset().unwrap();

    // ctrl starts at 0 → effective temp = 0
    assert_abs_diff_eq!(env.effective_temperature(), 0.0, epsilon = 1e-15);

    // Step with ctrl=3.0 → effective temp = 3.0
    let action = Tensor::from_slice(&[3.0_f32], &[1]);
    env.step(&action).unwrap();
    assert_abs_diff_eq!(env.effective_temperature(), 3.0, epsilon = 1e-10);
}

// ── 8. Builder validation ───────────────────────────────────────────────

#[test]
fn builder_missing_reward_returns_error() {
    let err = ThermCircuitEnv::builder(1).build().unwrap_err();
    assert!(
        matches!(err, ThermCircuitError::MissingField { field: "reward" }),
        "expected MissingField(reward), got {err:?}"
    );
}

#[test]
fn builder_zero_particles_returns_error() {
    let err = ThermCircuitEnv::builder(0)
        .reward(|_m, _d| 0.0)
        .build()
        .unwrap_err();
    assert!(
        matches!(err, ThermCircuitError::ZeroParticles),
        "expected ZeroParticles, got {err:?}"
    );
}

// ── 9. Multiple landscape components stack correctly ────────────────────

#[test]
fn multiple_landscape_components_stack() {
    // Two double wells on different DOFs of a 2-particle system.
    // Particle 0 starts at x=+0.5, particle 1 starts at x=-0.5.
    // Each should be pushed toward its respective +1 / -1 well.
    let mut env = ThermCircuitEnv::builder(2)
        .gamma(0.0)
        .seed(42)
        .with(DoubleWellPotential::new(3.0, 1.0, 0)) // DOF 0
        .with(DoubleWellPotential::new(3.0, 1.0, 1)) // DOF 1
        .sub_steps(100)
        .timestep(0.001)
        .reward(|_m, _d| 0.0)
        .on_reset(|_m, data| {
            data.qpos[0] = 0.5;
            data.qpos[1] = -0.5;
        })
        .build()
        .unwrap();

    env.reset().unwrap();
    let action = Tensor::zeros(&[0]);
    env.step(&action).unwrap();

    let x0 = env.data().qpos[0];
    let x1 = env.data().qpos[1];

    assert!(x0 > 0.5, "particle 0 should move toward +1 well: x0={x0}");
    assert!(x1 < -0.5, "particle 1 should move toward -1 well: x1={x1}");
}

// ── Phase 1 regression tests (moved from lib.rs) ───────────────────────

mod mjcf {
    use approx::assert_abs_diff_eq;
    use sim_therm_env::generate_mjcf;

    #[test]
    fn mjcf_parses_1_particle() {
        let xml = generate_mjcf(1, 1, 0.001);
        sim_mjcf::load_model(&xml).unwrap();
    }

    #[test]
    fn mjcf_parses_2_particles() {
        let xml = generate_mjcf(2, 1, 0.001);
        sim_mjcf::load_model(&xml).unwrap();
    }

    #[test]
    fn mjcf_parses_8_particles() {
        let xml = generate_mjcf(8, 4, 0.001);
        sim_mjcf::load_model(&xml).unwrap();
    }

    #[test]
    fn dimensions_1_particle_1_ctrl() {
        let xml = generate_mjcf(1, 1, 0.001);
        let model = sim_mjcf::load_model(&xml).unwrap();
        assert_eq!(model.nq, 1, "nq");
        assert_eq!(model.nv, 1, "nv");
        assert_eq!(model.nu, 1, "nu");
    }

    #[test]
    fn dimensions_2_particles_1_ctrl() {
        let xml = generate_mjcf(2, 1, 0.001);
        let model = sim_mjcf::load_model(&xml).unwrap();
        assert_eq!(model.nq, 2, "nq");
        assert_eq!(model.nv, 2, "nv");
        assert_eq!(model.nu, 1, "nu");
    }

    #[test]
    fn dimensions_8_particles_4_ctrl() {
        let xml = generate_mjcf(8, 4, 0.001);
        let model = sim_mjcf::load_model(&xml).unwrap();
        assert_eq!(model.nq, 8, "nq");
        assert_eq!(model.nv, 8, "nv");
        assert_eq!(model.nu, 4, "nu");
    }

    #[test]
    fn dimensions_no_actuators() {
        let xml = generate_mjcf(3, 0, 0.001);
        let model = sim_mjcf::load_model(&xml).unwrap();
        assert_eq!(model.nq, 3, "nq");
        assert_eq!(model.nv, 3, "nv");
        assert_eq!(model.nu, 0, "nu");
    }

    #[test]
    fn joint_names() {
        let xml = generate_mjcf(3, 0, 0.001);
        let model = sim_mjcf::load_model(&xml).unwrap();
        let names: Vec<Option<&str>> = model.jnt_name.iter().map(|n| n.as_deref()).collect();
        assert_eq!(names, vec![Some("x0"), Some("x1"), Some("x2")]);
    }

    #[test]
    fn actuator_names() {
        let xml = generate_mjcf(3, 2, 0.001);
        let model = sim_mjcf::load_model(&xml).unwrap();
        let names: Vec<Option<&str>> = model.actuator_name.iter().map(|n| n.as_deref()).collect();
        assert_eq!(names, vec![Some("ctrl_0"), Some("ctrl_1")]);
    }

    #[test]
    fn zero_gain_actuator_produces_zero_force() {
        let xml = generate_mjcf(1, 1, 0.001);
        let model = sim_mjcf::load_model(&xml).unwrap();
        let mut data = model.make_data();
        data.ctrl[0] = 5.0;
        data.step(&model).unwrap();
        assert_abs_diff_eq!(data.qpos[0], 0.0, epsilon = 1e-15);
        assert_abs_diff_eq!(data.qvel[0], 0.0, epsilon = 1e-15);
        assert_abs_diff_eq!(data.qfrc_actuator[0], 0.0, epsilon = 1e-15);
    }

    #[test]
    fn custom_timestep() {
        let xml = generate_mjcf(1, 0, 0.005);
        let model = sim_mjcf::load_model(&xml).unwrap();
        assert_abs_diff_eq!(model.timestep, 0.005, epsilon = 1e-15);
    }
}
