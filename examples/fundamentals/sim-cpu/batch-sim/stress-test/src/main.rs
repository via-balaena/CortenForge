//! Stress test — headless validation of the BatchSim API.
//!
//! 10 checks covering: construction, independent state, parallel stepping,
//! cross-contamination isolation, bitwise determinism, reset(i), reset_where,
//! reset_all, shared model, and single-env parity.
//!
//! Run: `cargo run -p example-batch-sim-stress-test --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::similar_names,
    clippy::needless_range_loop,
    clippy::len_zero,
    clippy::float_cmp
)]

use std::sync::Arc;

use sim_core::batch::BatchSim;
use sim_core::validation::{Check, print_report};

// ── MJCF ─────────────────────────────────────────────────────────────────

/// Single-link pendulum with a motor actuator (for ctrl tests).
const MJCF: &str = r#"
<mujoco model="batch-stress">
  <option gravity="0 0 -9.81" timestep="0.002" integrator="Euler">
    <flag energy="enable"/>
  </option>
  <worldbody>
    <body name="link" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0 0 -1" size="0.05" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="torque" joint="hinge" gear="1"/>
  </actuator>
</mujoco>
"#;

fn load() -> sim_core::Model {
    sim_mjcf::load_model(MJCF).expect("MJCF should parse")
}

// ── Checks ───────────────────────────────────────────────────────────────

/// Check 1: N environments created with correct count.
fn check_construction() -> Check {
    let model = Arc::new(load());
    let batch = BatchSim::new(Arc::clone(&model), 8);

    let len_ok = batch.len() == 8;
    let not_empty = !batch.is_empty();

    let empty = BatchSim::new(model, 0);
    let empty_ok = empty.len() == 0 && empty.is_empty();

    Check {
        name: "N environments created",
        pass: len_ok && not_empty && empty_ok,
        detail: format!(
            "len={}, is_empty={}, empty_len={}",
            batch.len(),
            batch.is_empty(),
            empty.len()
        ),
    }
}

/// Check 2: Environments have independent state.
fn check_independent_state() -> Check {
    let model = Arc::new(load());
    let mut batch = BatchSim::new(Arc::clone(&model), 4);

    // Perturb only env 0
    batch.env_mut(0).expect("env 0").qpos[0] = 1.0;

    let _errors = batch.step_all();

    let q0 = batch.env(0).expect("env 0").qpos[0];
    let q1 = batch.env(1).expect("env 1").qpos[0];

    // Env 0 started at 1.0 rad tilt, env 1 at 0.0 — after stepping they must differ
    let differ = (q0 - q1).abs() > 1e-10;

    Check {
        name: "Independent state",
        pass: differ,
        detail: format!(
            "env0 qpos={q0:.6}, env1 qpos={q1:.6}, diff={:.2e}",
            (q0 - q1).abs()
        ),
    }
}

/// Check 3: step_all advances all environments by one timestep.
fn check_step_all_advances() -> Check {
    let model = Arc::new(load());
    let mut batch = BatchSim::new(Arc::clone(&model), 8);
    let dt = model.timestep;

    let n_steps = 10;
    for _ in 0..n_steps {
        let _errors = batch.step_all();
    }

    let expected_time = n_steps as f64 * dt;
    let all_correct = (0..8).all(|i| {
        let t = batch.env(i).expect("env").time;
        (t - expected_time).abs() < 1e-12
    });

    Check {
        name: "step_all advances all",
        pass: all_correct,
        detail: format!(
            "expected t={expected_time:.6}, actual t[0]={:.6}",
            batch.env(0).expect("env 0").time
        ),
    }
}

/// Check 4: No cross-contamination — ctrl on env 0 doesn't affect env 1.
fn check_no_cross_contamination() -> Check {
    let model = Arc::new(load());
    let mut batch = BatchSim::new(Arc::clone(&model), 4);

    // Also create a standalone reference for env 1 (no ctrl applied)
    let mut standalone = model.make_data();

    // Apply ctrl only to env 0
    batch.env_mut(0).expect("env 0").ctrl[0] = 5.0;

    let n_steps = 20;
    for _ in 0..n_steps {
        let _errors = batch.step_all();
        standalone.step(&model).expect("step");
    }

    // Env 1 (no ctrl) should match standalone exactly (bitwise)
    let env1 = batch.env(1).expect("env 1");
    let qpos_match = env1.qpos == standalone.qpos;
    let qvel_match = env1.qvel == standalone.qvel;

    Check {
        name: "No cross-contamination",
        pass: qpos_match && qvel_match,
        detail: format!("qpos_match={qpos_match}, qvel_match={qvel_match}"),
    }
}

/// Check 5: Batch matches sequential stepping (bitwise determinism).
fn check_batch_matches_sequential() -> Check {
    let model = Arc::new(load());
    let n = 4;

    let mut batch = BatchSim::new(Arc::clone(&model), n);
    let mut sequential: Vec<sim_core::Data> = (0..n).map(|_| model.make_data()).collect();

    // Set distinct initial states
    let angles = [0.3, 0.7, 1.1, 1.5];
    for i in 0..n {
        batch.env_mut(i).expect("env").qpos[0] = angles[i];
        sequential[i].qpos[0] = angles[i];
    }

    // Step both 50 times
    for _ in 0..50 {
        let _errors = batch.step_all();
        for data in &mut sequential {
            data.step(&model).expect("step");
        }
    }

    // Compare all fields bitwise
    let mut all_match = true;
    let mut mismatch_detail = String::new();
    for i in 0..n {
        let b = batch.env(i).expect("env");
        let s = &sequential[i];
        if b.qpos != s.qpos {
            all_match = false;
            mismatch_detail = format!("env {i} qpos mismatch");
            break;
        }
        if b.qvel != s.qvel {
            all_match = false;
            mismatch_detail = format!("env {i} qvel mismatch");
            break;
        }
        if b.time != s.time {
            all_match = false;
            mismatch_detail = format!("env {i} time mismatch");
            break;
        }
        if b.energy_kinetic != s.energy_kinetic {
            all_match = false;
            mismatch_detail = format!("env {i} energy_kinetic mismatch");
            break;
        }
        if b.energy_potential != s.energy_potential {
            all_match = false;
            mismatch_detail = format!("env {i} energy_potential mismatch");
            break;
        }
    }

    Check {
        name: "Batch matches sequential",
        pass: all_match,
        detail: if all_match {
            format!("{n} envs, 50 steps, all fields bitwise equal")
        } else {
            mismatch_detail
        },
    }
}

/// Check 6: reset(i) resets only environment i.
fn check_reset_single() -> Check {
    let model = Arc::new(load());
    let mut batch = BatchSim::new(Arc::clone(&model), 4);

    // Tilt all and step
    for env in batch.envs_mut() {
        env.qpos[0] = 1.0;
    }
    for _ in 0..10 {
        let _errors = batch.step_all();
    }

    // Reset only env 2
    batch.reset(2);

    let env2 = batch.env(2).expect("env 2");
    let reset_time = env2.time == 0.0;
    let reset_qpos = env2.qpos == model.qpos0;
    let reset_qvel = env2.qvel.iter().all(|&v| v == 0.0);

    let env0 = batch.env(0).expect("env 0");
    let env0_untouched = env0.time > 0.0;

    let env3 = batch.env(3).expect("env 3");
    let env3_untouched = env3.time > 0.0;

    Check {
        name: "reset(i) resets only i",
        pass: reset_time && reset_qpos && reset_qvel && env0_untouched && env3_untouched,
        detail: format!(
            "env2: time={}, qpos_match={}, qvel_zero={}; env0: time={:.4}; env3: time={:.4}",
            env2.time, reset_qpos, reset_qvel, env0.time, env3.time
        ),
    }
}

/// Check 7: reset_where selectively resets masked environments.
fn check_reset_where_selective() -> Check {
    let model = Arc::new(load());
    let mut batch = BatchSim::new(Arc::clone(&model), 8);

    // Tilt all and step
    for env in batch.envs_mut() {
        env.qpos[0] = 1.0;
    }
    for _ in 0..10 {
        let _errors = batch.step_all();
    }

    // Reset even-indexed envs
    let mask = [true, false, true, false, true, false, true, false];
    batch.reset_where(&mask);

    let mut even_reset = true;
    let mut odd_untouched = true;
    for i in 0..8 {
        let env = batch.env(i).expect("env");
        if i % 2 == 0 {
            if env.time != 0.0 || env.qpos != model.qpos0 {
                even_reset = false;
            }
        } else if env.time == 0.0 {
            odd_untouched = false;
        }
    }

    Check {
        name: "reset_where selective",
        pass: even_reset && odd_untouched,
        detail: format!("even_reset={even_reset}, odd_untouched={odd_untouched}"),
    }
}

/// Check 8: reset_all resets every environment.
fn check_reset_all() -> Check {
    let model = Arc::new(load());
    let mut batch = BatchSim::new(Arc::clone(&model), 8);

    // Tilt all and step
    for env in batch.envs_mut() {
        env.qpos[0] = 1.0;
    }
    for _ in 0..10 {
        let _errors = batch.step_all();
    }

    batch.reset_all();

    let all_reset = batch.envs().enumerate().all(|(i, env)| {
        let time_ok = env.time == 0.0;
        let qpos_ok = env.qpos == model.qpos0;
        if !time_ok || !qpos_ok {
            eprintln!("env {i}: time={}, qpos_match={qpos_ok}", env.time);
        }
        time_ok && qpos_ok
    });

    Check {
        name: "reset_all resets everything",
        pass: all_reset,
        detail: format!("all 8 envs: time=0, qpos=qpos0: {all_reset}"),
    }
}

/// Check 9: Shared model accessible and consistent.
fn check_shared_model() -> Check {
    let model = Arc::new(load());
    let batch = BatchSim::new(Arc::clone(&model), 4);

    let nq_match = batch.model().nq == model.nq;
    let nv_match = batch.model().nv == model.nv;
    let nu_match = batch.model().nu == model.nu;
    let nbody_match = batch.model().nbody == model.nbody;

    Check {
        name: "Shared model",
        pass: nq_match && nv_match && nu_match && nbody_match,
        detail: format!(
            "nq={}/{}, nv={}/{}, nu={}/{}, nbody={}/{}",
            batch.model().nq,
            model.nq,
            batch.model().nv,
            model.nv,
            batch.model().nu,
            model.nu,
            batch.model().nbody,
            model.nbody,
        ),
    }
}

/// Check 10: Single-env batch matches standalone simulation (bitwise).
fn check_single_env_parity() -> Check {
    let model = Arc::new(load());
    let mut batch = BatchSim::new(Arc::clone(&model), 1);
    let mut standalone = model.make_data();

    // Same initial tilt
    batch.env_mut(0).expect("env 0").qpos[0] = 0.8;
    standalone.qpos[0] = 0.8;

    // Step both 50 times
    for _ in 0..50 {
        let _errors = batch.step_all();
        standalone.step(&model).expect("step");
    }

    let env = batch.env(0).expect("env 0");
    let qpos_match = env.qpos == standalone.qpos;
    let qvel_match = env.qvel == standalone.qvel;
    let time_match = env.time == standalone.time;
    let ke_match = env.energy_kinetic == standalone.energy_kinetic;
    let pe_match = env.energy_potential == standalone.energy_potential;

    Check {
        name: "Single-env matches standalone",
        pass: qpos_match && qvel_match && time_match && ke_match && pe_match,
        detail: format!(
            "qpos={qpos_match}, qvel={qvel_match}, time={time_match}, KE={ke_match}, PE={pe_match}"
        ),
    }
}

// ── Main ─────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Batch-Sim Stress Test ===\n");

    let checks = vec![
        check_construction(),
        check_independent_state(),
        check_step_all_advances(),
        check_no_cross_contamination(),
        check_batch_matches_sequential(),
        check_reset_single(),
        check_reset_where_selective(),
        check_reset_all(),
        check_shared_model(),
        check_single_env_parity(),
    ];

    let all_passed = print_report("Batch-Sim (10 checks)", &checks);

    if !all_passed {
        std::process::exit(1);
    }
}
