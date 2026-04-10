//! Phase 1 integration tests — the file the entire crate exists to enable.
//!
//! Per Phase 1 spec §6 (model invariants), §7 (equipartition gate),
//! §8 (callback firing-count), §9 (reproducibility from seed), and
//! §10 (stochastic gating sanity). The §7 equipartition test is THE
//! Phase 1 deliverable — passing it is the entrance condition for
//! every later phase.
//!
//! These tests compile against `sim_thermostat` as a normal library
//! consumer (integration tests live outside the lib's `src/` tree),
//! which is why the imports come through the public API (`PassiveStack`,
//! `LangevinThermostat`, `test_utils::*`) rather than `crate::`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]

use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};

use sim_core::{DVector, Data, Integrator, Model};
use sim_thermostat::test_utils::{WelfordOnline, assert_within_n_sigma};
use sim_thermostat::{LangevinThermostat, PassiveComponent, PassiveStack};

/// The Phase 1 1-DOF damped harmonic oscillator MJCF.
///
/// Single particle on a slide joint, unit mass, unit spring stiffness,
/// zero damping (the thermostat owns damping per Q4 option (a) — the
/// model's `dof_damping[0]` MUST be zero or the model damping would
/// compound with the thermostat damping and silently shift the
/// equilibrium temperature).
const SHO_1D_XML: &str = r#"
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
"#;

// ─── §6 model-invariants fixture ──────────────────────────────────────

/// Verifies the loaded model satisfies every property from spec §6's
/// invariant table. If this fails, the spec is no longer satisfiable
/// on this model and the rest of the suite is meaningless. Crash loud
/// rather than silently miscompare downstream.
#[test]
fn test_sho_model_invariants() {
    let model = sim_mjcf::load_model(SHO_1D_XML).expect("MJCF should load");

    assert_eq!(model.nv, 1, "model.nv must be 1");
    assert_eq!(model.nq, 1, "model.nq must be 1");
    assert_eq!(model.timestep, 0.001, "model.timestep must be 0.001");
    assert!(
        matches!(model.integrator, Integrator::Euler),
        "model.integrator must be Euler, got {:?}",
        model.integrator,
    );

    // Body 0 is the world body, body 1 is the user body. The user
    // body must have unit mass.
    assert_eq!(model.body_mass[1], 1.0, "moving body mass must be 1.0");

    // Critical: dof_damping[0] must be zero. The thermostat owns
    // damping (Q4 option (a) from recon log part 2). Model damping
    // would compound with thermostat damping and silently shift
    // the equilibrium temperature.
    assert_eq!(
        model.dof_damping[0], 0.0,
        "model.dof_damping[0] must be zero — thermostat owns damping",
    );

    // Linear restoring force sanity: stretch the spring to qpos=0.5,
    // call forward (computes qacc without integrating), and assert
    // qacc[0] ≈ -0.5. Spring force is F = −k·(x − springref) =
    // −1·(0.5 − 0) = −0.5; mass is 1.0; so qacc = −0.5.
    let mut data = model.make_data();
    data.qpos[0] = 0.5;
    data.qvel[0] = 0.0;
    data.forward(&model).expect("forward should succeed");
    assert!(
        (data.qacc[0] - (-0.5)).abs() < 1e-12,
        "spring force sanity: expected qacc[0] = -0.5, got {}",
        data.qacc[0],
    );
}

// ─── §7 equipartition gate (THE Phase 1 deliverable) ─────────────────

/// THE Phase 1 gate — equipartition on the central parameter set.
///
/// Per spec §7.1 option β: 100 trajectories of 100,000 measurement
/// steps each, after a 25,000-step burn-in. Each trajectory's
/// per-step ½v² is accumulated into a per-trajectory `WelfordOnline`,
/// then merged into a global accumulator via the Chan/Pébay
/// parallel-merge formula (chassis M4). The global accumulator's
/// mean is asserted to be within ±3σ of the equipartition target
/// `½kT`, with std error ≈ 4.5% of `½kT` per the spec §7.2
/// statistical accounting.
#[test]
fn test_equipartition_central_parameter_set() {
    let n_burn_in = 25_000;
    let n_measure = 100_000;
    let n_traj = 100_usize;
    let seed_base = 0x00C0_FFEE_u64;
    let k_b_t = 1.0_f64;
    let gamma_value = 0.1_f64;

    let mut global = WelfordOnline::new();

    for i in 0..n_traj {
        let mut model = sim_mjcf::load_model(SHO_1D_XML).expect("load");
        let mut data = model.make_data();

        let stack = PassiveStack::builder()
            .with(LangevinThermostat::new(
                DVector::from_element(model.nv, gamma_value),
                k_b_t,
                seed_base + i as u64,
            ))
            .build();
        stack.install(&mut model);

        // Burn in (no measurement).
        for _ in 0..n_burn_in {
            data.step(&model).expect("burn-in step");
        }

        // Measure ½ M v² into a per-trajectory accumulator.
        let mut traj = WelfordOnline::new();
        for _ in 0..n_measure {
            data.step(&model).expect("measure step");
            traj.push(0.5 * 1.0 * data.qvel[0] * data.qvel[0]);
        }

        // Merge into the global accumulator (Chan/Pébay, M4).
        global.merge(&traj);
    }

    let measured = global.mean();
    let expected = 0.5 * k_b_t;
    let std_error = global.std_error_of_mean();
    assert_within_n_sigma(
        measured,
        expected,
        std_error,
        3.0,
        "equipartition central: 1-DOF damped harmonic oscillator, 100 traj × 10⁵ steps",
    );
}

// ─── §7.4 sweep (separate test, not the gate) ────────────────────────

/// 3×3 sweep over γ and `k_B·T`. Verifies T-linearity (`⟨½v²⟩ ∝ T`)
/// and γ-independence (the stationary temperature must not depend on
/// γ — γ controls relaxation time, not equilibrium temperature).
///
/// Marked `#[ignore]` per spec §7.4 because the central combination
/// is already proven by `test_equipartition_central_parameter_set`,
/// the sweep is verification rather than the gate, and total cost
/// is order-of-minutes vs order-of-seconds. Run with
/// `cargo test -p sim-thermostat -- --ignored test_equipartition_sweep_gamma_T`
/// when manually verifying after touching the thermostat algorithm.
#[test]
#[ignore = "order-of-minutes runtime — opt-in via --ignored; central combo is gated by test_equipartition_central_parameter_set"]
fn test_equipartition_sweep_gamma_t() {
    let n_burn_in = 25_000;
    let n_measure = 100_000;
    let n_traj_per_combo = 30_usize;
    let seed_base = 0xBEEF_CAFE_u64;

    let gammas = [0.01_f64, 0.1, 1.0];
    let temperatures = [0.5_f64, 1.0, 2.0];

    for &gamma_value in &gammas {
        for &k_b_t in &temperatures {
            let mut global = WelfordOnline::new();

            for i in 0..n_traj_per_combo {
                let mut model = sim_mjcf::load_model(SHO_1D_XML).expect("load");
                let mut data = model.make_data();

                let stack = PassiveStack::builder()
                    .with(LangevinThermostat::new(
                        DVector::from_element(model.nv, gamma_value),
                        k_b_t,
                        seed_base + i as u64,
                    ))
                    .build();
                stack.install(&mut model);

                for _ in 0..n_burn_in {
                    data.step(&model).expect("burn-in");
                }
                let mut traj = WelfordOnline::new();
                for _ in 0..n_measure {
                    data.step(&model).expect("measure");
                    traj.push(0.5 * 1.0 * data.qvel[0] * data.qvel[0]);
                }
                global.merge(&traj);
            }

            let measured = global.mean();
            let expected = 0.5 * k_b_t;
            let std_error = global.std_error_of_mean();
            let description = format!("equipartition sweep: γ={gamma_value}, kT={k_b_t}");
            assert_within_n_sigma(measured, expected, std_error, 3.0, &description);
        }
    }
}

// ─── §8 callback firing-count test ────────────────────────────────────

/// A thin diagnostic wrapper that increments a counter inside `apply`
/// before delegating to the inner thermostat. Per spec §8 — establishes
/// the workspace's first hard-equality precedent for stateful
/// `cb_passive` invocation counts.
struct CountingWrapper {
    counter: Arc<AtomicUsize>,
    inner: LangevinThermostat,
}

impl PassiveComponent for CountingWrapper {
    fn apply(&self, model: &Model, data: &Data, qfrc_out: &mut DVector<f64>) {
        self.counter.fetch_add(1, Ordering::SeqCst);
        self.inner.apply(model, data, qfrc_out);
    }
    // No as_stochastic override — §8 does not exercise gating, and a
    // wrapper that hides the inner thermostat's stochastic state is
    // a separate concern (a Phase 5+ wrapper-composition question).
}

/// Per spec §8: `cb_passive` fires exactly once per `data.step` call.
/// Hard equality, not approximate. Future stateful `cb_passive`
/// consumers can copy this pattern.
#[test]
fn test_callback_firing_count() {
    let mut model = sim_mjcf::load_model(SHO_1D_XML).expect("load");
    let mut data = model.make_data();

    let counter = Arc::new(AtomicUsize::new(0));
    let stack = PassiveStack::builder()
        .with(CountingWrapper {
            counter: Arc::clone(&counter),
            inner: LangevinThermostat::new(
                DVector::from_element(model.nv, 0.1),
                1.0,
                0x00C0_FFEE_u64,
            ),
        })
        .build();
    stack.install(&mut model);

    let k_steps = 1_000;
    for _ in 0..k_steps {
        data.step(&model).expect("step");
    }

    assert_eq!(
        counter.load(Ordering::SeqCst),
        k_steps,
        "cb_passive should fire exactly once per data.step (hard equality, no flake)",
    );
}

// ─── §9 reproducibility from seed ────────────────────────────────────

/// Two simulations with identical MJCF, identical parameters, and
/// identical seeds must produce bit-for-bit identical `qpos` and
/// `qvel` after N steps. Hard f64 equality, no `assert_relative_eq!`.
///
/// A failure here means either the RNG implementation drifted
/// (catastrophic — the entire reproducibility-as-foundation argument
/// from recon log part 6 breaks) or the chassis introduced
/// nondeterminism somewhere it shouldn't have (e.g., a `HashMap`
/// iteration in `PassiveStack::install`). Either is a stop-the-line
/// bug.
#[test]
fn test_reproducibility_from_seed() {
    let seed = 0x00C0_FFEE_u64;
    let n_steps = 10_000;

    let mut model1 = sim_mjcf::load_model(SHO_1D_XML).expect("load 1");
    let mut data1 = model1.make_data();
    let stack1 = PassiveStack::builder()
        .with(LangevinThermostat::new(
            DVector::from_element(model1.nv, 0.1),
            1.0,
            seed,
        ))
        .build();
    stack1.install(&mut model1);

    let mut model2 = sim_mjcf::load_model(SHO_1D_XML).expect("load 2");
    let mut data2 = model2.make_data();
    let stack2 = PassiveStack::builder()
        .with(LangevinThermostat::new(
            DVector::from_element(model2.nv, 0.1),
            1.0,
            seed,
        ))
        .build();
    stack2.install(&mut model2);

    for _ in 0..n_steps {
        data1.step(&model1).expect("sim 1 step");
        data2.step(&model2).expect("sim 2 step");
    }

    assert_eq!(
        data1.qpos[0], data2.qpos[0],
        "reproducibility: qpos[0] must match bit-for-bit after {n_steps} steps",
    );
    assert_eq!(
        data1.qvel[0], data2.qvel[0],
        "reproducibility: qvel[0] must match bit-for-bit after {n_steps} steps",
    );
}

// ─── §10 stochastic gating sanity ────────────────────────────────────

/// Per spec §10: prove that `disable_stochastic()` zeroes the noise
/// contribution end-to-end before the FD/autograd code paths in
/// Phase 5+ depend on this property.
///
/// 1. Stretch the spring to `qpos = 1.0`, set `qvel = 0`.
/// 2. Disable stochastic via the RAII guard.
/// 3. Run 50,000 Euler steps (5× the M/γ time constant) — the
///    system should decay to near-rest, qpos and qvel both `< 1e-3`.
/// 4. Drop the guard (re-enables noise).
/// 5. Run one more step — qvel should jump above 1e-3 from the
///    re-engaged FDT noise.
#[test]
fn test_stochastic_gating_sanity() {
    let mut model = sim_mjcf::load_model(SHO_1D_XML).expect("load");
    let mut data = model.make_data();

    let stack = PassiveStack::builder()
        .with(LangevinThermostat::new(
            DVector::from_element(model.nv, 0.1),
            1.0,
            0x00C0_FFEE_u64,
        ))
        .build();
    stack.install(&mut model);

    // Initial state: stretched spring at rest.
    data.qpos[0] = 1.0;
    data.qvel[0] = 0.0;

    // Disable noise for the decay phase.
    {
        let _guard = stack.disable_stochastic();
        // Damping time constant M/γ = 10 time units = 10_000 steps.
        // Run 5× that to land in numerical-floor territory.
        for _ in 0..50_000 {
            data.step(&model).expect("decay step");
        }
        assert!(
            data.qvel[0].abs() < 1e-3 && data.qpos[0].abs() < 1e-3,
            "decayed state should be near rest, got qpos={}, qvel={}",
            data.qpos[0],
            data.qvel[0],
        );
    } // guard drops here, stochastic re-enabled

    // One step with noise re-enabled. Expected kick magnitude ≈ σ·h
    // = sqrt(2γkT/h)·h = sqrt(2·0.1·1·1000)·0.001 ≈ 0.014. We assert
    // a threshold an order of magnitude below the kick to remain
    // robust against single-sample variance while still catching
    // "noise didn't re-engage" failures.
    data.step(&model).expect("post-guard step");
    assert!(
        data.qvel[0].abs() > 1e-3,
        "guard drop should re-energize the system; got qvel={}",
        data.qvel[0],
    );
}
