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
/// Per spec §7.1 option β + §7.3 two-level Welford pattern: 100
/// trajectories of 200,000 measurement steps each, after a 50,000-step
/// burn-in. Each trajectory's per-step ½v² is folded into a per-trajectory
/// `WelfordOnline` whose `mean()` becomes one IID sample of the
/// equilibrium ⟨½v²⟩. The 100 trajectory means are pushed into a top-level
/// `WelfordOnline` whose `std_error_of_mean()` is the correct std error
/// of the grand mean (the trajectory means are IID by construction —
/// independent seeds + sufficient burn-in). The grand mean is then
/// asserted to be within ±3σ of `½kT`, with std error ≈ 3.2% of `½kT`
/// per the spec §7.2 statistical accounting.
///
/// `n_burn_in = 5·τ_eq` and `n_measure = 20·τ_eq` where `τ_eq = M/(γh) =
/// 10_000` steps is the energy equilibration time. Burn-in scales with
/// `τ_eq`, NOT with `τ_int = M/(2γ)` (which is the v² autocorrelation
/// time, used only for `N_eff` sizing). See
/// `06_findings/2026-04-09_phase1_burn_in_tau_int_vs_tau_eq.md` for the
/// foundational distinction and Crack 4 propagation chain.
///
/// `WelfordOnline::merge` is intentionally NOT used here. See
/// `06_findings/2026-04-09_phase1_statistical_propagation_chain.md` for
/// the propagation-chain post-mortem.
#[test]
fn test_equipartition_central_parameter_set() {
    // n_burn_in = 5·τ_eq, n_measure = 20·τ_eq where τ_eq = M/(γh) = 10_000.
    let n_burn_in = 50_000;
    let n_measure = 200_000;
    let n_traj = 100_usize;
    let seed_base = 0x00C0_FFEE_u64;
    let k_b_t = 1.0_f64;
    let gamma_value = 0.1_f64;

    // Top-level accumulator over the 100 trajectory means. The trajectory
    // means are IID by construction, so std_error_of_mean on this
    // accumulator is the correct std error of the grand mean.
    let mut across_trajectories = WelfordOnline::new();

    for i in 0..n_traj {
        let mut model = sim_mjcf::load_model(SHO_1D_XML).expect("load");
        let mut data = model.make_data();

        let stack = PassiveStack::builder()
            .with(LangevinThermostat::new(
                DVector::from_element(model.nv, gamma_value),
                k_b_t,
                seed_base,
                i as u64,
            ))
            .build();
        stack.install(&mut model);

        // Burn in (no measurement).
        for _ in 0..n_burn_in {
            data.step(&model).expect("burn-in step");
        }

        // Inner accumulator: collect per-step ½v² for this trajectory.
        let mut traj = WelfordOnline::new();
        for _ in 0..n_measure {
            data.step(&model).expect("measure step");
            traj.push(0.5 * 1.0 * data.qvel[0] * data.qvel[0]);
        }

        // Push the trajectory mean as one IID sample. Within-trajectory
        // autocorrelation is absorbed into how much each trajectory mean
        // varies around the grand mean — exactly the variance the
        // top-level accumulator captures.
        across_trajectories.push(traj.mean());
    }

    let measured = across_trajectories.mean();
    let expected = 0.5 * k_b_t;
    let std_error = across_trajectories.std_error_of_mean();
    assert_within_n_sigma(
        measured,
        expected,
        std_error,
        3.0,
        "equipartition central: 1-DOF damped harmonic oscillator, 100 traj × 2×10⁵ steps",
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
///
/// **Per-combo `τ_eq` scaling.** `n_burn_in` and `n_measure` are computed
/// per combo as `5·τ_eq` and `20·τ_eq` where `τ_eq = M/(γh)` is the
/// energy equilibration time. A fixed step count would leave the slow-γ
/// rows catastrophically un-equilibrated — see
/// `06_findings/2026-04-09_phase1_burn_in_tau_int_vs_tau_eq.md` for the
/// quantitative analysis (`γ=0.01` with fixed `25k` burn-in produces a
/// `49%`-of-`½kT` systematic bias).
#[test]
#[ignore = "order-of-minutes runtime — opt-in via --ignored; central combo is gated by test_equipartition_central_parameter_set"]
fn test_equipartition_sweep_gamma_t() {
    let n_traj_per_combo = 30_usize;
    let seed_base = 0xBEEF_CAFE_u64;

    // Per-combo (γ, τ_eq_in_steps) pairs. τ_eq = M/(γh) = 1/(γh) for M=1
    // and the Phase 1 fixture h=0.001 (locked in §6 + §7.2). Storing the
    // pair lets us avoid an f64→usize cast that would trip clippy without
    // adding a #[allow] override; the values are exact integers anyway.
    let gamma_tau_eq_pairs: [(f64, usize); 3] = [(0.01, 100_000), (0.10, 10_000), (1.00, 1_000)];
    let temperatures = [0.5_f64, 1.0, 2.0];

    for &(gamma_value, tau_eq_steps) in &gamma_tau_eq_pairs {
        let n_burn_in = 5 * tau_eq_steps;
        let n_measure = 20 * tau_eq_steps;

        for &k_b_t in &temperatures {
            // Two-level Welford pattern per spec §7.4 + §7.3 — fresh
            // top-level accumulator per (γ, kT) combination.
            let mut across_trajectories = WelfordOnline::new();

            for i in 0..n_traj_per_combo {
                let mut model = sim_mjcf::load_model(SHO_1D_XML).expect("load");
                let mut data = model.make_data();

                let stack = PassiveStack::builder()
                    .with(LangevinThermostat::new(
                        DVector::from_element(model.nv, gamma_value),
                        k_b_t,
                        seed_base,
                        i as u64,
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
                across_trajectories.push(traj.mean());
            }

            let measured = across_trajectories.mean();
            let expected = 0.5 * k_b_t;
            let std_error = across_trajectories.std_error_of_mean();
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
                0,
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
        // Reproducibility pair: both thermostats use identical
        // (master_seed, traj_id) — Ch 40 §3.4 (b).
        .with(LangevinThermostat::new(
            DVector::from_element(model1.nv, 0.1),
            1.0,
            seed,
            0,
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
            0,
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
/// 3. Run 200,000 Euler steps (10× the amplitude time constant
///    `2M/γ = 20`) — the system decays to envelope ≈ `exp(-10) ≈
///    4.5e-5`, ~22× margin under the `< 1e-3` threshold.
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
            0,
        ))
        .build();
    stack.install(&mut model);

    // Initial state: stretched spring at rest.
    data.qpos[0] = 1.0;
    data.qvel[0] = 0.0;

    // Disable noise for the decay phase.
    {
        let _guard = stack.disable_stochastic();
        // Amplitude time constant of the underdamped oscillator is
        // 2M/γ = 20 time units = 20_000 steps. Run 10× that to land
        // in numerical-floor territory: envelope ≈ exp(-10) ≈ 4.5e-5,
        // ~22× margin under the 1e-3 threshold below.
        //
        // Note: M/γ = 10 would be the time constant for free-particle
        // velocity decay or for underdamped-oscillator ENERGY decay
        // (energy ∝ amplitude², so energy decays at twice the
        // amplitude rate). Neither matches what we check here — we
        // check qpos and qvel, both of which scale with amplitude,
        // so the relevant time constant is 2M/γ = 20.
        for _ in 0..200_000 {
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
