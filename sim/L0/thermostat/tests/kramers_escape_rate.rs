//! Phase 3 — Kramers escape-rate validation tests.
//!
//! Validates that a 1-DOF particle in a quartic double-well potential
//! under the Langevin thermostat exhibits switching dynamics consistent
//! with Kramers' escape-rate formula.
//!
//! Tests:
//! - Gate A: absolute Kramers rate at central parameters (must-pass)
//! - Gate B: Arrhenius slope over kT sweep (must-pass, `#[ignore]`)
//! - Supporting: Boltzmann position distribution shape
//! - Supporting: Reproducibility from seed
//!
//! Spec: `docs/thermo_computing/03_phases/03_bistable_kramers.md`

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::float_cmp,
    clippy::doc_markdown,
    clippy::similar_names,
    clippy::suboptimal_flops,
    clippy::suspicious_operation_groupings
)]

use sim_core::DVector;
use sim_thermostat::test_utils::{WelfordOnline, WellState};
use sim_thermostat::{DoubleWellPotential, LangevinThermostat, PassiveStack};

// ─── MJCF model ───────────────────��────────────────────────────────────────

const BISTABLE_XML: &str = r#"
<mujoco model="bistable_1dof">
  <option timestep="0.001" gravity="0 0 0" integrator="Euler"/>
  <worldbody>
    <body name="particle">
      <joint name="x" type="slide" axis="1 0 0"
             stiffness="0" damping="0" springref="0" ref="0"/>
      <geom type="sphere" size="0.05" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"#;

// ─── Central parameter set (spec §7) ───────────────────────────────────────

const DELTA_V: f64 = 3.0;
const X_0: f64 = 1.0;
const MASS: f64 = 1.0;
const GAMMA: f64 = 10.0;
const K_B_T: f64 = 1.0;
const SEED_BASE: u64 = 20_260_410;

// ─── Test configuration ─────────────���──────────────────────────────────────

const N_STEPS_PER_TRAJ: usize = 5_000_000;
const N_BURN_IN: usize = 10_000;
const N_MEASURE: usize = N_STEPS_PER_TRAJ - N_BURN_IN;
const N_TRAJ_GATE_A: usize = 30;
const TIMESTEP: f64 = 0.001;
const T_MEASURE: f64 = N_MEASURE as f64 * TIMESTEP; // 4990 time units

// ─── Helpers ────────────────────────────────────────────────────��──────────

/// Hysteresis threshold for transition counting.
/// Only counts a transition when the particle moves from one well region
/// (|x| > X_THRESH) to the other. This filters out recrossings near the
/// barrier where the particle oscillates through x=0 without committing.
///
/// At κ = λ_r/ω_b = 0.313, ~69% of zero-crossings are recrossings.
/// Hysteresis is essential for measuring the genuine Kramers rate.
const X_THRESH: f64 = X_0 / 2.0; // 0.5

/// Run a trajectory and count committed transitions using hysteresis.
/// Returns (transition_count, ke_welford).
fn run_trajectory(k_b_t: f64, seed: u64) -> (usize, WelfordOnline) {
    let mut model = sim_mjcf::load_model(BISTABLE_XML).expect("load");
    let mut data = model.make_data();

    PassiveStack::builder()
        .with(DoubleWellPotential::new(DELTA_V, X_0, 0))
        .with(LangevinThermostat::new(
            DVector::from_element(model.nv, GAMMA),
            k_b_t,
            seed,
        ))
        .build()
        .install(&mut model);

    // Initial condition: start in the right well
    data.qpos[0] = X_0;
    data.qvel[0] = 0.0;
    data.forward(&model).expect("forward");

    // Burn-in
    for _ in 0..N_BURN_IN {
        data.step(&model).expect("burn-in step");
    }

    // Measurement: count committed transitions via hysteresis + track KE
    let mut transitions = 0usize;
    let mut last_well = WellState::from_position(data.qpos[0], X_THRESH);
    let mut ke_welford = WelfordOnline::new();

    for _ in 0..N_MEASURE {
        data.step(&model).expect("measure step");
        let x = data.qpos[0];
        let current = WellState::from_position(x, X_THRESH);

        // A transition is only counted when the particle moves from one
        // well to the other (Left→Right or Right→Left). Barrier visits
        // that return to the same well are filtered out.
        if current != WellState::Barrier && current != last_well && last_well != WellState::Barrier
        {
            transitions += 1;
            last_well = current;
        } else if current != WellState::Barrier {
            last_well = current;
        }

        ke_welford.push(0.5 * MASS * data.qvel[0] * data.qvel[0]);
    }

    (transitions, ke_welford)
}

// ─── Model invariants ─────────────────────���────────────────────────────────

#[test]
fn test_bistable_model_invariants() {
    use sim_core::types::Integrator;

    let model = sim_mjcf::load_model(BISTABLE_XML).expect("load");
    assert_eq!(model.nv, 1, "bistable model must have 1 velocity DOF");
    assert_eq!(model.nq, 1, "slide joint: nq = nv = 1");
    assert_eq!(model.timestep, 0.001);
    assert!(matches!(model.integrator, Integrator::Euler));
    assert_eq!(model.dof_damping[0], 0.0, "thermostat owns damping");
    assert_eq!(
        model.jnt_stiffness[0], 0.0,
        "potential comes from cb_passive"
    );
}

// ─── Gate A: Kramers rate at central parameters ───────────────────────────���

#[test]
fn gate_a_kramers_rate() {
    let well = DoubleWellPotential::new(DELTA_V, X_0, 0);
    let k_kramers = well.kramers_rate(GAMMA, MASS, K_B_T);

    let mut total_transitions = 0usize;
    let mut ke_across = WelfordOnline::new();

    for i in 0..N_TRAJ_GATE_A {
        let (transitions, ke_traj) = run_trajectory(K_B_T, SEED_BASE + i as u64);
        total_transitions += transitions;
        ke_across.push(ke_traj.mean());
    }

    // Kramers rate test
    let k_measured = total_transitions as f64 / (N_TRAJ_GATE_A as f64 * T_MEASURE);
    let relative_error = (k_measured - k_kramers).abs() / k_kramers;

    assert!(
        relative_error < 0.25,
        "Gate A FAILED: |k_measured - k_Kramers| / k_Kramers = {relative_error:.4} \
         (tolerance: 0.25)\n  k_measured = {k_measured:.6}\n  k_Kramers  = {k_kramers:.6}\n  \
         total_transitions = {total_transitions}\n  n_traj = {N_TRAJ_GATE_A}",
    );

    // Kinetic equipartition sanity check: ⟨½Mv²⟩ ≈ ½kT (hard 5% tolerance).
    // NOTE: The σ-based test is intentionally NOT used here because the Euler
    // discretization introduces a known systematic bias of O(h·γ/M) ≈ 1% (spec §3.5).
    // With 30 trajectories the SEM is ~0.05%, making the σ-based test detect
    // this bias at ~6σ — a correct detection of a known, accepted artifact.
    // The hard 5% tolerance is the spec requirement (§10).
    let expected_ke = 0.5 * K_B_T;
    let ke_relative_error = (ke_across.mean() - expected_ke).abs() / expected_ke;
    assert!(
        ke_relative_error < 0.05,
        "Gate A KE sanity FAILED: |⟨½Mv²⟩ - ½kT| / (½kT) = {ke_relative_error:.4} \
         (tolerance: 0.05)\n  measured = {:.6}\n  expected = {expected_ke:.6}",
        ke_across.mean(),
    );

    eprintln!(
        "Gate A PASSED: k_measured/k_Kramers = {:.4}, relative_error = {:.4}, \
         transitions = {total_transitions}, KE = {:.6} (expected {expected_ke:.6})",
        k_measured / k_kramers,
        relative_error,
        ke_across.mean(),
    );
}

// ─── Gate B: Arrhenius slope ──────────────────────���────────────────────────

#[test]
#[ignore = "450M steps — run with `cargo test -p sim-thermostat -- --ignored`"]
fn gate_b_arrhenius_slope() {
    let well = DoubleWellPotential::new(DELTA_V, X_0, 0);
    let temperatures = [0.75, 1.0, 1.5];
    let n_traj = 30usize;

    let mut inv_kt_values = Vec::with_capacity(temperatures.len());
    let mut log_k_values = Vec::with_capacity(temperatures.len());

    for &k_b_t in &temperatures {
        let mut total_transitions = 0usize;

        for i in 0..n_traj {
            // Use a different seed space for each temperature to avoid correlation
            let seed = SEED_BASE + 1000 * (k_b_t * 100.0) as u64 + i as u64;
            let (transitions, _) = run_trajectory_with_kbt(k_b_t, seed);
            total_transitions += transitions;
        }

        let k_measured = total_transitions as f64 / (n_traj as f64 * T_MEASURE);
        let k_theory = well.kramers_rate(GAMMA, MASS, k_b_t);

        eprintln!(
            "  kT={k_b_t:.2}: transitions={total_transitions}, \
             k_measured={k_measured:.6}, k_theory={k_theory:.6}, \
             ratio={:.3}",
            k_measured / k_theory,
        );

        inv_kt_values.push(1.0 / k_b_t);
        log_k_values.push(k_measured.ln());
    }

    // Linear regression: log(k) = a + m * (1/kT)
    let (slope, _intercept) = linear_regression(&inv_kt_values, &log_k_values);

    // Expected slope = -ΔV = -3.0
    let expected_slope = -DELTA_V;
    let slope_error = (slope - expected_slope).abs() / DELTA_V;

    assert!(
        slope_error < 0.10,
        "Gate B FAILED: |slope - (-ΔV)| / ΔV = {slope_error:.4} (tolerance: 0.10)\n  \
         measured slope = {slope:.4}\n  expected slope = {expected_slope:.4}",
    );

    eprintln!(
        "Gate B PASSED: slope = {slope:.4} (expected {expected_slope:.4}), \
         relative error = {slope_error:.4}",
    );
}

/// Same as `run_trajectory` but accepts kT as parameter.
fn run_trajectory_with_kbt(k_b_t: f64, seed: u64) -> (usize, WelfordOnline) {
    run_trajectory(k_b_t, seed)
}

/// Ordinary least-squares linear regression on (x, y) data.
/// Returns (slope, intercept).
fn linear_regression(x: &[f64], y: &[f64]) -> (f64, f64) {
    let n = x.len() as f64;
    let sx: f64 = x.iter().sum();
    let sy: f64 = y.iter().sum();
    let sxy: f64 = x.iter().zip(y.iter()).map(|(xi, yi)| xi * yi).sum();
    let sxx: f64 = x.iter().map(|xi| xi * xi).sum();

    let denom = n * sxx - sx * sx;
    let slope = (n * sxy - sx * sy) / denom;
    let intercept = (sy - slope * sx) / n;
    (slope, intercept)
}

// ─── Supporting: Boltzmann position distribution ───────────────────────────

/// Validates that the stationary position distribution has the correct
/// Boltzmann shape. Uses physically meaningful checks rather than a
/// chi-squared goodness-of-fit test, because the Euler discretization
/// introduces a known O(h·γ/M) ≈ 1% systematic deviation from exact
/// Boltzmann — a chi-squared test at high N detects this known artifact
/// rather than measuring whether the physics is correct.
///
/// Shape tests:
/// 1. Distribution is bimodal with peaks near ±x₀
/// 2. The log-density ratio between wells and barrier matches ΔV/kT
/// 3. The peak width is consistent with the well curvature
#[test]
fn supporting_boltzmann_distribution() {
    let mut model = sim_mjcf::load_model(BISTABLE_XML).expect("load");
    let mut data = model.make_data();

    PassiveStack::builder()
        .with(DoubleWellPotential::new(DELTA_V, X_0, 0))
        .with(LangevinThermostat::new(
            DVector::from_element(model.nv, GAMMA),
            K_B_T,
            SEED_BASE + 9999,
        ))
        .build()
        .install(&mut model);

    // Initial condition
    data.qpos[0] = X_0;
    data.qvel[0] = 0.0;
    data.forward(&model).expect("forward");

    // Burn-in
    for _ in 0..N_BURN_IN {
        data.step(&model).expect("burn-in");
    }

    // Collect position samples (2M steps — enough for shape, not so many
    // that the chi-squared detects the Euler discretization artifact)
    let n_samples = 2_000_000usize;
    let n_bins = 40usize;
    let x_min = -2.0 * X_0;
    let x_max = 2.0 * X_0;
    let bin_width = (x_max - x_min) / n_bins as f64;

    let mut histogram = vec![0usize; n_bins];

    for _ in 0..n_samples {
        data.step(&model).expect("measure");
        let x = data.qpos[0];
        if x >= x_min && x < x_max {
            let bin = ((x - x_min) / bin_width) as usize;
            let bin = bin.min(n_bins - 1);
            histogram[bin] += 1;
        }
    }

    // Test 1: Find peak positions (should be near ±x₀ = ±1.0)
    let bin_center = |i: usize| x_min + (i as f64 + 0.5) * bin_width;

    // Left half peak
    let left_peak_bin = (0..n_bins / 2).max_by_key(|&i| histogram[i]).unwrap();
    let left_peak_x = bin_center(left_peak_bin);

    // Right half peak
    let right_peak_bin = (n_bins / 2..n_bins).max_by_key(|&i| histogram[i]).unwrap();
    let right_peak_x = bin_center(right_peak_bin);

    assert!(
        (left_peak_x - (-X_0)).abs() < 0.2,
        "Left peak at {left_peak_x:.3}, expected near -{X_0}"
    );
    assert!(
        (right_peak_x - X_0).abs() < 0.2,
        "Right peak at {right_peak_x:.3}, expected near {X_0}"
    );

    // Test 2: Log-density ratio between well peaks and barrier
    // Expected: p(x₀) / p(0) = exp(ΔV/kT) = exp(3) ≈ 20.09
    let barrier_bin = n_bins / 2; // bin containing x=0
    let peak_count = (histogram[left_peak_bin] + histogram[right_peak_bin]) as f64 / 2.0;
    let barrier_count = histogram[barrier_bin] as f64;

    // Avoid divide-by-zero (barrier bin might be very sparse)
    assert!(
        barrier_count > 10.0,
        "Barrier bin has too few counts ({barrier_count}) for ratio test"
    );

    let measured_ratio = peak_count / barrier_count;
    // The ratio test should hold within ~25% (accounts for bin-width effects
    // and discretization bias). The LOG of the ratio should match ΔV/kT.
    let log_ratio = measured_ratio.ln();
    let expected_log_ratio = DELTA_V / K_B_T;
    let ratio_error = (log_ratio - expected_log_ratio).abs() / expected_log_ratio;

    assert!(
        ratio_error < 0.25,
        "Boltzmann ratio FAILED: ln(p_well/p_barrier) = {log_ratio:.3}, \
         expected {expected_log_ratio:.3}, relative error = {ratio_error:.3} \
         (tolerance: 0.25)\n  peak_count = {peak_count:.0}, \
         barrier_count = {barrier_count:.0}, ratio = {measured_ratio:.2}"
    );

    // Test 3: Peak width consistent with well curvature
    // σ_x = √(kT / V''(x₀)) = √(1/24) ≈ 0.204
    // The FWHM ≈ 2.35σ ≈ 0.48
    // Count CONTIGUOUS bins around the right peak that have > 50% of peak height
    let half_max = histogram[right_peak_bin] / 2;
    let mut fwhm_lo = right_peak_bin;
    while fwhm_lo > 0 && histogram[fwhm_lo - 1] > half_max {
        fwhm_lo -= 1;
    }
    let mut fwhm_hi = right_peak_bin;
    while fwhm_hi + 1 < n_bins && histogram[fwhm_hi + 1] > half_max {
        fwhm_hi += 1;
    }
    let measured_fwhm = (fwhm_hi - fwhm_lo + 1) as f64 * bin_width;
    // σ = √(kT / V''(x₀)) = √(kT / (8·ΔV/x₀²))
    let expected_sigma = (K_B_T * X_0 * X_0 / (8.0 * DELTA_V)).sqrt();
    let expected_fwhm = 2.355 * expected_sigma; // 2√(2ln2) · σ

    // FWHM should match within 50% (generous — bin quantization limits precision)
    let fwhm_error = (measured_fwhm - expected_fwhm).abs() / expected_fwhm;
    assert!(
        fwhm_error < 0.50,
        "Peak width FAILED: measured FWHM = {measured_fwhm:.3}, \
         expected ≈ {expected_fwhm:.3}, error = {fwhm_error:.3}"
    );

    eprintln!(
        "Boltzmann PASSED:\n  peaks at ({left_peak_x:.3}, {right_peak_x:.3}), \
         expected (±{X_0})\n  ln(peak/barrier) = {log_ratio:.3}, \
         expected {expected_log_ratio:.3} (error {ratio_error:.3})\n  \
         FWHM = {measured_fwhm:.3}, expected {expected_fwhm:.3} (error {fwhm_error:.3})"
    );
}

// ─── Supporting: Reproducibility ───────────────────��───────────────────────

#[test]
fn supporting_reproducibility() {
    let seed = SEED_BASE + 42;

    // Run two identical trajectories
    let (transitions_a, _) = run_trajectory(K_B_T, seed);
    let (transitions_b, _) = run_trajectory(K_B_T, seed);

    assert_eq!(
        transitions_a, transitions_b,
        "Reproducibility FAILED: same seed produced different transition counts \
         ({transitions_a} vs {transitions_b})"
    );

    // Also verify the trajectories produce identical position sequences
    // by running a shorter trajectory and checking final state
    let run_short = |s: u64| -> (f64, f64) {
        let mut model = sim_mjcf::load_model(BISTABLE_XML).expect("load");
        let mut data = model.make_data();

        PassiveStack::builder()
            .with(DoubleWellPotential::new(DELTA_V, X_0, 0))
            .with(LangevinThermostat::new(
                DVector::from_element(model.nv, GAMMA),
                K_B_T,
                s,
            ))
            .build()
            .install(&mut model);

        data.qpos[0] = X_0;
        data.qvel[0] = 0.0;
        data.forward(&model).expect("forward");

        for _ in 0..10_000 {
            data.step(&model).expect("step");
        }
        (data.qpos[0], data.qvel[0])
    };

    let (pos_a, vel_a) = run_short(seed);
    let (pos_b, vel_b) = run_short(seed);

    assert_eq!(pos_a, pos_b, "Position not bit-exact reproducible");
    assert_eq!(vel_a, vel_b, "Velocity not bit-exact reproducible");

    eprintln!("Reproducibility PASSED: {transitions_a} transitions, bit-exact");
}
