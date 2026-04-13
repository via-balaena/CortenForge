//! Phase 4 — Coupled bistable array + Ising comparison.
//!
//! Validates that an array of N=4 coupled double-well elements under the
//! Langevin thermostat produces pairwise correlations consistent with the
//! exact Ising model on the same coupling topology.
//!
//! Tests:
//! - Model invariants
//! - Gate A: NN pairwise correlations at central parameters (must-pass)
//! - Gate B: Coupling-strength sweep (`#[ignore]`)
//! - Supporting: Marginal symmetry
//! - Supporting: NNN and long-range correlations
//! - Supporting: Configuration frequency ratio
//! - Supporting: Reproducibility
//!
//! Spec: `docs/thermo_computing/03_phases/04_coupled_bistable_array.md`

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_possible_wrap,
    clippy::float_cmp,
    clippy::doc_markdown,
    clippy::similar_names,
    clippy::suboptimal_flops,
    clippy::needless_range_loop,
    clippy::too_many_lines,
    clippy::manual_midpoint,
    clippy::uninlined_format_args,
    clippy::inconsistent_digit_grouping
)]

use sim_core::DVector;
use sim_thermostat::test_utils::{WelfordOnline, WellState};
use sim_thermostat::{DoubleWellPotential, LangevinThermostat, PairwiseCoupling, PassiveStack};

// ─── MJCF model ────────────────────────────────────────────────────────────

const N: usize = 4;

// NOTE: Each body is offset in Y so the geoms never overlap — prevents
// spurious contact forces. The slide joints act along X, so the Y offset
// does not affect qpos or the physics. contype="0" conaffinity="0"
// explicitly disables contacts as a belt-and-suspenders measure.
const CHAIN_XML: &str = r#"
<mujoco model="bistable_chain_4">
  <option timestep="0.001" gravity="0 0 0" integrator="Euler"/>
  <worldbody>
    <body name="p0" pos="0 0 0">
      <joint name="x0" type="slide" axis="1 0 0" stiffness="0" damping="0"/>
      <geom type="sphere" size="0.05" mass="1" contype="0" conaffinity="0"/>
    </body>
    <body name="p1" pos="0 1 0">
      <joint name="x1" type="slide" axis="1 0 0" stiffness="0" damping="0"/>
      <geom type="sphere" size="0.05" mass="1" contype="0" conaffinity="0"/>
    </body>
    <body name="p2" pos="0 2 0">
      <joint name="x2" type="slide" axis="1 0 0" stiffness="0" damping="0"/>
      <geom type="sphere" size="0.05" mass="1" contype="0" conaffinity="0"/>
    </body>
    <body name="p3" pos="0 3 0">
      <joint name="x3" type="slide" axis="1 0 0" stiffness="0" damping="0"/>
      <geom type="sphere" size="0.05" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
"#;

// ─── Central parameter set (spec §7) ──────────────────────────────────────

const DELTA_V: f64 = 3.0;
const X_0: f64 = 1.0;
const MASS: f64 = 1.0;
const GAMMA: f64 = 10.0;
const K_B_T: f64 = 1.0;
const COUPLING_J: f64 = 0.5;
const SEED_BASE: u64 = 20_260_410_04;

// ─── Test configuration ───────────────────────────────────────────────────

const N_STEPS_PER_TRAJ: usize = 10_000_000;
const N_BURN_IN: usize = 20_000;
const N_MEASURE: usize = N_STEPS_PER_TRAJ - N_BURN_IN;
const N_TRAJ: usize = 20;
const X_THRESH: f64 = X_0 / 2.0;

// ─── Helpers ──────────────────────────────────────────────────────────────

/// Build model + data with the coupled bistable array installed.
fn setup(coupling_j: f64, seed: u64) -> (sim_core::Model, sim_core::Data) {
    let mut model = sim_mjcf::load_model(CHAIN_XML).expect("load");
    let mut data = model.make_data();

    let mut builder = PassiveStack::builder();
    for i in 0..N {
        builder = builder.with(DoubleWellPotential::new(DELTA_V, X_0, i));
    }
    builder = builder.with(PairwiseCoupling::chain(N, coupling_j));
    builder = builder.with(LangevinThermostat::new(
        DVector::from_element(N, GAMMA),
        K_B_T,
        seed,
        0,
    ));
    builder.build().install(&mut model);

    // Initial condition: all elements in the right well
    for i in 0..N {
        data.qpos[i] = X_0;
        data.qvel[i] = 0.0;
    }
    data.forward(&model).expect("forward");

    (model, data)
}

/// Results from a single trajectory measurement.
struct TrajectoryResult {
    /// Per-pair NN correlation (3 pairs for a 4-chain).
    nn_correlations: [f64; 3],
    /// Per-pair NNN correlation (2 pairs).
    nnn_correlations: [f64; 2],
    /// Long-range correlation (1 pair: (0,3)).
    long_range_correlation: f64,
    /// Per-element P(σ = +1).
    marginal_right: [f64; N],
    /// Per-DOF kinetic energy mean.
    ke_means: [f64; N],
    /// Configuration histogram (2^N = 16 bins).
    config_histogram: [usize; 16],
}

/// Run a single trajectory and collect all measurements.
fn run_trajectory(coupling_j: f64, seed: u64) -> TrajectoryResult {
    let (model, mut data) = setup(coupling_j, seed);

    // Burn-in
    for _ in 0..N_BURN_IN {
        data.step(&model).expect("burn-in step");
    }

    // Per-pair correlation accumulators (NN: 3, NNN: 2, long: 1)
    let mut nn_acc = [
        WelfordOnline::new(),
        WelfordOnline::new(),
        WelfordOnline::new(),
    ];
    let mut nnn_acc = [WelfordOnline::new(), WelfordOnline::new()];
    let mut long_acc = WelfordOnline::new();

    // Per-element marginal counters
    let mut right_count = [0usize; N];
    let mut well_count = [0usize; N];

    // Per-DOF kinetic energy
    let mut ke_acc: [WelfordOnline; N] = core::array::from_fn(|_| WelfordOnline::new());

    // Configuration histogram
    let mut config_histogram = [0usize; 16];

    for _ in 0..N_MEASURE {
        data.step(&model).expect("measure step");

        // Classify all elements
        let states: [WellState; N] =
            core::array::from_fn(|i| WellState::from_position(data.qpos[i], X_THRESH));

        // Per-element marginal tracking
        for i in 0..N {
            if states[i].is_in_well() {
                well_count[i] += 1;
                if states[i] == WellState::Right {
                    right_count[i] += 1;
                }
            }
        }

        // NN correlations (per-pair valid: both in wells)
        for (pair_idx, &(i, j)) in [(0, 1), (1, 2), (2, 3)].iter().enumerate() {
            if states[i].is_in_well() && states[j].is_in_well() {
                nn_acc[pair_idx].push(states[i].spin() * states[j].spin());
            }
        }

        // NNN correlations
        for (pair_idx, &(i, j)) in [(0, 2), (1, 3)].iter().enumerate() {
            if states[i].is_in_well() && states[j].is_in_well() {
                nnn_acc[pair_idx].push(states[i].spin() * states[j].spin());
            }
        }

        // Long-range correlation (0,3)
        if states[0].is_in_well() && states[3].is_in_well() {
            long_acc.push(states[0].spin() * states[3].spin());
        }

        // Configuration histogram (all must be in wells)
        if states.iter().all(|s| s.is_in_well()) {
            let mut config: u32 = 0;
            for i in 0..N {
                if states[i] == WellState::Right {
                    config |= 1 << i;
                }
            }
            config_histogram[config as usize] += 1;
        }

        // Kinetic energy per DOF
        for i in 0..N {
            ke_acc[i].push(0.5 * MASS * data.qvel[i] * data.qvel[i]);
        }
    }

    TrajectoryResult {
        nn_correlations: core::array::from_fn(|i| nn_acc[i].mean()),
        nnn_correlations: core::array::from_fn(|i| nnn_acc[i].mean()),
        long_range_correlation: long_acc.mean(),
        marginal_right: core::array::from_fn(|i| {
            if well_count[i] > 0 {
                right_count[i] as f64 / well_count[i] as f64
            } else {
                0.5 // fallback — should never happen
            }
        }),
        ke_means: core::array::from_fn(|i| ke_acc[i].mean()),
        config_histogram,
    }
}

/// Exact Ising NN correlation for open chain: tanh(βJ)^distance.
fn ising_correlation(beta_j: f64, distance: usize) -> f64 {
    beta_j.tanh().powi(distance as i32)
}

// ─── Model invariants ─────────────────────────────────────────────────────

#[test]
fn test_chain_model_invariants() {
    use sim_core::Integrator;

    let model = sim_mjcf::load_model(CHAIN_XML).expect("load");
    assert_eq!(model.nv, N, "chain model must have {N} velocity DOFs");
    assert_eq!(model.nq, N, "slide joints: nq = nv = {N}");
    assert_eq!(model.timestep, 0.001);
    assert!(matches!(model.integrator, Integrator::Euler));
    for i in 0..N {
        assert_eq!(
            model.dof_damping[i], 0.0,
            "thermostat owns damping on DOF {i}"
        );
        assert_eq!(
            model.jnt_stiffness[i], 0.0,
            "potential comes from cb_passive on joint {i}"
        );
    }
}

// ─── Gate A: Pairwise NN correlations ─────────────────────────────────────

#[test]
fn gate_a_nn_correlations() {
    let beta_j = COUPLING_J * X_0 * X_0 / K_B_T;
    let expected_nn = ising_correlation(beta_j, 1);

    // Two-level Welford: per-trajectory means → ensemble
    let mut nn_across = [
        WelfordOnline::new(),
        WelfordOnline::new(),
        WelfordOnline::new(),
    ];
    let mut ke_across: [WelfordOnline; N] = core::array::from_fn(|_| WelfordOnline::new());
    let mut marginal_across: [WelfordOnline; N] = core::array::from_fn(|_| WelfordOnline::new());
    let mut nnn_across = [WelfordOnline::new(), WelfordOnline::new()];
    let mut long_across = WelfordOnline::new();

    // Aggregate config histogram across all trajectories
    let mut total_config_histogram = [0usize; 16];

    for traj in 0..N_TRAJ {
        let result = run_trajectory(COUPLING_J, SEED_BASE + traj as u64);

        for pair in 0..3 {
            nn_across[pair].push(result.nn_correlations[pair]);
        }
        for pair in 0..2 {
            nnn_across[pair].push(result.nnn_correlations[pair]);
        }
        long_across.push(result.long_range_correlation);

        for i in 0..N {
            ke_across[i].push(result.ke_means[i]);
            marginal_across[i].push(result.marginal_right[i]);
        }

        for (c, &count) in result.config_histogram.iter().enumerate() {
            total_config_histogram[c] += count;
        }
    }

    // ── Gate A: NN correlations ──────────────────────────────────────

    // Average across 3 NN pairs (by chain symmetry, should be equal)
    let mut nn_pair_means = WelfordOnline::new();
    for pair in 0..3 {
        nn_pair_means.push(nn_across[pair].mean());
    }
    let nn_measured = nn_pair_means.mean();

    let nn_relative_error = (nn_measured - expected_nn).abs() / expected_nn;
    assert!(
        nn_relative_error < 0.25,
        "Gate A FAILED: |⟨σσ⟩ - tanh(βJ)| / tanh(βJ) = {nn_relative_error:.4} \
         (tolerance: 0.25)\n  measured = {nn_measured:.6}\n  \
         expected = {expected_nn:.6}\n  per-pair = [{:.4}, {:.4}, {:.4}]",
        nn_across[0].mean(),
        nn_across[1].mean(),
        nn_across[2].mean(),
    );

    // ── Gate A: KE sanity (per DOF) ──────────────────────────────────

    let expected_ke = 0.5 * K_B_T;
    for i in 0..N {
        let ke_error = (ke_across[i].mean() - expected_ke).abs() / expected_ke;
        assert!(
            ke_error < 0.05,
            "Gate A KE sanity FAILED on DOF {i}: |⟨½Mv²⟩ - ½kT| / (½kT) = {ke_error:.4} \
             (tolerance: 0.05)\n  measured = {:.6}\n  expected = {expected_ke:.6}",
            ke_across[i].mean(),
        );
    }

    // ── Supporting: Marginal symmetry ────────────────────────────────

    for i in 0..N {
        let p_right = marginal_across[i].mean();
        let marginal_error = (p_right - 0.5).abs();
        assert!(
            marginal_error < 0.05,
            "Marginal symmetry FAILED on element {i}: |P(σ=+1) - 0.5| = {marginal_error:.4} \
             (tolerance: 0.05)\n  P(σ=+1) = {p_right:.4}",
        );
    }

    // ── Supporting: NNN and long-range correlations ───────────────────

    let expected_nnn = ising_correlation(beta_j, 2);
    let nnn_measured = (nnn_across[0].mean() + nnn_across[1].mean()) / 2.0;
    let nnn_relative_error = (nnn_measured - expected_nnn).abs() / expected_nnn;
    assert!(
        nnn_relative_error < 0.30,
        "NNN correlation FAILED: |⟨σσ⟩ - tanh²(βJ)| / tanh²(βJ) = {nnn_relative_error:.4} \
         (tolerance: 0.30)\n  measured = {nnn_measured:.6}\n  expected = {expected_nnn:.6}",
    );

    let expected_long = ising_correlation(beta_j, 3);
    let long_measured = long_across.mean();
    let long_abs_error = (long_measured - expected_long).abs();
    assert!(
        long_abs_error < 0.08,
        "Long-range correlation FAILED: |⟨σ₀σ₃⟩ - tanh³(βJ)| = {long_abs_error:.4} \
         (tolerance: 0.08)\n  measured = {long_measured:.6}\n  expected = {expected_long:.6}",
    );

    // ── Supporting: Configuration frequency ratio ────────────────────

    let total_configs: usize = total_config_histogram.iter().sum();
    if total_configs > 0 {
        let max_count = *total_config_histogram.iter().max().unwrap();
        let min_count = *total_config_histogram
            .iter()
            .filter(|&&c| c > 0)
            .min()
            .unwrap_or(&1);
        let ratio = max_count as f64 / min_count.max(1) as f64;

        // Expected ratio ≈ 20:1 (exp(3)). Tolerance: [10, 40].
        assert!(
            ratio > 10.0 && ratio < 40.0,
            "Config ratio FAILED: p_max/p_min = {ratio:.1} (tolerance: 10-40)\n  \
             max_count = {max_count}, min_count = {min_count}, total = {total_configs}",
        );

        eprintln!(
            "  Config ratio: {ratio:.1} (expected ~20, tolerance 10-40), \
             total valid configs = {total_configs}"
        );
    }

    eprintln!(
        "Gate A PASSED:\n  NN correlation = {nn_measured:.4} (expected {expected_nn:.4}, \
         error {nn_relative_error:.4})\n  NNN = {nnn_measured:.4} (expected {expected_nnn:.4})\n  \
         Long-range = {long_measured:.4} (expected {expected_long:.4})\n  \
         KE per DOF = [{:.4}, {:.4}, {:.4}, {:.4}] (expected {expected_ke:.4})\n  \
         Marginals = [{:.4}, {:.4}, {:.4}, {:.4}] (expected 0.5)",
        ke_across[0].mean(),
        ke_across[1].mean(),
        ke_across[2].mean(),
        ke_across[3].mean(),
        marginal_across[0].mean(),
        marginal_across[1].mean(),
        marginal_across[2].mean(),
        marginal_across[3].mean(),
    );
}

// ─── Gate B: Coupling-strength sweep ──────────────────────────────────────

#[test]
#[ignore = "600M steps — run with `cargo test -p sim-thermostat -- --ignored gate_b`"]
fn gate_b_coupling_sweep() {
    let coupling_values = [0.25, 0.50, 0.75];
    let mut measured_correlations = Vec::with_capacity(coupling_values.len());

    for &j in &coupling_values {
        let beta_j = j * X_0 * X_0 / K_B_T;
        let expected = ising_correlation(beta_j, 1);

        let mut nn_across = [
            WelfordOnline::new(),
            WelfordOnline::new(),
            WelfordOnline::new(),
        ];

        for traj in 0..N_TRAJ {
            // Different seed space per J to avoid cross-J correlation
            let seed = SEED_BASE + 10_000 * (j * 100.0) as u64 + traj as u64;
            let result = run_trajectory(j, seed);
            for pair in 0..3 {
                nn_across[pair].push(result.nn_correlations[pair]);
            }
        }

        let nn_measured = (nn_across[0].mean() + nn_across[1].mean() + nn_across[2].mean()) / 3.0;
        let relative_error = (nn_measured - expected).abs() / expected;

        eprintln!(
            "  J={j:.2}: ⟨σσ⟩_NN = {nn_measured:.4}, expected = {expected:.4}, \
             error = {relative_error:.4}",
        );

        assert!(
            relative_error < 0.25,
            "Gate B FAILED at J={j}: |⟨σσ⟩ - tanh(βJ)| / tanh(βJ) = {relative_error:.4} \
             (tolerance: 0.25)\n  measured = {nn_measured:.6}\n  expected = {expected:.6}",
        );

        measured_correlations.push(nn_measured);
    }

    // Monotonicity: correlations should increase with J
    for i in 0..measured_correlations.len() - 1 {
        assert!(
            measured_correlations[i] < measured_correlations[i + 1],
            "Gate B monotonicity FAILED: ⟨σσ⟩(J={}) = {:.4} >= ⟨σσ⟩(J={}) = {:.4}",
            coupling_values[i],
            measured_correlations[i],
            coupling_values[i + 1],
            measured_correlations[i + 1],
        );
    }

    eprintln!("Gate B PASSED: correlations = {:?}", measured_correlations);
}

// ─── Supporting: Reproducibility ──────────────────────────────────────────

#[test]
fn supporting_reproducibility() {
    let seed = SEED_BASE + 42;
    let n_steps = 100_000;

    let run = |s: u64| -> ([f64; N], [f64; N]) {
        let (model, mut data) = setup(COUPLING_J, s);
        for _ in 0..n_steps {
            data.step(&model).expect("step");
        }
        let pos = core::array::from_fn(|i| data.qpos[i]);
        let vel = core::array::from_fn(|i| data.qvel[i]);
        (pos, vel)
    };

    let (pos_a, vel_a) = run(seed);
    let (pos_b, vel_b) = run(seed);

    for i in 0..N {
        assert_eq!(
            pos_a[i], pos_b[i],
            "Position not bit-exact reproducible on DOF {i}"
        );
        assert_eq!(
            vel_a[i], vel_b[i],
            "Velocity not bit-exact reproducible on DOF {i}"
        );
    }

    eprintln!("Reproducibility PASSED: {n_steps} steps, {N} DOFs, bit-exact");
}
