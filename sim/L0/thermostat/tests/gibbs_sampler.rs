//! Phase 6 — Native Gibbs sampler + three-way distribution comparison.
//!
//! Validates the `GibbsSampler` against `exact_distribution`, then compares
//! both against the physical Langevin sampler to isolate the continuous-to-
//! Ising mapping error from sampler error.
//!
//! Tests:
//! - Gate A: Gibbs correctness on Phase 5 params (must-pass)
//! - Gate B: Three-way comparison on Phase 5 params (must-pass)
//! - Gate C: Phase 4 §11.4 closure on NN chain (must-pass)
//! - Reproducibility: same seed → bit-identical histograms
//!
//! Spec: `docs/thermo_computing/03_phases/06_gibbs_sampler.md`

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
    clippy::needless_range_loop,
    clippy::too_many_lines,
    clippy::uninlined_format_args
)]

use sim_core::DVector;
use sim_thermostat::ising::{exact_distribution, kl_divergence, tv_distance};
use sim_thermostat::test_utils::WellState;
use sim_thermostat::{
    DoubleWellPotential, ExternalField, GibbsSampler, LangevinThermostat, PairwiseCoupling,
    PassiveStack,
};

// ─── Shared constants ────────────────────────────────────────────────────

const N: usize = 4;
const N_CONFIGS: usize = 1 << N; // 16
const DELTA_V: f64 = 3.0;
const X_0: f64 = 1.0;
const GAMMA: f64 = 10.0;
const K_B_T: f64 = 1.0;
const X_THRESH: f64 = X_0 / 2.0;

// Phase 5 target params (fully-connected)
const FC_EDGES: [(usize, usize); 6] = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)];
const FC_J: [f64; 6] = [0.8, -0.3, 0.1, 0.5, -0.2, 0.6];
const FC_H: [f64; 4] = [0.3, -0.2, 0.0, 0.15];

// Phase 4 NN chain params
const CHAIN_EDGES: [(usize, usize); 3] = [(0, 1), (1, 2), (2, 3)];
const CHAIN_J: f64 = 0.5;

// Langevin trajectory params
const LANGEVIN_N_TRAJ: usize = 10;
const LANGEVIN_N_STEPS: usize = 5_000_000;
const LANGEVIN_N_BURN_IN: usize = 20_000;

// Gibbs sampler params
const GIBBS_N_BURN_IN: usize = 1_000;
const GIBBS_N_SAMPLES: usize = 100_000;

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

// ─── Helpers ─────────────────────────────────────────────────────────────

/// Topology for the Langevin helper: either fully-connected with per-edge
/// J and per-site h, or a uniform NN chain.
enum Topology {
    FullyConnected,
    Chain,
}

/// Run a single Langevin trajectory and return the raw config histogram
/// (counts per 2^N bin).
fn langevin_trajectory_histogram(topology: &Topology, seed: u64) -> [usize; N_CONFIGS] {
    let mut model = sim_mjcf::load_model(CHAIN_XML).expect("load");
    let mut data = model.make_data();

    let mut builder = PassiveStack::builder();
    for i in 0..N {
        builder = builder.with(DoubleWellPotential::new(DELTA_V, X_0, i));
    }
    match topology {
        Topology::FullyConnected => {
            builder = builder.with(PairwiseCoupling::new(FC_J.to_vec(), FC_EDGES.to_vec()));
            builder = builder.with(ExternalField::new(FC_H.to_vec()));
        }
        Topology::Chain => {
            builder = builder.with(PairwiseCoupling::chain(N, CHAIN_J));
        }
    }
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

    // Burn-in
    for _ in 0..LANGEVIN_N_BURN_IN {
        data.step(&model).expect("burn-in step");
    }

    // Measure
    let mut histogram = [0_usize; N_CONFIGS];
    let n_measure = LANGEVIN_N_STEPS - LANGEVIN_N_BURN_IN;
    for _ in 0..n_measure {
        data.step(&model).expect("measure step");

        let states: [WellState; N] =
            core::array::from_fn(|i| WellState::from_position(data.qpos[i], X_THRESH));

        if states.iter().all(|s| s.is_in_well()) {
            let mut config: u32 = 0;
            for i in 0..N {
                if states[i] == WellState::Right {
                    config |= 1 << i;
                }
            }
            histogram[config as usize] += 1;
        }
    }

    histogram
}

/// Aggregate multiple Langevin trajectory histograms into a normalized
/// distribution in `Vec<(u32, f64)>` format.
fn langevin_distribution(topology: &Topology, seed_base: u64) -> Vec<(u32, f64)> {
    let mut total_histogram = [0_usize; N_CONFIGS];

    for traj in 0..LANGEVIN_N_TRAJ {
        let seed = seed_base + traj as u64;
        let h = langevin_trajectory_histogram(topology, seed);
        let traj_total: usize = h.iter().sum();
        eprintln!(
            "  Langevin trajectory {}/{} done (seed={}, valid_samples={})",
            traj + 1,
            LANGEVIN_N_TRAJ,
            seed,
            traj_total
        );
        for (i, count) in h.iter().enumerate() {
            total_histogram[i] += count;
        }
    }

    let total: usize = total_histogram.iter().sum();
    assert!(total > 0, "no valid Langevin samples collected");
    let total_f = total as f64;

    (0..N_CONFIGS as u32)
        .map(|c| (c, total_histogram[c as usize] as f64 / total_f))
        .collect()
}

/// Print a full probability table to stderr for diagnostic output.
fn print_probability_table(
    label: &str,
    exact: &[(u32, f64)],
    gibbs: &[(u32, f64)],
    langevin: &[(u32, f64)],
) {
    eprintln!("\n{label}:");
    eprintln!(
        "  {:>6}  {:>10}  {:>10}  {:>10}  {:>10}",
        "config", "P_exact", "P_gibbs", "P_langev", "|ex-lang|"
    );
    for i in 0..exact.len() {
        let (c, p_e) = exact[i];
        let p_g = gibbs[i].1;
        let p_l = langevin[i].1;
        eprintln!(
            "  {:>6}  {:>10.6}  {:>10.6}  {:>10.6}  {:>10.6}",
            c,
            p_e,
            p_g,
            p_l,
            (p_e - p_l).abs()
        );
    }
}

// ─── Gate A: Gibbs sampler correctness (must-pass) ───────────────────────

#[test]
fn gate_a_gibbs_correctness() {
    let exact = exact_distribution(N, &FC_EDGES, &FC_J, &FC_H, K_B_T);

    let mut sampler = GibbsSampler::new(N, &FC_EDGES, &FC_J, &FC_H, K_B_T, 2_026_041_006);
    let gibbs = sampler.sample(GIBBS_N_BURN_IN, GIBBS_N_SAMPLES);

    let tv = tv_distance(&exact, &gibbs);
    let kl = kl_divergence(&exact, &gibbs);

    eprintln!("Gate A: TV(Gibbs, exact) = {tv:.6}, KL(exact || Gibbs) = {kl:.6}");

    assert!(
        tv < 0.02,
        "Gate A FAILED: TV(Gibbs, exact) = {tv:.6} >= 0.02"
    );
    assert!(
        kl < 0.01,
        "Gate A FAILED: KL(exact || Gibbs) = {kl:.6} >= 0.01"
    );

    eprintln!("Gate A PASSED");
}

// ─── Gate B: Three-way comparison (must-pass) ────────────────────────────

#[test]
fn gate_b_three_way_comparison() {
    // Exact distribution
    let exact = exact_distribution(N, &FC_EDGES, &FC_J, &FC_H, K_B_T);

    // Gibbs distribution
    let mut sampler = GibbsSampler::new(N, &FC_EDGES, &FC_J, &FC_H, K_B_T, 2_026_041_006);
    let gibbs = sampler.sample(GIBBS_N_BURN_IN, GIBBS_N_SAMPLES);

    // Langevin distribution (10 traj × 5M steps)
    let langevin = langevin_distribution(&Topology::FullyConnected, 3_026_041_006);

    // All three TV distances
    let tv_gibbs_exact = tv_distance(&gibbs, &exact);
    let tv_langevin_exact = tv_distance(&langevin, &exact);
    let tv_langevin_gibbs = tv_distance(&langevin, &gibbs);

    eprintln!("Gate B:");
    eprintln!("  TV(Gibbs, exact)    = {tv_gibbs_exact:.6}");
    eprintln!("  TV(Langevin, exact) = {tv_langevin_exact:.6}");
    eprintln!("  TV(Langevin, Gibbs) = {tv_langevin_gibbs:.6}");

    print_probability_table("Gate B probability table", &exact, &gibbs, &langevin);

    // Gate criteria (§8.2)
    assert!(
        tv_gibbs_exact < 0.02,
        "Gate B FAILED: TV(Gibbs, exact) = {tv_gibbs_exact:.6} >= 0.02"
    );
    assert!(
        tv_langevin_exact < 0.15,
        "Gate B FAILED: TV(Langevin, exact) = {tv_langevin_exact:.6} >= 0.15"
    );
    assert!(
        tv_gibbs_exact < tv_langevin_exact,
        "Gate B FAILED: TV(Gibbs, exact) = {tv_gibbs_exact:.6} >= \
         TV(Langevin, exact) = {tv_langevin_exact:.6}"
    );
    assert!(
        tv_langevin_gibbs > 0.01,
        "Gate B FAILED: TV(Langevin, Gibbs) = {tv_langevin_gibbs:.6} <= 0.01 \
         (mapping error should be nonzero)"
    );

    eprintln!("Gate B PASSED");
}

// ─── Gate C: Phase 4 §11.4 closure (must-pass) ──────────────────────────

#[test]
fn gate_c_phase4_closure() {
    let chain_j = vec![CHAIN_J; 3];
    let chain_h = vec![0.0; N];

    // Exact distribution
    let exact = exact_distribution(N, &CHAIN_EDGES, &chain_j, &chain_h, K_B_T);

    // Gibbs distribution
    let mut sampler = GibbsSampler::new(N, &CHAIN_EDGES, &chain_j, &chain_h, K_B_T, 4_026_041_006);
    let gibbs = sampler.sample(GIBBS_N_BURN_IN, GIBBS_N_SAMPLES);

    // Langevin distribution (10 traj × 5M steps)
    let langevin = langevin_distribution(&Topology::Chain, 5_026_041_006);

    // All three TV distances
    let tv_gibbs_exact = tv_distance(&gibbs, &exact);
    let tv_langevin_exact = tv_distance(&langevin, &exact);
    let tv_langevin_gibbs = tv_distance(&langevin, &gibbs);

    eprintln!("Gate C (Phase 4 §11.4 closure):");
    eprintln!("  TV(Gibbs, exact)    = {tv_gibbs_exact:.6}");
    eprintln!("  TV(Langevin, exact) = {tv_langevin_exact:.6}");
    eprintln!("  TV(Langevin, Gibbs) = {tv_langevin_gibbs:.6}");

    print_probability_table("Gate C probability table", &exact, &gibbs, &langevin);

    // Gate criteria (§8.3)
    assert!(
        tv_gibbs_exact < 0.02,
        "Gate C FAILED: TV(Gibbs, exact) = {tv_gibbs_exact:.6} >= 0.02"
    );
    assert!(
        tv_langevin_exact < 0.10,
        "Gate C FAILED: TV(Langevin, exact) = {tv_langevin_exact:.6} >= 0.10"
    );
    assert!(
        tv_gibbs_exact < tv_langevin_exact,
        "Gate C FAILED: TV(Gibbs, exact) = {tv_gibbs_exact:.6} >= \
         TV(Langevin, exact) = {tv_langevin_exact:.6}"
    );

    eprintln!("Gate C PASSED");
}

// ─── Reproducibility ─────────────────────────────────────────────────────

#[test]
fn gibbs_reproducibility() {
    let run = || {
        let mut sampler = GibbsSampler::new(N, &FC_EDGES, &FC_J, &FC_H, K_B_T, 7_777);
        sampler.sample(GIBBS_N_BURN_IN, GIBBS_N_SAMPLES)
    };

    let d1 = run();
    let d2 = run();

    for (a, b) in d1.iter().zip(&d2) {
        assert_eq!(a.0, b.0, "config mismatch");
        assert_eq!(a.1, b.1, "probability not bit-exact for config {}", a.0);
    }

    eprintln!("Reproducibility PASSED: same seed → bit-identical histograms");
}
