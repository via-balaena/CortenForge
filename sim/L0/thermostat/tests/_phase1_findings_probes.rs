//! Phase 1 diagnostic probes — crime-scene evidence gathering.
//!
//! Not committed to the repo as of writing. Built to confirm or
//! disconfirm the two failure-mode hypotheses for §7 and §10:
//!
//! - Probe A: §10 decay envelope at multiple checkpoints, compared
//!   against the underdamped-oscillator amplitude theory
//!   `exp(−γt/2M)`. Empirically validates whether the spec's
//!   "M/γ = 10" time constant is correct or whether the actual
//!   time constant is `2M/γ = 20`.
//!
//! - Probe B: §7 small-scale (n_traj=20, n_measure=20_000) with
//!   BOTH the merged-Welford std error AND a manual
//!   across-trajectories std error computed in parallel. Locks
//!   whether the across-trajectories estimator gives the
//!   chassis-predicted ~4.5% number while the merged estimator
//!   gives the broken `~100×-too-small` number.
//!
//! - Probe C: §7 sample variance vs theoretical Var(½v²) = 0.5
//!   for unit kT. Checks for any anomaly in the variance estimate
//!   itself.
//!
//! - Probe D: §7 deterministic-reproducibility — run twice with
//!   the same seed_base, both runs must produce identical
//!   global.mean and global.std_error_of_mean.
//!
//! All probes are `#[ignore]` so they don't run as part of
//! `cargo test`. Run them manually with:
//!
//!     cargo test -p sim-thermostat --release --test _phase1_findings_probes -- --ignored --nocapture

// Diagnostic scaffold — to be deleted in the next session per
// 06_findings/2026-04-09_phase1_statistical_propagation_chain.md.
// Wholesale clippy allows are appropriate here because the file is
// throw-away lab work, not production code.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::doc_markdown,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::imprecise_flops,
    clippy::missing_panics_doc,
    clippy::too_many_lines,
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss,
    clippy::needless_pass_by_value,
    clippy::similar_names,
    clippy::unreadable_literal,
    clippy::approx_constant,
    clippy::manual_assert,
    clippy::print_stdout,
    clippy::print_stderr,
    clippy::uninlined_format_args
)]

use sim_core::DVector;
use sim_thermostat::test_utils::WelfordOnline;
use sim_thermostat::{LangevinThermostat, PassiveStack};

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

// ─── Probe A: §10 decay envelope ─────────────────────────────────────

#[test]
#[ignore = "phase 1 diagnostic probe"]
fn probe_a_decay_envelope_at_multiple_checkpoints() {
    let mut model = sim_mjcf::load_model(SHO_1D_XML).unwrap();
    let mut data = model.make_data();

    let stack = PassiveStack::builder()
        .with(LangevinThermostat::new(
            DVector::from_element(model.nv, 0.1),
            1.0,
            0x00C0_FFEE_u64,
        ))
        .build();
    stack.install(&mut model);

    data.qpos[0] = 1.0;
    data.qvel[0] = 0.0;

    println!("\n=== Probe A: §10 decay envelope ===");
    println!("γ=0.1, M=1, k=1, h=0.001 (underdamped, ζ=0.05)");
    println!("Theoretical envelope: exp(-γt/(2M)) = exp(-0.05·t)\n");
    println!(
        "{:>10}  {:>8}  {:>14}  {:>14}  {:>14}  {:>14}",
        "step", "time", "qpos", "qvel", "amplitude", "exp(-0.05t)",
    );

    let _guard = stack.disable_stochastic();
    let checkpoints = [10_000, 20_000, 50_000, 100_000, 150_000, 200_000];
    let mut total_steps = 0_usize;

    for &target in &checkpoints {
        while total_steps < target {
            data.step(&model).unwrap();
            total_steps += 1;
        }
        let t = (total_steps as f64) * 0.001;
        let amplitude = (data.qpos[0] * data.qpos[0] + data.qvel[0] * data.qvel[0]).sqrt();
        let theoretical = (-0.05 * t).exp();
        println!(
            "{:>10}  {:>8.2}  {:>14.6e}  {:>14.6e}  {:>14.6e}  {:>14.6e}",
            total_steps, t, data.qpos[0], data.qvel[0], amplitude, theoretical,
        );
    }

    println!("\nObservation: if amplitude ≈ exp(-0.05t) at every checkpoint,");
    println!("the decay rate is γ/(2M)=0.05 (Finding A confirmed).");
    println!("If amplitude ≈ exp(-0.1t), the spec's M/γ=10 was right and we have");
    println!("a different failure mode to investigate.\n");
}

// ─── Probe B: §7 with both estimators in parallel ────────────────────

#[test]
#[ignore = "phase 1 diagnostic probe"]
fn probe_b_seven_with_across_trajectories_estimator() {
    let n_burn_in = 25_000;
    let n_measure = 20_000; // smaller than the gate (was 100_000) for faster diagnostic
    let n_traj = 20_usize; // smaller than the gate (was 100) for faster diagnostic
    let seed_base = 0x00C0_FFEE_u64;
    let k_b_t = 1.0_f64;
    let gamma_value = 0.1_f64;

    let mut merged = WelfordOnline::new();
    let mut across_trajectories = WelfordOnline::new();
    let mut trajectory_means: Vec<f64> = Vec::with_capacity(n_traj);

    for i in 0..n_traj {
        let mut model = sim_mjcf::load_model(SHO_1D_XML).unwrap();
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
            data.step(&model).unwrap();
        }

        let mut traj = WelfordOnline::new();
        for _ in 0..n_measure {
            data.step(&model).unwrap();
            traj.push(0.5 * data.qvel[0] * data.qvel[0]);
        }

        let traj_mean = traj.mean();
        merged.merge(&traj);
        across_trajectories.push(traj_mean);
        trajectory_means.push(traj_mean);
    }

    println!("\n=== Probe B: §7 with both std error estimators ===");
    println!("Parameters: n_traj={n_traj}, n_measure={n_measure}, n_burn_in={n_burn_in}");
    println!("Theoretical: ⟨½v²⟩ = ½kT = 0.5\n");

    println!("MERGED (current implementation, what spec §7.3 does):");
    println!("  count    = {}", merged.count());
    println!("  mean     = {:.6}", merged.mean());
    println!("  variance = {:.6}", merged.variance());
    println!(
        "  std_err  = {:.6e}  ← TOO SMALL (assumes IID)",
        merged.std_error_of_mean()
    );

    println!("\nACROSS-TRAJECTORIES (correct option β-A):");
    println!("  count    = {}", across_trajectories.count());
    println!("  mean     = {:.6}", across_trajectories.mean());
    println!("  variance = {:.6}", across_trajectories.variance());
    println!(
        "  std_err  = {:.6e}  ← CORRECT (100 IID samples)",
        across_trajectories.std_error_of_mean(),
    );

    println!("\nMean comparison (must match exactly — both are unweighted grand means):");
    println!("  merged.mean()           = {:.10}", merged.mean());
    println!(
        "  across_traj.mean()      = {:.10}",
        across_trajectories.mean()
    );
    println!(
        "  difference              = {:.6e}",
        (merged.mean() - across_trajectories.mean()).abs(),
    );

    println!("\nTheoretical comparison (kT=1, n_traj={n_traj}):");
    let per_traj_std_predicted = (1.0_f64 / 2.0).sqrt() / (10_f64).sqrt();
    let across_traj_std_predicted = per_traj_std_predicted / (n_traj as f64).sqrt();
    println!(
        "  Predicted per-trajectory std = σ(½v²)/√N_eff ≈ 0.707/√10 = {per_traj_std_predicted:.6}"
    );
    println!(
        "  Predicted across-traj std    ≈ {per_traj_std_predicted:.6} / √{n_traj} = {across_traj_std_predicted:.6}"
    );
    println!(
        "  Observed across-traj std     = {:.6} (variance.sqrt())",
        across_trajectories.variance().sqrt() / (n_traj as f64).sqrt(),
    );

    println!("\nThe N_eff scales linearly with n_measure, so for the full gate");
    println!("(n_measure=100_000, n_traj=100), expected ~0.0224 (= 4.5% of ½kT).");

    println!("\nFirst 10 trajectory means (sanity-check IID):");
    for (i, m) in trajectory_means.iter().take(10).enumerate() {
        println!("  traj[{i:>2}].mean() = {m:.6}");
    }
    println!();
}

// ─── Probe C: variance check ─────────────────────────────────────────

#[test]
#[ignore = "phase 1 diagnostic probe"]
fn probe_c_variance_check() {
    // Expected: Var(½v²) for v ~ N(0, kT/M) and M=1, kT=1 is
    //   Var(½v²) = (1/4)·Var(v²) = (1/4)·2(kT)² = 0.5
    // Observed in main run: variance ≈ 0.557 (10% inflated). This
    // is consistent with non-Gaussian tails from semi-implicit
    // Euler discretization (kurtosis ≈ 3.2 instead of 3.0, see
    // analysis in conversation).

    let n_burn_in = 25_000;
    let n_measure = 100_000;
    let n_traj = 5_usize;
    let seed_base = 0x00C0_FFEE_u64;

    println!("\n=== Probe C: variance vs theoretical ===");
    println!("Theoretical Var(½v²) for v~N(0,kT/M), M=1, kT=1: 0.5\n");

    for i in 0..n_traj {
        let mut model = sim_mjcf::load_model(SHO_1D_XML).unwrap();
        let mut data = model.make_data();

        let stack = PassiveStack::builder()
            .with(LangevinThermostat::new(
                DVector::from_element(model.nv, 0.1),
                1.0,
                seed_base + i as u64,
            ))
            .build();
        stack.install(&mut model);

        for _ in 0..n_burn_in {
            data.step(&model).unwrap();
        }

        let mut acc = WelfordOnline::new();
        let mut sum_v4 = 0.0_f64;
        for _ in 0..n_measure {
            data.step(&model).unwrap();
            let v2 = data.qvel[0] * data.qvel[0];
            acc.push(0.5 * v2);
            sum_v4 += v2 * v2;
        }
        let mean_v2 = 2.0 * acc.mean(); // ⟨v²⟩
        let mean_v4 = sum_v4 / (n_measure as f64);
        let kurtosis = mean_v4 / (mean_v2 * mean_v2); // E[v⁴]/E[v²]² (= 3 for Gaussian)
        println!(
            "traj[{i}]: ⟨½v²⟩={:.6}, var(½v²)={:.6}, ⟨v²⟩={:.6}, ⟨v⁴⟩={:.6}, kurtosis={:.4}",
            acc.mean(),
            acc.variance(),
            mean_v2,
            mean_v4,
            kurtosis,
        );
    }

    println!("\nObservation: kurtosis = 3.0 → Gaussian → Var(½v²) = 0.5 exactly.");
    println!("Kurtosis > 3.0 → leptokurtic → Var(½v²) > 0.5 (non-Gaussian tails).\n");
}

// ─── Probe E: FULL gate scale with both estimators ───────────────────

#[test]
#[ignore = "phase 1 diagnostic probe — full gate scale"]
fn probe_e_full_gate_scale_with_both_estimators() {
    let n_burn_in = 25_000;
    let n_measure = 100_000;
    let n_traj = 100_usize;
    let seed_base = 0x00C0_FFEE_u64;
    let k_b_t = 1.0_f64;
    let gamma_value = 0.1_f64;

    let mut merged = WelfordOnline::new();
    let mut across_trajectories = WelfordOnline::new();

    println!("\n=== Probe E: FULL §7 gate scale with both estimators ===");
    println!("100 traj × 100k steps each — same scale as the actual gate.");
    println!("Will take ~60s in release mode.\n");

    for i in 0..n_traj {
        let mut model = sim_mjcf::load_model(SHO_1D_XML).unwrap();
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
            data.step(&model).unwrap();
        }

        let mut traj = WelfordOnline::new();
        for _ in 0..n_measure {
            data.step(&model).unwrap();
            traj.push(0.5 * data.qvel[0] * data.qvel[0]);
        }

        merged.merge(&traj);
        across_trajectories.push(traj.mean());
    }

    let expected = 0.5 * k_b_t;
    println!("Expected: ½kT = {expected}");
    println!();
    println!("MERGED estimator (current spec §7.3 — INCORRECT for std error):");
    println!("  count    = {}", merged.count());
    println!("  mean     = {:.10}", merged.mean());
    println!("  variance = {:.10}", merged.variance());
    println!("  std_err  = {:.10e}", merged.std_error_of_mean());
    println!(
        "  deviation = {:.6e}, relative = {:.4}σ",
        (merged.mean() - expected).abs(),
        (merged.mean() - expected).abs() / merged.std_error_of_mean(),
    );

    println!();
    println!("ACROSS-TRAJECTORIES estimator (option β-A — CORRECT):");
    println!("  count    = {}", across_trajectories.count());
    println!("  mean     = {:.10}", across_trajectories.mean());
    println!("  variance = {:.10}", across_trajectories.variance());
    println!(
        "  std_err  = {:.10e}",
        across_trajectories.std_error_of_mean()
    );
    println!(
        "  deviation = {:.6e}, relative = {:.4}σ",
        (across_trajectories.mean() - expected).abs(),
        (across_trajectories.mean() - expected).abs() / across_trajectories.std_error_of_mean(),
    );

    println!();
    println!("Mean check (must be bit-identical — both are unweighted grand means):");
    println!(
        "  Δmean = {:.6e}",
        (merged.mean() - across_trajectories.mean()).abs(),
    );
    println!();
    println!("Spec §7.2 predicted std error: ~0.0224 (= 4.5% of ½kT = 0.0224·0.5 / 0.5 = 4.5%)",);
    println!(
        "Refined prediction (Probe B finding — shorter τ_int for SHO than chassis OU formula): ~0.015"
    );
    println!();
    println!("If across_trajectories deviation is < 3σ, the corrected gate WILL pass.");
}

// ─── Probe F: vary seed_base to check for systematic bias ───────────

#[test]
#[ignore = "phase 1 diagnostic probe — multiple gates, ~30s"]
fn probe_f_vary_seed_base() {
    // Run the full §7 gate with 6 different seed_base values.
    // For each, record the across-trajectories grand mean and the
    // deviation from 0.5. If the deviation is purely sampling noise,
    // signs should be roughly evenly split (50/50) and the mean of
    // deviations should be near zero. If there's a systematic bias,
    // signs will all be the same and the mean of deviations will
    // be the bias magnitude.
    let n_burn_in = 25_000;
    let n_measure = 100_000;
    let n_traj = 100_usize;
    let k_b_t = 1.0_f64;
    let gamma_value = 0.1_f64;
    let expected = 0.5 * k_b_t;

    let seed_bases = [
        0x00C0_FFEE_u64, // the gate's canonical seed
        0x0BAD_F00D_u64,
        0xCAFE_BABE_u64,
        0xDEAD_BEEF_u64,
        0xFEED_FACE_u64,
        0xFACE_F00D_u64,
    ];

    println!("\n=== Probe F: vary seed_base to check for systematic bias ===");
    println!("Full gate scale × 6 different seed bases.");
    println!("Expected: ½kT = {expected}");
    println!("Predicted across-traj std error from theory: ~0.024\n");
    println!(
        "{:>12}  {:>14}  {:>14}  {:>10}  {:>10}",
        "seed_base", "mean", "deviation", "σ", "sign",
    );

    let mut deviations: Vec<f64> = Vec::new();

    for &seed_base in &seed_bases {
        let mut across_trajectories = WelfordOnline::new();
        for i in 0..n_traj {
            let mut model = sim_mjcf::load_model(SHO_1D_XML).unwrap();
            let mut data = model.make_data();

            let stack = PassiveStack::builder()
                .with(LangevinThermostat::new(
                    DVector::from_element(model.nv, gamma_value),
                    k_b_t,
                    seed_base.wrapping_add(i as u64),
                ))
                .build();
            stack.install(&mut model);

            for _ in 0..n_burn_in {
                data.step(&model).unwrap();
            }
            let mut traj = WelfordOnline::new();
            for _ in 0..n_measure {
                data.step(&model).unwrap();
                traj.push(0.5 * data.qvel[0] * data.qvel[0]);
            }
            across_trajectories.push(traj.mean());
        }

        let mean = across_trajectories.mean();
        let std_err = across_trajectories.std_error_of_mean();
        let deviation = mean - expected;
        let n_sigma = deviation / std_err;
        let sign = if deviation > 0.0 { "+" } else { "−" };
        deviations.push(deviation);
        println!(
            "  {seed_base:>#012x}  {mean:>14.10}  {deviation:>+14.6e}  {n_sigma:>+10.4}  {sign:>10}",
        );
    }

    println!();
    let n = deviations.len();
    let mean_dev = deviations.iter().sum::<f64>() / (n as f64);
    let var_dev = deviations
        .iter()
        .map(|d| (d - mean_dev).powi(2))
        .sum::<f64>()
        / ((n - 1) as f64);
    let std_dev = var_dev.sqrt();
    let n_positive = deviations.iter().filter(|&&d| d > 0.0).count();

    println!("Summary across {n} gates:");
    println!("  mean of deviations  = {mean_dev:+.6e}");
    println!("  std  of deviations  = {std_dev:.6e}");
    println!("  # positive          = {n_positive} / {n}");
    println!();
    println!("Interpretation:");
    println!("  - mean of deviations near 0 → sampling noise, no bias");
    println!("  - sign split ~50/50 → sampling noise, no bias");
    println!("  - all same sign + mean ≫ std/√n → systematic bias");
    println!();
    println!("Theoretical std of mean across gates: ~0.024 (matches single-gate std error)");
}

// ─── Probe D: deterministic reproducibility ──────────────────────────

#[test]
#[ignore = "phase 1 diagnostic probe"]
fn probe_d_seven_reproducibility() {
    fn run_small_gate() -> (f64, f64, usize) {
        let n_burn_in = 25_000;
        let n_measure = 20_000;
        let n_traj = 10_usize;
        let seed_base = 0x00C0_FFEE_u64;

        let mut merged = WelfordOnline::new();
        for i in 0..n_traj {
            let mut model = sim_mjcf::load_model(SHO_1D_XML).unwrap();
            let mut data = model.make_data();

            let stack = PassiveStack::builder()
                .with(LangevinThermostat::new(
                    DVector::from_element(model.nv, 0.1),
                    1.0,
                    seed_base + i as u64,
                ))
                .build();
            stack.install(&mut model);

            for _ in 0..n_burn_in {
                data.step(&model).unwrap();
            }
            let mut traj = WelfordOnline::new();
            for _ in 0..n_measure {
                data.step(&model).unwrap();
                traj.push(0.5 * data.qvel[0] * data.qvel[0]);
            }
            merged.merge(&traj);
        }
        (merged.mean(), merged.std_error_of_mean(), merged.count())
    }

    let (mean1, se1, n1) = run_small_gate();
    let (mean2, se2, n2) = run_small_gate();

    println!("\n=== Probe D: deterministic reproducibility ===");
    println!("Run 1: mean={mean1:.10}, std_err={se1:.10e}, count={n1}");
    println!("Run 2: mean={mean2:.10}, std_err={se2:.10e}, count={n2}");
    println!("Δmean = {:.6e}", (mean1 - mean2).abs());
    println!("Δse   = {:.6e}", (se1 - se2).abs());
    println!("If both Δs are 0.0, the simulation is deterministic and the");
    println!("failure mode is reproducible — not a flake.\n");

    assert_eq!(mean1, mean2, "deterministic mean should match bit-for-bit");
    assert_eq!(se1, se2, "deterministic std error should match bit-for-bit");
}
