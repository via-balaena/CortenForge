//! S2 — in-silico Kramers prediction harness for the D4 physical p-bit.
//!
//! This is the **prediction** the physical bistable beam's measurements get
//! checked against (`docs/thermo_computing/03_phases/d4_physical_pbit/recon.md`,
//! gate G1). It runs the *real* CortenForge sim path — `LangevinThermostat`
//! driving a `DoubleWellPotential` on a 1-DOF sim-core model, the same code the
//! whole thermo stack rests on — across a temperature sweep, and confirms three
//! things the bench experiment will measure:
//!
//! 1. **Kramers/Arrhenius law.** The switching rate vs noise level: a plot of
//!    `ln(rate)` against `1/kT` is a straight line of slope `−ΔV`. This is the
//!    headline curve; the printed table is the prediction the physical p-bit is
//!    overlaid on.
//! 2. **Exponential dwell times** (Poisson escape): the coefficient of variation
//!    of dwell times in a well is ≈ 1.
//! 3. **Equipartition** `⟨δx²⟩ = kT / (M·ω_a²)` — the relation the physical rig
//!    uses to calibrate the effective temperature `kT_eff` from in-well jitter.
//!
//! The simulated switching rate is also cross-checked against the analytic
//! `DoubleWellPotential::kramers_rate` (Phase 3's validated formula).
//!
//! Heavy; run explicitly with `--release`:
//! `cargo test -p sim-therm-env --release --test kramers_pbit -- --ignored --nocapture`

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::float_cmp,
    clippy::too_many_lines,
    clippy::doc_markdown,
    // false positive on the OLS slope denominator `n·Σx² − (Σx)²`.
    clippy::suspicious_operation_groupings
)]

use sim_rl::{Environment, Tensor};
use sim_therm_env::ThermCircuitEnv;
use sim_thermostat::DoubleWellPotential;

// ─── Central parameters (Phase 3 validated point: kramers_rate ≈ 0.01214 @kT=1) ─
const DELTA_V: f64 = 3.0;
const X_0: f64 = 1.0;
const GAMMA: f64 = 10.0;
const MASS: f64 = 1.0; // generated MJCF particle mass = 1
const TIMESTEP: f64 = 0.001;
const SUB_STEPS: usize = 100;
const SEED_BASE: u64 = 20_260_605;

/// Sim-time advanced per `env.step()` call.
const DT_PER_STEP: f64 = TIMESTEP * SUB_STEPS as f64;

/// Switch-detection hysteresis: count a transition only once the particle is
/// well past the barrier (avoids counting barrier-top jitter as switches).
const SWITCH_THRESH: f64 = X_0 * 0.5;

/// Target switch count per temperature, used to size each run.
const TARGET_SWITCHES: f64 = 50.0;

struct WellStats {
    kt: f64,
    sim_rate: f64,
    kramers_rate: f64,
    n_switches: usize,
    dwell_cv: f64,
    in_well_var: f64,
    equipartition_pred: f64,
}

/// Run a single fixed-temperature trajectory and measure switching + in-well
/// statistics. `kt` is the effective temperature (base `k_b_t = 1`, set via the
/// ctrl-temperature channel each step).
fn run_kt(well: &DoubleWellPotential, kt: f64, seed: u64) -> WellStats {
    // Size the run from the analytic rate so every temperature sees ~TARGET
    // switches, clamped to a sane band.
    let analytic = well.kramers_rate(GAMMA, MASS, kt);
    let n_steps = ((TARGET_SWITCHES / (analytic * DT_PER_STEP)) as usize).clamp(3_000, 15_000);

    let mut env = ThermCircuitEnv::builder(1)
        .gamma(GAMMA)
        .k_b_t(1.0)
        .timestep(TIMESTEP)
        .sub_steps(SUB_STEPS)
        .episode_steps(n_steps + 1_000)
        .seed(seed)
        .with_ctrl_temperature()
        .with(DoubleWellPotential::new(DELTA_V, X_0, 0))
        .reward(|_, _| 0.0)
        .build()
        .unwrap();
    env.reset().unwrap();
    let action = Tensor::from_slice(&[kt as f32], &[1]);

    // Equilibrate (τ_eq = M/γ = 0.1 ≪ burn-in time).
    for _ in 0..500 {
        env.step(&action).unwrap();
    }

    let mut x = env.data().qpos[0];
    let mut state = if x >= 0.0 { 1.0 } else { -1.0 };
    let mut t = 0.0_f64;
    let mut last_switch_t = 0.0_f64;
    let mut dwell: Vec<f64> = Vec::new();
    let mut var_acc = 0.0_f64;
    let mut var_n = 0u64;

    for _ in 0..n_steps {
        env.step(&action).unwrap();
        t += DT_PER_STEP;
        x = env.data().qpos[0];

        // Hysteretic switch detection.
        if state > 0.0 && x < -SWITCH_THRESH {
            state = -1.0;
            dwell.push(t - last_switch_t);
            last_switch_t = t;
        } else if state < 0.0 && x > SWITCH_THRESH {
            state = 1.0;
            dwell.push(t - last_switch_t);
            last_switch_t = t;
        }

        // In-well jitter about the current well minimum (for equipartition).
        if x.abs() > SWITCH_THRESH {
            let d = x - state * X_0;
            var_acc += d * d;
            var_n += 1;
        }
    }

    let n_switches = dwell.len();
    let sim_rate = n_switches as f64 / t;
    let dwell_cv = coefficient_of_variation(&dwell);
    let in_well_var = if var_n > 0 {
        var_acc / var_n as f64
    } else {
        f64::NAN
    };
    let omega_a = well.omega_a(MASS);
    let equipartition_pred = kt / (MASS * omega_a * omega_a);

    WellStats {
        kt,
        sim_rate,
        kramers_rate: analytic,
        n_switches,
        dwell_cv,
        in_well_var,
        equipartition_pred,
    }
}

fn coefficient_of_variation(xs: &[f64]) -> f64 {
    if xs.len() < 2 {
        return f64::NAN;
    }
    let n = xs.len() as f64;
    let mean = xs.iter().sum::<f64>() / n;
    let var = xs.iter().map(|x| (x - mean) * (x - mean)).sum::<f64>() / n;
    var.sqrt() / mean
}

/// Ordinary least-squares slope of `ys` vs `xs`.
fn lstsq_slope(xs: &[f64], ys: &[f64]) -> f64 {
    let n = xs.len() as f64;
    let sx: f64 = xs.iter().sum();
    let sy: f64 = ys.iter().sum();
    let sxx: f64 = xs.iter().map(|x| x * x).sum();
    let sxy: f64 = xs.iter().zip(ys).map(|(x, y)| x * y).sum();
    (n * sxy - sx * sy) / (n * sxx - sx * sx)
}

#[test]
#[ignore = "heavy; run with --release"]
fn kramers_law_matches_sim() {
    let well = DoubleWellPotential::new(DELTA_V, X_0, 0);
    let temps = [2.0_f64, 2.5, 3.0, 3.5, 4.0];

    println!(
        "\nD4 / S2 — in-silico Kramers prediction (ΔV={DELTA_V}, x₀={X_0}, γ={GAMMA}, M={MASS})"
    );
    println!(
        "{:>5} {:>8} {:>12} {:>12} {:>8} {:>8} {:>12} {:>12}",
        "kT", "ΔV/kT", "sim_rate", "kramers", "switches", "dwellCV", "<δx²>_sim", "<δx²>_pred"
    );

    let mut inv_kt = Vec::new();
    let mut ln_rate = Vec::new();
    let mut stats = Vec::new();

    for (i, &kt) in temps.iter().enumerate() {
        let s = run_kt(&well, kt, SEED_BASE + i as u64);
        println!(
            "{:>5.2} {:>8.3} {:>12.5} {:>12.5} {:>8} {:>8.3} {:>12.5} {:>12.5}",
            s.kt,
            DELTA_V / s.kt,
            s.sim_rate,
            s.kramers_rate,
            s.n_switches,
            s.dwell_cv,
            s.in_well_var,
            s.equipartition_pred
        );
        inv_kt.push(1.0 / kt);
        ln_rate.push(s.sim_rate.ln());
        stats.push(s);
    }

    // ── Assertion 1: Arrhenius slope ≈ −ΔV (the headline law) ──────────────
    let slope = lstsq_slope(&inv_kt, &ln_rate);
    println!(
        "\nArrhenius fit: ln(rate) vs 1/kT slope = {slope:.4}  (expected −ΔV = {:.4})",
        -DELTA_V
    );
    let slope_err = (slope + DELTA_V).abs() / DELTA_V;
    assert!(
        slope_err < 0.20,
        "Arrhenius slope {slope:.4} not within 20% of −ΔV={:.4} (err {:.1}%)",
        -DELTA_V,
        slope_err * 100.0
    );

    // ── Assertion 2: sim switching rate tracks analytic Kramers (within ~2×) ─
    for s in &stats {
        let ratio = s.sim_rate / s.kramers_rate;
        assert!(
            (0.4..2.5).contains(&ratio),
            "kT={:.2}: sim_rate {:.5} vs kramers {:.5} (ratio {:.2}) outside [0.4, 2.5]",
            s.kt,
            s.sim_rate,
            s.kramers_rate,
            ratio
        );
    }

    // ── Assertion 3: dwell times are ~exponential (Poisson escape) ─────────
    let mean_cv = stats.iter().map(|s| s.dwell_cv).sum::<f64>() / stats.len() as f64;
    assert!(
        (0.6..1.4).contains(&mean_cv),
        "mean dwell-time CV {mean_cv:.3} not ≈1 (expected exponential)"
    );

    // ── Assertion 4: equipartition holds COLD — the kT_eff calibration regime ─
    // At the switching temperatures above, kT ≈ ΔV, so the particle explores the
    // anharmonic walls and ⟨δx²⟩ falls BELOW the harmonic value kT/(M·ω_a²) (see
    // the <δx²> columns). Equipartition — the relation the rig uses to read
    // kT_eff from in-well jitter — is only quantitative when the bit is COLD
    // (kT ≪ ΔV, rarely switching). So the physical calibration must be taken at
    // low drive, deep in one well. Validate it there.
    let cold = run_kt(&well, 0.3, SEED_BASE + 100);
    let cold_ratio = cold.in_well_var / cold.equipartition_pred;
    println!(
        "\nCold equipartition check (kT={:.2}, {} switches): <δx²>_sim={:.5} vs kT/(Mω_a²)={:.5}  ratio={:.3}",
        cold.kt, cold.n_switches, cold.in_well_var, cold.equipartition_pred, cold_ratio
    );
    assert!(
        (0.85..1.25).contains(&cold_ratio),
        "cold equipartition ratio {cold_ratio:.3} not ≈1 — the kT_eff calibration relation is broken"
    );

    println!(
        "\nAll gates passed: the sim reproduces the Kramers law (slope≈−ΔV), exponential dwell"
    );
    println!(
        "times, and (cold) equipartition — the three things the physical p-bit is checked against."
    );
    println!(
        "Calibration note: read kT_eff from in-well jitter ONLY at low drive (kT ≪ ΔV); at the"
    );
    println!("switching temperatures anharmonicity suppresses ⟨δx²⟩ below the harmonic value.");
}
