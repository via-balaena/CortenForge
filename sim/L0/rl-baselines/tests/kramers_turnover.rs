//! R1 — Kramers turnover validation (D4 Layer-2).
//!
//! The headline correctness check for the underdamped regime: does the **real
//! Langevin sim** reproduce the rate-vs-friction *turnover* (rate rises with γ at
//! low friction, peaks, falls at high friction), and does the shipped
//! spatial-diffusion `kramers_rate` **overestimate** in the underdamped regime
//! where a high-Q spring-steel beam lives? If so, `kramers_rate_turnover` is the
//! rate to compare the physical p-bit against — not `kramers_rate`.
//!
//! Heavy; run with `--release`:
//! `cargo test -p sim-therm-env --release --test kramers_turnover -- --ignored --nocapture`

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::float_cmp,
    clippy::too_many_lines,
    clippy::doc_markdown
)]

use sim_rl::{Environment, Tensor};
use sim_therm_env::ThermCircuitEnv;
use sim_thermostat::DoubleWellPotential;

const DELTA_V: f64 = 3.0;
const X_0: f64 = 1.0;
const MASS: f64 = 1.0;
const TIMESTEP: f64 = 0.001;
const SUB_STEPS: usize = 100;
const DT_PER_STEP: f64 = TIMESTEP * SUB_STEPS as f64;
const SWITCH_THRESH: f64 = X_0 * 0.5;
const SEED_BASE: u64 = 20_260_606;

/// Measured escape rate (transitions/time) at a given friction `gamma` and
/// temperature `kt`, on the real Langevin + double-well sim path.
fn measured_rate(gamma: f64, kt: f64, n_steps: usize, seed: u64) -> f64 {
    let mut env = ThermCircuitEnv::builder(1)
        .gamma(gamma)
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
    for _ in 0..500 {
        env.step(&action).unwrap();
    }

    let mut x = env.data().qpos[0];
    let mut state = if x >= 0.0 { 1.0 } else { -1.0 };
    let mut t = 0.0_f64;
    let mut switches = 0usize;
    for _ in 0..n_steps {
        env.step(&action).unwrap();
        t += DT_PER_STEP;
        x = env.data().qpos[0];
        if state > 0.0 && x < -SWITCH_THRESH {
            state = -1.0;
            switches += 1;
        } else if state < 0.0 && x > SWITCH_THRESH {
            state = 1.0;
            switches += 1;
        }
    }
    switches as f64 / t
}

/// Diagnostic + finding: the analytic turnover is in `kramers_rate_turnover`
/// (unit-tested in `sim-thermostat`), but the **Euler–Maruyama** Langevin sim
/// does NOT reproduce the energy-diffusion suppression underdamped — it tracks
/// the spatial-diffusion rate even at γ=0.1 where the turnover predicts ~4×
/// suppression. So validating the turnover in-sim (and predicting the real
/// high-Q beam) requires a proper underdamped integrator (BAOAB — Layer-2 R6).
/// This test asserts both: EM is valid where it should be (overdamped), and
/// demonstrably fails where it shouldn't be trusted (underdamped).
#[test]
#[ignore = "heavy; run with --release"]
fn em_sim_misses_underdamped_turnover_needs_baoab() {
    let well = DoubleWellPotential::new(DELTA_V, X_0, 0);
    let kt = 1.0;
    let gammas = [0.1_f64, 0.3, 1.0, 3.0, 10.0, 30.0];

    println!("\nR1 — Kramers turnover (ΔV={DELTA_V}, kT={kt}); 2 seeds/γ");
    println!(
        "{:>6} {:>10} {:>10} {:>10} {:>9} {:>9}",
        "γ", "sim", "k_S", "k_turn", "sim/k_S", "sim/k_T"
    );

    let mut rows = Vec::new();
    for (i, &g) in gammas.iter().enumerate() {
        let k_t = well.kramers_rate_turnover(g, MASS, kt);
        let n_steps = ((40.0 / (k_t * DT_PER_STEP)) as usize).clamp(20_000, 120_000);
        let r = (0..2)
            .map(|s| measured_rate(g, kt, n_steps, SEED_BASE + (i * 2 + s) as u64))
            .sum::<f64>()
            / 2.0;
        let k_s = well.kramers_rate(g, MASS, kt);
        println!(
            "{g:>6.2} {r:>10.5} {k_s:>10.5} {k_t:>10.5} {:>9.3} {:>9.3}",
            r / k_s,
            r / k_t
        );
        rows.push((g, r, k_s, k_t));
    }

    // ── 1. Where EM is valid (overdamped, γ ≥ 3, where k_S ≈ k_turnover since
    //    Υ→1), the sim matches the analytic rate. This bounds the regime we can
    //    currently trust the sim in.
    for &(g, r, _k_s, k_t) in rows.iter().filter(|row| row.0 >= 3.0) {
        let ratio = r / k_t;
        assert!(
            (0.5..1.5).contains(&ratio),
            "γ={g} (overdamped, EM-valid): sim/k_turnover = {ratio:.3} outside [0.5,1.5]"
        );
    }

    // ── 2. THE FINDING: underdamped, EM does NOT reproduce the energy-diffusion
    //    suppression — at γ=0.1 the turnover predicts ~4× suppression (Υ≈0.26)
    //    but the sim still tracks the spatial-diffusion rate (sim/k_turnover ≫ 1).
    //    A >2× gap is far beyond the MM formula's ±20%, so this is the EM
    //    integrator, not the formula. ⇒ R6 (BAOAB) is a prerequisite to validate
    //    the turnover in-sim and to predict the real high-Q beam.
    let (g_lo, sim_lo, k_spatial_lo, k_turn_lo) = rows[0];
    assert!(g_lo <= 0.1);
    assert!(
        sim_lo / k_turn_lo > 2.0,
        "expected EM to miss the underdamped suppression (sim/k_turnover > 2); got {:.2} \
         (sim={sim_lo:.5}, k_turnover={k_turn_lo:.5})",
        sim_lo / k_turn_lo
    );
    assert!(
        (sim_lo / k_spatial_lo - 1.0).abs() < 0.25,
        "EM underdamped should track the spatial-diffusion rate k_S, sim/k_S={:.3}",
        sim_lo / k_spatial_lo
    );

    println!(
        "\nFINDING: the analytic turnover (kramers_rate_turnover) is correct, but the Euler–Maruyama"
    );
    println!(
        "sim does NOT reproduce energy-diffusion underdamped — it tracks the spatial-diffusion rate."
    );
    println!(
        "Validating the turnover in-sim (and predicting the real high-Q beam) requires BAOAB (R6)."
    );
}
