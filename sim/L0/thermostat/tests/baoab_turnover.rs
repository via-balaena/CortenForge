//! R6 — BAOAB reproduces the Kramers turnover (the EM sim could not).
//!
//! The R1 finding was that the production Euler–Maruyama path tracks the
//! *spatial-diffusion* rate even at γ=0.1 (sim/k_turnover ≈ 3.8), missing the
//! energy-diffusion suppression. This test runs the same γ-sweep through the
//! BAOAB integrator. Result: BAOAB shows the turnover and matches the analytic
//! rate overdamped, AND shows the energy-diffusion suppression EM missed
//! (BAOAB/k_S ≈ 0.84 at γ=0.1, where EM gave ≈1.0). It also reveals that the
//! Meľnikov–Meshkov *analytic* factor over-suppresses at moderate δ≈0.5 — so the
//! BAOAB integrator itself (not the closed-form formula) is the rate engine for
//! the high-Q beam.
//!
//! Heavy; run with `--release`:
//! `cargo test -p sim-thermostat --release --test baoab_turnover -- --ignored --nocapture`

#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::float_cmp,
    clippy::too_many_lines,
    clippy::doc_markdown
)]

use sim_thermostat::{Baoab1D, DoubleWellPotential};

const DELTA_V: f64 = 3.0;
const X_0: f64 = 1.0;
const MASS: f64 = 1.0;
const KT: f64 = 1.0;
const DT: f64 = 0.001;
const SWITCH_THRESH: f64 = X_0 * 0.5;
const SEED_BASE: u64 = 20_260_607;

/// Measured escape rate (transitions/time) from a BAOAB run.
fn baoab_rate(well: &DoubleWellPotential, gamma: f64, n_steps: usize, seed: u64) -> f64 {
    let mut sim = Baoab1D::new(well, MASS, gamma, KT, DT, seed, X_0);
    for _ in 0..50_000 {
        sim.step();
    }
    let mut x = sim.position();
    let mut state = if x >= 0.0 { 1.0 } else { -1.0 };
    let mut switches = 0usize;
    for _ in 0..n_steps {
        sim.step();
        x = sim.position();
        if state > 0.0 && x < -SWITCH_THRESH {
            state = -1.0;
            switches += 1;
        } else if state < 0.0 && x > SWITCH_THRESH {
            state = 1.0;
            switches += 1;
        }
    }
    switches as f64 / (n_steps as f64 * DT)
}

#[test]
#[ignore = "heavy; run with --release"]
fn baoab_reproduces_kramers_turnover() {
    let well = DoubleWellPotential::new(DELTA_V, X_0, 0);
    let gammas = [0.1_f64, 0.3, 1.0, 3.0, 10.0, 30.0];

    println!("\nR6 — BAOAB Kramers turnover (ΔV={DELTA_V}, kT={KT}); 2 seeds/γ");
    println!(
        "{:>6} {:>11} {:>11} {:>11} {:>9} {:>9}",
        "γ", "baoab", "k_S", "k_turn", "b/k_S", "b/k_T"
    );

    let mut rows = Vec::new();
    for (i, &g) in gammas.iter().enumerate() {
        let k_turn = well.kramers_rate_turnover(g, MASS, KT);
        let n_steps = ((60.0 / (k_turn * DT)) as usize).clamp(2_000_000, 30_000_000);
        let r = (0..2)
            .map(|s| baoab_rate(&well, g, n_steps, SEED_BASE + (i * 2 + s) as u64))
            .sum::<f64>()
            / 2.0;
        let k_s = well.kramers_rate(g, MASS, KT);
        println!(
            "{g:>6.2} {r:>11.5} {k_s:>11.5} {k_turn:>11.5} {:>9.3} {:>9.3}",
            r / k_s,
            r / k_turn
        );
        rows.push((g, r, k_s, k_turn));
    }

    let r_mid = rows[2].1; // γ=1.0 (near the rate peak)
    let r_hi = rows[5].1; // γ=30 (overdamped)

    // ── 1. The rate falls from the peak into the overdamped regime (the
    //    high-friction branch of the turnover).
    assert!(
        r_mid > r_hi,
        "rate should fall from the peak γ=1→30: {r_mid:.5}→{r_hi:.5}"
    );

    // ── 2. Where the integrator and the formula are both reliable (γ ≥ 3, where
    //    Υ→1 so k_S ≈ k_turnover), BAOAB matches the analytic rate.
    for &(g, r, _k_s, k_turn) in rows.iter().filter(|row| row.0 >= 3.0) {
        let ratio = r / k_turn;
        assert!(
            (0.7..1.3).contains(&ratio),
            "γ={g}: BAOAB/k_turnover = {ratio:.3} outside [0.7,1.3]"
        );
    }

    // ── 3. THE FINDING: underdamped, BAOAB shows energy-diffusion suppression
    //    that Euler–Maruyama completely missed — at γ=0.1, BAOAB/k_S ≈ 0.84
    //    (suppressed below the spatial-diffusion rate) where EM gave ≈1.0.
    let (_, b_lo, k_s_lo, k_turn_lo) = rows[0];
    assert!(
        b_lo / k_s_lo < 0.95,
        "γ=0.1: BAOAB should be suppressed below k_S (EM was not); BAOAB/k_S = {:.3}",
        b_lo / k_s_lo
    );

    // ── 4. ...but BAOAB (the trustworthy reference) shows the Meľnikov–Meshkov
    //    *analytic* factor OVER-predicts the suppression at moderate δ≈0.5:
    //    BAOAB/k_turnover ≫ 1. ⇒ for quantitative underdamped rates use the BAOAB
    //    integrator directly; treat the MM formula as a rough guide here.
    assert!(
        b_lo / k_turn_lo > 1.5,
        "γ=0.1: expected MM to over-suppress (BAOAB/k_turnover > 1.5); got {:.2}",
        b_lo / k_turn_lo
    );

    println!(
        "\nR6 result: BAOAB is a validated underdamped integrator (⟨v²⟩=kT/m, dt-independent). It"
    );
    println!(
        "shows the energy-diffusion suppression EM missed, AND reveals the MM analytic factor"
    );
    println!(
        "over-suppresses at moderate δ — so use the BAOAB integrator for high-Q rate predictions."
    );
}
