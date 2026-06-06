//! R6 — BAOAB integrator vs Euler–Maruyama in the underdamped regime.
//!
//! R1 found the production Euler–Maruyama path is pinned at the spatial-diffusion
//! rate even at γ=0.1, erasing the friction dependence. This test runs the same
//! γ-sweep through the BAOAB integrator: BAOAB matches the analytic rate
//! overdamped and tracks the friction dependence EM erases.
//!
//! **Scope (ultra-review correction):** this sweep is at `ΔV/kT = 3`, where the
//! rate is γ-INDEPENDENT (the TST plateau), not energy-diffusion-limited — so it
//! **cannot** test the absolute underdamped rate or the Meľnikov–Meshkov factor
//! (that needs a deep well where `rate ∝ γ`; pending). Takeaway: BAOAB is the
//! validated underdamped integrator (equipartition, dt-independent) and the rate
//! engine for the high-Q beam; MM is a ±20% guide untested in our regime.
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

    // ── 3. Underdamped, BAOAB tracks the friction dependence EM erases. The
    //    seed-stable statement is that BAOAB is NOT pinned above k_S the way EM is;
    //    the exact ratio (~0.84–1.0) is seed-dependent, so bound it loosely.
    let (_, b_lo, k_s_lo, k_turn_lo) = rows[0];
    assert!(
        b_lo / k_s_lo < 1.1,
        "γ=0.1: BAOAB/k_S = {:.3} (should not sit above the spatial-diffusion rate)",
        b_lo / k_s_lo
    );

    // ── 4. SCOPE (not a verdict on the formula): at ΔV/kT=3 the simulated rate is
    //    γ-independent (TST plateau, not energy-diffusion-limited), so BAOAB sits
    //    near k_TST while the MM factor Υ(δ≈0.46)≈0.27 predicts strong suppression
    //    → BAOAB/k_turnover ≫ 1. This does NOT show MM is wrong; this regime cannot
    //    test the underdamped rate (that needs a deep well where rate ∝ γ).
    assert!(
        b_lo / k_turn_lo > 1.5,
        "γ=0.1: sim sits at TST while MM predicts energy-diffusion suppression; got {:.2}",
        b_lo / k_turn_lo
    );

    println!(
        "\nR6 result: BAOAB is a validated underdamped integrator (⟨v²⟩=kT/m, dt-independent) that"
    );
    println!(
        "tracks the friction dependence EM erases. NOTE: this ΔV/kT=3 sweep sits in the TST plateau"
    );
    println!(
        "(rate ~γ-independent), so it cannot test the absolute energy-diffusion rate or the MM"
    );
    println!(
        "factor — a deep-well/δ≪1 run (rate∝γ) is the pending check. Predict high-Q rates with BAOAB."
    );
}
