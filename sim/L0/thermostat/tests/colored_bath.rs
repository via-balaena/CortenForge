//! R2 — when does injected (shaker) noise behave as a thermal bath?
//!
//! Drives a deep quartic well (no escape) with external OU colored noise of
//! correlation time `τ` and measures the kinetic temperature `m⟨v²⟩` against the
//! configurational temperature `⟨V′²⟩/⟨V″⟩`. For a Boltzmann state these are equal
//! (equipartition). The sweep finds the bandwidth condition: the in-well state
//! keeps a Boltzmann *shape* while `τ·ω_a ≪ 1` and departs as `τ·ω_a` grows — the
//! rig rule that the shaker must be driven *broadband*. **Scope:** this tests
//! shape, not absolute temperature (the OU rolloff also suppresses `kT`), and the
//! deep no-escape well does not probe escape / barrier-top distortion.
//!
//! Heavy; run with `--release`:
//! `cargo test -p sim-thermostat --release --test colored_bath -- --ignored --nocapture`

#![allow(
    clippy::cast_precision_loss,
    clippy::float_cmp,
    clippy::too_many_lines,
    clippy::doc_markdown
)]

use sim_thermostat::{ColoredDriveSim, DoubleWellPotential};

#[test]
#[ignore = "heavy; run with --release"]
fn injected_noise_is_thermal_only_when_broadband() {
    let well = DoubleWellPotential::new(10.0, 1.0, 0); // deep: no escape
    let (mass, gamma, kt_eff, dt) = (1.0, 1.0, 1.0, 0.001);
    let omega_a = well.omega_a(mass); // √80 ≈ 8.94 → 1/ω_a ≈ 0.112
    let taus = [0.002_f64, 0.01, 0.03, 0.1, 0.3, 1.0];

    println!(
        "\nR2 — injected colored-noise bath (ΔV=10, kT_eff(white)={kt_eff}, ω_a={omega_a:.2})"
    );
    println!(
        "{:>7} {:>8} {:>10} {:>10} {:>9}",
        "tau", "tau·ω_a", "kT_kin", "kT_conf", "kin/conf"
    );

    let mut rows = Vec::new();
    for (i, &tau) in taus.iter().enumerate() {
        let mut sim = ColoredDriveSim::new(
            &well,
            mass,
            gamma,
            kt_eff,
            tau,
            dt,
            20_260_607 + i as u64,
            1.0,
        );
        for _ in 0..100_000 {
            sim.step();
        }
        let n = 3_000_000usize;
        let (mut sum_v2, mut sum_fp2, mut sum_curv) = (0.0, 0.0, 0.0);
        for _ in 0..n {
            sim.step();
            sum_v2 += sim.velocity() * sim.velocity();
            let (fp2, curv) = sim.config_temp_terms();
            sum_fp2 += fp2;
            sum_curv += curv;
        }
        let kt_kin = mass * sum_v2 / n as f64;
        let kt_conf = sum_fp2 / sum_curv;
        let ratio = kt_kin / kt_conf;
        println!(
            "{tau:>7.3} {:>8.3} {kt_kin:>10.5} {kt_conf:>10.5} {ratio:>9.3}",
            tau * omega_a
        );
        rows.push((tau, tau * omega_a, ratio));
    }

    // Broadband (τ·ω_a ≪ 1): kinetic ≈ configurational ⇒ thermal/Boltzmann.
    let (_, twa_lo, ratio_lo) = rows[0];
    assert!(
        twa_lo < 0.05,
        "first τ should be broadband (τ·ω_a≪1), got {twa_lo:.3}"
    );
    assert!(
        (0.85..1.15).contains(&ratio_lo),
        "broadband bath should be thermal (kin/conf≈1); got {ratio_lo:.3}"
    );

    // Narrow band (τ·ω_a ≫ 1): kinetic ≪ configurational ⇒ NON-thermal. This is
    // the failure mode the rig must avoid (~50% divergence by τ·ω_a ≈ 9).
    let (_, twa_hi, ratio_hi) = rows[rows.len() - 1];
    assert!(
        twa_hi > 3.0,
        "last τ should be narrow-band (τ·ω_a≫1), got {twa_hi:.3}"
    );
    assert!(
        ratio_hi < 0.65,
        "narrow-band drive should be clearly non-thermal (kin/conf<0.65); got {ratio_hi:.3}"
    );

    // Thermal-ness degrades monotonically as the band narrows.
    for pair in rows.windows(2) {
        assert!(
            pair[1].2 <= pair[0].2 + 0.02,
            "kin/conf should fall as τ grows: {:.3}→{:.3}",
            pair[0].2,
            pair[1].2
        );
    }

    // Quantitative rule: keeping the Boltzmann SHAPE (kin/conf ratio > 0.95)
    // requires τ·ω_a ≲ 0.3, i.e. drive bandwidth ≳ a few × ω_a. NOTE this is shape
    // only — at τ·ω_a ≈ 0.27 the ABSOLUTE kT is already ~10% below kT_eff (the OU
    // rolloff under-drives the well), so calibrate kT from the measured variance.
    let near = rows.iter().find(|r| (0.2..0.35).contains(&r.1)).copied();
    let far = rows.iter().find(|r| (2.0..4.0).contains(&r.1)).copied();
    if let (Some(near), Some(far)) = (near, far) {
        assert!(
            near.2 > 0.94,
            "τ·ω_a≈{:.2}: should still be ~thermal, got {:.3}",
            near.1,
            near.2
        );
        assert!(
            far.2 < 0.85,
            "τ·ω_a≈{:.2}: should have departed, got {:.3}",
            far.1,
            far.2
        );
    }

    println!(
        "\nR2 result: injected noise preserves the in-well BOLTZMANN SHAPE only when broadband"
    );
    println!("(τ·ω_a ≪ 1). RIG RULE: drive broadband — bandwidth ≳ a few × ω_a (τ·ω_a ≲ 0.3 keeps");
    println!(
        "kin/conf within ~5%). SCOPE: this is SHAPE, not absolute T — the realized kT is already"
    );
    println!(
        "~10% below nominal at τ·ω_a≈0.3 (OU rolloff); calibrate kT from the MEASURED in-well variance."
    );
    println!(
        "Escape / barrier-top distortion / T_eff(noise power) under colored drive are not yet tested."
    );
}
