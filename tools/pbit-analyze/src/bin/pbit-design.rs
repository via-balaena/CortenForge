//! `pbit-design` — magnetoelastic rig design sweep (D4 Layer-2 R3).
//!
//! Sweeps the fixed-magnet axial gap for a default rig and reports the predicted
//! double well: bistability, barrier `ΔV`, well separation `x₀`, attempt
//! frequency, and the **RMS tip amplitude** required to reach `ΔV/kT_eff = 3` —
//! i.e. the feasibility table that specs the rig from physics. The feasible
//! window is where the beam is bistable *and* that RMS amplitude is large enough
//! for the Hall sensor to resolve yet small enough to be physical.
//!
//! Run: `cargo run -p pbit-analyze --bin pbit-design --release`

#![allow(clippy::cast_precision_loss)]

use pbit_analyze::magnetoelastic::{Beam, Magnet, Rig};

const TARGET_RATIO: f64 = 3.0;
/// RMS amplitude resolvable by the Hall sensor yet physically small (mm).
const RMS_MIN_MM: f64 = 0.02;
const RMS_MAX_MM: f64 = 3.0;
/// In-well RMS acceleration scale (g) above which the shaker drive becomes
/// impractical. This is `ω_a²·rms`; the *base* drive is ~this/Q (Q pinned by R1),
/// so it is a conservative upper bound on how hard the shaker must push.
const ACCEL_MAX_G: f64 = 25.0;
const G: f64 = 9.81;

fn main() {
    let beam = Beam {
        length_m: 45e-3,
        width_m: 6e-3,
        thickness_m: 0.08e-3,
        youngs_pa: 200e9,
        density_kgm3: 7850.0,
    };
    let tip = Magnet {
        remanence_t: 1.32,
        radius_m: 2.5e-3,
        thickness_m: 2e-3,
        density_kgm3: 7500.0,
    };
    let fixed = Magnet {
        remanence_t: 1.32,
        radius_m: 4e-3,
        thickness_m: 3e-3,
        density_kgm3: 7500.0,
    };
    let spread = 5e-3;

    let k_beam = beam.tip_stiffness_npm();
    let modal_g = (0.2427 * beam.mass_kg() + tip.mass_kg()) * 1e3;

    println!("pbit-design — magnetoelastic rig sweep (ΔV/kT_eff target = {TARGET_RATIO})");
    println!(
        "beam {:.0}×{:.0}×{:.2} mm spring steel | tip Ø{:.0}×{:.0} mm | fixed Ø{:.0}×{:.0} mm | spread ±{:.0} mm",
        beam.length_m * 1e3,
        beam.width_m * 1e3,
        beam.thickness_m * 1e3,
        tip.radius_m * 2e3,
        tip.thickness_m * 1e3,
        fixed.radius_m * 2e3,
        fixed.thickness_m * 1e3,
        spread * 1e3,
    );
    println!("k_beam = {k_beam:.2} N/m | modal mass = {modal_g:.3} g\n");
    println!(
        "{:>7} {:>9} {:>10} {:>7} {:>8} {:>9} {:>8} {:>9}",
        "gap_mm", "bistable", "ΔV_µJ", "x0_mm", "f_a_Hz", "rms_mm@3", "accel_g", "verdict"
    );

    let mut feasible_window: Option<(f64, f64)> = None;
    for step in 0..=30i32 {
        let gap_mm = 3.0 + 0.5 * f64::from(step);
        let rig = Rig {
            beam,
            tip,
            fixed,
            gap_axial_m: gap_mm * 1e-3,
            spread_lateral_m: spread,
            orientation_factor: 2.0,
        };
        let wm = rig.analyze();
        let fa = wm.attempt_freq_hz;
        if wm.bistable {
            let kt = wm.kt_for_ratio(TARGET_RATIO);
            let rms_m = wm.rms_amplitude_m(kt);
            let rms_mm = rms_m * 1e3;
            let accel_g = wm.omega_a * wm.omega_a * rms_m / G;
            let dv_uj = wm.delta_v_j * 1e6;
            let x0_mm = wm.x0_m * 1e3;
            let ok = (RMS_MIN_MM..RMS_MAX_MM).contains(&rms_mm) && accel_g <= ACCEL_MAX_G;
            let verdict = if ok {
                feasible_window = Some(match feasible_window {
                    Some((lo, _)) => (lo, gap_mm),
                    None => (gap_mm, gap_mm),
                });
                "OK"
            } else if accel_g > ACCEL_MAX_G {
                "drive too hard"
            } else {
                "amp too small"
            };
            println!(
                "{gap_mm:>7.1} {:>9} {dv_uj:>10.3} {x0_mm:>7.3} {fa:>8.1} {rms_mm:>9.4} {accel_g:>8.2} {verdict:>9}",
                "yes"
            );
        } else {
            println!(
                "{gap_mm:>7.1} {:>9} {:>10} {:>7} {fa:>8.1} {:>9} {:>8} {:>9}",
                "no", "-", "-", "-", "-", "monostable"
            );
        }
    }

    match feasible_window {
        Some((lo, hi)) => println!(
            "\nFeasible gap window (bistable, ΔV/kT=3 reachable with ≤{ACCEL_MAX_G:.0}g drive): {lo:.1}–{hi:.1} mm.\nDeeper wells (smaller gap) need impractical drive; operate near the bifurcation (wide end) where ΔV\nis shallow and the attempt frequency is shaker-friendly, and dial the magnet gap to tune ΔV/kT_eff."
        ),
        None => println!(
            "\nNo feasible gap for this magnet/beam combination — every bistable config needs an unphysical drive amplitude.\nTry weaker/smaller magnets, a stiffer beam, or operating nearer the bifurcation."
        ),
    }
}
