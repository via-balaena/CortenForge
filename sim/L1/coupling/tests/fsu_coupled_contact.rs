//! Option (c) — the coupled L4–L5 FSU as ONE quasi-static equilibrium.
//!
//! Rung 7 (`rung7_fsu_validation.rs`) characterised the assembled segment by
//! *analytic superposition*: it imposed a flexion angle, read each structure's
//! restoring moment separately, and summed them. This test validates the coupled
//! alternative — [`cf_fsu_model::CoupledFsu`] assembles the disc (as a linearised
//! bushing), the ligaments (pull-only tendons), and the facets (an SDF penalty,
//! oriented along the fixed vertebra's outward gradient so it genuinely separates
//! the bones) into a single model, and **solves for the equilibrium pose** under an
//! applied moment. The coupling is validated against rung 7 as the oracle:
//!
//! - **flexion** (facets open): the coupled equilibrium at 7.5 N·m reproduces rung 7's
//!   6.13° ROM — proving coupled ≡ analytic superposition when contact is inactive;
//! - **extension** (facets engage): disc + ligaments alone are too lax to reach the
//!   physiologic moment, but the oriented facet contact caps the extension ROM inside
//!   the literature band — the bones stop on the facets, coupled into one solve.
//!
//! rung 7 kept the facet term out of its headline because the *raw* grid contact
//! normal's sign was unvalidated. Orienting each repulsive force along L5's outward
//! SDF gradient makes the facet moment restoring by construction — the fix
//! `CoupledFsu` carries, asserted below.
//!
//! Env-gated + license-clean like the other rungs: `#[ignore]` + `$CF_L4_STL` +
//! `$CF_L5_STL` + `$CF_DISC_STL` (`BodyParts3D` meshes are CC BY-SA, not committed).
//! Run with:
//!
//! ```text
//! CF_L4_STL=/path/FMA13075.stl CF_L5_STL=/path/FMA13076.stl CF_DISC_STL=/path/FMA16036.stl \
//!   cargo test -p sim-coupling --release \
//!   --test fsu_coupled_contact -- --ignored --nocapture
//! ```

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use cf_fsu_geometry::load_from_env;
use cf_fsu_model::{CoupledFsu, CoupledParams};

// ── Physiologic probe + rung-7 oracle (facts / prior result, not tunable). ──
const PHYSIOLOGIC_MOMENT: f64 = 7.5; // N·m — the applied flexion/extension moment
const RUNG7_FLEXION_ROM_DEG: f64 = 6.13; // rung 7's headline flexion ROM at 7.5 N·m
const ROM_TOL_DEG: f64 = 0.15; // agreement window vs rung 7's grid-interpolated ROM
const RUNG7_K_DISC: f64 = -0.2819; // rung 7's measured disc bending stiffness (N·m/rad)
// Literature extension corridor (Yamamoto 1989 / Panjabi–White, widened for 7.5–10 N·m).
const LIT_EXTENSION_DEG: (f64, f64) = (2.5, 5.5);

#[test]
#[ignore = "needs $CF_L4_STL $CF_L5_STL $CF_DISC_STL (BodyParts3D, CC BY-SA, not committed)"]
fn l4_l5_coupled_flexion_extension_equilibrium() {
    println!("\n=== option (c) — coupled L4–L5 FSU equilibrium vs rung 7 ===\n");

    let l4 = load_from_env("CF_L4_STL").expect("load L4 mesh");
    let l5 = load_from_env("CF_L5_STL").expect("load L5 mesh");
    let disc = load_from_env("CF_DISC_STL").expect("load disc mesh");
    let fsu =
        CoupledFsu::build(&l4, &l5, disc, &CoupledParams::default()).expect("build coupled FSU");

    // ── Disc bushing reproduces rung 7's k_disc, flexion sense derived (not hardcoded). ──
    let k_disc = fsu.k_disc();
    println!("[disc]  k_disc = {k_disc:+.4} N·m/rad  (rung 7 {RUNG7_K_DISC:+.4})");
    assert!(
        k_disc < 0.0,
        "disc bending must be restoring, got {k_disc:+.4}"
    );
    assert!(
        (k_disc - RUNG7_K_DISC).abs() < 0.02,
        "k_disc must reproduce rung 7's {RUNG7_K_DISC:+.4}, got {k_disc:+.4}"
    );

    // Neutral pose is force-free (ligaments at slack, disc spring at reference).
    assert!(
        fsu.restoring_moment(0.0).abs() < 1e-6,
        "neutral coupled moment must vanish: {:.2e}",
        fsu.restoring_moment(0.0)
    );

    // ── Facet engagement asymmetry (rung 7's rung-4b result): facets open in flexion
    //    (+θ, zero contacts), engage in extension (−θ). And, the c2 fix: wherever the
    //    oriented penalty engages it is RESTORING (a +moment in the flexion-positive
    //    frame opposes the extension). ──
    let probe = 6.0_f64.to_radians();
    assert_eq!(
        fsu.facet_moment(probe).0,
        0,
        "facets must stay clear on the flexion side"
    );
    assert!(
        fsu.facet_moment(-probe).0 > 0,
        "facets must engage on the extension side (rung-4b articular geometry)"
    );
    println!("\n{:>9} {:>8} {:>14}", "θ_ext(°)", "nFacet", "M_facet(N·m)");
    for deg in [3.0_f64, 4.0, 5.0, 6.0, 7.0, 8.0] {
        let (n, m) = fsu.facet_moment(-deg.to_radians());
        println!("{deg:>9.1} {n:>8} {m:>+14.4}");
        if n > 0 {
            assert!(
                m > 0.0,
                "the oriented facet penalty must be restoring in extension (θ=−{deg}°, M={m:+.4})"
            );
        }
    }

    // ── FLEXION: the coupled equilibrium at 7.5 N·m reproduces rung 7's 6.13° ROM. ──
    let flex_deg = fsu
        .equilibrium(PHYSIOLOGIC_MOMENT)
        .expect("flexion equilibrium must exist within the ROM bracket")
        .to_degrees();
    println!(
        "\n[flexion]   coupled equilibrium at {PHYSIOLOGIC_MOMENT} N·m = {flex_deg:.3}°  (rung 7 {RUNG7_FLEXION_ROM_DEG}°)"
    );
    assert!(
        (flex_deg - RUNG7_FLEXION_ROM_DEG).abs() < ROM_TOL_DEG,
        "coupled flexion equilibrium {flex_deg:.3}° must reproduce rung 7's {RUNG7_FLEXION_ROM_DEG}° \
         (within {ROM_TOL_DEG}°) — coupled ≡ analytic superposition when contact is inactive"
    );
    // The facets do NOT engage at the flexion equilibrium (ligament/disc-limited).
    assert_eq!(
        fsu.facet_moment(flex_deg.to_radians()).0,
        0,
        "flexion equilibrium must be contact-free"
    );

    // ── EXTENSION: the facets cap the ROM (bones stop). Disc + ligaments alone never
    //    reach the physiologic moment (the ligaments are lax — rung 7's note that the
    //    physiological extension limiter is facet contact), so the equilibrium is set by
    //    the facet contact, which is engaged there. ──
    let ext = fsu
        .equilibrium(-PHYSIOLOGIC_MOMENT)
        .expect("extension equilibrium must exist within the ROM bracket");
    let ext_deg = ext.to_degrees().abs();
    let (n_ext, _) = fsu.facet_moment(ext);
    println!(
        "[extension] coupled equilibrium at {PHYSIOLOGIC_MOMENT} N·m = {ext_deg:.3}° with {n_ext} facet contacts \
         (lit band [{:.1},{:.1}]°) → {}",
        LIT_EXTENSION_DEG.0,
        LIT_EXTENSION_DEG.1,
        if ext_deg >= LIT_EXTENSION_DEG.0 && ext_deg <= LIT_EXTENSION_DEG.1 {
            "within band — facets are the extension limiter"
        } else {
            "outside band (K_facet uncalibrated — magnitude is sensitivity-only)"
        }
    );
    assert!(
        ext < 0.0,
        "extension equilibrium must be a negative (extension) angle"
    );
    assert!(
        n_ext > 0,
        "facet contact must be engaged at the extension equilibrium — it is the limiter"
    );
    // The spring restoring alone is far below the physiologic moment at this angle, so the
    // facet contact is genuinely what caps the ROM (not the ligaments).
    assert!(
        fsu.restoring_moment(ext).abs() < PHYSIOLOGIC_MOMENT,
        "ligaments+disc alone must be too lax to reach {PHYSIOLOGIC_MOMENT} N·m — facets are the limiter"
    );

    // ── K_facet convergence: the facet penalty is a NUMERICAL regularisation of a
    //    near-rigid bone contact, not a tissue property. As K_facet → ∞ the contact
    //    hardens and the extension ROM converges to the GEOMETRIC contact limit (the
    //    angle where the articular surfaces just touch). The oriented facet moment is
    //    exactly linear in K_facet, so we sweep it by scaling the base facet moment —
    //    no rebuild — and root-find the extension equilibrium at each stiffness. ──
    println!("\n── K_facet convergence (extension ROM → geometric rigid-contact limit) ──");
    let base_k = CoupledParams::default().k_facet;
    // Reuse the crate's own bracket-checked solver via the facet-scale sweep API (the
    // oriented facet moment is exactly linear in k_facet), rather than re-implementing the
    // bisection here — one validated root-finder, no drift.
    let ext_rom_at = |k_facet: f64| -> f64 {
        fsu.equilibrium_with_facet_scale(-PHYSIOLOGIC_MOMENT, k_facet / base_k)
            .expect("extension equilibrium must exist for a stiffer facet")
            .to_degrees()
            .abs()
    };
    let ks = [200.0, 500.0, 1000.0, 2000.0, 4000.0, 8000.0];
    let roms: Vec<f64> = ks.iter().map(|&k| ext_rom_at(k)).collect();
    let mut prev = f64::NAN;
    for (&k, &rom) in ks.iter().zip(&roms) {
        let delta = if prev.is_nan() {
            String::new()
        } else {
            format!("  (Δ {:+.3}°)", rom - prev)
        };
        println!("[K_facet={k:>6.0} N/mm] extension ROM = {rom:.3}°{delta}");
        prev = rom;
    }
    println!(
        "[note] Yamamoto 1989 extension ≈ 3.5° at 10 N·m; the rigid-contact limit (large K) \
         is the geometric onset, not a tuned penalty."
    );
    // The penalty is a numerical regularisation, so hardening it must CONVERGE the ROM
    // downward toward the geometric contact onset (a monotone-decreasing sequence that
    // flattens), and the whole converging range must stay physiological (lit band). This
    // is the honest account of the uncalibrated K_facet — not a tuned point-match.
    for w in roms.windows(2) {
        // Slack (1e-2°) sits well above the ~1e-6° root jitter from the discretised facet
        // moment but far below the ~0.1–0.5° real ROM steps, so this catches a genuine
        // reversal without flaking on numerical noise near convergence.
        assert!(
            w[1] <= w[0] + 1e-2,
            "extension ROM must converge DOWNWARD as K_facet stiffens (got {:.3}° → {:.3}°)",
            w[0],
            w[1]
        );
    }
    assert!(
        (roms[roms.len() - 2] - roms[roms.len() - 1]).abs() < (roms[0] - roms[1]).abs(),
        "the ROM must be flattening (later steps smaller than the first) — approaching a limit"
    );
    for (&k, &rom) in ks.iter().zip(&roms) {
        assert!(
            rom >= LIT_EXTENSION_DEG.0 && rom <= LIT_EXTENSION_DEG.1,
            "extension ROM at K_facet={k} must stay in the lit band [{:.1},{:.1}]°, got {rom:.3}°",
            LIT_EXTENSION_DEG.0,
            LIT_EXTENSION_DEG.1
        );
    }

    println!(
        "\n=== coupled FSU validated — flexion reproduces rung 7 (6.13°); oriented facet contact \
         is restoring and caps extension to {ext_deg:.2}° (bones stop) ===\n"
    );
}
