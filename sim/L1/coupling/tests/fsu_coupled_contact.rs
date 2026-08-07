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
//!   6.147° ROM — proving coupled ≡ analytic superposition when contact is inactive;
//! - **extension** (facets engage): disc + ligaments alone are too lax to reach the
//!   physiologic moment, but the oriented facet contact caps the extension ROM inside
//!   the literature band — the bones stop on the facets, coupled into one solve.
//!
//! rung 7 kept the facet term out of its headline because the *raw* grid contact
//! normal's sign was unvalidated. Orienting each repulsive force along L5's outward
//! SDF gradient makes the facet moment restoring by construction — the fix
//! `CoupledFsu` carries, asserted below.
//!
//! ⚠ **What the flexion agreement means changed at rung 4, and it is weaker than it reads.**
//! `CoupledFsu` now assembles on a quadratic disc while `rung7_fsu_validation.rs`
//! still superposes a linear one, so the two no longer share a disc — yet the coupled
//! flexion ROM still lands on rung 7's 6.147°. That is not a tighter proof of "coupled ≡
//! superposition"; it is the same
//! evidence as before *plus* a demonstration that the disc contributes ~0.17 % of the flexion
//! restoring moment at ROM, so even a large change in it is nearly invisible here. The
//! disc-level claim is anchored by `k_disc` below, which moved far beyond its tolerance at
//! both re-anchors — 4.69 × at rung 4, 4.55 × at β.4 measured the same way.
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
use cf_fsu_model::{CoupledFsu, CoupledParams, PHYSIOLOGIC_MOMENT, moment_ramp};

// ── Physiologic probe + rung-7 oracle (facts / prior result, not tunable). ──
// PHYSIOLOGIC_MOMENT (7.5 N·m) is shared with the viewer via cf_fsu_model::moment_ramp.
/// Rung 7's headline flexion ROM at 7.5 N·m — the oracle this coupled solve is checked against.
///
/// ⚠ RE-ANCHORED at β.4 from 6.13: `rung7_fsu_validation` now measures **6.147°** on the
/// post-α.1 disc. Still an independent oracle (a different gate, superposing rather than
/// solving), and the agreement tightened from 0.018° to ~0.001°.
const RUNG7_FLEXION_ROM_DEG: f64 = 6.147;
const ROM_TOL_DEG: f64 = 0.15; // agreement window vs rung 7's grid-interpolated ROM

/// The disc's small-strain bending stiffness (N·m/rad) the coupled bushing is built from.
///
/// ★★ **A reproduction pin, not a model input.** `CoupledFsu::build` *measures* it —
/// `sim/L1/fsu-model/src/coupled.rs`'s `k_disc = m / K_DISC_PROBE` — from the FEM disc on every
/// build, and nothing reads the constant below except this gate. **Restating it cannot change
/// any simulation result.** So the physics question it might look like it settles — *is the
/// disc's bending stiffness right?* — lives upstream in the FEM disc's geometry and material,
/// against literature, and is not answered here.
///
/// `rung7_fsu_validation` passing does not constrain it either: the disc carries ~0.17 % of
/// the restoring moment at ROM, so that gate would pass at either value.
///
/// **Re-anchored twice.** Rung 4, −0.2819 → −0.1882: `CoupledFsu::build` began assembling on
/// the quadratic disc, which does not bend-lock the way a linear element does. Rung β.4,
/// −0.1882 → −0.0972: ⚠ **that rationale does NOT transfer.** Rung 4 was an *element* change at
/// fixed geometry; this is the reverse — α.1 removed phantom material, so the old value was
/// measured on a mesh carrying slivers the anatomy does not have. The element ratio did *not*
/// hold across it (0.668 → 0.827); what localises the change to geometry is that **both arms
/// fell together** with no element code touched, and that the coupled ratio still lands on the
/// standalone gate's 0.827/0.829 through a different probe.
///
/// **Measured** (`cf-fsu-model`'s `coupled_element_shift`, both arms in one run, probe 0.86°):
///
/// | arm | rung 4 (pre-α.1) | rung β.4 (post-α.1) |
/// |---|---|---|
/// | raw Tet4 (`CoupledFsuTet4::build_baseline`) | −0.2819 | **−0.1175** |
/// | straight Tet10 (shipped, `CoupledFsu::build`) | −0.1882 | **−0.0972** |
/// | ratio | 0.668 | **0.827** |
///
/// ⚠ Rung 4's supporting claim — *"the linear arm reproduces −0.2819 to four decimals, so the
/// re-anchor is attributable to the element"* — is falsified: that arm now reads −0.1175. Its
/// ROM figures (6.1321°/6.1403°, 4.4731°/4.4739°) are a record of that measurement; β.4's are
/// in `coupled_element_shift`.
///
/// §0.3's model, re-derived on the new numbers, predicts ΔROM ≈ 0.0018° and the gate measures
/// +0.0018° — which is why [`ROM_TOL_DEG`] and [`LIT_EXTENSION_DEG`] did not move across
/// either re-anchor. **Never widen the tolerance to absorb an element _or geometry_ change.**
const RUNG7_K_DISC: f64 = -0.0972;
/// Gross-regression bound around [`RUNG7_K_DISC`], as a **fraction of it**.
///
/// ⚠⚠ **This was an absolute 0.02 until β.4, and holding it fixed while the anchor halved was
/// itself a widening** — the relative admission window went 10.6 % → 20.6 % with nobody
/// editing a line. Worse, the failure this gate exists to catch is the Tet10→Tet4 element flip
/// being reverted, and that arm now measures −0.1175: the margin to it collapsed from
/// **4.68 × the tolerance to 1.01 ×**, so a ~2 % drift — far inside the ~21 % mesh-realization
/// spread this arc measured — would have let a reverted flip pass.
///
/// Dimensionless so it cannot rot that way again. At 5 % the window is 0.0049 and the margin
/// to the Tet4 arm is **4.2 ×**, restoring rung 4's discriminating power. This is a TIGHTENING
/// (0.02 → 0.0049); it can only catch more.
const K_DISC_TOL_FRAC: f64 = 0.05;
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
        CoupledFsu::build(&l4, &l5, &disc, &CoupledParams::default()).expect("build coupled FSU");

    // ── Disc bushing: k_disc measured on the bonded FEM disc, which since rung 4 is the
    //    QUADRATIC disc (straight — seating it is rung 4b) — so it reproduces the re-anchored
    //    value, not rung 7's linear one (see RUNG7_K_DISC). Flexion sense still derived, not
    //    hardcoded. The bound is a gross-regression guard. ──
    let k_disc = fsu.k_disc();
    println!("[disc]  k_disc = {k_disc:+.4} N·m/rad  (anchor {RUNG7_K_DISC:+.4})");
    assert!(
        k_disc < 0.0,
        "disc bending must be restoring, got {k_disc:+.4}"
    );
    assert!(
        (k_disc - RUNG7_K_DISC).abs() < K_DISC_TOL_FRAC * RUNG7_K_DISC.abs(),
        "k_disc must reproduce the β.4 anchor {RUNG7_K_DISC:+.4} within {:.1} % ({:.4}), got \
         {k_disc:+.4}. If the element OR the geometry changed again this is a re-anchor (measure \
         it and restate \
         the constant with the measurement); never widen the tolerance to absorb it.",
        100.0 * K_DISC_TOL_FRAC,
        K_DISC_TOL_FRAC * RUNG7_K_DISC.abs()
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

    // ── FLEXION: the coupled equilibrium at 7.5 N·m reproduces rung 7's 6.147° ROM. ──
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

    // ── FULL RAMP: the viewer captures the whole moment sweep (extension → flexion), not
    //    just the ±7.5 endpoints, and its launch depends on EVERY intermediate equilibrium
    //    resolving. Sweep the SAME shared `moment_ramp()` the viewer captures, so a mid-ramp
    //    facet-engagement gap (which would abort the viewer at startup) is caught here, not
    //    in the GUI — and the two can never drift onto different grids. ──
    for applied in moment_ramp() {
        assert!(
            fsu.equilibrium(applied).is_some(),
            "every ramp moment must have an equilibrium — {applied:+.2} N·m returned None (would abort the viewer at startup)"
        );
    }

    // ── CAVEAT (honest account of the facet extension result; investigated 2026-07-08):
    //    like rung 7, the coupled model shares ONE pivot (the disc centre) and ONE axis
    //    (the disc's principal ML) across disc + ligaments + facets. That is right for the
    //    disc (which bends about its own centre) but is a SIMPLIFICATION for the facets,
    //    whose physiological extension axis-of-rotation is the vertebral-ML through a
    //    POSTERIOR IAR (~the posterior annulus, ~12–16 mm behind the disc centre here). A
    //    pivot/axis sweep showed the facet ENGAGEMENT MECHANICS (which facet, how many
    //    contacts) are pivot-dominated — but the extension ROM is ROBUST: the shipped
    //    disc-ML/disc-centre gives ≈4.5°, and the physiological vertebral-ML/posterior-IAR
    //    gives ≈3.6–4.8°, both in the literature band and near Yamamoto's 3.5°. Separately,
    //    the engagement is ONE-SIDED at EVERY axis/pivot (this specimen's right facet does
    //    not engage in extension) — real anatomical asymmetry, not a modelling artifact. A
    //    decoupled-facet model (vertebral-ML + posterior IAR) is a realism refinement for
    //    the engagement mechanics, not a correction to the (already in-band) ROM. ──

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
        // Same 1e-2° jitter slack as the monotonicity check: a converged sweep whose last
        // step is already tiny must not flake if sub-grid noise nudges it past the first step.
        (roms[roms.len() - 2] - roms[roms.len() - 1]).abs() < (roms[0] - roms[1]).abs() + 1e-2,
        "the ROM must be flattening (later steps no larger than the first) — approaching a limit"
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
        "\n=== coupled FSU validated — flexion reproduces rung 7 ({RUNG7_FLEXION_ROM_DEG}°); oriented facet contact \
         is restoring and caps extension to {ext_deg:.2}° (bones stop) ===\n"
    );
}
