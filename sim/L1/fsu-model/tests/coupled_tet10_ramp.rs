//! **The full-ROM ramp on the quadratic disc** — §4.5's rung-4 gate in
//! `docs/FSU_TET10_COUPLING_MIGRATION_PLAN.md`, and the only place anything drives the
//! flipped `CoupledFsu`'s disc FEM across its whole working range.
//!
//! Three things are checked, and the third is the one nothing in this arc had ever looked at:
//!
//! 1. **[`CoupledFsu::capture_ramp`] completes** over the shared [`moment_ramp`] — 25 solved
//!    equilibria walked as one continuous ±ROM chain in 0.1° sub-steps. `fsu_coupled_contact.rs`
//!    sweeps the same ramp but only calls `equilibrium`, which reads the analytic bushing and
//!    the facet SDF and never re-solves the disc; so this is the first gate that pays for the
//!    FEM at production angles.
//! 2. **Committed max node displacement**, so "the ramp ran" is backed by a number that moves
//!    if the deformation does.
//! 3. **Per-frame element validity on the DEFORMED configuration** (`detJ > 0` at every Gauss
//!    point of every element). Rung 3's ±6° sweep and the projector quality floors both gate
//!    the **rest** mesh; a rest-valid mesh can still fold under a drive, and Newton converging
//!    does not rule it out — it can converge onto a folded element. That is a different
//!    invariant and this is the first thing to assert it.
//!
//! ★ Rung 3's lesson is the reason this test exists in this shape: a geometric gate cannot see
//! conditioning, so the artifact has to be **driven**, not only measured. Whether a quadratic
//! disc walks 15° of chained 0.1° sub-steps is a question only running it answers — and the same
//! lesson is why the *conformed* disc is not here: driving it is exactly what revealed that it
//! inverts on a lofted geometry, which is why seating the coupled disc is rung 4b.
//!
//! **Committed measurement** (`CoupledParams::default`, `BodyParts3D` FMA13075/13076/16036,
//! both arms in one process so the cost ratio is the element and not two machines):
//!
//! | arm | frames | max node displacement | worst deformed `detJ/detJ_rest` | build | capture |
//! |---|---|---|---|---|---|
//! | raw Tet4 | 25, −4.474° … +6.147° | 2.0065 mm | 0.8737 | 6.4 s | 27.9 s |
//! | straight Tet10 | 25, −4.475° … +6.148° | 2.0070 mm | **0.8741** | 62.6 s | **557.2 s** |
//!
//! ⚠ RE-ANCHORED at β.4 — **every cell of both rows**, from one 656 s run. Rung 4's row read
//! Tet4 `−4.473°…+6.132°, 0.8484, 6.9 s, 32.2 s` and Tet10 `2.5908 mm, 0.8751, 67.8 s,
//! 583.1 s`. Updating only the displacement column would have left a table mixing two epochs
//! on a mesh with 29 % fewer nodes.
//!
//! Two results worth naming:
//!
//! - ⚠ **The pre-α.1 distortion gap has closed.** Rung 4 measured the quadratic arm distorting
//!   *less* (0.8751 vs 0.8484) and named it an unpredicted result. Post-α.1 the two arms are
//!   0.8741 vs **0.8737** — a 0.05 % gap, i.e. essentially identical. That earlier gap was a
//!   property of the phantom material, not of the element. The risk this gate was written
//!   against — a higher-order element degrading toward a fold at ±ROM — remains absent.
//! - **★ The plan's ~85 s Tet4 capture anchor was wrong: it is 27.9 s.** §5.1 derived its
//!   whole cost model from that figure (`85/160 ≈ 0.53 s` per solve) and predicted 14–45 min
//!   for Tet10. Measured: **9.3 min**, below the band's floor — the estimate was close only
//!   because the stale anchor overstated Tet4 by 3.05× while the per-solve element ratio
//!   (20.0×, not the assumed ~10×) understated Tet10 by 2.0×. The errors are compensating but NOT
//!   equal, and the residual shows: §5.1's model predicts ~850 s where 557 s is measured.
//!   Two partly-cancelling errors are not a validated model — quote the measurements here.
//!
//! ```text
//! CF_L4_STL=/path/FMA13075.stl CF_L5_STL=/path/FMA13076.stl CF_DISC_STL=/path/FMA16036.stl \
//!   cargo test -p cf-fsu-model --release --test coupled_tet10_ramp -- --ignored --nocapture
//! ```

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)] // tests may unwrap/expect/panic.

use cf_fsu_geometry::load_from_env;
use cf_fsu_model::{
    CoupledFsu, CoupledFsuTet4, CoupledParams, CoupledTrajectory, PHYSIOLOGIC_MOMENT, RAMP_FRAMES,
    moment_ramp,
};

/// Committed max node displacement over the full ramp (native mm), per arm — the plan's
/// "committed max node displacement", pinned to ±5 % of the measured value so that "the ramp
/// ran" is backed by a number that moves when the deformation does.
///
/// ⚠ RE-ANCHORED at β.4: α.1 removed phantom material, so the same ramp on the corrected
/// domain displaces less. **Both arms measured in one run** once the assert ordering below was
/// fixed — previously the Tet4 pin fired before the Tet10 arm was built, so only one number
/// per run was obtainable.
///
/// The Tet10 value was measured, not extrapolated from the old arm ratio — which would have
/// given 2.0091 against a measured 2.0070, inside this ±5 % pin and so undetectable here.
const MAX_DISPLACEMENT_MM: (f64, f64) = (2.0065, 2.0070); // (raw Tet4, straight Tet10)
/// Pin width on [`MAX_DISPLACEMENT_MM`], as a fraction.
const DISPLACEMENT_PIN: f64 = 0.05;

/// Assert one captured trajectory is a sound full-ROM sweep, and return its (max node
/// displacement in native mm, worst per-frame `detJ/detJ_rest`).
///
/// ★ **Soundness only — the committed-value pin lives at the call site.** Both are needed, but
/// they fail differently: a folded element makes every later number meaningless (fail fast,
/// here), whereas a displacement outside its band is a *comparison* whose answer is worth
/// having for BOTH arms before anything panics. Asserting the pin here meant the Tet4 arm
/// aborted the test before the Tet10 arm was ever built, so a re-anchor could only ever learn
/// one of the two numbers per run. The validity-first ordering *within* this function is
/// unchanged and deliberate — see the note below.
///
/// The validity check is asserted **first**, before the shape statistics — rung 3 measured why
/// the order is part of the diagnostic: with the validity check last, an inverted mesh got
/// reported as a displacement outside its band, which points at the wrong thing. Everything
/// downstream of a folded element is meaningless anyway.
///
/// The worst `detJ` ratio is *reported*, not pinned: `detJ > 0` is the invariant that means
/// something ("no element folded"), while the exact worst ratio is a property of one
/// specimen's mesh at one ROM. Pinning it would manufacture a tripwire that fires on
/// re-meshing without telling anyone anything about validity.
fn assert_sound_ramp(traj: &CoupledTrajectory, label: &str) -> (f64, f64) {
    assert_eq!(
        traj.frames.len(),
        RAMP_FRAMES,
        "{label}: every ramp moment must yield a captured frame"
    );

    // (3) Element validity at each frame's DEFORMED configuration.
    //
    // ⚠ Cross-check, not a discovery. `sim-soft`'s step-boundary gate CERTIFIES
    // `det F > 0` over each element's whole volume, so a frame that exists was already
    // proven fold-free and `capture_ramp` would have failed instead of returning a
    // negative ratio here. What this pins is that the four-point readout agrees with
    // that proof about the pose it read. The rest-configuration quality floors remain
    // unable to see either: they bound the mesh the projector produces, not what
    // driving it to ±ROM does.
    let worst = traj
        .frames
        .iter()
        .map(|f| f.min_jacobian_ratio)
        .fold(f64::INFINITY, f64::min);
    for f in &traj.frames {
        assert!(
            f.min_jacobian_ratio > 0.0,
            "{label}: the readout says an element folded at {:.3}° ({:+.2} N·m) — worst \
             detJ/detJ_rest = {:.4} — at a pose the solver's certified gate passed. The two \
             disagree, so one of them is not reading the solved state.",
            f.theta.to_degrees(),
            f.applied,
            f.min_jacobian_ratio
        );
    }

    // (2) Displacement, and the monotone angle chain the capture is built on.
    let max_disp = traj
        .frames
        .iter()
        .map(|f| {
            f.deformed_nodes_native
                .iter()
                .zip(&traj.rest_nodes_native)
                .map(|(d, r)| (d - r).norm())
                .fold(0.0_f64, f64::max)
        })
        .fold(0.0_f64, f64::max);
    for w in traj.frames.windows(2) {
        assert!(
            w[1].theta > w[0].theta,
            "{label}: a sorted moment ramp must give strictly increasing equilibrium angles \
             (extension → flexion), got {:.4}° then {:.4}°",
            w[0].theta.to_degrees(),
            w[1].theta.to_degrees()
        );
    }
    // ⚠ The validity readout must be wired to the DEFORMED configuration, and `> 0` cannot
    // tell you that it is: a readout accidentally reporting the REST mesh would be exactly
    // 1.0 at every frame and sail through. At rest the ratio IS 1.0 by construction (the L0
    // unit test pins that), so a worst below 1 across the sweep is the cheapest available
    // proof that these numbers describe a deformed disc at all. Mutation-checked: rewiring
    // `capture_ramp` to record the rest ratio takes `worst` to exactly 1.0000 and fires here,
    // while every other assert in this gate still passes.
    assert!(
        worst < 1.0,
        "{label}: worst detJ/detJ_rest is {worst:.4} — at or above 1.0 means the readout is \
         not seeing deformation, so `> 0` was never testing the deformed configuration"
    );
    // Every frame, including the last — this used to live inside a `windows(2)` loop reading
    // only `w[0]`, so the flexion peak (the most deformed frame) was never checked despite the
    // message saying "every frame".
    for f in &traj.frames {
        assert_eq!(
            f.deformed_nodes_native.len(),
            traj.rest_nodes_native.len(),
            "{label}: every frame carries the full node buffer"
        );
    }
    println!(
        "[{label}] {} frames, {:.3}° … {:+.3}°; max node displacement {max_disp:.4} mm; \
         worst deformed detJ/detJ_rest {worst:.4}",
        traj.frames.len(),
        traj.frames.first().unwrap().theta.to_degrees(),
        traj.frames.last().unwrap().theta.to_degrees(),
    );
    (max_disp, worst)
}

#[test]
#[ignore = "needs $CF_L4_STL $CF_L5_STL $CF_DISC_STL (BodyParts3D, CC BY-SA, not committed)"]
fn tet10_coupled_ramp_completes_over_the_full_rom() {
    let l4 = load_from_env("CF_L4_STL").expect("load L4 mesh");
    let l5 = load_from_env("CF_L5_STL").expect("load L5 mesh");
    let disc = load_from_env("CF_DISC_STL").expect("load disc mesh");
    let params = CoupledParams::default();

    println!(
        "\n=== rung 4 §4.5 — full ±{PHYSIOLOGIC_MOMENT} N·m ramp on the coupled FSU, both elements ===\n"
    );

    // The linear baseline first: it is cheap, and running it in the same process on the same
    // meshes is what makes the cost ratio below a measurement of the ELEMENT rather than of
    // two machines or two mesh loads.
    let t0 = std::time::Instant::now();
    let mut tet4 = CoupledFsuTet4::build_baseline(&l4, &l5, &disc, &params).expect("baseline arm");
    let build4 = t0.elapsed().as_secs_f64();
    let t0 = std::time::Instant::now();
    let ramp4 = tet4.capture_ramp(&moment_ramp()).expect("Tet4 ramp");
    let cap4 = t0.elapsed().as_secs_f64();
    let (disp4, det4) = assert_sound_ramp(&ramp4, "raw Tet4");

    let t0 = std::time::Instant::now();
    let mut tet10 = CoupledFsu::build(&l4, &l5, &disc, &params).expect("quadratic arm");
    let build10 = t0.elapsed().as_secs_f64();
    let t0 = std::time::Instant::now();
    let ramp10 = tet10.capture_ramp(&moment_ramp()).expect("quadratic ramp");
    let cap10 = t0.elapsed().as_secs_f64();
    let (disp10, det10) = assert_sound_ramp(&ramp10, "straight Tet10");

    println!(
        "\n[cost] build   Tet4 {build4:.1} s → Tet10 {build10:.1} s  ({:.1}×)\n\
         [cost] capture Tet4 {cap4:.1} s → Tet10 {cap10:.1} s  ({:.1}×)",
        build10 / build4,
        cap10 / cap4
    );
    println!(
        "[shape] max node displacement: Tet4 {disp4:.4} mm, Tet10 {disp10:.4} mm; \
         worst deformed detJ ratio: Tet4 {det4:.4}, Tet10 {det10:.4}"
    );

    // The committed pins, asserted last so the print above always lands: a re-anchor needs
    // BOTH arms' numbers, and asserting per-arm inside the capture hid whichever came second.
    for (disp, expect, label) in [
        (disp4, MAX_DISPLACEMENT_MM.0, "raw Tet4"),
        (disp10, MAX_DISPLACEMENT_MM.1, "straight Tet10"),
    ] {
        assert!(
            (disp - expect).abs() <= DISPLACEMENT_PIN * expect,
            "{label}: max node displacement {disp:.4} mm is outside ±{:.0} % of the committed \
             {expect:.4} mm",
            100.0 * DISPLACEMENT_PIN
        );
    }
}
