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
//! | raw Tet4 | 25, −4.473° … +6.132° | 2.5874 mm | 0.8484 | 6.9 s | 32.2 s |
//! | straight Tet10 | 25, −4.474° … +6.140° | 2.5908 mm | **0.8751** | 67.8 s | **583.1 s** |
//!
//! Two results worth naming, neither of them predicted:
//!
//! - **The quadratic disc deforms with *less* element distortion, not more** (0.8751 vs
//!   0.8484 worst ratio). The risk this gate was written against — that a higher-order element
//!   driven to ±ROM would degrade toward a fold — runs the other way on this geometry.
//! - **★ The plan's ~85 s Tet4 capture anchor was wrong: it is 32.2 s.** §5.1 derived its
//!   whole cost model from that figure (`85/160 ≈ 0.53 s` per solve) and predicted 14–45 min
//!   for Tet10. Measured: **9.7 min**, below the band's floor — the estimate was close only
//!   because the stale anchor overstated Tet4 by ~2.6× while the per-solve element ratio
//!   (18.1×, not the assumed ~10×) understated Tet10 by about as much. Two compensating
//!   errors are not a validated model: quote the measurements here, not §5.1's arithmetic.
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
const MAX_DISPLACEMENT_MM: (f64, f64) = (2.5874, 2.5908); // (raw Tet4, straight Tet10)
/// Pin width on [`MAX_DISPLACEMENT_MM`], as a fraction.
const DISPLACEMENT_PIN: f64 = 0.05;

/// Assert one captured trajectory is a sound full-ROM sweep against its committed
/// `expect_max_disp`, and return its (max node displacement in native mm,
/// worst per-frame `detJ/detJ_rest`).
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
fn assert_sound_ramp(traj: &CoupledTrajectory, label: &str, expect_max_disp: f64) -> (f64, f64) {
    assert_eq!(
        traj.frames.len(),
        RAMP_FRAMES,
        "{label}: every ramp moment must yield a captured frame"
    );

    // (3) Element validity at each frame's DEFORMED configuration.
    let worst = traj
        .frames
        .iter()
        .map(|f| f.min_jacobian_ratio)
        .fold(f64::INFINITY, f64::min);
    for f in &traj.frames {
        assert!(
            f.min_jacobian_ratio > 0.0,
            "{label}: an element folded at {:.3}° ({:+.2} N·m) — worst detJ/detJ_rest = {:.4}. \
             The rest-configuration quality floors cannot see this: they bound the mesh the \
             projector produces, not what driving it to ±ROM does.",
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
    assert!(
        (max_disp - expect_max_disp).abs() <= DISPLACEMENT_PIN * expect_max_disp,
        "{label}: max node displacement {max_disp:.4} mm is outside ±{:.0} % of the committed \
         {expect_max_disp:.4} mm",
        100.0 * DISPLACEMENT_PIN
    );
    for w in traj.frames.windows(2) {
        assert!(
            w[1].theta > w[0].theta,
            "{label}: a sorted moment ramp must give strictly increasing equilibrium angles \
             (extension → flexion), got {:.4}° then {:.4}°",
            w[0].theta.to_degrees(),
            w[1].theta.to_degrees()
        );
        assert_eq!(
            w[0].deformed_nodes_native.len(),
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
    let (disp4, det4) = assert_sound_ramp(&ramp4, "raw Tet4", MAX_DISPLACEMENT_MM.0);

    let t0 = std::time::Instant::now();
    let mut tet10 = CoupledFsu::build(&l4, &l5, &disc, &params).expect("quadratic arm");
    let build10 = t0.elapsed().as_secs_f64();
    let t0 = std::time::Instant::now();
    let ramp10 = tet10.capture_ramp(&moment_ramp()).expect("quadratic ramp");
    let cap10 = t0.elapsed().as_secs_f64();
    let (disp10, det10) = assert_sound_ramp(&ramp10, "straight Tet10", MAX_DISPLACEMENT_MM.1);

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
}
