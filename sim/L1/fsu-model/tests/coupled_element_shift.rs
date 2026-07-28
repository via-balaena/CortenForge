//! **What flipping `CoupledFsu` onto the quadratic disc does to the assembled
//! segment** — rung 4 of `docs/FSU_TET10_COUPLING_MIGRATION_PLAN.md`, the gate on §0.3's
//! central prediction.
//!
//! The plan predicts the flip is nearly invisible at the segment level even though it is
//! large at the disc level: the standalone disc softens ~33 % with the element order, but the
//! disc contributes only ~0.4 % of the flexion restoring moment at ROM (the segment is
//! ligament-`k`-dominated), so segment ROM should move by **~0.008°**. That number was
//! *derived*, never measured — nothing in this repo has ever assembled both arms.
//!
//! This test does, in one run: [`CoupledFsu::build`] (the shipped straight-Tet10 arm) and
//! [`CoupledFsuTet4::build_baseline`] (the pre-rung-4 raw-Tet4 arm), same three meshes, same
//! prologue, same ligaments and facet grids. Two arms are what make the shift a
//! **measurement** instead of an inference from a band that used to pass.
//!
//! **Committed measurement** (`CoupledParams::default`, `BodyParts3D` FMA13075/13076/16036,
//! both arms in one run; the ROM anchors themselves are enforced in
//! `sim-coupling`'s `fsu_coupled_contact.rs`, this table is the two-arm comparison):
//!
//! | arm | `k_disc` (N·m/rad) | flexion ROM | extension ROM | build |
//! |---|---|---|---|---|
//! | raw Tet4 (pre-rung-4) | −0.2819 | 6.1321° | 4.4731° | ~7 s |
//! | straight Tet10 (shipped) | −0.1882 | 6.1403° | 4.4739° | ~68 s |
//! | **Δ** | **0.668×** | **+0.0082°** | **+0.0008°** | ~10× |
//!
//! The `k_disc` and ROM columns are deterministic and pinned below; the build column is
//! wall clock on one machine and is quoted to one significant figure for that reason.
//!
//! So: the disc got **33 % softer** and the segment moved **0.0082°** — §0.3's derivation
//! ("~0.008°") confirmed to the digit, and the reason `ROM_TOL_DEG` (±0.15°) and
//! `LIT_EXTENSION_DEG` never had to move. The facet-capped extension side moved 10× less than
//! the ligament-limited flexion side, which is the mechanism the reframe claimed.
//!
//! It is deliberately *cheap*: the equilibrium solve reads the analytic bushing + the facet
//! SDF and never re-solves the disc FEM, so this costs two builds and no ramp. The expensive
//! full-ROM FEM sweep is `coupled_tet10_ramp.rs`.
//!
//! Lives in `tests/` rather than the library test module for the reason `sim-coupling` keeps
//! all of its gates there: an `#[ignore]`d, licence-gated figure of merit can never execute
//! under `cargo llvm-cov --lib`, so hosting it in the library target charges its whole body to
//! the crate's coverage grade while testing nothing there.
//!
//! ```text
//! CF_L4_STL=/path/FMA13075.stl CF_L5_STL=/path/FMA13076.stl CF_DISC_STL=/path/FMA16036.stl \
//!   cargo test -p cf-fsu-model --release --test coupled_element_shift -- --ignored --nocapture
//! ```

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)] // tests may unwrap/expect/panic.

use cf_fsu_geometry::load_from_env;
use cf_fsu_model::{CoupledFsu, CoupledFsuTet4, CoupledParams, PHYSIOLOGIC_MOMENT};

/// The flexion ROM shift the flip is allowed to introduce (degrees), **pre-registered before
/// the first run** rather than fitted to it.
///
/// Sits deliberately between the two numbers that matter: an order of magnitude above §0.3's
/// predicted ~0.008°, and a third of `fsu_coupled_contact.rs`'s `ROM_TOL_DEG` (0.15°) — the
/// literature tripwire the plan insists must not move. So a shift that lands here is
/// "prediction confirmed"; a shift that clears it is a finding *before* it becomes a
/// tripwire failure, which is the only way this gate is worth more than that one.
///
/// **Measured: +0.0082°**, against a prediction of ~0.008° derived before any of this was
/// built. Left at the pre-registered 0.05° rather than tightened onto the measurement: the
/// question this constant asks is "did the segment-level invisibility argument hold", and a
/// ±5 % pin around 0.0082° would instead ask "is this specimen's shift reproducible", which
/// is what the `k_disc` and ROM anchors below already ask, more directly.
const PREREGISTERED_ROM_SHIFT_DEG: f64 = 0.05;

/// The pre-rung-4 linear arm's disc bushing (N·m/rad) — **the known value this whole gate is
/// validated against**, not a fresh measurement.
///
/// `-0.2819` is the number `fsu_coupled_contact.rs` asserted before rung 4 and rung 7
/// published. Rung 4 changed how the probe is driven (walked to 0.86° in 0.1° sub-steps
/// instead of jumped there, because a conformed disc does not survive the jump) and moved the
/// whole assembly through a new generic `assemble`. If the linear arm still lands here, both
/// changes are element-neutral and the Tet10 number below is attributable to the element.
/// **Measured after: −0.2819** — four decimals, unchanged.
const BASELINE_K_DISC: f64 = -0.2819;
/// The shipped quadratic arm's disc bushing (N·m/rad). **Measured: −0.1882** — 0.668 of the
/// linear arm, which lands on the standalone straight-Tet10 ratio rung 1 committed (0.666 flex /
/// 0.663 ext). That agreement is the cross-check: the flip carried the *standalone* element
/// effect into the assembly rather than introducing an assembly-level artifact of its own.
const QUADRATIC_K_DISC: f64 = -0.1882;
/// Agreement window on both arms' `k_disc` (N·m/rad). Tight — these are re-runs of a
/// deterministic build on a pinned mesh, not a tolerance on a physical claim.
const K_DISC_TOL: f64 = 5.0e-4;

#[test]
#[ignore = "needs $CF_L4_STL $CF_L5_STL $CF_DISC_STL (BodyParts3D, CC BY-SA, not committed)"]
fn coupled_segment_shift_from_the_tet10_flip() {
    let l4 = load_from_env("CF_L4_STL").expect("load L4 mesh");
    let l5 = load_from_env("CF_L5_STL").expect("load L5 mesh");
    let disc = load_from_env("CF_DISC_STL").expect("load disc mesh");
    let params = CoupledParams::default();

    println!("\n=== rung 4 — segment-level effect of the Tet10 element flip ===\n");

    let t0 = std::time::Instant::now();
    let tet4 = CoupledFsuTet4::build_baseline(&l4, &l5, &disc, &params).expect("baseline arm");
    let t_tet4 = t0.elapsed().as_secs_f64();
    let t0 = std::time::Instant::now();
    let tet10 = CoupledFsu::build(&l4, &l5, &disc, &params).expect("quadratic arm");
    let t_tet10 = t0.elapsed().as_secs_f64();
    println!("[build] raw Tet4 {t_tet4:.1} s   straight Tet10 {t_tet10:.1} s");

    // ── k_disc: the disc bushing each arm measures off its own render disc. ──
    let (k4, k10) = (tet4.k_disc(), tet10.k_disc());
    println!(
        "[k_disc] raw Tet4 {k4:+.4}   straight Tet10 {k10:+.4} N·m/rad   ratio {:.3}",
        k10 / k4
    );
    assert!(
        k4 < 0.0 && k10 < 0.0,
        "both arms' disc bending must be restoring: Tet4 {k4:+.4}, Tet10 {k10:+.4}"
    );
    // Validate against the KNOWN value first: if the linear arm has drifted, nothing measured
    // on the quadratic one is attributable to the element.
    assert!(
        (k4 - BASELINE_K_DISC).abs() < K_DISC_TOL,
        "the pre-rung-4 linear arm must still reproduce the published {BASELINE_K_DISC:+.4} \
         within {K_DISC_TOL} — got {k4:+.4}. Rung 4 walks the probe to 0.86° instead of jumping \
         there and routes the build through a generic `assemble`; a drift here means one of \
         those was NOT element-neutral, and the Tet10 anchor below is not attributable."
    );
    assert!(
        (k10 - QUADRATIC_K_DISC).abs() < K_DISC_TOL,
        "the quadratic arm must reproduce {QUADRATIC_K_DISC:+.4} within {K_DISC_TOL}, got \
         {k10:+.4} — this is the value `fsu_coupled_contact.rs`'s RUNG7_K_DISC is anchored to"
    );

    // ── ROM at the physiologic moment, both directions, both arms. ──
    let rom = |fsu_flex: f64, fsu_ext: f64| (fsu_flex.to_degrees(), fsu_ext.to_degrees().abs());
    let (flex4, ext4) = rom(
        tet4.equilibrium(PHYSIOLOGIC_MOMENT)
            .expect("Tet4 flexion equilibrium"),
        tet4.equilibrium(-PHYSIOLOGIC_MOMENT)
            .expect("Tet4 extension equilibrium"),
    );
    let (flex10, ext10) = rom(
        tet10
            .equilibrium(PHYSIOLOGIC_MOMENT)
            .expect("Tet10 flexion equilibrium"),
        tet10
            .equilibrium(-PHYSIOLOGIC_MOMENT)
            .expect("Tet10 extension equilibrium"),
    );
    println!(
        "[ROM]  flexion   Tet4 {flex4:.4}° → Tet10 {flex10:.4}°   Δ {:+.4}°\n\
         [ROM]  extension Tet4 {ext4:.4}° → Tet10 {ext10:.4}°   Δ {:+.4}°",
        flex10 - flex4,
        ext10 - ext4
    );

    // ── The prediction, gated. ──
    let d_flex = (flex10 - flex4).abs();
    assert!(
        d_flex < PREREGISTERED_ROM_SHIFT_DEG,
        "§0.3 predicts the disc is ~0.4 % of the flexion restoring moment at ROM, so a \
         {:.1} % softer disc should shift segment flexion ROM by ~0.008° — measured {d_flex:.4}°, \
         beyond the pre-registered {PREREGISTERED_ROM_SHIFT_DEG}°. If this fires the prediction \
         was wrong: re-anchor honestly and record the measurement, never tune the element to the \
         old number.",
        100.0 * (1.0 - k10 / k4)
    );

    // The extension side is facet-capped, so it should move even less than flexion — the
    // bones stop on each other regardless of how stiff the disc is. Asserting the ORDER
    // (extension moves no more than flexion) tests the mechanism, not just the magnitude:
    // if a disc change moved the facet-limited side MORE, the limiter is not what we think.
    let d_ext = (ext10 - ext4).abs();
    assert!(
        d_ext <= d_flex + 1e-9,
        "the facet-capped extension ROM must be no more sensitive to the disc element than \
         the ligament-limited flexion ROM (flexion Δ {d_flex:.4}°, extension Δ {d_ext:.4}°) — \
         the extension limiter is facet contact, not the disc"
    );
}
