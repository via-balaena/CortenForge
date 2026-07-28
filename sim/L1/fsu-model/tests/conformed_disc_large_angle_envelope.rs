//! **§4.5 large-angle gate** of `docs/FSU_TET10_COUPLING_MIGRATION_PLAN.md`: the angle
//! envelope of the endplate-conformed disc, on both elements.
//!
//! Lives here rather than in `src/lib.rs`'s test module for the reason `sim-coupling` keeps all
//! of its gates in `tests/`: an `#[ignore]`d, licence-gated figure of merit can never execute
//! under `cargo llvm-cov --lib`, so hosting it in the library target charges its whole body to
//! the crate's coverage grade while testing nothing there. It needs no private item — only
//! `build_bonded_disc`, `build_bonded_disc_tet10` and the public drive methods — so moving it
//! costs no API surface. (The two anatomy gates that *do* reach private items,
//! `conform_seats_the_bonded_face_on_the_bone_fom` and rung 3's
//! `curved_tet10_midsides_seat_on_the_endplate_fom`, necessarily stay in the library.)

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)] // tests may unwrap/expect/panic.

use cf_fsu_geometry::oracle;
use cf_fsu_model::{
    BondedDisc, DiscParams, EndplateConform, build_bonded_disc, build_bonded_disc_tet10,
};
use sim_soft::{Element, Mesh};

/// Drive `disc` through a chained, warm-started large-angle sweep and return the peak
/// |moment| reached.
///
/// The shape is `CoupledFsu::capture_ramp`'s: one continuous monotone chain from rest down
/// to `-peak` and back up through neutral to `+peak`, every step at most `substep`. Chaining
/// is load-bearing — resetting to rest between the legs would warm-start the next solve from
/// the far-deformed state, a jump large enough to leave the SPD region on its own and turn a
/// geometry finding into a stepping artifact.
///
/// Every step is a hard check: `flexion_moment` panics on a diverged solve, so reaching the
/// end at all is the envelope result, and each step's angle is printed so a panic names the
/// angle it failed at rather than only the solver's message.
fn assert_large_angle_sweep<Msh, E, const N: usize, const G: usize>(
    disc: &mut BondedDisc<Msh, E, N, G>,
    peak: f64,
    substep: f64,
    label: &str,
) -> f64
where
    Msh: Mesh,
    E: Element<N, G> + Default,
{
    disc.set_flexion(0.0);
    let mut cur = 0.0_f64;
    let mut peak_moment = 0.0_f64;
    for target in [-peak, peak] {
        // usize step count from a small angle ratio — exact in f64.
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let steps = ((target - cur).abs() / substep).ceil().max(1.0) as usize;
        for s in 1..=steps {
            #[allow(clippy::cast_precision_loss)]
            let frac = s as f64 / steps as f64;
            let theta = cur + (target - cur) * frac;
            let (moment, resid) = disc.flexion_moment(theta);
            println!(
                "{label}: θ = {:+.3}° M = {moment:+.5} N·m resid {resid:.2e}",
                theta.to_degrees()
            );
            assert!(
                resid < 1e-8,
                "{label}: θ = {:+.3}° bond must conserve (resid {resid:.2e})",
                theta.to_degrees()
            );
            // The −peak → +peak leg lands exactly on θ = 0, where `θ·M < 0` is false for any
            // moment; require the response to vanish there instead (checked explicitly, since
            // `0·M ≤ ε` is vacuous).
            if theta.abs() > 1e-12 {
                assert!(
                    theta * moment < 0.0,
                    "{label}: θ = {:+.3}° must stay restoring (θ·M = {:.2e})",
                    theta.to_degrees(),
                    theta * moment
                );
                peak_moment = peak_moment.max(moment.abs());
            } else {
                assert!(
                    moment.abs() < 0.1 * peak_moment,
                    "{label}: the neutral moment must be ~0, got {moment:.2e} \
                     (scale {peak_moment:.2e})"
                );
            }
        }
        cur = target;
    }
    peak_moment
}

/// **The rung-2 large-angle gate** (§4.5): the first measurement of the angle envelope of
/// the conformed disc, on both elements.
///
/// Every FOM in this crate before this one is a **sub-degree** probe, and the Tet10 arm has
/// only ever been driven to ±0.5°. But `CoupledFsu::capture_ramp` walks the FEM disc to the
/// full ±ROM (~6° flexion / ~4.5° extension), and this crate's own deferral note names
/// large-angle failure as the reason the bonded conform was not adopted there: *"a
/// whole-face-conformed mesh spawns sliver tets that fail the sweep"* (`coupled.rs`). Rung 4
/// flips `CoupledFsu` onto the conformed quadratic disc, so the envelope has to be measured
/// on the standalone disc first, where a failure is cheap and diagnosable.
///
/// ⚠ If an arm does not reach ±6°, that is a **finding** about the conformed mesh's envelope
/// — the peak is to be reported and rung 4 re-scoped, never the target quietly lowered.
#[test]
#[ignore = "needs $CF_L4_STL/$CF_L5_STL/$CF_DISC_STL (BodyParts3D, CC BY-SA, not committed)"]
fn conformed_disc_survives_a_large_angle_sweep() {
    use cf_fsu_geometry::{load_from_env, segment_frame};

    let l4 = load_from_env("CF_L4_STL").unwrap();
    let l5 = load_from_env("CF_L5_STL").unwrap();
    let disc_mesh = load_from_env("CF_DISC_STL").unwrap();
    let (o4, o5) = (oracle(&l4).unwrap(), oracle(&l5).unwrap());
    let frame = segment_frame(&l4, &l5, &o4, &o5).unwrap();
    let params = DiscParams::default();
    let ep = EndplateConform {
        o4: &o4,
        o5: &o5,
        superior_axis: frame.superior_axis,
    };

    // ±6° = the flexion end of `CoupledFsu`'s ROM, in `CAPTURE_SUBSTEP`-sized steps — the
    // production stepping, so the envelope measured here is the one rung 4 will drive.
    let peak = 6.0_f64.to_radians();
    let substep = 0.1_f64.to_radians();

    let mut tet4 =
        build_bonded_disc(disc_mesh.clone(), &params, Some(ep)).expect("conformed Tet4 disc bonds");
    let m4 = assert_large_angle_sweep(&mut tet4, peak, substep, "Tet4 conformed");

    let mut tet10 =
        build_bonded_disc_tet10(disc_mesh, &params, Some(ep)).expect("conformed Tet10 disc bonds");
    let m10 = assert_large_angle_sweep(&mut tet10, peak, substep, "Tet10 conformed");

    println!(
        "large-angle envelope: both conformed arms reached ±6.0° — peak |M| \
         Tet4 {m4:.4} / Tet10 {m10:.4} N·m"
    );

    // COMMITTED (BodyParts3D L4/L5/disc, DiscParams::default, 0.1° steps, 180 solves/arm):
    // **both conformed arms complete the full ±6.0° chain**, every step converging,
    // conserving and strictly restoring; peak |M| = 0.0300 (Tet4) / 0.0205 (Tet10) N·m.
    //
    // ★ **RE-RUN AT RUNG 3, on the CURVED disc, and the numbers did not move.** The Tet10 arm
    // here is now the curved disc (bonded-face boundary midsides projected onto the real
    // endplate), which is exactly the thing that could have narrowed this envelope — a curved
    // element is more distortion-sensitive than a straight one, and at the *corner* quality
    // floor it visibly was: that mesh cannot take a 0.15° single jump at all. At
    // `DISC_MIDSIDE_CONFORM_QUALITY_FLOOR` = 0.4 the full ±6.0° chain completes again and the
    // peak |M| reproduces rung 2's 0.0300 / 0.0205 to four decimals. So curving the bonded face
    // costs no angle envelope, measured rather than assumed (853 s, `--release`, idle machine).
    //
    // Two things follow, and neither was known before this rung.
    //
    // 1. **The deferral note in `coupled.rs` does not apply to this path.** It says a
    //    "whole-face-conformed mesh spawns sliver tets that fail the sweep" — true of
    //    STRATEGY A (conform the surface, then re-mesh, which the #701 record measured
    //    directly), and it is what kept `CoupledFsu` on raw geometry. The STRATEGY-B
    //    node-level conform measured here reaches the full ROM on both elements, so that
    //    reason for the deferral is retired. (What remains is the `RUNG7_K_DISC` re-anchor,
    //    which is rung 4's own deliverable.)
    // 2. **The Tet10 angle envelope is no longer unmeasured.** Before this rung the
    //    quadratic arm had only ever been driven to ±0.5°; it reaches ±6.0°.
    //
    // The peaks are a cross-check as well as a record: linear extrapolation of the
    // small-angle stiffnesses (`conform_delta_by_element_fom`: −0.2760 / −0.1844 N·m/rad)
    // to 6° = 0.1047 rad predicts 0.0289 / 0.0193 N·m, so the disc is ~4-6 % stiffer at the
    // ROM extreme than a linear spring — mild, monotone hardening, not a divergence.
    //
    // COST, measured over three runs: **897 s** wall (`--release`) on an otherwise-idle
    // machine, 936 s, and 1138 s while sharing the machine with other work — read it as
    // ~15-19 min. Tet10-dominated at ~5 s per warm-started 0.1° solve against sub-second for
    // Tet4. That is a same-shape, same-order measurement of what
    // `docs/FSU_TET10_COUPLING_MIGRATION_PLAN.md` §5.1 *estimated* for a Tet10 `capture_ramp`
    // (150-175 solves, 14-45 min): 180 solves here in ~15 min lands at the bottom of that
    // band. It is not a substitute for §5.1's own measurement at rung 4 — `capture_ramp` also
    // runs the equilibrium bisection per frame.
    for (m, expect, name) in [(m4, 0.0300, "Tet4"), (m10, 0.0205, "Tet10")] {
        assert!(
            ((0.95 * expect)..=(1.05 * expect)).contains(&m),
            "{name} conformed peak |M| {m:.4} is outside ±5 % of the committed {expect:.4} N·m"
        );
    }
}
