//! **Per-element conform delta** of `docs/FSU_TET10_COUPLING_MIGRATION_PLAN.md` (§4.3's sanity
//! column): what seating the bonded band on the real endplate does to `k_disc`, on both
//! elements, over one 2x2 table.
//!
//! Built at rung 2; it also carries **rung 3's committed `k_disc` shift**, because the
//! conformed Tet10 arm is exactly the disc rung 3 curved — see the shift note at the committed
//! table below.
//!
//! Lives here rather than in `src/lib.rs`'s test module for the reason `sim-coupling` keeps all
//! of its gates in `tests/`: an `#[ignore]`d, licence-gated figure of merit can never execute
//! under `cargo llvm-cov --lib`, so hosting it in the library target charges its whole body to
//! the crate's coverage grade while testing nothing there. It needs no private item, so moving
//! it costs no API surface.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)] // tests may unwrap/expect/panic.

use cf_fsu_geometry::oracle;
use cf_fsu_model::{
    BondedDisc, DiscParams, EndplateConform, build_bonded_disc, build_bonded_disc_tet10,
};
use sim_soft::{Element, Mesh};

// The committed disc-stiffness table, shared with `rung5_step1_mesh_realization_noise_floor_fom`
// in `src/lib.rs`, which cross-checks its sweep's shipped row against the same raw column. One
// definition so a re-anchor cannot land on one gate and miss the other.
use cf_fsu_model::committed_anchors::{
    COMMITTED_TET4_CONFORMED, COMMITTED_TET4_RAW, COMMITTED_TET10_CONFORMED, COMMITTED_TET10_RAW,
};

/// `(k_flex, k_ext)` at ±0.5° for a bonded disc, measured through the **stepped**
/// `capture_flexion` sweep #701's conform FOM uses (−0.5, −0.25, 0, +0.25, +0.5°).
///
/// ⚠ The stepping is load-bearing, not a stylistic choice — **measured while building this
/// rung.** The RAW Tet4 disc survives a single +0.5° → −0.5° jump (that is exactly how rung
/// 1's element-order FOM probes it), but the **conformed** Tet4 disc does not: the same 1°
/// jump drives tet 7495 to a principal stretch of 2.845 and trips the solver's fail-closed
/// validity bound. Seating the band on the real endplate leaves the boundary elements less
/// tolerant of a large single step. So every arm here is walked in ≤ 0.5° increments, which
/// is also what makes the 2×2 table internally comparable.
///
/// The stepping does not move the measurement, which was checked rather than assumed: the raw
/// column below reproduces rung 1's two-probe numbers to four decimals (−0.1170 / −0.1156
/// Tet4, −0.0968 / −0.0958 Tet10). It only makes the conformed arms solvable at all.
///
/// ⚠ **Rung 3 nearly cost this stepping too, and that is why the curved disc has its own
/// quality floor.** With the midside projection held to the *corner* floor (0.05) the curved
/// disc no longer survives even a 0.15° jump from rest — it walks in 0.05° steps and stalls
/// Newton with a `NaN` residual at anything larger, which is what this test found first. At
/// `cf_fsu_model`'s `DISC_MIDSIDE_CONFORM_QUALITY_FLOOR` (0.4, chosen on the measured table in
/// that constant's doc) the 0.5° stepping below solves again, on the curved disc, unchanged.
fn k_disc_flex_ext<Msh, E, const N: usize, const G: usize>(
    disc: &mut BondedDisc<Msh, E, N, G>,
) -> (f64, f64)
where
    Msh: Mesh,
    E: Element<N, G> + Default,
{
    let angles: Vec<f64> = [-0.5_f64, -0.25, 0.0, 0.25, 0.5]
        .iter()
        .map(|d| d.to_radians())
        .collect();
    let traj = disc.capture_flexion(&angles);
    let k = |theta: f64| -> f64 {
        let f = traj
            .frames
            .iter()
            .find(|f| (f.theta - theta).abs() < 1e-12)
            .expect("swept angle present");
        assert!(
            f.conservation_resid < 1e-8,
            "bond must conserve (resid {:.2e})",
            f.conservation_resid
        );
        f.moment / theta
    };
    (k(0.5_f64.to_radians()), k(-0.5_f64.to_radians()))
}

/// **The rung-2 per-element conform delta** (§4.3 sanity column): what seating the bonded
/// band on the real endplate does to `k_disc`, measured on *both* elements over one 2×2
/// table so the conform axis and the element axis are separable.
///
/// Gate shape is #701's, deliberately: sound + measured + **no-regression**, not
/// require-improvement. A more-correct geometry may legitimately move `k_disc` either way,
/// and a require-improvement gate on it would false-fail exactly the change this arc is
/// making. The require-improvement gate in this rung is the geometric one
/// (`conform_seats_the_bonded_face_on_the_bone_fom`) — the residual is the claim, `k_disc`
/// is its physics consequence.
#[test]
#[ignore = "needs $CF_L4_STL/$CF_L5_STL/$CF_DISC_STL (BodyParts3D, CC BY-SA, not committed)"]
fn conform_delta_by_element_fom() {
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
    let (k4_raw_f, k4_raw_e) =
        k_disc_flex_ext(&mut build_bonded_disc(disc_mesh.clone(), &params, None).unwrap());
    let (k4_con_f, k4_con_e) =
        k_disc_flex_ext(&mut build_bonded_disc(disc_mesh.clone(), &params, Some(ep)).unwrap());
    let (k10_raw_f, k10_raw_e) =
        k_disc_flex_ext(&mut build_bonded_disc_tet10(disc_mesh.clone(), &params, None).unwrap());
    let (k10_con_f, k10_con_e) =
        k_disc_flex_ext(&mut build_bonded_disc_tet10(disc_mesh, &params, Some(ep)).unwrap());

    println!(
        "k_disc (N·m/rad), flex / ext —\n  \
         Tet4  raw {k4_raw_f:.4} / {k4_raw_e:.4}   conformed {k4_con_f:.4} / {k4_con_e:.4}\n  \
         Tet10 raw {k10_raw_f:.4} / {k10_raw_e:.4}   conformed {k10_con_f:.4} / {k10_con_e:.4}\n  \
         conform ratio (conformed/raw): Tet4 {:.3} / {:.3}, Tet10 {:.3} / {:.3}\n  \
         element ratio (Tet10/Tet4):    raw  {:.3} / {:.3}, conformed {:.3} / {:.3}",
        k4_con_f / k4_raw_f,
        k4_con_e / k4_raw_e,
        k10_con_f / k10_raw_f,
        k10_con_e / k10_raw_e,
        k10_raw_f / k4_raw_f,
        k10_raw_e / k4_raw_e,
        k10_con_f / k4_con_f,
        k10_con_e / k4_con_e,
    );

    // Every arm is a restoring spring.
    for (k, name) in [
        (k4_raw_f, "Tet4 raw flexion"),
        (k4_raw_e, "Tet4 raw extension"),
        (k4_con_f, "Tet4 conformed flexion"),
        (k4_con_e, "Tet4 conformed extension"),
        (k10_raw_f, "Tet10 raw flexion"),
        (k10_raw_e, "Tet10 raw extension"),
        (k10_con_f, "Tet10 conformed flexion"),
        (k10_con_e, "Tet10 conformed extension"),
    ] {
        assert!(
            k < 0.0 && k.is_finite(),
            "{name} k_disc must restore ({k:.4})"
        );
    }

    // COMMITTED (BodyParts3D L4/L5/disc, DiscParams::default, stepped ±0.5° sweep), N·m/rad:
    //
    //           flex      ext          flex      ext        conform ratio
    //   Tet4    raw −0.1170 / −0.1156  conf −0.1146 / −0.1132   0.979 / 0.979
    //   Tet10   raw −0.0968 / −0.0958  conf −0.0948 / −0.0938   0.979 / 0.979
    //   element ratio (Tet10/Tet4): raw 0.827 / 0.829, conformed 0.827 / 0.828
    //
    // ⚠ RE-ANCHORED at β.4 (α.1 removed phantom material; same element, different domain).
    // The absolutes fell 48-58 %. The CONFORM ratios barely moved (0.982/0.985 → 0.979/0.979);
    // the ELEMENT ratio moved 0.666 → 0.827 for the mechanism recorded in
    // `src/committed_anchors.rs`. What localises the change to the geometry is
    // that both element arms fell together with no element code touched. Source of truth for
    // the eight absolutes:
    // `src/committed_anchors.rs`, imported above.
    //
    // ★ **The rung-3 `k_disc` shift, which is this table's job to record.** The conformed Tet10
    // arm is now the *curved* disc (its bonded-face boundary midsides are projected onto the
    // real endplate, not left at their edge midpoints). Rung 2 measured the same arm with
    // straight midsides at −0.1844 / −0.1820; curving them moves it to −0.1845 / −0.1821, a
    // shift of **0.05 %**. ⚠ Those four figures and the RMS pair below are rung 2/3 measurements
    // taken PRE-α.1 and were NOT re-measured at β.4 — the straight-midside arm is not built by
    // this gate, so there is no post-α.1 number for it. They are a historical record of that
    // comparison; do not read them as current absolutes. The *relation* they establish still
    // holds: that shift is an order of magnitude smaller than the conform's own ~2.1 % and two
    // orders below the element order's ~17 %. Read that as the arc's framing holding rather
    // than as a null result: the geometric claim is measured directly by
    // `curved_tet10_midsides_seat_on_the_endplate_fom` (the bonded face's RMS distance to the
    // bone falls 0.796 → 0.694 mm), and `k_disc` is its physics consequence, which a
    // require-improvement gate would have false-failed in either direction.
    //
    // Two things this table settles. (a) **The conform costs ~2.1 % of `k_disc`, not ~4 %.**
    // #701's FOM printed its arms at two decimals and the ~4 % figure was read off that
    // rounding; at four decimals the same comparison is −0.1170 → −0.1146. (The reasoning is
    // rung 2's and survives β.4 intact; only the figure moved, from ~1.8 % to ~2.1 %.)
    // (b) **The two axes are orthogonal — more cleanly than when this was written.** The
    // conform ratio was the same to within 0.2 % on both elements; post-α.1 it is **0.979 on
    // both, identical to three decimals**. The element ratio is the same to within 0.3 %
    // conformed vs raw. Seating the band on the bone and raising the element order are
    // therefore separable changes, which is what lets rung 3 and rung 4 reason about them
    // one at a time.
    for (kc, kr, expect, name) in [
        (k4_con_f, k4_raw_f, 0.979, "Tet4 flexion"),
        (k4_con_e, k4_raw_e, 0.979, "Tet4 extension"),
        (k10_con_f, k10_raw_f, 0.979, "Tet10 flexion"),
        (k10_con_e, k10_raw_e, 0.979, "Tet10 extension"),
    ] {
        let ratio = kc / kr;
        assert!(
            ((0.95 * expect)..=(1.05 * expect)).contains(&ratio),
            "{name}: conform ratio {ratio:.3} is outside ±5 % of the committed {expect:.3} \
             ({kc:.4} vs {kr:.4})"
        );
    }
    // The raw column must still be rung 1's, so a drift in the shared geometry pipeline
    // fails here as well as there.
    for (k, expect, name) in [
        (k4_raw_f, COMMITTED_TET4_RAW.0, "Tet4 raw flexion"),
        (k4_raw_e, COMMITTED_TET4_RAW.1, "Tet4 raw extension"),
        (k10_raw_f, COMMITTED_TET10_RAW.0, "Tet10 raw flexion"),
        (k10_raw_e, COMMITTED_TET10_RAW.1, "Tet10 raw extension"),
        (
            k4_con_f,
            COMMITTED_TET4_CONFORMED.0,
            "Tet4 conformed flexion",
        ),
        (
            k4_con_e,
            COMMITTED_TET4_CONFORMED.1,
            "Tet4 conformed extension",
        ),
        (
            k10_con_f,
            COMMITTED_TET10_CONFORMED.0,
            "Tet10 conformed flexion",
        ),
        (
            k10_con_e,
            COMMITTED_TET10_CONFORMED.1,
            "Tet10 conformed extension",
        ),
    ] {
        assert!(
            ((1.05 * expect)..=(0.95 * expect)).contains(&k),
            "{name} k_disc {k:.4} is outside ±5 % of the committed {expect:.4} N·m/rad"
        );
    }
}
