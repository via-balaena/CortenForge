//! **Rung-2 per-element conform delta** of `docs/FSU_TET10_COUPLING_MIGRATION_PLAN.md` (§4.3's
//! sanity column): what seating the bonded band on the real endplate does to `k_disc`, on both
//! elements, over one 2x2 table.
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
/// column below reproduces rung 1's two-probe numbers to four decimals (−0.2811 / −0.2788
/// Tet4, −0.1873 / −0.1849 Tet10). It only makes the conformed arms solvable at all.
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
    //   Tet4    raw −0.2811 / −0.2788  conf −0.2760 / −0.2738   0.982 / 0.982
    //   Tet10   raw −0.1873 / −0.1849  conf −0.1844 / −0.1820   0.984 / 0.984
    //   element ratio (Tet10/Tet4): raw 0.666 / 0.663, conformed 0.668 / 0.665
    //
    // Two things this table settles. (a) **The conform costs ~1.8 % of `k_disc`, not ~4 %.**
    // #701's FOM printed its arms at two decimals (−0.28 → −0.27) and the ~4 % figure was
    // read off that rounding; at four decimals the same comparison is −0.2811 → −0.2760.
    // (b) **The two axes are very nearly orthogonal**: the conform ratio is the same to
    // within 0.2 % on both elements, and the element ratio is the same to within 0.3 %
    // conformed vs raw. Seating the band on the bone and raising the element order are
    // therefore separable changes, which is what lets rung 3 and rung 4 reason about them
    // one at a time.
    for (kc, kr, expect, name) in [
        (k4_con_f, k4_raw_f, 0.982, "Tet4 flexion"),
        (k4_con_e, k4_raw_e, 0.982, "Tet4 extension"),
        (k10_con_f, k10_raw_f, 0.984, "Tet10 flexion"),
        (k10_con_e, k10_raw_e, 0.984, "Tet10 extension"),
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
        (k4_raw_f, -0.2811, "Tet4 raw flexion"),
        (k4_raw_e, -0.2788, "Tet4 raw extension"),
        (k10_raw_f, -0.1873, "Tet10 raw flexion"),
        (k10_raw_e, -0.1849, "Tet10 raw extension"),
        (k4_con_f, -0.2760, "Tet4 conformed flexion"),
        (k4_con_e, -0.2738, "Tet4 conformed extension"),
        (k10_con_f, -0.1844, "Tet10 conformed flexion"),
        (k10_con_e, -0.1820, "Tet10 conformed extension"),
    ] {
        assert!(
            ((1.05 * expect)..=(0.95 * expect)).contains(&k),
            "{name} k_disc {k:.4} is outside ±5 % of the committed {expect:.4} N·m/rad"
        );
    }
}
