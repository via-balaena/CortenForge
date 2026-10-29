//! A geom's `fromto` compiled as MuJoCo 3.5.0 compiles it (book L14; ledger
//! L299, L327; A14 §2).
//!
//! MuJoCo's `mjCGeom::Compile` (`user_objects.cc:3677-3715`) takes a `fromto`
//! on a capsule, cylinder, ellipsoid or box only, and not beside a `pos`
//! other than 0; it puts half the segment's length in `size[1]` (in `size[2]`
//! for an ellipsoid or a box, whose `size[1]` takes `size[0]`), the middle in
//! `pos`, and the turn of z onto `from − to` (`mjuu_z2quat`) in `quat`; points
//! closer than mjEPS (`mjuu_normvec` gives 0 below 1e-7) are refused.
//!
//! `CASES` are MuJoCo 3.5.0's `geom_pos`, `geom_quat` and `geom_size` as bits,
//! measured on the census's unfused oracle, for four types and A14 §2's nine
//! directions (±x, +y, ±z, two general, two within 1e-9 of ±z) on a body turned
//! by `euler="30 20 10"`. The position and size take arithmetic and a square
//! root only, so they are compared bit for bit everywhere; the quaternion
//! passes through `atan2`, `cos` and `sin`, so off macOS arm64 within 1e-14.
//! A capsule's or cylinder's `size[2]` is not compared: MuJoCo keeps a written
//! third value there and sim-mjcf 0, for every capsule (the book's L18).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
// The cases are bit patterns, written as MuJoCo's values' raw bits.
#![allow(clippy::unreadable_literal)]

use sim_mjcf::{MjcfError, load_model};

/// `(type, written size, fromto, MuJoCo's geom_pos, geom_quat, geom_size)`.
#[allow(clippy::type_complexity)]
const CASES: &[(&str, &str, &str, [u64; 3], [u64; 4], [u64; 3])] = &[
    // capsule +x
    (
        "capsule",
        "0.05",
        "0 0 0 0.3 0 0",
        [0x3fc3333333333333, 0x0000000000000000, 0x0000000000000000],
        [
            0x3fe6a09e667f3bcd,
            0x0000000000000000,
            0xbfe6a09e667f3bcc,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fc3333333333333, 0x0000000000000000],
    ),
    // capsule -x
    (
        "capsule",
        "0.05",
        "0.3 0 0 0 0 0",
        [0x3fc3333333333333, 0x0000000000000000, 0x0000000000000000],
        [
            0x3fe6a09e667f3bcd,
            0x0000000000000000,
            0x3fe6a09e667f3bcc,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fc3333333333333, 0x0000000000000000],
    ),
    // capsule +y
    (
        "capsule",
        "0.05",
        "0.1 -0.2 0 0.1 0.2 0",
        [0x3fb999999999999a, 0x0000000000000000, 0x0000000000000000],
        [
            0x3fe6a09e667f3bcd,
            0x3fe6a09e667f3bcc,
            0x0000000000000000,
            0x8000000000000000,
        ],
        [0x3fa999999999999a, 0x3fc999999999999a, 0x0000000000000000],
    ),
    // capsule +z
    (
        "capsule",
        "0.05",
        "0.05 -0.02 0.1 0.05 -0.02 0.5",
        [0x3fa999999999999a, 0xbf947ae147ae147b, 0x3fd3333333333333],
        [
            0x3c91a62633145c07,
            0x3ff0000000000000,
            0x0000000000000000,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fc999999999999a, 0x0000000000000000],
    ),
    // capsule -z
    (
        "capsule",
        "0.05",
        "0 0 0.4 0 0 0",
        [0x0000000000000000, 0x0000000000000000, 0x3fc999999999999a],
        [
            0x3ff0000000000000,
            0x0000000000000000,
            0x0000000000000000,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fc999999999999a, 0x0000000000000000],
    ),
    // capsule g1
    (
        "capsule",
        "0.05",
        "0.1 0.2 0.3 -0.2 0.5 0.1",
        [0xbfa999999999999a, 0x3fd6666666666666, 0x3fc999999999999a],
        [
            0x3feb063e46a36235,
            0x3fd83c51209b80ab,
            0x3fd83c51209b80ac,
            0x8000000000000000,
        ],
        [0x3fa999999999999a, 0x3fce04c6f553bdd8, 0x0000000000000000],
    ),
    // capsule g2
    (
        "capsule",
        "0.05",
        "-0.3 0.1 0 0.2 -0.4 0.35",
        [0xbfa9999999999998, 0xbfc3333333333334, 0x3fc6666666666666],
        [
            0x3fe0e0d0ecfe5be7,
            0xbfe3395a203f4256,
            0xbfe3395a203f4256,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fd93f6104d67aed, 0x0000000000000000],
    ),
    // capsule nz+
    (
        "capsule",
        "0.05",
        "0 0 0 1e-10 0 0.3",
        [0x3dcb7cdfd9d7bdbb, 0x0000000000000000, 0x3fc3333333333333],
        [
            0x3c91a62633145c07,
            0x3ff0000000000000,
            0x0000000000000000,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fc3333333333333, 0x0000000000000000],
    ),
    // capsule nz-
    (
        "capsule",
        "0.05",
        "0 0 0.3 1e-10 0 0",
        [0x3dcb7cdfd9d7bdbb, 0x0000000000000000, 0x3fc3333333333333],
        [
            0x3ff0000000000000,
            0x0000000000000000,
            0x0000000000000000,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fc3333333333333, 0x0000000000000000],
    ),
    // cylinder +x
    (
        "cylinder",
        "0.05",
        "0 0 0 0.3 0 0",
        [0x3fc3333333333333, 0x0000000000000000, 0x0000000000000000],
        [
            0x3fe6a09e667f3bcd,
            0x0000000000000000,
            0xbfe6a09e667f3bcc,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fc3333333333333, 0x0000000000000000],
    ),
    // cylinder -x
    (
        "cylinder",
        "0.05",
        "0.3 0 0 0 0 0",
        [0x3fc3333333333333, 0x0000000000000000, 0x0000000000000000],
        [
            0x3fe6a09e667f3bcd,
            0x0000000000000000,
            0x3fe6a09e667f3bcc,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fc3333333333333, 0x0000000000000000],
    ),
    // cylinder +y
    (
        "cylinder",
        "0.05",
        "0.1 -0.2 0 0.1 0.2 0",
        [0x3fb999999999999a, 0x0000000000000000, 0x0000000000000000],
        [
            0x3fe6a09e667f3bcd,
            0x3fe6a09e667f3bcc,
            0x0000000000000000,
            0x8000000000000000,
        ],
        [0x3fa999999999999a, 0x3fc999999999999a, 0x0000000000000000],
    ),
    // cylinder +z
    (
        "cylinder",
        "0.05",
        "0.05 -0.02 0.1 0.05 -0.02 0.5",
        [0x3fa999999999999a, 0xbf947ae147ae147b, 0x3fd3333333333333],
        [
            0x3c91a62633145c07,
            0x3ff0000000000000,
            0x0000000000000000,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fc999999999999a, 0x0000000000000000],
    ),
    // cylinder -z
    (
        "cylinder",
        "0.05",
        "0 0 0.4 0 0 0",
        [0x0000000000000000, 0x0000000000000000, 0x3fc999999999999a],
        [
            0x3ff0000000000000,
            0x0000000000000000,
            0x0000000000000000,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fc999999999999a, 0x0000000000000000],
    ),
    // cylinder g1
    (
        "cylinder",
        "0.05",
        "0.1 0.2 0.3 -0.2 0.5 0.1",
        [0xbfa999999999999a, 0x3fd6666666666666, 0x3fc999999999999a],
        [
            0x3feb063e46a36235,
            0x3fd83c51209b80ab,
            0x3fd83c51209b80ac,
            0x8000000000000000,
        ],
        [0x3fa999999999999a, 0x3fce04c6f553bdd8, 0x0000000000000000],
    ),
    // cylinder g2
    (
        "cylinder",
        "0.05",
        "-0.3 0.1 0 0.2 -0.4 0.35",
        [0xbfa9999999999998, 0xbfc3333333333334, 0x3fc6666666666666],
        [
            0x3fe0e0d0ecfe5be7,
            0xbfe3395a203f4256,
            0xbfe3395a203f4256,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fd93f6104d67aed, 0x0000000000000000],
    ),
    // cylinder nz+
    (
        "cylinder",
        "0.05",
        "0 0 0 1e-10 0 0.3",
        [0x3dcb7cdfd9d7bdbb, 0x0000000000000000, 0x3fc3333333333333],
        [
            0x3c91a62633145c07,
            0x3ff0000000000000,
            0x0000000000000000,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fc3333333333333, 0x0000000000000000],
    ),
    // cylinder nz-
    (
        "cylinder",
        "0.05",
        "0 0 0.3 1e-10 0 0",
        [0x3dcb7cdfd9d7bdbb, 0x0000000000000000, 0x3fc3333333333333],
        [
            0x3ff0000000000000,
            0x0000000000000000,
            0x0000000000000000,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fc3333333333333, 0x0000000000000000],
    ),
    // box +x
    (
        "box",
        "0.05 0.07",
        "0 0 0 0.3 0 0",
        [0x3fc3333333333333, 0x0000000000000000, 0x0000000000000000],
        [
            0x3fe6a09e667f3bcd,
            0x0000000000000000,
            0xbfe6a09e667f3bcc,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fa999999999999a, 0x3fc3333333333333],
    ),
    // box -x
    (
        "box",
        "0.05 0.07",
        "0.3 0 0 0 0 0",
        [0x3fc3333333333333, 0x0000000000000000, 0x0000000000000000],
        [
            0x3fe6a09e667f3bcd,
            0x0000000000000000,
            0x3fe6a09e667f3bcc,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fa999999999999a, 0x3fc3333333333333],
    ),
    // box +y
    (
        "box",
        "0.05 0.07",
        "0.1 -0.2 0 0.1 0.2 0",
        [0x3fb999999999999a, 0x0000000000000000, 0x0000000000000000],
        [
            0x3fe6a09e667f3bcd,
            0x3fe6a09e667f3bcc,
            0x0000000000000000,
            0x8000000000000000,
        ],
        [0x3fa999999999999a, 0x3fa999999999999a, 0x3fc999999999999a],
    ),
    // box +z
    (
        "box",
        "0.05 0.07",
        "0.05 -0.02 0.1 0.05 -0.02 0.5",
        [0x3fa999999999999a, 0xbf947ae147ae147b, 0x3fd3333333333333],
        [
            0x3c91a62633145c07,
            0x3ff0000000000000,
            0x0000000000000000,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fa999999999999a, 0x3fc999999999999a],
    ),
    // box -z
    (
        "box",
        "0.05 0.07",
        "0 0 0.4 0 0 0",
        [0x0000000000000000, 0x0000000000000000, 0x3fc999999999999a],
        [
            0x3ff0000000000000,
            0x0000000000000000,
            0x0000000000000000,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fa999999999999a, 0x3fc999999999999a],
    ),
    // box g1
    (
        "box",
        "0.05 0.07",
        "0.1 0.2 0.3 -0.2 0.5 0.1",
        [0xbfa999999999999a, 0x3fd6666666666666, 0x3fc999999999999a],
        [
            0x3feb063e46a36235,
            0x3fd83c51209b80ab,
            0x3fd83c51209b80ac,
            0x8000000000000000,
        ],
        [0x3fa999999999999a, 0x3fa999999999999a, 0x3fce04c6f553bdd8],
    ),
    // box g2
    (
        "box",
        "0.05 0.07",
        "-0.3 0.1 0 0.2 -0.4 0.35",
        [0xbfa9999999999998, 0xbfc3333333333334, 0x3fc6666666666666],
        [
            0x3fe0e0d0ecfe5be7,
            0xbfe3395a203f4256,
            0xbfe3395a203f4256,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fa999999999999a, 0x3fd93f6104d67aed],
    ),
    // box nz+
    (
        "box",
        "0.05 0.07",
        "0 0 0 1e-10 0 0.3",
        [0x3dcb7cdfd9d7bdbb, 0x0000000000000000, 0x3fc3333333333333],
        [
            0x3c91a62633145c07,
            0x3ff0000000000000,
            0x0000000000000000,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fa999999999999a, 0x3fc3333333333333],
    ),
    // box nz-
    (
        "box",
        "0.05 0.07",
        "0 0 0.3 1e-10 0 0",
        [0x3dcb7cdfd9d7bdbb, 0x0000000000000000, 0x3fc3333333333333],
        [
            0x3ff0000000000000,
            0x0000000000000000,
            0x0000000000000000,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fa999999999999a, 0x3fc3333333333333],
    ),
    // ellipsoid +x
    (
        "ellipsoid",
        "0.05 0.07",
        "0 0 0 0.3 0 0",
        [0x3fc3333333333333, 0x0000000000000000, 0x0000000000000000],
        [
            0x3fe6a09e667f3bcd,
            0x0000000000000000,
            0xbfe6a09e667f3bcc,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fa999999999999a, 0x3fc3333333333333],
    ),
    // ellipsoid -x
    (
        "ellipsoid",
        "0.05 0.07",
        "0.3 0 0 0 0 0",
        [0x3fc3333333333333, 0x0000000000000000, 0x0000000000000000],
        [
            0x3fe6a09e667f3bcd,
            0x0000000000000000,
            0x3fe6a09e667f3bcc,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fa999999999999a, 0x3fc3333333333333],
    ),
    // ellipsoid +y
    (
        "ellipsoid",
        "0.05 0.07",
        "0.1 -0.2 0 0.1 0.2 0",
        [0x3fb999999999999a, 0x0000000000000000, 0x0000000000000000],
        [
            0x3fe6a09e667f3bcd,
            0x3fe6a09e667f3bcc,
            0x0000000000000000,
            0x8000000000000000,
        ],
        [0x3fa999999999999a, 0x3fa999999999999a, 0x3fc999999999999a],
    ),
    // ellipsoid +z
    (
        "ellipsoid",
        "0.05 0.07",
        "0.05 -0.02 0.1 0.05 -0.02 0.5",
        [0x3fa999999999999a, 0xbf947ae147ae147b, 0x3fd3333333333333],
        [
            0x3c91a62633145c07,
            0x3ff0000000000000,
            0x0000000000000000,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fa999999999999a, 0x3fc999999999999a],
    ),
    // ellipsoid -z
    (
        "ellipsoid",
        "0.05 0.07",
        "0 0 0.4 0 0 0",
        [0x0000000000000000, 0x0000000000000000, 0x3fc999999999999a],
        [
            0x3ff0000000000000,
            0x0000000000000000,
            0x0000000000000000,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fa999999999999a, 0x3fc999999999999a],
    ),
    // ellipsoid g1
    (
        "ellipsoid",
        "0.05 0.07",
        "0.1 0.2 0.3 -0.2 0.5 0.1",
        [0xbfa999999999999a, 0x3fd6666666666666, 0x3fc999999999999a],
        [
            0x3feb063e46a36235,
            0x3fd83c51209b80ab,
            0x3fd83c51209b80ac,
            0x8000000000000000,
        ],
        [0x3fa999999999999a, 0x3fa999999999999a, 0x3fce04c6f553bdd8],
    ),
    // ellipsoid g2
    (
        "ellipsoid",
        "0.05 0.07",
        "-0.3 0.1 0 0.2 -0.4 0.35",
        [0xbfa9999999999998, 0xbfc3333333333334, 0x3fc6666666666666],
        [
            0x3fe0e0d0ecfe5be7,
            0xbfe3395a203f4256,
            0xbfe3395a203f4256,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fa999999999999a, 0x3fd93f6104d67aed],
    ),
    // ellipsoid nz+
    (
        "ellipsoid",
        "0.05 0.07",
        "0 0 0 1e-10 0 0.3",
        [0x3dcb7cdfd9d7bdbb, 0x0000000000000000, 0x3fc3333333333333],
        [
            0x3c91a62633145c07,
            0x3ff0000000000000,
            0x0000000000000000,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fa999999999999a, 0x3fc3333333333333],
    ),
    // ellipsoid nz-
    (
        "ellipsoid",
        "0.05 0.07",
        "0 0 0.3 1e-10 0 0",
        [0x3dcb7cdfd9d7bdbb, 0x0000000000000000, 0x3fc3333333333333],
        [
            0x3ff0000000000000,
            0x0000000000000000,
            0x0000000000000000,
            0x0000000000000000,
        ],
        [0x3fa999999999999a, 0x3fa999999999999a, 0x3fc3333333333333],
    ),
];

#[test]
fn a_fromto_geom_is_posed_and_sized_as_mujocos() {
    let here = cfg!(all(target_os = "macos", target_arch = "aarch64"));
    let mut differ = Vec::new();
    for &(kind, size, fromto, pos, quat, sz) in CASES {
        let what = format!("{kind} fromto=\"{fromto}\"");
        let model = load_model(&format!(
            r#"<mujoco><worldbody><body name="b" euler="30 20 10"><freejoint/><geom type="{kind}" size="{size}" fromto="{fromto}"/></body></worldbody></mujoco>"#
        ))
        .unwrap_or_else(|e| panic!("{what}: {e}"));
        let got_pos = model.geom_pos[0];
        if (0..3).any(|k| got_pos[k].to_bits() != pos[k]) {
            differ.push(format!(
                "{what} pos: {got_pos:?}, MuJoCo {:?}",
                pos.map(f64::from_bits)
            ));
        }
        let q = model.geom_quat[0].into_inner();
        let got_quat = [q.w, q.i, q.j, q.k];
        let mujoco_quat = quat.map(f64::from_bits);
        let quat_ok = if here {
            got_quat
                .iter()
                .zip(&mujoco_quat)
                .all(|(a, b)| a.to_bits() == b.to_bits())
        } else {
            got_quat
                .iter()
                .zip(&mujoco_quat)
                .all(|(a, b)| (a - b).abs() <= 1e-14)
        };
        if !quat_ok {
            differ.push(format!("{what} quat: {got_quat:?}, MuJoCo {mujoco_quat:?}"));
        }
        let got_size = model.geom_size[0];
        let compared = if matches!(kind, "capsule" | "cylinder") {
            2
        } else {
            3
        };
        if (0..compared).any(|k| got_size[k].to_bits() != sz[k]) {
            differ.push(format!(
                "{what} size: {got_size:?}, MuJoCo {:?}",
                sz.map(f64::from_bits)
            ));
        }
    }
    assert!(
        differ.is_empty(),
        "{} differ:\n  {}",
        differ.len(),
        differ.join("\n  ")
    );
}

/// Each refused by MuJoCo 3.5.0 (the reasons are its, in sim-mjcf's site
/// wording): `(what, document, reason)`.
const REFUSED: &[(&str, &str, &str)] = &[
    (
        "a sphere",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom type="sphere" size="0.05" fromto="0 0 0 0 0 1"/></body></worldbody></mujoco>"#,
        "fromto requires a capsule, cylinder, ellipsoid or box",
    ),
    (
        "the default type, a sphere",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom size="0.05" fromto="0 0 0 0 0 1"/></body></worldbody></mujoco>"#,
        "fromto requires a capsule, cylinder, ellipsoid or box",
    ),
    (
        "a class's sphere",
        r#"<mujoco><default><default class="s"><geom type="sphere"/></default></default><worldbody><body name="b"><freejoint/><geom class="s" size="0.05" fromto="0 0 0 0 0 1"/></body></worldbody></mujoco>"#,
        "fromto requires a capsule, cylinder, ellipsoid or box",
    ),
    (
        "a class's fromto on a sphere",
        r#"<mujoco><default><default class="s"><geom fromto="0 0 0 0 0 1"/></default></default><worldbody><body name="b"><freejoint/><geom class="s" type="sphere" size="0.05"/></body></worldbody></mujoco>"#,
        "fromto requires a capsule, cylinder, ellipsoid or box",
    ),
    (
        "a plane",
        r#"<mujoco><worldbody><geom type="plane" size="1 1 1" fromto="0 0 0 0 0 1"/></worldbody></mujoco>"#,
        "fromto requires a capsule, cylinder, ellipsoid or box",
    ),
    (
        "a pos beside it",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom type="capsule" size="0.05" pos="0 0 0.1" fromto="0 0 0 0 0 1"/></body></worldbody></mujoco>"#,
        "both pos and fromto defined",
    ),
    (
        "a class's pos beside it",
        r#"<mujoco><default><default class="p"><geom pos="0 0 0.1"/></default></default><worldbody><body name="b"><freejoint/><geom class="p" type="capsule" size="0.05" fromto="0 0 0 0 0 1"/></body></worldbody></mujoco>"#,
        "both pos and fromto defined",
    ),
    (
        "coincident points",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom type="capsule" size="0.05" fromto="0.1 0.2 0.3 0.1 0.2 0.3"/></body></worldbody></mujoco>"#,
        "fromto points too close",
    ),
    (
        "points 1e-14 apart",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom type="capsule" size="0.05" fromto="0 0 0 0 0 1e-14"/></body></worldbody></mujoco>"#,
        "fromto points too close",
    ),
    (
        "points 3e-14 apart",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom type="capsule" size="0.05" fromto="0 0 0 0 0 3e-14"/></body></worldbody></mujoco>"#,
        "fromto points too close",
    ),
    (
        "a cable with no curve",
        r#"<mujoco><extension><plugin plugin="mujoco.elasticity.cable"/></extension><worldbody><composite type="cable" count="5 1 1" size="1"><geom type="capsule" size="0.01"/></composite></worldbody></mujoco>"#,
        "fromto points too close",
    ),
    (
        "a cable whose curve vanishes",
        r#"<mujoco><extension><plugin plugin="mujoco.elasticity.cable"/></extension><worldbody><composite type="cable" curve="cos(s) 0 0" count="5 1 1" size="1"><geom type="capsule" size="0.01"/></composite></worldbody></mujoco>"#,
        "fromto points too close",
    ),
    (
        "a mesh beside it",
        r#"<mujoco><asset><mesh name="m" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 2 1 0 1 3 0 3 2 1 2 3"/></asset><worldbody><body name="b"><freejoint/><geom type="capsule" mesh="m" size="0.05" fromto="0 0 0 0 0 1"/></body></worldbody></mujoco>"#,
        "fromto cannot be used with mesh geom",
    ),
];

#[test]
fn a_fromto_mujoco_refuses_is_refused() {
    let mut missed = Vec::new();
    for &(what, xml, reason) in REFUSED {
        match load_model(xml) {
            Err(MjcfError::InvalidValue {
                element,
                attribute,
                reason: r,
                ..
            }) if element == "geom" && attribute == "fromto" && r == reason => {}
            Err(other) => missed.push(format!("{what}: {other}")),
            Ok(_) => missed.push(format!("{what}: loads")),
        }
    }
    assert!(
        missed.is_empty(),
        "{} of {} not refused as MuJoCo refuses them:\n  {}",
        missed.len(),
        REFUSED.len(),
        missed.join("\n  ")
    );
}

/// A site's `fromto` takes the same rule (`fromto_pose`): MuJoCo 3.5.0 refuses
/// each of these (S21 oracle), and the refusal names the site.
#[test]
fn a_sites_fromto_refusals_name_the_site() {
    for (site, reason) in [
        (
            r#"type="sphere" fromto="0 0 0 0 0 1""#,
            "fromto requires a capsule, cylinder, ellipsoid or box",
        ),
        (
            r#"type="capsule" pos="0 0 0.1" fromto="0 0 0 0 0 1""#,
            "both pos and fromto defined",
        ),
        (
            r#"type="capsule" fromto="0.1 0.2 0.3 0.1 0.2 0.3""#,
            "fromto points too close",
        ),
    ] {
        let err = load_model(&format!(
            r#"<mujoco><worldbody><body name="b"><freejoint/><geom type="sphere" size="0.1"/><site name="s" size="0.05" {site}/></body></worldbody></mujoco>"#
        ))
        .unwrap_err();
        assert!(
            matches!(&err, MjcfError::InvalidValue { element, attribute, reason: r, .. }
                if element == "site" && attribute == "fromto" && r == reason),
            "{site}: {err}"
        );
    }
}

/// MuJoCo 3.5.0 loads each: a `pos` of 0 beside a fromto, and a box whose one
/// or three sizes become (size[0], size[0], half its length).
#[test]
fn a_zero_pos_beside_a_fromto_and_a_boxs_sizes_load_as_mujocos() {
    load_model(ZERO_POS).unwrap();
    for (size, mujoco) in [
        ("0.05", [0.05, 0.05, 0.5]),
        ("0.05 0.07 0.09", [0.05, 0.05, 0.5]),
    ] {
        let model = load_model(&format!(
            r#"<mujoco><worldbody><body name="b"><freejoint/><geom type="box" size="{size}" fromto="0 0 0 0 0 1"/></body></worldbody></mujoco>"#
        ))
        .unwrap();
        let got = model.geom_size[0];
        assert_eq!(
            [got.x, got.y, got.z].map(f64::to_bits),
            mujoco.map(f64::to_bits),
            "box size=\"{size}\": {got:?}"
        );
    }
}

const ZERO_POS: &str = r#"<mujoco><worldbody><body name="b"><freejoint/><geom type="capsule" size="0.05" pos="0 0 0" fromto="0 0 0 0 0 1"/></body></worldbody></mujoco>"#;

/// MuJoCo 3.5.0 turns a geom by its `fromto` and never resolves an
/// alternative beside it (`mjCGeom::Compile`, `user_objects.cc:3677-3724`), the
/// geom's own or its class's: each of these is the plain fromto's turn, z onto
/// −x, as bits (`z2quat` passes through `atan2`, `cos`, `sin`: within 1e-14 off
/// macOS arm64).
#[test]
fn a_fromto_replaces_an_alternative_as_mujocos_does() {
    let here = cfg!(all(target_os = "macos", target_arch = "aarch64"));
    let mujoco = [
        0x3fe6a09e667f3bcd_u64,
        0x0000000000000000,
        0xbfe6a09e667f3bcc,
        0x0000000000000000,
    ]
    .map(f64::from_bits);
    for (what, defaults, attr) in [
        ("its own euler", "", r#" euler="0 0 45""#),
        (
            "its class's euler",
            r#"<default><default class="e"><geom euler="0 0 45"/></default></default>"#,
            r#" class="e""#,
        ),
    ] {
        let model = load_model(&format!(
            r#"<mujoco>{defaults}<worldbody><body name="b"><freejoint/><geom type="capsule" size="0.05" fromto="0 0 0 0.3 0 0"{attr}/></body></worldbody></mujoco>"#
        ))
        .unwrap();
        let q = model.geom_quat[0].into_inner();
        let got = [q.w, q.i, q.j, q.k];
        let ok = got.iter().zip(&mujoco).all(|(a, b)| {
            if here {
                a.to_bits() == b.to_bits()
            } else {
                (a - b).abs() <= 1e-14
            }
        });
        assert!(ok, "beside {what}: {got:?}, MuJoCo {mujoco:?}");
    }
}

/// A `fromto` a geom takes from its class or childclass, on the paths that
/// move a geom before its class is applied: a frame (with a `pos`, on a body or
/// the worldbody), a body fusestatic folds away (at an offset, into a parent or
/// the world), beside the geom's own too-small alternative, and a cable's
/// capsules under a default `pos` (MuJoCo makes a cable's geoms from the
/// composite's own default, not the user's classes). MuJoCo 3.5.0 loads each (S21 oracle). Where the pose here is
/// MuJoCo's, its `geom_pos` bits are given; a fused body's is not moved yet
/// (the book's L42). `(what, document, capsule's geom index, MuJoCo's geom_pos)`.
#[allow(clippy::type_complexity)]
const CLASS_FROMTO: &[(&str, &str, usize, Option<[u64; 3]>)] = &[
    (
        "a class's fromto in a frame with a pos",
        r#"<mujoco><default><default class="c"><geom type="capsule" size="0.05" fromto="0 0 0.1 0 0 0.3"/></default></default><worldbody><body name="b"><freejoint/><frame pos="0.5 0 0"><geom class="c"/></frame></body></worldbody></mujoco>"#,
        0,
        Some([0x3fe0000000000000, 0x0000000000000000, 0x3fc999999999999a]),
    ),
    (
        "a childclass's fromto in a frame with a pos",
        r#"<mujoco><default><default class="c"><geom type="capsule" size="0.05" fromto="0 0 0.1 0 0 0.3"/></default></default><worldbody><body name="b"><freejoint/><frame childclass="c" pos="0.5 0 0"><geom/></frame></body></worldbody></mujoco>"#,
        0,
        Some([0x3fe0000000000000, 0x0000000000000000, 0x3fc999999999999a]),
    ),
    (
        "a class's fromto in a worldbody frame with a pos",
        r#"<mujoco><default><default class="c"><geom type="capsule" size="0.05" fromto="0 0 0.1 0 0 0.3"/></default></default><worldbody><frame pos="0.5 0 0"><geom class="c"/></frame></worldbody></mujoco>"#,
        0,
        Some([0x3fe0000000000000, 0x0000000000000000, 0x3fc999999999999a]),
    ),
    (
        "a class's fromto in a body fused at an offset",
        r#"<mujoco><default><default class="c"><geom type="capsule" size="0.05" fromto="0 0 0.1 0 0 0.3"/></default></default><compiler fusestatic="true"/><worldbody><body name="p"><joint type="hinge"/><geom type="sphere" size="0.1"/><body name="b" pos="0.1 0 0"><geom class="c"/></body></body></worldbody></mujoco>"#,
        1,
        None,
    ),
    (
        "a class's fromto in a body fused into the world",
        r#"<mujoco><default><default class="c"><geom type="capsule" size="0.05" fromto="0 0 0.1 0 0 0.3"/></default></default><compiler fusestatic="true"/><worldbody><body name="b" pos="0.1 0 0"><geom class="c"/></body></worldbody></mujoco>"#,
        0,
        None,
    ),
    (
        "a class's fromto beside its own too-small zaxis in a frame",
        r#"<mujoco><default><default class="c"><geom type="capsule" size="0.05" fromto="0 0 0.1 0 0 0.3"/></default></default><worldbody><body name="b"><freejoint/><frame euler="0 0 30"><geom class="c" zaxis="0 0 0"/></frame></body></worldbody></mujoco>"#,
        0,
        Some([0x0000000000000000, 0x0000000000000000, 0x3fc999999999999a]),
    ),
    (
        "a class's fromto beside its own too-small zaxis in a turned fused body",
        r#"<mujoco><default><default class="c"><geom type="capsule" size="0.05" fromto="0 0 0.1 0 0 0.3"/></default></default><compiler fusestatic="true"/><worldbody><body name="p"><joint type="hinge"/><geom type="sphere" size="0.1"/><body name="b" euler="0 0 30"><geom class="c" zaxis="0 0 0"/></body></body></worldbody></mujoco>"#,
        1,
        None,
    ),
    (
        "a cable under a default pos",
        r#"<mujoco><extension><plugin plugin="mujoco.elasticity.cable"/></extension><default><geom pos="0 0 0.01"/></default><worldbody><composite type="cable" curve="s 0 0" count="4 1 1" size="1"><geom type="capsule" size="0.01"/></composite></worldbody></mujoco>"#,
        2,
        Some([0x3fc5555540000000, 0x0000000000000000, 0x0000000000000000]),
    ),
    (
        "a cable under a childclass pos",
        r#"<mujoco><extension><plugin plugin="mujoco.elasticity.cable"/></extension><default><default class="p"><geom pos="0 0 0.01"/></default></default><worldbody><body name="w" childclass="p"><composite type="cable" curve="s 0 0" count="4 1 1" size="1"><geom type="capsule" size="0.01"/></composite></body></worldbody></mujoco>"#,
        2,
        Some([0x3fc5555540000000, 0x0000000000000000, 0x0000000000000000]),
    ),
];

#[test]
fn a_class_fromto_on_every_path_that_moves_a_geom_loads_as_mujocos() {
    let mut missed = Vec::new();
    for &(what, xml, geom, mujoco) in CLASS_FROMTO {
        match load_model(xml) {
            Err(e) => missed.push(format!("{what}: {e}")),
            Ok(model) => {
                if let Some(mujoco) = mujoco {
                    let got = model.geom_pos[geom];
                    if (0..3).any(|k| got[k].to_bits() != mujoco[k]) {
                        missed.push(format!(
                            "{what}: geom_pos {got:?}, MuJoCo {:?}",
                            mujoco.map(f64::from_bits)
                        ));
                    }
                }
            }
        }
    }
    assert!(
        missed.is_empty(),
        "{} of {} not as MuJoCo:\n  {}",
        missed.len(),
        CLASS_FROMTO.len(),
        missed.join("\n  ")
    );
}

/// A `fromto` geom in a body fusestatic folds into its parent at an offset:
/// MuJoCo 3.5.0 loads it, its `geom_pos` (0.1, 0, 0.15) (measured). Its pose
/// here is still the segment read in the parent's frame (moving the segment
/// is the book's L42); fusion sets it no `pos`, which beside a `fromto` is
/// refused.
#[test]
fn a_fused_fromto_geom_away_from_the_origin_loads() {
    load_model(FUSED_AT_AN_OFFSET).unwrap();
}

const FUSED_AT_AN_OFFSET: &str = r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p"><joint type="hinge"/><geom type="sphere" size="0.1"/><body name="b" pos="0.1 0 0"><geom type="capsule" size="0.05" fromto="0 0 0 0 0 0.3"/></body></body></worldbody></mujoco>"#;

/// A `framequat` and a `framezaxis` on a `fromto` capsule in a turned body read
/// MuJoCo 3.5.0's values after `mj_forward` (A14 §2.6's sensor case: before
/// `8f6322ca` z pointed the other way). The orientation passes through `sin`,
/// `cos` and `atan2`: bit for bit on macOS arm64, within 1e-14 elsewhere.
#[test]
fn a_fromto_capsules_frame_sensors_read_as_mujocos() {
    let model = load_model(FRAME_SENSORS).unwrap();
    let mut data = model.make_data();
    data.forward(&model).unwrap();
    let mujoco = [
        0x3fe47e5eb666f075_u64,
        0x3fe125440a4b81fc,
        0x3fe0e5ef7a77fab0,
        0x3fc3cbb15bc5abd7,
        0x3feaf2840902c59c,
        0xbfe0bbf4c2fbe374,
        0xbfc0e13aeecd9cb3,
    ]
    .map(f64::from_bits);
    let here = cfg!(all(target_os = "macos", target_arch = "aarch64"));
    let got: Vec<f64> = data.sensordata.iter().copied().collect();
    assert_eq!(got.len(), mujoco.len());
    for (k, (a, b)) in got.iter().zip(&mujoco).enumerate() {
        let ok = if here {
            a.to_bits() == b.to_bits()
        } else {
            (a - b).abs() <= 1e-14
        };
        assert!(ok, "sensordata[{k}]: {a}, MuJoCo {b}");
    }
}

const FRAME_SENSORS: &str = r#"<mujoco><worldbody><geom type="plane" size="0 0 1"/><body name="b" pos="0 0 0.5" euler="30 20 10"><freejoint/><geom name="c" type="capsule" size="0.05" fromto="0.1 0.2 0.3 -0.2 0.5 0.1"/></body></worldbody><sensor><framequat objtype="geom" objname="c"/><framezaxis objtype="geom" objname="c"/></sensor></mujoco>"#;
