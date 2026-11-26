//! An equality's data as MuJoCo 3.5.0 makes it (book Rigid-loading L41;
//! ledger L215): a weld's `anchor`, `relpose` as written and `torquescale`
//! as its reader writes them (`xml_native_reader.cc:2149-2193`), each
//! equality's `data[10]` MuJoCo's default 1 (`user_init.c:301-307`), and
//! what `mj_setConst` completes at `qpos0` (`engine_setconst.c:783-838`): a
//! connect's point in body2's frame, and a weld's anchor in body1's frame
//! and relative quaternion where the written quaternion is all 0, any other
//! normalized.
//!
//! Every expected row was printed by MuJoCo 3.5.0 on the census's unfused
//! oracle (2026-11-23).

// The values are MuJoCo's, printed to the last digit.
#![allow(clippy::unwrap_used, clippy::unreadable_literal)]

use sim_core::MjObjectType;
use sim_mjcf::load_model;

/// Each document, and the `eq_data` row MuJoCo 3.5.0 compiles for its one
/// equality: but the last two (a tendon equality, a flexcomp's edge
/// equality), a hinged box `a` and a free box `c`, each turned by an `euler`.
const CASES: &[(&str, [f64; 11])] = &[
    // a weld with no relpose: its anchor and quaternion made at qpos0
    (
        r#"<mujoco><worldbody><body name="a" pos="0 0 1" euler="0.1 0.2 0.3"><joint type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.05"/></body><body name="c" pos="0.3 0.1 1.2" euler="0.4 0 0.5"><freejoint/><geom type="box" size="0.05 0.05 0.05"/></body></worldbody><equality><weld body1="a" body2="c"/></equality></mujoco>"#,
        [
            0.0,
            0.0,
            0.0,
            0.29982197285251044,
            0.09878041041621292,
            0.2008711405673625,
            0.999993518920471,
            0.0026057385516249833,
            -0.0017635567004858253,
            0.0017498889732764835,
            1.0,
        ],
    ),
    // a relpose: its position kept, its unit quaternion too
    (
        r#"<mujoco><worldbody><body name="a" pos="0 0 1" euler="0.1 0.2 0.3"><joint type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.05"/></body><body name="c" pos="0.3 0.1 1.2" euler="0.4 0 0.5"><freejoint/><geom type="box" size="0.05 0.05 0.05"/></body></worldbody><equality><weld body1="a" body2="c" relpose="0.1 0 0 1 0 0 0"/></equality></mujoco>"#,
        [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    ),
    // a relpose whose quaternion is not unit: normalized
    (
        r#"<mujoco><worldbody><body name="a" pos="0 0 1" euler="0.1 0.2 0.3"><joint type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.05"/></body><body name="c" pos="0.3 0.1 1.2" euler="0.4 0 0.5"><freejoint/><geom type="box" size="0.05 0.05 0.05"/></body></worldbody><equality><weld body1="a" body2="c" relpose="0.1 0.2 0 2 0 0 0"/></equality></mujoco>"#,
        [0.0, 0.0, 0.0, 0.1, 0.2, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    ),
    // a relpose whose quaternion is all 0: made at qpos0, its position over
    (
        r#"<mujoco><worldbody><body name="a" pos="0 0 1" euler="0.1 0.2 0.3"><joint type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.05"/></body><body name="c" pos="0.3 0.1 1.2" euler="0.4 0 0.5"><freejoint/><geom type="box" size="0.05 0.05 0.05"/></body></worldbody><equality><weld body1="a" body2="c" relpose="0.1 0.2 0.3 0 0 0 0"/></equality></mujoco>"#,
        [
            0.0,
            0.0,
            0.0,
            0.29982197285251044,
            0.09878041041621292,
            0.2008711405673625,
            0.999993518920471,
            0.0026057385516249833,
            -0.0017635567004858253,
            0.0017498889732764835,
            1.0,
        ],
    ),
    // a torquescale
    (
        r#"<mujoco><worldbody><body name="a" pos="0 0 1" euler="0.1 0.2 0.3"><joint type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.05"/></body><body name="c" pos="0.3 0.1 1.2" euler="0.4 0 0.5"><freejoint/><geom type="box" size="0.05 0.05 0.05"/></body></worldbody><equality><weld body1="a" body2="c" torquescale="0.5"/></equality></mujoco>"#,
        [
            0.0,
            0.0,
            0.0,
            0.29982197285251044,
            0.09878041041621292,
            0.2008711405673625,
            0.999993518920471,
            0.0026057385516249833,
            -0.0017635567004858253,
            0.0017498889732764835,
            0.5,
        ],
    ),
    // a weld to the world
    (
        r#"<mujoco><worldbody><body name="a" pos="0 0 1" euler="0.1 0.2 0.3"><joint type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.05"/></body><body name="c" pos="0.3 0.1 1.2" euler="0.4 0 0.5"><freejoint/><geom type="box" size="0.05 0.05 0.05"/></body></worldbody><equality><weld body1="c"/></equality></mujoco>"#,
        [
            0.0,
            0.0,
            0.0,
            -0.3009343158621136,
            -0.10575298818778867,
            -1.1992726308166775,
            0.9999843884365196,
            -0.003490618186738092,
            1.5230791729859468e-05,
            -0.004363282701973242,
            1.0,
        ],
    ),
    // a connect: its point in body2's frame made at qpos0, data[10] MuJoCo's 1
    (
        r#"<mujoco><worldbody><body name="a" pos="0 0 1" euler="0.1 0.2 0.3"><joint type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.05"/></body><body name="c" pos="0.3 0.1 1.2" euler="0.4 0 0.5"><freejoint/><geom type="box" size="0.05 0.05 0.05"/></body></worldbody><equality><connect body1="a" body2="c" anchor="0.1 0 0"/></equality></mujoco>"#,
        [
            0.1,
            0.0,
            0.0,
            -0.20087462809503095,
            -0.09912288831478536,
            -0.1996487972147979,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ],
    ),
    // a weld's anchor in body2's frame
    (
        r#"<mujoco><worldbody><body name="a" pos="0 0 1" euler="0.1 0.2 0.3"><joint type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.05"/></body><body name="c" pos="0.3 0.1 1.2" euler="0.4 0 0.5"><freejoint/><geom type="box" size="0.05 0.05 0.05"/></body></worldbody><equality><weld body1="a" body2="c" anchor="0.05 0.02 -0.01"/></equality></mujoco>"#,
        [
            0.05,
            0.02,
            -0.01,
            0.3497863564185049,
            0.11900672071710364,
            0.19115225449527262,
            0.999993518920471,
            0.0026057385516249833,
            -0.0017635567004858253,
            0.0017498889732764835,
            1.0,
        ],
    ),
    // an anchor beside a relpose
    (
        r#"<mujoco><worldbody><body name="a" pos="0 0 1" euler="0.1 0.2 0.3"><joint type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.05"/></body><body name="c" pos="0.3 0.1 1.2" euler="0.4 0 0.5"><freejoint/><geom type="box" size="0.05 0.05 0.05"/></body></worldbody><equality><weld body1="a" body2="c" anchor="0.05 0.02 -0.01" relpose="0 0 0.1 0.9 0.1 0 0"/></equality></mujoco>"#,
        [
            0.05,
            0.02,
            -0.01,
            0.0,
            0.0,
            0.1,
            0.9938837346736189,
            0.11043152607484655,
            0.0,
            0.0,
            1.0,
        ],
    ),
    // a connect to the world
    (
        r#"<mujoco><worldbody><body name="a" pos="0 0 1" euler="0.1 0.2 0.3"><joint type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.05"/></body><body name="c" pos="0.3 0.1 1.2" euler="0.4 0 0.5"><freejoint/><geom type="box" size="0.05 0.05 0.05"/></body></worldbody><equality><connect body1="c" anchor="0 0 0.1"/></equality></mujoco>"#,
        [
            0.0,
            0.0,
            0.1,
            0.3,
            0.09930187397020385,
            1.2999975630705394,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ],
    ),
    // a tendon equality: its polycoef, data[10] MuJoCo's default 1
    (
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="hinge"/><geom type="box" size="0.1 0.05 0.05"/><body name="c" pos="0.3 0 0"><joint name="j2" type="hinge"/><geom type="box" size="0.05 0.05 0.05"/></body></body></worldbody><tendon><fixed name="t1"><joint joint="j1" coef="1"/></fixed><fixed name="t2"><joint joint="j2" coef="0.5"/></fixed></tendon><equality><tendon tendon1="t1" tendon2="t2" polycoef="0 2 0 0 0"/></equality></mujoco>"#,
        [0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    ),
    // a flexcomp's edge equality: MuJoCo's default row (`data[1]` and `data[10]` 1)
    (
        r#"<mujoco><worldbody><flexcomp name="f" type="grid" count="3 3 1" spacing="0.1 0.1 0.1" dim="2" radius="0.01" mass="1"><edge equality="true"/></flexcomp></worldbody></mujoco>"#,
        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    ),
];

/// 0 on macOS arm64, where MuJoCo's values were printed, and 1e-12 of the
/// row's largest entry elsewhere: the bodies' `euler` passes through `sin`
/// and `cos`, whose last bit is the platform's libm's.
const TOLERANCE: f64 = if cfg!(all(target_os = "macos", target_arch = "aarch64")) {
    0.0
} else {
    1e-12
};

/// Whether `ours` is `mujoco` within [`TOLERANCE`] of its largest entry, bit
/// for bit where that is 0; a value that is not a number fails.
fn close(ours: &[f64; 11], mujoco: &[f64; 11]) -> bool {
    if TOLERANCE == 0.0 {
        return ours
            .iter()
            .zip(mujoco)
            .all(|(a, b)| a.to_bits() == b.to_bits());
    }
    let scale = mujoco.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
    ours.iter()
        .zip(mujoco)
        .all(|(a, b)| (a - b).abs() <= TOLERANCE * scale)
}

#[test]
fn an_equalitys_data_is_mujocos() {
    let mut wrong = Vec::new();
    for (doc, mujoco) in CASES {
        let model = load_model(doc).unwrap();
        let ours = &model.eq_data[0];
        if !close(ours, mujoco) {
            wrong.push(format!("{ours:?} where MuJoCo's is {mujoco:?}: {doc}"));
        }
    }
    assert!(
        wrong.is_empty(),
        "{} of {}:\n{}",
        wrong.len(),
        CASES.len(),
        wrong.join("\n")
    );
}

/// A joint equality's `eq_data[10]` is its degree-10 coefficient here, 0
/// where none is written, where MuJoCo 3.5.0's row holds its default 1
/// there, which its coupling does not read (registry `D-POLYCOEF-DEGREE`);
/// the rest of the row is MuJoCo's (measured).
#[test]
fn a_joint_equalitys_tenth_entry_is_its_coefficient() {
    let model = load_model(r#"<mujoco><worldbody><body><joint name="j1" type="hinge" axis="0 1 0"/><geom size=".1" pos="0 0 -0.3"/><body pos="0 0 -0.5"><joint name="j2" type="hinge" axis="0 1 0"/><geom size=".1" pos="0 0 -0.3"/></body></body></worldbody><equality><joint joint1="j1" joint2="j2" polycoef="0 0.5 0 0 0"/></equality></mujoco>"#).unwrap();
    let mujoco = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0];
    let ours = model.eq_data[0];
    assert_eq!(ours[..10], mujoco[..10]);
    assert_eq!((ours[10], mujoco[10]), (0.0, 1.0));
}

/// A weld written with `site1` and `site2` keeps its two sites, with MuJoCo
/// 3.5.0's default row and its `torquescale` (measured: `eq_objtype` 6,
/// `eq_obj1id` 0, `eq_obj2id` 1, the row below): no anchor or relative pose
/// is made, the sites' positions and turns being the weld's.
#[test]
fn a_site_form_welds_row_is_mujocos_default() {
    let model = load_model(r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.05"/><site name="s1" pos="0.1 0 0"/></body><body name="c" pos="0.3 0.1 1.2"><freejoint/><geom type="box" size="0.05 0.05 0.05"/><site name="s2"/></body></worldbody><equality><weld site1="s1" site2="s2"/></equality></mujoco>"#).unwrap();
    let mujoco: [f64; 11] = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0];
    assert_eq!(model.eq_objtype, [MjObjectType::Site]);
    assert_eq!((model.eq_obj1id[0], model.eq_obj2id[0]), (0, 1));
    let bits = |row: &[f64; 11]| row.map(f64::to_bits);
    assert_eq!(bits(&model.eq_data[0]), bits(&mujoco));
}
