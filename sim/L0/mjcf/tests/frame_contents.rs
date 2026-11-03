//! A frame's contents and a composite's bodies where the document writes them
//! (book L24; ledger L344, L294).
//!
//! MuJoCo 3.5.0 adds each element to its body as it reads it, so a body's
//! geoms, sites and child bodies are numbered in the order the document
//! writes them, a frame's contents (a nested frame's too) and a composite's
//! bodies at the place the frame or composite stands. Every document was
//! measured on MuJoCo 3.5.0; each table row is its numbering.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
// `COMPOSED`'s poses are bit patterns, written as MuJoCo's values' raw bits.
#![allow(clippy::unreadable_literal)]

use nalgebra::Vector3;
use sim_core::Model;
use sim_mjcf::{
    ElementKind, MjcfError, MjcfFrame, MjcfGeom, MjcfGeomType, MjcfItem, Prefix, load_model,
    model_from_mjcf, parse_mjcf_str,
};

/// `(case, document, bodies, geoms, sites)`: each kind in id order as
/// `name@owner`, the owner a body's parent or the body holding a geom or a
/// site, `-` for an unnamed element, as MuJoCo 3.5.0 numbers them.
const ORDER: &[(&str, &str, &str, &str, &str)] = &[
    (
        "ord_geom_frame_geom",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom name="a" size=".1"/><frame pos="0 0 1"><geom name="f" size=".1"/></frame><geom name="c" size=".1"/></body></worldbody></mujoco>"#,
        "world@0 b@0",
        "a@1 f@1 c@1",
        "",
    ),
    (
        "ord_site_frame_site",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom size=".1"/><site name="a"/><frame pos="0 0 1"><site name="f"/></frame><site name="c"/></body></worldbody></mujoco>"#,
        "world@0 b@0",
        "-@1",
        "a@1 f@1 c@1",
    ),
    (
        "ord_two_frames",
        r#"<mujoco><worldbody><body name="b"><freejoint/><frame><geom name="f1" size=".1"/></frame><geom name="m" size=".1"/><frame><geom name="f2" size=".1"/></frame></body></worldbody></mujoco>"#,
        "world@0 b@0",
        "f1@1 m@1 f2@1",
        "",
    ),
    (
        "ord_in_frame_nested",
        r#"<mujoco><worldbody><body name="b"><freejoint/><frame><geom name="x" size=".1"/><frame pos="1 0 0"><geom name="y" size=".1"/></frame><geom name="z" size=".1"/></frame><geom name="w" size=".1"/></body></worldbody></mujoco>"#,
        "world@0 b@0",
        "x@1 y@1 z@1 w@1",
        "",
    ),
    (
        "ord_frame_first",
        r#"<mujoco><worldbody><body name="b"><freejoint/><frame><geom name="f" size=".1"/><body name="F"><geom size=".1"/></body></frame><geom name="g" size=".1"/><body name="B"><geom size=".1"/></body></body></worldbody></mujoco>"#,
        "world@0 b@0 F@1 B@1",
        "f@1 g@1 -@2 -@3",
        "",
    ),
    (
        "ord_mixed_kinds",
        r#"<mujoco><worldbody><body name="b"><freejoint/><site name="s0"/><frame><site name="s1"/><geom name="g1" size=".1"/><body name="F"><geom name="gF" size=".1"/></body><site name="s2"/></frame><geom name="g0" size=".1"/><body name="B"><geom name="gB" size=".1"/></body></body></worldbody></mujoco>"#,
        "world@0 b@0 F@1 B@1",
        "g1@1 g0@1 gF@2 gB@3",
        "s0@1 s1@1 s2@1",
    ),
    (
        "ord_body_frame_body",
        r#"<mujoco><worldbody><body name="p"><freejoint/><geom size=".1"/><body name="A"><geom size=".1"/><body name="A1"><geom size=".1"/></body></body><frame pos="0 0 1"><body name="F"><geom size=".1"/></body></frame><body name="C"><geom size=".1"/></body></body></worldbody></mujoco>"#,
        "world@0 p@0 A@1 A1@2 F@1 C@1",
        "-@1 -@2 -@3 -@4 -@5",
        "",
    ),
    (
        "ord_frame_body_subtree",
        r#"<mujoco><worldbody><body name="p"><freejoint/><geom size=".1"/><frame><body name="F"><geom size=".1"/><body name="F1"><geom size=".1"/></body></body></frame><body name="C"><geom size=".1"/><body name="C1"><geom size=".1"/></body></body></body></worldbody></mujoco>"#,
        "world@0 p@0 F@1 F1@2 C@1 C1@4",
        "-@1 -@2 -@3 -@4 -@5",
        "",
    ),
    (
        "ord_world_body_frame_body",
        r#"<mujoco><worldbody><body name="A"><geom size=".1"/></body><frame pos="0 0 1"><body name="F"><geom size=".1"/></body></frame><body name="C"><geom size=".1"/></body></worldbody></mujoco>"#,
        "world@0 A@0 F@0 C@0",
        "-@1 -@2 -@3",
        "",
    ),
    (
        "ord_world_geom_frame_geom",
        r#"<mujoco><worldbody><geom name="a" type="plane" size="1 1 1"/><frame pos="0 0 1"><geom name="f" size=".1"/></frame><geom name="c" size=".1"/></worldbody></mujoco>"#,
        "world@0",
        "a@0 f@0 c@0",
        "",
    ),
    (
        "cls_frame_childclass",
        r#"<mujoco><default><default class="k"><geom size=".3"/></default></default><worldbody><body name="b"><freejoint/><frame childclass="k"><geom name="g"/></frame><geom name="h" size=".2"/></body></worldbody></mujoco>"#,
        "world@0 b@0",
        "g@1 h@1",
        "",
    ),
    (
        "ord_body_comp_body",
        r#"<mujoco><worldbody><body name="A"><geom size=".1"/></body><composite type="cable" count="4 1 1" curve="s" size="1" prefix="c"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite><body name="C"><geom size=".1"/></body></worldbody></mujoco>"#,
        "world@0 A@0 cB_first@0 cB_1@2 cB_last@3 C@0",
        "-@1 cG0@2 cG1@3 cG2@4 -@5",
        "cS_first@2 cS_last@4",
    ),
    (
        "ord_geom_comp_in_body",
        r#"<mujoco><worldbody><body name="p" pos="0 0 1"><freejoint/><geom name="a" size=".1"/><composite type="cable" count="4 1 1" curve="s" size="1" prefix="c"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite><geom name="z" size=".1"/><body name="C"><geom size=".1"/></body></body></worldbody></mujoco>"#,
        "world@0 p@0 cB_first@1 cB_1@2 cB_last@3 C@1",
        "a@1 z@1 cG0@2 cG1@3 cG2@4 -@5",
        "cS_first@2 cS_last@4",
    ),
    (
        "ord_frame_bodies_around_nested",
        r#"<mujoco><worldbody><body name="p"><freejoint/><geom size=".1"/><frame pos="0 0 1"><body name="A"><geom name="gA" size=".1"/></body><frame pos="1 0 0"><body name="B"><geom name="gB" size=".1"/></body></frame><body name="C"><geom name="gC" size=".1"/></body></frame></body></worldbody></mujoco>"#,
        "world@0 p@0 A@1 B@1 C@1",
        "-@1 gA@2 gB@3 gC@4",
        "",
    ),
];

/// One kind's elements in id order, as `ORDER` writes them.
fn listed(names: &[Option<String>], owners: &[usize]) -> String {
    names
        .iter()
        .zip(owners)
        .map(|(name, owner)| format!("{}@{owner}", name.as_deref().unwrap_or("-")))
        .collect::<Vec<_>>()
        .join(" ")
}

fn numbering(model: &Model) -> [String; 3] {
    [
        listed(&model.body_name, &model.body_parent),
        listed(&model.geom_name, &model.geom_body),
        listed(&model.site_name, &model.site_body),
    ]
}

#[test]
fn a_frames_contents_and_a_composites_bodies_take_their_place() {
    for &(case, xml, bodies, geoms, sites) in ORDER {
        let model = load_model(xml).unwrap_or_else(|e| panic!("{case}: {e}"));
        assert_eq!(numbering(&model), [bodies, geoms, sites], "{case}");
    }
}

/// A sphere geom of radius 0.1 named `name`.
fn sphere(name: &str) -> MjcfGeom {
    MjcfGeom {
        name: Some(name.to_string()),
        geom_type: Some(MjcfGeomType::Sphere),
        size: Prefix::new(&[0.1]),
        ..Default::default()
    }
}

#[test]
fn a_frame_built_in_code_takes_the_place_its_items_give() {
    // A body holding geoms `a` and `c`, and a frame built in code holding
    // `f`: with a `Frame` entry between theirs it is numbered between them;
    // with none it comes after the body's own, as `MjcfBody::items` says.
    let mut model = parse_mjcf_str(
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom name="a" size=".1"/><geom name="c" size=".1"/></body></worldbody></mujoco>"#,
    )
    .unwrap();
    let frame = MjcfFrame {
        pos: Vector3::new(0.0, 0.0, 1.0),
        geoms: vec![sphere("f")],
        items: vec![MjcfItem::Geom],
        ..Default::default()
    };
    let body = &mut model.worldbody.children[0];
    body.frames.push(frame);
    let unlisted = model_from_mjcf(&model, None).unwrap();
    assert_eq!(numbering(&unlisted)[1], "a@1 c@1 f@1");

    let body = &mut model.worldbody.children[0];
    assert_eq!(
        body.items,
        [MjcfItem::Joint, MjcfItem::Geom, MjcfItem::Geom]
    );
    body.items.insert(2, MjcfItem::Frame);
    let listed = model_from_mjcf(&model, None).unwrap();
    assert_eq!(numbering(&listed)[1], "a@1 f@1 c@1");
    assert_eq!(listed.geom_pos[1], Vector3::new(0.0, 0.0, 1.0));
}

/// `(case, document, bodies, geoms, sites)` as `ORDER`: a cable written in a
/// frame, which MuJoCo 3.5.0 makes when it reads it, its first body one of
/// the frame's bodies (`user_composite.cc:381-383`).
const COMPOSITES_IN_FRAMES: &[(&str, &str, &str, &str, &str)] = &[
    (
        "comp_in_world_frame",
        r#"<mujoco><worldbody><frame pos="0 1 0"><composite type="cable" count="4 1 1" curve="s" size="1" prefix="c"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite></frame></worldbody></mujoco>"#,
        "world@0 cB_first@0 cB_1@1 cB_last@2",
        "cG0@1 cG1@2 cG2@3",
        "cS_first@1 cS_last@3",
    ),
    (
        "comp_in_turned_world_frame",
        r#"<mujoco><compiler angle="degree"/><worldbody><frame pos="0 1 0" euler="0 0 90"><composite type="cable" count="4 1 1" curve="s" size="1" prefix="c"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite></frame></worldbody></mujoco>"#,
        "world@0 cB_first@0 cB_1@1 cB_last@2",
        "cG0@1 cG1@2 cG2@3",
        "cS_first@1 cS_last@3",
    ),
    (
        "comp_in_body_frame",
        r#"<mujoco><worldbody><body name="p" pos="0 0 1"><geom size=".1"/><frame pos="0 1 0"><composite type="cable" count="4 1 1" curve="s" size="1" prefix="c"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite></frame></body></worldbody></mujoco>"#,
        "world@0 p@0 cB_first@1 cB_1@2 cB_last@3",
        "-@1 cG0@2 cG1@3 cG2@4",
        "cS_first@2 cS_last@4",
    ),
    (
        "comp_in_nested_frame",
        r#"<mujoco><compiler angle="degree"/><worldbody><frame pos="1 0 0"><frame pos="0 1 0" euler="0 0 45"><composite type="cable" count="4 1 1" curve="s" size="1" prefix="c"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite></frame></frame></worldbody></mujoco>"#,
        "world@0 cB_first@0 cB_1@1 cB_last@2",
        "cG0@1 cG1@2 cG2@3",
        "cS_first@1 cS_last@3",
    ),
    (
        "comp_frame_order",
        r#"<mujoco><worldbody><body name="A"><geom size=".1"/></body><frame pos="0 1 0"><composite type="cable" count="4 1 1" curve="s" size="1" prefix="c"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite></frame><body name="C"><geom size=".1"/></body></worldbody></mujoco>"#,
        "world@0 A@0 cB_first@0 cB_1@2 cB_last@3 C@0",
        "-@1 cG0@2 cG1@3 cG2@4 -@5",
        "cS_first@2 cS_last@4",
    ),
    (
        "comp_two_prefixes_frame",
        r#"<mujoco><worldbody><composite type="cable" count="4 1 1" curve="s" size="1" prefix="c"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite><frame pos="0 1 0"><composite type="cable" count="4 1 1" curve="s" size="1" prefix="d"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite></frame></worldbody></mujoco>"#,
        "world@0 cB_first@0 cB_1@1 cB_last@2 dB_first@0 dB_1@4 dB_last@5",
        "cG0@1 cG1@2 cG2@3 dG0@4 dG1@5 dG2@6",
        "cS_first@1 cS_last@3 dS_first@4 dS_last@6",
    ),
    (
        "comp_then_body_in_frame",
        r#"<mujoco><worldbody><frame pos="0 1 0"><composite type="cable" count="4 1 1" curve="s" size="1" prefix="c"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite><body name="X"><geom name="gX" size=".1"/></body></frame></worldbody></mujoco>"#,
        "world@0 cB_first@0 cB_1@1 cB_last@2 X@0",
        "cG0@1 cG1@2 cG2@3 gX@4",
        "cS_first@1 cS_last@3",
    ),
];

#[test]
fn a_composite_in_a_frame_is_one_of_the_frames_bodies() {
    for &(case, xml, bodies, geoms, sites) in COMPOSITES_IN_FRAMES {
        let model = load_model(xml).unwrap_or_else(|e| panic!("{case}: {e}"));
        assert_eq!(numbering(&model), [bodies, geoms, sites], "{case}");
    }
}

#[test]
fn a_frame_composes_a_cables_first_body_and_no_other() {
    // MuJoCo 3.5.0's cB_first: the frames' position, and their turn about z
    // (90 degrees; 45 in the nested frame); cB_1 as written in cB_first.
    for (case, pos, turn) in [
        ("comp_in_turned_world_frame", [0.0, 1.0, 0.0], 90.0_f64),
        ("comp_in_nested_frame", [1.0, 1.0, 0.0], 45.0),
    ] {
        let xml = COMPOSITES_IN_FRAMES
            .iter()
            .find(|row| row.0 == case)
            .unwrap()
            .1;
        let model = load_model(xml).unwrap();
        let half = turn.to_radians() / 2.0;
        let first = model.body_name_to_id["cB_first"];
        assert!(
            (model.body_pos[first] - Vector3::from(pos)).norm() < 1e-12,
            "{case}"
        );
        let q = model.body_quat[first];
        let want = [half.cos(), 0.0, 0.0, half.sin()];
        assert!(
            [q.w, q.i, q.j, q.k]
                .iter()
                .zip(want)
                .all(|(a, b)| (a - b).abs() < 1e-12),
            "{case}: {q:?}"
        );
        let next = model.body_name_to_id["cB_1"];
        assert_eq!(
            model.body_quat[next],
            nalgebra::UnitQuaternion::identity(),
            "{case}"
        );
    }
}

#[test]
fn two_cables_of_one_prefix_one_in_a_frame_repeat_a_body_name() {
    // MuJoCo 3.5.0: "repeated name 'cB_1' in body", the repeat that sorts
    // first; this crate names the first it meets ('cB_first', ledger L353).
    let xml = r#"<mujoco><worldbody><composite type="cable" count="4 1 1" curve="s" size="1" prefix="c"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite><frame pos="0 1 0"><composite type="cable" count="4 1 1" curve="s" size="1" prefix="c"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite></frame></worldbody></mujoco>"#;
    let err = load_model(xml).expect_err("MuJoCo refuses it");
    assert!(
        matches!(&err, MjcfError::DuplicateName { kind: ElementKind::Body, name, .. } if name == "cB_first"),
        "{err}"
    );
}

/// One composed element, `(kind, id, name, position, quaternion)`: a body
/// (`'B'`), geom (`'G'`) or site (`'S'`) by its id, and its position and
/// quaternion (w, x, y, z) as f64 bits.
type Composed = (char, usize, &'static str, [u64; 3], [u64; 4]);

/// `(case, document, elements)`: every body but the world, every geom and
/// every site, posed as MuJoCo 3.5.0 composes them through their frames
/// (`mjuu_frameaccum`; measured, ledger L255).
const COMPOSED: &[(&str, &str, &[Composed])] = &[
    (
        "bits_turned_frame_geom",
        r#"<mujoco><compiler angle="degree"/><worldbody><body name="b" pos="0.1 0.2 0.3"><freejoint/><frame pos="0.3 -0.7 1.1" euler="10 20 30"><geom name="g" size=".1" pos="0.7 0.1 -0.3" euler="5 -15 25"/><site name="s" pos="0.2 0.3 0.4" euler="30 10 -20"/></frame></body></worldbody></mujoco>"#,
        &[
            (
                'B',
                1,
                "b",
                [0x3fb999999999999a, 0x3fc999999999999a, 0x3fd3333333333333],
                [
                    0x3ff0000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                ],
            ),
            (
                'G',
                0,
                "g",
                [0x3fe70acb6c5c9a52, 0xbfc811ccf6cc8128, 0x3fe6bf39383bba17],
                [
                    0x3fec05abf1452832,
                    0x3fca15cfc7c62e76,
                    0xbf87c39bbd091bb3,
                    0x3fdc01b61d024c2a,
                ],
            ),
            (
                'S',
                0,
                "s",
                [0x3fdd59ed474467b0, 0xbfda3614c7775bcf, 0x3ff865b4b5082318],
                [
                    0x3fec6936dbddd3a4,
                    0x3fd2b0e2e918e377,
                    0x3fd5d4bd747ad218,
                    0x3fb9b60efccd6fae,
                ],
            ),
        ],
    ),
    (
        "bits_turned_frame_body",
        r#"<mujoco><compiler angle="degree"/><worldbody><body name="b"><freejoint/><geom size=".1"/><frame pos="0.3 -0.7 1.1" euler="10 20 30"><body name="c" pos="0.7 0.1 -0.3" euler="5 -15 25"><joint name="j" axis="0.3 0.5 0.7"/><geom name="g" size=".1" pos="0.1 0.1 0.1"/></body></frame></body></worldbody></mujoco>"#,
        &[
            (
                'B',
                1,
                "b",
                [0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3ff0000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                ],
            ),
            (
                'B',
                2,
                "c",
                [0x3fe70acb6c5c9a52, 0xbfc811ccf6cc8128, 0x3fe6bf39383bba17],
                [
                    0x3fec05abf1452832,
                    0x3fca15cfc7c62e76,
                    0xbf87c39bbd091bb3,
                    0x3fdc01b61d024c2a,
                ],
            ),
            (
                'G',
                0,
                "-",
                [0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3ff0000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                ],
            ),
            (
                'G',
                1,
                "g",
                [0x3fb999999999999a, 0x3fb999999999999a, 0x3fb999999999999a],
                [
                    0x3ff0000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                ],
            ),
        ],
    ),
    (
        "bits_nested_turned",
        r#"<mujoco><compiler angle="degree"/><worldbody><body name="b"><freejoint/><frame pos="0.3 0.1 0" euler="13 17 19"><frame pos="-0.2 0.5 0.4" euler="23 29 31"><frame pos="0.1 0.1 0.1" axisangle="0.3 0.4 0.5 37"><geom name="g" size=".1" pos="0.7 0.1 -0.3" quat="0.9 0.1 0.2 0.3"/><site name="s" pos="0.2 0.3 0.4"/></frame></frame></frame></body></worldbody></mujoco>"#,
        &[
            (
                'B',
                1,
                "b",
                [0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3ff0000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                ],
            ),
            (
                'G',
                0,
                "g",
                [0xbfcb87a30563a950, 0x3ff25838be23b2cb, 0x3fe93dc5edccc4f0],
                [
                    0x3fc497f416727ccd,
                    0x3fdd005c109e3635,
                    0x3fd98aaebf16c958,
                    0x3fe8fb8995a87cbd,
                ],
            ),
            (
                'S',
                0,
                "s",
                [0x3fdbadf7c99b2eec, 0x3fe4af5655784dd5, 0x3ff1f092548b989f],
                [
                    0x3fe08d39d89c9400,
                    0x3fdc1d2c34102c4e,
                    0x3fd945a25a7db178,
                    0x3fe3d18ceefa325a,
                ],
            ),
        ],
    ),
    (
        "bits_unit_quat_frame",
        r#"<mujoco><worldbody><body name="b"><freejoint/><frame pos="0.3 -0.7 1.1" quat="0.2222222222222222 0.4444444444444444 0.5555555555555556 0.6666666666666666"><geom name="g" size=".1" pos="0.7 0.1 -0.3" quat="0.2222222222222222 0.4444444444444444 0.5555555555555556 0.6666666666666666"/></frame></body></worldbody></mujoco>"#,
        &[
            (
                'B',
                1,
                "b",
                [0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3ff0000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                ],
            ),
            (
                'G',
                0,
                "g",
                [0xbfd254b3841bcff5, 0xbfd5a63df218050e, 0x3ff708379febc5d9],
                [
                    0xbfecd6e9e06522c4,
                    0x3fc948b0fcd6e9e0,
                    0x3fcf9add3c0ca458,
                    0x3fd2f684bda12f68,
                ],
            ),
        ],
    ),
    (
        "bits_xyaxes_frame",
        r#"<mujoco><worldbody><body name="b"><freejoint/><frame xyaxes="0.6 0.8 0 -0.8 0.6 0.1"><geom name="g" size=".1" pos="1 2 3" zaxis="0.1 0.2 0.9"/><site name="s" pos="0.5 0 0" xyaxes="0 1 0 -1 0 0.2"/></frame></body></worldbody></mujoco>"#,
        &[
            (
                'B',
                1,
                "b",
                [0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3ff0000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                ],
            ),
            (
                'G',
                0,
                "g",
                [0xbfe81aa0f524b9aa, 0x3ffd09fc5bedc59f, 0x4009791363068b54],
                [
                    0x3fec7d9c05305130,
                    0xbfb3bbcc06315ff4,
                    0x3f96a4a417ba857c,
                    0x3fdcaf2c90d551cb,
                ],
            ),
            (
                'S',
                0,
                "s",
                [0x3fd3333333333333, 0x3fd999999999999b, 0x0000000000000000],
                [
                    0x3fd3d12a7c1fc8ad,
                    0x3fb401941b38dbe6,
                    0x3fb3e36f3a0e8b17,
                    0x3fee38e7f5d72fc8,
                ],
            ),
        ],
    ),
    (
        "bits_zaxis_frame",
        r#"<mujoco><worldbody><body name="b"><freejoint/><frame pos="1 2 3" zaxis="0.3 -0.4 0.8"><geom name="g" size=".1" pos="0.1 0.2 0.3"/><body name="c" pos="0.4 0.5 0.6" zaxis="1 1 1"><joint/><geom size=".1"/></body></frame></body></worldbody></mujoco>"#,
        &[
            (
                'B',
                1,
                "b",
                [0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3ff0000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                ],
            ),
            (
                'B',
                2,
                "c",
                [0x3ff9afaf4ad97bce, 0x4001cf24bd089c66, 0x400cbfb0bbff7c5c],
                [
                    0x3febe400c4bf8429,
                    0xbfbdd9804945e05c,
                    0x3fdd661b1d4cbe88,
                    0x3fc00f0009a9062c,
                ],
            ),
            (
                'G',
                0,
                "g",
                [0x3ff345b6ab8f486a, 0x40007c30e2f5cfb9, 0x400a758dcf1204f9],
                [
                    0x3feec28cbb0319e1,
                    0x3fcc3ada0709eacf,
                    0x3fc52c238547701b,
                    0x0000000000000000,
                ],
            ),
            (
                'G',
                1,
                "-",
                [0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3ff0000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                ],
            ),
        ],
    ),
    (
        "bits_world_frame_body",
        r#"<mujoco><compiler angle="degree"/><worldbody><frame pos="0.3 -0.7 1.1" euler="10 20 30"><body name="c" pos="0.7 0.1 -0.3" euler="5 -15 25"><freejoint/><geom name="g" size=".1"/></body><geom name="wg" size=".1" pos="0.3 0.3 0.3" euler="1 2 3"/></frame></worldbody></mujoco>"#,
        &[
            (
                'B',
                1,
                "c",
                [0x3fe70acb6c5c9a52, 0xbfc811ccf6cc8128, 0x3fe6bf39383bba17],
                [
                    0x3fec05abf1452832,
                    0x3fca15cfc7c62e76,
                    0xbf87c39bbd091bb3,
                    0x3fdc01b61d024c2a,
                ],
            ),
            (
                'G',
                0,
                "wg",
                [0x3fe02f718f57f5c2, 0xbfd5afb59782a8d3, 0x3ff696bd22e4fb3d],
                [
                    0x3fedd6cffea9eae5,
                    0x3fc156df1585580c,
                    0x3fc47fe0150263d2,
                    0x3fd2d293ce9db9e9,
                ],
            ),
            (
                'G',
                1,
                "g",
                [0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3ff0000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                ],
            ),
        ],
    ),
    (
        "bits_unnormalised_frame_quat",
        r#"<mujoco><worldbody><body name="b"><freejoint/><frame quat="1 2 3 4"><geom name="g" size=".1" pos="1 1 1" quat="4 3 2 1"/><site name="s" pos="1 0 0" quat="2 0 0 1"/></frame></body></worldbody></mujoco>"#,
        &[
            (
                'B',
                1,
                "b",
                [0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3ff0000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                ],
            ),
            (
                'G',
                0,
                "g",
                [0x3fc9999999999998, 0x3fefffffffffffff, 0x3ff6666666666666],
                [
                    0xbfd9999999999999,
                    0x3fc9999999999998,
                    0x3fe9999999999999,
                    0x3fd9999999999998,
                ],
            ),
            (
                'S',
                0,
                "s",
                [0xbfe5555555555554, 0x3fe5555555555554, 0x3fd5555555555554],
                [
                    0xbfc4e6fdf33cf031,
                    0x3fe24a1e34d5522b,
                    0x3fd4e6fdf33cf031,
                    0x3fe783ddb1a48e37,
                ],
            ),
        ],
    ),
    (
        "bits_fromto_in_turned_frame",
        r#"<mujoco><compiler angle="degree"/><worldbody><body name="b"><freejoint/><frame pos="0.1 0.2 0.3" euler="10 20 30"><geom name="g" type="capsule" size=".05" fromto="0 0 0 0.3 0.4 0.5"/></frame></body></worldbody></mujoco>"#,
        &[
            (
                'B',
                1,
                "b",
                [0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3ff0000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                ],
            ),
            (
                'G',
                0,
                "g",
                [0x3fcb576c2f5ce964, 0x3fd9f257585935f6, 0x3fe20f6b382143a4],
                [
                    0x3fd636a5ff5cdedc,
                    0x3feca5a52df55cd3,
                    0xbfd13ab76ea9b2e4,
                    0xbfb3393a8c09305a,
                ],
            ),
        ],
    ),
    (
        "bits_frame_radian",
        r#"<mujoco><worldbody><body name="b"><freejoint/><frame euler="0.1 0.2 0.3"><geom name="g" size=".1" pos="1 0 0" euler="0.3 0.2 0.1"/></frame></body></worldbody></mujoco>"#,
        &[
            (
                'B',
                1,
                "b",
                [0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3ff0000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                ],
            ),
            (
                'G',
                0,
                "g",
                [0x3fefffd679f32c1b, 0x3f7578af92deb8eb, 0xbf6c85267bd4d147],
                [
                    0x3fefffd99d5d207a,
                    0x3f6c9ec8bc3410d5,
                    0x3f6c9b909f6604f1,
                    0x3f6c9ec8bc3410d5,
                ],
            ),
        ],
    ),
    (
        "comp_in_turned_world_frame",
        r#"<mujoco><compiler angle="degree"/><worldbody><frame pos="0 1 0" euler="0 0 90"><composite type="cable" count="4 1 1" curve="s" size="1" prefix="c"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite></frame></worldbody></mujoco>"#,
        &[
            (
                'B',
                1,
                "cB_first",
                [0x0000000000000000, 0x3ff0000000000000, 0x0000000000000000],
                [
                    0x3fe6a09e667f3bcd,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3fe6a09e667f3bcc,
                ],
            ),
            (
                'B',
                2,
                "cB_1",
                [0x3fd5555560000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3ff0000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                ],
            ),
            (
                'B',
                3,
                "cB_last",
                [0x3fd5555560000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3ff0000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                ],
            ),
            (
                'G',
                0,
                "cG0",
                [0x3fc5555560000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3fe6a09e667f3bcd,
                    0x0000000000000000,
                    0xbfe6a09e667f3bcc,
                    0x0000000000000000,
                ],
            ),
            (
                'G',
                1,
                "cG1",
                [0x3fc5555560000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3fe6a09e667f3bcd,
                    0x0000000000000000,
                    0xbfe6a09e667f3bcc,
                    0x0000000000000000,
                ],
            ),
            (
                'G',
                2,
                "cG2",
                [0x3fc5555540000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3fe6a09e667f3bcd,
                    0x0000000000000000,
                    0xbfe6a09e667f3bcc,
                    0x0000000000000000,
                ],
            ),
            (
                'S',
                0,
                "cS_first",
                [0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3ff0000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                ],
            ),
            (
                'S',
                1,
                "cS_last",
                [0x3fd5555540000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3ff0000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                ],
            ),
        ],
    ),
    (
        "comp_in_nested_frame",
        r#"<mujoco><compiler angle="degree"/><worldbody><frame pos="1 0 0"><frame pos="0 1 0" euler="0 0 45"><composite type="cable" count="4 1 1" curve="s" size="1" prefix="c"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite></frame></frame></worldbody></mujoco>"#,
        &[
            (
                'B',
                1,
                "cB_first",
                [0x3ff0000000000000, 0x3ff0000000000000, 0x0000000000000000],
                [
                    0x3fed906bcf328d46,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3fd87de2a6aea963,
                ],
            ),
            (
                'B',
                2,
                "cB_1",
                [0x3fd5555560000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3ff0000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                ],
            ),
            (
                'B',
                3,
                "cB_last",
                [0x3fd5555560000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3ff0000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                ],
            ),
            (
                'G',
                0,
                "cG0",
                [0x3fc5555560000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3fe6a09e667f3bcd,
                    0x0000000000000000,
                    0xbfe6a09e667f3bcc,
                    0x0000000000000000,
                ],
            ),
            (
                'G',
                1,
                "cG1",
                [0x3fc5555560000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3fe6a09e667f3bcd,
                    0x0000000000000000,
                    0xbfe6a09e667f3bcc,
                    0x0000000000000000,
                ],
            ),
            (
                'G',
                2,
                "cG2",
                [0x3fc5555540000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3fe6a09e667f3bcd,
                    0x0000000000000000,
                    0xbfe6a09e667f3bcc,
                    0x0000000000000000,
                ],
            ),
            (
                'S',
                0,
                "cS_first",
                [0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3ff0000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                ],
            ),
            (
                'S',
                1,
                "cS_last",
                [0x3fd5555540000000, 0x0000000000000000, 0x0000000000000000],
                [
                    0x3ff0000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                ],
            ),
        ],
    ),
];

#[test]
fn a_frame_composes_a_pose_as_mujocos_frameaccum() {
    // Bit for bit where the goldens were measured (macOS arm64); elsewhere
    // the libm that turns an euler angle may round otherwise, so within
    // 1e-12, which a NaN fails.
    let here = cfg!(all(target_os = "macos", target_arch = "aarch64"));
    let mut differ = Vec::new();
    for &(case, xml, elements) in COMPOSED {
        let model = load_model(xml).unwrap_or_else(|e| panic!("{case}: {e}"));
        for &(kind, id, name, pos, quat) in elements {
            let (p, q) = match kind {
                'B' => (model.body_pos[id], model.body_quat[id]),
                'G' => (model.geom_pos[id], model.geom_quat[id]),
                _ => (model.site_pos[id], model.site_quat[id]),
            };
            let got = [p.x, p.y, p.z, q.w, q.i, q.j, q.k];
            let want = pos.iter().chain(&quat).map(|b| f64::from_bits(*b));
            let same = got.iter().zip(want).all(|(a, b)| {
                if here {
                    a.to_bits() == b.to_bits()
                } else {
                    (a - b).abs() <= 1e-12
                }
            });
            if !same {
                differ.push(format!("{case} {kind} {name}: {got:?}"));
            }
        }
    }
    assert!(differ.is_empty(), "{differ:#?}");
}

/// A5 §3.7's frame cases (book L24's must-fail): a fault inside a frame,
/// refused as MuJoCo 3.5.0 refuses it. `(case, document, this crate's words,
/// MuJoCo's)`. Each was refused at this pull request's parent (measured).
/// The seventh, a geom's condim 2 in a frame, loads here and is book L25's.
const REFUSED_IN_FRAMES: &[(&str, &str, &str, &str)] = &[
    (
        "s37_nan_size",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom size=".1"/><frame><geom name="g" size="nan"/></frame></body></worldbody></mujoco>"#,
        "size on <geom> is not finite (NaN)",
        "nan size in geom",
    ),
    (
        "s37_zero_size",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom size=".1"/><frame><geom name="g" size="0"/></frame></body></worldbody></mujoco>"#,
        "size 0 must be positive",
        "size 0 must be positive in geom",
    ),
    (
        "s37_dup_joint",
        r#"<mujoco><worldbody><body name="b"><joint name="j"/><geom size=".1"/><frame><body name="c"><joint name="j"/><geom size=".1"/></body></frame></body></worldbody></mujoco>"#,
        "repeated name 'j' in joint",
        "repeated name 'j' in joint",
    ),
    (
        "s37_backwards_range",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom size=".1"/><frame><body name="c"><joint name="j" limited="true" range="1 -1"/><geom size=".1"/></body></frame></body></worldbody></mujoco>"#,
        "range[0] should be smaller than range[1] in joint",
        "range[0] should be smaller than range[1] in joint",
    ),
    (
        "s37_massless_moving",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom size=".1"/><frame><body name="c"><joint name="j"/></body></frame></body></worldbody></mujoco>"#,
        "mass and inertia of moving bodies must be larger than mjMINVAL",
        "mass and inertia of moving bodies must be larger than mjMINVAL",
    ),
    (
        "s37_free_below_top",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom size=".1"/><frame><body name="c"><freejoint/><geom size=".1"/></body></frame></body></worldbody></mujoco>"#,
        "free joint can only be used on top level",
        "free joint can only be used on top level",
    ),
];

#[test]
fn a_fault_inside_a_frame_is_refused_as_outside_one() {
    for &(case, xml, ours, _mujoco) in REFUSED_IN_FRAMES {
        let err = load_model(xml).expect_err(case).to_string();
        assert!(err.contains(ours), "{case}: {err}");
    }
}
