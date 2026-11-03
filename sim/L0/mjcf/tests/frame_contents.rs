//! A frame's contents and a composite's bodies where the document writes them
//! (book L24; ledger L344, L294).
//!
//! MuJoCo 3.5.0 adds each element to its body as it reads it, so a body's
//! geoms, sites and child bodies are numbered in the order the document
//! writes them, a frame's contents (a nested frame's too) and a composite's
//! bodies at the place the frame or composite stands. Every document was
//! measured on MuJoCo 3.5.0; each table row is its numbering.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use nalgebra::Vector3;
use sim_core::Model;
use sim_mjcf::{
    MjcfFrame, MjcfGeom, MjcfGeomType, MjcfItem, Prefix, load_model, model_from_mjcf,
    parse_mjcf_str,
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
