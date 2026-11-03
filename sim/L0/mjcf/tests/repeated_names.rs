//! A repeated name is refused for every kind this crate reads, as MuJoCo
//! 3.5.0 refuses it (book L23): a body named `world`, which is the world's
//! name; a geom, site, tendon, sensor, keyframe, pair, exclusion, equality,
//! frame, flex, skin or plugin instance; names in a frame, names a
//! `discardvisual` pass would drop, and names a composite makes. Two kinds
//! may share a name, and an empty name is no name. Where a document repeats
//! two names, the one MuJoCo's reader meets first is named.
//!
//! Every document was measured on MuJoCo 3.5.0, which refuses or loads it as
//! its row says; the expected name and kind are MuJoCo's. At the parent each
//! refused document but `order_actuator_then_sensor` loaded or named another
//! repeat (main refused a repeated body, joint, actuator, mesh and height
//! field only).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_mjcf::{ElementKind, MjcfError, load_model};

/// `(case, document, the repeated name's kind, the name)`, refused by MuJoCo.
const REFUSED: &[(&str, &str, ElementKind, &str)] = &[
    (
        "dup_body_world",
        r#"<mujoco><worldbody><body name="world" pos="0 0 0"><joint type="hinge"/><geom type="sphere" size="0.1" mass="1"/></body></worldbody></mujoco>"#,
        ElementKind::Body,
        "world",
    ),
    (
        "dup_discardvisual",
        r#"<mujoco><compiler discardvisual="true"/><worldbody><body name="a"><joint/><geom name="g" size="0.1" mass="1"/></body><body name="b" pos="1 0 0"><joint/><geom size="0.1" mass="1"/><geom name="g" size="0.1" contype="0" conaffinity="0" group="1"/></body></worldbody></mujoco>"#,
        ElementKind::Geom,
        "g",
    ),
    (
        "dup_equality_connect_weld",
        r#"<mujoco><worldbody><body name="a"><joint name="j1"/><geom type="sphere" size="0.1" mass="1"/><site name="s1"/></body><body name="b" pos="1 0 0"><joint name="j2"/><geom type="sphere" size="0.1" mass="1"/><site name="s2"/></body></worldbody><equality><connect name="e" body1="a" body2="b" anchor="0 0 0"/><weld name="e" body1="a" body2="b"/></equality></mujoco>"#,
        ElementKind::Equality,
        "e",
    ),
    (
        "dup_equality_flex_order",
        r#"<mujoco><worldbody><body name="a"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="b" pos="1 0 0"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="c" pos="0 1 0"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body></worldbody><equality><weld name="z" body1="a" body2="b"/><connect name="y" body1="a" body2="c" anchor="0 0 0"/><weld name="y" body1="b" body2="c"/><connect name="z" body1="b" body2="a" anchor="0 0 0"/></equality></mujoco>"#,
        ElementKind::Equality,
        "y",
    ),
    (
        "dup_exclude",
        r#"<mujoco><worldbody><body name="a"><joint/><geom name="g1" size="0.1" mass="1"/><geom name="g2" size="0.1" mass="1" pos="0 0 1"/></body><body name="b" pos="1 0 0"><joint/><geom name="g3" size="0.1" mass="1"/></body></worldbody><contact><exclude name="e" body1="a" body2="b"/><exclude name="e" body1="b" body2="world"/></contact></mujoco>"#,
        ElementKind::Exclude,
        "e",
    ),
    (
        "dup_flex",
        r#"<mujoco><worldbody><body name="a"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><geom type="sphere" size="0.1" mass="1"/></body><body name="b" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><geom type="sphere" size="0.1" mass="1"/></body><body name="c" pos="0 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><geom type="sphere" size="0.1" mass="1"/></body></worldbody><deformable><flex name="f" dim="1" body="a b" vertex="0 0 0 0 0 0" element="0 1"/><flex name="f" dim="1" body="b c" vertex="0 0 0 0 0 0" element="0 1"/></deformable></mujoco>"#,
        ElementKind::Flex,
        "f",
    ),
    (
        "dup_frame",
        r#"<mujoco><worldbody><frame name="f"><body name="a" pos="0 0 0"><joint type="hinge"/><geom type="sphere" size="0.1" mass="1"/></body></frame><frame name="f" pos="1 0 0"><body name="b" pos="0 0 0"><joint type="hinge"/><geom type="sphere" size="0.1" mass="1"/></body></frame></worldbody></mujoco>"#,
        ElementKind::Frame,
        "f",
    ),
    (
        "dup_frame_nested",
        r#"<mujoco><worldbody><frame name="f"><frame name="f"><body name="a" pos="0 0 0"><joint type="hinge"/><geom type="sphere" size="0.1" mass="1"/></body></frame></frame></worldbody></mujoco>"#,
        ElementKind::Frame,
        "f",
    ),
    (
        "dup_geom",
        r#"<mujoco><worldbody><body name="a"><joint/><geom name="g" size="0.1" mass="1"/></body><body name="b" pos="1 0 0"><joint/><geom name="g" size="0.1" mass="1"/></body></worldbody></mujoco>"#,
        ElementKind::Geom,
        "g",
    ),
    (
        "dup_geom_in_frame",
        r#"<mujoco><worldbody><body name="a"><joint/><geom name="g" size="0.1" mass="1"/><frame pos="0 0 1"><geom name="g" size="0.1" mass="1"/></frame></body></worldbody></mujoco>"#,
        ElementKind::Geom,
        "g",
    ),
    (
        "dup_geom_world_body",
        r#"<mujoco><worldbody><geom name="g" type="plane" size="1 1 0.1"/><body name="a"><joint/><geom name="g" size="0.1" mass="1"/></body></worldbody></mujoco>"#,
        ElementKind::Geom,
        "g",
    ),
    (
        "dup_key",
        r#"<mujoco><worldbody><body name="a"><joint name="j1"/><geom type="sphere" size="0.1" mass="1"/><site name="s1"/></body><body name="b" pos="1 0 0"><joint name="j2"/><geom type="sphere" size="0.1" mass="1"/><site name="s2"/></body></worldbody><keyframe><key name="k" qpos="0 0"/><key name="k" qpos="0.1 0"/></keyframe></mujoco>"#,
        ElementKind::Keyframe,
        "k",
    ),
    (
        "dup_pair",
        r#"<mujoco><worldbody><body name="a"><joint/><geom name="g1" size="0.1" mass="1"/><geom name="g2" size="0.1" mass="1" pos="0 0 1"/></body><body name="b" pos="1 0 0"><joint/><geom name="g3" size="0.1" mass="1"/></body></worldbody><contact><pair name="p" geom1="g1" geom2="g3"/><pair name="p" geom1="g2" geom2="g3"/></contact></mujoco>"#,
        ElementKind::Pair,
        "p",
    ),
    (
        "dup_plugin",
        r#"<mujoco><extension><plugin plugin="mujoco.elasticity.cable"><instance name="p"/><instance name="p"/></plugin></extension><worldbody><body name="a"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="b" pos="1 0 0"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="c" pos="0 1 0"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body></worldbody></mujoco>"#,
        ElementKind::Plugin,
        "p",
    ),
    (
        "dup_sensor_kinds",
        r#"<mujoco><worldbody><body name="a"><joint name="j1"/><geom type="sphere" size="0.1" mass="1"/><site name="s1"/></body><body name="b" pos="1 0 0"><joint name="j2"/><geom type="sphere" size="0.1" mass="1"/><site name="s2"/></body></worldbody><sensor><jointpos name="s" joint="j1"/><jointvel name="s" joint="j2"/></sensor></mujoco>"#,
        ElementKind::Sensor,
        "s",
    ),
    (
        "dup_site",
        r#"<mujoco><worldbody><body name="a"><joint/><geom type="sphere" size="0.1" mass="1"/><site name="s"/></body><body name="b" pos="1 0 0"><joint/><geom type="sphere" size="0.1" mass="1"/><site name="s"/></body></worldbody></mujoco>"#,
        ElementKind::Site,
        "s",
    ),
    (
        "dup_site_world_body",
        r#"<mujoco><worldbody><site name="s"/><body name="a"><joint/><geom type="sphere" size="0.1" mass="1"/><site name="s"/></body></worldbody></mujoco>"#,
        ElementKind::Site,
        "s",
    ),
    (
        "dup_skin",
        r#"<mujoco><worldbody><body name="a"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="b" pos="1 0 0"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="c" pos="0 1 0"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body></worldbody><deformable><skin name="s" vertex="0 0 0 1 0 0 0 1 0" face="0 1 2"><bone body="a" bindpos="0 0 0" bindquat="1 0 0 0" vertid="0 1 2" vertweight="1 1 1"/></skin><skin name="s" vertex="0 0 0 1 0 0 0 1 0" face="0 1 2"><bone body="b" bindpos="0 0 0" bindquat="1 0 0 0" vertid="0 1 2" vertweight="1 1 1"/></skin></deformable></mujoco>"#,
        ElementKind::Skin,
        "s",
    ),
    (
        "dup_tendon_fixed_spatial",
        r#"<mujoco><worldbody><body name="a"><joint name="j1"/><geom type="sphere" size="0.1" mass="1"/><site name="s1"/></body><body name="b" pos="1 0 0"><joint name="j2"/><geom type="sphere" size="0.1" mass="1"/><site name="s2"/></body></worldbody><tendon><fixed name="t"><joint joint="j1" coef="1"/></fixed><spatial name="t"><site site="s1"/><site site="s2"/></spatial></tendon></mujoco>"#,
        ElementKind::Tendon,
        "t",
    ),
    (
        "gen_generated_then_tendon_dup",
        r#"<mujoco><worldbody><body name="B_first" pos="0 2 0"><joint name="j1"/><geom size=".1" mass="1"/></body><composite type="cable" count="4 1 1" curve="s" size="1"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/><site/></composite></worldbody><tendon><fixed name="t"><joint joint="j1" coef="1"/></fixed><fixed name="t"><joint joint="j1" coef="2"/></fixed></tendon></mujoco>"#,
        ElementKind::Tendon,
        "t",
    ),
    (
        "gen_geom_G0",
        r#"<mujoco><worldbody><body name="u" pos="0 2 0"><joint/><geom name="G0" size=".1" mass="1"/></body><composite type="cable" count="4 1 1" curve="s" size="1"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/><site/></composite></worldbody></mujoco>"#,
        ElementKind::Geom,
        "G0",
    ),
    (
        "gen_site_S_first",
        r#"<mujoco><worldbody><body name="u" pos="0 2 0"><joint/><geom size=".1" mass="1"/><site name="S_first"/></body><composite type="cable" count="4 1 1" curve="s" size="1"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/><site/></composite></worldbody></mujoco>"#,
        ElementKind::Site,
        "S_first",
    ),
    (
        "order_actuator_then_sensor",
        r#"<mujoco><worldbody><body name="a"><joint name="j1"/><geom type="sphere" size="0.1" mass="1"/><site name="s1"/></body><body name="b" pos="1 0 0"><joint name="j2"/><geom type="sphere" size="0.1" mass="1"/><site name="s2"/></body></worldbody><sensor><jointpos name="s" joint="j1"/><jointpos name="s" joint="j2"/></sensor><actuator><motor name="m" joint="j1"/><motor name="m" joint="j2"/></actuator></mujoco>"#,
        ElementKind::Actuator,
        "m",
    ),
    (
        "order_geom_then_body",
        r#"<mujoco><worldbody><body name="a"><joint/><geom name="g" size="0.1" mass="1"/></body><body name="b" pos="1 0 0"><joint/><geom name="g" size="0.1" mass="1"/></body><body name="a" pos="2 0 0"><joint/><geom type="sphere" size="0.1" mass="1"/></body></worldbody></mujoco>"#,
        ElementKind::Geom,
        "g",
    ),
    (
        "order_tendon_written_first",
        r#"<mujoco><tendon><fixed name="t"><joint joint="j1" coef="1"/></fixed><fixed name="t"><joint joint="j2" coef="1"/></fixed></tendon><worldbody><body name="a"><joint name="j1"/><geom type="sphere" size="0.1" mass="1"/><site name="s1"/></body><body name="b" pos="1 0 0"><joint name="j2"/><geom type="sphere" size="0.1" mass="1"/><site name="s2"/></body><body name="a" pos="3 0 0"><joint type="hinge"/><geom type="sphere" size="0.1" mass="1"/></body></worldbody></mujoco>"#,
        ElementKind::Tendon,
        "t",
    ),
];

/// Documents MuJoCo 3.5.0 loads.
const LOADS: &[(&str, &str)] = &[
    (
        "dup_pair_exclude_cross_ok",
        r#"<mujoco><worldbody><body name="a"><joint/><geom name="g1" size="0.1" mass="1"/><geom name="g2" size="0.1" mass="1" pos="0 0 1"/></body><body name="b" pos="1 0 0"><joint/><geom name="g3" size="0.1" mass="1"/></body></worldbody><contact><pair name="x" geom1="g1" geom2="g3"/><exclude name="x" body1="a" body2="b"/></contact></mujoco>"#,
    ),
    (
        "ok_cross_kind",
        r#"<mujoco><worldbody><body name="x"><joint name="x"/><geom name="x" size="0.1" mass="1"/><site name="x"/></body></worldbody><actuator><motor name="x" joint="x"/></actuator><sensor><jointpos name="x" joint="x"/></sensor></mujoco>"#,
    ),
    (
        "ok_empty_name",
        r#"<mujoco><worldbody><body name=""><joint name=""/><geom name="" size="0.1" mass="1"/></body><body name="" pos="1 0 0"><joint name=""/><geom name="" size="0.1" mass="1"/></body></worldbody></mujoco>"#,
    ),
    (
        "ok_flex",
        r#"<mujoco><worldbody><body name="a"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><geom type="sphere" size="0.1" mass="1"/></body><body name="b" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><geom type="sphere" size="0.1" mass="1"/></body><body name="c" pos="0 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><geom type="sphere" size="0.1" mass="1"/></body></worldbody><deformable><flex name="f" dim="1" body="a b" vertex="0 0 0 0 0 0" element="0 1"/></deformable></mujoco>"#,
    ),
    (
        "ok_skin",
        r#"<mujoco><worldbody><body name="a"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="b" pos="1 0 0"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="c" pos="0 1 0"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body></worldbody><deformable><skin name="s" vertex="0 0 0 1 0 0 0 1 0" face="0 1 2"><bone body="a" bindpos="0 0 0" bindquat="1 0 0 0" vertid="0 1 2" vertweight="1 1 1"/></skin></deformable></mujoco>"#,
    ),
    (
        "ok_unnamed",
        r#"<mujoco><worldbody><body pos="0 0 0"><joint type="hinge"/><geom type="sphere" size="0.1" mass="1"/></body><body pos="1 0 0"><joint type="hinge"/><geom type="sphere" size="0.1" mass="1"/></body><geom type="plane" size="1 1 0.1"/><geom type="plane" size="1 1 0.1"/></worldbody></mujoco>"#,
    ),
];

#[test]
fn a_repeated_name_is_refused_with_mujocos_kind_and_name() {
    let mut missed = Vec::new();
    for &(case, xml, kind, name) in REFUSED {
        match load_model(xml) {
            Err(MjcfError::DuplicateName {
                kind: k, name: n, ..
            }) if k == kind && n == name => {}
            Err(e) => missed.push(format!("{case}: {e} (MuJoCo: '{name}' in {kind})")),
            Ok(_) => missed.push(format!("{case}: loads (MuJoCo: '{name}' in {kind})")),
        }
    }
    assert!(missed.is_empty(), "{}", missed.join("\n"));
}

#[test]
fn names_of_different_kinds_and_empty_names_load() {
    for &(case, xml) in LOADS {
        if let Err(e) = load_model(xml) {
            panic!("{case}: {e}");
        }
    }
}

/// Two cables of one prefix make one set of bodies twice. MuJoCo refuses
/// the model when it compiles, naming the repeated body name that sorts
/// first (`cB_1`); this crate names the first it meets in the tree
/// (`cB_first`; ledger L353).
#[test]
fn two_composites_of_one_prefix_repeat_a_body_name() {
    let xml = r#"<mujoco><worldbody><composite type="cable" count="4 1 1" curve="s" size="1" prefix="c" offset="0 0 0"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite><composite type="cable" count="4 1 1" curve="s" size="1" prefix="c" offset="0 1 0"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite></worldbody></mujoco>"#;
    let err = load_model(xml).expect_err("MuJoCo: repeated name 'cB_1' in body");
    assert!(
        matches!(&err, MjcfError::DuplicateName { kind: ElementKind::Body, name, .. } if name == "cB_first"),
        "{err}"
    );
}
