//! A self-closing element (`<body/>`) is the same element with no children,
//! as MuJoCo 3.5.0's reader takes it: each parser dispatcher reads both forms
//! in one arm (book L15). The counts and values below were measured on
//! MuJoCo 3.5.0 for the same documents.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use approx::assert_relative_eq;
use sim_mjcf::{MjcfError, load_model, parse_mjcf_str};

#[test]
fn a_self_closing_mocap_body_is_a_body() {
    let model = load_model(
        r#"<mujoco><worldbody><body name="t" mocap="true" pos="0 0 1"/></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_eq!(model.nbody, 2);
    assert_eq!(model.nmocap, 1);
    assert_relative_eq!(model.body_pos[1].z, 1.0);
}

#[test]
fn a_self_closing_body_in_a_body_is_a_body() {
    let model = load_model(
        r#"<mujoco><worldbody><body name="p"><geom size=".1"/><body name="c" pos="0 0 1"/></body></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_eq!(model.nbody, 3);
    assert_relative_eq!(model.body_pos[2].z, 1.0);
}

#[test]
fn a_self_closing_body_in_a_frame_is_a_body() {
    let model = load_model(
        r#"<mujoco><worldbody><frame pos="1 0 0"><body name="c" pos="0 0 1"/></frame></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_eq!(model.nbody, 2);
    assert_relative_eq!(model.body_pos[1].x, 1.0);
    assert_relative_eq!(model.body_pos[1].z, 1.0);
}

/// A self-closing cable has no `<geom>`, which MuJoCo 3.5.0 refuses too
/// ("Cable geom type must be sphere, capsule or box").
#[test]
fn a_self_closing_cable_is_read_and_refused_for_its_missing_geom() {
    let err = load_model(
        r#"<mujoco><worldbody><composite type="cable" count="5 1 1" curve="s" size="1"/></worldbody></mujoco>"#,
    )
    .unwrap_err();
    assert!(
        matches!(&err, MjcfError::InvalidModel { reason, .. } if reason.contains("<geom>")),
        "{err}"
    );
}

/// MuJoCo 3.5.0 defines a class before reading its children, so a
/// self-closing class is defined and inherits from its parent.
#[test]
fn a_self_closing_default_class_is_defined() {
    let model = load_model(
        r#"<mujoco><default><geom contype="4"/><default class="a"><geom contype="2"/><default class="e"/></default></default><worldbody><geom class="e" size=".1"/></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_eq!(model.ngeom, 1);
    assert_eq!(model.geom_contype[0], 2);
}

/// A self-closing `<contact/>` adds nothing; it does not clear an earlier
/// section's pairs.
#[test]
fn a_self_closing_contact_keeps_earlier_pairs() {
    let model = load_model(
        r#"<mujoco><worldbody><geom name="a" size=".1"/><body><freejoint/><geom name="b" size=".1"/></body></worldbody><contact><pair geom1="a" geom2="b"/></contact><contact/></mujoco>"#,
    )
    .unwrap();
    assert_eq!(model.contact_pairs.len(), 1);
}

#[test]
fn a_self_closing_flex_is_a_flex() {
    let model = load_model(
        r#"<mujoco><worldbody>
            <body name="a" pos="0 0 1"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.5" diaginertia="1e-15 1e-15 1e-15"/></body>
            <body name="b" pos="0.1 0 1"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.5" diaginertia="1e-15 1e-15 1e-15"/></body>
            <body name="c" pos="0 0.1 1"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.5" diaginertia="1e-15 1e-15 1e-15"/></body>
        </worldbody><deformable><flex name="f" dim="2" body="a b c" element="0 1 2"/></deformable></mujoco>"#,
    )
    .unwrap();
    assert_eq!(model.nflex, 1);
    assert_eq!(model.nflexvert, 3);
    assert_eq!(model.nflexedge, 3);
}

/// Read only: MuJoCo 3.5.0 refuses a skin without bones ("Missing data in
/// skin"); what is pinned is that the first skin ends where it closes.
#[test]
fn a_self_closing_skin_is_followed_by_its_sibling() {
    let mjcf = parse_mjcf_str(
        r#"<mujoco><worldbody/><deformable><skin name="a" vertex="0 0 0 1 0 0 0 1 0" face="0 1 2"/><skin name="c" vertex="0 0 0 1 0 0 0 1 0" face="0 1 2"/></deformable></mujoco>"#,
    )
    .unwrap();
    assert_eq!(mjcf.skins.len(), 2);
}

#[test]
fn a_self_closing_root_is_an_empty_model() {
    let model = load_model("<mujoco/>").unwrap();
    assert_eq!(model.nbody, 1);
    assert_eq!(model.ngeom, 0);
}

/// Every section closed on itself: MuJoCo 3.5.0 loads an empty model.
#[test]
fn self_closing_sections_are_empty_sections() {
    let model = load_model(
        r"<mujoco><compiler/><option/><size/><visual/><statistic/><default/><extension/><custom/><asset/><worldbody/><deformable/><contact/><equality/><tendon/><actuator/><sensor/><keyframe/></mujoco>",
    )
    .unwrap();
    assert_eq!(model.nbody, 1);
    assert_eq!(model.ngeom, 0);
}

#[test]
fn a_self_closing_extension_plugin_is_declared() {
    let mjcf = parse_mjcf_str(
        r#"<mujoco><extension><plugin plugin="mujoco.elasticity.cable"/></extension><worldbody/></mujoco>"#,
    )
    .unwrap();
    assert_eq!(mjcf.extensions.len(), 1);
    assert_eq!(mjcf.extensions[0].plugins.len(), 1);
    assert_eq!(
        mjcf.extensions[0].plugins[0].plugin,
        "mujoco.elasticity.cable"
    );
}

#[test]
fn a_config_with_an_end_tag_is_read() {
    let mjcf = parse_mjcf_str(
        r#"<mujoco><extension><plugin plugin="mujoco.elasticity.cable"><instance name="i"><config key="twist" value="1"></config></instance></plugin></extension><worldbody/></mujoco>"#,
    )
    .unwrap();
    let config = &mjcf.extensions[0].plugins[0].instances[0].config;
    assert_eq!(config.len(), 1);
    assert_eq!(config[0].key, "twist");
}

/// A document with two root elements: MuJoCo 3.5.0 reads the first
/// (tinyxml2's `RootElement`), whichever of them closes itself. The documents
/// are built by `format!` so that the census does not take them: it takes a
/// literal's `<mujoco>` spans one by one, which would split each in two.
#[test]
fn only_the_first_root_is_read() {
    let geom = r#"<worldbody><geom size=".1"/></worldbody>"#;
    let model = load_model(&format!(
        r#"<mujoco model="a">{geom}</mujoco><mujoco model="b"/>"#
    ))
    .unwrap();
    assert_eq!(model.ngeom, 1);
    let model = load_model(&format!(
        r#"<mujoco model="b"/><mujoco model="a">{geom}</mujoco>"#
    ))
    .unwrap();
    assert_eq!(model.ngeom, 0);
    // A later root is still read to its end: one cut short is refused.
    load_model(&format!(
        r#"<mujoco model="a">{geom}</mujoco><mujoco model="b"><worldbody>"#
    ))
    .unwrap_err();
}

/// Inside a `<frame>`, MuJoCo 3.5.0 does not check its schema, so an element
/// it gives no children may have some there and still load. Such a child,
/// written with an end tag, keeps its children to itself: a cable `<joint>`'s
/// `<geom>` is not the cable's geom (MuJoCo: the cable's capsules are 0.01).
/// Outside a frame MuJoCo refuses the document ("unrecognized element").
#[test]
fn inside_a_frame_a_cable_joints_children_stay_its_own() {
    let model = load_model(
        r#"<mujoco><worldbody><frame><body name="p"><composite type="cable" count="5 1 1" curve="s" size="1"><geom type="capsule" size=".01"/><joint kind="main" damping="0.1"><geom type="capsule" size=".05"/></joint></composite></body></frame></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_relative_eq!(model.geom_size[0].x, 0.01);
}

/// The same for a body `<plugin>`'s `<config>` holding another: MuJoCo 3.5.0
/// reads the outer one only (its `plugin_attr` holds `twist` alone).
#[test]
fn inside_a_frame_a_configs_children_stay_its_own() {
    let mjcf = parse_mjcf_str(
        r#"<mujoco><extension><plugin plugin="mujoco.elasticity.cable"/></extension><worldbody><frame><body name="b"><plugin plugin="mujoco.elasticity.cable"><config key="twist" value="1"><config key="bend" value="2"/></config></plugin><geom size=".1"/></body></frame></worldbody></mujoco>"#,
    )
    .unwrap();
    let plugin = mjcf.worldbody.frames[0].bodies[0].plugin.as_ref().unwrap();
    assert_eq!(plugin.config.len(), 1);
    assert_eq!(plugin.config[0].key, "twist");
}
