//! A body's joints laid out as MuJoCo 3.5.0 allows them (book L19; ledger
//! L216).
//!
//! MuJoCo checks a body's joints once it has compiled its geoms and joints
//! (`mjCBody::Compile`, `user_objects.cc:2517-2535`): at most 6 degrees of
//! freedom ("more than 6 dofs", so a free joint is its body's only joint),
//! then no ball or hinge after a ball ("ball followed by rotation"). Only
//! when it copies the tree into its model, after it has compiled the bodies,
//! the tendons and the actuators, does it require a free joint's body to be a
//! child of the world, once frames are dissolved and static bodies fused
//! ("free joint can only be used on top level", `user_model.cc:2725-2727`),
//! so a document with both faults is refused for the first. A slide after a
//! ball, which MuJoCo loads, sim-mjcf refuses as not supported, after every
//! check MuJoCo makes (`D-BALL-THEN-SLIDE`).
//!
//! Every document, reason and degree-of-freedom count below was measured on
//! the census's unfused oracle (2026-10-31).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_mjcf::{MjcfError, load_model};

/// `(what, document, MuJoCo's reason, the body it names)`.
const REFUSED: &[(&str, &str, &str, &str)] = &[
    (
        "ball slide hinge",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint type="ball"/><joint type="slide"/><joint type="hinge"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "ball followed by rotation",
        "b",
    ),
    (
        "ball then ball",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint type="ball"/><joint type="ball"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "ball followed by rotation",
        "b",
    ),
    (
        "ball then hinge",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint type="ball"/><joint type="hinge"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "ball followed by rotation",
        "b",
    ),
    (
        "default type ball then hinge",
        r#"<mujoco><default><joint type="ball"/></default><worldbody><body name="b" pos="0 0 1"><joint/><joint type="hinge"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "ball followed by rotation",
        "b",
    ),
    (
        "frame body ball hinge",
        r#"<mujoco><worldbody><frame pos="0 0 1"><body name="b"><joint type="ball"/><joint type="hinge"/><geom type="sphere" size="0.1"/></body></frame></worldbody></mujoco>"#,
        "ball followed by rotation",
        "b",
    ),
    (
        "free in body in frame in body",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint type="hinge"/><geom type="sphere" size="0.1"/><frame pos="0 0 0.5"><body name="b"><freejoint/><geom type="sphere" size="0.1"/></body></frame></body></worldbody></mujoco>"#,
        "free joint can only be used on top level",
        "b",
    ),
    (
        "free nested",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint type="hinge"/><geom type="sphere" size="0.1"/><body name="b" pos="0 0 0.5"><freejoint/><geom type="sphere" size="0.1"/></body></body></worldbody></mujoco>"#,
        "free joint can only be used on top level",
        "b",
    ),
    (
        "free nested frames",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint type="hinge"/><geom type="sphere" size="0.1"/><frame><frame pos="0 0 0.5"><body name="b"><freejoint/><geom type="sphere" size="0.1"/></body></frame></frame></body></worldbody></mujoco>"#,
        "free joint can only be used on top level",
        "b",
    ),
    (
        "free plus hinge",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint/><joint type="hinge"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "more than 6 dofs",
        "b",
    ),
    (
        "free plus slide",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint/><joint type="slide"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "more than 6 dofs",
        "b",
    ),
    (
        "free under static nofuse",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><geom type="sphere" size="0.1"/><body name="b" pos="0 0 0.5"><freejoint/><geom type="sphere" size="0.1"/></body></body></worldbody></mujoco>"#,
        "free joint can only be used on top level",
        "b",
    ),
    (
        "hinge then free",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint type="hinge"/><freejoint/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "more than 6 dofs",
        "b",
    ),
    (
        "seven hinges",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint type="hinge"/><joint type="hinge"/><joint type="hinge"/><joint type="hinge"/><joint type="hinge"/><joint type="hinge"/><joint type="hinge"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "more than 6 dofs",
        "b",
    ),
    (
        "three balls",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint type="ball"/><joint type="ball"/><joint type="ball"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "more than 6 dofs",
        "b",
    ),
    (
        "two free",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint/><freejoint/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "more than 6 dofs",
        "b",
    ),
    (
        "twofault ball hinge first",
        r#"<mujoco><worldbody><body name="c" pos="1 0 1"><joint type="ball"/><joint type="hinge"/><geom type="sphere" size="0.1"/></body><body name="a" pos="0 0 1"><joint type="hinge"/><geom type="sphere" size="0.1"/><body name="b" pos="0 0 0.5"><freejoint/><geom type="sphere" size="0.1"/></body></body></worldbody></mujoco>"#,
        "ball followed by rotation",
        "c",
    ),
    (
        "twofault nested free first",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint type="hinge"/><geom type="sphere" size="0.1"/><body name="b" pos="0 0 0.5"><freejoint/><geom type="sphere" size="0.1"/></body></body><body name="c" pos="1 0 1"><joint type="ball"/><joint type="hinge"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "ball followed by rotation",
        "c",
    ),
    (
        "type free nested",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint type="hinge"/><geom type="sphere" size="0.1"/><body name="b" pos="0 0 0.5"><joint type="free"/><geom type="sphere" size="0.1"/></body></body></worldbody></mujoco>"#,
        "free joint can only be used on top level",
        "b",
    ),
];

const LOADS: &[(&str, &str, usize)] = &[
    (
        "free alone",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        6,
    ),
    (
        "free under static fusestatic",
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="a" pos="0 0 1"><geom type="sphere" size="0.1"/><body name="b" pos="0 0 0.5"><freejoint/><geom type="sphere" size="0.1"/></body></body></worldbody></mujoco>"#,
        6,
    ),
    (
        "hinge hinge ball",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint type="hinge" pos="0.1 0 0"/><joint type="hinge" axis="1 0 0" pos="0 0.1 0"/><joint type="ball"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        5,
    ),
    (
        "hinge then ball",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint type="hinge" pos="0.1 0 0"/><joint type="ball"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        4,
    ),
    (
        "slide then ball",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint type="slide"/><joint type="ball"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        4,
    ),
    (
        "three slide three hinge",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><joint type="hinge" axis="1 0 0"/><joint type="hinge" axis="0 1 0"/><joint type="hinge" axis="0 0 1"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        6,
    ),
    (
        "world frame frame free",
        r#"<mujoco><worldbody><frame pos="0 0 1"><frame pos="0 0 0.5"><body name="b"><freejoint/><geom type="sphere" size="0.1"/></body></frame></frame></worldbody></mujoco>"#,
        6,
    ),
];

#[test]
fn a_joint_layout_mujoco_refuses_is_refused_for_the_fault_it_names() {
    let mut missed = Vec::new();
    for &(what, xml, reason, body) in REFUSED {
        let expected = format!("{reason} (body '{body}')");
        match load_model(xml) {
            Err(e @ MjcfError::InvalidModel { .. }) if e.to_string() == expected => {}
            Err(other) => missed.push(format!("{what}: {other}")),
            Ok(m) => missed.push(format!("{what}: loads, nv {}", m.nv)),
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

#[test]
fn a_joint_layout_mujoco_loads_loads() {
    let mut missed = Vec::new();
    for &(what, xml, nv) in LOADS {
        match load_model(xml) {
            Ok(m) if m.nv == nv && m.try_make_data().is_ok() => {}
            Ok(m) => missed.push(format!("{what}: nv {} (MuJoCo {nv})", m.nv)),
            Err(e) => missed.push(format!("{what}: {e}")),
        }
    }
    assert!(missed.is_empty(), "{}", missed.join("\n"));
}

/// MuJoCo loads a slide after a ball (nq 5, nv 4, measured); sim-mjcf
/// refuses it as not supported, as the parity rule's decisions decided
/// (01-decisions; `D-BALL-THEN-SLIDE`).
#[test]
fn a_slide_after_a_ball_is_not_supported() {
    let xml = r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint type="ball"/><joint type="slide"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#;
    match load_model(xml) {
        Err(e @ MjcfError::Unsupported { .. })
            if e.to_string()
                .ends_with("a slide joint after a ball joint on one body (body 'b')") => {}
        other => panic!("{:?}", other.map(|m| m.nv)),
    }
}
