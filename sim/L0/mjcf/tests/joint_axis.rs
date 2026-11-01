//! A joint's axis as MuJoCo 3.5.0 compiles it (book L20).
//!
//! `mjCJoint::Compile` (`user_objects.cc:2946-2960`), once it has checked the
//! joint's user data, springdamper and ranges (`:2884-2944`), sets a ball's
//! or a free joint's axis to (0, 0, 1), whatever is written; another joint's
//! axis it turns by the joint's frame and makes unit with `mjuu_normvec`,
//! which returns 0 where the squared norm is below mjEPS (1e-14), and then
//! refuses the joint: "axis too small in joint". Its body counts the degrees
//! of freedom after its joints, so a body of seven hinges, one of them with a
//! zero axis, is refused for the axis. Where this crate checks the axis
//! among its other checks: `tests/refusal_order.rs`.
//!
//! Every document, axis and reason below was measured on the census's
//! unfused oracle (2026-10-31). The axes take a square root and divisions
//! only, so they are compared bit for bit.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_mjcf::{MjcfError, load_model};

/// `(what, document, the joint MuJoCo names)`.
const REFUSED: &[(&str, &str, &str)] = &[
    (
        "class axis zero",
        r#"<mujoco><default><joint axis="0 0 0"/></default><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "j",
    ),
    (
        "hinge axis 1e-10",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 0 1e-10"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "j",
    ),
    (
        "hinge axis 1e-7",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 0 1e-7"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "j",
    ),
    (
        "hinge axis 9e-8",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 0 9e-8"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "j",
    ),
    (
        "hinge axis zero",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 0 0"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "j",
    ),
    (
        "rotated frame axis zero",
        r#"<mujoco><worldbody><frame euler="0 90 0"><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 0 0"/><geom type="sphere" size="0.1"/></body></frame></worldbody></mujoco>"#,
        "j",
    ),
    (
        "slide axis zero",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="slide" axis="0 0 0"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "j",
    ),
    (
        "twofault seven hinges one zero",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint type="hinge"/><joint type="hinge"/><joint type="hinge"/><joint type="hinge"/><joint type="hinge"/><joint type="hinge"/><joint name="z" type="hinge" axis="0 0 0"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "z",
    ),
];

/// `(what, document, MuJoCo's jnt_axis of its one joint)`.
const LOADS: &[(&str, &str, [f64; 3])] = &[
    (
        "ball axis x",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="ball" axis="1 0 0"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        [0.0, 0.0, 1.0],
    ),
    (
        "ball axis zero",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="ball" axis="0 0 0"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        [0.0, 0.0, 1.0],
    ),
    (
        "free axis x",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="free" axis="1 0 0"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        [0.0, 0.0, 1.0],
    ),
    (
        "free axis zero",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="free" axis="0 0 0"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        [0.0, 0.0, 1.0],
    ),
    (
        "hinge axis 1.1e-7",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 0 1.1e-7"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        [0.0, 0.0, 1.0],
    ),
    (
        "hinge axis tiny diag",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="6e-8 6e-8 6e-8"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        [
            0.577_350_269_189_625_7,
            0.577_350_269_189_625_7,
            0.577_350_269_189_625_7,
        ],
    ),
    (
        "hinge axis unwritten",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        [0.0, 0.0, 1.0],
    ),
    (
        "hinge axis xy",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="3 4 0"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        [0.6, 0.8, 0.0],
    ),
    (
        "slide axis unwritten",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="slide"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        [0.0, 0.0, 1.0],
    ),
];

#[test]
fn an_axis_too_small_to_make_unit_is_refused() {
    let mut missed = Vec::new();
    for &(what, xml, joint) in REFUSED {
        let expected =
            format!("invalid axis on <joint>: axis too small (joint '{joint}' in body 'b')");
        match load_model(xml) {
            Err(e @ MjcfError::InvalidValue { .. }) if e.to_string() == expected => {}
            Err(other) => missed.push(format!("{what}: {other}")),
            Ok(m) => missed.push(format!("{what}: loads, axis {:?}", m.jnt_axis[0])),
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
fn a_joints_axis_is_mujocos() {
    let mut differ = Vec::new();
    for &(what, xml, axis) in LOADS {
        match load_model(xml) {
            Ok(m) => {
                let got = [m.jnt_axis[0].x, m.jnt_axis[0].y, m.jnt_axis[0].z];
                if got.map(f64::to_bits) != axis.map(f64::to_bits) {
                    differ.push(format!("{what}: {got:?} (MuJoCo {axis:?})"));
                }
            }
            Err(e) => differ.push(format!("{what}: {e}")),
        }
    }
    assert!(differ.is_empty(), "{}", differ.join("\n"));
}
