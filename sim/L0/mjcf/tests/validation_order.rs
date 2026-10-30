//! The model is checked once every element has its class and its frames and
//! composites are expanded, and before `discardvisual` and `fusestatic`
//! remove any element (book L16): MuJoCo 3.5.0 checks a name against every
//! body a frame or a composite makes, and before it fuses or discards
//! (measured: each document below is refused, "repeated name … in body").

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_mjcf::{
    ElementKind, EqualityKind, MjcfError, load_model, model_from_mjcf, parse_mjcf_str, validate,
};

fn assert_repeated_body(err: &MjcfError, name: &str) {
    assert!(
        matches!(err, MjcfError::DuplicateName { kind: ElementKind::Body, name: n, .. } if n == name),
        "{err}"
    );
}

#[test]
fn a_body_name_repeated_in_a_frame_is_refused() {
    let err = load_model(
        r#"<mujoco><worldbody><body name="x"><joint/><geom size=".1"/></body><frame><body name="x" pos="1 0 0"><joint/><geom size=".1"/></body></frame></worldbody></mujoco>"#,
    )
    .unwrap_err();
    assert_repeated_body(&err, "x");
}

/// `validate` checks the same model the loader does.
#[test]
fn validate_sees_the_bodies_a_frame_holds() {
    let mjcf = parse_mjcf_str(
        r#"<mujoco><worldbody><body name="x"><joint/><geom size=".1"/></body><frame><body name="x" pos="1 0 0"><joint/><geom size=".1"/></body></frame></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_repeated_body(&validate(&mjcf).unwrap_err(), "x");
}

/// The name of a body `fusestatic` folds away is checked before it does.
#[test]
fn a_body_name_repeated_in_a_fused_body_is_refused() {
    let err = load_model(
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="a"><joint/><geom size=".1"/><body name="x" pos="0 0 1"><geom size=".1"/></body></body><body name="x" pos="1 0 0"><joint/><geom size=".1"/></body></worldbody></mujoco>"#,
    )
    .unwrap_err();
    assert_repeated_body(&err, "x");
}

#[test]
fn two_cables_with_one_prefix_are_refused() {
    let err = load_model(
        r#"<mujoco><worldbody><body name="h1"><composite prefix="c" type="cable" count="3 1 1" curve="s" size="1" initial="none"><geom type="capsule" size=".01"/></composite></body><body name="h2" pos="0 1 0"><composite prefix="c" type="cable" count="3 1 1" curve="s" size="1" initial="none"><geom type="capsule" size=".01"/></composite></body></worldbody></mujoco>"#,
    )
    .unwrap_err();
    assert_repeated_body(&err, "cB_first");
}

#[test]
fn a_cable_body_named_as_a_written_body_is_refused() {
    let err = load_model(
        r#"<mujoco><worldbody><body name="B_first"><joint/><geom size=".1"/></body><body name="h"><composite type="cable" count="3 1 1" curve="s" size="1" initial="none"><geom type="capsule" size=".01"/></composite></body></worldbody></mujoco>"#,
    )
    .unwrap_err();
    assert_repeated_body(&err, "B_first");
}

/// A model built in code whose equality order names an equality its lists do
/// not hold is refused, by `validate` and by the loader alike.
#[test]
fn an_equality_order_naming_a_missing_equality_is_refused() {
    let mut mjcf = parse_mjcf_str(
        r#"<mujoco><worldbody><body name="b"><joint/><geom size=".1"/></body></worldbody></mujoco>"#,
    )
    .unwrap();
    mjcf.equality.order.push((EqualityKind::Connect, 0));
    for err in [
        validate(&mjcf).unwrap_err(),
        model_from_mjcf(&mjcf, None).unwrap_err(),
    ] {
        assert!(
            matches!(&err, MjcfError::InvalidModel { reason, .. } if reason.contains("connect #0")),
            "{err}"
        );
    }
}

/// References into a frame resolve, as MuJoCo 3.5.0's do: an actuator on a
/// site a frame holds, a motor on a joint of a body a frame holds.
#[test]
fn an_actuator_reaches_what_a_frame_holds() {
    for xml in [
        r#"<mujoco><worldbody><body><joint name="j" type="hinge"/><geom size=".1"/><frame pos="0.1 0 0"><site name="s"/></frame></body></worldbody><actuator><general site="s" gear="1 0 0 0 0 0"/></actuator></mujoco>"#,
        r#"<mujoco><worldbody><frame pos="1 0 0"><body><joint name="j" type="hinge"/><geom size=".1"/></body></frame></worldbody><actuator><motor joint="j"/></actuator></mujoco>"#,
    ] {
        assert_eq!(load_model(xml).unwrap().nu, 1);
    }
}

#[test]
fn a_joint_name_repeated_in_a_frame_is_refused() {
    let err = load_model(
        r#"<mujoco><worldbody><body><joint name="j"/><geom size=".1"/><frame><body><joint name="j"/><geom size=".1"/></body></frame></body></worldbody></mujoco>"#,
    )
    .unwrap_err();
    assert!(
        matches!(&err, MjcfError::DuplicateName { kind: ElementKind::Joint, name, .. } if name == "j"),
        "{err}"
    );
}
