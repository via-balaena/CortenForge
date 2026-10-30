//! A tendon's stiffness, damping and frictionloss as MuJoCo 3.5.0 loads them
//! (book L16): a negative stiffness or damping, written or given by its
//! class, loads as written, and sim-core applies it as given
//! (`engine_passive.c:453-473`); a negative frictionloss, which MuJoCo loads
//! and makes no constraint row for (`engine_core_constraint.c:737`), is
//! refused (`D-TENDON-FRICTIONLOSS`). Each value is MuJoCo 3.5.0's for the
//! same document (measured).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use approx::assert_relative_eq;
use sim_mjcf::{MjcfError, load_model};

#[test]
fn a_tendons_negative_stiffness_and_damping_load() {
    let model = load_model(
        r#"<mujoco><worldbody><body><joint name="j0" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0 0 -1" size="0.05" mass="1"/></body></worldbody><tendon><fixed name="t" stiffness="-2" damping="-0.5"><joint joint="j0" coef="1"/></fixed></tendon></mujoco>"#,
    )
    .unwrap();
    assert_relative_eq!(model.tendon_stiffness[0], -2.0);
    assert_relative_eq!(model.tendon_damping[0], -0.5);
}

#[test]
fn a_tendons_negative_stiffness_and_damping_from_its_class_load() {
    let model = load_model(
        r#"<mujoco><default><tendon stiffness="-2" damping="-0.5"/></default><worldbody><body><joint name="j0" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0 0 -1" size="0.05" mass="1"/></body></worldbody><tendon><fixed name="t"><joint joint="j0" coef="1"/></fixed></tendon></mujoco>"#,
    )
    .unwrap();
    assert_relative_eq!(model.tendon_stiffness[0], -2.0);
    assert_relative_eq!(model.tendon_damping[0], -0.5);
}

#[test]
fn a_tendons_negative_frictionloss_is_refused() {
    let err = load_model(
        r#"<mujoco><worldbody><body><joint name="j0" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0 0 -1" size="0.05" mass="1"/></body></worldbody><tendon><fixed name="t" frictionloss="-1"><joint joint="j0" coef="1"/></fixed></tendon></mujoco>"#,
    )
    .unwrap_err();
    assert!(
        matches!(&err, MjcfError::InvalidValue { element, attribute, .. } if element == "tendon" && attribute == "frictionloss"),
        "{err}"
    );
}
