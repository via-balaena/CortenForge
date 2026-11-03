//! A height field's sizes as MuJoCo 3.5.0 checks them (book L22): each of
//! the four positive (`mjCHField::Compile`, `user_objects.cc:4471-4474`),
//! whether a geom names the height field or not, after every body's checks
//! and before the tendons' (`user_model.cc:4932-4956`). The parser refuses a
//! first or second size that is not positive and a third or fourth that is
//! negative, before any of them.
//!
//! Every document below was refused by MuJoCo 3.5.0 on the census's unfused
//! oracle (2026-11-02), "size parameter is not positive in hfield".

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_mjcf::load_model;

/// `(what, document)`, each with one height field, `h`.
const ZERO_SIZES: &[(&str, &str)] = &[
    (
        "third size 0, no geom names it",
        r#"<mujoco><asset><hfield name="h" size="1 1 0 0.5" nrow="2" ncol="2" elevation="0 0.5 1 0.25"/></asset><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
    ),
    (
        "fourth size 0, no geom names it",
        r#"<mujoco><asset><hfield name="h" size="1 1 1 0" nrow="2" ncol="2" elevation="0 0.5 1 0.25"/></asset><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
    ),
    (
        "fourth size 0, a static body's geom names it",
        r#"<mujoco><asset><hfield name="h" size="1 1 1 0" nrow="2" ncol="2" elevation="0 0.5 1 0.25"/></asset><worldbody><body name="b" pos="0 0 1"><geom type="hfield" hfield="h"/></body></worldbody></mujoco>"#,
    ),
    (
        "fourth size 0 beside a spatial tendon's negative armature",
        r#"<mujoco><asset><hfield name="h" size="1 1 1 0" nrow="2" ncol="2" elevation="0 0.5 1 0.25"/></asset><worldbody><site name="s0" pos="0 0 2"/><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><site name="s1"/></body></worldbody><tendon><spatial name="t" armature="-0.1"><site site="s0"/><site site="s1"/></spatial></tendon></mujoco>"#,
    ),
];

#[test]
fn a_third_or_fourth_size_of_0_is_refused_as_mujoco_refuses_it() {
    for &(what, xml) in ZERO_SIZES {
        let error = load_model(xml).err().map(|e| e.to_string());
        assert_eq!(
            error.as_deref(),
            Some("size parameter is not positive in hfield (hfield 'h')"),
            "{what}"
        );
    }
}
