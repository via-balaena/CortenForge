//! A document's `<compiler>`, `<option>` and `<flag>`, written more than once, as MuJoCo 3.5.0
//! reads them (`xml_native_reader.cc`, the Compiler and Option loops): each reads onto the same
//! settings and sets only what it writes. Each expected value was measured on MuJoCo 3.5.0.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp
)]

use sim_core::{DISABLE_CONTACT, DISABLE_GRAVITY, ENABLE_ENERGY, ENABLE_SLEEP, Model};
use sim_mjcf::load_model;

fn load(xml: &str) -> Model {
    load_model(xml).unwrap_or_else(|e| panic!("refused: {e}"))
}

/// A hinge with a range, under `sections`.
fn with(sections: &str) -> String {
    format!(
        r#"<mujoco>{sections}<worldbody><body><joint type="hinge" range="-1 1"/><geom size="0.1"/></body></worldbody></mujoco>"#
    )
}

#[test]
fn a_later_compiler_keeps_an_earlier_ones_settings() {
    // Before: a later <compiler> reset every setting it did not write, so
    // `angle="radian"` was lost and the range read in degrees.
    let m = load(&with(
        r#"<compiler angle="radian"/><compiler autolimits="true"/>"#,
    ));
    assert_eq!(m.jnt_range[0], (-1.0, 1.0));
    // Ledger L248 (dm_control's rodent): `angle`, then `meshdir`. MuJoCo:
    // the body's quat is (0.9848134698792882, 0.17361575258114187, 0, 0).
    let m = load(
        r#"<mujoco><compiler angle="radian"/><compiler meshdir="./"/>
        <worldbody><body euler="0.349 0 0"><freejoint/><geom size="0.1"/></body></worldbody></mujoco>"#,
    );
    let q = m.body_quat[1];
    assert!((q.w - 0.984_813_469_879_288_2).abs() < 1e-15, "{q}");
    assert!((q.i - 0.173_615_752_581_141_87).abs() < 1e-15, "{q}");
}

#[test]
fn a_later_option_keeps_an_earlier_ones_settings_and_flags() {
    let m = load(&with(
        r#"<option timestep="0.001"/><option><flag energy="enable"/></option>"#,
    ));
    assert_eq!(m.timestep, 0.001);
    assert_ne!(m.enableflags & ENABLE_ENERGY, 0);
    // Ledger L75: a second <option> kept the sleep flag (before: cleared it).
    let m = load(&with(
        r#"<option><flag sleep="enable"/></option><option timestep="0.001"/>"#,
    ));
    assert_eq!(m.timestep, 0.001);
    assert_ne!(m.enableflags & ENABLE_SLEEP, 0);
    let m = load(&with(r#"<option gravity="0 0 -1"/><option wind="1 0 0"/>"#));
    assert_eq!(m.gravity.z, -1.0);
    assert_eq!(m.wind.x, 1.0);
}

#[test]
fn a_flag_in_each_of_two_options_merges() {
    let m = load(&with(
        r#"<option><flag gravity="disable"/></option><option><flag contact="disable"/></option>"#,
    ));
    assert_eq!(m.disableflags, DISABLE_GRAVITY | DISABLE_CONTACT);
    let m = load(&with(
        r#"<option><flag gravity="disable"/></option><option><flag gravity="enable"/></option>"#,
    ));
    assert_eq!(m.disableflags, 0);
}

#[test]
fn a_later_options_prefix_lies_over_an_earlier_ones() {
    // Book L09's Also line: o_solref 0.05 0.5, then 0.07, is 0.07 0.5; an
    // o_friction of three, then one, keeps the second and third.
    let m = load(&with(
        r#"<option o_solref="0.05 0.5" o_friction="0.7 0.8 0.1"/><option o_solref="0.07" o_friction="0.3"/>"#,
    ));
    assert_eq!(m.o_solref, [0.07, 0.5]);
    assert_eq!(m.o_friction, [0.3, 0.8, 0.1, 0.0001, 0.0001]);
}

#[test]
fn a_later_options_disabled_actuator_groups_add_to_an_earlier_ones() {
    // MuJoCo ORs each group into the same mask: 1 2, then 3, is 0b1110.
    let m = load(&with(
        r#"<option actuatorgroupdisable="1 2"/><option actuatorgroupdisable="3"/>"#,
    ));
    assert_eq!(m.disableactuator, 0b1110);
    let m = load(&with(
        r#"<option actuatorgroupdisable="1 2"/><option timestep="0.001"/>"#,
    ));
    assert_eq!(m.disableactuator, 0b0110);
}
