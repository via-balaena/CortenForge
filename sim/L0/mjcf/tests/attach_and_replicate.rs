//! A `<replicate>` and an `<attach>` are refused as not supported
//! (registry D-REPLICATE, D-ATTACH), where each used to be skipped, making
//! nothing and saying nothing (ledger L242, L251).
//!
//! A `<replicate>` is MuJoCo 3.5.0's `mjs_attach` of its children, once per
//! index, each copy moved by `offset` and turned by `euler`, its names
//! suffixed with the index, and with them the sensors, actuators, tendons and
//! excludes that name a replicated element (measured; other referrers were
//! not): `xml_native_reader.cc:3660-3727`. An `<attach>` of another model
//! brings in a body of the `<asset><model>` it names, or the whole model, its
//! names prefixed, also by `mjs_attach`; one whose `prefix` and `body` name a
//! body the document already has moves that body into the attach's frame
//! instead (`mjs_setFrame`, refused when the frame's parent is not the body's;
//! `:3781-3814`, read, and measured). sim-mjcf has none of
//! these. Each `<replicate>` document in the first test makes two sites in
//! MuJoCo 3.5.0; each `<attach>` one in the second is refused there too, as
//! it names no model.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_mjcf::{MjcfError, load_model};

fn assert_unsupported(xml: &str, element: &str) {
    match load_model(xml).unwrap_err() {
        MjcfError::Unsupported { feature, .. } => assert_eq!(feature, element),
        other => panic!("not a stated limitation: {other}"),
    }
}

#[test]
fn a_replicate_is_not_supported_in_the_worldbody_a_body_or_a_frame() {
    for xml in [
        r#"<mujoco><worldbody><replicate count="2" offset="1 0 0"><site name="s" size="0.1"/></replicate></worldbody></mujoco>"#,
        r#"<mujoco><worldbody><body name="p"><replicate count="2" offset="1 0 0"><site name="s" size="0.1"/></replicate></body></worldbody></mujoco>"#,
        r#"<mujoco><worldbody><frame pos="0 0 1"><replicate count="2" offset="1 0 0"><site name="s" size="0.1"/></replicate></frame></worldbody></mujoco>"#,
    ] {
        assert_unsupported(xml, "<replicate>");
    }
}

#[test]
fn an_attach_is_not_supported_in_the_worldbody_a_body_or_a_frame() {
    for xml in [
        r#"<mujoco><worldbody><attach model="m" prefix="p"/></worldbody></mujoco>"#,
        r#"<mujoco><worldbody><body name="b"><attach model="m" prefix="p"/></body></worldbody></mujoco>"#,
        r#"<mujoco><worldbody><frame pos="0 0 1"><attach model="m" prefix="p"/></frame></worldbody></mujoco>"#,
    ] {
        assert_unsupported(xml, "<attach>");
    }
}

/// Forms MuJoCo 3.5.0 builds as nothing, or with no other model, are refused
/// too: a `<replicate>` with no children or a count of 0 or below makes
/// nothing there (D-REPLICATE, stricter for these), and an `<attach>` whose
/// `prefix` and `body` name a body the document already has only moves it
/// into the attach's frame, here `pb` to (1, 0, 2) (D-ATTACH). Measured on
/// MuJoCo 3.5.0.
#[test]
fn the_forms_mujoco_builds_without_attaching_are_refused_too() {
    for xml in [
        r#"<mujoco><worldbody><replicate count="2" offset="1 0 0"/></worldbody></mujoco>"#,
        r#"<mujoco><worldbody><replicate count="0"><site name="s"/></replicate></worldbody></mujoco>"#,
    ] {
        assert_unsupported(xml, "<replicate>");
    }
    assert_unsupported(
        r#"<mujoco><worldbody><body name="pb" pos="0 0 2"><geom size=".1"/></body><frame pos="1 0 0"><attach model="m" body="b" prefix="p"/></frame></worldbody></mujoco>"#,
        "<attach>",
    );
}
