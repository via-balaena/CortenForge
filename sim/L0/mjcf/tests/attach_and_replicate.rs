//! A `<replicate>` and an `<attach>` are refused as not supported
//! (registry D-REPLICATE, D-ATTACH), where each used to be skipped, making
//! nothing and saying nothing (ledger L242, L251).
//!
//! Both are MuJoCo 3.5.0's `mjs_attach`. A `<replicate>` makes one copy of
//! its children per index, each moved by `offset` and turned by `euler`, its
//! names suffixed with the index, and copies with them the sensors and
//! actuators that name a replicated element (measured; what else is copied
//! was not measured): `xml_native_reader.cc:3660-3723`. An `<attach>` brings
//! in a body of the `<asset><model>` it names, or the whole model, its names
//! prefixed (`:3781-3814`, by reading). sim-mjcf has neither. Each
//! `<replicate>` document below makes two sites in MuJoCo 3.5.0; each
//! `<attach>` one is refused there too, as it names no model.

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
