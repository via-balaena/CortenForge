//! `<custom>` tuples, read for the bodies `fusestatic` keeps
//! (`tests/fusestatic.rs`) and the geoms `discardvisual` keeps
//! (`tests/discard_visual.rs`): each tuple's `name` and each element's
//! `objtype` and `objname` are required, as MuJoCo 3.5.0's reader requires
//! them; what an element names is not checked (registry `D-TUPLE-UNCHECKED`).

use sim_mjcf::load_model;

/// A tuple with no `name` and an element with no `objtype` or `objname` are
/// refused, as MuJoCo 3.5.0 refuses each (measured: "required attribute
/// missing: 'name'", "'objtype'", "'objname'").
#[test]
fn a_tuples_required_attributes_are_required() {
    for (doc, attribute) in [
        (
            r#"<mujoco><worldbody><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody><custom><tuple><element objtype="body" objname="f"/></tuple></custom></mujoco>"#,
            "name",
        ),
        (
            r#"<mujoco><worldbody><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody><custom><tuple name="t"><element objname="f"/></tuple></custom></mujoco>"#,
            "objtype",
        ),
        (
            r#"<mujoco><worldbody><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody><custom><tuple name="t"><element objtype="body"/></tuple></custom></mujoco>"#,
            "objname",
        ),
    ] {
        let refused = load_model(doc).err();
        assert!(
            refused
                .as_ref()
                .is_some_and(|e| e.to_string().contains(&format!("'{attribute}'"))),
            "{refused:?}: {doc}"
        );
    }
}

/// A tuple naming an object the model lacks, an empty tuple, an element of a
/// type MuJoCo does not know, a repeated tuple name, an `objname` empty or
/// holding a space, and a `prm` that is no number or more than one, each of
/// which MuJoCo 3.5.0 refuses (measured: "unrecognized object 'nosuch' in
/// tuple 0", "tuple 't' (id = 0) is empty", "unknown object type",
/// "repeated name 't' in tuple", "object arrays with different sizes" for
/// both names, "bad format in attribute 'prm'", "attribute 'prm' has too
/// much data"), load here (registry `D-TUPLE-UNCHECKED`).
#[test]
fn a_tuples_elements_are_not_checked() {
    for doc in [
        r#"<mujoco><worldbody><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody><custom><tuple name="t"><element objtype="body" objname="nosuch"/></tuple></custom></mujoco>"#,
        r#"<mujoco><worldbody><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody><custom><tuple name="t"/></custom></mujoco>"#,
        r#"<mujoco><worldbody><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody><custom><tuple name="t"><element objtype="nosuchtype" objname="f"/></tuple></custom></mujoco>"#,
        r#"<mujoco><worldbody><body name="a b" pos="0 0 1"><freejoint/><geom name="g" size="0.1"/></body><body name="b" pos="1 0 1"><freejoint/><geom size="0.1"/></body></worldbody><custom><tuple name="t"><element objtype="geom" objname="g"/></tuple><tuple name="t"><element objtype="geom" objname="g"/></tuple></custom></mujoco>"#,
        r#"<mujoco><worldbody><body name="a b" pos="0 0 1"><freejoint/><geom name="g" size="0.1"/></body><body name="b" pos="1 0 1"><freejoint/><geom size="0.1"/></body></worldbody><custom><tuple name="t"><element objtype="geom" objname=""/></tuple></custom></mujoco>"#,
        r#"<mujoco><worldbody><body name="a b" pos="0 0 1"><freejoint/><geom name="g" size="0.1"/></body><body name="b" pos="1 0 1"><freejoint/><geom size="0.1"/></body></worldbody><custom><tuple name="t"><element objtype="body" objname="a b"/></tuple></custom></mujoco>"#,
        r#"<mujoco><worldbody><body name="a b" pos="0 0 1"><freejoint/><geom name="g" size="0.1"/></body><body name="b" pos="1 0 1"><freejoint/><geom size="0.1"/></body></worldbody><custom><tuple name="t"><element objtype="geom" objname="g" prm="abc"/></tuple></custom></mujoco>"#,
        r#"<mujoco><worldbody><body name="a b" pos="0 0 1"><freejoint/><geom name="g" size="0.1"/></body><body name="b" pos="1 0 1"><freejoint/><geom size="0.1"/></body></worldbody><custom><tuple name="t"><element objtype="geom" objname="g" prm="1 2"/></tuple></custom></mujoco>"#,
    ] {
        assert!(load_model(doc).is_ok(), "{doc}");
    }
}

/// A tuple's `xbody` element keeps no body from fusion: MuJoCo 3.5.0 makes no
/// map of `xbody` names, so it fuses the static body `s` the element names,
/// then refuses the tuple ("unrecognized object 's' in tuple 0", measured).
/// Here `s` is fused, and the tuple, whose elements are not checked
/// (registry `D-TUPLE-UNCHECKED`), loads.
#[test]
fn a_tuples_xbody_element_keeps_no_body() {
    let m = load_model(
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="0 0 0.5"><geom size="0.2"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody><custom><tuple name="t"><element objtype="xbody" objname="s"/></tuple></custom></mujoco>"#,
    );
    assert_eq!(m.map(|m| m.nbody).ok(), Some(2), "s fused");
}
