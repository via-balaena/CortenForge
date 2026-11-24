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

/// A tuple naming an object the model lacks, an empty tuple, and an element
/// of a type MuJoCo does not know, each of which MuJoCo 3.5.0 refuses
/// (measured: "unrecognized object 'nosuch' in tuple 0", "tuple 't' (id = 0)
/// is empty", "unknown object type"), load here (registry
/// `D-TUPLE-UNCHECKED`).
#[test]
fn a_tuples_elements_are_not_checked() {
    for doc in [
        r#"<mujoco><worldbody><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody><custom><tuple name="t"><element objtype="body" objname="nosuch"/></tuple></custom></mujoco>"#,
        r#"<mujoco><worldbody><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody><custom><tuple name="t"/></custom></mujoco>"#,
        r#"<mujoco><worldbody><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody><custom><tuple name="t"><element objtype="nosuchtype" objname="f"/></tuple></custom></mujoco>"#,
    ] {
        assert!(load_model(doc).is_ok(), "{doc}");
    }
}
