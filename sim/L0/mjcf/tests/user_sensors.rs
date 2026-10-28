//! A `<user>` sensor's dimension, stage and object as MuJoCo 3.5.0 reads them,
//! and the sensor elements sim-mjcf does not read refused (book L11).
//!
//! Every value here was measured on MuJoCo 3.5.0; the census compares the
//! loadable documents' sensor layout with MuJoCo's.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use sim_core::{MjObjectType, MjSensorDataType};
use sim_mjcf::{MjcfError, load_model};

/// A user sensor takes the `dim` it writes, 0 included, and the sensor after
/// it takes the next address (MuJoCo: nsensordata 1 for `dim="0"` and a
/// jointpos, 4 for `dim="3"`).
#[test]
fn a_user_sensor_takes_its_dim() {
    let model = load_model(
        r#"<mujoco><worldbody><body name="b1"><joint name="j" type="hinge"/><geom size="0.1"/></body></worldbody>
<sensor><user dim="0"/><user dim="3"/><jointpos joint="j"/></sensor></mujoco>"#,
    )
    .expect("loads");
    assert_eq!(model.sensor_dim, [0, 3, 1]);
    assert_eq!(model.sensor_adr, [0, 0, 3]);
    assert_eq!(model.nsensordata, 4);
}

/// A user sensor's `needstage` is its stage, `acc` when unwritten.
#[test]
fn a_user_sensors_needstage_is_its_stage() {
    let model = load_model(
        r#"<mujoco><worldbody><body name="b1"><joint name="j" type="hinge"/><geom size="0.1"/></body></worldbody>
<sensor><user dim="1" needstage="pos"/><user dim="1" needstage="vel"/><user dim="1" needstage="acc"/>
<user dim="1"/></sensor></mujoco>"#,
    )
    .expect("loads");
    assert_eq!(
        model.sensor_datatype,
        [
            MjSensorDataType::Position,
            MjSensorDataType::Velocity,
            MjSensorDataType::Acceleration,
            MjSensorDataType::Acceleration
        ]
    );
}

/// A user sensor's object, as MuJoCo resolves `objtype` and `objname`:
/// body 1, xbody 1, joint 0, geom 0, site 0, tendon 0, actuator 0.
#[test]
fn a_user_sensors_object_is_resolved() {
    let model = load_model(
        r#"<mujoco><worldbody><body name="b1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody>
<tendon><fixed name="t"><joint joint="j" coef="1"/></fixed></tendon><actuator><motor name="a" joint="j"/></actuator>
<sensor><user dim="1" objtype="body" objname="b1"/><user dim="1" objtype="xbody" objname="b1"/>
<user dim="1" objtype="joint" objname="j"/><user dim="1" objtype="geom" objname="g"/><user dim="1" objtype="site" objname="s"/>
<user dim="1" objtype="tendon" objname="t"/><user dim="1" objtype="actuator" objname="a"/><user dim="1"/></sensor></mujoco>"#,
    )
    .expect("loads");
    assert_eq!(
        model.sensor_objtype,
        [
            MjObjectType::Body,
            MjObjectType::XBody,
            MjObjectType::Joint,
            MjObjectType::Geom,
            MjObjectType::Site,
            MjObjectType::Tendon,
            MjObjectType::Actuator,
            MjObjectType::None
        ]
    );
    assert_eq!(model.sensor_objid, [1, 1, 0, 0, 0, 0, 0, 0]);
}

fn refusal(sensors: &str) -> MjcfError {
    load_model(&format!(
        r#"<mujoco><worldbody><body name="b1"><joint name="j" type="hinge"/><geom size="0.1"/><camera name="c"/></body></worldbody>
<sensor>{sensors}</sensor></mujoco>"#
    ))
    .expect_err(sensors)
}

/// A user sensor's refusals, each MuJoCo's (measured) or, where marked, a
/// registered difference.
#[test]
fn a_user_sensor_is_refused_as_mujoco_refuses_it() {
    for (sensors, message) in [
        ("<user/>", "required attribute missing: 'dim' on <user>"),
        (
            r#"<user dim="-1"/>"#,
            "sensor dim must be non-negative in sensor",
        ),
        (
            r#"<user dim="1" objtype="body"/>"#,
            "objtype 'body' given but objname is missing",
        ),
        (
            r#"<user dim="1" objname="b1"/>"#,
            "objname 'b1' given but objtype is missing",
        ),
        (r#"<user dim="1" objtype="body" objname="nope"/>"#, "nope"),
        (
            r#"<user dim="1" needstage="bogus"/>"#,
            "invalid keyword \"bogus\"",
        ),
        (
            r#"<user dim="1" datatype="bogus"/>"#,
            "invalid keyword \"bogus\"",
        ),
        // MuJoCo leaves an unknown object (D-USER-SENSOR-OBJTYPE).
        (
            r#"<user dim="1" objtype="bogus" objname="b1"/>"#,
            "invalid keyword \"bogus\"",
        ),
    ] {
        let err = refusal(sensors).to_string();
        assert!(err.contains(message), "{sensors}: {err}");
    }
}

/// What MuJoCo makes and sim-mjcf cannot is refused as not supported
/// (D-USER-SENSOR, D-UNREAD-SENSOR), naming it.
#[test]
fn what_sim_mjcf_cannot_make_is_not_supported() {
    for (sensors, what) in [
        (r#"<user dim="1" needstage="none"/>"#, "needstage \"none\""),
        (
            r#"<user dim="1" datatype="positive"/>"#,
            "datatype \"positive\"",
        ),
        (r#"<user dim="3" datatype="axis"/>"#, "datatype \"axis\""),
        (
            r#"<user dim="1" objtype="camera" objname="c"/>"#,
            "on a camera",
        ),
        (r#"<jointlimitpos joint="j"/>"#, "<jointlimitpos>"),
        ("<e_potential/>", "<e_potential>"),
        (
            r#"<insidesite site="s" objtype="body" objname="b1"/>"#,
            "<insidesite>",
        ),
        (r#"<plugin plugin="x"/>"#, "<plugin>"),
    ] {
        let err = refusal(sensors);
        assert!(
            matches!(err, MjcfError::Unsupported { .. }),
            "{sensors}: {err}"
        );
        assert!(err.to_string().contains(what), "{sensors}: {err}");
    }
}

/// An element in `<sensor>` that MuJoCo's schema does not know is refused,
/// as MuJoCo refuses it, where sim-mjcf skipped it.
#[test]
fn an_unknown_element_in_sensor_is_refused() {
    let err = refusal("<bogus/>");
    assert!(
        matches!(&err, MjcfError::UnknownElement { element, parent, .. } if element == "bogus" && parent == "sensor"),
        "{err}"
    );
    let err = refusal("<bogus></bogus>");
    assert!(matches!(err, MjcfError::UnknownElement { .. }), "{err}");
}
