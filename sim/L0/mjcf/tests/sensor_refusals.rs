//! A sensor's noise, cutoff and geoms checked as MuJoCo 3.5.0's
//! `mjCSensor::Compile` checks them (ledger L197, L199, L391): a negative
//! noise, then a negative cutoff ('negative noise in sensor', 'negative cutoff
//! in sensor', `user_objects.cc:7295-7303`); a geom distance, normal or
//! segment naming a height-field geom ('height fields are not supported in
//! geom distance sensors', `:7518-7523`; a body holding one is taken); a
//! cutoff above 0 on a quaternion, or on an axis other than a geom normal's,
//! whose cutoff is its search distance ('cutoff applied to axis or quaternion
//! datatype in sensor', `:7628-7632`). Within one sensor the noise is checked
//! before the cutoff, and both before the limit, the height field and the
//! quaternion or axis cutoff: pinned by the rows that set a negative noise or
//! cutoff beside one of those (MuJoCo refuses each of their faults alone).
//! Each document was run on MuJoCo 3.5.0 as written (2026-11-05): every
//! refused one is refused there for the same reason, every loaded one loads
//! there.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_mjcf::load_model;

/// `(case, document, this crate's error)`; MuJoCo's reason is the error's
/// words before the sensor's number.
const REFUSED: &[(&str, &str, &str)] = &[
    (
        "a joint position's negative noise",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody><sensor><jointpos joint="j" noise="-0.1"/></sensor></mujoco>"#,
        "negative noise in sensor (sensor #0)",
    ),
    (
        "a joint position's negative cutoff",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody><sensor><jointpos joint="j" cutoff="-0.1"/></sensor></mujoco>"#,
        "negative cutoff in sensor (sensor #0)",
    ),
    (
        "a negative noise beside a negative cutoff",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody><sensor><jointpos joint="j" cutoff="-0.1" noise="-0.1"/></sensor></mujoco>"#,
        "negative noise in sensor (sensor #0)",
    ),
    (
        "a touch's least negative noise",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom size=".1"/><site name="s"/></body></worldbody><sensor><touch site="s" noise="-1e-300"/></sensor></mujoco>"#,
        "negative noise in sensor (sensor #0)",
    ),
    (
        "a distance's negative cutoff",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size=".1"/></body><geom name="w" pos="1 0 1" size=".1"/></worldbody><sensor><distance geom1="g" geom2="w" cutoff="-0.1"/></sensor></mujoco>"#,
        "negative cutoff in sensor (sensor #0)",
    ),
    (
        "a later sensor's negative noise",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody><sensor><jointpos name="p" joint="j"/><jointvel name="v" joint="j" noise="-0.1"/></sensor></mujoco>"#,
        "negative noise in sensor (sensor 'v')",
    ),
    (
        "a frame quaternion's cutoff",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom size=".1"/><site name="s"/></body></worldbody><sensor><framequat objtype="site" objname="s" cutoff="0.3"/></sensor></mujoco>"#,
        "cutoff applied to axis or quaternion datatype in sensor (sensor #0)",
    ),
    (
        "a ball quaternion's least cutoff",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="ball"/><geom size=".1"/></body></worldbody><sensor><ballquat joint="j" cutoff="1e-300"/></sensor></mujoco>"#,
        "cutoff applied to axis or quaternion datatype in sensor (sensor #0)",
    ),
    (
        "a frame axis's cutoff",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom size=".1"/><site name="s"/></body></worldbody><sensor><framezaxis objtype="site" objname="s" cutoff="1"/></sensor></mujoco>"#,
        "cutoff applied to axis or quaternion datatype in sensor (sensor #0)",
    ),
    (
        "a height field as a distance's first geom",
        r#"<mujoco><asset><hfield name="f" nrow="2" ncol="2" size="1 1 .1 .1" elevation="0 0 0 0"/></asset><worldbody><geom name="h" type="hfield" hfield="f"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size=".1"/></body></worldbody><sensor><distance geom1="h" geom2="g"/></sensor></mujoco>"#,
        "height fields are not supported in geom distance sensors (sensor #0)",
    ),
    (
        "a height field as a normal's second geom",
        r#"<mujoco><asset><hfield name="f" nrow="2" ncol="2" size="1 1 .1 .1" elevation="0 0 0 0"/></asset><worldbody><geom name="h" type="hfield" hfield="f"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size=".1"/></body></worldbody><sensor><normal geom1="g" geom2="h"/></sensor></mujoco>"#,
        "height fields are not supported in geom distance sensors (sensor #0)",
    ),
    (
        "a height field in a static body as a segment's first geom",
        r#"<mujoco><asset><hfield name="f" nrow="2" ncol="2" size="1 1 .1 .1" elevation="0 0 0 0"/></asset><worldbody><body name="a" pos="3 0 0"><geom name="h" type="hfield" hfield="f"/></body><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size=".1"/></body></worldbody><sensor><fromto geom1="h" body2="b"/></sensor></mujoco>"#,
        "height fields are not supported in geom distance sensors (sensor #0)",
    ),
    (
        "an unlimited joint's limit force with a negative noise",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom size=".1"/><site name="s"/></body></worldbody><sensor><jointlimitfrc joint="j" noise="-0.1"/></sensor></mujoco>"#,
        "negative noise in sensor (sensor #0)",
    ),
    (
        "an unlimited joint's limit force with a negative cutoff",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom size=".1"/><site name="s"/></body></worldbody><sensor><jointlimitfrc joint="j" cutoff="-0.1"/></sensor></mujoco>"#,
        "negative cutoff in sensor (sensor #0)",
    ),
    (
        "an unlimited tendon's limit force with a negative cutoff",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom size=".1"/><site name="s"/></body></worldbody><tendon><fixed name="t"><joint joint="j" coef="1"/></fixed></tendon><sensor><tendonlimitfrc tendon="t" cutoff="-0.1"/></sensor></mujoco>"#,
        "negative cutoff in sensor (sensor #0)",
    ),
    (
        "a distance from a height field with a negative cutoff",
        r#"<mujoco><asset><hfield name="f" nrow="2" ncol="2" size="1 1 .1 .1" elevation="0 0 0 0"/></asset><worldbody><geom name="h" type="hfield" hfield="f"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size=".1"/></body></worldbody><sensor><distance geom1="h" geom2="g" cutoff="-0.1"/></sensor></mujoco>"#,
        "negative cutoff in sensor (sensor #0)",
    ),
    (
        "a distance from a height field with a negative noise",
        r#"<mujoco><asset><hfield name="f" nrow="2" ncol="2" size="1 1 .1 .1" elevation="0 0 0 0"/></asset><worldbody><geom name="h" type="hfield" hfield="f"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size=".1"/></body></worldbody><sensor><distance geom1="h" geom2="g" noise="-0.1"/></sensor></mujoco>"#,
        "negative noise in sensor (sensor #0)",
    ),
    (
        "a frame quaternion with a negative noise and a cutoff",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom size=".1"/><site name="s"/></body></worldbody><sensor><framequat objtype="site" objname="s" noise="-0.1" cutoff="1"/></sensor></mujoco>"#,
        "negative noise in sensor (sensor #0)",
    ),
    (
        "a frame x-axis with a negative noise and a cutoff",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom size=".1"/><site name="s"/></body></worldbody><sensor><framexaxis objtype="site" objname="s" noise="-0.1" cutoff="1"/></sensor></mujoco>"#,
        "negative noise in sensor (sensor #0)",
    ),
];

/// `(case, document)`: each loads in MuJoCo 3.5.0 too.
const LOADED: &[(&str, &str)] = &[
    (
        "a joint position's noise and cutoff of 0, and the least positive cutoff",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody><sensor><jointpos joint="j" noise="0" cutoff="0"/><jointvel joint="j" cutoff="1e-300"/></sensor></mujoco>"#,
    ),
    (
        "a frame quaternion's cutoff of 0, and a ball quaternion's unwritten",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="ball"/><geom size=".1"/><site name="s"/></body></worldbody><sensor><framequat objtype="site" objname="s" cutoff="0"/><ballquat joint="j"/></sensor></mujoco>"#,
    ),
    (
        "a geom normal's cutoff, its search distance",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size=".1"/></body><geom name="w" pos="1 0 1" size=".1"/></worldbody><sensor><normal geom1="g" geom2="w" cutoff="2"/></sensor></mujoco>"#,
    ),
    (
        "a distance to a body holding a height field",
        r#"<mujoco><asset><hfield name="f" nrow="2" ncol="2" size="1 1 .1 .1" elevation="0 0 0 0"/></asset><worldbody><body name="a" pos="3 0 0"><geom type="hfield" hfield="f"/></body><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size=".1"/></body></worldbody><sensor><distance body1="a" geom2="g"/></sensor></mujoco>"#,
    ),
    (
        "a distance from a body whose number is a height-field geom's",
        r#"<mujoco><asset><hfield name="f" nrow="2" ncol="2" size="1 1 .1 .1" elevation="0 0 0 0"/></asset><worldbody><geom name="p" type="plane" size="1 1 .1"/><geom name="h" type="hfield" hfield="f" pos="5 0 0"/><body name="a" pos="0 0 1"><geom size=".1"/></body><body name="c" pos="1 0 1"><geom name="g" size=".1"/></body></worldbody><sensor><distance body1="a" geom2="g"/></sensor></mujoco>"#,
    ),
];

#[test]
fn refused_as_mujoco_refuses_them() {
    let mut wrong = Vec::new();
    for &(case, xml, expected) in REFUSED {
        match load_model(xml) {
            Err(e) if e.to_string() == expected => {}
            Err(e) => wrong.push(format!("{case}: {e}")),
            Ok(_) => wrong.push(format!("{case}: loads")),
        }
    }
    assert!(wrong.is_empty(), "{}", wrong.join("\n"));
}

#[test]
fn loaded_as_mujoco_loads_them() {
    for &(case, xml) in LOADED {
        if let Err(e) = load_model(xml) {
            panic!("{case}: {e}");
        }
    }
}
