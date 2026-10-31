//! A cable composite's geom and site sizes and its `quat`, as MuJoCo 3.5.0
//! makes them (book L18; ledger L313).
//!
//! MuJoCo reads a cable's `<geom>` size over its composite default, 0 in
//! each place (`xml_native_reader.cc`, `OneComposite`: `ReadAttr(egeom, "size",
//! 3, dgeom.size, …, false, false)` onto `comp.def[0].spec.geom`), makes each
//! segment's capsule or cylinder from it with its half-length (or a box's
//! first value), and then checks each geom and site as any other
//! (`checksize`, `user_objects.cc:152-167`). A composite's `quat` turns its
//! vertices as written (`mjuu_rotVecQuat`), so a zero quaternion turns none.
//!
//! Every value and reason below was measured on the census's unfused oracle
//! (2026-10-31): 5 vertices, 4 segments of half-length 0.125.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_mjcf::{MjcfError, load_model};

fn cable(attrs: &str, children: &str) -> String {
    format!(
        r#"<mujoco><worldbody><composite type="cable" count="5 1 1" curve="s" size="1"{attrs}>{children}</composite></worldbody></mujoco>"#
    )
}

#[test]
fn a_cable_geom_or_site_mujoco_refuses_is_refused() {
    let mut missed = Vec::new();
    for (what, xml, element, reason) in [
        (
            "a capsule with no size",
            cable("", r#"<geom type="capsule"/>"#),
            "geom",
            "size 0 must be positive",
        ),
        (
            "a capsule of radius 0",
            cable("", r#"<geom type="capsule" size="0"/>"#),
            "geom",
            "size 0 must be positive",
        ),
        (
            "a box of two values",
            cable("", r#"<geom type="box" size="0.01 0.02"/>"#),
            "geom",
            "size 2 must be positive",
        ),
        (
            "a box of one value",
            cable("", r#"<geom type="box" size="0.01"/>"#),
            "geom",
            "size 1 must be positive",
        ),
        (
            "a site of size 0",
            cable("", r#"<geom type="capsule" size="0.01"/><site size="0"/>"#),
            "site",
            "size 0 must be positive",
        ),
    ] {
        match load_model(&xml) {
            Err(MjcfError::InvalidValue {
                element: e,
                attribute,
                reason: r,
                ..
            }) if e == element && attribute == "size" && r == reason => {}
            Err(other) => missed.push(format!("{what}: {other}")),
            Ok(m) => missed.push(format!("{what}: loads, geom size {:?}", m.geom_size[0])),
        }
    }
    assert!(missed.is_empty(), "{}", missed.join("\n"));
}

#[test]
fn a_cable_geoms_size_is_mujocos() {
    for kind in ["capsule", "cylinder"] {
        let model = load_model(&cable("", &format!(r#"<geom type="{kind}" size="0.01"/>"#)))
            .unwrap_or_else(|e| panic!("{kind}: {e}"));
        assert_eq!(model.ngeom, 4, "{kind}");
        for g in 0..4 {
            let s = model.geom_size[g];
            assert_eq!(
                [s.x.to_bits(), s.y.to_bits(), s.z.to_bits()],
                [0.01_f64.to_bits(), 0.125_f64.to_bits(), 0.0_f64.to_bits()],
                "{kind} geom {g}: {s:?}"
            );
        }
    }
}

/// A cable capsule's or cylinder's written third size value is kept, as
/// MuJoCo keeps it (measured: (0.01, 0.125, 0.3) and (0.01, 0.125, 0.4)).
#[test]
fn a_cable_geoms_third_size_is_kept() {
    for (geom, third) in [
        (r#"<geom type="capsule" size="0.01 0.5 0.3"/>"#, 0.3_f64),
        (r#"<geom type="cylinder" size="0.01 0 0.4"/>"#, 0.4),
    ] {
        let model = load_model(&cable("", geom)).unwrap_or_else(|e| panic!("{geom}: {e}"));
        for g in 0..4 {
            let s = model.geom_size[g];
            assert_eq!(
                [s.x.to_bits(), s.y.to_bits(), s.z.to_bits()],
                [0.01_f64.to_bits(), 0.125_f64.to_bits(), third.to_bits()],
                "{geom} geom {g}: {s:?}"
            );
        }
    }
}

/// `(quat, MuJoCo's first cable body's quat)`: a zero quaternion and an
/// unnormalised (2, 0, 0, 0) turn nothing; (0, 0, 0, 1) turns the cable half
/// a turn about z.
#[test]
fn a_cables_quat_turns_it_as_mujocos() {
    let mut differ = Vec::new();
    for (quat, first) in [
        ("0 0 0 0", [1.0, 0.0, 0.0, 0.0]),
        ("2 0 0 0", [1.0, 0.0, 0.0, 0.0]),
        ("0 0 0 1", [0.0, 0.0, 0.0, 1.0]),
    ] {
        match load_model(&cable(
            &format!(r#" quat="{quat}""#),
            r#"<geom type="capsule" size="0.01"/>"#,
        )) {
            Ok(model) => {
                let q = model.body_quat[1];
                let got = [q.w, q.i, q.j, q.k];
                if got.iter().zip(first).any(|(a, b)| (a - b).abs() > 1e-12) {
                    differ.push(format!("quat {quat}: first body {got:?}, MuJoCo {first:?}"));
                }
            }
            Err(e) => differ.push(format!("quat {quat}: {e}")),
        }
    }
    assert!(differ.is_empty(), "{}", differ.join("\n"));
}

/// A cable's `<geom>` that writes no type is MuJoCo's default sphere, which
/// a cable refuses (measured: "Cable geom type must be sphere, capsule or
/// box", MuJoCo's words for its cylinder, capsule or box rule).
#[test]
fn a_cable_geom_of_no_type_is_refused() {
    match load_model(&cable("", r#"<geom size="0.01"/>"#)) {
        Err(MjcfError::InvalidValue {
            element, attribute, ..
        }) => {
            assert_eq!((element.as_str(), attribute.as_str()), ("geom", "type"));
        }
        other => panic!("{other:?}"),
    }
}
