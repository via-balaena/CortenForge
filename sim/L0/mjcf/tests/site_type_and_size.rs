//! A site's type and size as MuJoCo 3.5.0 checks them (book L18; ledger
//! L235).
//!
//! MuJoCo's `mjCSite::Compile` refuses a plane, mesh or height-field site
//! (`user_objects.cc:3933-3935`), takes a signed distance field, resolves a
//! `fromto`, and then runs `checksize` (`:152-167`, called at `:3991`): each
//! of the first `mjGEOMINFO[type]` values must be above 0 (`user_objects.h:77`:
//! a sphere 1, a capsule or cylinder 2, an ellipsoid or box 3, an SDF none),
//! a place neither the site nor its class writes being MuJoCo's 0.005
//! (`mjs_defaultSite`, `user_init.c:159`). The reason it gives a site is its
//! geom's ("size 0 must be positive in geom"); sim-mjcf names the site.
//!
//! Every value and reason below was measured on the census's unfused oracle
//! (2026-10-31).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_core::GeomType;
use sim_mjcf::{MjcfError, load_model};

/// `(what, document, attribute, MuJoCo's reason)`.
const REFUSED: &[(&str, &str, &str, &str)] = &[
    (
        "a plane",
        r#"<mujoco><worldbody><site type="plane" size="1 1 1"/></worldbody></mujoco>"#,
        "type",
        "meshes, hfields and planes not allowed in site",
    ),
    (
        "a mesh",
        r#"<mujoco><worldbody><site type="mesh" size="0.1"/></worldbody></mujoco>"#,
        "type",
        "meshes, hfields and planes not allowed in site",
    ),
    (
        "a height field",
        r#"<mujoco><worldbody><site type="hfield" size="0.1"/></worldbody></mujoco>"#,
        "type",
        "meshes, hfields and planes not allowed in site",
    ),
    (
        "a plane from its class",
        r#"<mujoco><default><site type="plane"/></default><worldbody><site/></worldbody></mujoco>"#,
        "type",
        "meshes, hfields and planes not allowed in site",
    ),
    (
        "a plane of size 0 (its type first)",
        r#"<mujoco><worldbody><site type="plane" size="0"/></worldbody></mujoco>"#,
        "type",
        "meshes, hfields and planes not allowed in site",
    ),
    (
        "a sphere of size 0",
        r#"<mujoco><worldbody><site size="0"/></worldbody></mujoco>"#,
        "size",
        "size 0 must be positive",
    ),
    (
        "a sphere of a negative size",
        r#"<mujoco><worldbody><site size="-0.01"/></worldbody></mujoco>"#,
        "size",
        "size 0 must be positive",
    ),
    (
        "a box with a 0",
        r#"<mujoco><worldbody><site type="box" size="0.1 0 0.1"/></worldbody></mujoco>"#,
        "size",
        "size 1 must be positive",
    ),
    (
        "a capsule whose second value is 0",
        r#"<mujoco><worldbody><site type="capsule" size="0.1 0"/></worldbody></mujoco>"#,
        "size",
        "size 1 must be positive",
    ),
    (
        "an ellipsoid whose third value is negative",
        r#"<mujoco><worldbody><site type="ellipsoid" size="0.1 0.1 -1"/></worldbody></mujoco>"#,
        "size",
        "size 2 must be positive",
    ),
    (
        "a size of 0 from its class",
        r#"<mujoco><default><site size="0"/></default><worldbody><site/></worldbody></mujoco>"#,
        "size",
        "size 0 must be positive",
    ),
    (
        "a fromto capsule of radius 0",
        r#"<mujoco><worldbody><site type="capsule" size="0" fromto="0 0 0 0 0 1"/></worldbody></mujoco>"#,
        "size",
        "size 0 must be positive",
    ),
];

#[test]
fn a_site_mujoco_refuses_is_refused() {
    let mut missed = Vec::new();
    for &(what, xml, attribute, reason) in REFUSED {
        match load_model(xml) {
            Err(MjcfError::InvalidValue {
                element,
                attribute: a,
                reason: r,
                ..
            }) if element == "site" && a == attribute && r == reason => {}
            Err(other) => missed.push(format!("{what}: {other}")),
            Ok(m) => missed.push(format!(
                "{what}: loads, type {:?}, size {:?}",
                m.site_type[0], m.site_size[0]
            )),
        }
    }
    assert!(
        missed.is_empty(),
        "{} of {} not refused as MuJoCo refuses them:\n  {}",
        missed.len(),
        REFUSED.len(),
        missed.join("\n  ")
    );
}

/// `(what, document, MuJoCo's site_type, site_size)`.
const LOADS: &[(&str, &str, GeomType, [f64; 3])] = &[
    (
        "a signed distance field",
        r#"<mujoco><worldbody><site type="sdf" size="0.1"/></worldbody></mujoco>"#,
        GeomType::Sdf,
        [0.1, 0.005, 0.005],
    ),
    (
        "a signed distance field with no size",
        r#"<mujoco><worldbody><site type="sdf"/></worldbody></mujoco>"#,
        GeomType::Sdf,
        [0.005, 0.005, 0.005],
    ),
    (
        "a box of one value, the rest MuJoCo's 0.005",
        r#"<mujoco><worldbody><site type="box" size="0.1"/></worldbody></mujoco>"#,
        GeomType::Box,
        [0.1, 0.005, 0.005],
    ),
    (
        "a sphere whose unread second value is 0",
        r#"<mujoco><worldbody><site size="0.1 0"/></worldbody></mujoco>"#,
        GeomType::Sphere,
        [0.1, 0.0, 0.005],
    ),
];

#[test]
fn a_sites_type_and_size_are_mujocos() {
    let mut differ = Vec::new();
    for &(what, xml, kind, size) in LOADS {
        let model = load_model(xml).unwrap_or_else(|e| panic!("{what}: {e}"));
        let got = model.site_size[0];
        if model.site_type[0] != kind || (0..3).any(|k| got[k].to_bits() != size[k].to_bits()) {
            differ.push(format!(
                "{what}: {:?} {got:?}, MuJoCo {kind:?} {size:?}",
                model.site_type[0]
            ));
        }
    }
    assert!(
        differ.is_empty(),
        "{} of {} differ:\n  {}",
        differ.len(),
        LOADS.len(),
        differ.join("\n  ")
    );
}

/// A touch sensor on a site of type sdf: MuJoCo 3.5.0 stops with a fatal
/// error once the site's body has a contact (`mju_rayGeom: unexpected geom
/// type 8`, measured with this document); sim-mjcf steps, and the touch
/// reads 0, a site of no shape holding no contact point (`D-SITE-SDF-TOUCH`).
#[test]
fn a_touch_on_a_signed_distance_field_site_reads_zero() {
    let model = load_model(
        r#"<mujoco><worldbody><geom type="plane" size="1 1 1"/><body pos="0 0 0.09"><freejoint/><geom size="0.1"/><site name="s" type="sdf" size="0.2"/></body></worldbody><sensor><touch site="s"/></sensor></mujoco>"#,
    )
    .unwrap();
    let mut data = model.make_data();
    for _ in 0..5 {
        data.step(&model).expect("step");
    }
    assert!(
        data.ncon > 0,
        "no contact: the fixture does not reach the sensor"
    );
    assert_eq!(data.sensordata[0].to_bits(), 0.0_f64.to_bits());
}
