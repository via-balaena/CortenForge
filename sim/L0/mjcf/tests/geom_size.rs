//! A geom's size as MuJoCo 3.5.0 reads and checks it (book L18; ledger L333,
//! L183, L194).
//!
//! MuJoCo's default size is 0 in each place (`user_init.c:109-110`). A
//! `size` is read with `exact` false (`ReadAttr`, `xml_util.cc:798-817`), so
//! the numbers written take the first places over its class's, and a class's
//! over its parent's (`xml_native_reader.cc:1854`). A `fromto` then sets the
//! half-length (`user_objects.cc:3677-3715`), a primitive geom that names a
//! mesh is fitted to it (`FitGeom`), and `checksize` (`:152-167`, called at
//! `:3758`) refuses a plane whose third value is not above 0, and any other
//! geom whose first `mjGEOMINFO[type]` values are not (`user_objects.h:77`:
//! a sphere 1, a capsule or cylinder 2, an ellipsoid or box 3, a mesh none).
//!
//! Every value and reason below was measured on the census's unfused oracle
//! (2026-10-31). The sizes take arithmetic and a square root only, so they
//! are compared bit for bit.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_mjcf::{MjcfBody, MjcfError, MjcfGeom, MjcfModel, load_model, model_from_mjcf};

/// `(what, document, MuJoCo's geom_size of the first geom, in the places
/// compared)`. A sphere's radius only: sim-mjcf stores it in each place
/// (the census compares the radius only).
const LOADS: &[(&str, &str, &[f64])] = &[
    (
        "a bare sphere under the main class's size",
        r#"<mujoco><default><geom size="0.05"/></default><worldbody><geom/></worldbody></mujoco>"#,
        &[0.05],
    ),
    (
        "a capsule's one value over the main class's two",
        r#"<mujoco><default><geom size="0.05 0.3"/></default><worldbody><geom type="capsule" size="0.07"/></worldbody></mujoco>"#,
        &[0.07, 0.3, 0.0],
    ),
    (
        "a capsule's one value over a class's parent's two",
        r#"<mujoco><default><default class="a"><geom size="0.05 0.3"/><default class="b"><geom rgba="1 0 0 1"/></default></default></default><worldbody><geom class="b" type="capsule" size="0.07"/></worldbody></mujoco>"#,
        &[0.07, 0.3, 0.0],
    ),
    (
        "a geom of a class under the class's size, not the main class's",
        r#"<mujoco><default><geom size="0.2"/><default class="c"><geom size="0.05"/></default></default><worldbody><geom class="c"/><geom pos="1 0 0"/></worldbody></mujoco>"#,
        &[0.05],
    ),
    (
        "a box's two values over the main class's three",
        r#"<mujoco><default><geom size="1 2 3"/></default><worldbody><geom type="box" size="4 5"/></worldbody></mujoco>"#,
        &[4.0, 5.0, 3.0],
    ),
    (
        "a plane's two values over the main class's three",
        r#"<mujoco><default><geom size="1 1 0.1"/></default><worldbody><geom type="plane" size="5 5"/></worldbody></mujoco>"#,
        &[5.0, 5.0, 0.1],
    ),
    (
        "a fromto capsule's radius from its class (ledger L333)",
        r#"<mujoco><default><default class="k"><geom type="capsule" size="0.05"/></default></default><worldbody><body pos="0 0 1"><freejoint/><geom class="k" fromto="0 0 0 0 0 0.4"/></body></worldbody></mujoco>"#,
        &[0.05, 0.2, 0.0],
    ),
    (
        "a capsule's written third value kept beside a fromto",
        r#"<mujoco><worldbody><geom type="capsule" size="0.05 0 0.7" fromto="0 0 0 0 0 0.4"/></worldbody></mujoco>"#,
        &[0.05, 0.2, 0.7],
    ),
    (
        "a cylinder's written third value kept beside a fromto",
        r#"<mujoco><worldbody><geom type="cylinder" size="0.05 0.2 0.7" fromto="0 0 0 0 0 0.4"/></worldbody></mujoco>"#,
        &[0.05, 0.2, 0.7],
    ),
    (
        "a fromto box of one value",
        r#"<mujoco><worldbody><geom type="box" size="0.1" fromto="0 0 0 0 0 0.4"/></worldbody></mujoco>"#,
        &[0.1, 0.1, 0.2],
    ),
    (
        "a plane whose first two values are 0",
        r#"<mujoco><worldbody><geom type="plane" size="0 0 1"/></worldbody></mujoco>"#,
        &[0.0, 0.0, 1.0],
    ),
    (
        "a geom in a frame under the frame's childclass's size",
        r#"<mujoco><default><default class="c"><geom size="0.3"/></default></default><worldbody><frame childclass="c"><geom/></frame></worldbody></mujoco>"#,
        &[0.3],
    ),
];

#[test]
fn a_geoms_size_is_mujocos() {
    let mut differ = Vec::new();
    for &(what, xml, mujoco) in LOADS {
        let model = load_model(xml).unwrap_or_else(|e| panic!("{what}: {e}"));
        let got = model.geom_size[0];
        if (0..mujoco.len()).any(|k| got[k].to_bits() != mujoco[k].to_bits()) {
            differ.push(format!("{what}: {got:?}, MuJoCo {mujoco:?}"));
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

/// The second geom of A3 §3's `s3_08` takes the main class's size, not the
/// named class's (MuJoCo: 0.05 and 0.2, measured).
#[test]
fn s3_08s_second_geom_takes_the_main_class() {
    let model = load_model(
        r#"<mujoco><default><geom size="0.2"/><default class="c"><geom size="0.05"/></default></default><worldbody><geom class="c"/><geom pos="1 0 0"/></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_eq!(
        [
            model.geom_size[0].x.to_bits(),
            model.geom_size[1].x.to_bits()
        ],
        [0.05_f64.to_bits(), 0.2_f64.to_bits()]
    );
}

/// An SDF geom naming a mesh is not one MuJoCo fits: it loads, as MuJoCo
/// loads it (`D-MESH-FIT` covers the primitive types only).
#[test]
fn an_sdf_geom_naming_a_mesh_loads() {
    let model = load_model(
        r#"<mujoco><asset><mesh name="m" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 2 1 0 1 3 0 3 2 1 2 3"/></asset><worldbody><geom type="sdf" mesh="m"/></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_eq!(model.geom_type[0], sim_core::GeomType::Sdf);
}

/// A class's size reaches a moving body's mass (ledger L333): MuJoCo 3.5.0
/// gives 3.665191429188093 (a capsule of radius 0.05 and half-length 0.2 at
/// density 1000); the parsed default of 0.1 gave another.
#[test]
fn a_class_size_reaches_the_bodys_mass() {
    let model = load_model(
        r#"<mujoco><default><default class="k"><geom type="capsule" size="0.05"/></default></default><worldbody><body pos="0 0 1"><freejoint/><geom class="k" fromto="0 0 0 0 0 0.4"/></body></worldbody></mujoco>"#,
    )
    .unwrap();
    let mass = model.body_mass[1];
    assert!(
        (mass - 3.665_191_429_188_093).abs() <= 1e-12 * mass,
        "{mass}"
    );
}

/// `(what, document, MuJoCo's reason)`.
const REFUSED: &[(&str, &str, &str)] = &[
    (
        "a bare geom",
        r"<mujoco><worldbody><geom/></worldbody></mujoco>",
        "size 0 must be positive",
    ),
    (
        "a sphere of size 0",
        r#"<mujoco><worldbody><geom size="0"/></worldbody></mujoco>"#,
        "size 0 must be positive",
    ),
    (
        "a sphere of a negative size",
        r#"<mujoco><worldbody><geom size="-0.1"/></worldbody></mujoco>"#,
        "size 0 must be positive",
    ),
    (
        "a box with a 0",
        r#"<mujoco><worldbody><geom type="box" size=".1 0 .1"/></worldbody></mujoco>"#,
        "size 1 must be positive",
    ),
    (
        "a box of one value",
        r#"<mujoco><worldbody><geom type="box" size=".1"/></worldbody></mujoco>"#,
        "size 1 must be positive",
    ),
    (
        "a capsule of one value",
        r#"<mujoco><worldbody><geom type="capsule" size=".1"/></worldbody></mujoco>"#,
        "size 1 must be positive",
    ),
    (
        "a cylinder of one value",
        r#"<mujoco><worldbody><geom type="cylinder" size=".1"/></worldbody></mujoco>"#,
        "size 1 must be positive",
    ),
    (
        "a fromto capsule with no size",
        r#"<mujoco><worldbody><geom type="capsule" fromto="0 0 0 0 0 1"/></worldbody></mujoco>"#,
        "size 0 must be positive",
    ),
    (
        "a fromto ellipsoid with no size",
        r#"<mujoco><worldbody><geom type="ellipsoid" fromto="0 0 0 0 0 1"/></worldbody></mujoco>"#,
        "size 0 must be positive",
    ),
    (
        "a moving body's bare geom",
        r#"<mujoco><worldbody><body pos="0 0 1"><freejoint/><geom/></body></worldbody></mujoco>"#,
        "size 0 must be positive",
    ),
    (
        "a geom of a class with no size",
        r#"<mujoco><default><default class="c"><geom rgba="1 0 0 1"/></default></default><worldbody><geom class="c"/></worldbody></mujoco>"#,
        "size 0 must be positive",
    ),
    (
        "a plane with no size",
        r#"<mujoco><worldbody><geom type="plane"/></worldbody></mujoco>"#,
        "plane size(3) must be positive",
    ),
    (
        "a plane of two values",
        r#"<mujoco><worldbody><geom type="plane" size="2 3"/></worldbody></mujoco>"#,
        "plane size(3) must be positive",
    ),
    (
        "a plane of one value",
        r#"<mujoco><worldbody><geom type="plane" size="1"/></worldbody></mujoco>"#,
        "plane size(3) must be positive",
    ),
    (
        "a plane whose third value is negative",
        r#"<mujoco><worldbody><geom type="plane" size="1 1 -1"/></worldbody></mujoco>"#,
        "plane size(3) must be positive",
    ),
    (
        "an ellipsoid of two values",
        r#"<mujoco><worldbody><geom type="ellipsoid" size="0.1 0.2"/></worldbody></mujoco>"#,
        "size 2 must be positive",
    ),
    (
        "an element's 0 over its class's size",
        r#"<mujoco><default><geom size="0.05"/></default><worldbody><geom size="0"/></worldbody></mujoco>"#,
        "size 0 must be positive",
    ),
    (
        "a class's size of 0",
        r#"<mujoco><default><geom size="0"/></default><worldbody><geom/></worldbody></mujoco>"#,
        "size 0 must be positive",
    ),
    // Ledger L194 measured `load_model` of this capsule not returning within
    // 20 s (2026-10-18); at this commit's parent it loads, of size 0
    // (2026-10-31).
    (
        "a capsule of size 0 0",
        r#"<mujoco><worldbody><geom type="capsule" size="0 0"/></worldbody></mujoco>"#,
        "size 0 must be positive",
    ),
    (
        "a bare geom in a frame",
        r#"<mujoco><worldbody><frame pos="1 0 0"><geom/></frame></worldbody></mujoco>"#,
        "size 0 must be positive",
    ),
];

fn assert_refused_as_mujoco(cases: &[(&str, &str, &str)]) {
    let mut missed = Vec::new();
    for &(what, xml, reason) in cases {
        match load_model(xml) {
            Err(MjcfError::InvalidValue {
                element,
                attribute,
                reason: r,
                ..
            }) if element == "geom" && attribute == "size" && r == reason => {}
            Err(other) => missed.push(format!("{what}: {other}")),
            Ok(m) => missed.push(format!("{what}: loads, size {:?}", m.geom_size[0])),
        }
    }
    assert!(
        missed.is_empty(),
        "{} of {} not refused as MuJoCo refuses them:\n  {}",
        missed.len(),
        cases.len(),
        missed.join("\n  ")
    );
}

#[test]
fn a_size_mujoco_refuses_is_refused() {
    assert_refused_as_mujoco(REFUSED);
}

/// MuJoCo fits a primitive geom that names a mesh to the mesh before its
/// size check, replacing its size (`FitGeom`; measured on this tetrahedron: a
/// box naming it, with no size or with `size="0.1 0.2 0.3"`, loads as
/// (0.194, 0.387, 0.387), a capsule of size 0 0 loads too). sim-mjcf does not
/// fit one (ledger L337), so it refuses such a geom as not supported
/// (`D-MESH-FIT`).
#[test]
fn a_primitive_fitted_to_a_mesh_is_not_supported() {
    let mesh = r#"<asset><mesh name="m" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 2 1 0 1 3 0 3 2 1 2 3"/></asset>"#;
    let mut missed = Vec::new();
    for geom in [
        r#"<geom type="box" mesh="m"/>"#,
        r#"<geom type="sphere" mesh="m"/>"#,
        r#"<geom type="capsule" mesh="m" size="0 0"/>"#,
        r#"<geom type="box" mesh="m" size="0.1 0.2 0.3"/>"#,
    ] {
        let xml = format!("<mujoco>{mesh}<worldbody>{geom}</worldbody></mujoco>");
        match load_model(&xml) {
            Err(e @ MjcfError::Unsupported { .. }) if e.to_string().contains("mesh") => {}
            Err(other) => missed.push(format!("{geom}: {other}")),
            Ok(m) => missed.push(format!("{geom}: loads, size {:?}", m.geom_size[0])),
        }
    }
    assert!(missed.is_empty(), "{}", missed.join("\n"));
}

/// A plane or height field naming a mesh: MuJoCo refuses it, "invalid geom
/// type in fitting mesh" (`FitGeom` fits neither, `user_mesh.cc:1008-1009`;
/// measured), and so does sim-mjcf, as MuJoCo does.
#[test]
fn a_plane_or_height_field_naming_a_mesh_is_refused_as_mujoco_refuses_it() {
    let assets = r#"<asset><mesh name="m" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 2 1 0 1 3 0 3 2 1 2 3"/><hfield name="h" nrow="2" ncol="2" size="1 1 1 0.1" elevation="0 0 0 0"/></asset>"#;
    let mut missed = Vec::new();
    for geom in [
        r#"<geom type="plane" mesh="m" size="1 1 1"/>"#,
        r#"<geom type="plane" mesh="m"/>"#,
        r#"<geom type="hfield" hfield="h" mesh="m"/>"#,
    ] {
        let xml = format!("<mujoco>{assets}<worldbody>{geom}</worldbody></mujoco>");
        match load_model(&xml) {
            Err(MjcfError::InvalidValue {
                element,
                attribute,
                reason,
                ..
            }) if element == "geom"
                && attribute == "type"
                && reason == "invalid geom type in fitting mesh" => {}
            other => missed.push(format!("{geom}: {other:?}")),
        }
    }
    assert!(missed.is_empty(), "{}", missed.join("\n"));
}

/// A geom built in code with no size is refused as a document's is, and
/// one built with its size loads.
#[test]
fn a_geom_built_in_code_is_checked_as_a_documents() {
    let bare = MjcfModel::new("bare").with_body(MjcfBody::new("b").with_geom(MjcfGeom::default()));
    match model_from_mjcf(&bare, None) {
        Err(MjcfError::InvalidValue {
            element,
            attribute,
            reason,
            ..
        }) => assert_eq!(
            (element.as_str(), attribute.as_str(), reason.as_str()),
            ("geom", "size", "size 0 must be positive")
        ),
        other => panic!("{other:?}"),
    }
    let sized =
        MjcfModel::new("sized").with_body(MjcfBody::new("b").with_geom(MjcfGeom::sphere(0.1)));
    let model = model_from_mjcf(&sized, None).unwrap();
    assert_eq!(model.geom_size[0][0].to_bits(), 0.1_f64.to_bits());
}
