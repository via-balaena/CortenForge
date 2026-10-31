//! Which fault a document with two of them is refused for, as MuJoCo 3.5.0
//! picks it (book L18; RL-5c's review, RA-S1 and RA-N3).
//!
//! MuJoCo checks the names first (`ProcessLists`), then each asset a geom
//! names (`IndexAssets`, for a geom of any type), then compiles the bodies
//! from the world down: a body's geoms (their size check among them), then
//! its sites, then its children. Every reason below was measured on the
//! census's unfused oracle (2026-10-31).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_mjcf::{
    ElementKind, MjcfBody, MjcfError, MjcfGeom, MjcfModel, MjcfSite, Prefix, load_model, validate,
};

/// `(what, document, the element MuJoCo names, its name)`: each refused for
/// a size of 0 on the element named.
const SIZE_FIRST: &[(&str, &str, &str, &str)] = &[
    (
        "a site in an earlier body than a geom",
        r#"<mujoco><worldbody><body name="B1"><site name="S" size="0"/></body><body name="B2"><geom name="G" size="0"/></body></worldbody></mujoco>"#,
        "site",
        "S",
    ),
    (
        "a geom and a site in one body",
        r#"<mujoco><worldbody><body name="B1"><site name="S" size="0"/><geom name="G" size="0"/></body></worldbody></mujoco>"#,
        "geom",
        "G",
    ),
    (
        "a parent's site before its child's geom",
        r#"<mujoco><worldbody><body name="A"><site name="S" size="0"/><body name="A1"><geom name="G" size="0"/></body></body></worldbody></mujoco>"#,
        "site",
        "S",
    ),
    (
        "the world's site before a body's geom",
        r#"<mujoco><worldbody><site name="S" size="0"/><body name="B"><geom name="G" size="0"/></body></worldbody></mujoco>"#,
        "site",
        "S",
    ),
    (
        "a child body's geom before a later body's site",
        r#"<mujoco><worldbody><body name="A"><body name="A1"><geom name="G" size="0"/></body></body><body name="B"><site name="S" size="0"/></body></worldbody></mujoco>"#,
        "geom",
        "G",
    ),
];

#[test]
fn of_two_sizes_mujocos_first_is_refused() {
    let mut missed = Vec::new();
    for &(what, xml, element, name) in SIZE_FIRST {
        match load_model(xml) {
            Err(e) => {
                let size = matches!(&e, MjcfError::InvalidValue { element: el, attribute, .. }
                    if el == element && attribute == "size");
                if !(size && e.to_string().contains(&format!("'{name}'"))) {
                    missed.push(format!("{what}: {e}"));
                }
            }
            Ok(_) => missed.push(format!("{what}: loads")),
        }
    }
    assert!(missed.is_empty(), "{}", missed.join("\n"));
}

#[test]
fn a_missing_mesh_comes_before_a_size() {
    let mesh = r#"<asset><mesh name="m" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 2 1 0 1 3 0 3 2 1 2 3"/></asset>"#;
    for (what, world) in [
        (
            "in a later body",
            r#"<body name="B1"><geom name="F" size="0"/></body><body name="B2"><geom type="mesh" mesh="nosuch"/></body>"#,
        ),
        (
            "in the same body",
            r#"<geom name="Z" size="0"/><geom type="box" mesh="nosuch"/>"#,
        ),
        ("on a box", r#"<geom type="box" mesh="nosuch"/>"#),
    ] {
        let xml = format!("<mujoco>{mesh}<worldbody>{world}</worldbody></mujoco>");
        match load_model(&xml) {
            Err(MjcfError::UndefinedReference {
                kind: ElementKind::Mesh,
                name,
                ..
            }) => assert_eq!(name, "nosuch", "{what}"),
            other => panic!("{what}: {other:?}"),
        }
    }
}

/// A height field a geom names is checked with the meshes, before any size
/// (MuJoCo: "hfield 'nosuch' not found in geom 1", measured, in one body
/// and across two).
#[test]
fn a_missing_height_field_comes_before_a_size() {
    for world in [
        r#"<geom name="Z" size="0"/><geom type="hfield" hfield="nosuch"/>"#,
        r#"<body name="B1"><geom name="F" size="0"/></body><body name="B2"><geom type="hfield" hfield="nosuch"/></body>"#,
    ] {
        match load_model(&format!("<mujoco><worldbody>{world}</worldbody></mujoco>")) {
            Err(MjcfError::UndefinedReference {
                kind: ElementKind::Hfield,
                name,
                ..
            }) => assert_eq!(name, "nosuch", "{world}"),
            other => panic!("{world}: {other:?}"),
        }
    }
}

#[test]
fn a_repeated_name_comes_before_a_size() {
    match load_model(
        r#"<mujoco><worldbody><body name="B1"><geom size="0"/><body name="B2"/></body><body name="B2"/></worldbody></mujoco>"#,
    ) {
        Err(MjcfError::DuplicateName {
            kind: ElementKind::Body,
            name,
            ..
        }) => assert_eq!(name, "B2"),
        other => panic!("{other:?}"),
    }
}

/// `validate` refuses a site the loader refuses, as it does a geom.
#[test]
fn validate_refuses_the_sites_the_loader_refuses() {
    for (site_type, size) in [
        (Some("plane"), Prefix::new(&[1.0, 1.0, 1.0])),
        (None, Prefix::new(&[0.0])),
    ] {
        let site = MjcfSite {
            site_type: site_type.map(str::to_string),
            size,
            ..MjcfSite::default()
        };
        let mut body = MjcfBody::new("b").with_geom(MjcfGeom::sphere(0.1));
        body.sites.push(site);
        let model = MjcfModel::new("m").with_body(body);
        match validate(&model) {
            Err(MjcfError::InvalidValue { element, .. }) => assert_eq!(element, "site"),
            other => panic!("{site_type:?} {size:?}: {other:?}"),
        }
    }
}
