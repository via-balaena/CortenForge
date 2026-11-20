//! A geom's height field as MuJoCo 3.5.0 checks it (ledger L337): a geom of
//! type `hfield` names one and a geom of any other type names none
//! (`mjCGeom::Compile`, `user_objects.cc:3661-3664`; an empty name names
//! none), whether the geom or its class names it. MuJoCo checks it geom by
//! geom, after a geom's condim and a mesh geom's mesh (`:3651-3659`) and
//! before the rest of the geom: its fit to a mesh, its size, a plane in a
//! moving body. sim-mjcf checks the condim later, and its parser refuses a
//! size that is not finite before any check (ledger L353).
//! MuJoCo checks a visual geom that `discardvisual` discards too, as it
//! discards one only after (the closing `IndexAssets`, `user_model.cc:4976`).
//!
//! Every document below was loaded by MuJoCo 3.5.0 on the census's unfused
//! oracle (2026-11-14). It refuses each of `REFUSED` with "hfield geom
//! '<name>' (id = <id>) must have valid hfieldid", whatever the geom's type;
//! sim-mjcf's reason is that message without the name and id, which its
//! location gives.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_mjcf::{MjcfError, load_model, parse_mjcf_str, validate};

/// sim-mjcf's reason.
const REASON: &str = "hfield geom must have valid hfieldid";

/// Each document's assets: a tetrahedron mesh, `m`, and a height field, `h`.
const ASSETS: &str = r#"<asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/><hfield name="h" nrow="2" ncol="2" elevation="0 0 0 0" size="1 1 0.1 0.1"/></asset>"#;

/// `(what, before the assets, the world body's contents, where sim-mjcf
/// refuses it)`, each refused by MuJoCo.
const REFUSED: &[(&str, &str, &str, &str)] = &[
    (
        "a sphere naming a height field",
        "",
        r#"<geom type="sphere" hfield="h" size="0.1"/>"#,
        "geom #0 in body 'world'",
    ),
    (
        "a box naming a height field",
        "",
        r#"<geom type="box" hfield="h" size="0.1 0.1 0.1"/>"#,
        "geom #0 in body 'world'",
    ),
    (
        "a capsule naming a height field",
        "",
        r#"<geom type="capsule" hfield="h" size="0.1 0.1"/>"#,
        "geom #0 in body 'world'",
    ),
    (
        "a cylinder naming a height field",
        "",
        r#"<geom type="cylinder" hfield="h" size="0.1 0.1"/>"#,
        "geom #0 in body 'world'",
    ),
    (
        "an ellipsoid naming a height field",
        "",
        r#"<geom type="ellipsoid" hfield="h" size="0.1 0.1 0.1"/>"#,
        "geom #0 in body 'world'",
    ),
    (
        "a plane naming a height field",
        "",
        r#"<geom type="plane" hfield="h" size="1 1 1"/>"#,
        "geom #0 in body 'world'",
    ),
    (
        "a mesh geom naming its mesh and a height field",
        "",
        r#"<geom type="mesh" mesh="m" hfield="h"/>"#,
        "geom #0 in body 'world'",
    ),
    (
        "an SDF geom naming its mesh and a height field",
        "",
        r#"<geom type="sdf" mesh="m" hfield="h"/>"#,
        "geom #0 in body 'world'",
    ),
    (
        "a height-field geom naming no height field",
        "",
        r#"<geom type="hfield"/>"#,
        "geom #0 in body 'world'",
    ),
    (
        "a height-field geom naming the empty name",
        "",
        r#"<geom type="hfield" hfield=""/>"#,
        "geom #0 in body 'world'",
    ),
    (
        "a height-field geom naming a mesh and no height field",
        "",
        r#"<geom type="hfield" mesh="m"/>"#,
        "geom #0 in body 'world'",
    ),
    (
        "a sphere whose class names a height field",
        r#"<default><geom hfield="h"/></default>"#,
        r#"<geom size="0.1"/>"#,
        "geom #0 in body 'world'",
    ),
    (
        "a sphere naming a height field and a mesh, before its fit to the mesh",
        "",
        r#"<geom type="sphere" hfield="h" mesh="m"/>"#,
        "geom #0 in body 'world'",
    ),
    (
        "a box naming a height field, before its size of 0",
        "",
        r#"<body name="b"><freejoint/><geom type="box" hfield="h" size="0 0.1 0.1"/></body>"#,
        "geom #0 in body 'b'",
    ),
    (
        "a box naming a height field, before the next body's sphere of size 0",
        "",
        r#"<body name="b"><freejoint/><geom type="box" hfield="h" size="0.1 0.1 0.1"/></body><body name="c"><freejoint/><geom size="0"/></body>"#,
        "geom #0 in body 'b'",
    ),
    (
        "a moving body's plane naming a height field, before the plane in a moving body",
        "",
        r#"<body name="b"><freejoint/><geom type="plane" hfield="h" size="1 1 1"/><geom size="0.1"/></body>"#,
        "geom #0 in body 'b'",
    ),
    (
        "a visual sphere naming a height field, before discardvisual discards it",
        r#"<compiler discardvisual="true"/>"#,
        r#"<geom contype="0" conaffinity="0" hfield="h" size="0.1"/><geom size="0.1"/>"#,
        "geom #0 in body 'world'",
    ),
    (
        "a sphere in a frame naming a height field",
        "",
        r#"<frame pos="1 0 0"><geom hfield="h" size="0.1"/></frame>"#,
        "geom #0 in body 'world'",
    ),
    (
        "the world's second geom naming a height field",
        "",
        r#"<geom size="0.1"/><geom type="box" hfield="h" size="0.1 0.1 0.1"/>"#,
        "geom #1 in body 'world'",
    ),
    (
        "a body's named second geom naming a height field",
        "",
        r#"<body name="b" pos="0 0 1"><freejoint/><geom size="0.1"/><geom name="x" hfield="h" size="0.1"/></body>"#,
        "geom 'x' in body 'b'",
    ),
];

/// `(what, before the assets, the world body's contents)`, each loaded by
/// MuJoCo.
const LOADS: &[(&str, &str, &str)] = &[
    (
        "a height-field geom naming one",
        "",
        r#"<geom type="hfield" hfield="h"/>"#,
    ),
    (
        "a height-field geom whose class names one",
        r#"<default><geom hfield="h"/></default>"#,
        r#"<geom type="hfield"/>"#,
    ),
];

fn document(before: &str, world: &str) -> String {
    format!("<mujoco>{before}{ASSETS}<worldbody>{world}</worldbody></mujoco>")
}

/// Refused by `load_model` and by `validate`, with sim-mjcf's reason, at the
/// geom MuJoCo names.
#[test]
fn a_geom_naming_a_height_field_it_is_not_is_refused_as_mujoco_refuses_it() {
    let mut missed = Vec::new();
    for &(what, before, world, at) in REFUSED {
        let xml = document(before, world);
        let want = format!("{REASON} ({at})");
        let loaded = load_model(&xml).err();
        let validated = validate(&parse_mjcf_str(&xml).unwrap()).err();
        for (how, got) in [("load_model", loaded), ("validate", validated)] {
            match got {
                Some(e @ MjcfError::InvalidModel { .. }) if e.to_string() == want => {}
                other => missed.push(format!("{what}: {how}: {other:?}")),
            }
        }
    }
    assert!(
        missed.is_empty(),
        "{} of {} not refused as MuJoCo refuses them:\n  {}",
        missed.len(),
        2 * REFUSED.len(),
        missed.join("\n  ")
    );
}

/// A height-field geom naming one, or whose class names one, loads with it.
#[test]
fn a_height_field_geom_naming_one_loads() {
    for &(what, before, world) in LOADS {
        let model = load_model(&document(before, world)).unwrap_or_else(|e| panic!("{what}: {e}"));
        assert_eq!(model.geom_type[0], sim_core::GeomType::Hfield, "{what}");
        assert_eq!(model.geom_hfield[0], Some(0), "{what}");
    }
}
