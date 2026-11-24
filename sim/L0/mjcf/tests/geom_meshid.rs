//! A mesh or SDF geom's mesh as MuJoCo 3.5.0 checks it (ledger L357): a geom
//! of type `mesh` or `sdf` names one (`mjCGeom::Compile`,
//! `user_objects.cc:3656-3659`; an empty name names none), whether the geom
//! or its class names it. MuJoCo checks it geom by geom, after the geom's
//! condim and directly before its height field. sim-mjcf's loader refuses a
//! mesh geom's as it builds the geom, where it refused one before
//! Rigid-loading PR RL-9d in other words, and an SDF geom's as it checks
//! each body (`check_bodies`), after the geom's condim, as MuJoCo does.
//! Where it checks another fault first, it names that one:
//! `NAMED_OTHERWISE`. `validate`
//! checks both kinds after its other checks. A named mesh the model lacks
//! is refused before any body, in both.
//!
//! Every document below was loaded by MuJoCo 3.5.0 on the census's unfused
//! oracle (2026-11-19). It refuses each of `REFUSED` and `NAMED_OTHERWISE`
//! with "mesh geom '<name>' (id = <id>) must have valid meshid"; sim-mjcf's
//! reason is that message without the name and id, which its location
//! gives.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_mjcf::{MjcfError, load_model, parse_mjcf_str, validate};

/// sim-mjcf's reason.
const REASON: &str = "mesh geom must have valid meshid";

/// Each document's assets: a tetrahedron mesh, `m`, and a height field, `h`.
const ASSETS: &str = r#"<asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/><hfield name="h" nrow="2" ncol="2" elevation="0 0 0 0" size="1 1 0.1 0.1"/></asset>"#;

/// `(what, before the assets, the world body's contents, where sim-mjcf
/// refuses it)`, each refused by MuJoCo.
const REFUSED: &[(&str, &str, &str, &str)] = &[
    (
        "an SDF geom naming no mesh",
        "",
        r#"<geom type="sdf"/>"#,
        "geom #0 in body 'world'",
    ),
    (
        "an SDF geom naming the empty name",
        "",
        r#"<geom type="sdf" mesh=""/>"#,
        "geom #0 in body 'world'",
    ),
    (
        "a mesh geom naming no mesh",
        "",
        r#"<geom type="mesh"/>"#,
        "geom #0 in body 'world'",
    ),
    (
        "a mesh geom naming the empty name",
        "",
        r#"<geom type="mesh" mesh=""/>"#,
        "geom #0 in body 'world'",
    ),
    (
        "a geom whose class makes it an SDF",
        r#"<default><geom type="sdf"/></default>"#,
        "<geom/>",
        "geom #0 in body 'world'",
    ),
    (
        "a geom whose class makes it a mesh geom naming the empty name",
        r#"<default><geom type="mesh" mesh=""/></default>"#,
        "<geom/>",
        "geom #0 in body 'world'",
    ),
    (
        "an SDF geom in a frame",
        "",
        r#"<frame pos="1 0 0"><geom type="sdf"/></frame>"#,
        "geom #0 in body 'world'",
    ),
    (
        "the world's second geom",
        "",
        r#"<geom size="0.1"/><geom type="sdf"/>"#,
        "geom #1 in body 'world'",
    ),
    (
        "a body's named second geom",
        "",
        r#"<body name="b" pos="0 0 1"><freejoint/><geom size="0.1"/><geom name="x" type="mesh"/></body>"#,
        "geom 'x' in body 'b'",
    ),
    (
        "an SDF geom, before its shellinertia",
        "",
        r#"<body name="b"><freejoint/><geom type="sdf" shellinertia="true"/><geom size=".1"/></body>"#,
        "geom #0 in body 'b'",
    ),
    (
        "a visual SDF geom, checked before discardvisual discards it",
        r#"<compiler discardvisual="true"/>"#,
        r#"<geom type="sdf" contype="0" conaffinity="0"/><geom size="0.1"/>"#,
        "geom #0 in body 'world'",
    ),
];

/// `(what, before the assets, the world body's contents, the loader's error,
/// `validate`'s error)`, each refused by MuJoCo for
/// the SDF geom's mesh: the loader names another fault, which it checks
/// before it builds the model, as it did before Rigid-loading PR RL-9d;
/// `validate` names the other fault where it checks it.
const NAMED_OTHERWISE: &[(&str, &str, &str, &str, &str)] = &[
    (
        "an SDF geom naming a height field",
        "",
        r#"<geom type="sdf" hfield="h"/>"#,
        "hfield geom must have valid hfieldid (geom #0 in body 'world')",
        "hfield geom must have valid hfieldid (geom #0 in body 'world')",
    ),
    (
        "an SDF geom, then the next geom's height field",
        "",
        r#"<body name="b"><freejoint/><geom type="sdf"/><geom type="box" size="0.1 0.1 0.1" hfield="h"/></body>"#,
        "hfield geom must have valid hfieldid (geom #1 in body 'b')",
        "hfield geom must have valid hfieldid (geom #1 in body 'b')",
    ),
    (
        "an SDF geom, then the next body's sphere of size 0",
        "",
        r#"<body name="b"><freejoint/><geom type="sdf"/></body><body name="c"><freejoint/><geom size="0"/></body>"#,
        "invalid size on <geom>: size 0 must be positive (geom #1 in body 'c')",
        "invalid size on <geom>: size 0 must be positive (geom #1 in body 'c')",
    ),
];

/// `(what, before the assets, the world body's contents)`, each loaded by
/// MuJoCo.
const LOADS: &[(&str, &str, &str)] = &[
    (
        "an SDF geom naming its mesh",
        "",
        r#"<geom type="sdf" mesh="m"/>"#,
    ),
    (
        "a mesh geom naming its mesh",
        "",
        r#"<geom type="mesh" mesh="m"/>"#,
    ),
    (
        "a sphere naming the empty name",
        "",
        r#"<geom type="sphere" size="0.1" mesh=""/>"#,
    ),
    (
        "an SDF geom whose class names its mesh",
        r#"<default><geom mesh="m"/></default>"#,
        r#"<geom type="sdf"/>"#,
    ),
];

fn document(before: &str, world: &str) -> String {
    format!("<mujoco>{before}{ASSETS}<worldbody>{world}</worldbody></mujoco>")
}

/// Refused by `load_model` and by `validate`, with sim-mjcf's reason, at the
/// geom MuJoCo names.
#[test]
fn a_mesh_or_sdf_geom_naming_no_mesh_is_refused_as_mujoco_refuses_it() {
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

/// The loader and `validate` name what `NAMED_OTHERWISE` says.
#[test]
fn an_sdf_geom_naming_no_mesh_beside_an_earlier_check_is_named_as_ledgered() {
    let mut missed = Vec::new();
    for &(what, before, world, loaded, validated) in NAMED_OTHERWISE {
        let xml = document(before, world);
        let got_loaded = load_model(&xml).err().map(|e| e.to_string());
        if got_loaded.as_deref() != Some(loaded) {
            missed.push(format!("{what}: load_model: {got_loaded:?}"));
        }
        let got_validated = validate(&parse_mjcf_str(&xml).unwrap())
            .err()
            .map(|e| e.to_string());
        if got_validated.as_deref() != Some(validated) {
            missed.push(format!("{what}: validate: {got_validated:?}"));
        }
    }
    assert!(missed.is_empty(), "{}", missed.join("\n"));
}

/// A mesh or SDF geom naming its mesh, or whose class names it, loads with
/// it; a primitive naming the empty name names none.
#[test]
fn a_geom_naming_its_mesh_or_none_it_needs_loads() {
    for &(what, before, world) in LOADS {
        let model = load_model(&document(before, world)).unwrap_or_else(|e| panic!("{what}: {e}"));
        let needs_one = matches!(
            model.geom_type[0],
            sim_core::GeomType::Mesh | sim_core::GeomType::Sdf
        );
        assert_eq!(
            model.geom_mesh[0],
            needs_one.then_some(0),
            "{what}: {:?}",
            model.geom_type[0]
        );
    }
}
