//! An unnamed mesh or height field takes the name MuJoCo 3.5.0 gives it:
//! its file's name, without the directory and the last extension (book L13).
//!
//! Every case was measured on MuJoCo 3.5.0 with the same files.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use sim_mjcf::{MjcfError, load_model};
use std::path::Path;

/// A tetrahedron, as an OBJ file.
const TET: &str = "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n";

fn files(dir: &Path) {
    std::fs::create_dir_all(dir.join("sub")).unwrap();
    for name in ["tet.obj", "sub/tet.obj", "sub/two.obj", "a.b.c.obj", ".obj"] {
        std::fs::write(dir.join(name), TET).unwrap();
    }
}

fn load(dir: &Path, assets: &str, geoms: &str) -> Result<sim_core::Model, MjcfError> {
    load_model(&format!(
        r#"<mujoco><compiler meshdir="{}"/><asset>{assets}</asset>
<worldbody><body><freejoint/>{geoms}</body></worldbody></mujoco>"#,
        dir.display()
    ))
}

/// The name is the file's past the last `/` or `\`, cut at its last `.`; a
/// geom finds the mesh by it (MuJoCo: `tet`, `two`, `a.b.c`, `two`).
#[test]
fn an_unnamed_mesh_is_named_by_its_file() {
    let dir = tempfile::tempdir().unwrap();
    files(dir.path());
    for (file, name) in [
        ("tet.obj", "tet"),
        ("sub/two.obj", "two"),
        ("a.b.c.obj", "a.b.c"),
    ] {
        let model = load(
            dir.path(),
            &format!(r#"<mesh file="{file}"/>"#),
            &format!(r#"<geom type="mesh" mesh="{name}"/>"#),
        )
        .expect(file);
        assert_eq!(model.mesh_name, [name], "{file}");
    }
    // A `\` ends the directory too. MuJoCo also opens the file there, reading
    // `\` as a separator on every platform (`FilePath::PathReduce`); sim-mjcf
    // hands the path to the operating system (ledger L329), which on Windows
    // reads it as MuJoCo does and elsewhere finds no such file, so elsewhere
    // the name is checked in the error.
    let backslash = load(
        dir.path(),
        r#"<mesh file="sub\two.obj"/>"#,
        r#"<geom type="mesh" mesh="two"/>"#,
    );
    if cfg!(windows) {
        assert_eq!(backslash.expect("loads on Windows").mesh_name, ["two"]);
    } else {
        let err = backslash.expect_err("no such file elsewhere");
        assert!(
            matches!(&err, MjcfError::Asset { name, .. } if name == "two"),
            "{err}"
        );
    }
    // Only the last extension goes: `a` names no mesh (MuJoCo: "mesh 'a'
    // not found in geom 0").
    let err = load(
        dir.path(),
        r#"<mesh file="a.b.c.obj"/>"#,
        r#"<geom type="mesh" mesh="a"/>"#,
    )
    .expect_err("a");
    assert!(err.to_string().contains("'a'"), "{err}");
}

/// The name is given before discardvisual drops the meshes no geom names:
/// a geom naming an unnamed mesh by its file keeps it (MuJoCo: nmesh 1).
#[test]
fn an_unnamed_mesh_survives_discardvisual() {
    let dir = tempfile::tempdir().unwrap();
    files(dir.path());
    let model = load_model(&format!(
        r#"<mujoco><compiler meshdir="{}" discardvisual="true"/><asset><mesh file="tet.obj"/></asset>
<worldbody><body><freejoint/><geom type="mesh" mesh="tet"/></body></worldbody></mujoco>"#,
        dir.path().display()
    ))
    .expect("loads");
    assert_eq!(model.mesh_name, ["tet"]);
}

/// A mesh with no name and no file's name is refused, as MuJoCo refuses it
/// ("empty name in mesh"); two of one name are a repeated name ("repeated
/// name 'tet' in mesh").
#[test]
fn a_mesh_with_no_name_or_a_repeated_one_is_refused() {
    let dir = tempfile::tempdir().unwrap();
    files(dir.path());
    for assets in [
        r#"<mesh file=".obj"/>"#,
        r#"<mesh vertex="0 0 0 1 0 0 0 1 0 0 0 1"/>"#,
    ] {
        let err = load(dir.path(), assets, r#"<geom size="0.1"/>"#).expect_err(assets);
        assert!(
            err.to_string().contains("empty name in mesh"),
            "{assets}: {err}"
        );
    }
    // Unused, under discardvisual, as MuJoCo checks names before that pass.
    let err = load_model(&format!(
        r#"<mujoco><compiler meshdir="{}" discardvisual="true"/><asset><mesh file="tet.obj"/><mesh file="sub/tet.obj"/></asset>
<worldbody><body><freejoint/><geom size="0.1"/></body></worldbody></mujoco>"#,
        dir.path().display()
    ))
    .expect_err("unused repeated");
    assert!(
        matches!(&err, MjcfError::DuplicateName { name, .. } if name == "tet"),
        "{err}"
    );
    for assets in [
        r#"<mesh file="tet.obj"/><mesh file="sub/tet.obj"/>"#,
        r#"<mesh name="tet" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 2 1 0 1 3 0 3 2 1 2 3"/><mesh file="tet.obj"/>"#,
    ] {
        let err = load(dir.path(), assets, r#"<geom type="mesh" mesh="tet"/>"#).expect_err(assets);
        assert!(
            matches!(&err, MjcfError::DuplicateName { name, .. } if name == "tet"),
            "{assets}: {err}"
        );
    }
}

/// A height field is named by its file as a mesh is; one written inline with
/// no name is refused ("empty name in height field"), where the parent
/// required a name of both.
#[cfg(feature = "hfields")]
#[test]
fn an_unnamed_height_field_is_named_by_its_file() {
    let dir = tempfile::tempdir().unwrap();
    image::GrayImage::from_raw(2, 2, vec![0, 64, 128, 255])
        .unwrap()
        .save(dir.path().join("terrain.png"))
        .unwrap();
    let model = load(
        dir.path(),
        r#"<hfield file="terrain.png" size="1 1 0.1 0.1"/>"#,
        r#"<geom type="hfield" hfield="terrain"/>"#,
    )
    .expect("loads");
    assert_eq!(model.hfield_name, ["terrain"]);
    let err = load(
        dir.path(),
        r#"<hfield nrow="2" ncol="2" elevation="0 0 0 0" size="1 1 0.1 0.1"/>"#,
        r#"<geom size="0.1"/>"#,
    )
    .expect_err("inline");
    assert!(
        err.to_string().contains("empty name in height field"),
        "{err}"
    );
}
