//! An asset file's path read as MuJoCo 3.5.0 reads it (book Rigid-loading
//! L36; ledger L329): `/` and `\` both separate, on every platform, and `.`
//! and `..` are taken away by the path's text (`FilePath::PathReduce`,
//! `user_util.cc:1006-1035`), not by the file system, so a `..` past a
//! directory that does not exist still finds the file; under `strippath`
//! the file is cut at its last `/` or `\` (`mjuu_strippath`,
//! `user_util.cc:768-781`). A height field's path is read the same way.
//!
//! MuJoCo 3.5.0 loads each such document here (the start probes `a01`,
//! `a03`, `a05`, `a07`, `a08`, `a10` and `a10c`, and `C:\c.obj` under
//! `strippath`, 2026-11-16, in this layout) as it loads the same file named
//! plainly.

#![allow(clippy::unwrap_used, clippy::panic)]

use sim_core::Model;
use sim_mjcf::{MjcfError, load_model_from_file};
use std::path::Path;

/// The start probes' layout: the unit cube as `sub/c.obj`, a tetrahedron as
/// `c.obj` beside the model, and an empty directory `sub/deep`.
fn layout(dir: &Path) {
    std::fs::create_dir_all(dir.join("sub/deep")).unwrap();
    std::fs::write(
        dir.join("sub/c.obj"),
        "v -0.5 -0.5 -0.5\nv 0.5 -0.5 -0.5\nv -0.5 0.5 -0.5\nv 0.5 0.5 -0.5\n\
         v -0.5 -0.5 0.5\nv 0.5 -0.5 0.5\nv -0.5 0.5 0.5\nv 0.5 0.5 0.5\n\
         f 1 3 4\nf 1 4 2\nf 5 6 8\nf 5 8 7\nf 1 2 6\nf 1 6 5\n\
         f 3 7 8\nf 3 8 4\nf 1 5 7\nf 1 7 3\nf 2 4 8\nf 2 8 6\n",
    )
    .unwrap();
    std::fs::write(
        dir.join("c.obj"),
        "v 0.1 0.2 0.3\nv 1.1 0.2 0.3\nv 0.1 2.2 0.3\nv 0.1 0.2 3.3\n\
         f 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n",
    )
    .unwrap();
}

/// Write `xml` as `dir/model.xml` and load it from there, so the model's
/// directory is `dir`.
fn load_from(dir: &Path, xml: &str) -> Result<Model, MjcfError> {
    let path = dir.join("model.xml");
    std::fs::write(&path, xml).unwrap();
    load_model_from_file(&path)
}

/// One free body holding a mesh geom of `<mesh file="{file}"/>`, under
/// `<compiler {compiler}/>`: its mesh's vertices and its body's mass.
fn mesh_model(dir: &Path, compiler: &str, file: &str) -> Result<(Vec<[f64; 3]>, f64), MjcfError> {
    let model = load_from(
        dir,
        &format!(
            r#"<mujoco><compiler {compiler}/><asset><mesh name="m" file="{file}"/></asset>
<worldbody><body name="b"><freejoint/><geom type="mesh" mesh="m"/></body></worldbody></mujoco>"#
        ),
    )?;
    let vertices = model.mesh_data[0]
        .vertices()
        .iter()
        .map(|v| [v.x, v.y, v.z])
        .collect();
    Ok((vertices, model.body_mass[1]))
}

/// A mesh's path with `\`, with `..` past a directory that does not exist,
/// or under `strippath` loads its file, as MuJoCo loads it; at the parent
/// each was refused, its file not found (`\` read as a separator only on
/// Windows, `..` by the file system). `strippath` cuts at a `\` that reducing
/// the path leaves, a drive's (MuJoCo loads `C:\c.obj` as `c.obj`).
#[test]
fn a_mesh_path_is_read_as_mujoco_reads_it() {
    let dir = tempfile::tempdir().unwrap();
    layout(dir.path());
    let cube = mesh_model(dir.path(), "", "sub/c.obj").unwrap();
    let tet = mesh_model(dir.path(), "", "c.obj").unwrap();
    assert_ne!(cube, tet);
    for (compiler, file, expected) in [
        ("", r"sub\c.obj", &cube),
        ("", "sub/nowhere/../c.obj", &cube),
        (r#"meshdir="sub\""#, "c.obj", &cube),
        (r#"strippath="true""#, r"nowhere\c.obj", &tet),
        (r#"strippath="true""#, r"C:\c.obj", &tet),
        ("", r"sub\deep\..\c.obj", &cube),
    ] {
        let got = mesh_model(dir.path(), compiler, file)
            .unwrap_or_else(|e| panic!("{compiler} {file}: {e}"));
        assert_eq!(&got, expected, "{compiler} {file}");
    }
    // A name the reduced path does not find is refused still (MuJoCo:
    // "Error opening file 'sub/c.obj '").
    let err = mesh_model(dir.path(), "", "sub/c.obj ").unwrap_err();
    assert!(
        err.to_string()
            .contains("asset file not found: 'sub/c.obj '"),
        "{err}"
    );
}

/// A height field's path is read as a mesh's is (MuJoCo loads `sub\h.png`
/// and `sub/nowhere/../h.png` as `sub/h.png`).
#[cfg(feature = "hfields")]
#[test]
fn a_height_field_path_is_read_as_mujoco_reads_it() {
    let dir = tempfile::tempdir().unwrap();
    layout(dir.path());
    image::GrayImage::from_raw(4, 3, vec![0, 64, 128, 255, 10, 20, 30, 40, 255, 0, 255, 0])
        .unwrap()
        .save(dir.path().join("sub/h.png"))
        .unwrap();
    let height_field = |file: &str| {
        let model = load_from(
            dir.path(),
            &format!(
                r#"<mujoco><asset><hfield name="h" file="{file}" size="1 1 0.5 0.1"/></asset>
<worldbody><geom type="hfield" hfield="h"/></worldbody></mujoco>"#
            ),
        )
        .unwrap_or_else(|e| panic!("{file}: {e}"));
        ((*model.hfield_data[0]).clone(), model.hfield_size[0])
    };
    let plain = height_field("sub/h.png");
    for file in [r"sub\h.png", "sub/nowhere/../h.png"] {
        assert_eq!(height_field(file), plain, "{file}");
    }
}
