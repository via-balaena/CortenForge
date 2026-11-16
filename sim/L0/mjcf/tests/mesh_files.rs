//! A mesh file other than an STL read as MuJoCo 3.5.0 reads it (book
//! Rigid-loading L36; ledger L411 (b)): under a `scale` whose product is not
//! positive each face's last two vertices change places (`LoadOBJ`,
//! `user_mesh.cc:1085-1104`), its repeated vertices stay apart, and a
//! coordinate that is not finite is refused naming its vertex
//! (`ProcessVertices`, `:557-561`). A PLY file, which MuJoCo does not read,
//! is turned the same way.
//!
//! Every expected face list, mass and reason was printed by MuJoCo 3.5.0
//! (the census's unfused oracle, 2026-11-16) from the same files, one
//! process each. A mass is a sum of products of the file's floats, with no
//! library function, so its bits are the same on every platform.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::unreadable_literal
)]

use sim_core::Model;
use sim_mjcf::{MjcfError, load_model};
use std::path::Path;

/// A tetrahedron off the origin, its faces turned outward.
const TET_OBJ: &str = "v 0.1 0.2 0.3\nv 1.1 0.2 0.3\nv 0.1 2.2 0.3\nv 0.1 0.2 3.3\n\
f 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n";

/// The unit cube centred on the origin, its faces quads turned outward.
const CUBE_QUADS_OBJ: &str = "v -0.5 -0.5 -0.5\nv 0.5 -0.5 -0.5\nv -0.5 0.5 -0.5\nv 0.5 0.5 -0.5\n\
v -0.5 -0.5 0.5\nv 0.5 -0.5 0.5\nv -0.5 0.5 0.5\nv 0.5 0.5 0.5\n\
f 1 3 4 2\nf 5 6 8 7\nf 1 2 6 5\nf 3 7 8 4\nf 1 5 7 3\nf 2 4 8 6\n";

/// The tetrahedron as an ASCII PLY file.
const TET_PLY: &str = "ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\nproperty float y\n\
property float z\nelement face 4\nproperty list uchar int vertex_indices\nend_header\n\
0.1 0.2 0.3\n1.1 0.2 0.3\n0.1 2.2 0.3\n0.1 0.2 3.3\n3 0 2 1\n3 0 1 3\n3 0 3 2\n3 1 2 3\n";

/// A model of one free body holding a mesh geom of `bytes`, written to
/// `dir/file`, with the `<mesh>` attributes `attrs`.
fn load_bytes(dir: &Path, file: &str, bytes: &[u8], attrs: &str) -> Result<Model, MjcfError> {
    std::fs::write(dir.join(file), bytes).unwrap();
    load_model(&format!(
        r#"<mujoco><compiler meshdir="{}"/><asset><mesh name="m" file="{file}" {attrs}/></asset>
<worldbody><body name="b"><freejoint/><geom type="mesh" mesh="m"/></body></worldbody></mujoco>"#,
        dir.display()
    ))
}

/// The mesh's vertex count, its faces as flat vertex ids, and the body's
/// mass.
fn mesh_and_mass(model: &Model) -> (usize, Vec<u32>, f64) {
    let mesh = &model.mesh_data[0];
    let faces = mesh.triangles().iter().flatten().copied().collect();
    (mesh.vertex_count(), faces, model.body_mass[1])
}

/// The tetrahedron's faces turned, as MuJoCo turns them.
const TET_TURNED: [u32; 12] = [0, 1, 2, 0, 3, 1, 0, 2, 3, 1, 3, 2];

/// Under a left-handed `scale` an OBJ's faces are turned, each triangle of
/// a quad too, so the mirrored mesh keeps its volume: MuJoCo's faces and
/// masses in each mode (at the parent `exact` refused both as negative in
/// volume).
#[test]
fn a_left_handed_scale_turns_an_objs_faces() {
    let dir = tempfile::tempdir().unwrap();
    for (mode, mass) in [
        ("legacy", 2000.0000496705366),
        ("exact", 2000.0000496705366),
        ("shell", 16000.000150157852),
    ] {
        let attrs = format!(r#"scale="1 -2 1" inertia="{mode}""#);
        let model = load_bytes(dir.path(), "tet.obj", TET_OBJ.as_bytes(), &attrs).unwrap();
        assert_eq!(
            mesh_and_mass(&model),
            (4, TET_TURNED.to_vec(), mass),
            "{mode}"
        );
    }
    let cube_turned = vec![
        0, 3, 2, 0, 1, 3, 4, 7, 5, 4, 6, 7, 0, 5, 1, 0, 4, 5, 2, 7, 6, 2, 3, 7, 0, 6, 4, 0, 2, 6,
        1, 7, 3, 1, 5, 7,
    ];
    for (mode, mass) in [("legacy", 1000.0), ("exact", 1000.0), ("shell", 6000.0)] {
        let attrs = format!(r#"scale="-1 1 1" inertia="{mode}""#);
        let model = load_bytes(dir.path(), "cube.obj", CUBE_QUADS_OBJ.as_bytes(), &attrs).unwrap();
        assert_eq!(
            mesh_and_mass(&model),
            (8, cube_turned.clone(), mass),
            "{mode}"
        );
    }
}

/// An OBJ's repeated vertices stay apart, as MuJoCo's `ProcessVertices`
/// keeps them (it merges only an STL's): a cube whose last corner is written
/// twice, one face naming the second (MuJoCo: 9 vertices, that face's id 8).
#[test]
fn an_objs_repeated_vertices_stay_apart() {
    let dir = tempfile::tempdir().unwrap();
    let text = CUBE_QUADS_OBJ
        .replacen("v 0.5 0.5 0.5\n", "v 0.5 0.5 0.5\nv 0.5 0.5 0.5\n", 1)
        .replacen("f 3 7 8 4", "f 3 7 9 4", 1);
    let faces = vec![
        0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6, 0, 1, 5, 0, 5, 4, 2, 6, 8, 2, 8, 3, 0, 4, 6, 0, 6, 2,
        1, 3, 7, 1, 7, 5,
    ];
    for (mode, mass) in [("legacy", 1000.0), ("exact", 1000.0), ("shell", 6000.0)] {
        let attrs = format!(r#"inertia="{mode}""#);
        let model = load_bytes(dir.path(), "twice.obj", text.as_bytes(), &attrs).unwrap();
        assert_eq!(mesh_and_mass(&model), (9, faces.clone(), mass), "{mode}");
    }
}

/// A PLY file, read where MuJoCo reads none, is turned as an OBJ is: the
/// mirrored tetrahedron as a PLY is the OBJ's.
#[test]
fn a_left_handed_scale_turns_a_plys_faces() {
    let dir = tempfile::tempdir().unwrap();
    let attrs = r#"scale="1 -2 1" inertia="exact""#;
    let ply = load_bytes(dir.path(), "tet.ply", TET_PLY.as_bytes(), attrs).unwrap();
    let obj = load_bytes(dir.path(), "tet.obj", TET_OBJ.as_bytes(), attrs).unwrap();
    assert_eq!(mesh_and_mass(&ply), mesh_and_mass(&obj));
    assert_eq!(
        mesh_and_mass(&ply),
        (4, TET_TURNED.to_vec(), 2000.0000496705366)
    );
}

/// A coordinate too large for single precision becomes infinite and is
/// refused naming its vertex, not its float as an STL's is (MuJoCo: the
/// first vertex's x "vertex coordinate 0 is not finite", the second's y
/// "vertex coordinate 1").
#[test]
fn an_obj_coordinate_beyond_single_precision_is_refused() {
    let dir = tempfile::tempdir().unwrap();
    for (vertex, huge, words) in [
        (
            "v 0.1 0.2 0.3",
            "v 1e40 0 0",
            "vertex coordinate 0 is not finite",
        ),
        (
            "v 1.1 0.2 0.3",
            "v 0 -1e40 0",
            "vertex coordinate 1 is not finite",
        ),
    ] {
        let text = TET_OBJ.replacen(vertex, huge, 1);
        let err = load_bytes(dir.path(), "huge.obj", text.as_bytes(), "").unwrap_err();
        assert_eq!(err.to_string(), format!("mesh 'm': {words}"));
    }
}
