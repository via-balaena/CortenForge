//! A mesh file other than an STL read as MuJoCo 3.5.0 reads it (book
//! Rigid-loading L36; ledger L411 (b)): an MSH file as `LoadMSH` reads it
//! (`user_mesh.cc:1282-1368`); under a `scale` whose product is not positive
//! each face's last two vertices change places (`LoadOBJ`, `:1085-1104`;
//! `LoadMSH`, `:1359-1366`), its repeated vertices stay apart, and a
//! coordinate that is not finite is refused naming its vertex
//! (`ProcessVertices`, `:557-561`); a file's format is its `content_type`'s,
//! else its extension's (`LoadFromResource`, `:646-666`). A PLY file, and
//! with the `threemf` feature a 3MF file, which MuJoCo does not read, is read
//! and turned the same way (registry `D-MESH-FORMAT`).
//!
//! Every expected face list, mass and reason said to be MuJoCo's was printed
//! by MuJoCo 3.5.0 (the census's unfused oracle, 2026-11-16) from the same
//! bytes, one process each; a PLY or 3MF file, which MuJoCo does not read, is
//! held to the model of the same tetrahedron's OBJ. A mass is sums of
//! products of the file's floats and square roots, which IEEE 754 rounds
//! exactly, so its bits are the same on every platform.

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

/// An MSH file: its four counts, then its vertices' floats, `extra` floats
/// (normals and texture coordinates), and its faces' vertex ids.
fn msh(counts: [i32; 4], vertices: &[f32], extra: &[f32], faces: &[i32]) -> Vec<u8> {
    let mut bytes: Vec<u8> = counts.iter().flat_map(|n| n.to_le_bytes()).collect();
    for x in vertices.iter().chain(extra) {
        bytes.extend_from_slice(&x.to_le_bytes());
    }
    for i in faces {
        bytes.extend_from_slice(&i.to_le_bytes());
    }
    bytes
}

/// The tetrahedron's corners and faces, as an MSH file holds them.
const TET_FLOATS: [f32; 12] = [0.1, 0.2, 0.3, 1.1, 0.2, 0.3, 0.1, 2.2, 0.3, 0.1, 0.2, 3.3];
const TET_FACES: [i32; 12] = [0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3];

/// An MSH file is read as MuJoCo's `LoadMSH` reads it, its normals and
/// texture coordinates passed over, its repeated vertices kept apart, its
/// faces turned under a left-handed `scale` (at the parent it was refused as
/// of no format read here).
#[test]
fn an_msh_file_loads_as_mujoco_reads_it() {
    let dir = tempfile::tempdir().unwrap();
    let plain = msh([4, 0, 0, 4], &TET_FLOATS, &[], &TET_FACES);
    let with_extra = msh([4, 4, 4, 4], &TET_FLOATS, &[0.0; 20], &TET_FACES);
    // Normals or texture coordinates alone: their floats, 3 or 2 a vertex,
    // tell where the faces begin.
    let normals = msh([4, 4, 0, 4], &TET_FLOATS, &[0.0; 12], &TET_FACES);
    let texcoords = msh([4, 0, 4, 4], &TET_FLOATS, &[0.0; 8], &TET_FACES);
    // The fourth corner written twice, the last face naming the second.
    let twice_floats = [&TET_FLOATS[..], &TET_FLOATS[9..]].concat();
    let mut twice_faces = TET_FACES;
    twice_faces[11] = 4;
    let twice = msh([5, 0, 0, 4], &twice_floats, &[], &twice_faces);
    let tet = vec![0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3];
    let tet_twice = vec![0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 4];
    for (bytes, attrs, expected) in [
        (
            &plain,
            r#"inertia="exact""#,
            (4, tet.clone(), 1000.000024835268),
        ),
        (
            &with_extra,
            r#"inertia="exact""#,
            (4, tet.clone(), 1000.000024835268),
        ),
        (
            &normals,
            r#"inertia="exact""#,
            (4, tet.clone(), 1000.000024835268),
        ),
        (
            &texcoords,
            r#"inertia="exact""#,
            (4, tet, 1000.000024835268),
        ),
        (
            &plain,
            r#"scale="1 -2 1" inertia="exact""#,
            (4, TET_TURNED.to_vec(), 2000.0000496705366),
        ),
        (
            &twice,
            r#"inertia="exact""#,
            (5, tet_twice, 1000.000024835268),
        ),
    ] {
        let model = load_bytes(dir.path(), "tet.msh", bytes, attrs).unwrap();
        assert_eq!(mesh_and_mass(&model), expected, "{attrs}");
    }
}

/// An MSH file is refused in MuJoCo's order and words: its sizes, then its
/// vertices, then its faces.
#[test]
fn an_msh_file_is_refused_as_mujoco_refuses_it() {
    let dir = tempfile::tempdir().unwrap();
    let mut nan = TET_FLOATS;
    nan[4] = f32::NAN;
    let mut negative = TET_FACES;
    negative[4] = -1;
    let mut past = TET_FACES;
    past[4] = 4;
    let mut longer = msh([4, 0, 0, 4], &TET_FLOATS, &[], &TET_FACES);
    longer.extend_from_slice(&[0; 4]);
    for (bytes, words) in [
        (Vec::new(), "MSH file 't.msh' is empty"),
        (vec![0; 12], "missing header in MSH file 't.msh'"),
        (
            msh([3, 0, 0, 1], &TET_FLOATS[..9], &[], &[0, 1, 2]),
            "invalid sizes in MSH file 't.msh'",
        ),
        (
            msh([4, 3, 0, 4], &TET_FLOATS, &[0.0; 9], &TET_FACES),
            "invalid sizes in MSH file 't.msh'",
        ),
        (
            msh([4, 0, 3, 4], &TET_FLOATS, &[0.0; 6], &TET_FACES),
            "invalid sizes in MSH file 't.msh'",
        ),
        (
            msh([4, 0, 0, -1], &TET_FLOATS, &[], &[]),
            "invalid sizes in MSH file 't.msh'",
        ),
        (
            msh([178_956_970, 0, 0, 4], &[], &[], &[]),
            "too large sizes in MSH file 't.msh'.",
        ),
        (longer, "unexpected file size in MSH file 't.msh'"),
        (
            msh([4, 0, 0, 4], &nan, &[], &negative),
            "vertex coordinate 1 is not finite",
        ),
        (
            msh([4, 0, 0, 4], &TET_FLOATS, &[], &negative),
            "in face 1, vertex index -1 does not exist",
        ),
        (
            msh([4, 0, 0, 4], &TET_FLOATS, &[], &past),
            "in face 1, vertex index 4 does not exist",
        ),
    ] {
        let err = load_bytes(dir.path(), "t.msh", &bytes, "").unwrap_err();
        assert_eq!(err.to_string(), format!("mesh 'm': {words}"));
    }
}

/// The unit cube centred on the origin as a binary STL.
fn cube_stl() -> Vec<u8> {
    let corner = |i: u8| {
        [
            f32::from(i & 1) - 0.5,
            f32::from((i >> 1) & 1) - 0.5,
            f32::from((i >> 2) & 1) - 0.5,
        ]
    };
    let faces: [[u8; 3]; 12] = [
        [0, 2, 3],
        [0, 3, 1],
        [4, 5, 7],
        [4, 7, 6],
        [0, 1, 5],
        [0, 5, 4],
        [2, 6, 7],
        [2, 7, 3],
        [0, 4, 6],
        [0, 6, 2],
        [1, 3, 7],
        [1, 7, 5],
    ];
    let mut bytes = vec![b' '; 80];
    bytes.extend_from_slice(&12u32.to_le_bytes());
    for face in faces {
        bytes.extend_from_slice(&[0; 12]);
        for c in face.into_iter().flat_map(corner) {
            bytes.extend_from_slice(&c.to_le_bytes());
        }
        bytes.extend_from_slice(&[0; 2]);
    }
    bytes
}

/// A mesh's `content_type`, where written, names its format as MuJoCo's
/// does, past any `;parameter=value`; an empty one is the extension's.
#[test]
fn a_content_type_names_the_files_format() {
    let dir = tempfile::tempdir().unwrap();
    let tet = msh([4, 0, 0, 4], &TET_FLOATS, &[], &TET_FACES);
    for (file, bytes, attrs, (nvert, nface, mass)) in [
        (
            "cube",
            cube_stl(),
            r#"content_type="model/stl""#,
            (8, 12, 1000.0),
        ),
        (
            "tet.bin",
            tet,
            r#"content_type="model/vnd.mujoco.msh""#,
            (4, 4, 1000.000024835268),
        ),
        (
            "cube.dat",
            cube_stl(),
            r#"content_type="model/stl;version=1""#,
            (8, 12, 1000.0),
        ),
        (
            "cube.stl",
            cube_stl(),
            r#"content_type="""#,
            (8, 12, 1000.0),
        ),
    ] {
        let model = load_bytes(dir.path(), file, &bytes, attrs).unwrap();
        let (n, faces, m) = mesh_and_mass(&model);
        assert_eq!(
            (n, faces.len() / 3, m),
            (nvert, nface, mass),
            "{file} {attrs}"
        );
    }
}

/// A `content_type` not of MuJoCo's form, a type of no mesh format, and an
/// extension of no type are refused in MuJoCo's words.
#[test]
fn a_mesh_files_format_is_refused_in_mujocos_words() {
    let dir = tempfile::tempdir().unwrap();
    for (file, attrs, words) in [
        (
            "cube.stl",
            r#"content_type="stl""#,
            "invalid content type: 'stl'",
        ),
        (
            "cube.stl",
            r#"content_type="model/""#,
            "invalid content type: 'model/'",
        ),
        (
            "cube.stl",
            r#"content_type="model/stl;x""#,
            "invalid content type: 'model/stl;x'",
        ),
        (
            "cube.stl",
            r#"content_type="model;a=b/stl""#,
            "invalid content type: 'model;a=b/stl'",
        ),
        (
            "cube.stl",
            r#"content_type="model/stl=1;x""#,
            "invalid content type: 'model/stl=1;x'",
        ),
        (
            "cube.stl",
            r#"content_type="model/;a=b""#,
            "unsupported mesh type: 'model/'",
        ),
        (
            "cube.stl",
            r#"content_type="model/gltf""#,
            "unsupported mesh type: 'model/gltf'",
        ),
        (
            "cube.stl",
            r#"content_type="Model/STL""#,
            "unsupported mesh type: 'Model/STL'",
        ),
        ("cube.png", "", "unsupported mesh type: 'image/png'"),
        (
            "cube.xyz",
            "",
            "unknown or unsupported mesh file: 'cube.xyz'",
        ),
    ] {
        let err = load_bytes(dir.path(), file, &cube_stl(), attrs).unwrap_err();
        assert_eq!(
            err.to_string(),
            format!("mesh 'm': {words}"),
            "{file} {attrs}"
        );
    }
}

/// A PLY file, which MuJoCo names and refuses ("unsupported mesh type:
/// 'model/ply'"), is read here, by its extension or its type (registry
/// `D-MESH-FORMAT`).
#[test]
fn a_ply_file_loads_by_its_extension_or_its_type() {
    let dir = tempfile::tempdir().unwrap();
    let obj = load_bytes(dir.path(), "tet.obj", TET_OBJ.as_bytes(), "").unwrap();
    for (file, attrs) in [("tet.ply", ""), ("tet.txt", r#"content_type="model/ply""#)] {
        let model = load_bytes(dir.path(), file, TET_PLY.as_bytes(), attrs).unwrap();
        assert_eq!(mesh_and_mass(&model), mesh_and_mass(&obj), "{file}");
    }
}

/// A 3MF file, which MuJoCo does not read (`.3mf` is of no type to it), is
/// read with the `threemf` feature, by its extension or its type, as the
/// tetrahedron's OBJ is (registry `D-MESH-FORMAT`).
#[cfg(feature = "threemf")]
#[test]
fn a_3mf_file_loads_by_its_extension_or_its_type() {
    let dir = tempfile::tempdir().unwrap();
    let mut tet = mesh_types::IndexedMesh::new();
    for [x, y, z] in [
        [0.1, 0.2, 0.3],
        [1.1, 0.2, 0.3],
        [0.1, 2.2, 0.3],
        [0.1, 0.2, 3.3],
    ] {
        tet.vertices.push(mesh_types::Point3::new(x, y, z));
    }
    tet.faces
        .extend([[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]]);
    let path = dir.path().join("tet.3mf");
    mesh_io::save_3mf(&tet, &path).unwrap();
    let bytes = std::fs::read(&path).unwrap();
    let obj = load_bytes(dir.path(), "tet.obj", TET_OBJ.as_bytes(), "").unwrap();
    for (file, attrs) in [("tet.3mf", ""), ("tet.zip", r#"content_type="model/3mf""#)] {
        let model = load_bytes(dir.path(), file, &bytes, attrs).unwrap();
        assert_eq!(mesh_and_mass(&model), mesh_and_mass(&obj), "{file}");
    }
    // Turned under a left-handed `scale` as the OBJ is (MuJoCo's faces and
    // mass for the OBJ: `a_left_handed_scale_turns_an_objs_faces`).
    let attrs = r#"scale="1 -2 1" inertia="exact""#;
    let mirrored = load_bytes(dir.path(), "tet.3mf", &bytes, attrs).unwrap();
    assert_eq!(
        mesh_and_mass(&mirrored),
        (4, TET_TURNED.to_vec(), 2000.0000496705366)
    );
}

/// Without the `threemf` feature a `.3mf` file is refused as MuJoCo refuses
/// one, as of no type.
#[cfg(not(feature = "threemf"))]
#[test]
fn a_3mf_file_is_refused_without_the_feature() {
    let dir = tempfile::tempdir().unwrap();
    let err = load_bytes(dir.path(), "tet.3mf", b"PK", "").unwrap_err();
    assert_eq!(
        err.to_string(),
        "mesh 'm': unknown or unsupported mesh file: 'tet.3mf'"
    );
}
