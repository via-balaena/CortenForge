//! An OBJ file read as MuJoCo 3.5.0 reads it, through tinyobjloader (book
//! Rigid-loading L36; ledger L52 U13, L411 (c)): every vertex and the first
//! shape's faces only (`LoadOBJ`, `user_mesh.cc:1077-1130`), each polygon
//! ear-clipped, a negative index counted back, an index of 0 failing the
//! file, and each number summed as tinyobjloader sums it, a token it cannot
//! read taken as 0.
//!
//! Every face list, mass and reason said to be MuJoCo's was printed by
//! MuJoCo 3.5.0 (the unfused oracle, 2026-11-17) from the same bytes. A mass
//! is computed from the file's floats by `+`, `-`, `*`, `/`, `abs` and `sqrt`
//! alone, whose results IEEE 754 fixes to the bit, so its bits are the same
//! on every platform.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::unreadable_literal
)]

use sim_core::Model;
use sim_mjcf::{MjcfError, load_model};
use std::path::Path;

/// The unit cube centred on the origin.
const CUBE_V: &str = "v -0.5 -0.5 -0.5\nv 0.5 -0.5 -0.5\nv -0.5 0.5 -0.5\nv 0.5 0.5 -0.5\n\
v -0.5 -0.5 0.5\nv 0.5 -0.5 0.5\nv -0.5 0.5 0.5\nv 0.5 0.5 0.5\n";

/// The cube's faces, triangles turned outward.
const CUBE_F: &str = "f 1 3 4\nf 1 4 2\nf 5 6 8\nf 5 8 7\nf 1 2 6\nf 1 6 5\n\
f 3 7 8\nf 3 8 4\nf 1 5 7\nf 1 7 3\nf 2 4 8\nf 2 8 6\n";

/// The cube 3 along x.
const SHIFTED_V: &str = "v 2.5 -0.5 -0.5\nv 3.5 -0.5 -0.5\nv 2.5 0.5 -0.5\nv 3.5 0.5 -0.5\n\
v 2.5 -0.5 0.5\nv 3.5 -0.5 0.5\nv 2.5 0.5 0.5\nv 3.5 0.5 0.5\n";

/// [`CUBE_F`] on [`SHIFTED_V`], written after [`CUBE_V`].
const SHIFTED_F: &str = "f 9 11 12\nf 9 12 10\nf 13 14 16\nf 13 16 15\nf 9 10 14\nf 9 14 13\n\
f 11 15 16\nf 11 16 12\nf 9 13 15\nf 9 15 11\nf 10 12 16\nf 10 16 14\n";

/// [`CUBE_F`]'s faces as MuJoCo holds them.
const CUBE_FACES: [u32; 36] = [
    0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6, 0, 1, 5, 0, 5, 4, 2, 6, 7, 2, 7, 3, 0, 4, 6, 0, 6, 2, 1, 3,
    7, 1, 7, 5,
];

/// A model of one free body holding a mesh geom of the OBJ `text`, written
/// to `dir/file`, with the `<mesh>` attributes `attrs`.
fn load_obj(dir: &Path, file: &str, text: &str, attrs: &str) -> Result<Model, MjcfError> {
    std::fs::write(dir.join(file), text).unwrap();
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

/// Of an OBJ of two shapes, by `o` or by `g`, only the first keeps its
/// faces, and every vertex is kept: two cubes, the second 3 along x, give
/// MuJoCo's 16 vertices, the first cube's 12 faces and its mass (at the
/// parent 24 faces). A `g` with no face before it starts no shape.
#[test]
fn only_an_objs_first_shape_keeps_its_faces() {
    let dir = tempfile::tempdir().unwrap();
    for (name, text) in [
        (
            "two_o.obj",
            format!("{CUBE_V}{SHIFTED_V}o first\n{CUBE_F}o second\n{SHIFTED_F}"),
        ),
        (
            "two_g.obj",
            format!("{CUBE_V}{SHIFTED_V}g first\n{CUBE_F}g second\n{SHIFTED_F}"),
        ),
        (
            "empty_g_first.obj",
            format!("g a\ng b\n{CUBE_V}{SHIFTED_V}{CUBE_F}g c\n{SHIFTED_F}"),
        ),
    ] {
        let model = load_obj(dir.path(), name, &text, "").unwrap();
        assert_eq!(
            mesh_and_mass(&model),
            (16, CUBE_FACES.to_vec(), 1000.0),
            "{name}"
        );
    }
}

/// An L-shaped prism's hexagon caps are ear-clipped, not fanned: MuJoCo's
/// faces and its mass under `legacy` (at the parent the fan's triangles and
/// a mass 160 off).
#[test]
fn an_objs_polygons_are_ear_clipped() {
    let dir = tempfile::tempdir().unwrap();
    let text = "v 0 0 0.0\nv 2 0 0.0\nv 2 1 0.0\nv 1 1 0.0\nv 1 2 0.0\nv 0 2 0.0\n\
v 0 0 1.0\nv 2 0 1.0\nv 2 1 1.0\nv 1 1 1.0\nv 1 2 1.0\nv 0 2 1.0\n\
f 6 5 4 3 2 1\nf 7 8 9 10 11 12\nf 1 2 8 7\nf 2 3 9 8\nf 3 4 10 9\nf 4 5 11 10\nf 5 6 12 11\nf 6 1 7 12\n";
    let faces = vec![
        5, 4, 3, 3, 2, 1, 3, 1, 0, 5, 3, 0, 6, 7, 8, 6, 8, 9, 6, 9, 10, 6, 10, 11, 0, 1, 7, 0, 7,
        6, 1, 2, 8, 1, 8, 7, 2, 3, 9, 2, 9, 8, 3, 4, 10, 3, 10, 9, 4, 5, 11, 4, 11, 10, 5, 0, 6, 5,
        6, 11,
    ];
    let model = load_obj(dir.path(), "lprism.obj", text, r#"inertia="legacy""#).unwrap();
    assert_eq!(mesh_and_mass(&model), (12, faces, 3000.0));
}

/// A prism whose dart-shaped caps are written before its vertices: with an
/// `mtllib` and a bare `usemtl` after the second vertex, tinyobjloader cuts
/// the caps while only two vertices exist, as a fan; without them it cuts
/// them at the end, as ears. MuJoCo's faces and masses for both (at the
/// parent the fan for both).
#[test]
fn a_usemtl_after_an_mtllib_cuts_the_faces_read_so_far() {
    let dir = tempfile::tempdir().unwrap();
    let caps = "f 4 3 2 1\nf 5 6 7 8\n";
    let low = "v 0 0 0.0\nv 2 0 0.0\n";
    let rest = "v 2 2 0.0\nv 1 0.5 0.0\nv 0 0 1.0\nv 2 0 1.0\nv 2 2 1.0\nv 1 0.5 1.0\n";
    let sides = "f 1 2 6 5\nf 2 3 7 6\nf 3 4 8 7\nf 4 1 5 8\n";
    let side_faces = [
        0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7,
    ];
    for (text, top, mass) in [
        (
            format!("{caps}{low}mtllib a.mtl\nusemtl\n{rest}{sides}"),
            [4, 5, 6, 4, 6, 7],
            1654.8565289029486,
        ),
        (
            format!("{caps}{low}{rest}{sides}"),
            [5, 6, 7, 4, 5, 7],
            1500.0000000000002,
        ),
    ] {
        let model = load_obj(dir.path(), "dart.obj", &text, "").unwrap();
        let mut faces = vec![3, 2, 1, 3, 1, 0];
        faces.extend(top);
        faces.extend(side_faces);
        assert_eq!(mesh_and_mass(&model), (8, faces, mass));
    }
}

/// A negative index counts back from the vertices read so far: the cube
/// written with `-8`…`-1`, and each triangle written after its own three
/// vertices as `-3 -2 -1`, are MuJoCo's (at the parent both refused).
#[test]
fn an_objs_negative_indices_count_back() {
    let dir = tempfile::tempdir().unwrap();
    let backward = "f -8 -6 -5\nf -8 -5 -7\nf -4 -3 -1\nf -4 -1 -2\nf -8 -7 -3\nf -8 -3 -4\n\
f -6 -2 -1\nf -6 -1 -5\nf -8 -4 -2\nf -8 -2 -6\nf -7 -5 -1\nf -7 -1 -3\n";
    let model = load_obj(dir.path(), "back.obj", &format!("{CUBE_V}{backward}"), "").unwrap();
    assert_eq!(mesh_and_mass(&model), (8, CUBE_FACES.to_vec(), 1000.0));

    let corners: Vec<&str> = CUBE_V.lines().collect();
    let interleaved: String = CUBE_F
        .lines()
        .map(|line| {
            let mut tri = String::new();
            for id in line[2..].split(' ') {
                tri += corners[id.parse::<usize>().unwrap() - 1];
                tri += "\n";
            }
            tri + "f -3 -2 -1\n"
        })
        .collect();
    let model = load_obj(dir.path(), "interleaved.obj", &interleaved, "").unwrap();
    assert_eq!(mesh_and_mass(&model), (36, (0..36).collect(), 1000.0));
}

/// An index of 0 in a face, its texture coordinate's slot, an empty normal
/// slot, or a comment after a face's ids (read as an index) fails the file,
/// in MuJoCo's words (at the parent other words, or a load).
#[test]
fn an_index_of_0_fails_an_obj_file() {
    let dir = tempfile::tempdir().unwrap();
    for (file, extra, face) in [
        ("zero.obj", "", "f 0 3 4"),
        ("zero_vt.obj", "vt 0 0\n", "f 1/0 3/1 4/1"),
        ("empty_vn.obj", "vn 0 0 1\n", "f 1// 3// 4//"),
        ("comment.obj", "", "f 1 3 4 # c"),
    ] {
        let text = format!("{CUBE_V}{extra}{}", CUBE_F.replacen("f 1 3 4", face, 1));
        let err = load_obj(dir.path(), file, &text, "").unwrap_err();
        assert_eq!(
            err.to_string(),
            format!("mesh 'm': could not parse OBJ file '{file}'"),
            "{file}"
        );
    }
}

/// A tetrahedron's four vertex lines.
fn tet(vertices: &str) -> String {
    format!("{vertices}f 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n")
}

/// A coordinate whose token tinyobjloader cannot read is 0: `nan`, a missing
/// one, and an exponent with no digits; a token is read up to the first
/// character it cannot take. MuJoCo's masses (at the parent each refused).
#[test]
fn an_obj_token_tinyobjloader_cannot_read_is_0() {
    let dir = tempfile::tempdir().unwrap();
    for (vertices, attrs, mass) in [
        (
            "v nan 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\n",
            "",
            166.66666666666666,
        ),
        (
            "v 0 0\nv 1 0\nv 0 1\nv 0 0 1\n",
            r#"inertia="shell""#,
            2366.0254037844384,
        ),
        (
            "v 1.5e 0x10 1,5\nv 1e5.5 0 0\nv 0 1.5abc 0\nv 0 0 .5e1\n",
            r#"inertia="shell""#,
            551149444.6394224,
        ),
    ] {
        let model = load_obj(dir.path(), "tet.obj", &tet(vertices), attrs).unwrap();
        let (n, _, m) = mesh_and_mass(&model);
        assert_eq!((n, m), (4, mass), "{vertices}");
    }
}

/// A number is summed as tinyobjloader sums it, not rounded to the nearest
/// float: `8.411888599395751953e0` is read as 0x41069719, one above the
/// float nearest it. MuJoCo's mass for a tetrahedron of such numbers (at the
/// parent 3.5e-3 off).
#[test]
fn an_obj_number_is_summed_as_tinyobjloader_sums_it() {
    let dir = tempfile::tempdir().unwrap();
    let vertices = "v 8.411888599395751953e0 8.8978716731071472167e-1 6.0829824447631835937e1\n\
v 1.72690510780171280e-8 0.5 0.5\nv 0.25 2.11427281132209569792e+1 0.125\n\
v 0 0 3.0000001788139343261718750001\n";
    let model = load_obj(dir.path(), "tet.obj", &tet(vertices), r#"inertia="shell""#).unwrap();
    assert_eq!(mesh_and_mass(&model).2, 1292783.3693147341);
}
