//! An STL mesh file read as MuJoCo 3.5.0 reads it (book Rigid-loading L36;
//! ledger L43 (a)(b), L411 (a)): `LoadSTL` (`user_mesh.cc:1209-1277`) reads
//! a binary file by its size, refuses its faults in its order and words,
//! refuses a coordinate above 2^30 in magnitude and swaps each face's last
//! two vertices under a left-handed `scale`; `ProcessVertices` (`:543-614`)
//! refuses a coordinate that is not finite and makes vertices equal as
//! floats one, numbered as they first appear. This crate also reads an
//! ASCII STL and one of more than 200,000 faces, which MuJoCo refuses
//! (Q118).
//!
//! Every expected count, face list, mass and reason said to be MuJoCo's was
//! printed by MuJoCo 3.5.0 (the census's unfused oracle, 2026-11-16) from the
//! same bytes; a file MuJoCo does not read (ASCII, more than 200,000 faces) is
//! held to the model of its MuJoCo-measured twin, or refused in this crate's
//! words where a test says so. A mass is computed from the file's floats by
//! `+`, `-`, `*`, `/`, `abs` and `sqrt` alone, whose results IEEE 754 fixes
//! to the bit, so its bits are the same on every platform.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp,
    clippy::unreadable_literal
)]

use sim_core::Model;
use sim_mjcf::{MjcfError, load_model};
use std::fmt::Write as _;
use std::path::Path;

/// The unit cube centred on the origin: its corner `i` at
/// `(i & 1, i >> 1 & 1, i >> 2 & 1) - 0.5`.
fn cube_corners() -> Vec<[f32; 3]> {
    (0u8..8)
        .map(|i| {
            [
                f32::from(i & 1) - 0.5,
                f32::from((i >> 1) & 1) - 0.5,
                f32::from((i >> 2) & 1) - 0.5,
            ]
        })
        .collect()
}

/// The cube's twelve faces, each turned outward.
const CUBE: [[usize; 3]; 12] = [
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

/// A tetrahedron off the origin, its faces turned outward.
const TET_CORNERS: [[f32; 3]; 4] = [
    [0.1, 0.2, 0.3],
    [1.1, 0.2, 0.3],
    [0.1, 2.2, 0.3],
    [0.1, 0.2, 3.3],
];
const TET: [[usize; 3]; 4] = [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];

/// Each face's three corners, as an STL lists them.
fn soup(corners: &[[f32; 3]], faces: &[[usize; 3]]) -> Vec<[[f32; 3]; 3]> {
    faces.iter().map(|f| f.map(|i| corners[i])).collect()
}

/// A binary STL of `faces`: an 80-byte header, the count, and per face a
/// zero normal, its three vertices and two zero bytes.
fn binary_stl(header: &[u8], faces: &[[[f32; 3]; 3]]) -> Vec<u8> {
    let mut bytes = vec![b' '; 80];
    bytes[..header.len()].copy_from_slice(header);
    bytes.extend_from_slice(&u32::try_from(faces.len()).unwrap().to_le_bytes());
    for face in faces {
        bytes.extend_from_slice(&[0; 12]);
        for c in face.iter().flatten() {
            bytes.extend_from_slice(&c.to_le_bytes());
        }
        bytes.extend_from_slice(&[0; 2]);
    }
    bytes
}

/// An ASCII STL of `faces`, each number written as its shortest decimal.
fn ascii_stl(faces: &[[[f32; 3]; 3]]) -> String {
    let mut text = String::from("solid s\n");
    for face in faces {
        text.push_str("  facet normal 0 0 0\n    outer loop\n");
        for v in face {
            writeln!(text, "      vertex {:?} {:?} {:?}", v[0], v[1], v[2]).unwrap();
        }
        text.push_str("    endloop\n  endfacet\n");
    }
    text + "endsolid s\n"
}

/// A model of one free body holding a mesh geom of `file` (in `dir`) with
/// the `<mesh>` attributes `attrs`.
fn load(dir: &Path, file: &str, attrs: &str) -> Result<Model, MjcfError> {
    load_model(&format!(
        r#"<mujoco><compiler meshdir="{}"/><asset><mesh name="m" file="{file}" {attrs}/></asset>
<worldbody><body name="b"><freejoint/><geom type="mesh" mesh="m"/></body></worldbody></mujoco>"#,
        dir.display()
    ))
}

/// `bytes` written to `dir/file`, then loaded as [`load`] loads it.
fn write_and_load(dir: &Path, file: &str, bytes: &[u8], attrs: &str) -> Result<Model, MjcfError> {
    std::fs::write(dir.join(file), bytes).unwrap();
    load(dir, file, attrs)
}

/// The mesh's vertex count, its faces as flat vertex ids, and the body's
/// mass.
fn mesh_and_mass(model: &Model) -> (usize, Vec<u32>, f64) {
    let mesh = &model.mesh_data[0];
    let faces = mesh.triangles().iter().flatten().copied().collect();
    (mesh.vertex_count(), faces, model.body_mass[1])
}

/// The cube's faces as MuJoCo 3.5.0 numbers its eight vertices, in the order
/// they first appear in the file.
const CUBE_FACES: [u32; 36] = [
    0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7, 0, 3, 5, 0, 5, 4, 1, 7, 6, 1, 6, 2, 0, 4, 7, 0, 7, 1, 3, 2,
    6, 3, 6, 5,
];

/// An STL's vertices equal as floats are one, numbered as they first
/// appear: the cube's 36 are 8, and its faces MuJoCo's, in every inertia
/// mode MuJoCo computes from the faces; a file is an STL by its extension in
/// any case (`mjuu_extToContentType` compares it with `strcasecmp`), the text
/// after its last `.`, so a file named `.stl` is one (MuJoCo loads it).
#[test]
fn an_stls_repeated_vertices_are_one() {
    let dir = tempfile::tempdir().unwrap();
    let bytes = binary_stl(b"binary", &soup(&cube_corners(), &CUBE));
    for (mode, mass) in [("legacy", 1000.0), ("exact", 1000.0), ("shell", 6000.0)] {
        let model = write_and_load(
            dir.path(),
            "cube.stl",
            &bytes,
            &format!(r#"inertia="{mode}""#),
        )
        .unwrap();
        assert_eq!(
            mesh_and_mass(&model),
            (8, CUBE_FACES.to_vec(), mass),
            "{mode}"
        );
    }
    for file in ["CUBE.Stl", ".stl"] {
        let model = write_and_load(dir.path(), file, &bytes, "").unwrap();
        assert_eq!(
            mesh_and_mass(&model),
            (8, CUBE_FACES.to_vec(), 1000.0),
            "{file}"
        );
    }
}

/// `-0` and `0` are one vertex, whichever appears first (MuJoCo: 4
/// vertices, mass 999.9999999999999 both ways).
#[test]
fn negative_zero_is_zero() {
    let dir = tempfile::tempdir().unwrap();
    let corners = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 2.0, 0.0],
        [0.0, 0.0, 3.0],
    ];
    let tet_faces = vec![0, 1, 2, 0, 2, 3, 0, 3, 1, 2, 1, 3];
    for which in [0, 1] {
        let mut faces = soup(&corners, &[[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]]);
        faces[which][0] = [-0.0, -0.0, 0.0];
        let bytes = binary_stl(b"binary", &faces);
        let model = write_and_load(dir.path(), "tet.stl", &bytes, "").unwrap();
        assert_eq!(
            mesh_and_mass(&model),
            (4, tet_faces.clone(), 999.9999999999999),
            "{which}"
        );
    }
}

/// Under a `scale` whose product is not positive each face's last two
/// vertices change places, so the mirrored mesh keeps its volume: MuJoCo's
/// faces and masses in each mode; two negative factors do not swap, and a
/// product of 0 does.
#[test]
fn a_left_handed_scale_turns_each_face() {
    let dir = tempfile::tempdir().unwrap();
    let cube = binary_stl(b"binary", &soup(&cube_corners(), &CUBE));
    let mirrored_cube = vec![
        0, 2, 1, 0, 3, 2, 4, 6, 5, 4, 7, 6, 0, 5, 3, 0, 4, 5, 1, 6, 7, 1, 2, 6, 0, 7, 4, 0, 1, 7,
        3, 6, 2, 3, 5, 6,
    ];
    for mode in ["legacy", "exact"] {
        let model = write_and_load(
            dir.path(),
            "cube.stl",
            &cube,
            &format!(r#"scale="-1 1 1" inertia="{mode}""#),
        )
        .unwrap();
        assert_eq!(
            mesh_and_mass(&model),
            (8, mirrored_cube.clone(), 1000.0),
            "{mode}"
        );
    }
    // A flattened cube, which only `shell` weighs (MuJoCo: "mesh volume is
    // too small" in the other modes).
    for scale in ["0 1 1", "0 -1 1"] {
        let attrs = format!(r#"scale="{scale}" inertia="shell""#);
        let model = write_and_load(dir.path(), "cube.stl", &cube, &attrs).unwrap();
        assert_eq!(
            mesh_and_mass(&model),
            (8, mirrored_cube.clone(), 2000.0),
            "{scale}"
        );
    }
    let tet = binary_stl(b"binary", &soup(&TET_CORNERS, &TET));
    let turned = vec![0, 2, 1, 0, 3, 2, 0, 1, 3, 2, 3, 1];
    let unturned = vec![0, 1, 2, 0, 2, 3, 0, 3, 1, 2, 1, 3];
    for (attrs, faces, mass) in [
        (
            r#"scale="1 -2 1" inertia="legacy""#,
            &turned,
            2000.0000496705366,
        ),
        (
            r#"scale="1 -2 1" inertia="exact""#,
            &turned,
            2000.0000496705366,
        ),
        (
            r#"scale="1 -2 1" inertia="shell""#,
            &turned,
            16000.000150157852,
        ),
        (
            r#"scale="-1 -1 1" inertia="exact""#,
            &unturned,
            1000.000024835268,
        ),
        (
            r#"scale="-1 -1 -1" inertia="exact""#,
            &turned,
            1000.000024835268,
        ),
    ] {
        let model = write_and_load(dir.path(), "tet.stl", &tet, attrs).unwrap();
        assert_eq!(mesh_and_mass(&model), (4, faces.clone(), mass), "{attrs}");
    }
}

/// The reason a load failed, past the mesh's name.
fn reason(result: Result<Model, MjcfError>) -> String {
    let err = result.map(|_| ()).expect_err("refused");
    err.to_string()
}

/// A coordinate that is not finite is refused, naming the index of its
/// vertex's first float (MuJoCo: 48 for the seventeenth vertex, 0 for the
/// first); one above 2^30 in magnitude, an infinity among them, is refused
/// as out of bounds (MuJoCo's words name its resolved path; these the file
/// as written); 2^30 itself is read.
#[test]
fn an_stl_coordinate_not_finite_or_out_of_bounds_is_refused() {
    let dir = tempfile::tempdir().unwrap();
    let with = |face: usize, vertex: usize, value: f32| {
        let mut faces = soup(&cube_corners(), &CUBE);
        faces[face][vertex][0] = value;
        binary_stl(b"binary", &faces)
    };
    for (face, vertex, value, words) in [
        (5, 1, f32::NAN, "vertex coordinate 48 is not finite"),
        (0, 0, f32::NAN, "vertex coordinate 0 is not finite"),
        (
            5,
            1,
            f32::INFINITY,
            "vertex in STL file 'c.stl' exceed maximum bounds",
        ),
        (
            5,
            1,
            f32::NEG_INFINITY,
            "vertex in STL file 'c.stl' exceed maximum bounds",
        ),
        (
            5,
            1,
            1.5e9,
            "vertex in STL file 'c.stl' exceed maximum bounds",
        ),
        (
            5,
            1,
            1_073_741_952.0,
            "vertex in STL file 'c.stl' exceed maximum bounds",
        ),
    ] {
        let got = reason(write_and_load(
            dir.path(),
            "c.stl",
            &with(face, vertex, value),
            "",
        ));
        assert_eq!(got, format!("mesh 'm': {words}"), "{value}");
    }
    // 2^30 is read; the mesh it makes is too thin to weigh anything here, as
    // in MuJoCo.
    let got = reason(write_and_load(
        dir.path(),
        "c.stl",
        &with(5, 1, 1_073_741_824.0),
        "",
    ));
    assert!(
        got.contains("mass and inertia of moving bodies must be larger than mjMINVAL"),
        "{got}"
    );
}

/// A binary file whose header begins with `solid` is read by its size
/// (ledger L43 (b)).
#[test]
fn a_binary_stl_with_a_solid_header_loads() {
    let dir = tempfile::tempdir().unwrap();
    let bytes = binary_stl(
        b"solid cube made by a CAD tool",
        &soup(&cube_corners(), &CUBE),
    );
    let model = write_and_load(dir.path(), "cube.stl", &bytes, "").unwrap();
    assert_eq!(mesh_and_mass(&model), (8, CUBE_FACES.to_vec(), 1000.0));
}

/// A file MuJoCo cannot read as a binary STL, and this crate cannot read as
/// an ASCII one, is refused in MuJoCo's order and words: among them two
/// binary files whose header begins with `solid`, one not UTF-8 and one
/// UTF-8 but for its NUL bytes, and two texts beginning `solid` with no
/// vertex line, which are read as binary as MuJoCo reads them.
#[test]
fn an_stl_of_the_wrong_size_is_refused_as_mujoco_refuses_it() {
    let dir = tempfile::tempdir().unwrap();
    let cube = binary_stl(b"binary", &soup(&cube_corners(), &CUBE));
    let mut trailing = cube.clone();
    trailing.extend_from_slice(&[0; 10]);
    let truncated = cube[..cube.len() - 10].to_vec();
    let mut count_short = cube;
    count_short[80..84].copy_from_slice(&11u32.to_le_bytes());
    let mut solid_trailing = binary_stl(b"solid cube", &soup(&cube_corners(), &CUBE));
    solid_trailing.extend_from_slice(&[0; 10]);
    // A cube of side 2 at the origin: each byte past the header 0x00 or 0x40.
    let corners: Vec<[f32; 3]> = cube_corners()
        .iter()
        .map(|c| c.map(|x| 2.0 * x + 1.0))
        .collect();
    let mut solid_nul_trailing = binary_stl(b"solid cube", &soup(&corners, &CUBE));
    solid_nul_trailing.extend_from_slice(&[0; 10]);
    assert!(std::str::from_utf8(&solid_nul_trailing).is_ok() && solid_nul_trailing.contains(&0));
    for (bytes, words) in [
        (Vec::new(), "STL file 'c.stl' is empty"),
        (vec![1; 50], "invalid header in STL file 'c.stl'"),
        (
            binary_stl(b"binary", &[]),
            "number of faces should be between 1 and 200000 in STL file 'c.stl'; perhaps this is an ASCII file?",
        ),
        (
            trailing,
            "STL file 'c.stl' has wrong size; perhaps this is an ASCII file?",
        ),
        (
            truncated,
            "STL file 'c.stl' has wrong size; perhaps this is an ASCII file?",
        ),
        (
            count_short,
            "STL file 'c.stl' has wrong size; perhaps this is an ASCII file?",
        ),
        (
            solid_trailing,
            "STL file 'c.stl' has wrong size; perhaps this is an ASCII file?",
        ),
        (
            solid_nul_trailing,
            "STL file 'c.stl' has wrong size; perhaps this is an ASCII file?",
        ),
        (
            b"solid\nendsolid\n".to_vec(),
            "invalid header in STL file 'c.stl'",
        ),
        (
            [&b"solid"[..], &[b' '; 79]].concat(),
            "number of faces should be between 1 and 200000 in STL file 'c.stl'; perhaps this is an ASCII file?",
        ),
    ] {
        let got = reason(write_and_load(dir.path(), "c.stl", &bytes, ""));
        assert_eq!(got, format!("mesh 'm': {words}"));
    }
}

/// An ASCII STL, which MuJoCo refuses, is read as its binary twin: each
/// number rounded to single precision, so the two load to the same model
/// (book L36, Q118). MuJoCo's mass for the binary file: 231.00000943243512.
#[test]
fn an_ascii_stl_loads_as_its_binary_twin() {
    let dir = tempfile::tempdir().unwrap();
    let corners: Vec<[f32; 3]> = cube_corners()
        .iter()
        .map(|&[x, y, z]| [x * 0.3 + 0.1, y * 0.7 + 0.2, z * 1.1 + 0.3])
        .collect();
    let faces = soup(&corners, &CUBE);
    let binary = write_and_load(dir.path(), "b.stl", &binary_stl(b"binary", &faces), "").unwrap();
    let ascii = write_and_load(dir.path(), "a.stl", ascii_stl(&faces).as_bytes(), "").unwrap();
    assert_eq!(
        mesh_and_mass(&binary),
        (8, CUBE_FACES.to_vec(), 231.00000943243512)
    );
    assert_eq!(mesh_and_mass(&ascii), mesh_and_mass(&binary));
    assert_eq!(ascii.body_inertia[1], binary.body_inertia[1]);
    assert_eq!(ascii.body_ipos[1], binary.body_ipos[1]);
    assert_eq!(ascii.body_iquat[1], binary.body_iquat[1]);
    assert_eq!(
        ascii.mesh_data[0].vertices(),
        binary.mesh_data[0].vertices()
    );
    // Past leading whitespace, its keywords in any case, as mesh-io reads one.
    let shouted = format!(" \n\t{}", ascii_stl(&faces).replace("vertex", "VERTEX"));
    let shouted = write_and_load(dir.path(), "s.stl", shouted.as_bytes(), "").unwrap();
    assert_eq!(mesh_and_mass(&shouted), mesh_and_mass(&binary));
}

/// An ASCII STL that does not hold three numbers to a vertex is refused
/// naming the file and the line, and one whose vertices do not make whole
/// faces naming the file and their count (MuJoCo reads no ASCII STL: these
/// words are this crate's).
#[test]
fn a_malformed_ascii_stl_is_refused() {
    let dir = tempfile::tempdir().unwrap();
    let text = ascii_stl(&soup(&cube_corners(), &CUBE));
    let fourth = "      vertex -0.5 -0.5 0.5\n";
    for (bad, words) in [
        (
            text.replacen("vertex -0.5 -0.5 -0.5", "vertex -0.5 -0.5", 1),
            "in ASCII STL file 'a.stl', line 4: a vertex has 2 numbers, not 3",
        ),
        (
            text.replacen("vertex -0.5 -0.5 -0.5", "vertex -0.5 -0.5 -0.5 1", 1),
            "in ASCII STL file 'a.stl', line 4: a vertex has 4 numbers, not 3",
        ),
        (
            text.replacen("vertex -0.5 -0.5 -0.5", "vertex -0.5 -0.5x -0.5", 1),
            "in ASCII STL file 'a.stl', line 4: invalid float literal",
        ),
        (
            text.replacen("    endloop", &format!("{fourth}    endloop"), 1),
            "ASCII STL file 'a.stl' holds 37 vertices, not three to a face",
        ),
    ] {
        let got = reason(write_and_load(dir.path(), "a.stl", bad.as_bytes(), ""));
        assert_eq!(got, format!("mesh 'm': {words}"));
    }
}

/// The cube's surface cut into a `cells` by `cells` grid per side, two
/// faces a cell, each turned outward.
fn grid_cube(cells: u16) -> Vec<[[f32; 3]; 3]> {
    let mut faces = Vec::new();
    let at = |k: u16| -0.5 + f32::from(k) / f32::from(cells);
    for axis in 0..3 {
        for side in [-0.5f32, 0.5] {
            let point = |first: f32, second: f32| {
                let mut corner = [0.0; 3];
                corner[axis] = side;
                corner[(axis + 1) % 3] = first;
                corner[(axis + 2) % 3] = second;
                corner
            };
            for row in 0..cells {
                for col in 0..cells {
                    let [c00, c10, c11, c01] = [
                        point(at(row), at(col)),
                        point(at(row + 1), at(col)),
                        point(at(row + 1), at(col + 1)),
                        point(at(row), at(col + 1)),
                    ];
                    if side > 0.0 {
                        faces.extend([[c00, c10, c11], [c00, c11, c01]]);
                    } else {
                        faces.extend([[c00, c11, c10], [c00, c01, c11]]);
                    }
                }
            }
        }
    }
    faces
}

/// An STL of more than 200,000 faces, which MuJoCo refuses, is read (book
/// L36, Q118) as the same faces read under the limit would be: its 202,800
/// faces and 101,402 vertices equal those of an OBJ of its vertices as they
/// first appear (MuJoCo loads that OBJ: 101,402 vertices, 202,800 faces,
/// mass 1000.0000000000007, 2026-11-16).
#[test]
fn an_stl_of_more_than_200000_faces_loads() {
    let dir = tempfile::tempdir().unwrap();
    let faces = grid_cube(130);
    let stl = write_and_load(dir.path(), "big.stl", &binary_stl(b"binary", &faces), "").unwrap();
    let mut ids = std::collections::HashMap::new();
    let mut obj = String::new();
    let mut face_lines = String::new();
    for face in &faces {
        face_lines.push('f');
        for v in face {
            let next = ids.len() + 1;
            let id = *ids.entry(v.map(f32::to_bits)).or_insert_with(|| {
                writeln!(obj, "v {:?} {:?} {:?}", v[0], v[1], v[2]).unwrap();
                next
            });
            write!(face_lines, " {id}").unwrap();
        }
        face_lines.push('\n');
    }
    obj.push_str(&face_lines);
    let from_obj = write_and_load(dir.path(), "big.obj", obj.as_bytes(), "").unwrap();
    let (nvert, nface) = (
        stl.mesh_data[0].vertex_count(),
        stl.mesh_data[0].triangle_count(),
    );
    assert_eq!((nvert, nface), (101_402, 202_800));
    assert_eq!(mesh_and_mass(&stl), mesh_and_mass(&from_obj));
    assert_eq!(stl.body_mass[1], 1000.0000000000007);
}

/// With its repeated vertices one, an `exact` STL's faces are checked for
/// their orientation as an embedded mesh's are: one face turned inward, or
/// one face twice, is refused naming the vertices MuJoCo names (ledger
/// L411 (a)); under `legacy` both load, the doubled face weighing twice.
#[test]
fn an_exact_stl_with_a_misoriented_face_is_refused() {
    let dir = tempfile::tempdir().unwrap();
    let mut inward = soup(&cube_corners(), &CUBE);
    inward[7].swap(1, 2);
    let mut doubled = soup(&cube_corners(), &CUBE);
    doubled.push(doubled[3]);
    for (faces, words) in [(&inward, "2 and 3"), (&doubled, "5 and 7")] {
        let got = reason(write_and_load(
            dir.path(),
            "c.stl",
            &binary_stl(b"binary", faces),
            r#"inertia="exact""#,
        ));
        assert_eq!(
            got,
            format!(
                "mesh 'm': its faces have inconsistent orientation. Please check the faces \
                 containing the vertices {words}."
            )
        );
    }
    let model = write_and_load(dir.path(), "c.stl", &binary_stl(b"binary", &doubled), "").unwrap();
    let (nvert, faces, mass) = mesh_and_mass(&model);
    assert_eq!((nvert, faces.len(), mass), (8, 39, 1078.754578754579));
}
