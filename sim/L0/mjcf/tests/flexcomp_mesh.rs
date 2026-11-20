//! A `<flexcomp type="mesh">` made as MuJoCo 3.5.0's `MakeMesh` makes it
//! (`user_flexcomp.cc:1132-1219`; ledger L398).
//!
//! `MakeMesh` reads the flexcomp's mesh file, merging its repeated vertices
//! whatever its format, and makes the file's vertices the flexcomp's points,
//! in place of any it writes. In dim 2 the file's faces are its elements, in
//! place of any it writes. In dim 3 `origin` is its first point, and each face
//! that makes a tetrahedron of a volume of at least `mjMINVAL` with it adds
//! that tetrahedron after the elements it writes. Make then goes on as it
//! does for a direct type.
//!
//! Each row holds a document's parts, which [`document`] puts together. Each
//! document was loaded by MuJoCo 3.5.0 on the census's unfused oracle
//! (2026-11-19) from a directory holding it and its files, the same bytes
//! this file writes beside it. Each `LOADS` row is MuJoCo's flex: its dim,
//! its elements, each vertex's body and its world position after a forward.
//! Each `REFUSED` row holds MuJoCo's reason, which this crate gives, word for
//! word, on the attribute the row names. Both refuse each `OTHER_WORDS` row,
//! this crate in its own words: a missing file, and a file of no known type,
//! which this crate names as written and MuJoCo by its whole path. MuJoCo
//! refuses both `EXTENSION` rows, a PLY file and an ASCII STL. This crate
//! reads both (registry `D-MESH-FORMAT`, `D-STL-ASCII`) as the OBJ and the
//! binary STL of the same tetrahedron, which MuJoCo loads.
//!
//! A pinned point, which MuJoCo puts on the body holding the flexcomp, is a
//! jointless, massless body of its own here (registry `D-FLEX-PINNED-BODY`).

// The GENERATED values are printed as Python's `repr` prints them, which
// round-trips each f64 exactly.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::path::Path;

use sim_mjcf::{MjcfError, load_model_from_file, parse_mjcf_str, validate};

/// MuJoCo's model of one document: its body count, and its one flex.
struct Expected {
    nbody: usize,
    dim: usize,
    elem: &'static [usize],
    vertbody: &'static [usize],
    xpos: &'static [[f64; 3]],
}

/// A tetrahedron as an OBJ file, each face turned outward.
const TET_OBJ: &str = "v 0 0 0\nv 1 0 0\nv 0 2 0\nv 0 0 3\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n";

/// The tetrahedron's faces as its STL lists them.
const TET_TRIANGLES: [[[f32; 3]; 3]; 4] = [
    [[0.0, 0.0, 0.0], [0.0, 2.0, 0.0], [1.0, 0.0, 0.0]],
    [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 3.0]],
    [[0.0, 0.0, 0.0], [0.0, 0.0, 3.0], [0.0, 2.0, 0.0]],
    [[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 3.0]],
];

/// The files the documents read, by name, each the bytes MuJoCo read.
fn files() -> Vec<(&'static str, Vec<u8>)> {
    let text = |s: &str| s.as_bytes().to_vec();
    let msh = msh(
        &[
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 2.0, 0.0],
            [0.0, 0.0, 3.0],
        ],
        &[[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
    );
    vec![
        ("tet.obj", text(TET_OBJ)),
        // An OBJ's text under a name of no known type.
        ("tet.xyz", text(TET_OBJ)),
        // Its first vertex again, last; the first face names it.
        (
            "tet_dup.obj",
            text(
                "v 0 0 0\nv 1 0 0\nv 0 2 0\nv 0 0 3\nv 0 0 0\nf 5 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n",
            ),
        ),
        (
            "tet_vt.obj",
            text(&format!("{TET_OBJ}vt 0 0\nvt 1 0\nvt 0 1\nvt 1 1\n")),
        ),
        // Its second face names a ninth vertex.
        (
            "tet_badidx.obj",
            text("v 0 0 0\nv 1 0 0\nv 0 2 0\nv 0 0 3\nf 1 3 2\nf 1 2 9\nf 1 4 3\nf 2 3 4\n"),
        ),
        // Its faces counted back from the last vertex read.
        (
            "tet_neg.obj",
            text(
                "v 0 0 0\nv 1 0 0\nv 0 2 0\nv 0 0 3\nf -4 -2 -3\nf -4 -3 -1\nf -4 -1 -2\nf -3 -2 -1\n",
            ),
        ),
        // MuJoCo reads only the first shape's faces.
        (
            "twoshapes.obj",
            text(
                "v 0 0 0\nv 1 0 0\nv 0 2 0\nv 0 0 3\no first\nf 1 3 2\nf 1 2 4\no second\nf 1 4 3\nf 2 3 4\n",
            ),
        ),
        (
            "lines.obj",
            text("v 0 0 0\nv 1 0 0\nv 0 2 0\nv 0 0 3\nl 1 2\n"),
        ),
        ("empty.obj", Vec::new()),
        // A unit square in z = 0, two triangles.
        (
            "flat.obj",
            text("v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1 2 3\nf 1 3 4\n"),
        ),
        // A square pyramid, its base one quad.
        (
            "pyramid.obj",
            text(
                "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nv 0.5 0.5 1\nf 1 4 3 2\nf 1 2 5\nf 2 3 5\nf 3 4 5\nf 4 1 5\n",
            ),
        ),
        ("tet.stl", stl(&TET_TRIANGLES)),
        ("tet_ascii.stl", text(&ascii_stl(&TET_TRIANGLES))),
        ("tet.msh", msh.clone()),
        ("TETU.MSH", msh),
        (
            "tet.ply",
            text(
                "ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\nproperty float y\nproperty float z\nelement face 4\nproperty list uchar int vertex_indices\nend_header\n0 0 0\n1 0 0\n0 2 0\n0 0 3\n3 0 2 1\n3 0 1 3\n3 0 3 2\n3 1 2 3\n",
            ),
        ),
    ]
}

/// A binary STL of `triangles`, each face's normal 0.
fn stl(triangles: &[[[f32; 3]; 3]]) -> Vec<u8> {
    let mut out = vec![0; 80];
    out.extend(u32::try_from(triangles.len()).unwrap().to_le_bytes());
    for triangle in triangles {
        out.extend([0.0_f32; 3].iter().flat_map(|x| x.to_le_bytes()));
        out.extend(triangle.iter().flatten().flat_map(|x| x.to_le_bytes()));
        out.extend([0, 0]);
    }
    out
}

/// An ASCII STL of `triangles`, each coordinate a whole number.
fn ascii_stl(triangles: &[[[f32; 3]; 3]]) -> String {
    use std::fmt::Write;
    let mut out = String::from("solid t\n");
    for triangle in triangles {
        out.push_str("facet normal 0 0 0\nouter loop\n");
        for [x, y, z] in triangle {
            writeln!(out, "vertex {x} {y} {z}").unwrap();
        }
        out.push_str("endloop\nendfacet\n");
    }
    out.push_str("endsolid t\n");
    out
}

/// An MSH file of `vertices` and `faces`: its four counts (no normals, no
/// texture coordinates), then the vertices' floats and the faces' ids.
fn msh(vertices: &[[f32; 3]], faces: &[[i32; 3]]) -> Vec<u8> {
    let count = |n: usize| i32::try_from(n).unwrap().to_le_bytes();
    let mut out = Vec::new();
    for n in [vertices.len(), 0, 0, faces.len()] {
        out.extend(count(n));
    }
    out.extend(vertices.iter().flatten().flat_map(|x| x.to_le_bytes()));
    out.extend(faces.iter().flatten().flat_map(|i| i.to_le_bytes()));
    out
}

/// A directory holding [`files`] in `assets/`, as MuJoCo read them.
fn with_files() -> tempfile::TempDir {
    let dir = tempfile::tempdir().unwrap();
    std::fs::create_dir(dir.path().join("assets")).unwrap();
    for (name, bytes) in files() {
        std::fs::write(dir.path().join("assets").join(name), bytes).unwrap();
    }
    dir
}

/// A document's parts: its `<compiler>` element (or none), its flexcomp's
/// attributes past its name and type, and the flexcomp's children past its
/// `<edge>`.
type Parts = (&'static str, &'static str, &'static str);

/// The document MuJoCo loaded from `parts`: one mesh flexcomp, `f`, in the
/// body `h`.
fn document((compiler, attributes, children): Parts) -> String {
    format!(
        r#"<mujoco>{compiler}<worldbody><body name="h" pos="0 0 1"><flexcomp name="f" type="mesh" {attributes}><edge equality="false"/>{children}</flexcomp></body></worldbody></mujoco>"#
    )
}

/// The [`document`] of `parts` loaded from a file in `dir`, as MuJoCo loaded
/// it, `@DIR@` in it standing for `dir`.
fn load_in(dir: &Path, id: &str, parts: Parts) -> Result<sim_core::Model, MjcfError> {
    let path = dir.join(format!("{id}.xml"));
    let doc = document(parts).replace("@DIR@", &dir.to_string_lossy());
    std::fs::write(&path, doc).unwrap();
    load_model_from_file(&path)
}

/// A loader value against MuJoCo's: equal on macOS arm64, where both run the
/// same libm; elsewhere within 1e-12 relative (a `NaN` fails either way).
// Exact on the one platform the expected bits are from, by design.
#[allow(clippy::float_cmp)]
fn same(ours: f64, mujoco: f64) -> bool {
    if cfg!(all(target_os = "macos", target_arch = "aarch64")) {
        ours == mujoco
    } else {
        (ours - mujoco).abs() <= 1e-12 * (1.0 + mujoco.abs())
    }
}

/// Whether body `b` is a pinned point's stand-in: jointless, on the world,
/// massless.
// A stand-in's mass is exactly 0: it is written, not computed.
#[allow(clippy::float_cmp)]
fn stand_in(model: &sim_core::Model, b: usize) -> bool {
    model.body_jnt_num[b] == 0 && model.body_parent[b] == 0 && model.body_mass[b] == 0.0
}

/// Every difference between this crate's model and MuJoCo's flex `e`.
fn differences(what: &str, model: &sim_core::Model, e: &Expected) -> Vec<String> {
    let mut data = model.make_data();
    data.forward(model).expect("forward");
    let mut out = Vec::new();
    let mut differ = |field: &str, ours: String, mujoco: String| {
        if ours != mujoco {
            out.push(format!("{what}: {field}: ours {ours}, MuJoCo {mujoco}"));
        }
    };
    // A body past MuJoCo's is a pinned point's stand-in.
    let extra: Vec<usize> = (e.nbody..model.nbody).collect();
    differ(
        "bodies past MuJoCo's all stand-ins",
        (model.nbody >= e.nbody && extra.iter().all(|&b| stand_in(model, b))).to_string(),
        "true".into(),
    );
    differ("nflex", model.nflex.to_string(), "1".into());
    if model.nflex != 1 {
        return out;
    }
    differ("dim", model.flex_dim[0].to_string(), e.dim.to_string());
    let base = model.flex_vertadr[0];
    let mut elem = Vec::new();
    for el in model.flex_elemadr[0]..model.flex_elemadr[0] + model.flex_elemnum[0] {
        let adr = model.flexelem_dataadr[el];
        for k in 0..model.flexelem_datanum[el] {
            elem.push(model.flexelem_data[adr + k] - base);
        }
    }
    differ("elements", format!("{elem:?}"), format!("{:?}", e.elem));
    differ(
        "nflexvert",
        model.flex_vertnum[0].to_string(),
        e.vertbody.len().to_string(),
    );
    for (v, &mj) in e.vertbody.iter().enumerate().take(model.flex_vertnum[0]) {
        let ours = model.flexvert_bodyid[base + v];
        // MuJoCo's body 1 holds the flexcomp; a point it pins is a stand-in here.
        let ok = ours == mj || (mj == 1 && ours >= e.nbody && stand_in(model, ours));
        differ(&format!("vertex {v} body"), ok.to_string(), "true".into());
        let xpos: [f64; 3] = data.flexvert_xpos[base + v].into();
        if !xpos.iter().zip(e.xpos[v]).all(|(&a, b)| same(a, b)) {
            differ(
                &format!("vertex {v} xpos"),
                format!("{xpos:?}"),
                format!("{:?}", e.xpos[v]),
            );
        }
    }
    out
}

#[test]
fn a_mesh_flexcomp_mujoco_loads_is_made_as_mujoco_makes_it() {
    let dir = with_files();
    let mut missed = Vec::new();
    for &(what, parts, ref e) in LOADS {
        match load_in(dir.path(), &what[..3], parts) {
            Ok(model) => missed.extend(differences(what, &model, e)),
            Err(err) => missed.push(format!("{what}: refused: {err}")),
        }
    }
    assert!(missed.is_empty(), "{}", missed.join("\n"));
}

#[test]
fn a_mesh_flexcomp_mujoco_refuses_is_refused_for_its_reason() {
    let dir = with_files();
    let mut missed = Vec::new();
    for &(what, parts, attribute, reason) in REFUSED {
        match load_in(dir.path(), &what[..3], parts) {
            Err(MjcfError::InvalidValue {
                element,
                attribute: a,
                reason: r,
                ..
            }) if element == "flexcomp" && a == attribute && r == reason => {}
            Err(e) => missed.push(format!("{what}: {e} (MuJoCo: {reason}, on {attribute})")),
            Ok(_) => missed.push(format!("{what}: loads (MuJoCo: {reason})")),
        }
    }
    assert!(missed.is_empty(), "{}", missed.join("\n"));
}

#[test]
fn a_missing_file_or_one_of_no_known_type_is_refused_in_this_crates_words() {
    let dir = with_files();
    let mut missed = Vec::new();
    for &(what, parts, ours, _mujoco) in OTHER_WORDS {
        match load_in(dir.path(), &what[..3], parts) {
            Err(e) if e.to_string().contains(ours) => {}
            Err(e) => missed.push(format!("{what}: {e} (expected: {ours})")),
            Ok(_) => missed.push(format!("{what}: loads")),
        }
    }
    assert!(missed.is_empty(), "{}", missed.join("\n"));
}

#[test]
fn a_ply_or_an_ascii_stl_flexcomp_is_made_as_its_twin_mujoco_loads() {
    let dir = with_files();
    let mut missed = Vec::new();
    for &(what, parts, twin) in EXTENSION {
        let (_, _, e) = LOADS.iter().find(|(w, _, _)| w.starts_with(twin)).unwrap();
        match load_in(dir.path(), &what[..3], parts) {
            Ok(model) => missed.extend(differences(what, &model, e)),
            Err(err) => missed.push(format!("{what}: refused: {err}")),
        }
    }
    assert!(missed.is_empty(), "{}", missed.join("\n"));
}

/// A dim-3 face naming a vertex the file does not hold. MuJoCo's volume test
/// reads past its points there, which C++ leaves undefined: on this document
/// MuJoCo 3.5.0 drops the face and loads, in each of 5 runs (measured
/// 2026-11-19). This crate keeps the tetrahedron untested, and Make's element
/// check refuses it (registry `D-FLEXCOMP-MESH-UNDEFINED`): the first face's
/// tetrahedron is kept, the second names point 9 of 5.
#[test]
fn a_dim_3_face_naming_no_vertex_is_refused_by_makes_element_check() {
    let dir = with_files();
    let parts = (
        "",
        r#"file="assets/tet_badidx.obj" dim="3" origin="0.2 0.3 0.4""#,
        "",
    );
    let err = load_in(dir.path(), "bad3", parts).unwrap_err();
    assert_eq!(
        err.to_string(),
        "invalid element on <flexcomp>: element 7 has point id 9, number of points is 5 (flex 'f')"
    );
}

/// [`validate`] has no model directory, so a mesh flexcomp naming a relative
/// file with no `meshdir` or `assetdir` is refused there, where the loader,
/// given the directory, makes it.
#[test]
fn validate_refuses_a_relative_mesh_file_it_has_no_directory_for() {
    let dir = with_files();
    let parts = ("", r#"file="assets/tet.obj" dim="2""#, "");
    load_in(dir.path(), "rel", parts).unwrap();
    let err = validate(&parse_mjcf_str(&document(parts)).unwrap()).unwrap_err();
    assert!(
        err.to_string()
            .contains("asset file 'assets/tet.obj' is relative but no base path provided"),
        "{err}"
    );
}

/// An STL of more than 200,000 faces, which MuJoCo 3.5.0 refuses for a
/// flexcomp as for a `<mesh>` ("number of faces should be between 1 and
/// 200000"), and this crate reads (registry `D-STL-FACES`): one triangle
/// 200,001 times, its three vertices merged, on a rigid flexcomp.
#[test]
fn an_stl_of_more_than_200000_faces_makes_a_flexcomp() {
    let dir = with_files();
    let triangle = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    std::fs::write(
        dir.path().join("assets/big.stl"),
        stl(&vec![triangle; 200_001]),
    )
    .unwrap();
    let parts = ("", r#"file="assets/big.stl" dim="2" rigid="true""#, "");
    let model = load_in(dir.path(), "big", parts).unwrap();
    assert_eq!(
        (model.nflex, model.flex_elemnum[0], model.flex_vertnum[0]),
        (1, 200_001, 3)
    );
}

// GENERATED by S45/l9d/c3/tests/gen_tests.py from MuJoCo 3.5.0's answers (`mjflex.py`).
const LOADS: &[(&str, Parts, Expected)] = &[
    (
        "m01: obj dim2",
        ("<compiler />", r#"file="assets/tet.obj" dim="2""#, ""),
        Expected {
            nbody: 6,
            dim: 2,
            elem: &[0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3],
            vertbody: &[2, 3, 4, 5],
            xpos: &[
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [0.0, 0.0, 4.0],
            ],
        },
    ),
    (
        "m02: stl dim2",
        ("<compiler />", r#"file="assets/tet.stl" dim="2""#, ""),
        Expected {
            nbody: 6,
            dim: 2,
            elem: &[0, 1, 2, 0, 2, 3, 0, 3, 1, 2, 1, 3],
            vertbody: &[2, 3, 4, 5],
            xpos: &[
                [0.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [1.0, 0.0, 1.0],
                [0.0, 0.0, 4.0],
            ],
        },
    ),
    (
        "m03: obj dim3 origin inside",
        (
            "<compiler />",
            r#"file="assets/tet.obj" dim="3" origin="0.2 0.3 0.4""#,
            "",
        ),
        Expected {
            nbody: 7,
            dim: 3,
            elem: &[0, 3, 1, 2, 0, 2, 1, 4, 0, 4, 1, 3, 0, 3, 2, 4],
            vertbody: &[2, 3, 4, 5, 6],
            xpos: &[
                [0.2, 0.3, 1.4],
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [0.0, 0.0, 4.0],
            ],
        },
    ),
    (
        "m04: obj dim3 origin outside",
        (
            "<compiler />",
            r#"file="assets/tet.obj" dim="3" origin="-1 -1 -1""#,
            "",
        ),
        Expected {
            nbody: 6,
            dim: 3,
            elem: &[0, 2, 1, 3],
            vertbody: &[2, 3, 4, 5],
            xpos: &[
                [-1.0, -1.0, 0.0],
                [1.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [0.0, 0.0, 4.0],
            ],
        },
    ),
    (
        "m05: obj dim3 origin vertex",
        (
            "<compiler />",
            r#"file="assets/tet.obj" dim="3" origin="0 0 0""#,
            "",
        ),
        Expected {
            nbody: 6,
            dim: 3,
            elem: &[0, 2, 1, 3],
            vertbody: &[2, 3, 4, 5],
            xpos: &[
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [0.0, 0.0, 4.0],
            ],
        },
    ),
    (
        "m06: obj dup vertices dim2",
        ("<compiler />", r#"file="assets/tet_dup.obj" dim="2""#, ""),
        Expected {
            nbody: 6,
            dim: 2,
            elem: &[0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3],
            vertbody: &[2, 3, 4, 5],
            xpos: &[
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [0.0, 0.0, 4.0],
            ],
        },
    ),
    (
        "m12: points written too",
        (
            "<compiler />",
            r#"file="assets/tet.obj" dim="2" point="5 5 5" element="0 0 0""#,
            "",
        ),
        Expected {
            nbody: 6,
            dim: 2,
            elem: &[0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3],
            vertbody: &[2, 3, 4, 5],
            xpos: &[
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [0.0, 0.0, 4.0],
            ],
        },
    ),
    (
        "m13: strippath",
        (
            r#"<compiler strippath="true" meshdir="assets"/>"#,
            r#"file="nowhere/assets/tet.obj" dim="2""#,
            "",
        ),
        Expected {
            nbody: 6,
            dim: 2,
            elem: &[0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3],
            vertbody: &[2, 3, 4, 5],
            xpos: &[
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [0.0, 0.0, 4.0],
            ],
        },
    ),
    (
        "m14: meshdir",
        (
            r#"<compiler meshdir="assets"/>"#,
            r#"file="tet.obj" dim="2""#,
            "",
        ),
        Expected {
            nbody: 6,
            dim: 2,
            elem: &[0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3],
            vertbody: &[2, 3, 4, 5],
            xpos: &[
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [0.0, 0.0, 4.0],
            ],
        },
    ),
    (
        "m15: obj vt dim2",
        ("<compiler />", r#"file="assets/tet_vt.obj" dim="2""#, ""),
        Expected {
            nbody: 6,
            dim: 2,
            elem: &[0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3],
            vertbody: &[2, 3, 4, 5],
            xpos: &[
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [0.0, 0.0, 4.0],
            ],
        },
    ),
    (
        "m16: scale and pos",
        (
            "<compiler />",
            r#"file="assets/tet.obj" dim="2" scale="2 2 2" pos="1 0 0""#,
            "",
        ),
        Expected {
            nbody: 6,
            dim: 2,
            elem: &[0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3],
            vertbody: &[2, 3, 4, 5],
            xpos: &[
                [1.0, 0.0, 1.0],
                [3.0, 0.0, 1.0],
                [1.0, 4.0, 1.0],
                [1.0, 0.0, 7.0],
            ],
        },
    ),
    (
        "m21: dim3 origin on face plane",
        (
            "<compiler />",
            r#"file="assets/tet.obj" dim="3" origin="0.2 0.2 0""#,
            "",
        ),
        Expected {
            nbody: 7,
            dim: 3,
            elem: &[0, 2, 1, 4, 0, 4, 1, 3, 0, 3, 2, 4],
            vertbody: &[2, 3, 4, 5, 6],
            xpos: &[
                [0.2, 0.2, 1.0],
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [0.0, 0.0, 4.0],
            ],
        },
    ),
    (
        "m22: dim3 written element",
        (
            "",
            r#"file="assets/tet.obj" dim="3" origin="0.2 0.3 0.4" element="0 1 2 3""#,
            "",
        ),
        Expected {
            nbody: 7,
            dim: 3,
            elem: &[0, 1, 2, 3, 0, 3, 1, 2, 0, 2, 1, 4, 0, 4, 1, 3, 0, 3, 2, 4],
            vertbody: &[2, 3, 4, 5, 6],
            xpos: &[
                [0.2, 0.3, 1.4],
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [0.0, 0.0, 4.0],
            ],
        },
    ),
    (
        "m23: dim2 written point only",
        ("", r#"file="assets/tet.obj" dim="2" point="5 5 5""#, ""),
        Expected {
            nbody: 6,
            dim: 2,
            elem: &[0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3],
            vertbody: &[2, 3, 4, 5],
            xpos: &[
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [0.0, 0.0, 4.0],
            ],
        },
    ),
    (
        "m24: dim2 written element only",
        ("", r#"file="assets/tet.obj" dim="2" element="0 1 2""#, ""),
        Expected {
            nbody: 6,
            dim: 2,
            elem: &[0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3],
            vertbody: &[2, 3, 4, 5],
            xpos: &[
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [0.0, 0.0, 4.0],
            ],
        },
    ),
    (
        "n05: obj quad dim2",
        ("", r#"file="assets/pyramid.obj" dim="2""#, ""),
        Expected {
            nbody: 7,
            dim: 2,
            elem: &[0, 3, 2, 0, 2, 1, 0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4],
            vertbody: &[2, 3, 4, 5, 6],
            xpos: &[
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [1.0, 1.0, 1.0],
                [0.0, 1.0, 1.0],
                [0.5, 0.5, 2.0],
            ],
        },
    ),
    (
        "n06: rigid dim2",
        ("", r#"file="assets/tet.obj" dim="2" rigid="true""#, ""),
        Expected {
            nbody: 2,
            dim: 2,
            elem: &[0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3],
            vertbody: &[1, 1, 1, 1],
            xpos: &[
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [0.0, 0.0, 4.0],
            ],
        },
    ),
    (
        "n07: dim3 pin origin",
        (
            "",
            r#"file="assets/tet.obj" dim="3" origin="0.2 0.3 0.4""#,
            r#"<pin id="0"/>"#,
        ),
        Expected {
            nbody: 6,
            dim: 3,
            elem: &[0, 3, 1, 2, 0, 2, 1, 4, 0, 4, 1, 3, 0, 3, 2, 4],
            vertbody: &[1, 2, 3, 4, 5],
            xpos: &[
                [0.2, 0.3, 1.4],
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [0.0, 0.0, 4.0],
            ],
        },
    ),
    (
        "n08: dim3 dup",
        (
            "",
            r#"file="assets/tet_dup.obj" dim="3" origin="0.2 0.3 0.4""#,
            "",
        ),
        Expected {
            nbody: 7,
            dim: 3,
            elem: &[0, 3, 1, 2, 0, 2, 1, 4, 0, 4, 1, 3, 0, 3, 2, 4],
            vertbody: &[2, 3, 4, 5, 6],
            xpos: &[
                [0.2, 0.3, 1.4],
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [0.0, 0.0, 4.0],
            ],
        },
    ),
    (
        "n13: obj neg index dim2",
        ("", r#"file="assets/tet_neg.obj" dim="2""#, ""),
        Expected {
            nbody: 6,
            dim: 2,
            elem: &[0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3],
            vertbody: &[2, 3, 4, 5],
            xpos: &[
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [0.0, 0.0, 4.0],
            ],
        },
    ),
    (
        "n14: dim3 vt",
        (
            "",
            r#"file="assets/tet_vt.obj" dim="3" origin="0.2 0.3 0.4""#,
            "",
        ),
        Expected {
            nbody: 7,
            dim: 3,
            elem: &[0, 3, 1, 2, 0, 2, 1, 4, 0, 4, 1, 3, 0, 3, 2, 4],
            vertbody: &[2, 3, 4, 5, 6],
            xpos: &[
                [0.2, 0.3, 1.4],
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [0.0, 0.0, 4.0],
            ],
        },
    ),
    (
        "n15: dim3 scale pos",
        (
            "",
            r#"file="assets/tet.obj" dim="3" origin="0.2 0.3 0.4" scale="2 2 2" pos="1 0 0""#,
            "",
        ),
        Expected {
            nbody: 7,
            dim: 3,
            elem: &[0, 3, 1, 2, 0, 2, 1, 4, 0, 4, 1, 3, 0, 3, 2, 4],
            vertbody: &[2, 3, 4, 5, 6],
            xpos: &[
                [1.4, 0.6, 1.8],
                [1.0, 0.0, 1.0],
                [3.0, 0.0, 1.0],
                [1.0, 4.0, 1.0],
                [1.0, 0.0, 7.0],
            ],
        },
    ),
    (
        "n16: obj two shapes dim2",
        ("", r#"file="assets/twoshapes.obj" dim="2""#, ""),
        Expected {
            nbody: 6,
            dim: 2,
            elem: &[0, 2, 1, 0, 1, 3],
            vertbody: &[2, 3, 4, 5],
            xpos: &[
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [0.0, 0.0, 4.0],
            ],
        },
    ),
    (
        "n17: flat dim2",
        ("", r#"file="assets/flat.obj" dim="2""#, ""),
        Expected {
            nbody: 6,
            dim: 2,
            elem: &[0, 1, 2, 0, 2, 3],
            vertbody: &[2, 3, 4, 5],
            xpos: &[
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [1.0, 1.0, 1.0],
                [0.0, 1.0, 1.0],
            ],
        },
    ),
    (
        "n18: absolute file",
        ("", r#"file="@DIR@/assets/tet.obj" dim="2""#, ""),
        Expected {
            nbody: 6,
            dim: 2,
            elem: &[0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3],
            vertbody: &[2, 3, 4, 5],
            xpos: &[
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [0.0, 0.0, 4.0],
            ],
        },
    ),
    (
        "n19: stl dim3",
        (
            "",
            r#"file="assets/tet.stl" dim="3" origin="0.2 0.3 0.4""#,
            "",
        ),
        Expected {
            nbody: 7,
            dim: 3,
            elem: &[0, 2, 1, 3, 0, 3, 1, 4, 0, 4, 1, 2, 0, 2, 3, 4],
            vertbody: &[2, 3, 4, 5, 6],
            xpos: &[
                [0.2, 0.3, 1.4],
                [0.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [1.0, 0.0, 1.0],
                [0.0, 0.0, 4.0],
            ],
        },
    ),
    (
        "n20: dim3 written point",
        (
            "",
            r#"file="assets/tet.obj" dim="3" origin="0.2 0.3 0.4" point="9 9 9""#,
            "",
        ),
        Expected {
            nbody: 7,
            dim: 3,
            elem: &[0, 3, 1, 2, 0, 2, 1, 4, 0, 4, 1, 3, 0, 3, 2, 4],
            vertbody: &[2, 3, 4, 5, 6],
            xpos: &[
                [0.2, 0.3, 1.4],
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [0.0, 0.0, 4.0],
            ],
        },
    ),
];

const REFUSED: &[(&str, Parts, &str, &str)] = &[
    (
        "o01: no file at dim 1",
        ("<compiler />", r#"dim="1""#, ""),
        "file",
        "File is required",
    ),
    (
        "o02: an MSH file at dim 1",
        ("<compiler />", r#"file="assets/tet.msh" dim="1""#, ""),
        "dim",
        "Flex dim must be at least 2 for mesh",
    ),
    (
        "m07: empty obj",
        ("<compiler />", r#"file="assets/empty.obj" dim="2""#, ""),
        "file",
        "Vertex and face data required",
    ),
    (
        "m08: lines obj",
        ("<compiler />", r#"file="assets/lines.obj" dim="2""#, ""),
        "file",
        "Vertex and face data required",
    ),
    (
        "m10: msh",
        ("<compiler />", r#"file="assets/tet.msh" dim="2""#, ""),
        "file",
        "legacy MSH files are not supported in flexcomp",
    ),
    (
        "m17: dim1",
        ("<compiler />", r#"file="assets/tet.obj" dim="1""#, ""),
        "dim",
        "Flex dim must be at least 2 for mesh",
    ),
    (
        "m18: no file",
        ("<compiler />", r#"dim="2""#, ""),
        "file",
        "File is required",
    ),
    (
        "m19: dim3 no origin",
        ("<compiler />", r#"file="assets/tet.obj" dim="3""#, ""),
        "origin",
        "origin must be specified for mesh flexcomps if dim=3",
    ),
    (
        "m25: dim3 written element bad id",
        (
            "",
            r#"file="assets/tet.obj" dim="3" origin="0.2 0.3 0.4" element="0 1 2 9""#,
            "",
        ),
        "element",
        "element 3 has point id 9, number of points is 5",
    ),
    (
        "n01: obj bad index dim2",
        ("", r#"file="assets/tet_badidx.obj" dim="2""#, ""),
        "element",
        "element 5 has point id 8, number of points is 4",
    ),
    (
        "n02: flat dim3 origin in plane",
        (
            "",
            r#"file="assets/flat.obj" dim="3" origin="0.5 0.5 0""#,
            "",
        ),
        "point",
        "Point and element required",
    ),
    (
        "n03: upper msh",
        ("", r#"file="assets/TETU.MSH" dim="2""#, ""),
        "file",
        "legacy MSH files are not supported in flexcomp",
    ),
    (
        "n04: strippath dir only",
        (
            r#"<compiler strippath="true"/>"#,
            r#"file="assets/" dim="2""#,
            "",
        ),
        "file",
        "File is required",
    ),
    (
        "n10: missing msh",
        ("", r#"file="assets/nope.msh" dim="2""#, ""),
        "file",
        "legacy MSH files are not supported in flexcomp",
    ),
    (
        "n12: empty file attr",
        ("", r#"file="" dim="2""#, ""),
        "file",
        "File is required",
    ),
    (
        "n21: flat dim3 origin above",
        (
            "",
            r#"file="assets/flat.obj" dim="3" origin="0.5 0.5 1""#,
            "",
        ),
        "point",
        "Point and element required",
    ),
];

const OTHER_WORDS: &[(&str, Parts, &str, &str)] = &[
    (
        "m09: missing file",
        ("<compiler />", r#"file="assets/nope.obj" dim="2""#, ""),
        "asset file not found: 'assets/nope.obj'",
        "Error opening file 'assets/nope.obj'",
    ),
    (
        "m20: unknown ext",
        ("<compiler />", r#"file="assets/tet.xyz" dim="2""#, ""),
        "unknown or unsupported mesh file: 'assets/tet.xyz'",
        "unknown or unsupported mesh file: '<dir>/assets/tet.xyz'",
    ),
    (
        "n11: missing xyz",
        ("", r#"file="assets/nope.xyz" dim="2""#, ""),
        "asset file not found: 'assets/nope.xyz'",
        "Error opening file 'assets/nope.xyz'",
    ),
];

const EXTENSION: &[(&str, Parts, &str)] = &[
    (
        "m11: ply",
        ("<compiler />", r#"file="assets/tet.ply" dim="2""#, ""),
        "m01",
    ),
    (
        "n09: ascii stl dim2",
        ("", r#"file="assets/tet_ascii.stl" dim="2""#, ""),
        "m02",
    ),
];
