//! A mesh's data checked where and as MuJoCo 3.5.0 checks it (ledger L427,
//! L405's count half).
//!
//! MuJoCo checks a mesh's data in two places. As it begins to compile the
//! model, after reading it and before naming any asset
//! (`mjCModel::TryCompile`, `user_model.cc:4872`), it checks every mesh's
//! written vertex and face counts, in mesh order, each a multiple of 3
//! (`ProcessVertices`, `user_mesh.cc:543-552`). Then, when it compiles each
//! mesh, after every asset name is found and before any body
//! (`mjCMesh::TryCompile`, `user_mesh.cc:686-770`), it reads the file. It
//! takes the mesh's own normals, texture coordinates, faces and vertices
//! where the file gave none, and refuses them where the file gave some
//! ("repeated … specification"), its normals also where the file's faces
//! carry normals' ids, as an MSH's do. Last it checks the whole
//! (`CheckInitialMesh`, `:1806-1842`). sim-mjcf checks each in that place
//! and order.
//!
//! Every document below was loaded by MuJoCo 3.5.0 on the census's unfused
//! oracle (2026-11-19), its files the same bytes. It refuses each in
//! `REFUSED` for the reason that row's error gives, which sim-mjcf states in
//! its own form: the mesh's name before the reason, or the element after it.
//! Four MuJoCo refuses for a fault it meets before the mesh's data, as
//! sim-mjcf does, each in its own words: an unknown class, twice, which
//! MuJoCo refuses as it reads the model; a geom's missing mesh
//! (`IndexAssets`); and an STL of no faces. MuJoCo loads each in `LOADS`
//! with four vertices, four faces and a body mass of 999.9999999999999.
//!
//! Of two meshes MuJoCo refuses, it names whichever its threads compile
//! first unless `usethread` is false, when it names the first. sim-mjcf
//! always names the first. With threads, the first was named in 191, 191
//! and 197 of 200 loads of three such models.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_mjcf::{load_model, model_from_mjcf, parse_mjcf_str};
use std::path::Path;

/// A tetrahedron's vertices and its faces, turned outward.
const TET: [[f32; 3]; 4] = [
    [0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 2.0, 0.0],
    [0.0, 0.0, 3.0],
];
const TET_FACES: [[usize; 3]; 4] = [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];

/// The tetrahedron as an OBJ file.
const TET_OBJ: &str = "v 0 0 0\nv 1 0 0\nv 0 2 0\nv 0 0 3\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n";

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

/// An MSH file of the tetrahedron's vertices: its four counts (vertices,
/// normals, texture coordinates, faces), the vertices, `extra` (the normals'
/// and texture coordinates' floats) and `faces`.
fn msh(counts: [i32; 4], extra: &[f32], faces: &[i32]) -> Vec<u8> {
    let mut out: Vec<u8> = counts.iter().flat_map(|n| n.to_le_bytes()).collect();
    out.extend(TET.iter().flatten().flat_map(|x| x.to_le_bytes()));
    out.extend(extra.iter().flat_map(|x| x.to_le_bytes()));
    out.extend(faces.iter().flat_map(|n| n.to_le_bytes()));
    out
}

/// The files the documents name, written to `dir`.
fn write_files(dir: &Path) {
    let with = |line: &str| [TET_OBJ, line].concat().into_bytes();
    let faces: Vec<i32> = TET_FACES
        .iter()
        .flatten()
        .map(|&i| i32::try_from(i).unwrap())
        .collect();
    let files: [(&str, Vec<u8>); 17] = [
        ("tet.stl", stl(&TET_FACES.map(|f| f.map(|i| TET[i])))),
        ("tri.stl", stl(&[[TET[0], TET[1], TET[2]]])),
        ("zero.stl", stl(&[])),
        ("tet.obj", TET_OBJ.as_bytes().to_vec()),
        ("tet_vn.obj", with("vn 0 0 1\n")),
        ("tet_vt.obj", with("vt 0 0\n")),
        ("tet_vn_bare.obj", with("vn\n")),
        ("tet_vn_space.obj", with("vn \n")),
        ("tet_vnx.obj", with("vnx 1 2 3\n")),
        ("tet_vn_indent.obj", with("  vn 0 0 1\n")),
        ("tri.obj", b"v 0 0 0\nv 1 0 0\nv 0 2 0\nf 1 3 2\n".to_vec()),
        (
            "lines.obj",
            b"v 0 0 0\nv 1 0 0\nv 0 2 0\nv 0 0 3\nl 1 2\n".to_vec(),
        ),
        ("empty.obj", Vec::new()),
        ("tet.msh", msh([4, 0, 0, 4], &[], &faces)),
        ("nofaces.msh", msh([4, 0, 0, 0], &[], &[])),
        (
            "tet_n.msh",
            msh([4, 4, 0, 4], &[0.0, 0.0, 1.0].repeat(4), &faces),
        ),
        ("tet_t.msh", msh([4, 0, 4, 4], &[0.0; 8], &faces)),
    ];
    for (name, bytes) in files {
        std::fs::write(dir.join(name), bytes).unwrap();
    }
}

/// One free body holding a mesh geom of `m`: an empty world below.
const BODY: &str = r#"<body name="b" pos="0 0 1"><freejoint/><geom type="mesh" mesh="m"/></body>"#;

/// A document of the `<compiler>` attributes `compiler` beside the files'
/// directory, the assets `assets` and the world body's contents `world`
/// ([`BODY`] where empty).
fn document(dir: &Path, compiler: &str, assets: &str, world: &str) -> String {
    let world = if world.is_empty() { BODY } else { world };
    format!(
        r#"<mujoco><compiler meshdir="{}" {compiler}/><asset>{assets}</asset><worldbody>{world}</worldbody></mujoco>"#,
        dir.display()
    )
}

/// `(what, compiler attributes, assets, world, sim-mjcf's error)`, each
/// refused by MuJoCo 3.5.0 for that reason.
const REFUSED: &[(&str, &str, &str, &str, &str)] = &[
    (
        "ten vertex numbers",
        "",
        r#"<mesh name="m" vertex="0 0 0 1 0 0 0 2 0 0" face="0 1 2"/>"#,
        "",
        "vertex data must be a multiple of 3 (mesh 'm')",
    ),
    (
        "thirteen vertex numbers before a missing face vertex",
        "",
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3  9" face="0 2 1  0 1 3  0 3 2  1 2 9"/>"#,
        "",
        "vertex data must be a multiple of 3 (mesh 'm')",
    ),
    (
        "thirteen face numbers before a normal of two",
        "",
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3  0" normal="0 0"/>"#,
        "",
        "face data must be a multiple of 3 (mesh 'm')",
    ),
    (
        "thirteen face numbers before an odd texcoord",
        "",
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3  0" texcoord="0"/>"#,
        "",
        "face data must be a multiple of 3 (mesh 'm')",
    ),
    (
        "seven vertex numbers",
        "",
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0" face="0 1 2"/>"#,
        "",
        "vertex data must be a multiple of 3 (mesh 'm')",
    ),
    (
        "an unnamed mesh, then the next mesh's ten vertex numbers",
        "",
        r#"<mesh vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/><mesh name="m" vertex="0 0 0 1 0 0 0 2 0 0" face="0 1 2"/>"#,
        "",
        "vertex data must be a multiple of 3 (mesh 'm')",
    ),
    (
        "a mesh of 3 vertices, then the next mesh's face of four numbers",
        "",
        r#"<mesh name="m3" vertex="0 0 0  1 0 0  0 2 0"/><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1 3"/>"#,
        "",
        "face data must be a multiple of 3 (mesh 'm')",
    ),
    (
        "ten vertex numbers before a height field's size of 0",
        "",
        r#"<hfield name="h" nrow="2" ncol="2" elevation="0 0 0 0" size="1 1 0 0.1"/><mesh name="m" vertex="0 0 0 1 0 0 0 2 0 0" face="0 1 2"/>"#,
        "",
        "vertex data must be a multiple of 3 (mesh 'm')",
    ),
    (
        "ten vertex numbers before a geom's missing mesh",
        "",
        r#"<mesh name="m" vertex="0 0 0 1 0 0 0 2 0 0" face="0 1 2"/>"#,
        r#"<geom type="mesh" mesh="nope"/>"#,
        "vertex data must be a multiple of 3 (mesh 'm')",
    ),
    (
        "an unnamed file mesh's ten vertex numbers",
        "",
        r#"<mesh file="tet.obj" vertex="0 0 0 1 0 0 0 2 0 0"/>"#,
        r#"<geom type="mesh" mesh="tet"/>"#,
        "vertex data must be a multiple of 3 (mesh #0)",
    ),
    (
        "an unknown class before ten vertex numbers",
        "",
        r#"<mesh name="m" vertex="0 0 0 1 0 0 0 2 0 0" face="0 1 2"/>"#,
        r#"<geom class="nope" size="0.1"/><body name="b" pos="0 0 1"><freejoint/><geom type="mesh" mesh="m"/></body>"#,
        "unknown default class name 'nope' (geom in body 'world')",
    ),
    (
        "an unnamed mesh's unknown class",
        "",
        r#"<mesh class="nope" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/>"#,
        "",
        "unknown default class name 'nope' (mesh #0)",
    ),
    (
        "3 vertices and a face",
        "",
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0" face="0 1 2"/>"#,
        "",
        "mesh 'm': at least 4 vertices required",
    ),
    (
        "3 vertices and no face",
        "",
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0"/>"#,
        "",
        "mesh 'm': at least 4 vertices required",
    ),
    (
        "no vertices and no file",
        "",
        r#"<mesh name="m"/>"#,
        "",
        "mesh 'm': at least 4 vertices required",
    ),
    (
        "3 vertices and a missing face vertex",
        "",
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0" face="0 1 5"/>"#,
        "",
        "mesh 'm': at least 4 vertices required",
    ),
    (
        "a normal of four numbers",
        "",
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3" normal="0 0 1 0"/>"#,
        "",
        "mesh 'm': normal data must be a multiple of 3",
    ),
    (
        "a normal of four numbers before an odd texcoord",
        "",
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3" normal="0 0 1 0" texcoord="0"/>"#,
        "",
        "mesh 'm': normal data must be a multiple of 3",
    ),
    (
        "an odd texcoord",
        "",
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3" texcoord="0 0 1"/>"#,
        "",
        "mesh 'm': texcoord must be a multiple of 2",
    ),
    (
        "a texcoord of two pairs for four vertices",
        "",
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3" texcoord="0 0 1 1"/>"#,
        "",
        "mesh 'm': texcoord must be 2*nv if face texcoord indices are not provided in an OBJ file",
    ),
    (
        "a texcoord of two pairs, its content type an STL's",
        "",
        r#"<mesh name="m" content_type="model/stl" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3" texcoord="0 0 1 1"/>"#,
        "",
        "mesh 'm': texcoord must be 2*nv if face texcoord indices are not provided in an OBJ file",
    ),
    (
        "an odd texcoord under discardvisual",
        r#"discardvisual="true""#,
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3" texcoord="0 0 1"/>"#,
        "",
        "mesh 'm': texcoord must be a multiple of 2",
    ),
    (
        "a texcoord of two pairs under discardvisual",
        r#"discardvisual="true""#,
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3" texcoord="0 0 1 1"/>"#,
        "",
        "mesh 'm': texcoord must be 2*nv if face texcoord indices are not provided in an OBJ file",
    ),
    (
        "a texcoord of two pairs before a missing face vertex",
        "",
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 9" texcoord="0 0 1 1"/>"#,
        "",
        "mesh 'm': texcoord must be 2*nv if face texcoord indices are not provided in an OBJ file",
    ),
    (
        "a missing vertex in the last face",
        "",
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 9"/>"#,
        "",
        "mesh 'm': in face 3, vertex index 9 does not exist",
    ),
    (
        "a missing vertex in the first face, before the last's",
        "",
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 9  0 1 3  0 3 2  1 2 7"/>"#,
        "",
        "mesh 'm': in face 0, vertex index 9 does not exist",
    ),
    (
        "a negative vertex index",
        "",
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 -1"/>"#,
        "",
        "mesh 'm': in face 3, vertex index -1 does not exist",
    ),
    (
        "a negative vertex index before a geom's condim",
        "",
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 -1"/>"#,
        r#"<body name="b" pos="0 0 1"><freejoint/><geom type="mesh" mesh="m" condim="2"/></body>"#,
        "mesh 'm': in face 3, vertex index -1 does not exist",
    ),
    (
        "a negative vertex index in a mesh no geom names",
        "",
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 -1"/><mesh name="n" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/>"#,
        r#"<body name="b" pos="0 0 1"><freejoint/><geom type="mesh" mesh="n"/></body>"#,
        "mesh 'm': in face 3, vertex index -1 does not exist",
    ),
    (
        "3 vertices in a mesh no geom names",
        "",
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0" face="0 1 2"/><mesh name="n" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/>"#,
        r#"<body name="b" pos="0 0 1"><freejoint/><geom type="mesh" mesh="n"/></body>"#,
        "mesh 'm': at least 4 vertices required",
    ),
    (
        "3 vertices before a geom's condim",
        "",
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0" face="0 1 2"/>"#,
        r#"<body name="b" pos="0 0 1"><freejoint/><geom type="mesh" mesh="m" condim="2"/></body>"#,
        "mesh 'm': at least 4 vertices required",
    ),
    (
        "a missing face vertex, then a mesh of 3 vertices, in mesh order",
        r#"usethread="false""#,
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 9"/><mesh name="m3" vertex="0 0 0  1 0 0  0 2 0" face="0 1 2"/>"#,
        "",
        "mesh 'm': in face 3, vertex index 9 does not exist",
    ),
    (
        "a mesh of 3 vertices, then a missing face vertex, in mesh order",
        r#"usethread="false""#,
        r#"<mesh name="m0" vertex="0 0 0  1 0 0  0 2 0" face="0 1 2"/><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 9"/>"#,
        "",
        "mesh 'm0': at least 4 vertices required",
    ),
    (
        "a negative vertex index beside a geom's missing mesh",
        "",
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 -1"/>"#,
        r#"<geom type="mesh" mesh="nope"/>"#,
        "unknown mesh 'nope' (geom #0 in body 'world')",
    ),
    (
        "an STL and a face",
        "",
        r#"<mesh name="m" file="tet.stl" face="0 2 1  0 1 3  0 3 2  1 2 3"/>"#,
        "",
        "mesh 'm': repeated face specification",
    ),
    (
        "an STL, vertices and a face",
        "",
        r#"<mesh name="m" file="tet.stl" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/>"#,
        "",
        "mesh 'm': repeated face specification",
    ),
    (
        "an STL and vertices",
        "",
        r#"<mesh name="m" file="tet.stl" vertex="0 0 0  1 0 0  0 2 0  0 0 3"/>"#,
        "",
        "mesh 'm': repeated vertex specification",
    ),
    (
        "an OBJ and a face",
        "",
        r#"<mesh name="m" file="tet.obj" face="0 2 1  0 1 3  0 3 2  1 2 3"/>"#,
        "",
        "mesh 'm': repeated face specification",
    ),
    (
        "an OBJ and vertices",
        "",
        r#"<mesh name="m" file="tet.obj" vertex="0 0 0  1 0 0  0 2 0  0 0 3"/>"#,
        "",
        "mesh 'm': repeated vertex specification",
    ),
    (
        "an MSH and a face",
        "",
        r#"<mesh name="m" file="tet.msh" face="0 2 1  0 1 3  0 3 2  1 2 3"/>"#,
        "",
        "mesh 'm': repeated face specification",
    ),
    (
        "an MSH of no faces and vertices",
        "",
        r#"<mesh name="m" file="nofaces.msh" vertex="0 0 0  1 0 0  0 2 0  0 0 3"/>"#,
        "",
        "mesh 'm': repeated vertex specification",
    ),
    (
        "an OBJ with a normal and a normal",
        "",
        r#"<mesh name="m" file="tet_vn.obj" normal="0 0 1"/>"#,
        "",
        "mesh 'm': repeated normal specification",
    ),
    (
        "an OBJ with a texcoord and a texcoord",
        "",
        r#"<mesh name="m" file="tet_vt.obj" texcoord="0 0"/>"#,
        "",
        "mesh 'm': repeated texcoord specification",
    ),
    (
        "an STL and a face, its mesh named by no geom",
        "",
        r#"<mesh name="m" file="tet.stl" face="0 2 1  0 1 3  0 3 2  1 2 3"/><mesh name="n" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/>"#,
        r#"<body name="b" pos="0 0 1"><freejoint/><geom type="mesh" mesh="n"/></body>"#,
        "mesh 'm': repeated face specification",
    ),
    (
        "an MSH of no faces and a missing face vertex",
        "",
        r#"<mesh name="m" file="nofaces.msh" face="0 2 9  0 1 3  0 3 2  1 2 3"/>"#,
        "",
        "mesh 'm': in face 0, vertex index 9 does not exist",
    ),
    (
        "an MSH with normals and a normal",
        "",
        r#"<mesh name="m" file="tet_n.msh" normal="0 0 1  0 0 1  0 0 1  0 0 1"/>"#,
        "",
        "mesh 'm': repeated normal specification",
    ),
    (
        "an MSH with faces and a normal",
        "",
        r#"<mesh name="m" file="tet.msh" normal="0 0 1"/>"#,
        "",
        "mesh 'm': repeated facenormal specification",
    ),
    (
        "an MSH with faces and a normal of four numbers",
        "",
        r#"<mesh name="m" file="tet.msh" normal="0 0 1 0"/>"#,
        "",
        "mesh 'm': repeated facenormal specification",
    ),
    (
        "an MSH with texcoords and faces, and a normal",
        "",
        r#"<mesh name="m" file="tet_t.msh" normal="0 0 1"/>"#,
        "",
        "mesh 'm': repeated facenormal specification",
    ),
    (
        "an MSH with faces, a vertex and a normal",
        "",
        r#"<mesh name="m" file="tet.msh" vertex="0 0 0  1 0 0  0 2 0  0 0 3" normal="0 0 1"/>"#,
        "",
        "mesh 'm': repeated vertex specification",
    ),
    (
        "an MSH with faces, a face and a normal",
        "",
        r#"<mesh name="m" file="tet.msh" face="0 2 1  0 1 3  0 3 2  1 2 3" normal="0 0 1"/>"#,
        "",
        "mesh 'm': repeated face specification",
    ),
    (
        "an MSH with texcoords and a texcoord",
        "",
        r#"<mesh name="m" file="tet_t.msh" texcoord="0 0  0 0  0 0  0 0"/>"#,
        "",
        "mesh 'm': repeated texcoord specification",
    ),
    (
        "an STL triangle and a face",
        "",
        r#"<mesh name="m" file="tri.stl" face="0 2 1  0 1 3  0 3 2  1 2 3"/>"#,
        "",
        "mesh 'm': repeated face specification",
    ),
    (
        "an OBJ with a normal, a normal and a face",
        "",
        r#"<mesh name="m" file="tet_vn.obj" normal="0 0 1" face="0 2 1  0 1 3  0 3 2  1 2 3"/>"#,
        "",
        "mesh 'm': repeated normal specification",
    ),
    (
        "an STL and a texcoord of one pair per vertex as read",
        "",
        r#"<mesh name="m" file="tet.stl" texcoord="0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0"/>"#,
        "",
        "mesh 'm': texcoord must be 2*nv if face texcoord indices are not provided in an OBJ file",
    ),
    (
        "an OBJ and a normal of four numbers",
        "",
        r#"<mesh name="m" file="tet.obj" normal="0 0 1 0"/>"#,
        "",
        "mesh 'm': normal data must be a multiple of 3",
    ),
    (
        "an MSH with normals and a texcoord of two pairs",
        "",
        r#"<mesh name="m" file="tet_n.msh" texcoord="0 0 1 1"/>"#,
        "",
        "mesh 'm': texcoord must be 2*nv if face texcoord indices are not provided in an OBJ file",
    ),
    (
        "an STL, vertices, a face, a normal and a texcoord",
        "",
        r#"<mesh name="m" file="tet.stl" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3" normal="0 0 1" texcoord="0 0"/>"#,
        "",
        "mesh 'm': repeated face specification",
    ),
    (
        "an OBJ with a normal, a normal and an odd texcoord",
        "",
        r#"<mesh name="m" file="tet_vn.obj" normal="0 0 1" texcoord="0"/>"#,
        "",
        "mesh 'm': repeated normal specification",
    ),
    (
        "an MSH with normals and a normal of two numbers",
        "",
        r#"<mesh name="m" file="tet_n.msh" normal="0 0"/>"#,
        "",
        "mesh 'm': repeated normal specification",
    ),
    (
        "an STL triangle",
        "",
        r#"<mesh name="m" file="tri.stl"/>"#,
        "",
        "mesh 'm': at least 4 vertices required",
    ),
    (
        "an OBJ of 3 vertices",
        "",
        r#"<mesh name="m" file="tri.obj"/>"#,
        "",
        "mesh 'm': at least 4 vertices required",
    ),
    (
        "an empty OBJ",
        "",
        r#"<mesh name="m" file="empty.obj"/>"#,
        "",
        "mesh 'm': at least 4 vertices required",
    ),
    (
        "an STL of no faces, vertices and a face",
        "",
        r#"<mesh name="m" file="zero.stl" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/>"#,
        "",
        "mesh 'm': number of faces should be between 1 and 200000 in STL file 'zero.stl'; perhaps this is an ASCII file?",
    ),
    (
        r#"an OBJ line "vn " and a normal"#,
        "",
        r#"<mesh name="m" file="tet_vn_space.obj" normal="0 0 1"/>"#,
        "",
        "mesh 'm': repeated normal specification",
    ),
    (
        "an indented OBJ normal and a normal",
        "",
        r#"<mesh name="m" file="tet_vn_indent.obj" normal="0 0 1"/>"#,
        "",
        "mesh 'm': repeated normal specification",
    ),
];

/// `(what, compiler attributes, assets, world)`, each loaded by MuJoCo
/// 3.5.0.
const LOADS: &[(&str, &str, &str, &str)] = &[
    (
        "an MSH of no faces and a face",
        "",
        r#"<mesh name="m" file="nofaces.msh" face="0 2 1  0 1 3  0 3 2  1 2 3"/>"#,
        "",
    ),
    (
        "an MSH of no faces, a face and a normal",
        "",
        r#"<mesh name="m" file="nofaces.msh" face="0 2 1  0 1 3  0 3 2  1 2 3" normal="0 0 1"/>"#,
        "",
    ),
    (
        "an OBJ of lines only and a face",
        "",
        r#"<mesh name="m" file="lines.obj" face="0 2 1  0 1 3  0 3 2  1 2 3"/>"#,
        "",
    ),
    (
        "an empty OBJ, vertices and a face",
        "",
        r#"<mesh name="m" file="empty.obj" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/>"#,
        "",
    ),
    (
        "an OBJ with a texcoord and a normal",
        "",
        r#"<mesh name="m" file="tet_vt.obj" normal="0 0 1"/>"#,
        "",
    ),
    (
        "an OBJ with a normal and a texcoord",
        "",
        r#"<mesh name="m" file="tet_vn.obj" texcoord="0 0"/>"#,
        "",
    ),
    (
        "an MSH with texcoords",
        "",
        r#"<mesh name="m" file="tet_t.msh"/>"#,
        "",
    ),
    (
        "an MSH with normals",
        "",
        r#"<mesh name="m" file="tet_n.msh"/>"#,
        "",
    ),
    (
        "an OBJ and a texcoord of two pairs",
        "",
        r#"<mesh name="m" file="tet.obj" texcoord="0 0 1 1"/>"#,
        "",
    ),
    (
        "an STL and a normal",
        "",
        r#"<mesh name="m" file="tet.stl" normal="0 0 1"/>"#,
        "",
    ),
    (
        "an OBJ and a normal",
        "",
        r#"<mesh name="m" file="tet.obj" normal="0 0 1"/>"#,
        "",
    ),
    (
        "an STL and a texcoord of one pair per vertex as merged",
        "",
        r#"<mesh name="m" file="tet.stl" texcoord="0 0  1 0  0 1  1 1"/>"#,
        "",
    ),
    (
        "a texcoord of one pair per vertex",
        "",
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3" texcoord="0 0  1 0  0 1  1 1"/>"#,
        "",
    ),
    (
        "a normal of any count of threes",
        "",
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3" normal="0 0 1  0 1 0"/>"#,
        "",
    ),
    (
        "a texcoord of two pairs, its content type an OBJ's",
        "",
        r#"<mesh name="m" content_type="model/obj" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3" texcoord="0 0 1 1"/>"#,
        "",
    ),
    (
        "an empty normal",
        "",
        r#"<mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3" normal=""/>"#,
        "",
    ),
    (
        r#"an OBJ line "vn" and a normal"#,
        "",
        r#"<mesh name="m" file="tet_vn_bare.obj" normal="0 0 1"/>"#,
        "",
    ),
    (
        r#"an OBJ line "vnx" and a normal"#,
        "",
        r#"<mesh name="m" file="tet_vnx.obj" normal="0 0 1"/>"#,
        "",
    ),
];

#[test]
fn a_meshs_data_is_refused_where_and_as_mujoco_refuses_it() {
    let dir = tempfile::tempdir().unwrap();
    write_files(dir.path());
    let mut missed = Vec::new();
    for &(what, compiler, assets, world, want) in REFUSED {
        match load_model(&document(dir.path(), compiler, assets, world)) {
            Err(e) if e.to_string() == want => {}
            Err(e) => missed.push(format!("{what}: {e}")),
            Ok(_) => missed.push(format!("{what}: loads")),
        }
    }
    assert!(
        missed.is_empty(),
        "{} of {} not refused as MuJoCo refuses them:\n  {}",
        missed.len(),
        REFUSED.len(),
        missed.join("\n  ")
    );
}

/// Each loads with the tetrahedron, as MuJoCo 3.5.0 loads it: four
/// vertices, four faces, and the mass MuJoCo gives its body, computed by
/// `+`, `-`, `*` and `/` alone, whose results IEEE 754 fixes to the bit.
#[test]
fn a_meshs_data_mujoco_loads_loads() {
    let dir = tempfile::tempdir().unwrap();
    write_files(dir.path());
    let mut missed = Vec::new();
    for &(what, compiler, assets, world) in LOADS {
        match load_model(&document(dir.path(), compiler, assets, world)) {
            Ok(model) => {
                let mesh = &model.mesh_data[0];
                let got = (
                    mesh.vertex_count(),
                    mesh.triangles().len(),
                    model.body_mass[1].to_bits(),
                );
                if got != (4, 4, 999.999_999_999_999_9_f64.to_bits()) {
                    missed.push(format!("{what}: {got:?}"));
                }
            }
            Err(e) => missed.push(format!("{what}: {e}")),
        }
    }
    assert!(missed.is_empty(), "{}", missed.join("\n"));
}

/// A mesh's `normal` and `texcoord` are read as MuJoCo 3.5.0 reads them, as
/// floats: one past a float's range, either way, is refused, where MuJoCo
/// refuses it ("number is too large"); one that is not a number is refused
/// ("bad format"). A `NaN` MuJoCo loads is refused, as every `NaN` written in a
/// document is (registry `D-NONFINITE`).
#[test]
fn a_meshs_normal_and_texcoord_are_read_as_floats() {
    let tet = r#"vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3""#;
    let refused = [
        (
            r#"normal="0 0 1e40""#,
            r#"number "1e40" is out of range in attribute 'normal' on <mesh>"#,
        ),
        (
            r#"normal="0 0 1e-40""#,
            r#"number "1e-40" is out of range in attribute 'normal' on <mesh>"#,
        ),
        (
            r#"texcoord="0 1e40""#,
            r#"number "1e40" is out of range in attribute 'texcoord' on <mesh>"#,
        ),
        (
            r#"texcoord="1e-40 0  0 0  0 0  0 0""#,
            r#"number "1e-40" is out of range in attribute 'texcoord' on <mesh>"#,
        ),
        (r#"normal="0 0 x""#, "attribute 'normal' on <mesh>"),
        (r#"normal="0 0 nan""#, "normal on <mesh> is not finite"),
    ];
    for (attr, want) in refused {
        let xml = format!(
            r#"<mujoco><asset><mesh name="m" {tet} {attr}/></asset><worldbody>{BODY}</worldbody></mujoco>"#
        );
        let err = load_model(&xml).unwrap_err().to_string();
        assert!(err.contains(want), "{attr}: {err}");
    }
    for attr in [
        r#"normal="0 0 1e-30""#,
        r#"texcoord="0e-50 0  0 0  0 0  0 0""#,
    ] {
        let xml = format!(
            r#"<mujoco><asset><mesh name="m" {tet} {attr}/></asset><worldbody>{BODY}</worldbody></mujoco>"#
        );
        load_model(&xml).unwrap_or_else(|e| panic!("{attr}: {e}"));
    }
}

/// A vertex that is not finite, set in code where no document could write
/// it, is refused as MuJoCo 3.5.0 refuses one (`ProcessVertices`,
/// `user_mesh.cc:557-561`), naming the vertex, after the face count.
#[test]
fn a_vertex_set_in_code_not_finite_is_refused_as_mujoco_refuses_it() {
    let xml = format!(
        r#"<mujoco><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody>{BODY}</worldbody></mujoco>"#
    );
    let mut mjcf = parse_mjcf_str(&xml).unwrap();
    mjcf.meshes[0].vertex.as_mut().unwrap()[4] = f64::NAN;
    let err = model_from_mjcf(&mjcf, None).unwrap_err().to_string();
    assert_eq!(err, "vertex coordinate 1 is not finite (mesh 'm')");
    mjcf.meshes[0].face.as_mut().unwrap().pop();
    let err = model_from_mjcf(&mjcf, None).unwrap_err().to_string();
    assert_eq!(err, "face data must be a multiple of 3 (mesh 'm')");
}
