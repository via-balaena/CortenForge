//! `mjc_ccd` against MuJoCo's, bit for bit: every case of the golden
//! `ccd.json` (`sim/L0/tests/scripts/gen_ccd_reference.py`, from the
//! unfused MuJoCo 3.5.0 oracle; it describes the cases).

// Test code: a malformed golden is a failed test.
#![allow(clippy::expect_used, clippy::panic)]

use std::collections::HashMap;

use nalgebra::Point3;
use serde_json::Value;

use super::{
    CcdConfig, CcdMesh, CcdObj, CcdStatus, HullGraph, POLYTOPE, Polytope, RawContact, STALE_S3D,
    mjc_ccd, mjc_convex_hfield, mjc_plane_convex,
};
use crate::heightfield::HeightFieldData;
use crate::mesh::TriangleMeshData;
use crate::mesh_polygons::{MeshPolygons, make_polygons};
use crate::types::GeomType;

/// The golden's groups that hold no `mjc_ccd` case.
const NOT_CCD_CASES: [&str; 8] = [
    "oracle",
    "meshes",
    "merge",
    "graph_meshes",
    "plane_meshes",
    "plane_convex",
    "hfield_fields",
    "hfield_convex",
];

fn golden() -> Value {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../tests/assets/golden/ccd/ccd.json"
    );
    let text = std::fs::read_to_string(path).expect("the ccd golden reads");
    serde_json::from_str(&text).expect("the ccd golden parses")
}

/// A number of the golden; the generator writes a NaN as the string "nan".
fn number(v: &Value) -> f64 {
    match v.as_str() {
        Some("nan") => f64::NAN,
        _ => v.as_f64().expect("a number"),
    }
}

fn count(v: &Value) -> usize {
    usize::try_from(v.as_u64().expect("a count")).expect("a count")
}

fn floats<const N: usize>(v: &Value) -> [f64; N] {
    let a = v.as_array().expect("an array");
    assert_eq!(a.len(), N, "{v}");
    std::array::from_fn(|i| number(&a[i]))
}

fn counts(v: &Value) -> Vec<usize> {
    v.as_array().expect("an array").iter().map(count).collect()
}

/// A mesh of the golden (`meshes`, `graph_meshes`): our mesh data as the
/// loader builds it ([`loaded`]) and MuJoCo's polygons.
struct Mesh {
    data: TriangleMeshData,
    mujocos: MeshPolygons,
}

/// Whose polygons a mesh takes.
#[derive(Clone, Copy)]
enum Polygons {
    Mujocos,
    Ours,
}

/// A golden mesh's data as sim-mjcf's loader builds a mesh written with
/// vertices and no faces: its vertices as authored, read in single
/// precision; qhull's hull of them, whose faces it takes, and the polygons
/// merged from those ([`TriangleMeshData::compute_hull`]); then MuJoCo's
/// compiled vertices, which keep the hull and the polygons' paths and
/// remake their normals ([`TriangleMeshData::replace_vertices`]).
fn loaded(m: &Value) -> TriangleMeshData {
    #[allow(clippy::cast_possible_truncation)] // the loader reads them as floats
    let authored = points(&m["authored"])
        .into_iter()
        .map(|p| p.map(|c| f64::from(c as f32)))
        .collect();
    let mut data = TriangleMeshData::new(authored, Vec::new());
    data.compute_hull(None).expect("qhull makes the hull");
    data.replace_vertices(points(&m["vert"]));
    data
}

/// A golden mesh's hull graph as MuJoCo holds it: each hull vertex's index
/// among the mesh's vertices (`vert_globalid`) and its neighbours among the
/// hull's vertices, in MuJoCo's order.
fn mujocos_graph(m: &Value) -> (Vec<usize>, Vec<Vec<u32>>) {
    let neighbours = m["graph"]["neighbours"]
        .as_array()
        .expect("each hull vertex's neighbours")
        .iter()
        .map(|n| {
            counts(n)
                .into_iter()
                .map(|i| u32::try_from(i).expect("a hull vertex"))
                .collect()
        })
        .collect();
    (counts(&m["graph"]["vertex"]), neighbours)
}

fn points(v: &Value) -> Vec<Point3<f64>> {
    let flat: Vec<f64> = v.as_array().expect("vertices").iter().map(number).collect();
    flat.chunks_exact(3)
        .map(|c| Point3::new(c[0], c[1], c[2]))
        .collect()
}

/// A mesh's polygons as MuJoCo merged them.
fn mujoco_polygons(m: &Value) -> MeshPolygons {
    MeshPolygons {
        paths: m["polygons"]
            .as_array()
            .expect("polygons")
            .iter()
            .map(counts)
            .collect(),
        normals: m["polynormals"]
            .as_array()
            .expect("polygon normals")
            .iter()
            .map(floats)
            .collect(),
        map: m["polymap"]
            .as_array()
            .expect("a polygon map")
            .iter()
            .map(counts)
            .collect(),
    }
}

/// The golden's `meshes` and `graph_meshes`, each as the loader builds it.
fn meshes(doc: &Value) -> HashMap<String, Mesh> {
    let mut out = HashMap::new();
    for group in ["meshes", "graph_meshes"] {
        for (name, m) in doc[group].as_object().expect("a group of meshes") {
            let data = loaded(m);
            // Our selection of the search agrees with MuJoCo's on these meshes.
            assert_eq!(
                m["hillclimb"].as_bool(),
                Some(data.vertex_count() >= 10),
                "mesh {name}'s search"
            );
            out.insert(
                name.clone(),
                Mesh {
                    data,
                    mujocos: mujoco_polygons(m),
                },
            );
        }
    }
    out
}

fn geom_type(name: &str) -> GeomType {
    match name {
        "sphere" => GeomType::Sphere,
        "capsule" => GeomType::Capsule,
        "ellipsoid" => GeomType::Ellipsoid,
        "cylinder" => GeomType::Cylinder,
        "box" => GeomType::Box,
        "mesh" => GeomType::Mesh,
        other => panic!("geom type {other}"),
    }
}

/// The object of a case, a mesh on its own hull graph with the polygons
/// `polygons` asks for.
fn object<'a>(v: &Value, meshes: &'a HashMap<String, Mesh>, polygons: Polygons) -> CcdObj<'a> {
    let mesh = v["mesh"].as_str().map(|name| {
        let m = &meshes[name];
        let hull = m.data.hull().expect("the mesh has a hull");
        CcdMesh {
            verts: m.data.vertices(),
            hull: Some(HullGraph {
                vertex: &hull.vertex,
                neighbours: &hull.neighbours,
            }),
            polygons: match polygons {
                Polygons::Mujocos => &m.mujocos,
                Polygons::Ours => m.data.polygons(),
            },
        }
    });
    CcdObj::new(
        geom_type(v["type"].as_str().expect("a type")),
        floats(&v["pos"]),
        floats(&v["mat"]),
        floats(&v["size"]),
        number(&v["margin"]),
        mesh,
    )
    .expect("a geom mjc_ccd takes")
}

fn config(v: &Value) -> CcdConfig {
    CcdConfig {
        max_iterations: count(&v["max_iterations"]),
        tolerance: number(&v["tolerance"]),
        max_contacts: count(&v["max_contacts"]),
        dist_cutoff: number(&v["dist_cutoff"]),
    }
}

/// The first quantity in which the status is not MuJoCo's to the bit.
fn first_difference(st: &CcdStatus, want: &Value) -> Option<String> {
    let same = |a: f64, b: f64| a.to_bits() == b.to_bits() || (a.is_nan() && b.is_nan());
    let dist = number(&want["dist"]);
    if !same(st.dist, dist) {
        return Some(format!("dist {:e}, MuJoCo {dist:e}", st.dist));
    }
    let counts = [
        ("nx", st.nx),
        ("gjk_iterations", st.gjk_iterations),
        ("epa_iterations", st.epa_iterations),
        ("nsimplex", st.nsimplex),
    ];
    for (name, ours) in counts {
        let theirs = count(&want[name]);
        if ours != theirs {
            return Some(format!("{name} {ours}, MuJoCo {theirs}"));
        }
    }
    let status = want["epa_status"].as_i64().expect("an EPA status");
    if st.epa_status as i64 != status {
        return Some(format!("EPA status {:?}, MuJoCo {status}", st.epa_status));
    }
    for (key, ours) in [("x1", &st.x1), ("x2", &st.x2)] {
        for (i, w) in want[key].as_array().expect("witnesses").iter().enumerate() {
            let w: [f64; 3] = floats(w);
            if !w.iter().zip(ours[i]).all(|(a, b)| same(*a, b)) {
                return Some(format!("{key}[{i}] {:?}, MuJoCo {w:?}", ours[i]));
            }
        }
    }
    None
}

/// `mjc_ccd` on a case, with `S3D`'s fourth weight as MuJoCo keeps it or as
/// ours resets it, and a mesh's polygons MuJoCo's or ours.
fn run(case: &Value, meshes: &HashMap<String, Mesh>, stale: bool, polygons: Polygons) -> CcdStatus {
    let mut o1 = object(&case["obj"][0], meshes, polygons);
    let mut o2 = object(&case["obj"][1], meshes, polygons);
    STALE_S3D.with(|s| s.set(stale));
    let st = mjc_ccd(&config(&case["config"]), &mut o1, &mut o2);
    STALE_S3D.with(|s| s.set(false));
    st
}

/// Every case of the golden: the distance, the witness points, the GJK and
/// EPA iteration counts, the EPA status and the simplex size, to the bit,
/// each mesh as the loader builds it (its hull graph MuJoCo's,
/// [`each_meshs_graph_is_mujocos`]) taking MuJoCo's polygons, whose normals
/// MuJoCo made from its vertices before rounding them (the golden holds
/// only the rounded vertices). With `S3D`'s fourth weight kept as MuJoCo
/// keeps it every case is MuJoCo's; with ours reset, a case differs only
/// where keeping it gives MuJoCo's (registry `D-GJK-S3D`), and the golden
/// holds such a case.
#[test]
fn mjc_ccd_matches_mujoco_bitwise() {
    let doc = golden();
    let meshes = meshes(&doc);
    let mut compared = 0;
    let mut failures = Vec::new();
    let mut stale_weight = Vec::new();
    let groups = doc.as_object().expect("the golden's groups");
    for (group, cases) in groups
        .iter()
        .filter(|(k, _)| !NOT_CCD_CASES.contains(&k.as_str()))
    {
        let cases = cases.as_object().expect("a group of cases");
        assert!(!cases.is_empty(), "no {group} cases");
        for (name, case) in cases {
            let want = &case["out"];
            let mujocos = first_difference(&run(case, &meshes, true, Polygons::Mujocos), want);
            let ours = first_difference(&run(case, &meshes, false, Polygons::Mujocos), want);
            match (ours, mujocos) {
                (None, None) => {}
                (Some(_), None) => stale_weight.push(format!("{group}/{name}")),
                (_, Some(d)) => failures.push(format!("{group}/{name}: {d}")),
            }
            compared += 1;
        }
    }
    assert!(
        failures.is_empty(),
        "{} of {compared} cases differ from MuJoCo's:\n{}",
        failures.len(),
        failures.join("\n")
    );
    assert!(
        !stale_weight.is_empty(),
        "no case shows S3D's stale weight: the reset is not checked"
    );
}

/// `reset` sets EPA's face limit from each call's `ccd_iterations`, as
/// MuJoCo sizes its storage (`engine_collision_gjk.c:2304-2333`): 6 faces an
/// iteration, for at least `mjMAX_EPA_ITERATIONS` (170), on a thread's
/// storage after any earlier call.
#[test]
fn reset_takes_each_call_s_face_limit() {
    let mut pt = Polytope::EMPTY;
    for (iterations, faces) in [
        (35, 1020),
        (170, 1020),
        (171, 1026),
        (300, 1800),
        (35, 1020),
    ] {
        pt.reset(iterations);
        assert_eq!(pt.max_faces, faces, "ccd_iterations {iterations}");
    }
}

/// Each call empties the thread's polytope before EPA fills it. With two
/// edges left on its horizon, as an EPA that stops on finding fewer than
/// three can leave them, every case of the golden in which EPA iterates is
/// still MuJoCo's, bit for bit.
#[test]
fn each_call_starts_from_an_empty_polytope() {
    let doc = golden();
    let meshes = meshes(&doc);
    let mut compared = 0;
    let groups = doc.as_object().expect("the golden's groups");
    for (group, cases) in groups
        .iter()
        .filter(|(k, _)| !NOT_CCD_CASES.contains(&k.as_str()))
    {
        for (name, case) in cases.as_object().expect("a group of cases") {
            let want = &case["out"];
            if count(&want["epa_iterations"]) == 0 {
                continue;
            }
            let mut left = POLYTOPE.replace(Polytope::EMPTY);
            left.horizon.extend([(0, 0), (1, 1)]);
            POLYTOPE.set(left);
            let st = run(case, &meshes, true, Polygons::Mujocos);
            assert_eq!(first_difference(&st, want), None, "{group}/{name}");
            compared += 1;
        }
    }
    assert!(compared > 0, "no case in which EPA iterates");
}

/// Registry `D-GJK-S3D` in a contact: the golden's `s3d` cases, a sphere
/// against an ellipsoid and a capsule against an ellipsoid or a cylinder,
/// overlapping, as `mjc_Convex`
/// calls `mjc_ccd`. With `S3D`'s fourth weight as MuJoCo keeps it each case
/// is MuJoCo's to the bit; with ours reset none is. On a sphere and an
/// ellipsoid ours is within 1e-7 of the exact distance (the generator's
/// `exact_dist`, from the sphere centre's signed distance to the
/// ellipsoid), where MuJoCo's parts from it by more than 5e-7. Measured:
/// ours 3.0e-8 at most; MuJoCo's 5.8e-7 to 3.1e-6.
#[test]
fn s3d_reset_changes_contacts_toward_the_exact_distance() {
    let doc = golden();
    let meshes = meshes(&doc);
    let cases = doc["s3d"].as_object().expect("the s3d cases");
    let mut exact_checked = 0;
    for (name, case) in cases {
        let want = &case["out"];
        let kept = run(case, &meshes, true, Polygons::Mujocos);
        assert_eq!(
            first_difference(&kept, want),
            None,
            "{name}: MuJoCo's weight"
        );
        let ours = run(case, &meshes, false, Polygons::Mujocos);
        assert!(
            first_difference(&ours, want).is_some(),
            "{name}: ours is MuJoCo's"
        );
        if let Some(exact) = case["exact_dist"].as_f64() {
            let theirs = number(&want["dist"]);
            assert!(
                (theirs - exact).abs() > 5e-7,
                "{name}: MuJoCo's {theirs}, exact {exact}"
            );
            assert!(
                (ours.dist - exact).abs() < 1e-7,
                "{name}: ours {}, exact {exact}",
                ours.dist
            );
            exact_checked += 1;
        }
    }
    assert!(exact_checked > 0, "no case with an exact distance");
}

/// Each mesh's hull graph as the loader builds it ([`loaded`]) is MuJoCo's
/// `mesh_graph`, each hull vertex's index among the mesh's vertices and its
/// neighbours in MuJoCo's order, on every mesh of the golden that holds
/// MuJoCo's (`graph_meshes`, `plane_meshes`). MuJoCo makes it of the
/// vertices as written, before it moves them to the centre of mass, turns
/// and rounds them: qhull's graph of the compiled vertices is not MuJoCo's
/// on any of the `graph_meshes`.
#[test]
fn each_meshs_graph_is_mujocos() {
    let doc = golden();
    let mut compared = 0;
    let mut of_compiled = 0;
    for group in ["graph_meshes", "plane_meshes"] {
        for (name, m) in doc[group].as_object().expect("a group of meshes") {
            let mesh = loaded(m);
            let hull = mesh.hull().expect("the mesh has a hull");
            let (vertex, neighbours) = mujocos_graph(m);
            assert_eq!(hull.vertex, vertex, "{group}/{name}: vert_globalid");
            assert_eq!(hull.neighbours, neighbours, "{group}/{name}: neighbours");
            let faces: Vec<[u32; 3]> = m["faces"]
                .as_array()
                .expect("the hull's faces")
                .iter()
                .map(|f| match counts(f)[..] {
                    [a, b, c] => [a, b, c].map(|i| u32::try_from(i).expect("a vertex")),
                    _ => panic!("{group}/{name}: a face of three vertices"),
                })
                .collect();
            assert_eq!(hull.faces, faces, "{group}/{name}: face_globalid");
            compared += 1;
            if group == "graph_meshes" {
                let mut compiled = TriangleMeshData::new(points(&m["vert"]), Vec::new());
                compiled.compute_hull(None).expect("qhull makes the hull");
                let other = compiled.hull().expect("the mesh has a hull");
                of_compiled +=
                    usize::from(other.vertex == vertex && other.neighbours == neighbours);
            }
        }
    }
    assert_eq!(compared, 12, "meshes with MuJoCo's graph");
    assert_eq!(
        of_compiled, 0,
        "graph_meshes whose compiled vertices give MuJoCo's graph"
    );
}

/// The polygons the loader merges from its hull are MuJoCo's: the same
/// faces in the same order, each the same corners from the same corner, and
/// each vertex on the same faces, on every mesh of `meshes` and
/// `graph_meshes`. MuJoCo makes their normals from the compiled vertices
/// before it rounds them, and the golden holds only the rounded vertices:
/// the loader's normals, made as MuJoCo's, are compared with MuJoCo's in
/// sim-mjcf's `tests/mesh_hull.rs`.
#[test]
fn mesh_polygons_are_mujocos() {
    let doc = golden();
    let meshes = meshes(&doc);
    assert!(
        meshes
            .values()
            .any(|m| m.mujocos.paths.iter().any(|p| p.len() > 3)),
        "no polygon past a triangle"
    );
    assert_eq!(meshes.len(), 8, "meshes");
    for (name, m) in &meshes {
        let ours = m.data.polygons();
        assert_eq!(ours.paths, m.mujocos.paths, "mesh {name}: polygons");
        assert_eq!(
            ours.map, m.mujocos.map,
            "mesh {name}: each vertex's polygons"
        );
    }
}

/// `MakePolygons` where its comparison is not a strict weak order: on the
/// golden's `merge` meshes, whose hulls hold neighbouring faces nearer than
/// the merge tolerance, our merge of MuJoCo's own hull faces gives MuJoCo's
/// polygons in its order, each from its corner, and each vertex's polygons.
/// The same on `graph_meshes`, from MuJoCo's faces of their hulls.
#[test]
fn polygon_merge_is_mujocos_from_its_hull_faces() {
    let doc = golden();
    let merge = doc["merge"].as_object().expect("merge meshes");
    assert!(!merge.is_empty(), "no merge meshes");
    let graph = doc["graph_meshes"].as_object().expect("graph meshes");
    let mut differ = Vec::new();
    // `merge` holds the vertices as `MakePolygons` merges them in `vert`,
    // `graph_meshes` in `authored` (its `vert` is as MuJoCo compiled them).
    let all = merge
        .iter()
        .map(|(name, m)| (name, &m["vert"], m))
        .chain(graph.iter().map(|(name, m)| (name, &m["authored"], m)));
    for (name, verts, m) in all {
        let faces: Vec<[usize; 3]> = m["faces"]
            .as_array()
            .expect("faces")
            .iter()
            .map(|f| {
                let f = counts(f);
                [f[0], f[1], f[2]]
            })
            .collect();
        let ours = make_polygons(&points(verts), &faces);
        let paths: Vec<Vec<usize>> = m["polygons"]
            .as_array()
            .expect("polygons")
            .iter()
            .map(counts)
            .collect();
        let map: Vec<Vec<usize>> = m["polymap"]
            .as_array()
            .expect("a polygon map")
            .iter()
            .map(counts)
            .collect();
        if ours.paths != paths || ours.map != map {
            differ.push(name.as_str());
        }
    }
    assert!(differ.is_empty(), "polygons not MuJoCo's: {differ:?}");
}

/// A box on a mesh and two meshes under MULTICCD, on the loader's meshes and
/// polygons as the golden lets the test make them (their normals from
/// MuJoCo's rounded vertices, the only ones it holds; `sim-mjcf`'s
/// `tests/mesh_hull.rs` compares the loader's own), with `S3D`'s fourth
/// weight as MuJoCo keeps it (registry `D-GJK-S3D`): every case is MuJoCo's
/// to the bit. The multi-contact clips the polygons, keeps a face's first
/// corners from its path's start and prunes a clipped face to four corners
/// from it; the golden holds cases of four contacts.
#[test]
fn box_on_mesh_multiccd_as_mujoco() {
    let doc = golden();
    let meshes = meshes(&doc);
    let cases = doc["multi_mesh"].as_object().expect("the multi_mesh cases");
    let mut four = 0;
    for (name, case) in cases {
        let ours = run(case, &meshes, true, Polygons::Ours);
        assert_eq!(first_difference(&ours, &case["out"]), None, "{name}");
        four += usize::from(ours.nx == 4);
    }
    assert!(four > 0, "no case of four contacts");
}

/// A mesh of the golden's plane cases (`plane_meshes`): our mesh data as
/// the loader builds it ([`loaded`]) and MuJoCo's radius.
struct PlaneMesh {
    data: TriangleMeshData,
    rbound: f64,
}

fn plane_meshes(doc: &Value) -> HashMap<String, PlaneMesh> {
    doc["plane_meshes"]
        .as_object()
        .expect("plane meshes")
        .iter()
        .map(|(name, m)| {
            let mesh = PlaneMesh {
                data: loaded(m),
                rbound: number(&m["rbound"]),
            };
            (name.clone(), mesh)
        })
        .collect()
}

/// Our `mjc_PlaneConvex` on a plane case: its geom made without a margin, a
/// mesh on its own hull graph (or, without `graph`, on none) with no
/// polygons (the plane takes none), spaced by MuJoCo's radius.
fn plane_run(case: &Value, meshes: &HashMap<String, PlaneMesh>, graph: bool) -> Vec<RawContact> {
    let none = MeshPolygons::default();
    let mesh = case["mesh"].as_str().map(|name| &meshes[name]);
    let ccd_mesh = mesh.map(|m| CcdMesh {
        verts: m.data.vertices(),
        hull: m.data.hull().filter(|_| graph).map(|hull| HullGraph {
            vertex: &hull.vertex,
            neighbours: &hull.neighbours,
        }),
        polygons: &none,
    });
    let mut obj = CcdObj::new(
        geom_type(case["type"].as_str().expect("a type")),
        floats(&case["pos"]),
        floats(&case["mat"]),
        floats(&case["size"]),
        0.0,
        ccd_mesh,
    )
    .expect("a geom mjc_PlaneConvex takes");
    mjc_plane_convex(
        &floats(&case["plane"]["pos"]),
        &floats(&case["plane"]["mat"]),
        &mut obj,
        mesh.map_or(0.0, |m| m.rbound),
        number(&case["margin"]),
    )
}

/// Whether two numbers are one, or both NaN.
fn same_bits(a: f64, b: f64) -> bool {
    a.to_bits() == b.to_bits() || (a.is_nan() && b.is_nan())
}

/// Our contacts are MuJoCo's, each `dist`, `pos` and normal bit for bit.
fn contacts_match(ours: &[RawContact], want: &Value) -> bool {
    let want = want.as_array().expect("contacts");
    ours.len() == want.len()
        && ours.iter().zip(want).all(|(c, w)| {
            same_bits(c.dist, number(&w["dist"]))
                && (0..3).all(|k| {
                    same_bits(c.pos[k], number(&w["pos"][k]))
                        && same_bits(c.normal[k], number(&w["normal"][k]))
                })
        })
}

/// Whether a plane case's mesh has a hull graph on MuJoCo's side: none in
/// `mesh_bare` and every case named `_bare` or `bare_`.
fn has_graph(group: &str, name: &str) -> bool {
    !(group == "mesh_bare" || name.contains("_bare") || name.starts_with("bare_"))
}

/// `mjc_PlaneConvex` (and `mjccd_support` under it) against MuJoCo's, bit
/// for bit, on every plane case of the golden (`plane_convex`): ellipsoids,
/// those apart at a margin of their own distance, meshes on the loader's
/// hull graph (MuJoCo's) and without one, the comparisons' edges (vertices
/// at the margin exactly; a first vertex past the spacing without a graph),
/// NaN turns (MuJoCo's NaN contact) and a tetrahedron (on each face, at
/// random turns, and where two corners tie exactly for the deepest).
#[test]
fn plane_convex_matches_mujoco_bitwise() {
    let doc = golden();
    let meshes = plane_meshes(&doc);
    let mut total = 0;
    for (group, cases) in doc["plane_convex"].as_object().expect("plane groups") {
        let cases = cases.as_object().expect("plane cases");
        assert!(!cases.is_empty(), "plane group {group} is empty");
        for (name, case) in cases {
            let ours = plane_run(case, &meshes, has_graph(group, name));
            assert!(
                contacts_match(&ours, &case["contacts"]),
                "{group}/{name}: ours {:?}, MuJoCo {}",
                ours.iter().map(|c| (c.dist, c.pos)).collect::<Vec<_>>(),
                case["contacts"]
            );
            total += 1;
        }
    }
    assert_eq!(total, 711, "plane cases");
}

/// A height field of the golden (`hfield_fields`) as sim-core holds it:
/// MuJoCo's data scaled by the field's top, as `mjc_ConvexHField` scales
/// each height it reads, and its size.
fn hfield_of(field: &Value) -> (HeightFieldData, [f64; 4]) {
    let (nrow, ncol) = (count(&field["nrow"]), count(&field["ncol"]));
    let size: [f64; 4] = floats(&field["size"]);
    let heights = field["data"]
        .as_array()
        .expect("data")
        .iter()
        .map(|h| number(h) * size[2])
        .collect();
    // A grid's extent, far below 2^52, exact as a float.
    #[allow(clippy::cast_precision_loss)]
    let cell = 2.0 * size[0] / (ncol - 1) as f64;
    (HeightFieldData::new(heights, ncol, nrow, cell), size)
}

/// Our `mjc_ConvexHField` on a height-field case, its mesh on the loader's
/// hull graph (MuJoCo's), with MuJoCo's bounding radius.
fn hfield_run(case: &Value, doc: &Value, meshes: &HashMap<String, PlaneMesh>) -> Vec<RawContact> {
    let none = MeshPolygons::default();
    let field = case["field"].as_str().expect("a field");
    let (hfield, size) = hfield_of(&doc["hfield_fields"][field]);
    let mesh = case["mesh"].as_str().map(|name| {
        let m = &meshes[name];
        let hull = m.data.hull().expect("the mesh has a hull");
        CcdMesh {
            verts: m.data.vertices(),
            hull: Some(HullGraph {
                vertex: &hull.vertex,
                neighbours: &hull.neighbours,
            }),
            polygons: &none,
        }
    });
    let mut obj = CcdObj::new(
        geom_type(case["type"].as_str().expect("a type")),
        floats(&case["pos"]),
        floats(&case["mat"]),
        floats(&case["size"]),
        0.0,
        mesh,
    )
    .expect("a geom mjc_ConvexHField takes");
    mjc_convex_hfield(
        &hfield,
        &size,
        &floats(&case["hfield"]["pos"]),
        &floats(&case["hfield"]["mat"]),
        &mut obj,
        number(&case["rbound"]),
        number(&case["margin"]),
        (count(&case["ccd"][0]), number(&case["ccd"][1])),
    )
}

/// `mjc_ConvexHField` against MuJoCo's, bit for bit, on every height-field
/// case of the golden (`hfield_convex`): four fields (square cells, cells of
/// two spacings, a fine grid, one cell), each convex type and two meshes on
/// the loader's hull graph at random poses, the box tests' edges, mjMAXCONPAIR
/// contacts, a geom apart listed within the margin, a geom past an edge
/// within the margin (unlisted), boxes resting exactly on a field's flat part
/// (unlisted), a sphere under the base within the margin, a field at 1e154
/// (its contacts' normals zero, skipped), and NaN turns.
#[test]
fn hfield_convex_matches_mujoco_bitwise() {
    let doc = golden();
    let meshes = plane_meshes(&doc);
    let mut total = 0;
    for (group, cases) in doc["hfield_convex"]
        .as_object()
        .expect("height-field groups")
    {
        let cases = cases.as_object().expect("height-field cases");
        assert!(!cases.is_empty(), "height-field group {group} is empty");
        for (name, case) in cases {
            let ours = hfield_run(case, &doc, &meshes);
            assert!(
                contacts_match(&ours, &case["contacts"]),
                "{group}/{name}: ours {:?}, MuJoCo {}",
                ours.iter().map(|c| (c.dist, c.pos)).collect::<Vec<_>>(),
                case["contacts"]
            );
            total += 1;
        }
    }
    assert_eq!(total, 352, "height-field cases");
}
