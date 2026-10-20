//! `mjc_ccd` against MuJoCo's, bit for bit: every case of the golden
//! `ccd.json` (`sim/L0/tests/scripts/gen_ccd_reference.py`, from the
//! unfused MuJoCo 3.5.0 oracle; it describes the cases).

// Test code: a malformed golden is a failed test.
#![allow(clippy::expect_used, clippy::panic)]

use std::collections::HashMap;

use nalgebra::Point3;
use serde_json::Value;

use super::{CcdConfig, CcdMesh, CcdObj, CcdStatus, HullGraph, STALE_S3D, mjc_ccd};
use crate::mesh::TriangleMeshData;
use crate::mesh_polygons::{MeshPolygons, make_polygons};
use crate::types::GeomType;

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

/// A mesh of the golden as MuJoCo compiled it: our mesh data over its
/// vertices (its hull, each hull vertex's index among the mesh's vertices,
/// the polygons we merge), MuJoCo's polygons, those started at the corner
/// ours start at, and our mesh data over the vertices as authored.
struct Mesh {
    data: TriangleMeshData,
    mujocos: MeshPolygons,
    started_as_ours: MeshPolygons,
    authored: TriangleMeshData,
}

/// Whose polygons a mesh takes.
#[derive(Clone, Copy)]
enum Polygons {
    Mujocos,
    Ours,
    MujocosStartedAsOurs,
}

/// Whether `a` is `b` started from another of its corners.
fn is_rotation(a: &[usize], b: &[usize]) -> bool {
    a.len() == b.len() && (0..a.len()).any(|r| a.iter().cycle().skip(r).take(a.len()).eq(b))
}

/// Our mesh data over `verts`, its faces one triangle: the hull and our
/// polygons come from the vertices alone (polygons merged from the mesh's
/// faces instead would show).
fn mesh_data(verts: Vec<Point3<f64>>) -> TriangleMeshData {
    let mut data = TriangleMeshData::from_faces(verts, vec![[0, 1, 2]]);
    data.compute_convex_hull(None);
    data
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

fn meshes(doc: &Value) -> HashMap<String, Mesh> {
    let mut out = HashMap::new();
    for (name, m) in doc["meshes"].as_object().expect("meshes") {
        let verts = points(&m["vert"]);
        // Our selection of the search agrees with MuJoCo's on these meshes.
        assert_eq!(
            m["hillclimb"].as_bool(),
            Some(verts.len() >= 10),
            "mesh {name}'s search"
        );
        let mujocos = mujoco_polygons(m);
        let data = mesh_data(verts);
        assert_eq!(
            data.hull_vertex().len(),
            data.convex_hull()
                .expect("the mesh has a hull")
                .vertices
                .len(),
            "mesh {name}: a hull vertex that is not a mesh vertex"
        );
        let started_as_ours = MeshPolygons {
            paths: mujocos
                .paths
                .iter()
                .map(|path| {
                    let ours = data
                        .polygons()
                        .paths
                        .iter()
                        .find(|q| is_rotation(q, path))
                        .expect("each of MuJoCo's polygons is one of ours");
                    let start = path
                        .iter()
                        .position(|&v| v == ours[0])
                        .expect("a shared corner");
                    [&path[start..], &path[..start]].concat()
                })
                .collect(),
            ..mujocos.clone()
        };
        let authored = mesh_data(points(&m["authored"]));
        out.insert(
            name.clone(),
            Mesh {
                data,
                mujocos,
                started_as_ours,
                authored,
            },
        );
    }
    out
}

/// A mesh of the golden on MuJoCo's own hull graph (`graph_meshes`): its
/// vertices as MuJoCo compiled them, its graph (each hull vertex's index
/// among the mesh's vertices, `vert_globalid`, and its neighbours among the
/// hull's vertices) and its polygons. Its cases are compared on MuJoCo's
/// graph and polygons.
struct GraphMesh {
    verts: Vec<Point3<f64>>,
    vertex: Vec<usize>,
    neighbours: Vec<Vec<u32>>,
    polygons: MeshPolygons,
    /// Our mesh data over its vertices, for its cases on our hull's graph.
    ours: TriangleMeshData,
}

fn graph_meshes(doc: &Value) -> HashMap<String, GraphMesh> {
    let mut out = HashMap::new();
    for (name, m) in doc["graph_meshes"].as_object().expect("graph meshes") {
        let verts = points(&m["vert"]);
        assert_eq!(
            m["hillclimb"].as_bool(),
            Some(verts.len() >= 10),
            "mesh {name}'s search"
        );
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
        let ours = mesh_data(verts.clone());
        assert_eq!(
            ours.hull_vertex().len(),
            ours.convex_hull()
                .expect("the mesh has a hull")
                .vertices
                .len(),
            "mesh {name}: a hull vertex that is not a mesh vertex"
        );
        out.insert(
            name.clone(),
            GraphMesh {
                verts,
                vertex: counts(&m["graph"]["vertex"]),
                neighbours,
                polygons: mujoco_polygons(m),
                ours,
            },
        );
    }
    out
}

/// The golden's meshes: on our hull's graph (`meshes`) and on MuJoCo's
/// (`graph_meshes`, which `our_graph` puts on ours).
struct Meshes {
    ours: HashMap<String, Mesh>,
    on_mujocos_graph: HashMap<String, GraphMesh>,
    our_graph: bool,
}

impl Meshes {
    fn of(doc: &Value) -> Self {
        Self {
            ours: meshes(doc),
            on_mujocos_graph: graph_meshes(doc),
            our_graph: false,
        }
    }
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

/// The object of a case; a mesh of `graph_meshes` takes MuJoCo's polygons
/// whichever `polygons` asks for.
fn object<'a>(v: &Value, meshes: &'a Meshes, polygons: Polygons) -> CcdObj<'a> {
    let mesh = v["mesh"].as_str().map(|name| {
        if let Some(g) = meshes.on_mujocos_graph.get(name) {
            let hull = if meshes.our_graph {
                HullGraph {
                    vertex: g.ours.hull_vertex(),
                    neighbours: &g.ours.convex_hull().expect("the mesh has a hull").adjacency,
                }
            } else {
                HullGraph {
                    vertex: &g.vertex,
                    neighbours: &g.neighbours,
                }
            };
            return CcdMesh {
                verts: &g.verts,
                hull: Some(hull),
                polygons: &g.polygons,
            };
        }
        let m = &meshes.ours[name];
        let hull = m.data.convex_hull().expect("the mesh has a hull");
        CcdMesh {
            verts: m.data.vertices(),
            hull: Some(HullGraph {
                vertex: m.data.hull_vertex(),
                neighbours: &hull.adjacency,
            }),
            polygons: match polygons {
                Polygons::Mujocos => &m.mujocos,
                Polygons::Ours => m.data.polygons(),
                Polygons::MujocosStartedAsOurs => &m.started_as_ours,
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
fn run(case: &Value, meshes: &Meshes, stale: bool, polygons: Polygons) -> CcdStatus {
    let mut o1 = object(&case["obj"][0], meshes, polygons);
    let mut o2 = object(&case["obj"][1], meshes, polygons);
    STALE_S3D.with(|s| s.set(stale));
    let st = mjc_ccd(&config(&case["config"]), &mut o1, &mut o2);
    STALE_S3D.with(|s| s.set(false));
    st
}

/// Every case of the golden: the distance, the witness points, the GJK and
/// EPA iteration counts, the EPA status and the simplex size, to the bit,
/// a mesh taking MuJoCo's polygons (and a mesh of `graph_meshes` MuJoCo's
/// hull graph). With `S3D`'s fourth weight kept as MuJoCo keeps it every
/// case is MuJoCo's; with ours reset, a case differs only where keeping it
/// gives MuJoCo's (registry `D-GJK-S3D`), and the golden holds such a case.
#[test]
fn mjc_ccd_matches_mujoco_bitwise() {
    let doc = golden();
    let meshes = Meshes::of(&doc);
    let mut compared = 0;
    let mut failures = Vec::new();
    let mut stale_weight = Vec::new();
    let groups = doc.as_object().expect("the golden's groups");
    for (group, cases) in groups
        .iter()
        .filter(|(k, _)| !["oracle", "meshes", "merge", "graph_meshes"].contains(&k.as_str()))
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

/// The cases on `graph_meshes` need MuJoCo's hull graph: on our hull's
/// graph (with MuJoCo's polygons and its `S3D` weight) the golden holds
/// cases that are not MuJoCo's to the bit, where a hill climb ends on
/// another vertex than MuJoCo's.
#[test]
fn graph_cases_need_mujocos_graph() {
    let doc = golden();
    let mut meshes = Meshes::of(&doc);
    meshes.our_graph = true;
    let on_graph_meshes = |case: &&Value| {
        (0..2).any(|i| {
            case["obj"][i]["mesh"]
                .as_str()
                .is_some_and(|name| meshes.on_mujocos_graph.contains_key(name))
        })
    };
    let differ = doc
        .as_object()
        .expect("the golden's groups")
        .iter()
        .filter(|(k, _)| !["oracle", "meshes", "merge", "graph_meshes"].contains(&k.as_str()))
        .flat_map(|(_, cases)| cases.as_object().expect("a group of cases").values())
        .filter(on_graph_meshes)
        .filter(|case| {
            let st = run(case, &meshes, true, Polygons::Mujocos);
            first_difference(&st, &case["out"]).is_some()
        })
        .count();
    assert!(
        differ > 0,
        "no case differs on our hull's graph: MuJoCo's graph is not checked"
    );
}

/// The polygons we merge from our hull are MuJoCo's from qhull's. On the
/// vertices as authored, where MuJoCo merges them: the same faces in the
/// same order, each the same corners round in the same direction (from any
/// one of them), and each vertex on the same faces. On the compiled
/// vertices, each face's normal within 1e-6 of MuJoCo's, which come from
/// the vertices before they are rounded to floats.
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
    for (name, m) in &meshes {
        let theirs = &m.mujocos;
        let ours = m.authored.polygons();
        assert_eq!(
            ours.paths.len(),
            theirs.paths.len(),
            "mesh {name}: polygons"
        );
        for (i, (a, b)) in ours.paths.iter().zip(&theirs.paths).enumerate() {
            assert!(
                is_rotation(a, b),
                "mesh {name} polygon {i}: {a:?}, MuJoCo {b:?}"
            );
        }
        assert_eq!(ours.map, theirs.map, "mesh {name}: each vertex's polygons");

        let compiled = m.data.polygons();
        for (path, normal) in theirs.paths.iter().zip(&theirs.normals) {
            let j = compiled
                .paths
                .iter()
                .position(|q| is_rotation(q, path))
                .expect("each of MuJoCo's polygons is one of ours");
            let gap = compiled.normals[j]
                .iter()
                .zip(normal)
                .map(|(x, y)| (x - y).abs())
                .fold(0.0, f64::max);
            assert!(
                gap < 1e-6,
                "mesh {name} polygon {path:?}: normal {:?}, MuJoCo {normal:?}",
                compiled.normals[j]
            );
        }
    }
}

/// `MakePolygons` where its comparison is not a strict weak order: on the
/// golden's `merge` meshes, whose hulls hold neighbouring faces nearer than
/// the merge tolerance, our merge of MuJoCo's own hull faces gives MuJoCo's
/// polygons in its order, each from its corner, and each vertex's polygons.
/// The same on the meshes on MuJoCo's graph (`graph_meshes`), whose cases
/// take MuJoCo's polygons.
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

/// Whether two statuses list the same contacts, each witness pair within
/// 1e-9, in any order.
fn same_contacts(a: &CcdStatus, b: &CcdStatus) -> bool {
    let near = |i: usize, j: usize| {
        (0..3).all(|k| {
            (a.x1[i][k] - b.x1[j][k]).abs() < 1e-9 && (a.x2[i][k] - b.x2[j][k]).abs() < 1e-9
        })
    };
    a.nx == b.nx && (0..a.nx).all(|i| (0..b.nx).any(|j| near(i, j)))
}

/// A box on a mesh and two meshes under MULTICCD, end to end on our
/// polygons: every case's contact count and distance are MuJoCo's, and its
/// contacts are those MuJoCo's own polygons give when each is started at
/// the corner ours start at. That start is where the multi-contact's
/// pruning of a clipped face to four corners begins (registry
/// `D-HULL-ORDER`): the golden holds cases where it keeps other corners
/// than MuJoCo's, and cases of four contacts where it keeps MuJoCo's.
#[test]
fn box_on_mesh_multiccd_as_mujoco() {
    let doc = golden();
    let meshes = Meshes::of(&doc);
    let cases = doc["multi_mesh"].as_object().expect("the multi_mesh cases");
    let mut other_corners = 0;
    let mut mujocos_four = 0;
    for (name, case) in cases {
        let ours = run(case, &meshes, false, Polygons::Ours);
        let mujocos = run(case, &meshes, false, Polygons::Mujocos);
        let started = run(case, &meshes, false, Polygons::MujocosStartedAsOurs);
        let want = &case["out"];
        assert_eq!(ours.nx, count(&want["nx"]), "{name}: contacts");
        let dist = number(&want["dist"]);
        assert!(
            (ours.dist - dist).abs() < 1e-12,
            "{name}: dist {}, MuJoCo {dist}",
            ours.dist
        );
        assert!(
            same_contacts(&ours, &started),
            "{name}: not the contacts of MuJoCo's polygons started as ours"
        );
        if !same_contacts(&ours, &mujocos) {
            other_corners += 1;
        } else if ours.nx == 4 {
            mujocos_four += 1;
        }
    }
    assert!(
        other_corners > 0,
        "no case where the start keeps other corners"
    );
    assert!(
        mujocos_four > 0,
        "no case of four contacts kept as MuJoCo's"
    );
}
