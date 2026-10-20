//! `mjc_ccd` against MuJoCo's, bit for bit: every case of the golden
//! `ccd.json` (`sim/L0/tests/scripts/gen_ccd_reference.py`, from the
//! unfused MuJoCo 3.5.0 oracle; it describes the cases).

// Test code: a malformed golden is a failed test.
#![allow(clippy::expect_used, clippy::panic)]

use std::collections::HashMap;

use cf_geometry::convex_hull;
use nalgebra::Point3;
use serde_json::Value;

use super::{CcdConfig, CcdMesh, CcdObj, CcdStatus, HullGraph, STALE_S3D, mjc_ccd};
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

/// A mesh of the golden as MuJoCo compiled it, with our hull's graph over
/// it: each hull vertex's index among the mesh's vertices and its
/// neighbours.
struct Mesh {
    verts: Vec<Point3<f64>>,
    hull_vertex: Vec<usize>,
    neighbours: Vec<Vec<u32>>,
}

fn meshes(doc: &Value) -> HashMap<String, Mesh> {
    let mut out = HashMap::new();
    for (name, m) in doc["meshes"].as_object().expect("meshes") {
        let flat: Vec<f64> = m["vert"]
            .as_array()
            .expect("vertices")
            .iter()
            .map(number)
            .collect();
        let verts: Vec<Point3<f64>> = flat
            .chunks_exact(3)
            .map(|c| Point3::new(c[0], c[1], c[2]))
            .collect();
        // Our selection of the search agrees with MuJoCo's on these meshes.
        assert_eq!(
            m["hillclimb"].as_bool(),
            Some(verts.len() >= 10),
            "mesh {name}'s search"
        );
        let hull = convex_hull(&verts, None).expect("the mesh has a hull");
        let hull_vertex = hull
            .vertices
            .iter()
            .map(|h| {
                verts
                    .iter()
                    .position(|v| v == h)
                    .expect("a hull vertex is a mesh vertex")
            })
            .collect();
        out.insert(
            name.clone(),
            Mesh {
                verts,
                hull_vertex,
                neighbours: hull.adjacency,
            },
        );
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

fn object<'a>(v: &Value, meshes: &'a HashMap<String, Mesh>) -> CcdObj<'a> {
    let mesh = v["mesh"].as_str().map(|name| {
        let m = &meshes[name];
        CcdMesh {
            verts: &m.verts,
            hull: Some(HullGraph {
                vertex: &m.hull_vertex,
                neighbours: &m.neighbours,
            }),
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
/// ours resets it.
fn run(case: &Value, meshes: &HashMap<String, Mesh>, stale: bool) -> CcdStatus {
    let mut o1 = object(&case["obj"][0], meshes);
    let mut o2 = object(&case["obj"][1], meshes);
    STALE_S3D.with(|s| s.set(stale));
    let st = mjc_ccd(&config(&case["config"]), &mut o1, &mut o2);
    STALE_S3D.with(|s| s.set(false));
    st
}

/// Every case of the golden: the distance, the witness points, the GJK and
/// EPA iteration counts, the EPA status and the simplex size, to the bit.
/// With `S3D`'s fourth weight kept as MuJoCo keeps it every case is
/// MuJoCo's; with ours reset, a case differs only where keeping it gives
/// MuJoCo's (registry `D-GJK-S3D`), and the golden holds such a case.
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
        .filter(|(k, _)| !["oracle", "meshes"].contains(&k.as_str()))
    {
        let cases = cases.as_object().expect("a group of cases");
        assert!(!cases.is_empty(), "no {group} cases");
        for (name, case) in cases {
            let want = &case["out"];
            let mujocos = first_difference(&run(case, &meshes, true), want);
            match (first_difference(&run(case, &meshes, false), want), mujocos) {
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
