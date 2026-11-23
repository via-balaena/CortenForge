//! A geom of type `sdf` over a mesh with no plugin, a loaded field, against
//! MuJoCo 3.5.0's: its octree read at points (`oct_distance`,
//! `oct_gradient`).
//!
//! The golden is `assets/golden/sdf/sdf.json`, from
//! `scripts/gen_sdf_reference.py` on the unfused oracle. On macOS arm64,
//! where it was made, every value is compared bit for bit. Elsewhere an
//! octree corner's sign can differ where that platform's `acos` does (the
//! angles TriangleMeshDistance weights a vertex's pseudonormal by), so each
//! value is held within [`TOL`] of MuJoCo's, a NaN failing (not measured off
//! macOS).

use std::sync::OnceLock;

use serde_json::Value;

/// Whether values are compared bit for bit: on the platform the golden was
/// made on.
const BITWISE: bool = cfg!(all(target_os = "macos", target_arch = "aarch64"));

/// The bound off macOS arm64 (see the module's doc).
const TOL: f64 = 1e-9;

fn golden() -> &'static Value {
    static GOLDEN: OnceLock<Value> = OnceLock::new();
    GOLDEN.get_or_init(|| {
        serde_json::from_str(include_str!("../assets/golden/sdf/sdf.json"))
            .expect("sdf golden parses")
    })
}

/// A golden double: its bits as a 64-bit integer in hex.
fn double(v: &Value) -> f64 {
    f64::from_bits(u64::from_str_radix(v.as_str().expect("hex"), 16).expect("hex bits"))
}

fn doubles(v: &Value) -> Vec<f64> {
    v.as_array().expect("array").iter().map(double).collect()
}

/// Whether `ours` is MuJoCo's `theirs`: the same bits on macOS arm64, else
/// within [`TOL`] (a NaN on either side fails).
fn same(ours: f64, theirs: f64) -> bool {
    if BITWISE {
        ours.to_bits() == theirs.to_bits()
    } else {
        (ours - theirs).abs() <= TOL
    }
}

/// Each loaded field's octree read where MuJoCo reads it: at each golden
/// point (a grid over 1.3 times its root box, random points within three
/// times it, points far along each axis and on its faces),
/// `MeshOctree::distance` and `MeshOctree::gradient` against MuJoCo's
/// `mjc_distance` and `mjc_gradient` of the field alone.
#[test]
fn each_octree_read_is_mujocos() {
    let fields = golden()["fields"].as_object().expect("fields");
    assert_eq!(fields.len(), 3, "golden fields");
    let mut failures = Vec::new();
    let mut points = 0;
    for (name, case) in fields {
        let model = sim_mjcf::load_model(case["xml"].as_str().expect("xml")).expect("load");
        let octree = model.mesh_data[0].octree().expect("an octree");
        assert_eq!(
            octree.len() as u64,
            case["octnum"].as_u64().expect("octnum"),
            "{name}"
        );
        for ((p, d), g) in case["points"]
            .as_array()
            .expect("points")
            .iter()
            .zip(case["dist"].as_array().expect("dist"))
            .zip(case["grad"].as_array().expect("grad"))
        {
            let p: [f64; 3] = doubles(p).try_into().expect("a point");
            let (dist, grad) = (
                octree.distance(p).expect("a leaf"),
                octree.gradient(p).expect("a leaf"),
            );
            let theirs = (double(d), doubles(g));
            if !same(dist, theirs.0) || grad.iter().zip(&theirs.1).any(|(&a, &b)| !same(a, b)) {
                failures.push(format!("{name} at {p:?}: ({dist}, {grad:?}) / {theirs:?}"));
            }
            points += 1;
        }
    }
    assert!(points > 1000, "golden points {points}");
    assert!(
        failures.is_empty(),
        "{} of {points}:\n{}",
        failures.len(),
        failures.join("\n")
    );
}
