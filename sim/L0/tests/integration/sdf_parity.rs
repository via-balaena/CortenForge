//! A geom of type `sdf` over a mesh with no plugin, a loaded field, against
//! MuJoCo 3.5.0's: its octree read at points (`oct_distance`,
//! `oct_gradient`), and scenes where it meets each geom MuJoCo's collision
//! table sends to it (`mjc_SDF`; a plane through `mjc_PlaneConvex` over its
//! mesh; a height field not at all; a flex through the convex route), casts
//! a rangefinder's ray (`mj_raySdf`) and the geom-distance sensors.
//!
//! The golden is `assets/golden/sdf/sdf.json`, from
//! `scripts/gen_sdf_reference.py` on the unfused oracle, which says what each
//! scene holds. On macOS arm64, where it was made, every value is compared
//! bit for bit. Elsewhere an octree corner's sign can differ where that
//! platform's `acos` does (the angles TriangleMeshDistance weights a
//! vertex's pseudonormal by), so each value read after `mj_forward` is held
//! within [`TOL`] of MuJoCo's, a NaN failing, and a scene's steps, whose
//! turns take `sin` and `cos`, are not compared there (not measured off
//! macOS).

use std::sync::{Arc, OnceLock};

use nalgebra::Point3;
use serde_json::Value;
use sim_core::{SdfGrid, ShapeSphere};

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

/// The ways scene `name`'s forward, and with `steps` its steps, differ from
/// MuJoCo's; empty where they are its.
fn scene_differences(name: &str, steps: bool) -> Vec<String> {
    let case = &golden()["scenes"][name];
    let xml = case["xml"].as_str().expect("xml");
    let mut out = Vec::new();
    let model = match sim_mjcf::load_model(xml) {
        Ok(model) => model,
        Err(e) => return vec![format!("refused: {e}")],
    };
    let mut data = model.make_data();
    if let Err(e) = data.forward(&model) {
        return vec![format!("forward: {e:?}")];
    }
    let ncon = case["ncon"].as_u64().expect("ncon") as usize;
    if data.ncon != ncon {
        out.push(format!("ncon {} / {ncon}", data.ncon));
        return out;
    }
    let id = |x: Option<usize>| x.map_or(-1, |v| v as i64);
    for (i, (c, theirs)) in data.contacts[..data.ncon]
        .iter()
        .zip(case["contacts"].as_array().expect("contacts"))
        .enumerate()
    {
        for (key, ours) in [
            ("geom", [id(c.geom[0]), id(c.geom[1])]),
            ("flex", [id(c.flex[0]), id(c.flex[1])]),
            ("vert", [id(c.vert[0]), id(c.vert[1])]),
            ("elem", [id(c.elem[0]), id(c.elem[1])]),
        ] {
            let theirs: Vec<i64> = theirs[key]
                .as_array()
                .expect("ids")
                .iter()
                .map(|v| v.as_i64().expect("id"))
                .collect();
            if ours[..] != theirs[..] {
                out.push(format!("contact {i} {key} {ours:?} / {theirs:?}"));
            }
        }
        let frame: Vec<f64> = [c.normal, c.frame[0], c.frame[1]]
            .iter()
            .flat_map(|v| [v.x, v.y, v.z])
            .collect();
        let values = [
            ("dist", vec![-c.depth], vec![double(&theirs["dist"])]),
            (
                "pos",
                c.pos.iter().copied().collect(),
                doubles(&theirs["pos"]),
            ),
            ("frame", frame, doubles(&theirs["frame"])),
        ];
        for (key, ours, theirs) in values {
            if ours.len() != theirs.len() || ours.iter().zip(&theirs).any(|(&a, &b)| !same(a, b)) {
                out.push(format!("contact {i} {key} {ours:?} / {theirs:?}"));
            }
        }
    }
    let sensors = doubles(&case["sensordata"]);
    let ours: Vec<f64> = data.sensordata.iter().copied().collect();
    if ours.len() != sensors.len() || ours.iter().zip(&sensors).any(|(&a, &b)| !same(a, b)) {
        out.push(format!("sensordata {ours:?} / {sensors:?}"));
    }
    if BITWISE && steps {
        for (k, step) in case["steps"].as_array().expect("steps").iter().enumerate() {
            if let Err(e) = data.step(&model) {
                out.push(format!("step {k}: {e:?}"));
                break;
            }
            let qpos = doubles(&step["qpos"]);
            let ncon = step["ncon"].as_u64().expect("ncon") as usize;
            let ours: Vec<f64> = data.qpos.iter().copied().collect();
            if data.ncon != ncon
                || ours
                    .iter()
                    .zip(&qpos)
                    .any(|(a, b)| a.to_bits() != b.to_bits())
            {
                out.push(format!(
                    "step {k}: ncon {} / {ncon}, qpos {ours:?} / {qpos:?}",
                    data.ncon
                ));
                break;
            }
        }
    }
    out
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

/// The scenes whose steps are not compared: a flexcomp grid with edge
/// equality leaves MuJoCo's at its second step by up to 2.6e-21 in `qpos`
/// over any geom (measured at `main` and here, over a box, a mesh and this
/// field alike: ledger L443), so its steps say nothing of the field.
const FORWARD_ONLY: [&str; 2] = ["f01_flex_on_field", "f02_flex_on_field_no_midphase"];

/// Each golden scene's forward (contacts and sensor readings) and five
/// steps against MuJoCo's: a loaded field against each primitive, a mesh,
/// another field, a plane, a height field and a flex, turned and moved; its
/// options; explicit pairs; rangefinders; the geom-distance sensors.
#[test]
fn each_scene_is_mujocos() {
    let scenes = golden()["scenes"].as_object().expect("scenes");
    assert_eq!(scenes.len(), 49, "golden scenes");
    let failures: Vec<String> = scenes
        .keys()
        .filter_map(|name| {
            let differences = scene_differences(name, !FORWARD_ONLY.contains(&name.as_str()));
            (!differences.is_empty()).then(|| format!("{name}: {}", differences.join("; ")))
        })
        .collect();
    assert!(
        failures.is_empty(),
        "{} of {}:\n{}",
        failures.len(),
        scenes.len(),
        failures.join("\n")
    );
}

/// Registry D-SDF-INITPOINTS: a box deep in a cube field, the golden scene
/// `o06_initpoints_50_deep` with 51 and 80 starting points. MuJoCo stops on
/// its 51st contact (measured: "mjc_gradient: SDF type not available"); here
/// each is listed, inside the field and finite.
#[test]
fn past_fifty_starting_points_each_contact_is_listed() {
    let xml = golden()["scenes"]["o06_initpoints_50_deep"]["xml"]
        .as_str()
        .expect("xml");
    assert!(xml.contains(r#"sdf_initpoints="50""#), "the scene's option");
    for initpoints in [51, 80] {
        let model = sim_mjcf::load_model(&xml.replace(
            r#"sdf_initpoints="50""#,
            &format!(r#"sdf_initpoints="{initpoints}""#),
        ))
        .expect("load");
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        assert_eq!(data.ncon, initpoints, "one contact per starting point");
        for c in &data.contacts[..data.ncon] {
            assert!(
                c.depth >= 0.0 && c.pos.iter().all(|x| x.is_finite()),
                "{c:?}"
            );
        }
    }
}

/// Registry D-SDF-SHAPED-PAIR: two loaded cube fields 0.01 into each other
/// (the golden scene `s01_two_fields_same_mesh`) meet as MuJoCo's, 40
/// contacts; given a shape of its own, the first meets the second not at
/// all, where MuJoCo has no field with a shape.
#[test]
fn a_shaped_field_meets_no_loaded_field() {
    let xml = golden()["scenes"]["s01_two_fields_same_mesh"]["xml"]
        .as_str()
        .expect("xml");
    let mut model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    assert_eq!(data.ncon, 40, "two loaded fields");
    let grid = Arc::new(SdfGrid::sphere(Point3::origin(), 0.1, 16, 0.05));
    model.shape_data.push(Arc::new(ShapeSphere::new(grid, 0.1)));
    model.nshape = model.shape_data.len();
    model.geom_shape[0] = Some(model.nshape - 1);
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    assert_eq!(data.ncon, 0, "a shaped field and a loaded one");
}
