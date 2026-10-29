//! A site's rangefinder against MuJoCo 3.5.0's, which casts `mj_ray` from the
//! site along its z axis (`engine_sensor.c:468-482`) over the geoms its body
//! does not hold and its alpha does not hide (`ray_eliminate`,
//! `engine_ray.c:68-99`), a primitive by `mju_rayGeom`, a height field by
//! `mj_rayHfield`, a mesh by `mj_rayMesh`, and clips the reading to ±cutoff
//! (the rangefinder's datatype is real, `user_objects.cc:7209-7217`).
//!
//! The golden is `assets/golden/rangefinder/rangefinder.json`, from
//! `scripts/gen_rangefinder_reference.py` on the unfused oracle: a rangefinder
//! among five primitives and an unbounded plane at 240 states (hits and
//! misses), rays through a geom whose alpha is 0 and through a geom of the
//! sensor's own body, a ray stopped by a geom whose alpha is 0.25, a hit and a
//! miss under a cutoff of 0.5, a hit at 150 with no cutoff; and at 80 turned
//! states, a rangefinder over a convex mesh slab, one between two height
//! fields, and one between two meshes MuJoCo keeps in their written frame.
//! No geom is written turned: every turn but the slab's comes from the states'
//! qpos, and the height fields' elevations are ones sim-mjcf loads as MuJoCo
//! compiles them, which it does not for every field until book Rigid-loading
//! L45. Every reading is compared bit for bit on macOS arm64, where the golden
//! was made, elsewhere within `1e-13 · max(1, |MuJoCo's|)`, a NaN failing; but
//! the slab's. MuJoCo turns the slab to its principal frame, which sim-mjcf
//! does not (book Rigid-loading L35), and rounds the turned vertices to
//! single precision (ledger L281): its readings hit where MuJoCo's hit, each
//! within `1e-13 · max(1, |MuJoCo's|)` (measured on macOS arm64: 18 of the
//! 80 differ, by 1.2e-15 at most, by that measure). The ray golden's rays on
//! the same slab, given MuJoCo's compiled vertices and pose, are MuJoCo's bit
//! for bit (sim-core's `rays_meet_each_mesh_as_mujoco`); none of them passes
//! through a vertex or across an edge, where a ray can read otherwise (ledger
//! L281).

use std::sync::Arc;

use nalgebra::Point3;
use serde_json::Value;
use sim_core::{GeomType, SdfGrid, ShapeSphere};

fn golden() -> Value {
    serde_json::from_str(include_str!(
        "../assets/golden/rangefinder/rangefinder.json"
    ))
    .expect("rangefinder golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

fn matches_golden(ours: &[f64], mj: &[f64]) -> bool {
    ours.len() == mj.len()
        && ours.iter().zip(mj).all(|(a, b)| {
            a.to_bits() == b.to_bits()
                || (!cfg!(all(target_os = "macos", target_arch = "aarch64"))
                    && (a - b).abs() <= 1e-13 * b.abs().max(1.0))
        })
}

/// The slab's readings: a hit where MuJoCo's is one, each within `1e-13 ·
/// max(1, |MuJoCo's|)`, a NaN failing.
fn matches_slab(ours: &[f64], mj: &[f64]) -> bool {
    ours.len() == mj.len()
        && ours
            .iter()
            .zip(mj)
            .all(|(a, b)| (*a >= 0.0) == (*b >= 0.0) && (a - b).abs() <= 1e-13 * b.abs().max(1.0))
}

#[test]
fn rangefinders_read_as_mujoco() {
    let golden = golden();
    let mut failures = Vec::new();
    let mut readings = 0;
    for case in golden["cases"].as_array().expect("cases") {
        let name = case["name"].as_str().expect("name");
        let model = sim_mjcf::load_model(case["xml"].as_str().expect("xml")).expect("load");
        for (k, state) in case["states"]
            .as_array()
            .expect("states")
            .iter()
            .enumerate()
        {
            let mut data = model.make_data();
            data.qpos
                .as_mut_slice()
                .copy_from_slice(&floats(&state["qpos"]));
            data.forward(&model).expect("forward");
            let mj = floats(&state["sensordata"]);
            readings += mj.len();
            let ours = data.sensordata.as_slice();
            let matches = if name == "mesh" {
                matches_slab(ours, &mj)
            } else {
                matches_golden(ours, &mj)
            };
            if !matches {
                failures.push(format!("{name} state {k}: {ours:?}, MuJoCo {mj:?}"));
            }
        }
    }
    assert_eq!(
        readings,
        240 + 2 + 3 + 80 + 1 + 80 + 80,
        "every reading compared"
    );
    assert!(
        failures.is_empty(),
        "{} differ:\n{}",
        failures.len(),
        failures.join("\n")
    );
}

/// Registry D-SDF-RAY: MuJoCo's ray meets a signed distance field by
/// marching its plugin's field (`mj_raySdf`, `engine_ray.c:885-948`); a
/// field in sim-core is a shape of its own, and a rangefinder casts the ray
/// of the shape's grid (`raycast::raycast_geom`), an analytic shape's too,
/// as this sphere is. A sphere field of radius 1
/// centred 3 above the rangefinder, sampled on a grid of cells 8/39 wide that
/// holds the rangefinder: unturned, it reads 2 within a cell; turned away, −1.
/// The same field on a grid of cells 3/19 wide that ends 1.5 short of the
/// rangefinder reads 2 within a cell too, the shape's ray tracing from where it
/// enters the grid (ledger L282).
#[test]
fn a_rangefinder_casts_a_signed_distance_field_through_its_shape() {
    let read = |padding: f64, resolution: usize, quat: [f64; 4]| {
        let mut model = sim_mjcf::load_model(
            r#"<mujoco><option gravity="0 0 0"/><worldbody>
  <geom type="sphere" size="1" pos="0 0 3"/>
  <body name="eye"><freejoint/><geom type="sphere" size="0.01" contype="0" conaffinity="0"/>
    <site name="r"/>
  </body>
</worldbody><sensor><rangefinder site="r"/></sensor></mujoco>"#,
        )
        .expect("load");
        let grid = Arc::new(SdfGrid::sphere(Point3::origin(), 1.0, resolution, padding));
        model.shape_data.push(Arc::new(ShapeSphere::new(grid, 1.0)));
        model.nshape = 1;
        model.geom_type[0] = GeomType::Sdf;
        model.geom_shape[0] = Some(0);
        let mut data = model.make_data();
        data.qpos
            .as_mut_slice()
            .copy_from_slice(&[0.0, 0.0, 0.0, quat[0], quat[1], quat[2], quat[3]]);
        data.forward(&model).expect("forward");
        data.sensordata[0]
    };
    let (unturned, turned_away) = ([1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]);
    let up = read(3.0, 40, unturned);
    assert!((up - 2.0).abs() <= 8.0 / 39.0, "up: {up}");
    assert_eq!(
        read(3.0, 40, turned_away).to_bits(),
        (-1.0_f64).to_bits(),
        "turned away"
    );
    let outside = read(0.5, 20, unturned);
    assert!(
        (outside - 2.0).abs() <= 3.0 / 19.0,
        "outside the grid: {outside}"
    );
}

/// A plane bounds a ray by its size where the size is positive, as MuJoCo
/// 3.5.0's `ray_plane` does (`engine_ray.c:204-230`): a rangefinder 1 above a
/// plane of size 1 × 1, looking down, reads 1 inside it and −1 past its x
/// edge, and 1 past it where that half-size is 0 (measured on the S21 oracle).
/// Arithmetic only, so bit for bit everywhere.
#[test]
fn a_rangefinder_misses_past_a_bounded_planes_edge_as_mujoco() {
    for (x, size, mujoco) in [
        ("0.5", "1 1 0.1", 1.0),
        ("1.5", "1 1 0.1", -1.0),
        ("1.5", "0 0 0.1", 1.0),
        ("1.5", "0 1 0.1", 1.0),
    ] {
        let model = sim_mjcf::load_model(&format!(
            r#"<mujoco><worldbody><geom type="plane" size="{size}"/><body name="b" pos="{x} 0 1"><joint type="slide" axis="0 0 1"/><geom type="sphere" size="0.01" contype="0" conaffinity="0"/><site name="s" zaxis="0 0 -1"/></body></worldbody><sensor><rangefinder site="s"/></sensor></mujoco>"#
        ))
        .expect("load");
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        assert_eq!(
            data.sensordata[0].to_bits(),
            f64::to_bits(mujoco),
            "x {x}, plane size {size}: {}",
            data.sensordata[0]
        );
    }
}
