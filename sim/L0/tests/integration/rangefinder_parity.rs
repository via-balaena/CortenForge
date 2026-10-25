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
//! Every turn comes from the states' qpos, none from the compiler's
//! orientations; the height fields' elevations are ones sim-mjcf loads as
//! MuJoCo compiles them, which it does not for every field until book
//! Rigid-loading L45. Every reading is compared bit for bit on macOS arm64,
//! where the golden was made, elsewhere within `1e-13 · max(1, |MuJoCo's|)`,
//! a NaN failing; but the slab's. MuJoCo turns the slab to its principal
//! frame, which sim-mjcf does not (book Rigid-loading L35): its readings hit
//! where MuJoCo's hit, each within `1e-8 · max(1, |MuJoCo's|)` (measured: 18
//! of the 80 differ, by 4.4e-9 at most). Given MuJoCo's compiled vertices and
//! pose, the same slab's rays are MuJoCo's bit for bit (sim-core's
//! `rays_meet_each_mesh_as_mujoco`).

use serde_json::Value;

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

/// The slab's readings: a hit where MuJoCo's is one, each within `1e-8 ·
/// max(1, |MuJoCo's|)`, a NaN failing.
fn matches_slab(ours: &[f64], mj: &[f64]) -> bool {
    ours.len() == mj.len()
        && ours
            .iter()
            .zip(mj)
            .all(|(a, b)| (*a >= 0.0) == (*b >= 0.0) && (a - b).abs() <= 1e-8 * b.abs().max(1.0))
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
