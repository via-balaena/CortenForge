//! A site's rangefinder against MuJoCo 3.5.0's, which casts `mj_ray` from the
//! site along its z axis (`engine_sensor.c:468-482`) over the geoms its body
//! does not hold and its alpha does not hide (`ray_eliminate`,
//! `engine_ray.c:68-99`), a primitive by `mju_rayGeom`, and clips the reading
//! to ±cutoff (the rangefinder's datatype is real, `user_objects.cc:7209-7217`).
//!
//! The golden is `assets/golden/rangefinder/rangefinder.json`, from
//! `scripts/gen_rangefinder_reference.py` on the unfused oracle: a rangefinder
//! among five primitives and an unbounded plane at 240 states (hits and
//! misses), rays through a geom whose alpha is 0 and through a geom of the
//! sensor's own body, a ray stopped by a geom whose alpha is 0.25, a hit and a
//! miss under a cutoff of 0.5, a hit at 150 with no cutoff, and a rangefinder
//! over a convex mesh at 80 states. In the primitives' cases every turn comes
//! from the states' qpos, none from the compiler's orientations; the mesh's
//! geom MuJoCo turns to the mesh's principal frame, which sim-mjcf does not
//! (Rigid-loading's L35). The primitives' readings are compared bit for bit
//! on macOS arm64, where the golden was made; elsewhere within `1e-13 ·
//! max(1, |MuJoCo's|)`, a NaN failing. The mesh's readings hit where
//! MuJoCo's hit, each within `1e-8 · max(1, |MuJoCo's|)` (measured: 4.4e-9
//! at most); a mesh's ray is not MuJoCo's operation for operation (ledger
//! L267), and how much of the difference is that, the frame, or MuJoCo's
//! single-precision vertices is not isolated.

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

/// A mesh's readings: a hit where MuJoCo's is one, each within `1e-8 ·
/// max(1, |MuJoCo's|)`, a NaN failing.
fn matches_mesh(ours: &[f64], mj: &[f64]) -> bool {
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
                matches_mesh(ours, &mj)
            } else {
                matches_golden(ours, &mj)
            };
            if !matches {
                failures.push(format!("{name} state {k}: {ours:?}, MuJoCo {mj:?}"));
            }
        }
    }
    assert_eq!(readings, 240 + 2 + 3 + 80 + 1, "every reading compared");
    assert!(
        failures.is_empty(),
        "{} differ:\n{}",
        failures.len(),
        failures.join("\n")
    );
}
