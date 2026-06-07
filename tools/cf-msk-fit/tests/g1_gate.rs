//! G1 gate — the formal validation tolerances for the scan→placed→articulated
//! knee, asserted against the OpenSim oracle. Mirrors `examples/g1_scorecard`.

use cf_anthro::detect_landmarks;
use cf_anthro::synthetic::LegSpec;
use cf_msk_fit::Fitter;
use cf_msk_fit::scorecard::{agreement, placed_moment_arm_m};
use mesh_sdf::{WALL_THRESHOLD_FACTOR_DEFAULT, flood_filled_sdf};
use mesh_types::{Aabb, IndexedMesh, Point3, Vector3};

const MUSCLES: [&str; 4] = ["rect_fem_r", "vas_int_r", "bifemlh_r", "semimem_r"];
const EPS: f64 = 1e-3;
const BONE_RADIUS: f64 = 0.010;

fn assets() -> String {
    format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392",
        env!("CARGO_MANIFEST_DIR")
    )
}

fn aabb_of(m: &IndexedMesh) -> (Point3<f64>, Point3<f64>) {
    let mut lo = Point3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut hi = Point3::new(f64::MIN, f64::MIN, f64::MIN);
    for v in &m.vertices {
        lo = Point3::new(lo.x.min(v.x), lo.y.min(v.y), lo.z.min(v.z));
        hi = Point3::new(hi.x.max(v.x), hi.y.max(v.y), hi.z.max(v.z));
    }
    (lo, hi)
}

#[test]
fn g1_gate_moment_arms_joint_center_and_containment() {
    let (leg, _gt) = LegSpec::default_leg().build(220, 96);
    let lm = detect_landmarks(&leg).expect("detect landmarks");
    let sub = cf_osim::osim::parse_knee_subgraph(
        &std::fs::read_to_string(format!("{}/gait2392.osim", assets())).unwrap(),
    );
    let fit = Fitter::new(&sub, &lm);
    let reference: serde_json::Value = serde_json::from_str(
        &std::fs::read_to_string(format!("{}/knee_moment_arms_opensim.json", assets())).unwrap(),
    )
    .unwrap();

    // [A] Moment arms: de-scaled placed model vs real OpenSim. Placement is a
    // similarity, so this de-scales to the cf-osim model and reproduces the S1
    // agreement (~0.3mm / ~1.0) through the pose() path — a loose REGRESSION
    // bound (RMSE<5mm, corr>=0.95), not an independent fidelity measurement.
    for m in MUSCLES {
        let rows = reference["muscles"][m].as_array().unwrap();
        let mut ours = Vec::new();
        let mut refs = Vec::new();
        for row in rows {
            let theta = row["angle_rad"].as_f64().unwrap();
            refs.push(row["moment_arm_m"].as_f64().unwrap());
            ours.push(placed_moment_arm_m(&fit, m, theta, EPS));
        }
        let a = agreement(&ours, &refs).expect("agreement");
        assert!(
            a.rmse_mm < 5.0,
            "{m}: moment-arm RMSE {:.3}mm exceeds 5mm",
            a.rmse_mm
        );
        assert!(
            a.corr >= 0.95,
            "{m}: moment-arm corr {:.4} below 0.95",
            a.corr
        );
    }

    // [B] Joint center pinned to the detected landmark — 0 by construction
    // (knee == kp == lm.knee_point); this asserts the anchor wiring, not a fit.
    let p0 = fit.pose(0.0);
    let jc_mm = (p0.knee - lm.knee_point).norm() * 1000.0;
    assert!(jc_mm < 5.0, "joint center {jc_mm:.3}mm off the landmark");

    // [C] Bones (femur+tibia) inside the skin envelope at the scan pose: each
    // centerline sample deeper than the tube radius. t∈(0.1..0.9) skips the
    // hip/ankle/knee endpoints (on the segment boundaries, not a failure).
    let (lo, hi) = aabb_of(&leg);
    let pad = Vector3::new(0.02, 0.02, 0.02);
    let (sdf, _r) = flood_filled_sdf(
        leg,
        Aabb::new(lo - pad, hi + pad),
        0.005,
        WALL_THRESHOLD_FACTOR_DEFAULT,
    )
    .unwrap();
    for bone in [&p0.femur, &p0.tibia] {
        for k in 1..10 {
            let t = f64::from(k) / 10.0;
            let pt = bone.proximal + (bone.distal - bone.proximal) * t;
            let d = sdf.evaluate(pt);
            assert!(
                d < -BONE_RADIUS,
                "bone centerline at t={t} only {:.1}mm inside",
                -d * 1000.0
            );
        }
    }
}
