//! G1 scorecard — grade the placed, articulated knee on the synthetic reference
//! leg against the OpenSim oracle + the scan envelope.
//!
//!   cargo run -p cf-msk-fit --example g1_scorecard
//!
//! Three checks (per the G1 plan), each scoped honestly:
//!   [A] Moment-arm REGRESSION: de-scaled placed model vs real OpenSim 4.6.
//!       Placement is a similarity, so this de-scales to the cf-osim model and
//!       equals the S1 cf-osim↔OpenSim agreement (~0.3 mm) — it re-confirms that
//!       through the pose() path; it does NOT independently grade the placement.
//!   [B] Joint center pinned to the detected landmark (STRUCTURAL — 0 by
//!       construction; confirms the anchor wiring, cannot "fail").
//!   [C] Skeleton (bone) containment inside the skin envelope at the scan pose —
//!       a real geometric check that can fail.

use cf_anthro::detect_landmarks;
use cf_anthro::synthetic::LegSpec;
use cf_msk_fit::Fitter;
use cf_msk_fit::scorecard::{agreement, placed_length_m, placed_moment_arm_m};
use mesh_sdf::{WALL_THRESHOLD_FACTOR_DEFAULT, flood_filled_sdf};
use mesh_types::{Aabb, IndexedMesh, Point3, Vector3};

const MUSCLES: [&str; 4] = ["rect_fem_r", "vas_int_r", "bifemlh_r", "semimem_r"];
const EPS: f64 = 1e-3; // finite-difference step (rad)
const BONE_RADIUS: f64 = 0.010; // matches the bones-in-scan tube radius

fn aabb_of(m: &IndexedMesh) -> (Point3<f64>, Point3<f64>) {
    let mut lo = Point3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut hi = Point3::new(f64::MIN, f64::MIN, f64::MIN);
    for v in &m.vertices {
        lo = Point3::new(lo.x.min(v.x), lo.y.min(v.y), lo.z.min(v.z));
        hi = Point3::new(hi.x.max(v.x), hi.y.max(v.y), hi.z.max(v.z));
    }
    (lo, hi)
}

fn yn(b: bool) -> &'static str {
    if b { "PASS" } else { "FAIL" }
}

fn main() {
    let (leg, _gt) = LegSpec::default_leg().build(220, 96);
    let lm = detect_landmarks(&leg).expect("detect landmarks");
    let assets = format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392",
        env!("CARGO_MANIFEST_DIR")
    );
    let sub = cf_osim::osim::parse_knee_subgraph(
        &std::fs::read_to_string(format!("{assets}/gait2392.osim")).unwrap(),
    );
    let fit = Fitter::new(&sub, &lm);
    let reference: serde_json::Value = serde_json::from_str(
        &std::fs::read_to_string(format!("{assets}/knee_moment_arms_opensim.json")).unwrap(),
    )
    .unwrap();

    println!("=== G1 SCORECARD — placed/articulated knee on the synthetic reference leg ===");
    println!("model→scan scale = {:.4}\n", fit.scale());

    // [A] Moment-arm regression: equals the S1 cf-osim↔OpenSim agreement
    // (placement de-scales out) re-derived through the pose() path.
    println!("[A] Moment-arm regression — de-scaled placed model vs OpenSim 4.6 (0→-100°)");
    println!(
        "  {:<11} {:>9} {:>9} {:>7} {:>10}",
        "muscle", "MA RMSE", "MA max|Δ|", "corr", "len RMSE"
    );
    let mut worst_rmse = 0.0_f64;
    let mut worst_corr = 1.0_f64;
    for m in MUSCLES {
        let rows = reference["muscles"][m].as_array().unwrap();
        let (mut ours_ma, mut ref_ma) = (Vec::new(), Vec::new());
        let (mut ours_len, mut ref_len) = (Vec::new(), Vec::new());
        for row in rows {
            let theta = row["angle_rad"].as_f64().unwrap();
            ref_ma.push(row["moment_arm_m"].as_f64().unwrap());
            ref_len.push(row["length_m"].as_f64().unwrap());
            ours_ma.push(placed_moment_arm_m(&fit, m, theta, EPS));
            ours_len.push(placed_length_m(&fit, m, theta));
        }
        let ma = agreement(&ours_ma, &ref_ma).unwrap();
        let len = agreement(&ours_len, &ref_len).unwrap();
        worst_rmse = worst_rmse.max(ma.rmse_mm);
        worst_corr = worst_corr.min(ma.corr);
        println!(
            "  {:<11} {:>7.3}mm {:>7.3}mm {:>7.4} {:>8.3}mm",
            m, ma.rmse_mm, ma.max_abs_mm, ma.corr, len.rmse_mm
        );
    }
    let a_pass = worst_rmse < 5.0 && worst_corr >= 0.95;
    println!(
        "  → worst MA RMSE {worst_rmse:.3}mm, worst corr {worst_corr:.4}: {} (loose regression",
        yn(a_pass)
    );
    println!("    bounds ≤5mm/≥0.95; the ~0.3mm fidelity is the S1 result, re-confirmed here)");

    // [B] Joint-center anchor — pinned to the detected landmark by construction
    // (knee == kp == lm.knee_point), so this is 0 by wiring, not a measured fit.
    let p0 = fit.pose(0.0);
    let jc_mm = (p0.knee - lm.knee_point).norm() * 1000.0;
    let b_pass = jc_mm < 5.0;
    println!(
        "\n[B] Joint center pinned to detected landmark: {jc_mm:.3}mm (structural) → {}",
        yn(b_pass)
    );

    // [C] Bone containment at the scan (extension) pose.
    let (lo, hi) = aabb_of(&leg);
    let pad = Vector3::new(0.02, 0.02, 0.02);
    let (sdf, _report) = flood_filled_sdf(
        leg,
        Aabb::new(lo - pad, hi + pad),
        0.005,
        WALL_THRESHOLD_FACTOR_DEFAULT,
    )
    .unwrap();
    let mut worst_depth = f64::NEG_INFINITY; // signed distance; >0 is outside
    for bone in [&p0.femur, &p0.tibia] {
        // Sample t∈(0.1..0.9): skip the hip/ankle/knee endpoints, which sit ON
        // the segment boundaries (the centerline reaches the skin there) and
        // aren't a containment failure.
        for k in 1..10 {
            let t = f64::from(k) / 10.0;
            let pt = bone.proximal + (bone.distal - bone.proximal) * t;
            worst_depth = worst_depth.max(sdf.evaluate(pt));
        }
    }
    let surface_inside_mm = (-worst_depth - BONE_RADIUS) * 1000.0;
    let c_pass = -worst_depth > BONE_RADIUS;
    println!("\n[C] Skeleton containment at extension (femur+tibia centerlines vs skin SDF)");
    println!(
        "  worst centerline {:.0}mm inside; r {:.0}mm → tube surface {:.0}mm inside → {}",
        -worst_depth * 1000.0,
        BONE_RADIUS * 1000.0,
        surface_inside_mm,
        yn(c_pass)
    );
    println!("  (muscle-in-envelope + through-ROM containment needs a pose-matched scan — v2)");

    println!(
        "\n=== G1 GATE (synthetic reference leg): [A] {} · [B] {} · [C] {} ===",
        yn(a_pass),
        yn(b_pass),
        yn(c_pass)
    );
    println!("    [A] regression (≡ S1) · [B] structural · [C] real geometric check");
}
