//! cf-msk-fit — place the validated OpenSim knee onto a scan's landmarks.
//!
//! S3 of the musculoskeletal-builder arc: the bridge where the model (cf-osim's
//! gait2392 knee, validated against real OpenSim) meets the scan (cf-anthro's
//! detected knee point, limb axis, and segment lengths). This v1 uses a single
//! **similarity transform** (rotate OpenSim → scan frame, scale, translate) that
//! pins the knee joint center to the detected knee and lines the femur up the
//! thigh; it produces the femur/tibia bones and the muscle/tendon paths in the
//! scan frame, ready to render inside the scan.
//!
//! Approximations (v1, called out honestly):
//! - A SINGLE scale (thigh / OpenSim-femur length) is applied to the whole
//!   model, so the shank muscles inherit the femur scale. A proper per-segment
//!   anisotropic scale (the recon's `ScaleSpec`) is the S3 follow-up.
//! - The transform is built at full knee extension (θ=0).
//! - Medio-lateral / antero-posterior orientation follows gait2392's frame
//!   convention; if a real scan's M-L axis differs, it gets re-pinned at v2.

use cf_anthro::Landmarks;
use cf_osim::oracle::{Kinematics, Variant};
use cf_osim::osim::Subgraph;
use nalgebra::{Matrix3, Point3, Vector3};

/// A bone as a segment from its proximal to distal end (scan frame, meters).
#[derive(Debug, Clone, Copy)]
pub struct Bone {
    pub proximal: Point3<f64>,
    pub distal: Point3<f64>,
}

/// A muscle/tendon path placed in the scan frame.
#[derive(Debug, Clone)]
pub struct PlacedMuscle {
    pub name: String,
    pub polyline: Vec<Point3<f64>>,
}

/// The OpenSim knee placed in the scan frame.
#[derive(Debug, Clone)]
pub struct Placement {
    pub knee: Point3<f64>,
    /// Femur: knee (distal) → hip (proximal), scaled to the thigh length.
    pub femur: Bone,
    /// Tibia: knee (proximal) → ankle (distal), scaled to the shank length.
    pub tibia: Bone,
    pub muscles: Vec<PlacedMuscle>,
    /// The uniform scale applied (thigh length / OpenSim femur length).
    pub scale: f64,
}

/// Place the OpenSim knee subgraph onto the detected landmarks.
pub fn place_knee(sub: &Subgraph, lm: &Landmarks) -> Placement {
    let kin = Kinematics::new(sub);
    let v = Variant::TRUTH;
    let th = 0.0; // full extension
    let world = |body: &str, loc: Vector3<f64>| kin.body_pose(body, th, v) * Point3::from(loc);

    // Key points in OpenSim world coordinates.
    let knee_osim = world("tibia_r", Vector3::zeros());
    let hip_osim = world("femur_r", Vector3::zeros());
    let femur_vec = hip_osim - knee_osim;
    let femur_len = femur_vec.norm();
    debug_assert!(
        femur_len > 1e-6 && lm.thigh_length_m > 1e-6,
        "degenerate femur ({femur_len}) or thigh length ({}) — can't place",
        lm.thigh_length_m
    );
    let scale = lm.thigh_length_m / femur_len;

    // Rotation: femur axis → scan +z (proximal); gait2392 lateral (+z) → scan +x.
    let w = femur_vec / femur_len;
    let lat = Vector3::z();
    let proj = lat - lat.dot(&w) * w;
    let x_dir = if proj.norm() < 1e-9 {
        Vector3::x()
    } else {
        proj.normalize()
    };
    let y_dir = w.cross(&x_dir);
    let r = Matrix3::new(
        x_dir.x, x_dir.y, x_dir.z, // row 0 → scan x
        y_dir.x, y_dir.y, y_dir.z, // row 1 → scan y
        w.x, w.y, w.z, // row 2 → scan z (femur axis)
    );

    let kp = lm.knee_point;
    let xform = |q: Point3<f64>| kp + scale * (r * (q - knee_osim));

    let zc = Vector3::z();
    let femur = Bone {
        proximal: kp + lm.thigh_length_m * zc,
        distal: kp,
    };
    let tibia = Bone {
        proximal: kp,
        distal: kp - lm.shank_length_m * zc,
    };

    let muscles = sub
        .muscles
        .iter()
        .map(|m| PlacedMuscle {
            name: m.name.clone(),
            polyline: m
                .path
                .iter()
                .filter(|pp| pp.active(th))
                .map(|pp| xform(world(&pp.body, pp.location_at(th, false))))
                .collect(),
        })
        .collect();

    Placement {
        knee: kp,
        femur,
        tibia,
        muscles,
        scale,
    }
}
