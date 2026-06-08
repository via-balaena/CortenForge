//! A1 checkpoint — the **general kinematic-tree IR** reproduces the validated
//! knee oracle to machine zero.
//!
//! This is the productionized, CI-enforced form of the throwaway A1 spike
//! (`cf-osim`'s `spike_general_fk.rs`): it builds the source-agnostic
//! [`cf_msk_lib::Model`] from the knee subgraph (`Model::from_subgraph` — pelvis
//! → femur[hip welded at neutral] → tibia[knee coupled]) and grades its
//! transform-axis FK against `cf_osim::oracle::Kinematics` (`Variant::TRUTH`) for
//! all four muscles across the 0→100° flexion sweep.
//!
//! The general FK is the *same math* as the bespoke `Kinematics::femur()/tibia()`,
//! routed through a generic tree walker + transform-axis composition — so they
//! must agree to ~machine epsilon. A nonzero residual here means the IR
//! generalization diverged from the validated knee path, which gates the whole
//! leg-region arc.

use cf_msk_lib::Model;
use cf_osim::oracle::{Kinematics, Variant};
use cf_osim::osim::{Muscle, Subgraph, parse_knee_subgraph};
use nalgebra::Point3;
use std::collections::HashMap;
use std::f64::consts::PI;

const DEG: f64 = PI / 180.0;

fn template() -> Subgraph {
    let path = format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392/gait2392.osim",
        env!("CARGO_MANIFEST_DIR")
    );
    parse_knee_subgraph(&std::fs::read_to_string(path).expect("read gait2392.osim"))
}

/// Muscle path length at knee angle `theta` via the general IR FK: sum of the
/// straight segments between the muscle's active path points, placed in the world
/// by `Model::body_pose`. Uses `Variant::TRUTH` activity/location (nothing frozen),
/// matching the oracle.
fn ir_path_length(model: &Model, m: &Muscle, theta: f64) -> f64 {
    let q = HashMap::from([("knee_angle_r".to_string(), theta)]);
    let pts: Vec<Point3<f64>> = m
        .path
        .iter()
        .filter(|p| p.active(theta))
        .map(|p| model.body_pose(&p.body, &q) * Point3::from(p.location_at(theta, false)))
        .collect();
    pts.windows(2).map(|w| (w[1] - w[0]).norm()).sum()
}

fn ir_moment_arm(model: &Model, m: &Muscle, theta: f64, eps: f64) -> f64 {
    -(ir_path_length(model, m, theta + eps) - ir_path_length(model, m, theta - eps)) / (2.0 * eps)
}

#[test]
fn general_ir_fk_reproduces_knee_oracle_to_machine_zero() {
    let sub = template();
    let oracle = Kinematics::new(&sub);
    let model = Model::from_subgraph(&sub);

    let angles: Vec<f64> = (0..=20).map(|i| -(i as f64) * 5.0 * DEG).collect();
    let eps = 0.5 * DEG;

    let mut worst = 0.0_f64;
    for m in &sub.muscles {
        for &th in &angles {
            let ours = ir_moment_arm(&model, m, th, eps);
            let truth = oracle.moment_arm(m, th, eps, Variant::TRUTH);
            let dma_mm = (ours - truth).abs() * 1000.0;
            worst = worst.max(dma_mm);
            // Same math, routed through the generic IR — must agree to ~machine eps.
            assert!(
                dma_mm < 1e-9,
                "{}: general IR moment arm diverges from the oracle by {dma_mm:.2e} mm at {:.0}°",
                m.name,
                -th / DEG
            );
        }
    }
    assert!(worst < 1e-9, "worst |Δ moment arm| = {worst:.2e} mm");
}

/// The bridge wires the chain the spike validated: pelvis (root) → femur_r (hip
/// welded, just the offset) → tibia_r (knee: 1 rotation + 3 coupled translations).
#[test]
fn from_subgraph_builds_the_knee_chain() {
    let model = Model::from_subgraph(&template());
    assert_eq!(
        model
            .bodies
            .iter()
            .map(|b| b.name.as_str())
            .collect::<Vec<_>>(),
        ["pelvis", "femur_r", "tibia_r"]
    );
    assert_eq!(model.bodies[0].parent, None);
    assert_eq!(model.bodies[1].parent, Some(0));
    assert_eq!(model.bodies[2].parent, Some(1));
    assert!(model.bodies[1].joint.is_empty(), "hip is welded at neutral");
    // tibia: one rotation (the flexion hinge) + three translation axes (tx/ty/tz).
    let tibia = &model.bodies[2].joint;
    assert_eq!(tibia.iter().filter(|a| a.rotation).count(), 1);
    assert_eq!(tibia.iter().filter(|a| !a.rotation).count(), 3);
    assert_eq!(model.coordinates.len(), 1);
    assert_eq!(model.coordinates[0].name, "knee_angle_r");
}
