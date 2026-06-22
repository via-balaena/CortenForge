//! Shared fixtures for the perf/scale baseline benchmarks.
//!
//! The criterion harness (`benches/perf_baseline.rs`) and the scaling probe
//! (`examples/probe.rs`) both need the *same* GPU-compatible model, so it lives
//! here as one source rather than being duplicated in each. See
//! `PERF_BASELINE.md` for the recorded results + methodology.

// Benchmark support code: env indices widen to `f64` for distinct initial
// states (precision loss is irrelevant), and the smoke test `expect`s on a
// broken fixture (failing loudly is the honest behavior for a bench helper).
#![allow(clippy::cast_precision_loss, clippy::expect_used)]

use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, Vector3};
use sim_core::sdf::PhysicsShape;
use sim_core::types::GeomType;
use sim_core::{Data, Model, SdfGrid, ShapeConvex};

/// Batch sizes swept for every configuration.
pub const N_ENVS: &[usize] = &[1, 16, 64, 256, 1024, 4096];

/// Physics steps per measured iteration (one full step = one GPU substep).
pub const STEPS: u32 = 100;

/// The shared benchmark model: a free body (nv = 6) + ground plane + SDF sphere.
///
/// Free joints only, so BOTH the CPU and GPU engines accept it. The geom is
/// kept present deliberately: a geom-less model is unrepresentative of real RL
/// envs (and historically mis-sized an empty GPU buffer — see `PERF_BASELINE.md`).
#[must_use]
pub fn bench_model() -> Model {
    let mut model = Model::free_body(1.0, Vector3::new(0.1, 0.2, 0.3));
    model.add_ground_plane();
    add_sdf_sphere_geom(&mut model, 1, 5.0, 16);
    model
}

/// Build `n` environments with slightly distinct initial states.
///
/// They diverge as a real batch would. `drop_z` selects the fixture: far above
/// the plane = free fall (no contact); just above = sustained contact.
#[must_use]
pub fn make_datas(model: &Model, n: usize, drop_z: f64) -> Vec<Data> {
    (0..n)
        .map(|i| {
            let mut d = model.make_data();
            d.qpos[0] = (i as f64) * 0.01; // distinct x per env
            d.qpos[2] = drop_z;
            d.qpos[3] = 1.0; // identity quaternion (w component)
            d
        })
        .collect()
}

/// Attach an SDF-sphere geom to `body_id` (mirrors the GPU pipeline test fixture).
pub fn add_sdf_sphere_geom(model: &mut Model, body_id: usize, radius: f64, grid_res: usize) {
    let grid = SdfGrid::sphere(Point3::origin(), radius, grid_res, 1.0);
    let shape: Arc<dyn PhysicsShape> = Arc::new(ShapeConvex::new(Arc::new(grid)));
    let shape_id = model.shape_data.len();
    model.shape_data.push(shape);
    model.nshape = model.shape_data.len();

    let geom_id = model.ngeom;
    model.ngeom += 1;
    let half = radius + 1.0;

    model.geom_type.push(GeomType::Sdf);
    model.geom_body.push(body_id);
    model.geom_pos.push(Vector3::zeros());
    model.geom_quat.push(UnitQuaternion::identity());
    model.geom_size.push(Vector3::new(half, half, half));
    model.geom_friction.push(Vector3::new(1.0, 0.005, 0.0001));
    model.geom_condim.push(3);
    model.geom_contype.push(1);
    model.geom_conaffinity.push(1);
    model.geom_margin.push(1.0);
    model.geom_gap.push(0.0);
    model.geom_priority.push(0);
    model.geom_solmix.push(1.0);
    model.geom_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]);
    model.geom_solref.push([0.02, 1.0]);
    model.geom_fluid.push([0.0; 12]);
    model.geom_name.push(Some(format!("sdf_sphere_{geom_id}")));
    model.geom_rbound.push(radius);
    model.geom_aabb.push([0.0, 0.0, 0.0, half, half, half]);
    model.geom_mesh.push(None);
    model.geom_hfield.push(None);
    model.geom_shape.push(Some(shape_id));
    model.geom_group.push(0);
    model.geom_rgba.push([0.7, 0.3, 0.3, 1.0]);
    model.geom_user.push(vec![]);
    model.geom_plugin.push(None);

    if model.body_geom_num[body_id] == 0 {
        model.body_geom_adr[body_id] = geom_id;
    }
    model.body_geom_num[body_id] += 1;
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Smoke: the shared fixture builds a valid nv=6 model and steps once on the
    /// CPU without error. Guards the fixture (and gives the bench crate a
    /// lib-level coverage surface) independent of any GPU.
    #[test]
    fn bench_model_builds_and_steps() {
        let model = bench_model();
        assert_eq!(model.nv, 6, "free body should have 6 DOF");
        // Ground plane + SDF sphere.
        assert_eq!(model.ngeom, 2, "fixture should carry plane + sphere geoms");

        let mut datas = make_datas(&model, 4, 20.0);
        assert_eq!(datas.len(), 4);
        // Distinct initial x per env (so a batch genuinely diverges).
        assert!((datas[0].qpos[0] - datas[3].qpos[0]).abs() > 0.0);

        datas[0].step(&model).expect("cpu step on bench fixture");
        assert!(datas[0].qpos[2].is_finite());
    }
}
