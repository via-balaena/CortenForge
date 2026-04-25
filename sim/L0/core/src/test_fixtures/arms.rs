//! Multi-link reaching arms used by the chassis stock-task suites.
//!
//! Mirrors `MJCF_2DOF` / `MJCF_6DOF` / `MJCF_6DOF_OBSTACLE` in
//! `sim-rl/src/tasks.rs` and the same constants duplicated in
//! `sim-ml-chassis/src/test_stock_tasks.rs`. These fixtures serve the
//! chassis-side `#[cfg(test)]` suites; sim-rl's own copies stay on
//! sim-mjcf because sim-rl is L0-integration tier (sim-mjcf allowed
//! there per plan §2.1).
//!
//! All three fixtures use:
//! - `compiler angle="radian" inertiafromgeom="true"`
//! - `option gravity="0 0 -9.81" timestep="0.002" integrator="RK4"`
//! - `flag contact="disable"` (geom contype=0 / conaffinity=0)
//!
//! Joint damping values (2.0, 1.5, 1.0, 0.5) are preserved from MJCF;
//! capsule masses + thin-rod inertia are computed from the segment
//! length using the same `m·L²/12` approximation as the existing
//! `Model::n_link_pendulum` factory.

use nalgebra::Vector3;

use super::builders::{
    add_body, add_capsule_geom, add_hinge_joint, add_motor, add_site, add_sphere_geom, finalize,
    set_options,
};
use crate::types::Model;
use crate::types::enums::Integrator;

const PI: f64 = std::f64::consts::PI;

/// Compute thin-rod inertia for a segment of length `L` and mass `m`,
/// COM at the segment's midpoint along the local X axis.
fn segment_inertia(mass: f64, length: f64) -> Vector3<f64> {
    let i_transverse = mass * length * length / 12.0;
    let i_axial = 0.01 * i_transverse;
    Vector3::new(i_axial, i_transverse, i_transverse)
}

/// 2-link planar reaching arm with a fingertip site. Body order:
/// world(0), `upper_arm`(1), `forearm`(2). Mirrors `MJCF_2DOF`.
///
/// Shape: nbody=3, njnt=2, nq=2, nv=2, nu=2, ngeom=2, nsite=1.
#[must_use]
pub fn reaching_2dof() -> Model {
    let mut model = Model::empty();
    set_options(
        &mut model,
        0.002,
        Vector3::new(0.0, 0.0, -9.81),
        Integrator::RungeKutta4,
        true,
    );

    // upper_arm: capsule from (0,0,0) to (0.5,0,0), mass 0.5
    let upper_len = 0.5_f64;
    let upper_mass = 0.5_f64;
    let upper = add_body(
        &mut model,
        0,
        "upper_arm",
        Vector3::zeros(),
        upper_mass,
        segment_inertia(upper_mass, upper_len),
        Vector3::new(upper_len * 0.5, 0.0, 0.0),
    );
    let shoulder = add_hinge_joint(
        &mut model,
        upper,
        "shoulder",
        Vector3::new(0.0, -1.0, 0.0),
        0.0,
        2.0,
        0.0,
        true,
        (-PI, PI),
    );
    add_capsule_geom(
        &mut model,
        upper,
        Some("upper_geom"),
        Vector3::zeros(),
        Vector3::new(upper_len, 0.0, 0.0),
        0.03,
        false,
    );

    // forearm: capsule from (0,0,0) to (0.4,0,0), mass 0.3
    let fore_len = 0.4_f64;
    let fore_mass = 0.3_f64;
    let forearm = add_body(
        &mut model,
        upper,
        "forearm",
        Vector3::new(upper_len, 0.0, 0.0),
        fore_mass,
        segment_inertia(fore_mass, fore_len),
        Vector3::new(fore_len * 0.5, 0.0, 0.0),
    );
    let elbow = add_hinge_joint(
        &mut model,
        forearm,
        "elbow",
        Vector3::new(0.0, -1.0, 0.0),
        0.0,
        1.0,
        0.0,
        true,
        (-2.6, 2.6),
    );
    add_capsule_geom(
        &mut model,
        forearm,
        Some("forearm_geom"),
        Vector3::zeros(),
        Vector3::new(fore_len, 0.0, 0.0),
        0.025,
        false,
    );
    add_site(
        &mut model,
        forearm,
        "fingertip",
        Vector3::new(fore_len, 0.0, 0.0),
        0.015,
    );

    add_motor(
        &mut model,
        shoulder,
        Some("shoulder_motor"),
        10.0,
        Some((-1.0, 1.0)),
    );
    add_motor(
        &mut model,
        elbow,
        Some("elbow_motor"),
        5.0,
        Some((-1.0, 1.0)),
    );

    finalize(&mut model);
    model
}

/// Three-segment 6-DOF reaching arm with alternating pitch/yaw hinges
/// and a fingertip site on the distal segment. Mirrors `MJCF_6DOF`.
///
/// Body order: world(0), seg1(1), seg2(2), seg3(3). Joint order:
/// j1 (pitch on seg1), j2 (yaw on seg1), j3 (pitch on seg2),
/// j4 (yaw on seg2), j5 (pitch on seg3), j6 (yaw on seg3). Motor gears
/// are 10/8/6/5/4/3 in joint order.
///
/// Shape: nbody=4, njnt=6, nq=6, nv=6, nu=6, ngeom=3, nsite=1.
#[must_use]
pub fn reaching_6dof() -> Model {
    let mut model = Model::empty();
    build_6dof_arm(&mut model, /* with_obstacle */ false);
    finalize(&mut model);
    model
}

/// 6-DOF reaching arm + a static obstacle body and a target site on
/// worldbody. Mirrors `MJCF_6DOF_OBSTACLE`.
///
/// Body order: world(0), seg1(1), seg2(2), seg3(3), obstacle(4).
/// Site order: target(0, on world), fingertip(1, on seg3).
///
/// Shape: nbody=5, njnt=6, nq=6, nv=6, nu=6, ngeom=4, nsite=2.
#[must_use]
pub fn reaching_6dof_obstacle() -> Model {
    let mut model = Model::empty();
    set_options(
        &mut model,
        0.002,
        Vector3::new(0.0, 0.0, -9.81),
        Integrator::RungeKutta4,
        true,
    );
    // Target site sits on worldbody (body 0) — added before the arm so
    // it lands at site index 0 (matches MJCF site-emission order).
    add_site(
        &mut model,
        0,
        "target",
        Vector3::new(0.681_474, 0.154_033, 0.101_028),
        0.015,
    );
    add_6dof_segments(&mut model);
    add_6dof_motors(&mut model);

    // Obstacle: static body (no joints, no DOFs) at (0.730, 0.046, 0.030)
    // with a non-contact sphere geom of radius 0.06.
    let obstacle = add_body(
        &mut model,
        0,
        "obstacle",
        Vector3::new(0.730, 0.046, 0.030),
        0.0,
        Vector3::zeros(),
        Vector3::zeros(),
    );
    add_sphere_geom(
        &mut model,
        obstacle,
        Some("obstacle"),
        Vector3::zeros(),
        0.06,
        false,
    );

    finalize(&mut model);
    model
}

/// Internal helper: 6-DOF arm scaffold (without obstacle / target site).
/// Shared between [`reaching_6dof`] and [`reaching_6dof_obstacle`].
fn build_6dof_arm(model: &mut Model, with_obstacle: bool) {
    set_options(
        model,
        0.002,
        Vector3::new(0.0, 0.0, -9.81),
        Integrator::RungeKutta4,
        true,
    );
    add_6dof_segments(model);
    add_6dof_motors(model);
    let _ = with_obstacle; // caller adds obstacle/target separately
}

/// Push the three arm segments + their joints + their geoms + the
/// fingertip site. Joints are added in `j1..=j6` order; capsules and
/// site follow MJCF emission order.
fn add_6dof_segments(model: &mut Model) {
    // Segment 1: pos (0,0,0), capsule 0..0.3, mass 0.5
    let seg1_len = 0.3_f64;
    let seg1_mass = 0.5_f64;
    let seg1 = add_body(
        model,
        0,
        "seg1",
        Vector3::zeros(),
        seg1_mass,
        segment_inertia(seg1_mass, seg1_len),
        Vector3::new(seg1_len * 0.5, 0.0, 0.0),
    );
    add_hinge_joint(
        model,
        seg1,
        "j1",
        Vector3::new(0.0, -1.0, 0.0),
        0.0,
        2.0,
        0.0,
        true,
        (-PI, PI),
    );
    add_hinge_joint(
        model,
        seg1,
        "j2",
        Vector3::new(0.0, 0.0, 1.0),
        0.0,
        1.5,
        0.0,
        true,
        (-1.57, 1.57),
    );
    add_capsule_geom(
        model,
        seg1,
        Some("seg1_geom"),
        Vector3::zeros(),
        Vector3::new(seg1_len, 0.0, 0.0),
        0.03,
        false,
    );

    // Segment 2: pos (0.3,0,0), capsule 0..0.25, mass 0.3
    let seg2_len = 0.25_f64;
    let seg2_mass = 0.3_f64;
    let seg2 = add_body(
        model,
        seg1,
        "seg2",
        Vector3::new(seg1_len, 0.0, 0.0),
        seg2_mass,
        segment_inertia(seg2_mass, seg2_len),
        Vector3::new(seg2_len * 0.5, 0.0, 0.0),
    );
    add_hinge_joint(
        model,
        seg2,
        "j3",
        Vector3::new(0.0, -1.0, 0.0),
        0.0,
        1.5,
        0.0,
        true,
        (-2.6, 2.6),
    );
    add_hinge_joint(
        model,
        seg2,
        "j4",
        Vector3::new(0.0, 0.0, 1.0),
        0.0,
        1.0,
        0.0,
        true,
        (-1.57, 1.57),
    );
    add_capsule_geom(
        model,
        seg2,
        Some("seg2_geom"),
        Vector3::zeros(),
        Vector3::new(seg2_len, 0.0, 0.0),
        0.025,
        false,
    );

    // Segment 3: pos (0.25,0,0), capsule 0..0.2, mass 0.2
    let seg3_len = 0.2_f64;
    let seg3_mass = 0.2_f64;
    let seg3 = add_body(
        model,
        seg2,
        "seg3",
        Vector3::new(seg2_len, 0.0, 0.0),
        seg3_mass,
        segment_inertia(seg3_mass, seg3_len),
        Vector3::new(seg3_len * 0.5, 0.0, 0.0),
    );
    add_hinge_joint(
        model,
        seg3,
        "j5",
        Vector3::new(0.0, -1.0, 0.0),
        0.0,
        1.0,
        0.0,
        true,
        (-2.6, 2.6),
    );
    add_hinge_joint(
        model,
        seg3,
        "j6",
        Vector3::new(0.0, 0.0, 1.0),
        0.0,
        0.5,
        0.0,
        true,
        (-1.57, 1.57),
    );
    add_capsule_geom(
        model,
        seg3,
        Some("seg3_geom"),
        Vector3::zeros(),
        Vector3::new(seg3_len, 0.0, 0.0),
        0.02,
        false,
    );
    add_site(
        model,
        seg3,
        "fingertip",
        Vector3::new(seg3_len, 0.0, 0.0),
        0.015,
    );
}

/// Push 6 motors (gear 10/8/6/5/4/3) on joints 0..=5.
fn add_6dof_motors(model: &mut Model) {
    let gears = [10.0, 8.0, 6.0, 5.0, 4.0, 3.0];
    for (jnt_id, &gear) in gears.iter().enumerate() {
        add_motor(model, jnt_id, None, gear, Some((-1.0, 1.0)));
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::*;

    fn step_once(model: &Model) {
        let mut data = model.make_data();
        data.forward(model).expect("forward");
        data.step(model).expect("step");
    }

    #[test]
    fn reaching_2dof_shape() {
        let m = reaching_2dof();
        assert_eq!(m.nbody, 3, "world + upper_arm + forearm");
        assert_eq!(m.njnt, 2);
        assert_eq!(m.nq, 2);
        assert_eq!(m.nv, 2);
        assert_eq!(m.nu, 2);
        assert_eq!(m.ngeom, 2);
        assert_eq!(m.nsite, 1);
        assert_eq!(m.body_name[1].as_deref(), Some("upper_arm"));
        assert_eq!(m.body_name[2].as_deref(), Some("forearm"));
        assert_eq!(m.jnt_name[0].as_deref(), Some("shoulder"));
        assert_eq!(m.jnt_name[1].as_deref(), Some("elbow"));
        assert_eq!(m.actuator_name[0].as_deref(), Some("shoulder_motor"));
        assert_eq!(m.actuator_name[1].as_deref(), Some("elbow_motor"));
        assert_eq!(m.actuator_gear[0][0], 10.0);
        assert_eq!(m.actuator_gear[1][0], 5.0);
        assert_eq!(m.site_name[0].as_deref(), Some("fingertip"));
        step_once(&m);
    }

    #[test]
    fn reaching_6dof_shape() {
        let m = reaching_6dof();
        assert_eq!(m.nbody, 4, "world + seg1 + seg2 + seg3");
        assert_eq!(m.njnt, 6);
        assert_eq!(m.nq, 6);
        assert_eq!(m.nv, 6);
        assert_eq!(m.nu, 6);
        assert_eq!(m.ngeom, 3);
        assert_eq!(m.nsite, 1);
        assert_eq!(m.body_name[1].as_deref(), Some("seg1"));
        assert_eq!(m.body_name[3].as_deref(), Some("seg3"));
        assert_eq!(m.jnt_name[0].as_deref(), Some("j1"));
        assert_eq!(m.jnt_name[5].as_deref(), Some("j6"));
        assert_eq!(m.actuator_gear[0][0], 10.0);
        assert_eq!(m.actuator_gear[5][0], 3.0);
        step_once(&m);
    }

    #[test]
    fn reaching_6dof_obstacle_shape() {
        let m = reaching_6dof_obstacle();
        assert_eq!(m.nbody, 5, "world + 3 arm segs + obstacle");
        assert_eq!(m.njnt, 6);
        assert_eq!(m.nq, 6);
        assert_eq!(m.nv, 6);
        assert_eq!(m.ngeom, 4, "3 arm capsules + 1 obstacle sphere");
        assert_eq!(m.nsite, 2, "target + fingertip");
        // Site 0 is target on worldbody, site 1 is fingertip on seg3
        assert_eq!(m.site_name[0].as_deref(), Some("target"));
        assert_eq!(m.site_name[1].as_deref(), Some("fingertip"));
        assert_eq!(m.site_body[0], 0);
        assert_eq!(m.site_body[1], 3);
        // Obstacle is body 4
        assert_eq!(m.body_name[4].as_deref(), Some("obstacle"));
        step_once(&m);
    }
}
