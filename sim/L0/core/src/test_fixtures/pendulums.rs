//! Single-hinge pendulum variants.
//!
//! Each fixture mirrors a pendulum XML body found in the L0-pure test
//! suites. Bodies, joints, and motors are named to match the original
//! MJCF (`pendulum`, `hinge`, `motor`) so call-site name lookups work
//! after migration.
//!
//! Inertial values are thin-rod approximations sized to the capsule
//! length 0.5 m and mass 0.5 kg unless the call site specifies
//! otherwise. The original MJCFs leave inertia implicit (geom-derived);
//! these closed-form values are within a few percent of MuJoCo's
//! `inertiafromgeom` output for a uniform 0.5 m capsule.

use nalgebra::Vector3;

use super::builders::{
    add_body, add_capsule_geom, add_hinge_joint, add_jointpos_sensor, add_jointvel_sensor,
    add_mocap_body, add_motor, add_site, finalize, set_options,
};
use crate::types::Model;
use crate::types::enums::Integrator;

/// Build the canonical pendulum body + hinge + capsule scaffold used by
/// every preset in this module. Body name "pendulum" at world position
/// (0, 0, 1), hinge "hinge" rotating about the Y axis, capsule from
/// (0,0,0) to (0,0,-0.5) with radius 0.05 m.
fn pendulum_scaffold(model: &mut Model, gravity_on: bool) -> usize {
    let mass = 0.5_f64;
    let length = 0.5_f64;
    let i_transverse = mass * length * length / 12.0;
    let i_axial = 0.01 * i_transverse;
    let body = add_body(
        model,
        0,
        "pendulum",
        Vector3::new(0.0, 0.0, 1.0),
        mass,
        Vector3::new(i_transverse, i_transverse, i_axial),
        Vector3::new(0.0, 0.0, -length * 0.5),
    );
    let hinge = add_hinge_joint(
        model,
        body,
        "hinge",
        Vector3::new(0.0, 1.0, 0.0),
        0.0,
        0.0,
        0.0,
        false,
        (-std::f64::consts::PI, std::f64::consts::PI),
    );
    add_capsule_geom(
        model,
        body,
        None,
        Vector3::zeros(),
        Vector3::new(0.0, 0.0, -length),
        0.05,
        false,
    );
    if !gravity_on {
        // pendulum_xml() variants in env.rs / vec_env.rs / rollout.rs leave
        // gravity implicit (defaults to MuJoCo's standard g = 9.81). Caller
        // sets gravity via set_options before calling the scaffold.
    }
    hinge
}

/// 1-DOF pendulum, hinge joint, capsule geom, motor actuator. No
/// sensor. Matches `pendulum_xml()` in
/// `sim-ml-chassis/src/{env.rs:349,524, rollout.rs:226}`.
///
/// Shape: nbody=2, njnt=1, nq=1, nv=1, nu=1, ngeom=1, timestep=0.01.
#[must_use]
pub fn pendulum_basic() -> Model {
    let mut model = Model::empty();
    set_options(
        &mut model,
        0.01,
        Vector3::new(0.0, 0.0, -9.81),
        Integrator::Euler,
        false,
    );
    let hinge = pendulum_scaffold(&mut model, true);
    add_motor(&mut model, hinge, Some("motor"), 1.0, None);
    finalize(&mut model);
    model
}

/// `pendulum_basic` plus a `jointpos` sensor named `angle`. Matches
/// `pendulum_xml()` in `sim-ml-chassis/src/vec_env.rs:423` and
/// `space.rs:920`.
///
/// Shape: nbody=2, njnt=1, nq=1, nv=1, nu=1, ngeom=1, nsensor=1.
#[must_use]
pub fn pendulum_with_angle_sensor() -> Model {
    let mut model = Model::empty();
    set_options(
        &mut model,
        0.01,
        Vector3::new(0.0, 0.0, -9.81),
        Integrator::Euler,
        false,
    );
    let hinge = pendulum_scaffold(&mut model, true);
    add_motor(&mut model, hinge, Some("motor"), 1.0, None);
    add_jointpos_sensor(&mut model, hinge, "angle");
    finalize(&mut model);
    model
}

/// `pendulum_basic` with motor `ctrllimited="true" ctrlrange="-1 1"`.
/// Matches `pendulum_clamped()` in `sim-ml-chassis/src/space.rs:1485`.
///
/// Shape: nbody=2, njnt=1, nq=1, nv=1, nu=1, ngeom=1.
#[must_use]
pub fn pendulum_clamped() -> Model {
    let mut model = Model::empty();
    set_options(
        &mut model,
        0.01,
        Vector3::new(0.0, 0.0, -9.81),
        Integrator::Euler,
        false,
    );
    let hinge = pendulum_scaffold(&mut model, true);
    add_motor(&mut model, hinge, Some("motor"), 1.0, Some((-1.0, 1.0)));
    finalize(&mut model);
    model
}

/// `pendulum_basic` plus a mocap target body (no joints, kinematic)
/// named `target` at (0.5, 0, 0.5) with a non-contact sphere geom.
/// Matches `pendulum_mocap()` in `sim-ml-chassis/src/space.rs:1573`.
///
/// Shape: nbody=3 (world+target+pendulum), njnt=1, nmocap=1, nu=1.
/// MuJoCo body ordering puts the mocap body first under worldbody.
#[must_use]
pub fn pendulum_mocap() -> Model {
    let mut model = Model::empty();
    set_options(
        &mut model,
        0.01,
        Vector3::new(0.0, 0.0, -9.81),
        Integrator::Euler,
        false,
    );
    let target = add_mocap_body(&mut model, "target", Vector3::new(0.5, 0.0, 0.5));
    super::builders::add_sphere_geom(&mut model, target, None, Vector3::zeros(), 0.05, false);
    let hinge = pendulum_scaffold(&mut model, true);
    add_motor(&mut model, hinge, Some("motor"), 1.0, None);
    finalize(&mut model);
    model
}

/// `pendulum_basic` plus a `tip` site at the capsule's free end. Matches
/// `pendulum()` in `sim-ml-chassis/src/space.rs:920`.
///
/// Shape: nbody=2, njnt=1, ngeom=1, nsite=1, nu=1, nsensor=1.
#[must_use]
pub fn pendulum_with_tip_site() -> Model {
    let mut model = Model::empty();
    set_options(
        &mut model,
        0.01,
        Vector3::new(0.0, 0.0, -9.81),
        Integrator::Euler,
        false,
    );
    let hinge = pendulum_scaffold(&mut model, true);
    add_motor(&mut model, hinge, Some("motor"), 1.0, None);
    // Body at top of pendulum is body 1; site lives on it at the capsule end.
    add_site(&mut model, 1, "tip", Vector3::new(0.0, 0.0, -0.5), 0.01);
    add_jointpos_sensor(&mut model, hinge, "angle");
    finalize(&mut model);
    model
}

/// Bench-timestep pendulum: timestep=0.002, motor with
/// ctrllimited="-1 1", and both `jointpos` and `jointvel` sensors.
/// Matches `pendulum_model` in
/// `sim-ml-chassis/benches/bridge_benchmarks.rs:62`.
///
/// Shape: nbody=2, njnt=1, ngeom=1, nu=1, nsensor=2.
#[must_use]
pub fn pendulum_bench() -> Model {
    let mut model = Model::empty();
    set_options(
        &mut model,
        0.002,
        Vector3::new(0.0, 0.0, -9.81),
        Integrator::Euler,
        false,
    );
    let hinge = pendulum_scaffold(&mut model, true);
    add_motor(&mut model, hinge, Some("motor"), 1.0, Some((-1.0, 1.0)));
    add_jointpos_sensor(&mut model, hinge, "angle");
    add_jointvel_sensor(&mut model, hinge, "angvel");
    finalize(&mut model);
    model
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
    fn pendulum_basic_shape() {
        let m = pendulum_basic();
        assert_eq!(m.nbody, 2);
        assert_eq!(m.njnt, 1);
        assert_eq!(m.nq, 1);
        assert_eq!(m.nv, 1);
        assert_eq!(m.nu, 1);
        assert_eq!(m.ngeom, 1);
        assert_eq!(m.nsite, 0);
        assert_eq!(m.nsensor, 0);
        assert_eq!(m.body_name[1].as_deref(), Some("pendulum"));
        assert_eq!(m.jnt_name[0].as_deref(), Some("hinge"));
        assert_eq!(m.actuator_name[0].as_deref(), Some("motor"));
        assert_eq!(m.timestep, 0.01);
        step_once(&m);
    }

    #[test]
    fn pendulum_with_angle_sensor_shape() {
        let m = pendulum_with_angle_sensor();
        assert_eq!(m.nbody, 2);
        assert_eq!(m.nsensor, 1);
        assert_eq!(m.nsensordata, 1);
        assert_eq!(m.sensor_name[0].as_deref(), Some("angle"));
        step_once(&m);
    }

    #[test]
    fn pendulum_clamped_shape() {
        let m = pendulum_clamped();
        assert_eq!(m.nu, 1);
        assert_eq!(m.actuator_ctrlrange[0], (-1.0, 1.0));
        step_once(&m);
    }

    #[test]
    fn pendulum_mocap_shape() {
        let m = pendulum_mocap();
        assert_eq!(m.nbody, 3, "world + target (mocap) + pendulum");
        assert_eq!(m.nmocap, 1);
        assert_eq!(m.body_mocapid[1], Some(0));
        assert_eq!(m.body_name[1].as_deref(), Some("target"));
        assert_eq!(m.body_name[2].as_deref(), Some("pendulum"));
        step_once(&m);
    }

    #[test]
    fn pendulum_with_tip_site_shape() {
        let m = pendulum_with_tip_site();
        assert_eq!(m.nsite, 1);
        assert_eq!(m.site_name[0].as_deref(), Some("tip"));
        assert_eq!(m.nsensor, 1);
        step_once(&m);
    }

    #[test]
    fn pendulum_bench_shape() {
        let m = pendulum_bench();
        assert_eq!(m.timestep, 0.002);
        assert_eq!(m.actuator_ctrlrange[0], (-1.0, 1.0));
        assert_eq!(m.nsensor, 2);
        assert_eq!(m.nsensordata, 2);
        assert_eq!(m.sensor_name[0].as_deref(), Some("angle"));
        assert_eq!(m.sensor_name[1].as_deref(), Some("angvel"));
        step_once(&m);
    }
}
