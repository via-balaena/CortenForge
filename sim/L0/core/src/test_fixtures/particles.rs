//! Single-DOF slide-axis particle presets.
//!
//! These fixtures all share the same skeleton: a single body containing
//! a 1-DOF slide joint along the X axis with a unit-mass sphere geom.
//! They differ only in joint stiffness/damping/springref and in the
//! presence/parameters of an actuator.
//!
//! Each preset matches one of the canonical XML strings in
//! `sim-thermostat`'s test suite (`SHO_1D_XML`, `RATCHET_XML`, `SR_XML`,
//! `BISTABLE_XML`, `MINIMAL_XML`, `CHAIN_XML`'s single-element form,
//! the ad-hoc XML in `boltzmann_learning.rs:245`).

use nalgebra::Vector3;

use super::builders::{
    add_body, add_general_actuator, add_slide_joint, add_sphere_geom, finalize, set_options,
};
use crate::types::Model;
use crate::types::enums::Integrator;

/// Zero-gravity, contact-disabled slide particle along X axis. Mass = 1
/// kg, radius = 0.05 m, body name `name`, joint name `joint_name`.
fn slide_particle_at(
    model: &mut Model,
    name: &str,
    joint_name: &str,
    pos: Vector3<f64>,
    stiffness: f64,
    damping: f64,
    springref: f64,
) -> usize {
    let body = add_body(
        model,
        0,
        name,
        pos,
        1.0,
        Vector3::new(0.01, 0.01, 0.01),
        Vector3::zeros(),
    );
    let joint = add_slide_joint(
        model,
        body,
        joint_name,
        Vector3::x(),
        stiffness,
        damping,
        springref,
    );
    add_sphere_geom(model, body, None, Vector3::zeros(), 0.05, false);
    joint
}

/// 1D simple harmonic oscillator: slide joint with stiffness=1,
/// damping=0, springref=0. Body name `particle`, joint `x`.
/// Matches `SHO_1D_XML` in
/// `sim-thermostat/tests/langevin_thermostat.rs`.
///
/// Shape: nbody=2, njnt=1, nq=1, nv=1, ngeom=1, nu=0.
#[must_use]
pub fn sho_1d() -> Model {
    let mut model = Model::empty();
    set_options(
        &mut model,
        0.001,
        Vector3::zeros(),
        Integrator::Euler,
        false,
    );
    slide_particle_at(&mut model, "particle", "x", Vector3::zeros(), 1.0, 0.0, 0.0);
    finalize(&mut model);
    model
}

/// Brownian-ratchet particle: slide joint plus an inert `ratchet_ctrl` actuator.
///
/// The `general` actuator has `gainprm=0` and `ctrlrange=(0, 1)`; the
/// test harness reads `data.ctrl` as the ratchet drive signal but the
/// actuator itself produces no joint force. Matches `RATCHET_XML` in
/// `sim-thermostat/tests/d1*.rs`.
///
/// Shape: nbody=2, njnt=1, nq=1, nv=1, ngeom=1, nu=1.
#[must_use]
pub fn ratchet() -> Model {
    let mut model = Model::empty();
    set_options(&mut model, 0.001, Vector3::zeros(), Integrator::Euler, true);
    let joint = slide_particle_at(&mut model, "particle", "x", Vector3::zeros(), 0.0, 0.0, 0.0);
    add_general_actuator(
        &mut model,
        joint,
        Some("ratchet_ctrl"),
        [0.0; 9],
        [0.0; 9],
        Some((0.0, 1.0)),
    );
    finalize(&mut model);
    model
}

/// Stochastic-resonance particle: same shape as [`ratchet`] with a `temp_ctrl` actuator.
///
/// The actuator's `ctrlrange=(0, 10)`; the harness uses `data.ctrl[0]`
/// as a temperature setpoint. Matches `SR_XML` in
/// `sim-thermostat/tests/d2*.rs` and `sim-opt/tests/d2c_sr_rematch*.rs`.
///
/// Shape: nbody=2, njnt=1, nq=1, nv=1, ngeom=1, nu=1.
#[must_use]
pub fn stochastic_resonance() -> Model {
    let mut model = Model::empty();
    set_options(&mut model, 0.001, Vector3::zeros(), Integrator::Euler, true);
    let joint = slide_particle_at(&mut model, "particle", "x", Vector3::zeros(), 0.0, 0.0, 0.0);
    add_general_actuator(
        &mut model,
        joint,
        Some("temp_ctrl"),
        [0.0; 9],
        [0.0; 9],
        Some((0.0, 10.0)),
    );
    finalize(&mut model);
    model
}

/// Bistable 1-DOF particle: slide joint, no spring, no actuator.
///
/// Stiffness=0, damping=0, springref=0. Test harness applies a custom
/// double-well force. Matches `BISTABLE_XML` in
/// `sim-thermostat/tests/kramers_escape_rate.rs`.
///
/// Shape: nbody=2, njnt=1, nq=1, nv=1, ngeom=1, nu=0.
#[must_use]
pub fn bistable_1dof() -> Model {
    let mut model = Model::empty();
    set_options(
        &mut model,
        0.001,
        Vector3::zeros(),
        Integrator::Euler,
        false,
    );
    slide_particle_at(&mut model, "particle", "x", Vector3::zeros(), 0.0, 0.0, 0.0);
    finalize(&mut model);
    model
}

/// Single slide particle named `p0` on joint `x0`, no actuator, no
/// spring. Matches the ad-hoc inline XML in
/// `sim-thermostat/tests/boltzmann_learning.rs:245` (an
/// `ExternalField` symmetry test).
///
/// Shape: nbody=2, njnt=1, nq=1, nv=1, ngeom=1, nu=0.
#[must_use]
pub fn single_slide() -> Model {
    let mut model = Model::empty();
    set_options(
        &mut model,
        0.001,
        Vector3::zeros(),
        Integrator::Euler,
        false,
    );
    slide_particle_at(&mut model, "p0", "x0", Vector3::zeros(), 0.0, 0.0, 0.0);
    finalize(&mut model);
    model
}

/// Two slide particles named `e0` and `e1`, joints `x0` and `x1`,
/// spaced 0.2 m apart along X. Matches `MINIMAL_XML` in
/// `sim-thermostat/src/ising_learner.rs`.
///
/// Shape: nbody=3, njnt=2, nq=2, nv=2, ngeom=2, nu=0.
#[must_use]
pub fn ising_pair() -> Model {
    let mut model = Model::empty();
    set_options(
        &mut model,
        0.001,
        Vector3::zeros(),
        Integrator::Euler,
        false,
    );
    slide_particle_at(&mut model, "e0", "x0", Vector3::zeros(), 0.0, 0.0, 0.0);
    slide_particle_at(
        &mut model,
        "e1",
        "x1",
        Vector3::new(0.2, 0.0, 0.0),
        0.0,
        0.0,
        0.0,
    );
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
    fn sho_1d_shape() {
        let m = sho_1d();
        assert_eq!(m.nbody, 2);
        assert_eq!(m.nq, 1);
        assert_eq!(m.nv, 1);
        assert_eq!(m.nu, 0);
        assert_eq!(m.body_name[1].as_deref(), Some("particle"));
        assert_eq!(m.jnt_name[0].as_deref(), Some("x"));
        assert_eq!(m.jnt_stiffness[0], 1.0);
        step_once(&m);
    }

    #[test]
    fn ratchet_shape() {
        let m = ratchet();
        assert_eq!(m.nu, 1);
        assert_eq!(m.actuator_name[0].as_deref(), Some("ratchet_ctrl"));
        assert_eq!(m.actuator_ctrlrange[0], (0.0, 1.0));
        // gainprm[0] == 0 → actuator produces no joint force
        assert_eq!(m.actuator_gainprm[0][0], 0.0);
        step_once(&m);
    }

    #[test]
    fn stochastic_resonance_shape() {
        let m = stochastic_resonance();
        assert_eq!(m.nu, 1);
        assert_eq!(m.actuator_name[0].as_deref(), Some("temp_ctrl"));
        assert_eq!(m.actuator_ctrlrange[0], (0.0, 10.0));
        step_once(&m);
    }

    #[test]
    fn bistable_1dof_shape() {
        let m = bistable_1dof();
        assert_eq!(m.nu, 0);
        assert_eq!(m.jnt_stiffness[0], 0.0);
        step_once(&m);
    }

    #[test]
    fn single_slide_shape() {
        let m = single_slide();
        assert_eq!(m.nbody, 2);
        assert_eq!(m.body_name[1].as_deref(), Some("p0"));
        assert_eq!(m.jnt_name[0].as_deref(), Some("x0"));
        step_once(&m);
    }

    #[test]
    fn ising_pair_shape() {
        let m = ising_pair();
        assert_eq!(m.nbody, 3);
        assert_eq!(m.njnt, 2);
        assert_eq!(m.nq, 2);
        assert_eq!(m.nv, 2);
        assert_eq!(m.body_name[1].as_deref(), Some("e0"));
        assert_eq!(m.body_name[2].as_deref(), Some("e1"));
        assert_eq!(m.jnt_name[0].as_deref(), Some("x0"));
        assert_eq!(m.jnt_name[1].as_deref(), Some("x1"));
        // e1 sits 0.2 m to the right of e0 at rest
        assert_eq!(m.body_pos[2].x, 0.2);
        step_once(&m);
    }
}
