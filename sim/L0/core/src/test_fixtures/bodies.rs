//! Free-body and miscellaneous single-purpose fixtures.

use nalgebra::Vector3;

use super::builders::{
    add_body, add_box_geom, add_capsule_geom, add_freejoint, add_hinge_joint, add_jointpos_sensor,
    add_jointvel_sensor, add_motor, add_slide_joint, add_sphere_geom, finalize, set_options,
};
use crate::types::Model;
use crate::types::enums::Integrator;

/// 6-DOF free-floating box body with explicit diagonal inertia.
///
/// Body `box` at (0, 0, 1) with a non-contact box geom (half-extent
/// 0.1). Matches `FREE_BODY_XML` in
/// `sim-thermostat/tests/multi_dof_equipartition.rs`.
///
/// Shape: nbody=2, njnt=1, nq=7, nv=6, ngeom=1, nu=0.
#[must_use]
pub fn free_body_diag(mass: f64, diag_inertia: Vector3<f64>) -> Model {
    let mut model = Model::empty();
    set_options(
        &mut model,
        0.001,
        Vector3::zeros(),
        Integrator::Euler,
        false,
    );
    let body = add_body(
        &mut model,
        0,
        "box",
        Vector3::new(0.0, 0.0, 1.0),
        mass,
        diag_inertia,
        Vector3::zeros(),
    );
    add_freejoint(&mut model, body, "free");
    add_box_geom(
        &mut model,
        body,
        None,
        Vector3::zeros(),
        Vector3::new(0.1, 0.1, 0.1),
        false,
    );
    finalize(&mut model);
    model
}

/// 2-link hinge chain with explicit `inertial` blocks and stiffness=20.
///
/// Each link has mass=1, diaginertia=[0.01, 0.1, 0.1]. Capsules carry
/// mass=0 to avoid double-counting (the inertial block is the
/// authoritative mass source). Body names `link1`/`link2`, joint names
/// `j1`/`j2`. Matches `HINGE_CHAIN_XML` in
/// `sim-thermostat/tests/multi_dof_equipartition.rs`.
///
/// Shape: nbody=3, njnt=2, nq=2, nv=2, ngeom=2, nu=0.
#[must_use]
pub fn hinge_chain_2dof_inertial() -> Model {
    let mut model = Model::empty();
    set_options(
        &mut model,
        0.001,
        Vector3::zeros(),
        Integrator::Euler,
        false,
    );
    let inertia = Vector3::new(0.01, 0.1, 0.1);
    let mass = 1.0_f64;

    let link1 = add_body(
        &mut model,
        0,
        "link1",
        Vector3::zeros(),
        mass,
        inertia,
        Vector3::new(0.25, 0.0, 0.0),
    );
    add_hinge_joint(
        &mut model,
        link1,
        "j1",
        Vector3::new(0.0, 1.0, 0.0),
        20.0,
        0.0,
        0.0,
        false,
        (-std::f64::consts::PI, std::f64::consts::PI),
    );
    add_capsule_geom(
        &mut model,
        link1,
        None,
        Vector3::zeros(),
        Vector3::new(0.5, 0.0, 0.0),
        0.05,
        false,
    );

    let link2 = add_body(
        &mut model,
        link1,
        "link2",
        Vector3::new(0.5, 0.0, 0.0),
        mass,
        inertia,
        Vector3::new(0.25, 0.0, 0.0),
    );
    add_hinge_joint(
        &mut model,
        link2,
        "j2",
        Vector3::new(0.0, 1.0, 0.0),
        20.0,
        0.0,
        0.0,
        false,
        (-std::f64::consts::PI, std::f64::consts::PI),
    );
    add_capsule_geom(
        &mut model,
        link2,
        None,
        Vector3::zeros(),
        Vector3::new(0.5, 0.0, 0.0),
        0.05,
        false,
    );

    finalize(&mut model);
    model
}

/// Cart-pole: cart on a slide joint along X, pole hinged about Y.
///
/// Cart has a force motor (`force`) on the slide joint; sensors are
/// `cart_pos` (jointpos on slide) and `pole_vel` (jointvel on hinge).
/// Matches the cart-pole XML in `sim-ml-chassis/src/space.rs:1210`.
///
/// Shape: nbody=3, njnt=2, nq=2, nv=2, nu=1, ngeom=2, nsensor=2.
#[must_use]
pub fn cart_pole() -> Model {
    let mut model = Model::empty();
    set_options(
        &mut model,
        0.01,
        Vector3::new(0.0, 0.0, -9.81),
        Integrator::Euler,
        false,
    );

    let cart = add_body(
        &mut model,
        0,
        "cart",
        Vector3::new(0.0, 0.0, 0.5),
        1.0,
        Vector3::new(0.01, 0.01, 0.01),
        Vector3::zeros(),
    );
    let slide = add_slide_joint(&mut model, cart, "slide", Vector3::x(), 0.0, 0.0, 0.0);
    add_sphere_geom(&mut model, cart, None, Vector3::zeros(), 0.1, false);

    let pole_len = 0.5_f64;
    let pole_mass = 0.1_f64;
    let i_transverse = pole_mass * pole_len * pole_len / 12.0;
    let pole = add_body(
        &mut model,
        cart,
        "pole",
        Vector3::new(0.0, 0.0, 0.1),
        pole_mass,
        Vector3::new(i_transverse, i_transverse, 0.001),
        Vector3::new(0.0, 0.0, pole_len * 0.5),
    );
    let hinge = add_hinge_joint(
        &mut model,
        pole,
        "hinge",
        Vector3::new(0.0, 1.0, 0.0),
        0.0,
        0.0,
        0.0,
        false,
        (-std::f64::consts::PI, std::f64::consts::PI),
    );
    add_capsule_geom(
        &mut model,
        pole,
        None,
        Vector3::zeros(),
        Vector3::new(0.0, 0.0, pole_len),
        0.02,
        false,
    );

    add_motor(&mut model, slide, Some("force"), 1.0, None);
    add_jointpos_sensor(&mut model, slide, "cart_pos");
    add_jointvel_sensor(&mut model, hinge, "pole_vel");

    finalize(&mut model);
    model
}

/// Smallest valid model: 1 body, 1 hinge joint, 1 sphere, 1 motor.
///
/// Used by `sim-ml-chassis::env` builder validation tests
/// (`builder_missing_*`, `builder_zero_sub_steps`) where the model
/// shape is irrelevant — only that a `Model` exists with `nu >= 1` and
/// `nq >= 1`.
///
/// Shape: nbody=2, njnt=1, nq=1, nv=1, nu=1, ngeom=1.
#[must_use]
pub fn builder_test_minimal() -> Model {
    let mut model = Model::empty();
    set_options(
        &mut model,
        0.002,
        Vector3::new(0.0, 0.0, -9.81),
        Integrator::Euler,
        false,
    );
    let body = add_body(
        &mut model,
        0,
        "b",
        Vector3::zeros(),
        1.0,
        Vector3::new(0.01, 0.01, 0.01),
        Vector3::zeros(),
    );
    let jnt = add_hinge_joint(
        &mut model,
        body,
        "j",
        Vector3::new(0.0, 1.0, 0.0),
        0.0,
        0.0,
        0.0,
        false,
        (-std::f64::consts::PI, std::f64::consts::PI),
    );
    add_sphere_geom(&mut model, body, None, Vector3::zeros(), 0.1, false);
    add_motor(&mut model, jnt, None, 1.0, None);
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
    fn free_body_diag_shape() {
        let m = free_body_diag(1.0, Vector3::new(0.5, 1.0, 1.5));
        assert_eq!(m.nbody, 2);
        assert_eq!(m.njnt, 1);
        assert_eq!(m.nq, 7, "freejoint contributes 3 pos + 4 quat to nq");
        assert_eq!(m.nv, 6, "freejoint contributes 6 to nv");
        assert_eq!(m.body_name[1].as_deref(), Some("box"));
        assert_eq!(m.body_inertia[1], Vector3::new(0.5, 1.0, 1.5));
        step_once(&m);
    }

    #[test]
    fn hinge_chain_2dof_inertial_shape() {
        let m = hinge_chain_2dof_inertial();
        assert_eq!(m.nbody, 3);
        assert_eq!(m.njnt, 2);
        assert_eq!(m.nq, 2);
        assert_eq!(m.nv, 2);
        assert_eq!(m.nu, 0);
        assert_eq!(m.body_name[1].as_deref(), Some("link1"));
        assert_eq!(m.body_name[2].as_deref(), Some("link2"));
        assert_eq!(m.jnt_stiffness[0], 20.0);
        assert_eq!(m.jnt_stiffness[1], 20.0);
        // Both links carry mass=1 with diaginertia=[0.01, 0.1, 0.1]
        assert_eq!(m.body_mass[1], 1.0);
        assert_eq!(m.body_inertia[1], Vector3::new(0.01, 0.1, 0.1));
        step_once(&m);
    }

    #[test]
    fn cart_pole_shape() {
        let m = cart_pole();
        assert_eq!(m.nbody, 3);
        assert_eq!(m.njnt, 2);
        assert_eq!(m.nq, 2);
        assert_eq!(m.nv, 2);
        assert_eq!(m.nu, 1);
        assert_eq!(m.nsensor, 2);
        assert_eq!(m.nsensordata, 2);
        assert_eq!(m.body_name[1].as_deref(), Some("cart"));
        assert_eq!(m.body_name[2].as_deref(), Some("pole"));
        assert_eq!(m.jnt_name[0].as_deref(), Some("slide"));
        assert_eq!(m.jnt_name[1].as_deref(), Some("hinge"));
        assert_eq!(m.actuator_name[0].as_deref(), Some("force"));
        assert_eq!(m.sensor_name[0].as_deref(), Some("cart_pos"));
        assert_eq!(m.sensor_name[1].as_deref(), Some("pole_vel"));
        step_once(&m);
    }

    #[test]
    fn builder_test_minimal_shape() {
        let m = builder_test_minimal();
        assert_eq!(m.nbody, 2);
        assert_eq!(m.nq, 1);
        assert_eq!(m.nu, 1);
        step_once(&m);
    }
}
