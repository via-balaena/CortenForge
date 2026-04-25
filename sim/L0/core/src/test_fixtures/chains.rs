//! Multi-element chain fixtures.
//!
//! Parametric chain factories sized by an `n: usize` count. Concrete
//! presets used in the test suites compose them at fixed N.

use nalgebra::Vector3;

use super::builders::{
    add_body, add_capsule_geom, add_hinge_joint, add_motor, add_slide_joint, add_sphere_geom,
    finalize, set_options,
};
use crate::types::Model;
use crate::types::enums::Integrator;

/// Bistable chain of `n` slide-axis particles spaced 1 m apart along Y.
///
/// Each particle has its own slide joint along X (`x0`, `x1`, ...,
/// `x{n-1}`) with stiffness=0, damping=0. Body names are
/// `p0`..`p{n-1}`. Sphere geoms are non-contact (`contype=0`,
/// `conaffinity=0`). No actuators.
///
/// Matches `CHAIN_XML` (n=4) in `coupled_bistable_array.rs`,
/// `gibbs_sampler.rs`, `boltzmann_learning.rs`.
///
/// # Panics
///
/// Panics if `n == 0`.
#[must_use]
pub fn bistable_chain(n: usize) -> Model {
    assert!(n >= 1, "bistable_chain requires at least 1 element");
    let mut model = Model::empty();
    set_options(
        &mut model,
        0.001,
        Vector3::zeros(),
        Integrator::Euler,
        false,
    );
    for i in 0..n {
        let body_name = format!("p{i}");
        let joint_name = format!("x{i}");
        // `i as f64` is bounded by realistic chain lengths (≤ ~64) — exact under f64.
        #[allow(clippy::cast_precision_loss)]
        let pos = Vector3::new(0.0, i as f64, 0.0);
        let body = add_body(
            &mut model,
            0,
            &body_name,
            pos,
            1.0,
            Vector3::new(0.01, 0.01, 0.01),
            Vector3::zeros(),
        );
        add_slide_joint(&mut model, body, &joint_name, Vector3::x(), 0.0, 0.0, 0.0);
        add_sphere_geom(&mut model, body, None, Vector3::zeros(), 0.05, false);
    }
    finalize(&mut model);
    model
}

/// Realistic `n`-link serial hinge chain on a ground plane under gravity.
///
/// Each link is 0.15 m long, 0.5 kg, with damping=0.5 on its hinge
/// joint and a contact-enabled capsule geom; `n` motors carry
/// `ctrlrange=(-1, 1)`. Body names `link0`..`link{n-1}`, joint names
/// `j0`..`j{n-1}`, motor names `m0`..`m{n-1}`. Top body hangs at
/// z=1.5; subsequent links nest along -Z relative to parent.
///
/// Matches `chain_model` (n=10) in
/// `sim-ml-chassis/benches/bridge_benchmarks.rs:292`.
///
/// # Panics
///
/// Panics if `n == 0`.
#[must_use]
pub fn hinge_chain(n: usize) -> Model {
    assert!(n >= 1, "hinge_chain requires at least 1 link");
    let mut model = Model::empty();
    set_options(
        &mut model,
        0.002,
        Vector3::new(0.0, 0.0, -9.81),
        Integrator::Euler,
        false,
    );
    super::builders::add_ground_plane(&mut model);

    let link_len = 0.15_f64;
    let link_mass = 0.5_f64;
    let i_transverse = link_mass * link_len * link_len / 12.0;
    let i_axial = 0.01 * i_transverse;
    let inertia = Vector3::new(i_transverse, i_transverse, i_axial);

    let mut parent = 0_usize;
    let mut joint_ids = Vec::with_capacity(n);
    for i in 0..n {
        let body_name = format!("link{i}");
        let joint_name = format!("j{i}");
        let pos = if i == 0 {
            Vector3::new(0.0, 0.0, 1.5)
        } else {
            Vector3::new(0.0, 0.0, -link_len)
        };
        let body = add_body(
            &mut model,
            parent,
            &body_name,
            pos,
            link_mass,
            inertia,
            Vector3::new(0.0, 0.0, -link_len * 0.5),
        );
        let jnt = add_hinge_joint(
            &mut model,
            body,
            &joint_name,
            Vector3::new(0.0, 1.0, 0.0),
            0.0,
            0.5,
            0.0,
            false,
            (-std::f64::consts::PI, std::f64::consts::PI),
        );
        add_capsule_geom(
            &mut model,
            body,
            None,
            Vector3::zeros(),
            Vector3::new(0.0, 0.0, -link_len),
            0.04,
            true,
        );
        joint_ids.push(jnt);
        parent = body;
    }
    for (i, &jnt) in joint_ids.iter().enumerate() {
        let name = format!("m{i}");
        add_motor(&mut model, jnt, Some(&name), 1.0, Some((-1.0, 1.0)));
    }
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
    fn bistable_chain_4_shape() {
        let m = bistable_chain(4);
        assert_eq!(m.nbody, 5, "world + 4 particles");
        assert_eq!(m.njnt, 4);
        assert_eq!(m.nq, 4);
        assert_eq!(m.nv, 4);
        assert_eq!(m.nu, 0);
        for i in 0..4 {
            assert_eq!(m.body_name[i + 1].as_deref(), Some(&*format!("p{i}")));
            assert_eq!(m.jnt_name[i].as_deref(), Some(&*format!("x{i}")));
        }
        // p1 sits 1.0 m along Y from p0
        assert_eq!(m.body_pos[2].y, 1.0);
        step_once(&m);
    }

    #[test]
    fn bistable_chain_handles_n_1() {
        let m = bistable_chain(1);
        assert_eq!(m.nbody, 2);
        assert_eq!(m.njnt, 1);
        step_once(&m);
    }

    #[test]
    fn hinge_chain_10_shape() {
        let m = hinge_chain(10);
        assert_eq!(m.nbody, 11, "world + 10 links");
        assert_eq!(m.njnt, 10);
        assert_eq!(m.nq, 10);
        assert_eq!(m.nv, 10);
        assert_eq!(m.nu, 10);
        // Ground plane + 10 capsules
        assert_eq!(m.ngeom, 11);
        for i in 0..10 {
            assert_eq!(m.body_name[i + 1].as_deref(), Some(&*format!("link{i}")));
            assert_eq!(m.actuator_name[i].as_deref(), Some(&*format!("m{i}")));
        }
        step_once(&m);
    }
}
