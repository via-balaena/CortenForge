//! Derived sensor computations — subtree COM, momentum, body acceleration, angular momentum.
//!
//! These functions compute quantities that multiple sensor stages need:
//! subtree center of mass, subtree linear/angular momentum, body linear/angular
//! acceleration, and site force/torque via subtree inverse dynamics.

use crate::types::{Data, MjJointType, MjObjectType, Model};
use nalgebra::Vector3;

pub fn compute_subtree_com(model: &Model, data: &Data, root_body: usize) -> (Vector3<f64>, f64) {
    let mut total_mass = 0.0;
    let mut weighted_com = Vector3::zeros();

    // Iterate through all bodies that are descendants of root_body
    for body_id in root_body..model.nbody {
        // Check if this body is a descendant (simplified: check parent chain)
        let mut is_descendant = body_id == root_body;
        let mut current = body_id;
        while !is_descendant && current != 0 {
            current = model.body_parent[current];
            if current == root_body {
                is_descendant = true;
            }
        }

        if is_descendant {
            let mass = model.body_mass[body_id];
            let com = data.xipos[body_id]; // COM in world frame
            total_mass += mass;
            weighted_com += mass * com;
        }
    }

    if total_mass > 1e-10 {
        (weighted_com / total_mass, total_mass)
    } else {
        (Vector3::zeros(), 0.0)
    }
}

/// Compute subtree linear momentum for a given body.
pub fn compute_subtree_momentum(model: &Model, data: &Data, root_body: usize) -> Vector3<f64> {
    let mut momentum = Vector3::zeros();

    for body_id in root_body..model.nbody {
        // Check if descendant
        let mut is_descendant = body_id == root_body;
        let mut current = body_id;
        while !is_descendant && current != 0 {
            current = model.body_parent[current];
            if current == root_body {
                is_descendant = true;
            }
        }

        if is_descendant {
            let mass = model.body_mass[body_id];
            let omega = Vector3::new(
                data.cvel[body_id][0],
                data.cvel[body_id][1],
                data.cvel[body_id][2],
            );
            let v_origin = Vector3::new(
                data.cvel[body_id][3],
                data.cvel[body_id][4],
                data.cvel[body_id][5],
            );
            // Shift from body origin (xpos) to body COM (xipos)
            let dif = data.xipos[body_id] - data.xpos[body_id];
            let v_com = v_origin + omega.cross(&dif);
            momentum += mass * v_com;
        }
    }

    momentum
}

/// Compute linear acceleration for a body from joint accelerations.
pub fn compute_body_acceleration(model: &Model, data: &Data, body_id: usize) -> Vector3<f64> {
    let mut acc = Vector3::zeros();

    // For each joint affecting this body, compute acceleration contribution
    let jnt_start = model.body_jnt_adr[body_id];
    let jnt_end = jnt_start + model.body_jnt_num[body_id];

    for jnt_id in jnt_start..jnt_end {
        let dof_adr = model.jnt_dof_adr[jnt_id];
        let axis = model.jnt_axis[jnt_id];

        match model.jnt_type[jnt_id] {
            MjJointType::Hinge => {
                // Angular acceleration contributes via cross product with COM offset
                let qacc = data.qacc[dof_adr];
                let world_axis = data.xquat[body_id] * axis;
                let com_offset = data.xipos[body_id] - data.xpos[body_id];
                // a = α × r (tangential acceleration from angular acceleration)
                acc += qacc * world_axis.cross(&com_offset);

                // Also centripetal from angular velocity
                let omega = data.qvel[dof_adr] * world_axis;
                acc += omega.cross(&omega.cross(&com_offset));
            }
            MjJointType::Slide => {
                let qacc = data.qacc[dof_adr];
                let world_axis = data.xquat[body_id] * axis;
                acc += qacc * world_axis;
            }
            MjJointType::Free => {
                // Direct linear acceleration
                acc.x += data.qacc[dof_adr];
                acc.y += data.qacc[dof_adr + 1];
                acc.z += data.qacc[dof_adr + 2];
            }
            MjJointType::Ball => {
                // Ball joint only contributes angular acceleration
                // Linear acceleration comes from centripetal effects
                let omega = Vector3::new(
                    data.qvel[dof_adr],
                    data.qvel[dof_adr + 1],
                    data.qvel[dof_adr + 2],
                );
                let alpha = Vector3::new(
                    data.qacc[dof_adr],
                    data.qacc[dof_adr + 1],
                    data.qacc[dof_adr + 2],
                );
                let world_omega = data.xquat[body_id] * omega;
                let world_alpha = data.xquat[body_id] * alpha;
                let com_offset = data.xipos[body_id] - data.xpos[body_id];
                acc += world_alpha.cross(&com_offset);
                acc += world_omega.cross(&world_omega.cross(&com_offset));
            }
        }
    }

    acc
}

/// Compute angular acceleration for a body from joint accelerations.
pub fn compute_body_angular_acceleration(
    model: &Model,
    data: &Data,
    body_id: usize,
) -> Vector3<f64> {
    let mut alpha = Vector3::zeros();

    let jnt_start = model.body_jnt_adr[body_id];
    let jnt_end = jnt_start + model.body_jnt_num[body_id];

    for jnt_id in jnt_start..jnt_end {
        let dof_adr = model.jnt_dof_adr[jnt_id];
        let axis = model.jnt_axis[jnt_id];

        match model.jnt_type[jnt_id] {
            MjJointType::Hinge => {
                let qacc = data.qacc[dof_adr];
                let world_axis = data.xquat[body_id] * axis;
                alpha += qacc * world_axis;
            }
            MjJointType::Ball => {
                let world_alpha = data.xquat[body_id]
                    * Vector3::new(
                        data.qacc[dof_adr],
                        data.qacc[dof_adr + 1],
                        data.qacc[dof_adr + 2],
                    );
                alpha += world_alpha;
            }
            MjJointType::Free => {
                alpha.x += data.qacc[dof_adr + 3];
                alpha.y += data.qacc[dof_adr + 4];
                alpha.z += data.qacc[dof_adr + 5];
            }
            MjJointType::Slide => {
                // Prismatic joints don't contribute angular acceleration
            }
        }
    }

    alpha
}

/// Compute interaction force and torque at a sensor's site via subtree inverse dynamics.
///
/// For a force/torque sensor attached to a site on body B, we compute the net
/// force and torque that the rest of the system exerts on the subtree rooted at B.
///
/// This is computed as: F = sum over subtree bodies of (m_i * a_i - f_ext_i)
/// where a_i is the body acceleration (from qacc) and f_ext_i are the external
/// forces (gravity, applied, actuator, passive, constraint).
///
/// The torque is computed about the sensor site position.
///
/// Returns (force_world, torque_world) in world frame.
pub fn compute_site_force_torque(
    model: &Model,
    data: &Data,
    sensor_id: usize,
) -> (Vector3<f64>, Vector3<f64>) {
    let objid = model.sensor_objid[sensor_id];

    // Get the body and site position
    let (body_id, site_pos) = match model.sensor_objtype[sensor_id] {
        MjObjectType::Site if objid < model.nsite => {
            (model.site_body[objid], data.site_xpos[objid])
        }
        MjObjectType::Body if objid < model.nbody => (objid, data.xpos[objid]),
        _ => return (Vector3::zeros(), Vector3::zeros()),
    };

    // Sum forces and torques over all bodies in the subtree rooted at body_id.
    // For each body in subtree:
    //   f_inertial = m * a_com  (inertial force from acceleration)
    //   f_gravity  = m * g      (gravity acts on all bodies)
    //
    // The interaction force = sum of (f_inertial - f_gravity) for subtree
    // This equals the net constraint/contact force transmitted through the joint.
    let mut force_total = Vector3::zeros();
    let mut torque_total = Vector3::zeros();

    for bid in body_id..model.nbody {
        // Check if this body is in the subtree rooted at body_id
        let mut is_descendant = bid == body_id;
        let mut current = bid;
        while !is_descendant && current != 0 {
            current = model.body_parent[current];
            if current == body_id {
                is_descendant = true;
            }
        }

        if !is_descendant {
            continue;
        }

        let mass = model.body_mass[bid];
        if mass < 1e-15 {
            continue;
        }

        // Compute body COM acceleration
        let a_com = compute_body_acceleration(model, data, bid);

        // Inertial force = m * a (what's needed to produce the acceleration)
        let f_inertial = mass * a_com;

        // Gravity force acting on this body
        let f_gravity = mass * model.gravity;

        // Net force on this body from the rest of the system
        // F_net = m*a, so the interaction force through the cut is m*a - f_gravity
        // (because gravity is an external force, not an interaction force)
        let f_body = f_inertial - f_gravity;

        force_total += f_body;

        // Torque contribution about sensor site
        let r = data.xipos[bid] - site_pos;
        torque_total += r.cross(&f_body);

        // Angular inertia contribution: I * alpha + omega x (I * omega)
        // Use the body inertia in world frame
        let inertia = model.body_inertia[bid];
        let alpha = compute_body_angular_acceleration(model, data, bid);

        // Get body angular velocity
        let omega = Vector3::new(data.cvel[bid][0], data.cvel[bid][1], data.cvel[bid][2]);

        // Rotate inertia to world frame via body's inertial frame orientation
        let xi_mat = data.ximat[bid];
        // I_world = R * diag(I) * R^T
        let i_omega = xi_mat
            * Vector3::new(
                inertia.x * (xi_mat.transpose() * omega).x,
                inertia.y * (xi_mat.transpose() * omega).y,
                inertia.z * (xi_mat.transpose() * omega).z,
            );
        let i_alpha = xi_mat
            * Vector3::new(
                inertia.x * (xi_mat.transpose() * alpha).x,
                inertia.y * (xi_mat.transpose() * alpha).y,
                inertia.z * (xi_mat.transpose() * alpha).z,
            );

        // Angular: I*alpha + omega x (I*omega)
        torque_total += i_alpha + omega.cross(&i_omega);
    }

    (force_total, torque_total)
}

/// Check if a body is a descendant of (or equal to) a given root body.
pub fn is_body_in_subtree(model: &Model, body_id: usize, root: usize) -> bool {
    let mut current = body_id;
    loop {
        if current == root {
            return true;
        }
        if current == 0 {
            return false;
        }
        current = model.body_parent[current];
    }
}

/// Compute subtree angular momentum about the subtree's center of mass.
///
/// For each body in the subtree:
///   L += I_i * omega_i + m_i * (r_i - r_com) x v_i
///
/// where I_i is the body inertia, omega_i angular velocity, r_i position,
/// r_com subtree COM, v_i linear velocity, and m_i mass.
pub fn compute_subtree_angmom(model: &Model, data: &Data, root_body: usize) -> Vector3<f64> {
    // First compute subtree COM
    let (com, _total_mass) = compute_subtree_com(model, data, root_body);

    let mut angmom = Vector3::zeros();

    for body_id in root_body..model.nbody {
        if !is_body_in_subtree(model, body_id, root_body) {
            continue;
        }

        let mass = model.body_mass[body_id];

        let omega = Vector3::new(
            data.cvel[body_id][0],
            data.cvel[body_id][1],
            data.cvel[body_id][2],
        );
        let v_origin = Vector3::new(
            data.cvel[body_id][3],
            data.cvel[body_id][4],
            data.cvel[body_id][5],
        );

        // Shift velocity from body origin (xpos) to body COM (xipos)
        let com_offset = data.xipos[body_id] - data.xpos[body_id];
        let v_com = v_origin + omega.cross(&com_offset);

        // Orbital angular momentum: L_orbital = m * (xipos - subtree_com) x v_com
        let r = data.xipos[body_id] - com;
        angmom += mass * r.cross(&v_com);
        let inertia = model.body_inertia[body_id];
        let xi_mat = data.ximat[body_id];
        // I_world * omega = R * diag(I) * R^T * omega
        let omega_local = xi_mat.transpose() * omega;
        let i_omega_local = Vector3::new(
            inertia.x * omega_local.x,
            inertia.y * omega_local.y,
            inertia.z * omega_local.z,
        );
        angmom += xi_mat * i_omega_local;
    }

    angmom
}
