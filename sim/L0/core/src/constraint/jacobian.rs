//! Contact constraint Jacobian construction.
//!
//! Computes the contact Jacobian rows for rigid-body and flex contacts,
//! including normal and tangential (friction) directions.
//! Corresponds to the Jacobian computation in MuJoCo's
//! `engine_core_constraint.c` (§15.2).

use nalgebra::{DMatrix, Vector3};

use crate::types::{Contact, Data, MjJointType, Model};

/// Compute the contact Jacobian for a flex-rigid contact.
///
/// The flex vertex side has a trivial Jacobian: the contact frame direction is
/// directly projected onto the vertex's 3 translational DOF columns (no kinematic
/// chain traversal). The rigid body side uses the standard `add_body_jacobian` pattern.
///
/// Convention: `contact.flex_vertex` = vertex index, `contact.geom1/geom2` = rigid geom.
/// Normal points FROM rigid surface TOWARD flex vertex (outward from rigid geom).
pub fn compute_flex_contact_jacobian(
    model: &Model,
    data: &Data,
    contact: &Contact,
    vertex_idx: usize,
) -> DMatrix<f64> {
    let nv = model.nv;
    let dim = contact.dim;
    let normal = contact.normal;
    let tangent1 = contact.frame[0];
    let tangent2 = contact.frame[1];

    let mut j = DMatrix::zeros(dim, nv);

    // Flex vertex side: trivial Jacobian (identity on DOF columns).
    // The vertex's 3 translational DOFs map directly to Cartesian velocity.
    // (§27F) Pinned vertices have no DOFs — Jacobian columns are all zero (immovable).
    let dof_base = model.flexvert_dofadr[vertex_idx];

    // Helper: project direction onto vertex DOFs with given sign
    let add_vertex_jacobian = |j: &mut DMatrix<f64>, row: usize, dir: &Vector3<f64>, sign: f64| {
        if dof_base == usize::MAX {
            return; // Pinned vertex: no DOF columns to fill
        }
        j[(row, dof_base)] += sign * dir.x;
        j[(row, dof_base + 1)] += sign * dir.y;
        j[(row, dof_base + 2)] += sign * dir.z;
    };

    // Rigid body side: standard kinematic chain traversal.
    let rigid_body = model.geom_body[contact.geom2];
    let add_body_jacobian =
        |j: &mut DMatrix<f64>, row: usize, direction: &Vector3<f64>, body_id: usize, sign: f64| {
            if body_id == 0 {
                return;
            }
            let mut current_body = body_id;
            while current_body != 0 {
                let jnt_start = model.body_jnt_adr[current_body];
                let jnt_end = jnt_start + model.body_jnt_num[current_body];
                for jnt_id in jnt_start..jnt_end {
                    let dof_adr = model.jnt_dof_adr[jnt_id];
                    let jnt_body = model.jnt_body[jnt_id];
                    match model.jnt_type[jnt_id] {
                        MjJointType::Hinge => {
                            let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                            let jpos =
                                data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                            let r = contact.pos - jpos;
                            let j_col = axis.cross(&r);
                            j[(row, dof_adr)] += sign * direction.dot(&j_col);
                        }
                        MjJointType::Slide => {
                            let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                            j[(row, dof_adr)] += sign * direction.dot(&axis);
                        }
                        MjJointType::Ball => {
                            let jpos =
                                data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                            let r = contact.pos - jpos;
                            let rot = data.xquat[jnt_body].to_rotation_matrix();
                            for i in 0..3 {
                                let omega_world = rot * Vector3::ith(i, 1.0);
                                let j_col = omega_world.cross(&r);
                                j[(row, dof_adr + i)] += sign * direction.dot(&j_col);
                            }
                        }
                        MjJointType::Free => {
                            j[(row, dof_adr)] += sign * direction.x;
                            j[(row, dof_adr + 1)] += sign * direction.y;
                            j[(row, dof_adr + 2)] += sign * direction.z;

                            // Angular DOFs (3–5): body-frame axes R·eᵢ, lever arm from xpos.
                            let jpos = data.xpos[jnt_body];
                            let r = contact.pos - jpos;
                            let rot = data.xquat[jnt_body].to_rotation_matrix();
                            for i in 0..3 {
                                let omega = rot * Vector3::ith(i, 1.0);
                                j[(row, dof_adr + 3 + i)] += sign * direction.dot(&omega.cross(&r));
                            }
                        }
                    }
                }
                current_body = model.body_parent[current_body];
            }
        };

    // Row 0: normal direction
    // Narrowphase normal points FROM rigid surface TOWARD flex vertex (outward).
    // MuJoCo convention: normal FROM body1 TO body2, body1 gets -1, body2 gets +1.
    // Here: body1 = rigid (normal points away from it), body2 = flex vertex.
    // Vertex (body2) contributes positively, rigid (body1) negatively.
    add_vertex_jacobian(&mut j, 0, &normal, 1.0);
    add_body_jacobian(&mut j, 0, &normal, rigid_body, -1.0);

    // Rows 1-2: tangent directions (sliding friction)
    if dim >= 3 {
        add_vertex_jacobian(&mut j, 1, &tangent1, 1.0);
        add_body_jacobian(&mut j, 1, &tangent1, rigid_body, -1.0);

        add_vertex_jacobian(&mut j, 2, &tangent2, 1.0);
        add_body_jacobian(&mut j, 2, &tangent2, rigid_body, -1.0);
    }

    // Row 3: torsional friction — flex vertices have no angular DOFs, so only
    // the rigid side contributes. The vertex side is zero (no torsion).
    // Rigid is body1, so sign is -1.0 (but friction cost is symmetric/Huber,
    // so sign only affects direction labeling, not correctness).
    if dim >= 4 {
        add_angular_jacobian(&mut j, 3, &normal, rigid_body, -1.0, model, data);
    }

    // Rows 4-5: rolling friction — same reasoning, vertex has no angular DOFs
    if dim >= 6 {
        add_angular_jacobian(&mut j, 4, &tangent1, rigid_body, -1.0, model, data);
        add_angular_jacobian(&mut j, 5, &tangent2, rigid_body, -1.0, model, data);
    }

    j
}

/// Compute the full dim×nv contact Jacobian for a contact point.
///
/// Returns a `contact.dim`×nv matrix where rows depend on contact dimension:
/// - dim=1: Row 0: normal direction
/// - dim=3: Row 0: normal, Row 1-2: tangents (sliding friction)
/// - dim=4: Rows 0-2: as dim=3, Row 3: torsional (spin about normal)
/// - dim=6: Rows 0-3: as dim=4, Rows 4-5: rolling (angular velocity tangent components)
pub fn compute_contact_jacobian(model: &Model, data: &Data, contact: &Contact) -> DMatrix<f64> {
    let nv = model.nv;
    let dim = contact.dim;

    // Flex-rigid contacts: vertex side has trivial Jacobian (identity on DOF columns),
    // rigid side uses the standard kinematic chain traversal.
    if let Some(vi) = contact.flex_vertex {
        return compute_flex_contact_jacobian(model, data, contact, vi);
    }

    let body1 = model.geom_body[contact.geom1];
    let body2 = model.geom_body[contact.geom2];

    // Use the pre-computed contact frame for consistency.
    // contact.frame[] was computed during contact construction via compute_tangent_frame().
    // This ensures the Jacobian rows align exactly with the RHS computation in
    // assemble_contact_system() and force application in mj_fwd_constraint().
    let normal = contact.normal;
    let tangent1 = contact.frame[0];
    let tangent2 = contact.frame[1];

    // Allocate dim×nv Jacobian
    let mut j = DMatrix::zeros(dim, nv);

    // Helper: add body Jacobian contribution for one direction
    let add_body_jacobian =
        |j: &mut DMatrix<f64>, row: usize, direction: &Vector3<f64>, body_id: usize, sign: f64| {
            if body_id == 0 {
                return; // World has no DOFs
            }

            let mut current_body = body_id;
            while current_body != 0 {
                let jnt_start = model.body_jnt_adr[current_body];
                let jnt_end = jnt_start + model.body_jnt_num[current_body];

                for jnt_id in jnt_start..jnt_end {
                    let dof_adr = model.jnt_dof_adr[jnt_id];
                    let jnt_body = model.jnt_body[jnt_id];

                    match model.jnt_type[jnt_id] {
                        MjJointType::Hinge => {
                            let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                            let jpos =
                                data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                            let r = contact.pos - jpos;
                            let j_col = axis.cross(&r);
                            j[(row, dof_adr)] += sign * direction.dot(&j_col);
                        }
                        MjJointType::Slide => {
                            let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                            j[(row, dof_adr)] += sign * direction.dot(&axis);
                        }
                        MjJointType::Ball => {
                            let jpos =
                                data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                            let r = contact.pos - jpos;
                            let rot = data.xquat[jnt_body].to_rotation_matrix();
                            for i in 0..3 {
                                let omega_world = rot * Vector3::ith(i, 1.0);
                                let j_col = omega_world.cross(&r);
                                j[(row, dof_adr + i)] += sign * direction.dot(&j_col);
                            }
                        }
                        MjJointType::Free => {
                            // Linear DOFs (0–2): world-frame identity.
                            j[(row, dof_adr)] += sign * direction.x;
                            j[(row, dof_adr + 1)] += sign * direction.y;
                            j[(row, dof_adr + 2)] += sign * direction.z;

                            // Angular DOFs (3–5): body-frame axes R·eᵢ, lever arm from xpos.
                            let jpos = data.xpos[jnt_body];
                            let r = contact.pos - jpos;
                            let rot = data.xquat[jnt_body].to_rotation_matrix();
                            for i in 0..3 {
                                let omega = rot * Vector3::ith(i, 1.0);
                                j[(row, dof_adr + 3 + i)] += sign * direction.dot(&omega.cross(&r));
                            }
                        }
                    }
                }
                current_body = model.body_parent[current_body];
            }
        };

    // Compute relative velocity J * qvel = v2 - v1 (velocity of body2 relative to body1)
    // Normal points FROM body1 (geom1) TO body2 (geom2).
    // Positive normal velocity = bodies separating (good)
    // Negative normal velocity = bodies approaching (need constraint)
    //
    // Body2 contributes positively (its velocity in +normal direction = separating)
    // Body1 contributes negatively (its velocity in +normal direction = approaching body2)

    // Row 0: normal direction (always present)
    add_body_jacobian(&mut j, 0, &normal, body2, 1.0);
    add_body_jacobian(&mut j, 0, &normal, body1, -1.0);

    // Rows 1-2: tangent directions (dim >= 3: sliding friction)
    if dim >= 3 {
        add_body_jacobian(&mut j, 1, &tangent1, body2, 1.0);
        add_body_jacobian(&mut j, 1, &tangent1, body1, -1.0);

        add_body_jacobian(&mut j, 2, &tangent2, body2, 1.0);
        add_body_jacobian(&mut j, 2, &tangent2, body1, -1.0);
    }

    // Row 3: torsional/spin (dim >= 4: angular velocity about contact normal)
    // This constrains relative spinning about the contact normal
    if dim >= 4 {
        add_angular_jacobian(&mut j, 3, &normal, body2, 1.0, model, data);
        add_angular_jacobian(&mut j, 3, &normal, body1, -1.0, model, data);
    }

    // Rows 4-5: rolling friction (dim = 6: angular velocity in tangent plane)
    // This constrains relative rolling in the tangent directions
    if dim >= 6 {
        add_angular_jacobian(&mut j, 4, &tangent1, body2, 1.0, model, data);
        add_angular_jacobian(&mut j, 4, &tangent1, body1, -1.0, model, data);

        add_angular_jacobian(&mut j, 5, &tangent2, body2, 1.0, model, data);
        add_angular_jacobian(&mut j, 5, &tangent2, body1, -1.0, model, data);
    }

    j
}

/// Add angular Jacobian contribution for a body in a given direction.
///
/// This computes how joint velocities affect angular velocity of the body
/// in the specified direction. Used for torsional and rolling constraints.
#[inline]
pub fn add_angular_jacobian(
    j: &mut DMatrix<f64>,
    row: usize,
    direction: &Vector3<f64>,
    body_id: usize,
    sign: f64,
    model: &Model,
    data: &Data,
) {
    if body_id == 0 {
        return; // World has no DOFs
    }

    let mut current_body = body_id;
    while current_body != 0 {
        let jnt_start = model.body_jnt_adr[current_body];
        let jnt_end = jnt_start + model.body_jnt_num[current_body];

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let jnt_body = model.jnt_body[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge => {
                    // Hinge contributes angular velocity along its axis
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    j[(row, dof_adr)] += sign * direction.dot(&axis);
                }
                MjJointType::Slide => {
                    // Slide joints don't contribute to angular velocity
                }
                MjJointType::Ball => {
                    // Ball joint contributes angular velocity in all 3 axes
                    let rot = data.xquat[jnt_body].to_rotation_matrix();
                    for i in 0..3 {
                        let omega_world = rot * Vector3::ith(i, 1.0);
                        j[(row, dof_adr + i)] += sign * direction.dot(&omega_world);
                    }
                }
                MjJointType::Free => {
                    // Angular DOFs (3–5): body-frame axes, matching Ball case above.
                    let rot = data.xquat[jnt_body].to_rotation_matrix();
                    for i in 0..3 {
                        let omega = rot * Vector3::ith(i, 1.0);
                        j[(row, dof_adr + 3 + i)] += sign * direction.dot(&omega);
                    }
                }
            }
        }
        current_body = model.body_parent[current_body];
    }
}
