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

// =============================================================================
// Unit tests — DT-75: contact Jacobian free-joint body-frame axes fix
// (relocated from monolith, Phase 12)
// =============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::similar_names)]
mod contact_jac_free_joint_tests {
    use super::*;
    use crate::constraint::impedance::{DEFAULT_SOLIMP, DEFAULT_SOLREF};
    use crate::forward::mj_fwd_position;
    use crate::jacobian::mj_jac;
    use crate::types::{MjJointType, Model};
    use approx::assert_relative_eq;
    use nalgebra::{DVector, UnitQuaternion, Vector3};
    use std::f64::consts::{FRAC_PI_4, FRAC_PI_6};

    // =========================================================================
    // Helpers
    // =========================================================================

    /// Single free-joint body with minimal geom fields for contact tests.
    ///
    /// Returns identity orientation with FK already run. Caller can write a
    /// non-identity quaternion into `data.qpos[3..7]` and re-run
    /// `mj_fwd_position` for non-trivial orientations.
    fn make_free_body_contact_model() -> (Model, Data) {
        let mut model = Model::empty();

        model.nbody = 2;
        model.body_parent = vec![0, 0];
        model.body_rootid = vec![0, 1];
        model.body_jnt_adr = vec![0, 0];
        model.body_jnt_num = vec![0, 1];
        model.body_dof_adr = vec![0, 0];
        model.body_dof_num = vec![0, 6];
        model.body_pos = vec![Vector3::zeros(); 2];
        model.body_quat = vec![UnitQuaternion::identity(); 2];
        model.body_ipos = vec![Vector3::zeros(); 2];
        model.body_iquat = vec![UnitQuaternion::identity(); 2];
        model.body_mass = vec![0.0, 1.0];
        model.body_inertia = vec![Vector3::zeros(), Vector3::new(0.01, 0.01, 0.01)];
        model.body_name = vec![Some("world".into()), Some("b1".into())];
        model.body_subtreemass = vec![1.0, 1.0];

        model.njnt = 1;
        model.nq = 7;
        model.nv = 6;
        model.jnt_type = vec![MjJointType::Free];
        model.jnt_body = vec![1];
        model.jnt_qpos_adr = vec![0];
        model.jnt_dof_adr = vec![0];
        model.jnt_pos = vec![Vector3::zeros()];
        model.jnt_axis = vec![Vector3::z()];
        model.jnt_limited = vec![false];
        model.jnt_range = vec![(0.0, 0.0)];
        model.jnt_stiffness = vec![0.0];
        model.jnt_springref = vec![0.0];
        model.jnt_damping = vec![0.0];
        model.jnt_armature = vec![0.0];
        model.jnt_solref = vec![[0.02, 1.0]];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.jnt_name = vec![Some("j1".into())];

        model.dof_body = vec![1; 6];
        model.dof_jnt = vec![0; 6];
        model.dof_parent = vec![None; 6];
        model.dof_armature = vec![0.0; 6];
        model.dof_damping = vec![0.0; 6];
        model.dof_frictionloss = vec![0.0; 6];

        model.nsite = 0;
        model.site_body = vec![];
        model.site_pos = vec![];
        model.site_quat = vec![];
        model.site_type = vec![];
        model.site_size = vec![];
        model.site_name = vec![];

        // Geom fields: compute_contact_jacobian only reads geom_body, but
        // mj_fwd_position reads geom_pos/geom_quat to compute geom world poses.
        model.ngeom = 2;
        model.geom_body = vec![0, 1]; // geom 0 → world, geom 1 → free body
        model.geom_pos = vec![Vector3::zeros(); 2];
        model.geom_quat = vec![UnitQuaternion::identity(); 2];
        model.body_geom_adr = vec![0, 1];
        model.body_geom_num = vec![1, 1];

        // Identity quaternion for the free joint
        model.qpos0 = DVector::zeros(7);
        model.qpos0[3] = 1.0; // w component
        model.timestep = 0.001;

        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_qld_csr_metadata();

        let mut data = model.make_data();
        mj_fwd_position(&model, &mut data);

        (model, data)
    }

    /// Two free-joint bodies (no kinematic chain between them) with geom fields.
    ///
    /// nbody=3, njnt=2, nv=12, nq=14, ngeom=3. Both free bodies are children of
    /// the world body. Caller sets orientations in qpos and calls mj_fwd_position.
    fn make_two_free_body_contact_model() -> (Model, Data) {
        let mut model = Model::empty();

        // --- Topology: 3 bodies (world + 2 free), no kinematic chain ---
        model.nbody = 3;
        model.body_parent = vec![0, 0, 0];
        model.body_rootid = vec![0, 1, 2];
        model.body_jnt_adr = vec![0, 0, 1];
        model.body_jnt_num = vec![0, 1, 1];
        model.body_dof_adr = vec![0, 0, 6];
        model.body_dof_num = vec![0, 6, 6];
        model.body_pos = vec![Vector3::zeros(); 3];
        model.body_quat = vec![UnitQuaternion::identity(); 3];
        model.body_ipos = vec![Vector3::zeros(); 3];
        model.body_iquat = vec![UnitQuaternion::identity(); 3];
        model.body_mass = vec![0.0, 1.0, 1.0];
        model.body_inertia = vec![
            Vector3::zeros(),
            Vector3::new(0.01, 0.01, 0.01),
            Vector3::new(0.01, 0.01, 0.01),
        ];
        model.body_name = vec![Some("world".into()), Some("b1".into()), Some("b2".into())];
        model.body_subtreemass = vec![2.0, 1.0, 1.0];

        // --- 2 free joints ---
        model.njnt = 2;
        model.nq = 14; // 7 + 7
        model.nv = 12; // 6 + 6
        model.jnt_type = vec![MjJointType::Free; 2];
        model.jnt_body = vec![1, 2];
        model.jnt_qpos_adr = vec![0, 7];
        model.jnt_dof_adr = vec![0, 6];
        model.jnt_axis = vec![Vector3::z(); 2];
        model.jnt_pos = vec![Vector3::zeros(); 2];
        model.jnt_limited = vec![false; 2];
        model.jnt_range = vec![(0.0, 0.0); 2];
        model.jnt_stiffness = vec![0.0; 2];
        model.jnt_springref = vec![0.0; 2];
        model.jnt_damping = vec![0.0; 2];
        model.jnt_armature = vec![0.0; 2];
        model.jnt_solref = vec![[0.02, 1.0]; 2];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; 2];
        model.jnt_name = vec![Some("j1".into()), Some("j2".into())];

        // --- DOF metadata (12 DOFs: 6 per free joint) ---
        model.dof_body = vec![1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2];
        model.dof_jnt = vec![0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1];
        model.dof_parent = vec![None; 12];
        model.dof_armature = vec![0.0; 12];
        model.dof_damping = vec![0.0; 12];
        model.dof_frictionloss = vec![0.0; 12];

        // --- No sites needed for contact tests ---
        model.nsite = 0;
        model.site_body = vec![];
        model.site_pos = vec![];
        model.site_quat = vec![];
        model.site_type = vec![];
        model.site_size = vec![];
        model.site_name = vec![];

        // --- Geoms: compute_contact_jacobian reads geom_body; mj_fwd_position
        // reads geom_pos/geom_quat to compute geom world poses ---
        model.ngeom = 3;
        model.geom_body = vec![0, 1, 2];
        model.geom_pos = vec![Vector3::zeros(); 3];
        model.geom_quat = vec![UnitQuaternion::identity(); 3];
        model.body_geom_adr = vec![0, 1, 2];
        model.body_geom_num = vec![1, 1, 1];

        // --- qpos0: identity quaternions for both free joints ---
        // qpos layout: [x1,y1,z1, w1,i1,j1,k1, x2,y2,z2, w2,i2,j2,k2]
        model.qpos0 = DVector::zeros(14);
        model.qpos0[3] = 1.0; // body 1: identity quaternion w-component
        model.qpos0[10] = 1.0; // body 2: identity quaternion w-component
        model.timestep = 0.001;

        // --- Ancestor metadata (required by compute_qld_csr_metadata → make_data) ---
        model.body_ancestor_joints = vec![vec![]; 3];
        model.body_ancestor_mask = vec![vec![]; 3];
        model.compute_ancestors();
        model.compute_qld_csr_metadata();

        let data = model.make_data();
        (model, data)
    }

    // =========================================================================
    // 5a — contact_jac_free_body_frame_vs_world_frame
    // =========================================================================

    /// Verify that free-joint angular DOF columns use body-frame axes (R·eᵢ),
    /// not world-frame unit vectors. At 45° about Z, R·e₀ ≠ e₀, so world-frame
    /// would produce measurably wrong values.
    #[test]
    fn contact_jac_free_body_frame_vs_world_frame() {
        let (model, mut data) = make_free_body_contact_model();

        // Rotate body to 45° about Z
        let q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_4);
        data.qpos[3] = q.w;
        data.qpos[4] = q.i;
        data.qpos[5] = q.j;
        data.qpos[6] = q.k;
        mj_fwd_position(&model, &mut data);

        let contact = Contact::new(
            Vector3::new(0.3, 0.2, 0.1), // pos
            Vector3::new(0.0, 0.0, 1.0), // normal: +Z
            0.01,                        // depth
            0,                           // geom1 (world geom)
            1,                           // geom2 (free body geom)
            0.5,                         // friction → dim=3
        );
        let j = compute_contact_jacobian(&model, &data, &contact);
        assert_eq!(j.nrows(), 3);
        assert_eq!(j.ncols(), 6);

        let rot = data.xquat[1].to_rotation_matrix();
        let r = contact.pos - data.xpos[1];
        let normal = contact.normal;

        // Verify angular DOF columns (3, 4, 5) use body-frame axes
        for i in 0..3 {
            let omega = rot * Vector3::ith(i, 1.0);
            let expected = normal.dot(&omega.cross(&r));
            assert_relative_eq!(j[(0, 3 + i)], expected, epsilon = 1e-12);
        }

        // Sanity: at 45° about Z, R·e₀ ≠ e₀ — world-frame would give wrong answer
        let body_e0 = rot * Vector3::x();
        assert!(
            (body_e0 - Vector3::x()).norm() > 0.1,
            "R·e₀ should differ from e₀ at 45°"
        );
    }

    // =========================================================================
    // 5b — contact_jac_all_rows_match_mj_jac_projection
    // =========================================================================

    /// Verify all 3 translational rows (normal + 2 tangents) of the contact
    /// Jacobian match the relative body Jacobian projected along each direction.
    #[test]
    fn contact_jac_all_rows_match_mj_jac_projection() {
        let (model, mut data) = make_free_body_contact_model();

        // Rotate body to 45° about Z
        let q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_4);
        data.qpos[3] = q.w;
        data.qpos[4] = q.i;
        data.qpos[5] = q.j;
        data.qpos[6] = q.k;
        mj_fwd_position(&model, &mut data);

        let contact = Contact::new(
            Vector3::new(0.3, 0.2, 0.1),
            Vector3::new(0.0, 0.0, 1.0),
            0.01,
            0,
            1,
            0.5,
        );
        let j = compute_contact_jacobian(&model, &data, &contact);

        let body1 = model.geom_body[contact.geom1]; // world (body 0)
        let body2 = model.geom_body[contact.geom2]; // free body

        let (jacp2, _) = mj_jac(&model, &data, body2, &contact.pos);
        let (jacp1, _) = mj_jac(&model, &data, body1, &contact.pos);

        let directions = [contact.normal, contact.frame[0], contact.frame[1]];
        let nv = model.nv;
        for (row, dir) in directions.iter().enumerate() {
            for dof in 0..nv {
                let col2 = Vector3::new(jacp2[(0, dof)], jacp2[(1, dof)], jacp2[(2, dof)]);
                let col1 = Vector3::new(jacp1[(0, dof)], jacp1[(1, dof)], jacp1[(2, dof)]);
                let expected = dir.dot(&(col2 - col1));
                assert_relative_eq!(j[(row, dof)], expected, epsilon = 1e-12);
            }
        }
    }

    // =========================================================================
    // 5c — angular_jac_free_body_frame (torsional + rolling, condim=6)
    // =========================================================================

    /// Verify all 6 rows (normal + 2 tangent + torsional + 2 rolling) of a
    /// condim=6 contact match the relative translational and rotational
    /// Jacobians from mj_jac.
    #[test]
    fn angular_jac_free_body_frame() {
        let (model, mut data) = make_free_body_contact_model();

        // Rotate body to 45° about Z
        let q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_4);
        data.qpos[3] = q.w;
        data.qpos[4] = q.i;
        data.qpos[5] = q.j;
        data.qpos[6] = q.k;
        mj_fwd_position(&model, &mut data);

        let contact = Contact::with_condim(
            Vector3::new(0.3, 0.2, 0.1),
            Vector3::new(0.0, 0.0, 1.0),
            0.01,
            0,
            1,
            0.5,   // sliding friction
            0.01,  // torsional friction
            0.005, // rolling friction
            6_i32,
            DEFAULT_SOLREF,
            DEFAULT_SOLIMP,
        );
        let j = compute_contact_jacobian(&model, &data, &contact);
        assert_eq!(j.nrows(), 6);
        assert_eq!(j.ncols(), 6);

        let body1 = model.geom_body[contact.geom1];
        let body2 = model.geom_body[contact.geom2];

        let (jacp2, jacr2) = mj_jac(&model, &data, body2, &contact.pos);
        let (jacp1, jacr1) = mj_jac(&model, &data, body1, &contact.pos);

        let nv = model.nv;

        // Rows 0–2 (normal + tangents): relative translational Jacobian projection
        let trans_dirs = [contact.normal, contact.frame[0], contact.frame[1]];
        for (row, dir) in trans_dirs.iter().enumerate() {
            for dof in 0..nv {
                let col2 = Vector3::new(jacp2[(0, dof)], jacp2[(1, dof)], jacp2[(2, dof)]);
                let col1 = Vector3::new(jacp1[(0, dof)], jacp1[(1, dof)], jacp1[(2, dof)]);
                let expected = dir.dot(&(col2 - col1));
                assert_relative_eq!(j[(row, dof)], expected, epsilon = 1e-12);
            }
        }

        // Row 3 (torsional): relative rotational Jacobian projected along normal
        // Rows 4–5 (rolling): relative rotational Jacobian projected along tangents
        let rot_dirs = [contact.normal, contact.frame[0], contact.frame[1]];
        for (i, dir) in rot_dirs.iter().enumerate() {
            let row = 3 + i;
            for dof in 0..nv {
                let col2 = Vector3::new(jacr2[(0, dof)], jacr2[(1, dof)], jacr2[(2, dof)]);
                let col1 = Vector3::new(jacr1[(0, dof)], jacr1[(1, dof)], jacr1[(2, dof)]);
                let expected = dir.dot(&(col2 - col1));
                assert_relative_eq!(j[(row, dof)], expected, epsilon = 1e-12);
            }
        }
    }

    // =========================================================================
    // 5d — contact_jac_two_free_bodies (strongest: both bodies non-identity)
    // =========================================================================

    /// Two free-joint bodies at different non-identity orientations, condim=6
    /// contact between them. Validates all 6 rows × 12 DOFs against mj_jac.
    #[test]
    fn contact_jac_two_free_bodies() {
        let (model, mut data) = make_two_free_body_contact_model();

        // Body 1: 45° about Z
        let q1 = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_4);
        data.qpos[3] = q1.w;
        data.qpos[4] = q1.i;
        data.qpos[5] = q1.j;
        data.qpos[6] = q1.k;
        // Body 2: 30° about X
        let q2 = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), FRAC_PI_6);
        data.qpos[10] = q2.w;
        data.qpos[11] = q2.i;
        data.qpos[12] = q2.j;
        data.qpos[13] = q2.k;
        mj_fwd_position(&model, &mut data);

        let contact = Contact::with_condim(
            Vector3::new(0.3, 0.2, 0.1),
            Vector3::new(0.0, 0.0, 1.0),
            0.01,
            1, // geom1 on body 1
            2, // geom2 on body 2
            0.5,
            0.01,
            0.005,
            6_i32,
            DEFAULT_SOLREF,
            DEFAULT_SOLIMP,
        );
        let j = compute_contact_jacobian(&model, &data, &contact);
        assert_eq!(j.nrows(), 6);
        assert_eq!(j.ncols(), 12);

        let body1 = model.geom_body[contact.geom1]; // body 1
        let body2 = model.geom_body[contact.geom2]; // body 2

        let (jacp1, jacr1) = mj_jac(&model, &data, body1, &contact.pos);
        let (jacp2, jacr2) = mj_jac(&model, &data, body2, &contact.pos);

        let nv = model.nv;

        // Rows 0–2: relative translational Jacobian
        let trans_dirs = [contact.normal, contact.frame[0], contact.frame[1]];
        for (row, dir) in trans_dirs.iter().enumerate() {
            for dof in 0..nv {
                let col2 = Vector3::new(jacp2[(0, dof)], jacp2[(1, dof)], jacp2[(2, dof)]);
                let col1 = Vector3::new(jacp1[(0, dof)], jacp1[(1, dof)], jacp1[(2, dof)]);
                let expected = dir.dot(&(col2 - col1));
                assert_relative_eq!(j[(row, dof)], expected, epsilon = 1e-12);
            }
        }

        // Rows 3–5: relative rotational Jacobian
        let rot_dirs = [contact.normal, contact.frame[0], contact.frame[1]];
        for (i, dir) in rot_dirs.iter().enumerate() {
            let row = 3 + i;
            for dof in 0..nv {
                let col2 = Vector3::new(jacr2[(0, dof)], jacr2[(1, dof)], jacr2[(2, dof)]);
                let col1 = Vector3::new(jacr1[(0, dof)], jacr1[(1, dof)], jacr1[(2, dof)]);
                let expected = dir.dot(&(col2 - col1));
                assert_relative_eq!(j[(row, dof)], expected, epsilon = 1e-12);
            }
        }
    }

    // =========================================================================
    // 5e — contact_jac_free_identity_unchanged (regression)
    // =========================================================================

    /// At identity orientation, body-frame axes coincide with world-frame axes.
    /// Verify the contact Jacobian produces the same values as both the old
    /// world-frame formula and the mj_jac projection.
    #[test]
    fn contact_jac_free_identity_unchanged() {
        let (model, data) = make_free_body_contact_model();

        // Confirm identity orientation
        let rot = data.xquat[1].to_rotation_matrix();
        for i in 0..3 {
            let body_axis = rot * Vector3::ith(i, 1.0);
            let world_axis = Vector3::ith(i, 1.0);
            assert_relative_eq!(body_axis, world_axis, epsilon = 1e-15);
        }

        let contact = Contact::new(
            Vector3::new(0.3, 0.2, 0.1),
            Vector3::new(0.0, 0.0, 1.0),
            0.01,
            0,
            1,
            0.5,
        );
        let j = compute_contact_jacobian(&model, &data, &contact);

        // Cross-check against mj_jac projection
        let body2 = model.geom_body[contact.geom2];
        let (jacp2, _) = mj_jac(&model, &data, body2, &contact.pos);

        let directions = [contact.normal, contact.frame[0], contact.frame[1]];
        let nv = model.nv;
        for (row, dir) in directions.iter().enumerate() {
            for dof in 0..nv {
                let col = Vector3::new(jacp2[(0, dof)], jacp2[(1, dof)], jacp2[(2, dof)]);
                let expected = dir.dot(&col);
                assert_relative_eq!(j[(row, dof)], expected, epsilon = 1e-12);
            }
        }
    }

    // =========================================================================
    // 5f — contact_jac_free_frictionless (condim=1)
    // =========================================================================

    /// Frictionless contact (condim=1, dim=1): only the normal row, no tangent,
    /// torsional, or rolling rows. Verify angular DOFs use body-frame axes.
    #[test]
    fn contact_jac_free_frictionless() {
        let (model, mut data) = make_free_body_contact_model();

        // Rotate body to 45° about Z (non-identity to make body-frame meaningful)
        let q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_4);
        data.qpos[3] = q.w;
        data.qpos[4] = q.i;
        data.qpos[5] = q.j;
        data.qpos[6] = q.k;
        mj_fwd_position(&model, &mut data);

        let contact = Contact::with_condim(
            Vector3::new(0.3, 0.2, 0.1),
            Vector3::z(),
            0.01,
            0,
            1,
            0.0, // no sliding friction
            0.0, // no torsional
            0.0, // no rolling
            1_i32,
            DEFAULT_SOLREF,
            DEFAULT_SOLIMP,
        );
        let j = compute_contact_jacobian(&model, &data, &contact);
        assert_eq!(j.nrows(), 1);
        assert_eq!(j.ncols(), 6);

        // Cross-check the single normal row against mj_jac projection
        let body2 = model.geom_body[contact.geom2];
        let (jacp2, _) = mj_jac(&model, &data, body2, &contact.pos);
        let normal = contact.normal;

        for dof in 0..model.nv {
            let col = Vector3::new(jacp2[(0, dof)], jacp2[(1, dof)], jacp2[(2, dof)]);
            let expected = normal.dot(&col);
            assert_relative_eq!(j[(0, dof)], expected, epsilon = 1e-12);
        }
    }
}
