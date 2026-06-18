//! Joint visitor pattern and motion subspace computation.
//!
//! Provides the single source of truth for joint iteration via
//! [`Model::visit_joints()`] and the motion subspace matrix S used by
//! CRBA, RNE, and analytical derivatives.

use crate::types::{Data, MjJointType, Model};

/// Context passed to joint visitors with pre-computed addresses and metadata.
///
/// This provides all the commonly-needed information about a joint in one place,
/// avoiding repeated lookups of `jnt_dof_adr`, `jnt_qpos_adr`, etc.
#[derive(Debug, Clone, Copy)]
pub struct JointContext {
    /// Joint index in model arrays.
    pub jnt_id: usize,
    /// Joint type (Hinge, Slide, Ball, Free).
    pub jnt_type: MjJointType,
    /// Starting index in qvel/qacc/qfrc arrays (DOF address).
    pub dof_adr: usize,
    /// Starting index in qpos array (position address).
    pub qpos_adr: usize,
    /// Number of velocity DOFs for this joint.
    pub nv: usize,
    /// Number of position coordinates for this joint.
    pub nq: usize,
}

/// Visitor trait for operations that iterate over joints.
///
/// This is the **single source of truth** for joint iteration. All code that
/// processes joints should use `Model::visit_joints()` with this trait to ensure:
///
/// 1. **Consistency**: When a new joint type is added, the compiler forces updates
/// 2. **Correctness**: Address computations are centralized, not copy-pasted
/// 3. **Performance**: Addresses are computed once per joint, not per operation
///
/// # Example
///
/// ```ignore
/// struct MyVisitor<'a> {
///     model: &'a Model,
///     data: &'a mut Data,
/// }
///
/// impl JointVisitor for MyVisitor<'_> {
///     fn visit_hinge(&mut self, ctx: JointContext) {
///         // Process hinge joint at ctx.dof_adr, ctx.qpos_adr
///     }
///     // ... other methods have default no-op implementations
/// }
///
/// model.visit_joints(&mut MyVisitor { model, data });
/// ```
pub trait JointVisitor {
    /// Called for each Hinge joint (1 DOF revolute).
    ///
    /// Default: no-op. Override if your visitor needs to process hinge joints.
    #[inline]
    fn visit_hinge(&mut self, _ctx: JointContext) {}

    /// Called for each Slide joint (1 DOF prismatic).
    ///
    /// Default: no-op. Override if your visitor needs to process slide joints.
    #[inline]
    fn visit_slide(&mut self, _ctx: JointContext) {}

    /// Called for each Ball joint (3 DOF spherical, quaternion orientation).
    ///
    /// Default: no-op. Override if your visitor needs to process ball joints.
    #[inline]
    fn visit_ball(&mut self, _ctx: JointContext) {}

    /// Called for each Free joint (6 DOF floating base).
    ///
    /// Default: no-op. Override if your visitor needs to process free joints.
    #[inline]
    fn visit_free(&mut self, _ctx: JointContext) {}
}

impl Model {
    /// Iterate over all joints with the visitor pattern.
    ///
    /// This is the **single source of truth** for joint iteration. Use this method
    /// instead of manually iterating with `for jnt_id in 0..self.njnt` to ensure:
    ///
    /// - Consistent address computation across the codebase
    /// - Automatic handling of new joint types (compiler enforces trait updates)
    /// - Centralized joint metadata in `JointContext`
    ///
    /// # Example
    ///
    /// ```ignore
    /// struct MyVisitor { /* ... */ }
    ///
    /// impl JointVisitor for MyVisitor {
    ///     fn visit_hinge(&mut self, ctx: JointContext) {
    ///         println!("Hinge joint {} at DOF {}", ctx.jnt_id, ctx.dof_adr);
    ///     }
    /// }
    ///
    /// model.visit_joints(&mut MyVisitor { /* ... */ });
    /// ```
    #[inline]
    pub fn visit_joints<V: JointVisitor>(&self, visitor: &mut V) {
        for jnt_id in 0..self.njnt {
            let jnt_type = self.jnt_type[jnt_id];
            let ctx = JointContext {
                jnt_id,
                jnt_type,
                dof_adr: self.jnt_dof_adr[jnt_id],
                qpos_adr: self.jnt_qpos_adr[jnt_id],
                nv: jnt_type.nv(),
                nq: jnt_type.nq(),
            };

            match jnt_type {
                MjJointType::Hinge => visitor.visit_hinge(ctx),
                MjJointType::Slide => visitor.visit_slide(ctx),
                MjJointType::Ball => visitor.visit_ball(ctx),
                MjJointType::Free => visitor.visit_free(ctx),
            }
        }
    }
}

/// Compute the joint motion subspace matrix S (6 x nv).
///
/// S maps joint velocity to spatial velocity in world frame:
/// v_spatial = S * qdot
///
/// Format: rows 0-2 = angular velocity, rows 3-5 = linear velocity
// Paired joint-frame identifiers (pos_a/pos_b, axis_a/axis_b) are intentionally similar.
#[allow(clippy::similar_names)]
#[allow(clippy::inline_always)] // Hot path in CRBA/RNE - profiling shows inlining improves debug performance
#[inline(always)]
pub(crate) fn joint_motion_subspace(
    model: &Model,
    data: &Data,
    jnt_id: usize,
) -> nalgebra::SMatrix<f64, 6, 6> {
    let body_id = model.jnt_body[jnt_id];
    let jnt_type = model.jnt_type[jnt_id];
    let nv = jnt_type.nv();

    let mut s = nalgebra::SMatrix::<f64, 6, 6>::zeros();

    match jnt_type {
        MjJointType::Hinge => {
            // Revolute joint: S = [axis; axis × r]^T where r is from joint to
            // body origin. Both axis and anchor are read in the PARTIAL frame
            // (computed in forward kinematics) — for a multi-joint body the
            // later joints must not rotate an earlier joint's axis/anchor.
            let axis_world = data.xaxis[jnt_id];
            let r = data.xpos[body_id] - data.xanchor[jnt_id];

            // Angular part (rows 0-2)
            s[(0, 0)] = axis_world.x;
            s[(1, 0)] = axis_world.y;
            s[(2, 0)] = axis_world.z;

            // Linear part (rows 3-5) = axis × r
            let lin = axis_world.cross(&r);
            s[(3, 0)] = lin.x;
            s[(4, 0)] = lin.y;
            s[(5, 0)] = lin.z;
        }
        MjJointType::Slide => {
            // Prismatic joint: S = [0; axis]^T (partial-frame axis).
            let axis_world = data.xaxis[jnt_id];
            s[(3, 0)] = axis_world.x;
            s[(4, 0)] = axis_world.y;
            s[(5, 0)] = axis_world.z;
        }
        MjJointType::Ball => {
            // Ball joint: 3 angular DOFs in body-local frame.
            // qvel is body-frame angular velocity; S maps body-frame → world-frame.
            // S = [R; 0] where R = xquat rotation matrix.
            // Matches MuJoCo's cdof: axis rotated to world frame.
            //
            // `xquat[body]` is the body's FINAL orientation, which is the
            // correct subspace frame only when no later joint on the same body
            // rotates it — i.e. this ball is the last (typically only) joint on
            // its body. Every real/tested model satisfies this; a ball followed
            // by another same-body joint would need a stored partial frame.
            debug_assert!(
                jnt_id + 1 == model.body_jnt_adr[body_id] + model.body_jnt_num[body_id],
                "ball joint {jnt_id} is not the last joint on its body — angular \
                 subspace would over-rotate (see joint_motion_subspace)"
            );
            let rot = data.xquat[body_id].to_rotation_matrix().into_inner();
            for i in 0..3 {
                for j in 0..3 {
                    s[(j, i)] = rot[(j, i)];
                }
            }
        }
        MjJointType::Free => {
            // Free joint: 6 DOFs (3 linear + 3 angular)
            // DOF order in qvel: [vx, vy, vz, ωx, ωy, ωz]
            // A free joint is always the only joint on its body (MuJoCo
            // invariant), so `xquat[body]` is the correct angular frame.
            debug_assert!(
                jnt_id + 1 == model.body_jnt_adr[body_id] + model.body_jnt_num[body_id]
                    && model.body_jnt_num[body_id] == 1,
                "free joint {jnt_id} must be the sole joint on its body"
            );
            // Linear DOFs (0-2): world-frame velocity → world-frame spatial linear
            for i in 0..3 {
                s[(3 + i, i)] = 1.0;
            }
            // Angular DOFs (3-5): body-local angular velocity → world-frame spatial angular
            // S = [R; 0] for the angular block, matching MuJoCo's cdof convention.
            let rot = data.xquat[body_id].to_rotation_matrix().into_inner();
            for i in 0..3 {
                for j in 0..3 {
                    s[(j, 3 + i)] = rot[(j, i)];
                }
            }
        }
    }

    // Only return the columns needed
    let _ = nv; // Used implicitly through jnt_type
    s
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used)]
    use super::joint_motion_subspace;
    use crate::forward::mj_fwd_position;
    use crate::test_fixtures::builders::{add_body, add_hinge_joint, add_slide_joint, finalize};
    use crate::types::Model;
    use nalgebra::Vector3;

    /// Ground truth for a joint DOF's motion subspace column: the FD geometric
    /// Jacobian of the body pose — the spatial velocity (ω, v) of the body
    /// frame when only that one DOF moves. The motion subspace IS that
    /// Jacobian column, so this validates the partial-frame construction
    /// without needing an external reference (e.g. MuJoCo).
    fn assert_subspace_matches_fd(
        model: &Model,
        qpos: &[f64],
        // (jnt_id, qpos_adr, body_id) for each scalar DOF to check
        dofs: &[(usize, usize, usize)],
    ) {
        let mut data = model.make_data();
        data.qpos.as_mut_slice()[..qpos.len()].copy_from_slice(qpos);
        mj_fwd_position(model, &mut data);

        let eps = 1e-7;
        for &(jnt_id, qpos_adr, body_id) in dofs {
            let s = joint_motion_subspace(model, &data, jnt_id);
            let analytic_ang = Vector3::new(s[(0, 0)], s[(1, 0)], s[(2, 0)]);
            let analytic_lin = Vector3::new(s[(3, 0)], s[(4, 0)], s[(5, 0)]);

            let mut dp = data.clone();
            dp.qpos[qpos_adr] += eps;
            mj_fwd_position(model, &mut dp);
            let fd_lin = (dp.xpos[body_id] - data.xpos[body_id]) / eps;
            let dq = dp.xquat[body_id] * data.xquat[body_id].inverse();
            let fd_ang = dq.scaled_axis() / eps;

            assert!(
                (analytic_ang - fd_ang).norm() < 1e-6,
                "joint {jnt_id} angular subspace {analytic_ang:?} != FD {fd_ang:?}"
            );
            assert!(
                (analytic_lin - fd_lin).norm() < 1e-6,
                "joint {jnt_id} linear subspace {analytic_lin:?} != FD {fd_lin:?}"
            );
        }
    }

    fn link(model: &mut Model, parent: usize, name: &str, pos: Vector3<f64>) -> usize {
        add_body(
            model,
            parent,
            name,
            pos,
            1.0,
            Vector3::new(0.02, 0.03, 0.04),
            Vector3::new(0.01, -0.02, -0.15),
        )
    }

    /// Two non-parallel hinges on ONE body: the earlier joint's axis must NOT
    /// be rotated by the later joint's DOF (the `joint_motion_subspace` bug
    /// fixed by reading the partial-frame `xaxis`).
    #[test]
    fn multi_hinge_body_subspace_matches_fd() {
        let ax0 = Vector3::new(1.0, 0.3, 0.0).normalize();
        let ax1 = Vector3::new(0.2, 1.0, 0.1).normalize();
        let mut m = Model::empty();
        let b = link(&mut m, 0, "l0", Vector3::zeros());
        let j0 = add_hinge_joint(&mut m, b, "h0", ax0, 0.0, 0.0, 0.0, false, (-3.0, 3.0));
        let j1 = add_hinge_joint(&mut m, b, "h1", ax1, 0.0, 0.0, 0.0, false, (-3.0, 3.0));
        finalize(&mut m);
        assert_subspace_matches_fd(&m, &[0.3, -0.35], &[(j0, 0, b), (j1, 1, b)]);
    }

    /// Slide-then-hinge on ONE body: the slide axis must not rotate with the
    /// later hinge, and the hinge lever must use the partial-frame anchor.
    #[test]
    fn slide_then_hinge_body_subspace_matches_fd() {
        let ax0 = Vector3::new(1.0, 0.2, 0.0).normalize();
        let ax1 = Vector3::new(0.1, 1.0, 0.3).normalize();
        let mut m = Model::empty();
        let b = link(&mut m, 0, "l0", Vector3::zeros());
        let j0 = add_slide_joint(&mut m, b, "s0", ax0, 0.0, 0.0, 0.0);
        let j1 = add_hinge_joint(&mut m, b, "h0", ax1, 0.0, 0.0, 0.0, false, (-3.0, 3.0));
        finalize(&mut m);
        assert_subspace_matches_fd(&m, &[0.2, -0.3], &[(j0, 0, b), (j1, 1, b)]);
    }

    /// Single offset-pivot hinge (`jnt_pos ≠ 0`): the linear lever `â × r` must
    /// use the partial-frame anchor. (Single-joint, so this also guards that
    /// the new partial-frame path is byte-equivalent to the old final-frame
    /// path when there are no later same-body joints.)
    #[test]
    fn offset_pivot_hinge_subspace_matches_fd() {
        let ax0 = Vector3::new(1.0, 0.3, 0.0).normalize();
        let mut m = Model::empty();
        let b = link(&mut m, 0, "l0", Vector3::zeros());
        let j = add_hinge_joint(&mut m, b, "h0", ax0, 0.0, 0.0, 0.0, false, (-3.0, 3.0));
        m.jnt_pos[j] = Vector3::new(0.08, -0.05, 0.06);
        finalize(&mut m);
        assert_subspace_matches_fd(&m, &[0.5], &[(j, 0, b)]);
    }

    /// The forward velocity stage (`velocity.rs`) builds the body spatial
    /// velocity from its OWN copy of the motion subspace; it must stay
    /// consistent with `joint_motion_subspace` (the original bug was these two
    /// drifting apart on a multi-joint body). Guards `cvel[b] == Σ_j S_j·qvel_j`.
    #[test]
    fn forward_cvel_matches_joint_subspace() {
        let ax0 = Vector3::new(1.0, 0.3, 0.0).normalize();
        let ax1 = Vector3::new(0.2, 1.0, 0.1).normalize();
        let mut m = Model::empty();
        let b = link(&mut m, 0, "l0", Vector3::zeros());
        add_hinge_joint(&mut m, b, "h0", ax0, 0.0, 0.0, 0.0, false, (-3.0, 3.0));
        add_hinge_joint(&mut m, b, "h1", ax1, 0.0, 0.0, 0.0, false, (-3.0, 3.0));
        finalize(&mut m);

        let mut data = m.make_data();
        data.qpos.as_mut_slice().copy_from_slice(&[0.3, -0.35]);
        data.qvel.as_mut_slice().copy_from_slice(&[0.8, -0.6]);
        data.forward(&m).expect("forward");

        let mut expected = nalgebra::Vector6::zeros();
        for jid in 0..m.njnt {
            let s = joint_motion_subspace(&m, &data, jid);
            let da = m.jnt_dof_adr[jid];
            for d in 0..m.jnt_type[jid].nv() {
                expected += s.column(d) * data.qvel[da + d];
            }
        }
        assert!(
            (data.cvel[b] - expected).norm() < 1e-12,
            "cvel {:?} != Σ S·qvel {:?}",
            data.cvel[b].as_slice(),
            expected.as_slice()
        );
    }
}
