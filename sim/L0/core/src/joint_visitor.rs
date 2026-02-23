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
            // Revolute joint: S = [axis; axis × r]^T where r is from joint to body origin
            let axis_world = data.xquat[body_id] * model.jnt_axis[jnt_id];
            let jpos_world = data.xpos[body_id] + data.xquat[body_id] * model.jnt_pos[jnt_id];
            let r = data.xpos[body_id] - jpos_world;

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
            // Prismatic joint: S = [0; axis]^T
            let axis_world = data.xquat[body_id] * model.jnt_axis[jnt_id];
            s[(3, 0)] = axis_world.x;
            s[(4, 0)] = axis_world.y;
            s[(5, 0)] = axis_world.z;
        }
        MjJointType::Ball => {
            // Ball joint: 3 angular DOFs
            // S = [I_3x3; 0_3x3] (angular velocity in world frame)
            for i in 0..3 {
                s[(i, i)] = 1.0;
            }
        }
        MjJointType::Free => {
            // Free joint: 6 DOFs (3 linear + 3 angular)
            // DOF order in qvel: [vx, vy, vz, ωx, ωy, ωz]
            // Linear DOFs map to linear velocity
            for i in 0..3 {
                s[(3 + i, i)] = 1.0; // Linear velocity
            }
            // Angular DOFs map to angular velocity
            for i in 0..3 {
                s[(i, 3 + i)] = 1.0; // Angular velocity
            }
        }
    }

    // Only return the columns needed
    let _ = nv; // Used implicitly through jnt_type
    s
}
