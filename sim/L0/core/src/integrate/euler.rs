//! Euler position integration on SO(3) manifold + quaternion normalization.
//!
//! Implements MuJoCo's position integration step: each joint type gets its
//! appropriate integration (scalar for hinge/slide, quaternion exponential map
//! for ball/free). After integration, quaternions are renormalized to prevent
//! numerical drift.

use nalgebra::{DVector, UnitQuaternion, Vector3};

use crate::joint_visitor::{JointContext, JointVisitor};
use crate::types::{Data, ENABLE_SLEEP, Model, SleepState};

/// Proper position integration that handles quaternions on SO(3) manifold.
pub fn mj_integrate_pos(model: &Model, data: &mut Data, h: f64) {
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
    let mut visitor = PositionIntegrateVisitor {
        qpos: &mut data.qpos,
        qvel: &data.qvel,
        h,
        sleep_enabled,
        jnt_body: &model.jnt_body,
        body_sleep_state: &data.body_sleep_state,
    };
    model.visit_joints(&mut visitor);
}

// mj_integrate_pos_flex DELETED (§27F): Flex vertices now have slide joints.
// Standard mj_integrate_pos handles slide joint position integration.

/// Visitor for position integration that handles different joint types.
struct PositionIntegrateVisitor<'a> {
    qpos: &'a mut DVector<f64>,
    qvel: &'a DVector<f64>,
    h: f64,
    sleep_enabled: bool,
    jnt_body: &'a [usize],
    body_sleep_state: &'a [SleepState],
}

impl PositionIntegrateVisitor<'_> {
    /// Integrate a quaternion with angular velocity on SO(3) manifold.
    /// `qpos_offset` is the offset into qpos where the quaternion starts [w,x,y,z].
    /// `omega` is the angular velocity vector.
    #[inline]
    fn integrate_quaternion(&mut self, qpos_offset: usize, omega: Vector3<f64>) {
        let omega_norm = omega.norm();
        let angle = omega_norm * self.h;

        // Skip if angle is negligible (avoids division by zero)
        if angle > 1e-10 && omega_norm > 1e-10 {
            let axis = omega / omega_norm;
            let dq = UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(axis), angle);
            let q_old = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                self.qpos[qpos_offset],
                self.qpos[qpos_offset + 1],
                self.qpos[qpos_offset + 2],
                self.qpos[qpos_offset + 3],
            ));
            let q_new = q_old * dq;
            self.qpos[qpos_offset] = q_new.w;
            self.qpos[qpos_offset + 1] = q_new.i;
            self.qpos[qpos_offset + 2] = q_new.j;
            self.qpos[qpos_offset + 3] = q_new.k;
        }
    }
}

impl PositionIntegrateVisitor<'_> {
    /// Check if this joint's body is sleeping (§16.5a'').
    #[inline]
    fn is_sleeping(&self, ctx: &JointContext) -> bool {
        self.sleep_enabled && self.body_sleep_state[self.jnt_body[ctx.jnt_id]] == SleepState::Asleep
    }
}

impl JointVisitor for PositionIntegrateVisitor<'_> {
    #[inline]
    fn visit_hinge(&mut self, ctx: JointContext) {
        if self.is_sleeping(&ctx) {
            return;
        }
        // Simple scalar: qpos += qvel * h
        self.qpos[ctx.qpos_adr] += self.qvel[ctx.dof_adr] * self.h;
    }

    #[inline]
    fn visit_slide(&mut self, ctx: JointContext) {
        if self.is_sleeping(&ctx) {
            return;
        }
        // Simple scalar: qpos += qvel * h
        self.qpos[ctx.qpos_adr] += self.qvel[ctx.dof_adr] * self.h;
    }

    #[inline]
    fn visit_ball(&mut self, ctx: JointContext) {
        if self.is_sleeping(&ctx) {
            return;
        }
        // Quaternion: integrate angular velocity on SO(3)
        let omega = Vector3::new(
            self.qvel[ctx.dof_adr],
            self.qvel[ctx.dof_adr + 1],
            self.qvel[ctx.dof_adr + 2],
        );
        self.integrate_quaternion(ctx.qpos_adr, omega);
    }

    #[inline]
    fn visit_free(&mut self, ctx: JointContext) {
        if self.is_sleeping(&ctx) {
            return;
        }
        // Position: linear integration (first 3 components)
        self.qpos[ctx.qpos_adr] += self.qvel[ctx.dof_adr] * self.h;
        self.qpos[ctx.qpos_adr + 1] += self.qvel[ctx.dof_adr + 1] * self.h;
        self.qpos[ctx.qpos_adr + 2] += self.qvel[ctx.dof_adr + 2] * self.h;

        // Orientation: quaternion integration (last 4 components, DOFs 3-5)
        let omega = Vector3::new(
            self.qvel[ctx.dof_adr + 3],
            self.qvel[ctx.dof_adr + 4],
            self.qvel[ctx.dof_adr + 5],
        );
        self.integrate_quaternion(ctx.qpos_adr + 3, omega);
    }
}

/// Normalize all quaternions in qpos to prevent numerical drift.
pub fn mj_normalize_quat(model: &Model, data: &mut Data) {
    let mut visitor = QuaternionNormalizeVisitor {
        qpos: &mut data.qpos,
    };
    model.visit_joints(&mut visitor);
}

/// Visitor for normalizing quaternions in joints that use them.
struct QuaternionNormalizeVisitor<'a> {
    qpos: &'a mut DVector<f64>,
}

impl QuaternionNormalizeVisitor<'_> {
    /// Normalize a quaternion at the given offset in qpos.
    #[inline]
    fn normalize_quaternion(&mut self, offset: usize) {
        let norm = (self.qpos[offset].powi(2)
            + self.qpos[offset + 1].powi(2)
            + self.qpos[offset + 2].powi(2)
            + self.qpos[offset + 3].powi(2))
        .sqrt();
        if norm > 1e-10 {
            self.qpos[offset] /= norm;
            self.qpos[offset + 1] /= norm;
            self.qpos[offset + 2] /= norm;
            self.qpos[offset + 3] /= norm;
        } else {
            // Degenerate quaternion - reset to identity [w=1, x=0, y=0, z=0]
            self.qpos[offset] = 1.0;
            self.qpos[offset + 1] = 0.0;
            self.qpos[offset + 2] = 0.0;
            self.qpos[offset + 3] = 0.0;
        }
    }
}

impl JointVisitor for QuaternionNormalizeVisitor<'_> {
    // Hinge and Slide have no quaternions - use default no-ops

    #[inline]
    fn visit_ball(&mut self, ctx: JointContext) {
        // Ball joint: quaternion at qpos_adr
        self.normalize_quaternion(ctx.qpos_adr);
    }

    #[inline]
    fn visit_free(&mut self, ctx: JointContext) {
        // Free joint: quaternion at qpos_adr + 3 (after position xyz)
        self.normalize_quaternion(ctx.qpos_adr + 3);
    }
}
