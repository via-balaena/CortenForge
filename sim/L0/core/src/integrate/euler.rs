//! Euler position integration on SO(3) manifold + quaternion normalization.
//!
//! Implements MuJoCo's position integration step: each joint type gets its
//! appropriate integration (scalar for hinge/slide, quaternion exponential map
//! for ball/free). After integration, quaternions are renormalized to prevent
//! numerical drift.

use nalgebra::{DVector, Vector3};

use crate::joint_visitor::{JointContext, JointVisitor};
use crate::quat::quat_integrate;
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
    /// Turns the quaternion at `qpos_offset` (`[w, x, y, z]`) by the angular
    /// velocity `omega` over the step, as MuJoCo's `mju_quatIntegrate`
    /// ([`quat_integrate`]).
    #[inline]
    fn integrate_quaternion(&mut self, qpos_offset: usize, omega: Vector3<f64>) {
        let q = &mut self.qpos.as_mut_slice()[qpos_offset..qpos_offset + 4];
        let mut quat = [q[0], q[1], q[2], q[3]];
        quat_integrate(&mut quat, [omega.x, omega.y, omega.z], self.h);
        q.copy_from_slice(&quat);
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
