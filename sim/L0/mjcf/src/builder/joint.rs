//! Joint processing.
//!
//! Converts MJCF `<joint>` elements into `Model` joint and DOF arrays,
//! including angle conversion, limit handling, and kinematic tree linkage.

use nalgebra::{UnitQuaternion, Vector3};
use sim_core::MjJointType;
use tracing::warn;

use super::{DEFAULT_SOLIMP, DEFAULT_SOLREF, ModelBuilder, ModelConversionError};
use crate::types::{AngleUnit, MjcfJoint, MjcfJointType};

impl ModelBuilder {
    /// Process a joint and add its DOFs to the kinematic tree.
    ///
    /// # Arguments
    /// * `joint` - The MJCF joint to process
    /// * `body_id` - Index of the body this joint belongs to
    /// * `parent_last_dof` - Index of the last DOF in the parent kinematic chain
    ///   (from parent body or previous joint on same body)
    /// * `world_pos` - Body's world-frame position (for free joint qpos0)
    /// * `world_quat` - Body's world-frame orientation (for free joint qpos0)
    ///
    /// # MuJoCo Semantics
    ///
    /// The `dof_parent` array forms a kinematic tree:
    /// - First DOF of this joint links to `parent_last_dof`
    /// - Subsequent DOFs within this joint link to the previous DOF
    ///
    /// This enables correct propagation in CRBA/RNE algorithms.
    pub(crate) fn process_joint(
        &mut self,
        joint: &MjcfJoint,
        body_id: usize,
        parent_last_dof: Option<usize>,
        world_pos: Vector3<f64>,
        world_quat: UnitQuaternion<f64>,
    ) -> std::result::Result<usize, ModelConversionError> {
        let jnt_id = self.jnt_type.len();

        // Store name mapping
        if !joint.name.is_empty() {
            self.joint_name_to_id.insert(joint.name.clone(), jnt_id);
        }

        // Convert joint type
        let joint_type = joint.joint_type.unwrap_or(MjcfJointType::Hinge);
        let jnt_type = match joint_type {
            MjcfJointType::Hinge => MjJointType::Hinge,
            MjcfJointType::Slide => MjJointType::Slide,
            MjcfJointType::Ball => MjJointType::Ball,
            MjcfJointType::Free => MjJointType::Free,
            MjcfJointType::Cylindrical | MjcfJointType::Planar => {
                return Err(ModelConversionError {
                    message: format!("Joint type {joint_type:?} not yet supported in Model",),
                });
            }
        };

        let nq = jnt_type.nq();
        let nv = jnt_type.nv();
        let qpos_adr = self.nq;
        let dof_adr = self.nv;

        // Add joint arrays
        self.jnt_type.push(jnt_type);
        self.jnt_body.push(body_id);
        self.jnt_qpos_adr.push(qpos_adr);
        self.jnt_dof_adr.push(dof_adr);
        self.jnt_pos.push(joint.pos.unwrap_or_else(Vector3::zeros));
        // Normalize joint axis, handling zero vector edge case
        // MuJoCo defaults to Z-axis for unspecified/zero axes
        let raw_axis = joint.axis.unwrap_or_else(Vector3::z);
        let axis = if raw_axis.norm() > 1e-10 {
            raw_axis.normalize()
        } else {
            warn!(
                joint_name = ?joint.name,
                "Joint axis is zero or near-zero, defaulting to Z-axis"
            );
            Vector3::z()
        };
        self.jnt_axis.push(axis);
        // Autolimits: infer limited from range presence when no explicit value.
        let limited = match joint.limited {
            Some(v) => v,
            None => {
                if self.compiler.autolimits {
                    joint.range.is_some()
                } else {
                    false
                }
            }
        };
        self.jnt_limited.push(limited);

        // Angle conversion: hinge and ball joint range/ref/springref are angle-valued.
        // When compiler.angle == Degree (default), convert to radians.
        // Slide and Free joints are not angle-valued — never convert.
        let deg2rad = std::f64::consts::PI / 180.0;
        let is_angle_joint = matches!(jnt_type, MjJointType::Hinge | MjJointType::Ball);
        let convert = is_angle_joint && self.compiler.angle == AngleUnit::Degree;

        let range = joint
            .range
            .unwrap_or((-std::f64::consts::PI, std::f64::consts::PI));
        self.jnt_range.push(if convert {
            (range.0 * deg2rad, range.1 * deg2rad)
        } else {
            range
        });
        self.jnt_stiffness.push(joint.stiffness.unwrap_or(0.0));

        // ref_pos and spring_ref: angle-valued for hinge joints only.
        // Ball joint ref is a quaternion, not angle-valued.
        let convert_ref =
            matches!(jnt_type, MjJointType::Hinge) && self.compiler.angle == AngleUnit::Degree;
        let spring_ref = joint.spring_ref.unwrap_or(0.0);
        self.jnt_springref.push(if convert_ref {
            spring_ref * deg2rad
        } else {
            spring_ref
        });
        self.jnt_damping.push(joint.damping.unwrap_or(0.0));
        self.jnt_armature.push(joint.armature.unwrap_or(0.0));
        self.jnt_solref
            .push(joint.solref_limit.unwrap_or(DEFAULT_SOLREF));
        self.jnt_solimp
            .push(joint.solimp_limit.unwrap_or(DEFAULT_SOLIMP));
        self.jnt_name.push(if joint.name.is_empty() {
            None
        } else {
            Some(joint.name.clone())
        });
        self.jnt_group.push(joint.group.unwrap_or(0));

        // Add DOF arrays with correct kinematic tree linkage
        // MuJoCo semantics: dof_parent forms a tree structure for CRBA/RNE
        for i in 0..nv {
            self.dof_body.push(body_id);
            self.dof_jnt.push(jnt_id);

            // First DOF links to parent chain, subsequent DOFs link within joint
            let parent = if i == 0 {
                parent_last_dof
            } else {
                Some(dof_adr + i - 1)
            };
            self.dof_parent.push(parent);

            self.dof_armature.push(joint.armature.unwrap_or(0.0));
            self.dof_damping.push(joint.damping.unwrap_or(0.0));
            self.dof_frictionloss
                .push(joint.frictionloss.unwrap_or(0.0));
            self.dof_solref
                .push(joint.solreffriction.unwrap_or(DEFAULT_SOLREF));
            self.dof_solimp
                .push(joint.solimpfriction.unwrap_or(DEFAULT_SOLIMP));
        }

        // Add qpos0 values (default positions)
        match jnt_type {
            MjJointType::Hinge => {
                let ref_pos = joint.ref_pos.unwrap_or(0.0);
                let ref_val = if self.compiler.angle == AngleUnit::Degree {
                    ref_pos * deg2rad
                } else {
                    ref_pos
                };
                self.qpos0_values.push(ref_val);
            }
            MjJointType::Slide => {
                // Slide ref is translational — not angle-valued
                self.qpos0_values.push(joint.ref_pos.unwrap_or(0.0));
            }
            MjJointType::Ball => {
                // Quaternion identity [w, x, y, z] = [1, 0, 0, 0]
                self.qpos0_values.extend_from_slice(&[1.0, 0.0, 0.0, 0.0]);
            }
            MjJointType::Free => {
                // Free joint qpos0 is the body's world position and orientation
                // Position [x, y, z] from world frame, quaternion [w, x, y, z] from world frame
                let q = world_quat.into_inner();
                self.qpos0_values.extend_from_slice(&[
                    world_pos.x,
                    world_pos.y,
                    world_pos.z,
                    q.w,
                    q.i,
                    q.j,
                    q.k,
                ]);
            }
        }

        // Update dimensions
        self.nq += nq;
        self.nv += nv;

        Ok(jnt_id)
    }
}
