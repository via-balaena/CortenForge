//! Joint processing.
//!
//! Converts MJCF `<joint>` elements into `Model` joint and DOF arrays,
//! including angle conversion, limit handling, and kinematic tree linkage.

use nalgebra::{UnitQuaternion, Vector3};
use sim_core::{MjJointType, Model};

use super::limits::{self, Pair};
use super::orientation::{MJ_EPS, normvec};
use super::{DEFAULT_SOLIMP, DEFAULT_SOLREF, ModelBuilder, written_over};
use crate::error::{ElementKind, Location, MjcfError, Result};
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
    ) -> Result<usize> {
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
        };

        // The axis as MuJoCo's `mjCJoint::Compile` makes it
        // (`user_objects.cc:2946-2960`): a ball's or a free joint's is z,
        // whatever is written; another's is made unit by `normvec`, which
        // returns 0 where its squared norm is below mjEPS. MuJoCo refuses that
        // one ("axis too small in joint"); here it is kept as z and refused by
        // `check_joints`, after the checks this crate makes before it.
        let (axis, axis_too_small) = match jnt_type {
            MjJointType::Ball | MjJointType::Free => (Vector3::z(), false),
            MjJointType::Hinge | MjJointType::Slide => {
                let written = joint.axis.unwrap_or_else(Vector3::z);
                let mut unit = [written.x, written.y, written.z];
                if normvec(&mut unit) < MJ_EPS {
                    (Vector3::z(), true)
                } else {
                    (Vector3::from(unit), false)
                }
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
        self.jnt_axis.push(axis);
        self.jnt_axis_too_small.push(axis_too_small);
        let limit = limits::resolve(
            Pair::Joint(jnt_type),
            joint.limited,
            joint.range,
            self.compiler.autolimits,
        );
        self.jnt_limited.push(limit.limited);
        self.jnt_limit_fault.push(limit.fault);

        // A hinge's and a ball's range are angles: converted from degrees
        // when the compiler reads degrees (the default) and the joint is
        // limited; otherwise stored as written, `0 0` when none, as MuJoCo
        // stores it (`user_objects.cc:2916-2924`).
        let deg2rad = std::f64::consts::PI / 180.0;
        let is_angle_joint = matches!(jnt_type, MjJointType::Hinge | MjJointType::Ball);
        let convert = is_angle_joint && self.compiler.angle == AngleUnit::Degree;
        let (lo, hi) = joint.range.unwrap_or((0.0, 0.0));
        self.jnt_range.push(if convert && limit.limited {
            (lo * deg2rad, hi * deg2rad)
        } else {
            (lo, hi)
        });
        self.jnt_stiffness.push(joint.stiffness.unwrap_or(0.0));

        // ref_pos and spring_ref: angle-valued for hinge joints only.
        // Ball joint ref is a quaternion, not angle-valued.
        let convert_ref =
            matches!(jnt_type, MjJointType::Hinge) && self.compiler.angle == AngleUnit::Degree;
        let spring_ref = joint.spring_ref.unwrap_or(0.0);
        let spring_ref_converted = if convert_ref {
            spring_ref * deg2rad
        } else {
            spring_ref
        };
        self.jnt_springref.push(spring_ref_converted);
        self.jnt_damping.push(joint.damping.unwrap_or(0.0));
        self.jnt_solref
            .push(written_over(joint.solref_limit, DEFAULT_SOLREF));
        self.jnt_solimp
            .push(written_over(joint.solimp_limit, DEFAULT_SOLIMP));
        self.jnt_name.push(if joint.name.is_empty() {
            None
        } else {
            Some(joint.name.clone())
        });
        self.jnt_group.push(joint.group.unwrap_or(0));
        self.jnt_actgravcomp
            .push(joint.actuatorgravcomp.unwrap_or(false));
        self.jnt_margin.push(joint.margin.unwrap_or(0.0));
        self.jnt_user_raw.push(joint.user.clone());

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
                .push(written_over(joint.solreffriction, DEFAULT_SOLREF));
            self.dof_solimp
                .push(written_over(joint.solimpfriction, DEFAULT_SOLIMP));
        }

        // Add qpos0 and qpos_spring values (default positions + spring reference)
        match jnt_type {
            MjJointType::Hinge => {
                let ref_pos = joint.ref_pos.unwrap_or(0.0);
                let ref_val = if self.compiler.angle == AngleUnit::Degree {
                    ref_pos * deg2rad
                } else {
                    ref_pos
                };
                self.qpos0_values.push(ref_val);
                // qpos_spring: uses springref (already converted to radians for hinge)
                self.qpos_spring_values.push(spring_ref_converted);
            }
            MjJointType::Slide => {
                // Slide ref is translational — not angle-valued
                self.qpos0_values.push(joint.ref_pos.unwrap_or(0.0));
                self.qpos_spring_values.push(spring_ref_converted);
            }
            MjJointType::Ball => {
                // Quaternion identity [w, x, y, z] = [1, 0, 0, 0]
                let qpos0_ball = [1.0, 0.0, 0.0, 0.0];
                self.qpos0_values.extend_from_slice(&qpos0_ball);
                // Ball: qpos_spring = qpos0 (MuJoCo ignores springref for ball joints)
                self.qpos_spring_values.extend_from_slice(&qpos0_ball);
            }
            MjJointType::Free => {
                // Free joint qpos0 is the body's world position and orientation
                // Position [x, y, z] from world frame, quaternion [w, x, y, z] from world frame
                let q = world_quat.into_inner();
                let qpos0_free = [world_pos.x, world_pos.y, world_pos.z, q.w, q.i, q.j, q.k];
                self.qpos0_values.extend_from_slice(&qpos0_free);
                // Free: qpos_spring = qpos0 (MuJoCo ignores springref for free joints)
                self.qpos_spring_values.extend_from_slice(&qpos0_free);
            }
        }

        // Update dimensions
        self.nq += nq;
        self.nv += nv;

        Ok(jnt_id)
    }

    /// The joints of a body, as a range of joint indices.
    fn body_joints(&self, body: usize) -> std::ops::Range<usize> {
        let start = self.body_jnt_adr[body];
        start..start + self.body_jnt_num[body]
    }

    /// Where body `body` is, for an error.
    fn body_location(&self, body: usize) -> Location {
        Location::nth(ElementKind::Body, self.body_name[body].as_deref(), body)
    }

    /// MuJoCo 3.5.0's checks of each body's joints, body by body in its
    /// order, as `mjCJoint::Compile` and `mjCBody::Compile` make them: joint
    /// by joint, its `limited` and `range` ([`limits::resolve`],
    /// `user_objects.cc:2895-2914`), then a hinge's or a slide's axis too
    /// small to make unit (`:2957-2960`); then the body's more than 6 degrees
    /// of freedom, then a ball or a hinge after a ball (`:2522-2537`). The
    /// caller runs it after every check this crate makes before building the
    /// model, so that a document it refused before these checks existed is
    /// refused for the same fault, unless that fault is a keyframe's: MuJoCo
    /// compiles keyframes after these checks, and this crate resolves them
    /// after building the model.
    pub(crate) fn check_joints(&self) -> Result<()> {
        for body in 1..self.body_parent.len() {
            let joints = self.body_joints(body);
            for j in joints.clone() {
                let at =
                    || {
                        Location::nth(ElementKind::Joint, self.jnt_name[j].as_deref(), j)
                            .within_nth(ElementKind::Body, self.body_name[body].as_deref(), body)
                    };
                if let Some(fault) = self.jnt_limit_fault[j].filter(|&f| f.is_mujocos()) {
                    return Err(fault.error(Pair::Joint(self.jnt_type[j])).at(at()));
                }
                if self.jnt_axis_too_small[j] {
                    return Err(
                        MjcfError::invalid_value("joint", "axis", "axis too small").at(at())
                    );
                }
            }
            let kinds = &self.jnt_type[joints];
            if kinds.iter().map(|j| j.nv()).sum::<usize>() > 6 {
                return Err(
                    MjcfError::invalid_model("more than 6 dofs").at(self.body_location(body))
                );
            }
            if after_ball(kinds, &[MjJointType::Ball, MjJointType::Hinge]) {
                return Err(MjcfError::invalid_model("ball followed by rotation")
                    .at(self.body_location(body)));
            }
        }
        Ok(())
    }

    /// MuJoCo 3.5.0's rule for a free joint, which it applies as it copies its
    /// tree into its model (`CopyTree`, `user_model.cc:2725-2727`), after it
    /// has compiled the bodies, the tendons and the actuators: a free joint's
    /// body is a child of the world, frames being dissolved and static bodies
    /// fused by then. Bodies are checked in order, as MuJoCo checks them.
    pub(crate) fn check_free_joints_on_top_level(&self) -> Result<()> {
        let nested = (1..self.body_parent.len()).find(|&b| {
            self.body_parent[b] != 0
                && self.jnt_type[self.body_joints(b)].contains(&MjJointType::Free)
        });
        match nested {
            Some(b) => Err(
                MjcfError::invalid_model("free joint can only be used on top level")
                    .at(self.body_location(b)),
            ),
            None => Ok(()),
        }
    }
}

/// A slide after a ball on one body of the built `model`, which MuJoCo 3.5.0
/// loads, refused as not supported (`D-BALL-THEN-SLIDE`, a decision of the
/// parity rule's, 01-decisions). The caller runs it once the keyframes are
/// resolved.
pub fn check_ball_then_slide(model: &Model) -> Result<()> {
    for body in 1..model.nbody {
        let start = model.body_jnt_adr[body];
        let joints = &model.jnt_type[start..start + model.body_jnt_num[body]];
        if after_ball(joints, &[MjJointType::Slide]) {
            return Err(
                MjcfError::unsupported("a slide joint after a ball joint on one body").at(
                    Location::nth(ElementKind::Body, model.body_name[body].as_deref(), body),
                ),
            );
        }
    }
    Ok(())
}

/// Whether a joint of one of `kinds` follows a ball among a body's `joints`.
fn after_ball(joints: &[MjJointType], kinds: &[MjJointType]) -> bool {
    joints
        .iter()
        .skip_while(|&&j| j != MjJointType::Ball)
        .skip(1)
        .any(|j| kinds.contains(j))
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use crate::builder::load_model;
    use sim_core::MjJointType;

    #[test]
    fn test_free_joint() {
        let model = load_model(
            r#"
            <mujoco model="floating">
                <worldbody>
                    <body name="ball" pos="0 0 1">
                        <joint type="free"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nq, 7); // 3 pos + 4 quat
        assert_eq!(model.nv, 6); // 3 linear + 3 angular
        assert_eq!(model.jnt_type[0], MjJointType::Free);
    }

    #[test]
    fn test_ball_joint() {
        let model = load_model(
            r#"
            <mujoco model="ball_joint">
                <worldbody>
                    <body name="link" pos="0 0 1">
                        <joint type="ball"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nq, 4); // quaternion
        assert_eq!(model.nv, 3); // 3 angular DOFs
        assert_eq!(model.jnt_type[0], MjJointType::Ball);
    }

    /// Test that dof_parent forms correct kinematic tree for simple chain.
    ///
    /// For a 2-link pendulum (world -> link1 -> link2), each with 1 hinge:
    /// - DOF 0 (link1's hinge): parent = None (attached to world)
    /// - DOF 1 (link2's hinge): parent = Some(0) (attached to link1's DOF)
    #[test]
    fn test_dof_parent_chain() {
        let model = load_model(
            r#"
            <mujoco model="chain">
                <worldbody>
                    <body name="link1" pos="0 0 1">
                        <joint name="j1" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.01"/>
                        <body name="link2" pos="0 0 -0.5">
                            <joint name="j2" type="hinge" axis="0 1 0"/>
                            <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.01"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nv, 2);
        // DOF 0: link1's hinge, parent is world (None)
        assert_eq!(model.dof_parent[0], None);
        // DOF 1: link2's hinge, parent is DOF 0 (link1's hinge)
        assert_eq!(model.dof_parent[1], Some(0));
    }

    /// Test dof_parent for multi-DOF joints (ball joint).
    ///
    /// For a ball joint (3 DOFs):
    /// - DOF 0: parent = parent body's last DOF (or None)
    /// - DOF 1: parent = DOF 0
    /// - DOF 2: parent = DOF 1
    #[test]
    fn test_dof_parent_ball_joint() {
        let model = load_model(
            r#"
            <mujoco model="ball_chain">
                <worldbody>
                    <body name="link1" pos="0 0 1">
                        <joint name="j1" type="ball"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nv, 3);
        // DOF 0: first DOF of ball, parent is world (None)
        assert_eq!(model.dof_parent[0], None);
        // DOF 1: second DOF of ball, parent is DOF 0
        assert_eq!(model.dof_parent[1], Some(0));
        // DOF 2: third DOF of ball, parent is DOF 1
        assert_eq!(model.dof_parent[2], Some(1));
    }

    /// Test dof_parent for free joint (6 DOFs).
    #[test]
    fn test_dof_parent_free_joint() {
        let model = load_model(
            r#"
            <mujoco model="free_body">
                <worldbody>
                    <body name="floating" pos="0 0 1">
                        <joint type="free"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nv, 6);
        // DOF 0: first DOF of free, parent is world (None)
        assert_eq!(model.dof_parent[0], None);
        // DOFs 1-5: chain within free joint
        for i in 1..6 {
            assert_eq!(model.dof_parent[i], Some(i - 1));
        }
    }

    /// Test dof_parent for chain with mixed joint types.
    ///
    /// world -> link1 (ball, 3 DOF) -> link2 (hinge, 1 DOF)
    /// - DOFs 0,1,2: ball joint, parents: None, 0, 1
    /// - DOF 3: hinge, parent is DOF 2 (last DOF of ball)
    #[test]
    fn test_dof_parent_mixed_joints() {
        let model = load_model(
            r#"
            <mujoco model="mixed">
                <worldbody>
                    <body name="link1" pos="0 0 1">
                        <joint name="ball" type="ball"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <body name="link2" pos="0 0 -0.5">
                            <joint name="hinge" type="hinge" axis="0 1 0"/>
                            <inertial pos="0 0 0" mass="0.5" diaginertia="0.05 0.05 0.01"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nv, 4); // 3 (ball) + 1 (hinge)

        // Ball joint DOFs
        assert_eq!(model.dof_parent[0], None);
        assert_eq!(model.dof_parent[1], Some(0));
        assert_eq!(model.dof_parent[2], Some(1));

        // Hinge DOF: parent is ball's last DOF
        assert_eq!(model.dof_parent[3], Some(2));
    }

    /// Test dof_parent for branching tree (two children of same parent).
    ///
    /// world -> base (hinge) -+-> left_arm (hinge)
    ///                        +-> right_arm (hinge)
    ///
    /// DOF 0: base
    /// DOF 1: left_arm, parent = 0
    /// DOF 2: right_arm, parent = 0
    #[test]
    fn test_dof_parent_branching() {
        let model = load_model(
            r#"
            <mujoco model="branching">
                <worldbody>
                    <body name="base" pos="0 0 1">
                        <joint name="j_base" type="hinge" axis="0 0 1"/>
                        <inertial pos="0 0 0" mass="2.0" diaginertia="0.2 0.2 0.1"/>
                        <body name="left_arm" pos="0.5 0 0">
                            <joint name="j_left" type="hinge" axis="0 1 0"/>
                            <inertial pos="0 0 0" mass="0.5" diaginertia="0.05 0.05 0.01"/>
                        </body>
                        <body name="right_arm" pos="-0.5 0 0">
                            <joint name="j_right" type="hinge" axis="0 1 0"/>
                            <inertial pos="0 0 0" mass="0.5" diaginertia="0.05 0.05 0.01"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nv, 3);

        // Base joint: attached to world
        assert_eq!(model.dof_parent[0], None);
        // Both arms attach to base
        assert_eq!(model.dof_parent[1], Some(0)); // left_arm
        assert_eq!(model.dof_parent[2], Some(0)); // right_arm
    }

    /// Test dof_parent for body with multiple joints (uncommon but valid).
    ///
    /// In MuJoCo, a single body can have multiple joints. The DOFs chain:
    /// world -> link (hinge1, hinge2)
    /// - DOF 0: hinge1, parent = None
    /// - DOF 1: hinge2, parent = DOF 0
    #[test]
    fn test_dof_parent_multiple_joints_one_body() {
        let model = load_model(
            r#"
            <mujoco model="multi_joint">
                <worldbody>
                    <body name="link" pos="0 0 1">
                        <joint name="j1" type="hinge" axis="1 0 0"/>
                        <joint name="j2" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nv, 2);
        assert_eq!(model.njnt, 2);

        // First joint's DOF: attached to world
        assert_eq!(model.dof_parent[0], None);
        // Second joint's DOF: attached to first joint
        assert_eq!(model.dof_parent[1], Some(0));
    }

    #[test]
    fn test_angle_conversion_hinge_joint_degrees() {
        // Default compiler: angle=degree. Hinge range in degrees -> converted to radians.
        let model = load_model(
            r#"
            <mujoco model="deg_test">
                <worldbody>
                    <body name="b">
                        <joint name="j" type="hinge" limited="true" range="-90 90"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let pi = std::f64::consts::PI;
        assert!(
            (model.jnt_range[0].0 - (-pi / 2.0)).abs() < 1e-10,
            "range lo: expected {}, got {}",
            -pi / 2.0,
            model.jnt_range[0].0
        );
        assert!(
            (model.jnt_range[0].1 - (pi / 2.0)).abs() < 1e-10,
            "range hi: expected {}, got {}",
            pi / 2.0,
            model.jnt_range[0].1
        );
    }

    #[test]
    fn test_angle_conversion_ball_joint_range_degrees() {
        // Ball joint range is angle-valued (swing/twist limits).
        // With default compiler.angle=degree, range values must be converted to radians.
        let model = load_model(
            r#"
            <mujoco model="ball_deg_test">
                <worldbody>
                    <body name="b">
                        <joint name="j" type="ball" limited="true" range="0 60"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let pi = std::f64::consts::PI;
        let expected_lo = 0.0;
        let expected_hi = 60.0 * pi / 180.0; // pi/3
        assert!(
            (model.jnt_range[0].0 - expected_lo).abs() < 1e-10,
            "ball range lo: expected {expected_lo}, got {}",
            model.jnt_range[0].0
        );
        assert!(
            (model.jnt_range[0].1 - expected_hi).abs() < 1e-10,
            "ball range hi: expected {expected_hi}, got {}",
            model.jnt_range[0].1
        );
    }

    #[test]
    fn test_angle_conversion_ball_joint_range_radian_passthrough() {
        // With angle="radian", ball joint range values pass through unchanged.
        let pi_3 = std::f64::consts::PI / 3.0;
        let model = load_model(&format!(
            r#"
            <mujoco model="ball_rad_test">
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <joint name="j" type="ball" limited="true" range="0 {pi_3}"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        ))
        .expect("should load");

        assert!(
            (model.jnt_range[0].0 - 0.0).abs() < 1e-10,
            "ball radian range lo: expected 0.0, got {}",
            model.jnt_range[0].0
        );
        assert!(
            (model.jnt_range[0].1 - pi_3).abs() < 1e-10,
            "ball radian range hi: expected {pi_3}, got {}",
            model.jnt_range[0].1
        );
    }

    #[test]
    fn test_angle_conversion_radian_passthrough() {
        // With angle="radian", values pass through unchanged.
        let model = load_model(
            r#"
            <mujoco model="rad_test">
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <joint name="j" type="hinge" limited="true" range="-1.57 1.57"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        assert!(
            (model.jnt_range[0].0 - (-1.57)).abs() < 1e-10,
            "range lo: expected -1.57, got {}",
            model.jnt_range[0].0
        );
        assert!(
            (model.jnt_range[0].1 - 1.57).abs() < 1e-10,
            "range hi: expected 1.57, got {}",
            model.jnt_range[0].1
        );
    }

    #[test]
    fn test_angle_conversion_slide_joint_not_converted() {
        // Slide joint range is translational — never converted even with angle=degree.
        let model = load_model(
            r#"
            <mujoco model="slide_test">
                <worldbody>
                    <body name="b">
                        <joint name="j" type="slide" limited="true" range="-0.5 0.5"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        assert!(
            (model.jnt_range[0].0 - (-0.5)).abs() < 1e-10,
            "slide range should not be converted"
        );
        assert!(
            (model.jnt_range[0].1 - 0.5).abs() < 1e-10,
            "slide range should not be converted"
        );
    }

    #[test]
    fn test_angle_conversion_springref_degrees() {
        // Hinge springref in degrees -> converted to radians.
        let model = load_model(
            r#"
            <mujoco model="springref_deg">
                <worldbody>
                    <body name="b">
                        <joint name="j" type="hinge" stiffness="100" springref="45"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let expected = 45.0 * std::f64::consts::PI / 180.0;
        assert!(
            (model.jnt_springref[0] - expected).abs() < 1e-10,
            "springref: expected {expected}, got {}",
            model.jnt_springref[0]
        );
    }

    #[test]
    fn test_autolimits_infers_limited_from_range() {
        // Default autolimits=true: range present + no explicit limited -> limited=true.
        let model = load_model(
            r#"
            <mujoco model="autolimits_test">
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <joint name="j" type="hinge" range="-1.0 1.0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        assert!(
            model.jnt_limited[0],
            "autolimits should infer limited=true from range"
        );
    }

    #[test]
    fn test_autolimits_false_requires_explicit_limited() {
        // With autolimits=false, a range without `limited` is refused, as
        // MuJoCo 3.5.0's `checklimited` refuses it (`user_objects.cc:171-181`).
        let error = load_model(
            r#"
            <mujoco model="no_autolimits">
                <compiler angle="radian" autolimits="false"/>
                <worldbody>
                    <body name="b">
                        <joint name="j" type="hinge" range="-1.0 1.0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect_err("a range without limited is refused")
        .to_string();
        assert!(
            error.contains("joint has `range` but not `limited`"),
            "{error}"
        );
    }

    #[test]
    fn test_autolimits_explicit_limited_takes_precedence() {
        // Explicit limited=false overrides autolimits inference.
        let model = load_model(
            r#"
            <mujoco model="explicit_limited">
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <joint name="j" type="hinge" limited="false" range="-1.0 1.0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        assert!(
            !model.jnt_limited[0],
            "explicit limited=false should override autolimits"
        );
    }

    #[test]
    fn test_autolimits_no_range_no_limited() {
        // No range and no limited -> limited=false (regardless of autolimits).
        let model = load_model(
            r#"
            <mujoco model="no_range">
                <worldbody>
                    <body name="b">
                        <joint name="j" type="hinge"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        assert!(!model.jnt_limited[0], "no range should mean not limited");
    }
}
