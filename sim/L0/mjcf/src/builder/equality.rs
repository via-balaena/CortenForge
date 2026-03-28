//! Equality constraint processing.
//!
//! Converts MJCF `<equality>` elements (connect, weld, joint, distance, tendon)
//! into Model arrays. Handles body/joint/geom/tendon name resolution, polynomial
//! coefficient packing, and default parameter application.
//!
//! Constraints are processed in **XML document order** (not grouped by type),
//! matching MuJoCo's semantics. The [`MjcfEquality::order`] vec records the
//! insertion sequence from the parser; we iterate it here.

use nalgebra::Vector3;

use super::{DEFAULT_SOLIMP, DEFAULT_SOLREF, ModelBuilder, ModelConversionError};
use crate::types::{
    EqualityKind, MjcfConnect, MjcfDistance, MjcfEquality, MjcfEqualityDefaults, MjcfJointEquality,
    MjcfTendonEquality, MjcfWeld,
};
use sim_core::EqualityType;

/// Apply equality defaults to active/solref/solimp fields.
/// Element-level values take precedence; defaults fill in gaps.
fn apply_eq_defaults(
    defaults: Option<&MjcfEqualityDefaults>,
    active: &mut Option<bool>,
    solref: &mut Option<[f64; 2]>,
    solimp: &mut Option<[f64; 5]>,
) {
    if let Some(d) = defaults {
        if active.is_none() {
            *active = d.active;
        }
        if solref.is_none() {
            *solref = d.solref;
        }
        if solimp.is_none() {
            *solimp = d.solimp;
        }
    }
}

impl ModelBuilder {
    /// Process equality constraints from MJCF **in document order**.
    ///
    /// MuJoCo assigns equality constraint IDs in the order they appear in the
    /// XML, regardless of type (connect, weld, joint, distance, tendon). The
    /// parser records this order in [`MjcfEquality::order`]; we iterate it here
    /// to produce the same eq_id assignment as MuJoCo.
    ///
    /// Must be called AFTER all bodies and joints are processed (requires name lookups).
    pub(crate) fn process_equality_constraints(
        &mut self,
        equality: &MjcfEquality,
    ) -> std::result::Result<(), ModelConversionError> {
        for &(kind, idx) in &equality.order {
            match kind {
                EqualityKind::Connect => self.process_connect(&equality.connects[idx])?,
                EqualityKind::Weld => self.process_weld(&equality.welds[idx])?,
                EqualityKind::Joint => self.process_joint_equality(&equality.joints[idx])?,
                EqualityKind::Distance => self.process_distance(&equality.distances[idx])?,
                EqualityKind::Tendon => self.process_tendon_equality(&equality.tendons[idx])?,
            }
        }
        Ok(())
    }

    /// Process a single Connect constraint (3 DOF position constraint).
    ///
    /// eq_data layout (matching MuJoCo's dual-anchor convention):
    ///   `[0..3]` = anchor in body1's local frame (user-specified)
    ///   `[3..6]` = anchor in body2's local frame (auto-computed)
    ///
    /// The body2 anchor is auto-computed so both anchors map to the same world
    /// point at qpos0, matching MuJoCo's `engine_setconst.c` behavior.
    fn process_connect(
        &mut self,
        connect: &MjcfConnect,
    ) -> std::result::Result<(), ModelConversionError> {
        let body1_id =
            self.body_name_to_id
                .get(&connect.body1)
                .ok_or_else(|| ModelConversionError {
                    message: format!(
                        "Connect constraint references unknown body1: '{}'",
                        connect.body1
                    ),
                })?;

        let body2_id = if let Some(ref body2_name) = connect.body2 {
            *self
                .body_name_to_id
                .get(body2_name)
                .ok_or_else(|| ModelConversionError {
                    message: format!("Connect constraint references unknown body2: '{body2_name}'"),
                })?
        } else {
            0 // World body
        };

        let eq_defaults = self
            .resolver
            .equality_defaults(connect.class.as_deref())
            .cloned();
        let mut active = connect.active;
        let mut solref = connect.solref;
        let mut solimp = connect.solimp;
        apply_eq_defaults(eq_defaults.as_ref(), &mut active, &mut solref, &mut solimp);

        let mut data = [0.0; 11];
        data[0] = connect.anchor.x;
        data[1] = connect.anchor.y;
        data[2] = connect.anchor.z;

        // Auto-compute body2 anchor: convert body1 anchor to world, then to body2 frame.
        let (p1, q1) = self.body_world_pose(*body1_id);
        let anchor_world = p1 + q1 * connect.anchor;
        let (p2, q2) = self.body_world_pose(body2_id);
        let anchor_body2 = q2.inverse() * (anchor_world - p2);
        data[3] = anchor_body2.x;
        data[4] = anchor_body2.y;
        data[5] = anchor_body2.z;

        self.eq_type.push(EqualityType::Connect);
        self.eq_obj1id.push(*body1_id);
        self.eq_obj2id.push(body2_id);
        self.eq_data.push(data);
        self.eq_active.push(active.unwrap_or(true));
        self.eq_solimp.push(solimp.unwrap_or(DEFAULT_SOLIMP));
        self.eq_solref.push(solref.unwrap_or(DEFAULT_SOLREF));
        let eq_id = self.eq_name.len();
        if let Some(ref name) = connect.name {
            self.eq_name_to_id.insert(name.clone(), eq_id);
        }
        self.eq_name.push(connect.name.clone());
        Ok(())
    }

    /// Process a single Weld constraint (6 DOF pose constraint).
    ///
    /// eq_data layout (matching MuJoCo's engine_setconst.c):
    ///   `[0..3]` = anchor in body2's local frame (user-specified `anchor` attr)
    ///   `[3..6]` = anchor in body1's local frame (auto-computed)
    ///   `[6..10]` = relpose quaternion [w,x,y,z] = neg(q1_ref) * q2_ref
    ///
    /// MuJoCo convention: the `anchor` XML attribute is in body2's frame.
    /// Body1's anchor is auto-computed so both anchor points map to the same
    /// world-space position at qpos0.
    fn process_weld(&mut self, weld: &MjcfWeld) -> std::result::Result<(), ModelConversionError> {
        let body1_id =
            self.body_name_to_id
                .get(&weld.body1)
                .ok_or_else(|| ModelConversionError {
                    message: format!("Weld constraint references unknown body1: '{}'", weld.body1),
                })?;

        let body2_id = if let Some(ref body2_name) = weld.body2 {
            *self
                .body_name_to_id
                .get(body2_name)
                .ok_or_else(|| ModelConversionError {
                    message: format!("Weld constraint references unknown body2: '{body2_name}'"),
                })?
        } else {
            0 // World body
        };

        let mut data = [0.0; 11];

        let (pos1, quat1) = self.body_world_pose(*body1_id);
        let (pos2, quat2) = self.body_world_pose(body2_id);

        // Anchor is in body2's frame (MuJoCo convention). Convert to body1's frame.
        let anchor_body2 = weld.anchor;
        let world_point = pos2 + quat2 * anchor_body2;
        let anchor_body1 = quat1.inverse() * (world_point - pos1);

        data[0] = anchor_body2.x;
        data[1] = anchor_body2.y;
        data[2] = anchor_body2.z;
        data[3] = anchor_body1.x;
        data[4] = anchor_body1.y;
        data[5] = anchor_body1.z;

        if let Some(relpose) = weld.relpose {
            // User-specified relpose quaternion
            data[6] = relpose[3]; // qw
            data[7] = relpose[4]; // qx
            data[8] = relpose[5]; // qy
            data[9] = relpose[6]; // qz
        } else {
            // Auto-compute relpose: neg(q1) * q2
            let rel_quat = quat1.inverse() * quat2;
            data[6] = rel_quat.w;
            data[7] = rel_quat.i;
            data[8] = rel_quat.j;
            data[9] = rel_quat.k;
        }

        let eq_defaults = self
            .resolver
            .equality_defaults(weld.class.as_deref())
            .cloned();
        let mut active = weld.active;
        let mut solref = weld.solref;
        let mut solimp = weld.solimp;
        apply_eq_defaults(eq_defaults.as_ref(), &mut active, &mut solref, &mut solimp);

        self.eq_type.push(EqualityType::Weld);
        self.eq_obj1id.push(*body1_id);
        self.eq_obj2id.push(body2_id);
        self.eq_data.push(data);
        self.eq_active.push(active.unwrap_or(true));
        self.eq_solimp.push(solimp.unwrap_or(DEFAULT_SOLIMP));
        self.eq_solref.push(solref.unwrap_or(DEFAULT_SOLREF));
        let eq_id = self.eq_name.len();
        if let Some(ref name) = weld.name {
            self.eq_name_to_id.insert(name.clone(), eq_id);
        }
        self.eq_name.push(weld.name.clone());
        Ok(())
    }

    /// Process a single Joint equality constraint (polynomial coupling).
    ///
    /// `eq_data[0..5]` = polynomial coefficients (q1 = c0 + c1*q2 + c2*q2² + ...).
    fn process_joint_equality(
        &mut self,
        joint_eq: &MjcfJointEquality,
    ) -> std::result::Result<(), ModelConversionError> {
        let joint1_id =
            self.joint_name_to_id
                .get(&joint_eq.joint1)
                .ok_or_else(|| ModelConversionError {
                    message: format!(
                        "Joint equality references unknown joint1: '{}'",
                        joint_eq.joint1
                    ),
                })?;

        let joint2_id = if let Some(ref joint2_name) = joint_eq.joint2 {
            *self
                .joint_name_to_id
                .get(joint2_name)
                .ok_or_else(|| ModelConversionError {
                    message: format!("Joint equality references unknown joint2: '{joint2_name}'"),
                })?
        } else {
            usize::MAX
        };

        let mut data = [0.0; 11];
        for (i, &coef) in joint_eq.polycoef.iter().take(5).enumerate() {
            data[i] = coef;
        }

        let eq_defaults = self
            .resolver
            .equality_defaults(joint_eq.class.as_deref())
            .cloned();
        let mut active = joint_eq.active;
        let mut solref = joint_eq.solref;
        let mut solimp = joint_eq.solimp;
        apply_eq_defaults(eq_defaults.as_ref(), &mut active, &mut solref, &mut solimp);

        self.eq_type.push(EqualityType::Joint);
        self.eq_obj1id.push(*joint1_id);
        self.eq_obj2id.push(joint2_id);
        self.eq_data.push(data);
        self.eq_active.push(active.unwrap_or(true));
        self.eq_solimp.push(solimp.unwrap_or(DEFAULT_SOLIMP));
        self.eq_solref.push(solref.unwrap_or(DEFAULT_SOLREF));
        let eq_id = self.eq_name.len();
        if let Some(ref name) = joint_eq.name {
            self.eq_name_to_id.insert(name.clone(), eq_id);
        }
        self.eq_name.push(joint_eq.name.clone());
        Ok(())
    }

    /// Process a single Distance constraint (1 DOF scalar distance between geom centers).
    ///
    /// `eq_data[0]` = target distance.
    fn process_distance(
        &mut self,
        dist: &MjcfDistance,
    ) -> std::result::Result<(), ModelConversionError> {
        let geom1_id =
            *self
                .geom_name_to_id
                .get(&dist.geom1)
                .ok_or_else(|| ModelConversionError {
                    message: format!(
                        "Distance constraint references unknown geom1: '{}'",
                        dist.geom1
                    ),
                })?;

        let geom2_id = if let Some(ref geom2_name) = dist.geom2 {
            *self
                .geom_name_to_id
                .get(geom2_name)
                .ok_or_else(|| ModelConversionError {
                    message: format!(
                        "Distance constraint references unknown geom2: '{geom2_name}'"
                    ),
                })?
        } else {
            usize::MAX
        };

        let target_distance = if let Some(d) = dist.distance {
            d.max(0.0)
        } else {
            self.compute_initial_geom_distance(geom1_id, geom2_id)
        };

        let mut data = [0.0; 11];
        data[0] = target_distance;

        let eq_defaults = self
            .resolver
            .equality_defaults(dist.class.as_deref())
            .cloned();
        let mut active = dist.active;
        let mut solref = dist.solref;
        let mut solimp = dist.solimp;
        apply_eq_defaults(eq_defaults.as_ref(), &mut active, &mut solref, &mut solimp);

        self.eq_type.push(EqualityType::Distance);
        self.eq_obj1id.push(geom1_id);
        self.eq_obj2id.push(geom2_id);
        self.eq_data.push(data);
        self.eq_active.push(active.unwrap_or(true));
        self.eq_solimp.push(solimp.unwrap_or(DEFAULT_SOLIMP));
        self.eq_solref.push(solref.unwrap_or(DEFAULT_SOLREF));
        let eq_id = self.eq_name.len();
        if let Some(ref name) = dist.name {
            self.eq_name_to_id.insert(name.clone(), eq_id);
        }
        self.eq_name.push(dist.name.clone());
        Ok(())
    }

    /// Process a single Tendon equality constraint (polynomial coupling).
    fn process_tendon_equality(
        &mut self,
        ten_eq: &MjcfTendonEquality,
    ) -> std::result::Result<(), ModelConversionError> {
        let t1_id = *self
            .tendon_name_to_id
            .get(ten_eq.tendon1.as_str())
            .ok_or_else(|| ModelConversionError {
                message: format!(
                    "Tendon equality references unknown tendon1: '{}'",
                    ten_eq.tendon1
                ),
            })?;

        let t2_id = if let Some(ref t2_name) = ten_eq.tendon2 {
            *self
                .tendon_name_to_id
                .get(t2_name.as_str())
                .ok_or_else(|| ModelConversionError {
                    message: format!("Tendon equality references unknown tendon2: '{t2_name}'"),
                })?
        } else {
            usize::MAX
        };

        let mut data = [0.0; 11];
        for (i, &coef) in ten_eq.polycoef.iter().take(5).enumerate() {
            data[i] = coef;
        }

        let eq_defaults = self
            .resolver
            .equality_defaults(ten_eq.class.as_deref())
            .cloned();
        let mut active = ten_eq.active;
        let mut solref = ten_eq.solref;
        let mut solimp = ten_eq.solimp;
        apply_eq_defaults(eq_defaults.as_ref(), &mut active, &mut solref, &mut solimp);

        self.eq_type.push(EqualityType::Tendon);
        self.eq_obj1id.push(t1_id);
        self.eq_obj2id.push(t2_id);
        self.eq_data.push(data);
        self.eq_active.push(active.unwrap_or(true));
        self.eq_solimp.push(solimp.unwrap_or(DEFAULT_SOLIMP));
        self.eq_solref.push(solref.unwrap_or(DEFAULT_SOLREF));
        let eq_id = self.eq_name.len();
        if let Some(ref name) = ten_eq.name {
            self.eq_name_to_id.insert(name.clone(), eq_id);
        }
        self.eq_name.push(ten_eq.name.clone());
        Ok(())
    }

    /// Get world-space position and orientation of a body at build time.
    ///
    /// Body 0 (world) returns identity. Non-world bodies are indexed into
    /// `body_world_pos/quat` at `(body_id - 1)`.
    fn body_world_pose(&self, body_id: usize) -> (Vector3<f64>, nalgebra::UnitQuaternion<f64>) {
        if body_id == 0 {
            (Vector3::zeros(), nalgebra::UnitQuaternion::identity())
        } else {
            (
                self.body_world_pos[body_id - 1],
                self.body_world_quat[body_id - 1],
            )
        }
    }

    /// Compute world-space distance between two geom centers at build time.
    ///
    /// Used when `distance: None` to derive target from initial configuration.
    /// `geom2_id == usize::MAX` means world origin.
    fn compute_initial_geom_distance(&self, geom1_id: usize, geom2_id: usize) -> f64 {
        let p1 = self.geom_world_position(geom1_id);
        let p2 = if geom2_id == usize::MAX {
            Vector3::zeros()
        } else {
            self.geom_world_position(geom2_id)
        };
        (p1 - p2).norm()
    }

    /// Compute the world-space position of a geom at build time.
    fn geom_world_position(&self, geom_id: usize) -> Vector3<f64> {
        let body_id = self.geom_body[geom_id];
        if body_id == 0 {
            self.geom_pos[geom_id]
        } else {
            let body_world_pos = self.body_world_pos[body_id - 1];
            let body_world_quat = self.body_world_quat[body_id - 1];
            body_world_pos + body_world_quat * self.geom_pos[geom_id]
        }
    }
}
