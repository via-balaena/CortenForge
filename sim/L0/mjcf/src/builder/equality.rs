//! Equality constraint processing.
//!
//! Converts MJCF `<equality>` elements (connect, weld, joint, distance, tendon)
//! into Model arrays. Handles body/joint/geom/tendon name resolution, polynomial
//! coefficient packing, and default parameter application.

use nalgebra::Vector3;

use super::{DEFAULT_SOLIMP, DEFAULT_SOLREF, ModelBuilder, ModelConversionError};
use crate::types::{MjcfEquality, MjcfEqualityDefaults};
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
    /// Process equality constraints from MJCF.
    ///
    /// Converts `MjcfEquality` (connects, welds, joints, distances) into Model arrays.
    /// Must be called AFTER all bodies and joints are processed (requires name lookups).
    ///
    /// # MuJoCo Semantics
    ///
    /// - **Connect**: Ball-and-socket constraint. Removes 3 translational DOFs.
    ///   `eq_data[0..3]` = anchor point in body1's local frame.
    ///
    /// - **Weld**: Fixed frame constraint. Removes 6 DOFs (translation + rotation).
    ///   `eq_data[0..3]` = anchor point, `eq_data[3..7]` = relative quaternion [w,x,y,z].
    ///
    /// - **Joint**: Polynomial coupling between joints.
    ///   `eq_data[0..5]` = polynomial coefficients (q2 = c0 + c1*q1 + c2*q1² + ...).
    ///
    /// - **Distance**: Maintains fixed distance between geom centers.
    ///   `eq_data[0]` = target distance.
    pub(crate) fn process_equality_constraints(
        &mut self,
        equality: &MjcfEquality,
    ) -> std::result::Result<(), ModelConversionError> {
        // Process Connect constraints (3 DOF position constraint)
        for connect in &equality.connects {
            let body1_id =
                self.body_name_to_id
                    .get(&connect.body1)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "Connect constraint references unknown body1: '{}'",
                            connect.body1
                        ),
                    })?;

            // body2 defaults to world (body 0) if not specified
            let body2_id = if let Some(ref body2_name) = connect.body2 {
                *self
                    .body_name_to_id
                    .get(body2_name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "Connect constraint references unknown body2: '{body2_name}'"
                        ),
                    })?
            } else {
                0 // World body
            };

            // Apply equality defaults cascade
            let eq_defaults = self
                .resolver
                .equality_defaults(connect.class.as_deref())
                .cloned();
            let mut active = connect.active;
            let mut solref = connect.solref;
            let mut solimp = connect.solimp;
            apply_eq_defaults(eq_defaults.as_ref(), &mut active, &mut solref, &mut solimp);

            // Pack data: anchor point in body1 frame, rest zeroed
            let mut data = [0.0; 11];
            data[0] = connect.anchor.x;
            data[1] = connect.anchor.y;
            data[2] = connect.anchor.z;

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
        }

        // Process Weld constraints (6 DOF pose constraint)
        for weld in &equality.welds {
            let body1_id =
                self.body_name_to_id
                    .get(&weld.body1)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "Weld constraint references unknown body1: '{}'",
                            weld.body1
                        ),
                    })?;

            let body2_id = if let Some(ref body2_name) = weld.body2 {
                *self
                    .body_name_to_id
                    .get(body2_name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "Weld constraint references unknown body2: '{body2_name}'"
                        ),
                    })?
            } else {
                0 // World body
            };

            // Pack data: anchor point + relative pose quaternion
            // relpose format: [x, y, z, qw, qx, qy, qz]
            let mut data = [0.0; 11];
            data[0] = weld.anchor.x;
            data[1] = weld.anchor.y;
            data[2] = weld.anchor.z;
            if let Some(relpose) = weld.relpose {
                // relpose = [x, y, z, qw, qx, qy, qz]
                // We store anchor separately, so relpose goes to data[3..10]
                data[3] = relpose[3]; // qw
                data[4] = relpose[4]; // qx
                data[5] = relpose[5]; // qy
                data[6] = relpose[6]; // qz
                // Note: relpose position offset is in addition to anchor
                data[7] = relpose[0]; // dx
                data[8] = relpose[1]; // dy
                data[9] = relpose[2]; // dz
            } else {
                // Default: identity quaternion, no offset
                data[3] = 1.0; // qw = 1 (identity)
            }

            // Apply equality defaults cascade
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
        }

        // Process Joint coupling constraints
        for joint_eq in &equality.joints {
            let joint1_id = self.joint_name_to_id.get(&joint_eq.joint1).ok_or_else(|| {
                ModelConversionError {
                    message: format!(
                        "Joint equality references unknown joint1: '{}'",
                        joint_eq.joint1
                    ),
                }
            })?;

            // joint2 is optional - if not specified, constraint locks joint1 to polycoef[0]
            let joint2_id = if let Some(ref joint2_name) = joint_eq.joint2 {
                *self
                    .joint_name_to_id
                    .get(joint2_name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "Joint equality references unknown joint2: '{joint2_name}'"
                        ),
                    })?
            } else {
                // MuJoCo convention: joint2 = -1 means constraint locks joint1 to constant
                // We use usize::MAX as sentinel (will never be a valid joint ID)
                usize::MAX
            };

            // Pack polynomial coefficients (up to 5 terms)
            // MuJoCo: q2 = c0 + c1*q1 + c2*q1² + c3*q1³ + c4*q1⁴
            let mut data = [0.0; 11];
            for (i, &coef) in joint_eq.polycoef.iter().take(5).enumerate() {
                data[i] = coef;
            }

            // Apply equality defaults cascade
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
        }

        // Process Distance constraints (1 DOF scalar distance between geom centers)
        for dist in &equality.distances {
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

            // geom2 defaults to usize::MAX sentinel (world origin) if not specified
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
                d.max(0.0) // Distance is non-negative by definition
            } else {
                self.compute_initial_geom_distance(geom1_id, geom2_id)
            };

            let mut data = [0.0; 11];
            data[0] = target_distance;

            // Apply equality defaults cascade
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
        }

        // Process Tendon equality constraints
        for ten_eq in &equality.tendons {
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
                usize::MAX // Sentinel: single-tendon mode
            };

            // Pack polynomial coefficients (up to 5 terms)
            let mut data = [0.0; 11];
            for (i, &coef) in ten_eq.polycoef.iter().take(5).enumerate() {
                data[i] = coef;
            }

            // Apply equality defaults cascade
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
        }

        Ok(())
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
