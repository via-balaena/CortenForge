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

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use crate::builder::load_model;
    use approx::assert_relative_eq;
    use sim_core::EqualityType;

    /// Connect with body2=world (omitted attribute) defaults `body2_id` to 0
    /// and pushes one Connect equality with the named anchor in body1's frame.
    #[test]
    fn connect_with_world_body2() {
        let xml = r#"
            <mujoco model="m">
                <worldbody>
                    <body name="a" pos="0 0 1">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
                <equality>
                    <connect name="c1" body1="a" anchor="0.2 0 0"/>
                </equality>
            </mujoco>
        "#;
        let model = load_model(xml).expect("load");
        assert_eq!(model.neq, 1);
        assert_eq!(model.eq_type[0], EqualityType::Connect);
        assert_eq!(model.eq_obj2id[0], 0, "world body");
        assert_relative_eq!(model.eq_data[0][0], 0.2, epsilon = 1e-12);
        assert_eq!(model.eq_name[0].as_deref(), Some("c1"));
        assert_eq!(model.eq_name_to_id.get("c1"), Some(&0));
    }

    /// Connect with explicit body2 auto-computes the body2 anchor so both
    /// anchors map to the same world-space point at qpos0. The check rebuilds
    /// world coordinates from each side independently and compares.
    #[test]
    fn connect_explicit_body2_autocomputes_anchor() {
        let xml = r#"
            <mujoco model="m">
                <worldbody>
                    <body name="a" pos="1 0 0">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                    <body name="b" pos="0 1 0">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
                <equality>
                    <connect body1="a" body2="b" anchor="0.5 0 0"/>
                </equality>
            </mujoco>
        "#;
        let model = load_model(xml).expect("load");
        assert_eq!(model.neq, 1);
        let a = *model.body_name_to_id.get("a").unwrap();
        let b = *model.body_name_to_id.get("b").unwrap();
        assert_eq!(model.eq_obj1id[0], a);
        assert_eq!(model.eq_obj2id[0], b);
        let anchor1 = nalgebra::Vector3::new(
            model.eq_data[0][0],
            model.eq_data[0][1],
            model.eq_data[0][2],
        );
        let anchor2 = nalgebra::Vector3::new(
            model.eq_data[0][3],
            model.eq_data[0][4],
            model.eq_data[0][5],
        );
        // body a at (1,0,0), b at (0,1,0); anchor in a-frame is (0.5,0,0)
        // → world = (1.5, 0, 0); anchor in b-frame = (1.5, -1, 0).
        let world_via_a = nalgebra::Vector3::new(1.0, 0.0, 0.0) + anchor1;
        let world_via_b = nalgebra::Vector3::new(0.0, 1.0, 0.0) + anchor2;
        assert_relative_eq!((world_via_a - world_via_b).norm(), 0.0, epsilon = 1e-12);
    }

    /// Weld with explicit `relpose` packs the user-supplied quaternion into
    /// `eq_data[6..10]` verbatim instead of computing one from world frames.
    #[test]
    fn weld_explicit_relpose_packed_verbatim() {
        let xml = r#"
            <mujoco model="m">
                <worldbody>
                    <body name="a">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
                <equality>
                    <weld body1="a" body2="b" anchor="0 0 0"
                          relpose="0 0 0 1 0 0 0"/>
                </equality>
            </mujoco>
        "#;
        let model = load_model(xml).expect("load");
        assert_eq!(model.eq_type[0], EqualityType::Weld);
        // relpose attr layout is [x y z qw qx qy qz] → data[6..10] = [qw,qx,qy,qz]
        assert_relative_eq!(model.eq_data[0][6], 1.0, epsilon = 1e-12);
        assert_relative_eq!(model.eq_data[0][7], 0.0, epsilon = 1e-12);
        assert_relative_eq!(model.eq_data[0][8], 0.0, epsilon = 1e-12);
        assert_relative_eq!(model.eq_data[0][9], 0.0, epsilon = 1e-12);
    }

    /// Weld without `relpose` auto-computes `q1.inverse() * q2`. With both
    /// bodies at identity orientation, the relpose is identity (1,0,0,0).
    #[test]
    fn weld_auto_relpose_identity_when_aligned() {
        let xml = r#"
            <mujoco model="m">
                <worldbody>
                    <body name="a" pos="1 0 0">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                    <body name="b" pos="0 1 0">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
                <equality>
                    <weld body1="a" body2="b" anchor="0 0 0"/>
                </equality>
            </mujoco>
        "#;
        let model = load_model(xml).expect("load");
        // Both bodies at identity orientation → q1⁻¹·q2 = identity quat.
        assert_relative_eq!(model.eq_data[0][6], 1.0, epsilon = 1e-12);
        assert_relative_eq!(model.eq_data[0][7], 0.0, epsilon = 1e-12);
        assert_relative_eq!(model.eq_data[0][8], 0.0, epsilon = 1e-12);
        assert_relative_eq!(model.eq_data[0][9], 0.0, epsilon = 1e-12);
    }

    /// Joint equality packs polycoef into `eq_data[0..5]` and resolves both
    /// joint names; coefficients beyond index 4 are dropped (5-element cap).
    #[test]
    fn joint_equality_polycoef_packed_into_data() {
        let xml = r#"
            <mujoco model="m">
                <worldbody>
                    <body name="a">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <body name="b">
                            <joint name="j2" type="hinge" axis="0 0 1"/>
                            <geom type="sphere" size="0.1" mass="1.0"/>
                        </body>
                    </body>
                </worldbody>
                <equality>
                    <joint joint1="j1" joint2="j2"
                           polycoef="0.1 0.2 0.3 0.4 0.5"/>
                </equality>
            </mujoco>
        "#;
        let model = load_model(xml).expect("load");
        assert_eq!(model.eq_type[0], EqualityType::Joint);
        for (i, expect) in [0.1, 0.2, 0.3, 0.4, 0.5].iter().enumerate() {
            assert_relative_eq!(model.eq_data[0][i], *expect, epsilon = 1e-12);
        }
    }

    /// Joint equality with no `joint2` records `usize::MAX` as the second
    /// object id (the sentinel for "no second joint" used downstream).
    #[test]
    fn joint_equality_omitted_joint2_sentinel() {
        let xml = r#"
            <mujoco model="m">
                <worldbody>
                    <body name="a">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
                <equality>
                    <joint joint1="j1" polycoef="1 0 0 0 0"/>
                </equality>
            </mujoco>
        "#;
        let model = load_model(xml).expect("load");
        assert_eq!(model.eq_obj2id[0], usize::MAX);
    }

    /// Distance with no explicit `distance` attribute auto-computes the
    /// target as the world-space distance between the two geom centers at
    /// qpos0. With geoms at (1,0,0) and (0,1,0), expect √2.
    #[test]
    fn distance_auto_computed_from_geom_world_positions() {
        let xml = r#"
            <mujoco model="m">
                <worldbody>
                    <body name="ba" pos="1 0 0">
                        <geom name="g1" type="sphere" size="0.05" mass="1.0"/>
                    </body>
                    <body name="bb" pos="0 1 0">
                        <geom name="g2" type="sphere" size="0.05" mass="1.0"/>
                    </body>
                </worldbody>
                <equality>
                    <distance geom1="g1" geom2="g2"/>
                </equality>
            </mujoco>
        "#;
        let model = load_model(xml).expect("load");
        assert_eq!(model.eq_type[0], EqualityType::Distance);
        let expected = (2.0_f64).sqrt();
        assert_relative_eq!(model.eq_data[0][0], expected, epsilon = 1e-12);
    }

    /// Distance with an explicit negative target is clamped to 0 (the
    /// `d.max(0.0)` guard at process_distance:311). Also covers the
    /// `geom2`-omitted path which records `usize::MAX` as obj2id.
    #[test]
    fn distance_negative_clamped_and_geom2_world_origin() {
        let xml = r#"
            <mujoco model="m">
                <worldbody>
                    <body name="ba" pos="1 0 0">
                        <geom name="g1" type="sphere" size="0.05" mass="1.0"/>
                    </body>
                </worldbody>
                <equality>
                    <distance geom1="g1" distance="-3.0"/>
                </equality>
            </mujoco>
        "#;
        let model = load_model(xml).expect("load");
        assert_relative_eq!(model.eq_data[0][0], 0.0, epsilon = 1e-12);
        assert_eq!(model.eq_obj2id[0], usize::MAX);
    }

    /// Tendon equality with a single fixed tendon resolves `tendon1` to its
    /// id and packs polycoef into `eq_data[0..5]`. Also covers the
    /// `tendon2`-omitted sentinel branch.
    #[test]
    fn tendon_equality_resolves_tendon_id_and_polycoef() {
        let xml = r#"
            <mujoco model="m">
                <worldbody>
                    <body name="a">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
                <tendon>
                    <fixed name="t1">
                        <joint joint="j1" coef="1.0"/>
                    </fixed>
                </tendon>
                <equality>
                    <tendon tendon1="t1" polycoef="2.0 0 0 0 0"/>
                </equality>
            </mujoco>
        "#;
        let model = load_model(xml).expect("load");
        assert_eq!(model.eq_type[0], EqualityType::Tendon);
        assert_eq!(model.eq_obj1id[0], 0, "first tendon");
        assert_eq!(model.eq_obj2id[0], usize::MAX, "no tendon2");
        assert_relative_eq!(model.eq_data[0][0], 2.0, epsilon = 1e-12);
    }

    /// Equality constraints are stored in **document order** regardless of
    /// type, matching MuJoCo's eq_id assignment. Class defaults from
    /// `<default class="cl"><equality solref=…/></default>` are also
    /// applied via `apply_eq_defaults` when the element doesn't override.
    #[test]
    fn equality_document_order_and_class_defaults() {
        let xml = r#"
            <mujoco model="m">
                <default>
                    <default class="cl">
                        <equality solref="0.05 1.5"/>
                    </default>
                </default>
                <worldbody>
                    <body name="a">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
                <equality>
                    <connect body1="a" anchor="0 0 0"/>
                    <joint joint1="j1" polycoef="1 0 0 0 0" class="cl"/>
                    <weld body1="b" anchor="0 0 0"/>
                </equality>
            </mujoco>
        "#;
        let model = load_model(xml).expect("load");
        assert_eq!(model.neq, 3);
        // Order: Connect (0) → Joint (1) → Weld (2).
        assert_eq!(model.eq_type[0], EqualityType::Connect);
        assert_eq!(model.eq_type[1], EqualityType::Joint);
        assert_eq!(model.eq_type[2], EqualityType::Weld);
        // Joint constraint inherits solref from class "cl".
        assert_relative_eq!(model.eq_solref[1][0], 0.05, epsilon = 1e-12);
        assert_relative_eq!(model.eq_solref[1][1], 1.5, epsilon = 1e-12);
    }

    /// Each `process_*` resolves names via `body_name_to_id`/
    /// `joint_name_to_id`/`geom_name_to_id`/`tendon_name_to_id` and returns
    /// a `ModelConversionError` containing the offending name if missing.
    /// One test sweeps all four resolver branches by trying each in turn.
    #[test]
    fn equality_unknown_name_errors() {
        let cases: &[(&str, &str)] = &[
            (
                r#"<mujoco model="m"><worldbody>
                    <body name="a"><geom type="sphere" size="0.1" mass="1.0"/></body>
                </worldbody><equality>
                    <connect body1="ghost" anchor="0 0 0"/>
                </equality></mujoco>"#,
                "ghost",
            ),
            (
                r#"<mujoco model="m"><worldbody>
                    <body name="a">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody><equality>
                    <joint joint1="ghost" polycoef="1 0 0 0 0"/>
                </equality></mujoco>"#,
                "ghost",
            ),
            (
                r#"<mujoco model="m"><worldbody>
                    <body name="a"><geom name="g1" type="sphere" size="0.1" mass="1.0"/></body>
                </worldbody><equality>
                    <distance geom1="ghost"/>
                </equality></mujoco>"#,
                "ghost",
            ),
            (
                r#"<mujoco model="m"><worldbody>
                    <body name="a">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
                <tendon><fixed name="t1"><joint joint="j1" coef="1.0"/></fixed></tendon>
                <equality>
                    <tendon tendon1="ghost" polycoef="1 0 0 0 0"/>
                </equality></mujoco>"#,
                "ghost",
            ),
        ];
        for (xml, needle) in cases {
            let err = load_model(xml).expect_err("should reject unknown name");
            let msg = err.to_string();
            assert!(
                msg.contains(needle),
                "error did not name the missing entity: {msg}"
            );
        }
    }
}
