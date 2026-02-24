//! Body tree traversal.
//!
//! Processes MJCF `<body>` elements into `Model` body arrays, recursively
//! traversing the kinematic tree. Handles worldbody geoms/sites, mocap
//! validation, inertia computation dispatch, and child body recursion.

use nalgebra::{UnitQuaternion, Vector3};
use sim_core::SleepPolicy;
use tracing::warn;

use super::mass::{compute_inertia_from_geoms, extract_inertial_properties};
use super::orientation::resolve_orientation;
use super::{ModelBuilder, ModelConversionError};
use crate::types::{InertiaFromGeom, MjcfBody, MjcfGeom};

impl ModelBuilder {
    /// Process geoms and sites directly attached to worldbody (body 0).
    ///
    /// In MJCF, the worldbody can have geoms (like ground planes) and sites
    /// directly attached to it. These are static geometries at world coordinates.
    ///
    /// No childclass is applied here — MuJoCo's worldbody accepts no attributes
    /// (including childclass), so worldbody elements use only their own explicit
    /// class or the unnamed top-level default.
    pub(crate) fn process_worldbody_geoms_and_sites(
        &mut self,
        worldbody: &MjcfBody,
    ) -> std::result::Result<(), ModelConversionError> {
        // Track geom start address for body 0
        let geom_adr = self.geom_type.len();

        // Process worldbody geoms
        for geom in &worldbody.geoms {
            let geom = self.resolver.apply_to_geom(geom);
            self.process_geom(&geom, 0)?;
        }

        // Process worldbody sites
        for site in &worldbody.sites {
            let site = self.resolver.apply_to_site(site);
            self.process_site(&site, 0)?;
        }

        // Update body 0's geom range
        let num_geoms = self.geom_type.len() - geom_adr;
        self.body_geom_adr[0] = geom_adr;
        self.body_geom_num[0] = num_geoms;

        Ok(())
    }

    /// Process a body and its descendants.
    ///
    /// This is the public-facing entry point that initializes world frame tracking.
    ///
    /// # Arguments
    /// * `body` - The MJCF body to process
    /// * `parent_id` - Index of the parent body in the Model
    /// * `parent_last_dof` - Index of the last DOF in the parent body's kinematic chain
    ///   (None for bodies attached to world)
    ///
    /// # Returns
    /// The body index in the Model
    pub(crate) fn process_body(
        &mut self,
        body: &MjcfBody,
        parent_id: usize,
        parent_last_dof: Option<usize>,
        inherited_childclass: Option<&str>,
    ) -> std::result::Result<usize, ModelConversionError> {
        // Start with parent's world frame (world body is at origin)
        let (parent_world_pos, parent_world_quat) = if parent_id == 0 {
            (Vector3::zeros(), UnitQuaternion::identity())
        } else {
            // For non-world parents, we need to look up their world positions
            // Since we process in topological order, parent is already processed
            // We stored parent's world position when we added them
            (
                self.body_world_pos[parent_id - 1],
                self.body_world_quat[parent_id - 1],
            )
        };
        self.process_body_with_world_frame(
            body,
            parent_id,
            parent_last_dof,
            parent_world_pos,
            parent_world_quat,
            inherited_childclass,
        )
    }

    /// Internal body processing with world frame tracking.
    fn process_body_with_world_frame(
        &mut self,
        body: &MjcfBody,
        parent_id: usize,
        parent_last_dof: Option<usize>,
        parent_world_pos: Vector3<f64>,
        parent_world_quat: UnitQuaternion<f64>,
        inherited_childclass: Option<&str>,
    ) -> std::result::Result<usize, ModelConversionError> {
        let body_id = self.body_parent.len();

        // Validate mocap body constraints (before any state mutations)
        if body.mocap {
            if parent_id != 0 {
                return Err(ModelConversionError {
                    message: format!(
                        "mocap body '{}' must be a direct child of worldbody",
                        body.name
                    ),
                });
            }
            if !body.joints.is_empty() {
                return Err(ModelConversionError {
                    message: format!(
                        "mocap body '{}' must not have joints (has {})",
                        body.name,
                        body.joints.len()
                    ),
                });
            }
        }

        // Store name mapping
        if !body.name.is_empty() {
            self.body_name_to_id.insert(body.name.clone(), body_id);
        }

        // Determine root: if parent is world, this body is its own root
        let root_id = if parent_id == 0 {
            body_id
        } else {
            self.body_rootid[parent_id]
        };

        // Body position/orientation relative to parent.
        // Priority: euler > axisangle > xyaxes > zaxis > quat (MuJoCo convention).
        let body_pos = body.pos;
        let body_quat = resolve_orientation(
            body.quat,
            body.euler,
            body.axisangle,
            None,
            None,
            &self.compiler,
        );

        // Compute world frame position for this body (used for free joint qpos0)
        let world_pos = parent_world_pos + parent_world_quat * body_pos;
        let world_quat = parent_world_quat * body_quat;

        // Store world positions for child body processing
        self.body_world_pos.push(world_pos);
        self.body_world_quat.push(world_quat);

        // Determine effective childclass for this body's children
        let effective_childclass = body.childclass.as_deref().or(inherited_childclass);

        // Resolve geom defaults once for both inertia computation and geom processing.
        // Apply childclass: if a geom has no explicit class, set it from effective_childclass.
        let resolved_geoms: Vec<MjcfGeom> = body
            .geoms
            .iter()
            .map(|g| {
                let mut g = g.clone();
                if g.class.is_none() {
                    g.class = effective_childclass.map(|s| s.to_string());
                }
                self.resolver.apply_to_geom(&g)
            })
            .collect();

        // Process inertial properties with full MuJoCo semantics.
        // Gated by compiler.inertiafromgeom:
        //   True  — always compute from geoms (overrides explicit <inertial>)
        //   Auto  — compute from geoms only when no explicit <inertial>
        //   False — use explicit <inertial> or zero
        let (mass, inertia, ipos, iquat) = match self.compiler.inertiafromgeom {
            InertiaFromGeom::True => {
                compute_inertia_from_geoms(&resolved_geoms, &self.mesh_name_to_id, &self.mesh_data)
            }
            InertiaFromGeom::Auto => {
                if let Some(ref inertial) = body.inertial {
                    extract_inertial_properties(inertial)
                } else {
                    compute_inertia_from_geoms(
                        &resolved_geoms,
                        &self.mesh_name_to_id,
                        &self.mesh_data,
                    )
                }
            }
            InertiaFromGeom::False => {
                if let Some(ref inertial) = body.inertial {
                    extract_inertial_properties(inertial)
                } else {
                    (
                        0.0,
                        Vector3::zeros(),
                        Vector3::zeros(),
                        UnitQuaternion::identity(),
                    )
                }
            }
        };

        // Track joint/DOF addresses for this body
        let jnt_adr = self.jnt_type.len();
        let dof_adr = self.nv;
        let geom_adr = self.geom_type.len();

        // Add body arrays
        self.body_parent.push(parent_id);
        self.body_rootid.push(root_id);
        self.body_jnt_adr.push(jnt_adr);
        self.body_dof_adr.push(dof_adr);
        self.body_geom_adr.push(geom_adr);
        self.body_pos.push(body_pos);
        self.body_quat.push(body_quat);
        self.body_ipos.push(ipos);
        self.body_iquat.push(iquat);
        self.body_mass.push(mass);
        self.body_inertia.push(inertia);
        if body.mocap {
            self.body_mocapid.push(Some(self.nmocap));
            self.nmocap += 1;
        } else {
            self.body_mocapid.push(None);
        }
        self.body_name.push(if body.name.is_empty() {
            None
        } else {
            Some(body.name.clone())
        });
        // Parse body-level sleep policy
        let sleep_policy = body.sleep.as_ref().and_then(|s| match s.as_str() {
            "auto" => Some(SleepPolicy::Auto),
            "allowed" => Some(SleepPolicy::Allowed),
            "never" => Some(SleepPolicy::Never),
            "init" => Some(SleepPolicy::Init),
            _ => {
                warn!(
                    "body '{}': unknown sleep policy '{}', ignoring",
                    body.name, s
                );
                None
            }
        });
        self.body_sleep_policy.push(sleep_policy);
        self.body_gravcomp.push(body.gravcomp.unwrap_or(0.0));

        // Process joints for this body, tracking the last DOF for kinematic tree linkage
        // MuJoCo semantics: first DOF of first joint links to parent body's last DOF,
        // subsequent DOFs form a chain within and across joints
        let mut body_nv = 0;
        let mut current_last_dof = parent_last_dof;

        for joint in &body.joints {
            let mut joint = joint.clone();
            if joint.class.is_none() {
                joint.class = effective_childclass.map(|s| s.to_string());
            }
            let joint = self.resolver.apply_to_joint(&joint);
            let jnt_id =
                self.process_joint(&joint, body_id, current_last_dof, world_pos, world_quat)?;
            let jnt_nv = self.jnt_type[jnt_id].nv();
            body_nv += jnt_nv;

            // Update current_last_dof to the last DOF of this joint
            if jnt_nv > 0 {
                current_last_dof = Some(self.nv - 1);
            }
        }

        // Update body joint/DOF counts
        self.body_jnt_num.push(body.joints.len());
        self.body_dof_num.push(body_nv);

        // Process geoms for this body (using pre-resolved defaults)
        for geom in &resolved_geoms {
            self.process_geom(geom, body_id)?;
        }
        self.body_geom_num.push(body.geoms.len());

        // Process sites for this body
        for site in &body.sites {
            let mut site = site.clone();
            if site.class.is_none() {
                site.class = effective_childclass.map(|s| s.to_string());
            }
            let site = self.resolver.apply_to_site(&site);
            self.process_site(&site, body_id)?;
        }

        // Recursively process children, passing this body's last DOF and world frame
        for child in &body.children {
            self.process_body_with_world_frame(
                child,
                body_id,
                current_last_dof,
                world_pos,
                world_quat,
                effective_childclass,
            )?;
        }

        Ok(body_id)
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use crate::builder::load_model;
    use crate::builder::orientation::euler_seq_to_quat;
    use nalgebra::{UnitQuaternion, Vector3};
    use sim_core::GeomType;

    /// Test site parsing and model population.
    #[test]
    fn test_sites() {
        let model = load_model(
            r#"
            <mujoco model="site_test">
                <worldbody>
                    <body name="arm" pos="0 0 1">
                        <joint name="j1" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <geom type="capsule" size="0.05 0.5"/>
                        <site name="end_effector" pos="0 0 -1" size="0.02"/>
                        <site name="sensor_mount" pos="0.1 0 0" size="0.01 0.01 0.02"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nsite, 2);
        assert_eq!(model.site_body.len(), 2);

        // Check first site (end_effector)
        assert_eq!(model.site_body[0], 1); // Attached to arm (body 1)
        assert!((model.site_pos[0].z - (-1.0)).abs() < 1e-10);
        assert_eq!(model.site_name[0], Some("end_effector".to_string()));

        // Check second site (sensor_mount)
        assert_eq!(model.site_body[1], 1);
        assert!((model.site_pos[1].x - 0.1).abs() < 1e-10);
    }

    // =========================================================================
    // Site orientation tests (item #21)
    // =========================================================================

    /// AC1: Site euler orientation with default angle="degree".
    #[test]
    fn test_site_euler_orientation_degrees() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" euler="90 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let expected = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 90.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site euler 90deg X deg: got {got:?}, expected {expected:?}"
        );
    }

    /// AC2: Site euler orientation with explicit angle="radian".
    #[test]
    fn test_site_euler_orientation_radians() {
        let pi_2 = std::f64::consts::FRAC_PI_2;
        let model = load_model(&format!(
            r#"
            <mujoco>
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" euler="{pi_2} 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        ))
        .expect("should load");

        let expected = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), pi_2);
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site euler pi/2 X rad: got {got:?}, expected {expected:?}"
        );
    }

    /// AC3: Site euler with non-default eulerseq="ZYX".
    #[test]
    fn test_site_euler_orientation_zyx_eulerseq() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" eulerseq="ZYX"/>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" euler="0.3 0.2 0.1"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let expected = euler_seq_to_quat(Vector3::new(0.3, 0.2, 0.1), "ZYX");
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site euler ZYX: got {got:?}, expected {expected:?}"
        );
    }

    /// AC4: Site axisangle orientation with default angle="degree".
    #[test]
    fn test_site_axisangle_orientation_degrees() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" axisangle="0 0 1 90"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site axisangle 90deg Z deg: got {got:?}, expected {expected:?}"
        );
    }

    /// AC5: Site axisangle orientation with angle="radian".
    #[test]
    fn test_site_axisangle_orientation_radians() {
        let pi_2 = std::f64::consts::FRAC_PI_2;
        let model = load_model(&format!(
            r#"
            <mujoco>
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" axisangle="0 1 0 {pi_2}"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        ))
        .expect("should load");

        let expected = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), pi_2);
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site axisangle pi/2 Y rad: got {got:?}, expected {expected:?}"
        );
    }

    /// AC6: Site xyaxes orientation with orthogonal inputs (90deg Z rotation).
    #[test]
    fn test_site_xyaxes_orientation_orthogonal() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" xyaxes="0 1 0 -1 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // x=(0,1,0), y=(-1,0,0), z=cross(x,y)=(0,0,1) -> 90deg about Z
        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site xyaxes orthogonal 90deg Z: got {got:?}, expected {expected:?}"
        );
    }

    /// AC7: Site xyaxes with non-orthogonal inputs (Gram-Schmidt -> 45deg Z rotation).
    #[test]
    fn test_site_xyaxes_orientation_gram_schmidt() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" xyaxes="1 1 0 0 1 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Gram-Schmidt: x=norm(1,1,0)=(1/sqrt2, 1/sqrt2, 0), y orthogonalized -> 45deg about Z
        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 45.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-6,
            "site xyaxes Gram-Schmidt 45deg Z: got {got:?}, expected {expected:?}"
        );
    }

    /// AC8: Site zaxis general direction (Z->X = 90deg about Y).
    #[test]
    fn test_site_zaxis_orientation_general() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" zaxis="1 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Minimal rotation from (0,0,1) to (1,0,0): 90deg about Y
        let expected =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f64::consts::FRAC_PI_2);
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site zaxis Z->X (90deg Y): got {got:?}, expected {expected:?}"
        );
    }

    /// AC9: Site zaxis parallel to default Z -> identity.
    #[test]
    fn test_site_zaxis_orientation_parallel_identity() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" zaxis="0 0 1"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let expected = UnitQuaternion::identity();
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site zaxis (0,0,1) identity: got {got:?}, expected {expected:?}"
        );
    }

    /// AC10: Site zaxis anti-parallel (0,0,-1) -> 180deg about X.
    #[test]
    fn test_site_zaxis_orientation_antiparallel() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" zaxis="0 0 -1"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Anti-parallel: fallback axis = X, angle = atan2(0, -1) = PI -> 180deg about X
        let expected = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::PI);
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site zaxis anti-parallel 180deg X: got {got:?}, expected {expected:?}"
        );
    }

    /// AC11: Site quat orientation (regression for pre-#19 behavior).
    #[test]
    fn test_site_quat_orientation_regression() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" quat="0.7071068 0 0.7071068 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Direct quaternion (wxyz): 90deg about Y
        let expected =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f64::consts::FRAC_PI_2);
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-6,
            "site quat regression 90deg Y: got {got:?}, expected {expected:?}"
        );
    }

    /// AC12: Site orientation with default class (defaults provide type/size,
    /// element provides orientation — no interference).
    #[test]
    fn test_site_orientation_with_default_class() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="sensor_site">
                        <site type="cylinder" size="0.02 0.01"/>
                    </default>
                </default>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" class="sensor_site" euler="0 0 90"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Type from default class
        assert_eq!(
            model.site_type[0],
            GeomType::Cylinder,
            "site type should come from default class"
        );

        // Orientation from element (not interfered by defaults)
        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site euler 90deg Z with default class: got {got:?}, expected {expected:?}"
        );
    }

    /// AC13: Orientation priority — euler takes precedence over quat when both specified.
    #[test]
    fn test_site_orientation_priority_euler_over_quat() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" euler="90 0 0" quat="1 0 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // euler wins over quat per priority chain. quat=(1,0,0,0) is identity;
        // euler="90 0 0" is 90deg X. Result must be 90deg X, NOT identity.
        let expected = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 90.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "euler should take priority over quat: got {got:?}, expected {expected:?}"
        );
    }

    /// AC14: Site orientation composed with frame rotation.
    #[test]
    fn test_site_orientation_in_frame() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame euler="0 0 90">
                            <site name="s" euler="90 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Frame: 90deg Z, Site: 90deg X. Composed: frame_q * site_q
        let frame_q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90.0_f64.to_radians());
        let site_q = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 90.0_f64.to_radians());
        let expected = frame_q * site_q;
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-6,
            "site in frame composed: got {got:?}, expected {expected:?}"
        );
    }

    /// AC15: Site axisangle with non-unit axis (normalization).
    #[test]
    fn test_site_axisangle_non_unit_axis() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" axisangle="0 0 3 90"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Axis (0,0,3) normalizes to (0,0,1) -> same as axisangle="0 0 1 90"
        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site axisangle non-unit axis: got {got:?}, expected {expected:?}"
        );
    }

    /// AC16: Site zaxis with non-unit direction (normalization, parallel to Z).
    #[test]
    fn test_site_zaxis_non_unit_direction() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" zaxis="0 0 5"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // (0,0,5) normalizes to (0,0,1) -> parallel to default Z -> identity
        let expected = UnitQuaternion::identity();
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site zaxis non-unit parallel: got {got:?}, expected {expected:?}"
        );
    }
}
