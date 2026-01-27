//! MJCF to Model conversion (Phase 4 of MuJoCo consolidation).
//!
//! This module converts parsed `MjcfModel` structures directly into the
//! MuJoCo-aligned `Model` struct from sim-core.
//!
//! # MuJoCo Semantics
//!
//! The conversion follows MuJoCo's exact semantics for:
//! - **dof_parent**: Forms a kinematic tree linking DOFs across joints and bodies
//! - **nq vs nv**: Position dimension can exceed velocity dimension (e.g., Ball: nq=4, nv=3)
//! - **Inertia computation**: Parallel axis theorem for composite bodies
//! - **Capsule inertia**: Exact formula including hemispherical end caps

use nalgebra::{DVector, Matrix3, UnitQuaternion, Vector3};
use sim_core::{ActuatorDynamics, ActuatorTransmission, GeomType, Integrator, MjJointType, Model};
use tracing::warn;

use crate::error::Result;
use crate::types::{
    MjcfActuator, MjcfActuatorType, MjcfBody, MjcfGeom, MjcfGeomType, MjcfInertial, MjcfIntegrator,
    MjcfJoint, MjcfJointType, MjcfModel, MjcfOption, MjcfSite,
};

/// Error type for MJCF to Model conversion.
#[derive(Debug, Clone)]
pub struct ModelConversionError {
    /// Error message.
    pub message: String,
}

impl std::fmt::Display for ModelConversionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "MJCF to Model conversion error: {}", self.message)
    }
}

impl std::error::Error for ModelConversionError {}

/// Convert a parsed `MjcfModel` to a sim-core `Model`.
///
/// This is the primary entry point for the new Model-based MJCF loading.
/// It builds all Model arrays in a single tree traversal.
///
/// # Example
///
/// ```ignore
/// use sim_mjcf::{parse_mjcf_str, model_from_mjcf};
///
/// let mjcf = parse_mjcf_str(r#"
///     <mujoco>
///         <worldbody>
///             <body name="pendulum" pos="0 0 1">
///                 <joint type="hinge" axis="0 1 0"/>
///                 <geom type="capsule" size="0.05 0.5"/>
///             </body>
///         </worldbody>
///     </mujoco>
/// "#)?;
///
/// let model = model_from_mjcf(&mjcf)?;
/// let mut data = model.make_data();
/// data.step(&model);
/// ```
pub fn model_from_mjcf(mjcf: &MjcfModel) -> std::result::Result<Model, ModelConversionError> {
    let mut builder = ModelBuilder::new();

    // Set global options
    builder.set_options(&mjcf.option);

    // Process worldbody's own geoms and sites (attached to body 0)
    // These must be processed BEFORE child bodies so geom indices are correct
    builder.process_worldbody_geoms_and_sites(&mjcf.worldbody)?;

    // Recursively process body tree starting from worldbody children
    // World body (body 0) has no DOFs, so parent_last_dof is None
    for child in &mjcf.worldbody.children {
        builder.process_body(child, 0, None)?;
    }

    // Process actuators
    for actuator in &mjcf.actuators {
        builder.process_actuator(actuator)?;
    }

    // Build final model
    Ok(builder.build())
}

/// Parse MJCF XML and convert directly to Model.
///
/// Convenience function that combines parsing and conversion.
pub fn load_model(xml: &str) -> Result<Model> {
    let mjcf = crate::parse_mjcf_str(xml)?;
    model_from_mjcf(&mjcf).map_err(|e| crate::error::MjcfError::Unsupported(e.message))
}

/// Builder for constructing Model from MJCF.
struct ModelBuilder {
    // Dimensions (computed during building)
    nq: usize,
    nv: usize,

    // Body arrays
    body_parent: Vec<usize>,
    body_rootid: Vec<usize>,
    body_jnt_adr: Vec<usize>,
    body_jnt_num: Vec<usize>,
    body_dof_adr: Vec<usize>,
    body_dof_num: Vec<usize>,
    body_geom_adr: Vec<usize>,
    body_geom_num: Vec<usize>,
    body_pos: Vec<Vector3<f64>>,
    body_quat: Vec<UnitQuaternion<f64>>,
    body_ipos: Vec<Vector3<f64>>,
    body_iquat: Vec<UnitQuaternion<f64>>,
    body_mass: Vec<f64>,
    body_inertia: Vec<Vector3<f64>>,
    body_name: Vec<Option<String>>,

    // Joint arrays
    jnt_type: Vec<MjJointType>,
    jnt_body: Vec<usize>,
    jnt_qpos_adr: Vec<usize>,
    jnt_dof_adr: Vec<usize>,
    jnt_pos: Vec<Vector3<f64>>,
    jnt_axis: Vec<Vector3<f64>>,
    jnt_limited: Vec<bool>,
    jnt_range: Vec<(f64, f64)>,
    jnt_stiffness: Vec<f64>,
    jnt_damping: Vec<f64>,
    jnt_armature: Vec<f64>,
    jnt_name: Vec<Option<String>>,

    // DOF arrays
    dof_body: Vec<usize>,
    dof_jnt: Vec<usize>,
    dof_parent: Vec<Option<usize>>,
    dof_armature: Vec<f64>,
    dof_damping: Vec<f64>,

    // Geom arrays
    geom_type: Vec<GeomType>,
    geom_body: Vec<usize>,
    geom_pos: Vec<Vector3<f64>>,
    geom_quat: Vec<UnitQuaternion<f64>>,
    geom_size: Vec<Vector3<f64>>,
    geom_friction: Vec<Vector3<f64>>,
    geom_contype: Vec<u32>,
    geom_conaffinity: Vec<u32>,
    geom_name: Vec<Option<String>>,

    // Site arrays
    site_body: Vec<usize>,
    site_type: Vec<GeomType>,
    site_pos: Vec<Vector3<f64>>,
    site_quat: Vec<UnitQuaternion<f64>>,
    site_size: Vec<Vector3<f64>>,
    site_name: Vec<Option<String>>,

    // World frame tracking (for free joint qpos0 initialization)
    // Stores accumulated world positions during tree traversal
    body_world_pos: Vec<Vector3<f64>>,
    body_world_quat: Vec<UnitQuaternion<f64>>,

    // Actuator arrays
    actuator_trntype: Vec<ActuatorTransmission>,
    actuator_dyntype: Vec<ActuatorDynamics>,
    actuator_trnid: Vec<usize>,
    actuator_gear: Vec<f64>,
    actuator_ctrlrange: Vec<(f64, f64)>,
    actuator_forcerange: Vec<(f64, f64)>,
    actuator_name: Vec<Option<String>>,
    actuator_act_adr: Vec<usize>,
    actuator_act_num: Vec<usize>,

    // Total activation states (sum of actuator_act_num)
    na: usize,

    // Options
    timestep: f64,
    gravity: Vector3<f64>,
    wind: Vector3<f64>,
    magnetic: Vector3<f64>,
    density: f64,
    viscosity: f64,
    solver_iterations: usize,
    solver_tolerance: f64,
    impratio: f64,
    cone: u8,
    disableflags: u32,
    enableflags: u32,
    integrator: Integrator,

    // qpos0 values (built as we process joints)
    qpos0_values: Vec<f64>,

    // Name to index lookups (for actuator wiring)
    joint_name_to_id: std::collections::HashMap<String, usize>,
    body_name_to_id: std::collections::HashMap<String, usize>,
    site_name_to_id: std::collections::HashMap<String, usize>,
}

impl ModelBuilder {
    fn new() -> Self {
        // Initialize with world body (body 0)
        Self {
            nq: 0,
            nv: 0,

            // World body initialized
            body_parent: vec![0],
            body_rootid: vec![0],
            body_jnt_adr: vec![0],
            body_jnt_num: vec![0],
            body_dof_adr: vec![0],
            body_dof_num: vec![0],
            body_geom_adr: vec![0],
            body_geom_num: vec![0],
            body_pos: vec![Vector3::zeros()],
            body_quat: vec![UnitQuaternion::identity()],
            body_ipos: vec![Vector3::zeros()],
            body_iquat: vec![UnitQuaternion::identity()],
            body_mass: vec![0.0],
            body_inertia: vec![Vector3::zeros()],
            body_name: vec![Some("world".to_string())],

            jnt_type: vec![],
            jnt_body: vec![],
            jnt_qpos_adr: vec![],
            jnt_dof_adr: vec![],
            jnt_pos: vec![],
            jnt_axis: vec![],
            jnt_limited: vec![],
            jnt_range: vec![],
            jnt_stiffness: vec![],
            jnt_damping: vec![],
            jnt_armature: vec![],
            jnt_name: vec![],

            dof_body: vec![],
            dof_jnt: vec![],
            dof_parent: vec![],
            dof_armature: vec![],
            dof_damping: vec![],

            geom_type: vec![],
            geom_body: vec![],
            geom_pos: vec![],
            geom_quat: vec![],
            geom_size: vec![],
            geom_friction: vec![],
            geom_contype: vec![],
            geom_conaffinity: vec![],
            geom_name: vec![],

            site_body: vec![],
            site_type: vec![],
            site_pos: vec![],
            site_quat: vec![],
            site_size: vec![],
            site_name: vec![],

            // World frame tracking (world body at origin)
            body_world_pos: vec![],
            body_world_quat: vec![],

            actuator_trntype: vec![],
            actuator_dyntype: vec![],
            actuator_trnid: vec![],
            actuator_gear: vec![],
            actuator_ctrlrange: vec![],
            actuator_forcerange: vec![],
            actuator_name: vec![],
            actuator_act_adr: vec![],
            actuator_act_num: vec![],

            na: 0,

            // MuJoCo defaults
            timestep: 0.002,
            gravity: Vector3::new(0.0, 0.0, -9.81),
            wind: Vector3::zeros(),
            magnetic: Vector3::zeros(),
            density: 0.0,
            viscosity: 0.0,
            solver_iterations: 100,
            solver_tolerance: 1e-8,
            impratio: 1.0,
            cone: 0,
            disableflags: 0,
            enableflags: 0,
            integrator: Integrator::Euler,

            qpos0_values: vec![],

            joint_name_to_id: std::collections::HashMap::new(),
            body_name_to_id: std::collections::HashMap::from([("world".to_string(), 0)]),
            site_name_to_id: std::collections::HashMap::new(),
        }
    }

    fn set_options(&mut self, option: &MjcfOption) {
        self.timestep = option.timestep;
        self.gravity = option.gravity;
        self.solver_iterations = option.iterations;
        self.solver_tolerance = option.tolerance;
        self.integrator = match option.integrator {
            MjcfIntegrator::Euler => Integrator::Euler,
            MjcfIntegrator::RK4 => Integrator::RungeKutta4,
            MjcfIntegrator::Implicit | MjcfIntegrator::ImplicitFast => Integrator::Implicit,
        };
    }

    /// Process geoms and sites directly attached to worldbody (body 0).
    ///
    /// In MJCF, the worldbody can have geoms (like ground planes) and sites
    /// directly attached to it. These are static geometries at world coordinates.
    fn process_worldbody_geoms_and_sites(
        &mut self,
        worldbody: &MjcfBody,
    ) -> std::result::Result<(), ModelConversionError> {
        // Track geom start address for body 0
        let geom_adr = self.geom_type.len();

        // Process worldbody geoms
        for geom in &worldbody.geoms {
            self.process_geom(geom, 0)?;
        }

        // Process worldbody sites
        for site in &worldbody.sites {
            self.process_site(site, 0)?;
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
    fn process_body(
        &mut self,
        body: &MjcfBody,
        parent_id: usize,
        parent_last_dof: Option<usize>,
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
    ) -> std::result::Result<usize, ModelConversionError> {
        let body_id = self.body_parent.len();

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

        // Body position/orientation relative to parent
        let body_pos = body.pos;
        let body_quat = quat_from_wxyz(body.quat);

        // Compute world frame position for this body (used for free joint qpos0)
        let world_pos = parent_world_pos + parent_world_quat * body_pos;
        let world_quat = parent_world_quat * body_quat;

        // Store world positions for child body processing
        self.body_world_pos.push(world_pos);
        self.body_world_quat.push(world_quat);

        // Process inertial properties with full MuJoCo semantics
        let (mass, inertia, ipos, iquat) = if let Some(ref inertial) = body.inertial {
            extract_inertial_properties(inertial)
        } else {
            // Compute from geoms if no explicit inertial
            compute_inertia_from_geoms(&body.geoms)
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
        self.body_name.push(if body.name.is_empty() {
            None
        } else {
            Some(body.name.clone())
        });

        // Process joints for this body, tracking the last DOF for kinematic tree linkage
        // MuJoCo semantics: first DOF of first joint links to parent body's last DOF,
        // subsequent DOFs form a chain within and across joints
        let mut body_nv = 0;
        let mut current_last_dof = parent_last_dof;

        for joint in &body.joints {
            let jnt_id =
                self.process_joint(joint, body_id, current_last_dof, world_pos, world_quat)?;
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

        // Process geoms for this body
        for geom in &body.geoms {
            self.process_geom(geom, body_id)?;
        }
        self.body_geom_num.push(body.geoms.len());

        // Process sites for this body
        for site in &body.sites {
            self.process_site(site, body_id)?;
        }

        // Recursively process children, passing this body's last DOF and world frame
        for child in &body.children {
            self.process_body_with_world_frame(
                child,
                body_id,
                current_last_dof,
                world_pos,
                world_quat,
            )?;
        }

        Ok(body_id)
    }

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
    fn process_joint(
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
        let jnt_type = match joint.joint_type {
            MjcfJointType::Hinge => MjJointType::Hinge,
            MjcfJointType::Slide => MjJointType::Slide,
            MjcfJointType::Ball => MjJointType::Ball,
            MjcfJointType::Free => MjJointType::Free,
            MjcfJointType::Cylindrical | MjcfJointType::Planar => {
                return Err(ModelConversionError {
                    message: format!(
                        "Joint type {:?} not yet supported in Model",
                        joint.joint_type
                    ),
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
        self.jnt_pos.push(joint.pos);
        // Normalize joint axis, handling zero vector edge case
        // MuJoCo defaults to Z-axis for unspecified/zero axes
        let axis = if joint.axis.norm() > 1e-10 {
            joint.axis.normalize()
        } else {
            warn!(
                joint_name = ?joint.name,
                "Joint axis is zero or near-zero, defaulting to Z-axis"
            );
            Vector3::z()
        };
        self.jnt_axis.push(axis);
        self.jnt_limited.push(joint.limited);
        self.jnt_range.push(
            joint
                .range
                .unwrap_or((-std::f64::consts::PI, std::f64::consts::PI)),
        );
        self.jnt_stiffness.push(joint.stiffness);
        self.jnt_damping.push(joint.damping);
        self.jnt_armature.push(joint.armature);
        self.jnt_name.push(if joint.name.is_empty() {
            None
        } else {
            Some(joint.name.clone())
        });

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

            self.dof_armature.push(joint.armature);
            self.dof_damping.push(joint.damping);
        }

        // Add qpos0 values (default positions)
        match jnt_type {
            MjJointType::Hinge | MjJointType::Slide => {
                self.qpos0_values.push(joint.ref_pos);
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

    fn process_geom(
        &mut self,
        geom: &MjcfGeom,
        body_id: usize,
    ) -> std::result::Result<usize, ModelConversionError> {
        let geom_id = self.geom_type.len();

        // Convert geom type
        let geom_type = match geom.geom_type {
            MjcfGeomType::Sphere => GeomType::Sphere,
            MjcfGeomType::Box => GeomType::Box,
            MjcfGeomType::Capsule => GeomType::Capsule,
            MjcfGeomType::Cylinder => GeomType::Cylinder,
            MjcfGeomType::Ellipsoid => GeomType::Ellipsoid,
            MjcfGeomType::Plane => GeomType::Plane,
            MjcfGeomType::Mesh | MjcfGeomType::TriangleMesh => GeomType::Mesh,
            MjcfGeomType::Hfield => {
                warn!(
                    geom_name = ?geom.name,
                    "Heightfield geom type not yet supported in Model, using Box as fallback"
                );
                GeomType::Box
            }
            MjcfGeomType::Sdf => {
                warn!(
                    geom_name = ?geom.name,
                    "SDF geom type not yet supported in Model, using Box as fallback"
                );
                GeomType::Box
            }
        };

        // Handle fromto for capsules/cylinders
        let (pos, quat, size) = if let Some(fromto) = geom.fromto {
            compute_fromto_pose(fromto, &geom.size)
        } else {
            (
                geom.pos,
                quat_from_wxyz(geom.quat),
                geom_size_to_vec3(&geom.size, geom_type),
            )
        };

        self.geom_type.push(geom_type);
        self.geom_body.push(body_id);
        self.geom_pos.push(pos);
        self.geom_quat.push(quat);
        self.geom_size.push(size);
        self.geom_friction.push(geom.friction);
        #[allow(clippy::cast_sign_loss)]
        {
            self.geom_contype.push(geom.contype as u32);
            self.geom_conaffinity.push(geom.conaffinity as u32);
        }
        self.geom_name.push(geom.name.clone());

        Ok(geom_id)
    }

    fn process_site(
        &mut self,
        site: &MjcfSite,
        body_id: usize,
    ) -> std::result::Result<usize, ModelConversionError> {
        let site_id = self.site_body.len();

        // Store name mapping for actuator site transmission
        if !site.name.is_empty() {
            self.site_name_to_id.insert(site.name.clone(), site_id);
        }

        self.site_body.push(body_id);

        // Convert site type string to GeomType
        let geom_type = match site.site_type.as_str() {
            "sphere" => GeomType::Sphere,
            "capsule" => GeomType::Capsule,
            "cylinder" => GeomType::Cylinder,
            "box" => GeomType::Box,
            "ellipsoid" => GeomType::Ellipsoid,
            _ => GeomType::Sphere, // Default fallback
        };
        self.site_type.push(geom_type);

        self.site_pos.push(site.pos);
        self.site_quat.push(quat_from_wxyz(site.quat));

        // Convert site size (MuJoCo uses single value for sphere, array for others)
        let size = if site.size.is_empty() {
            Vector3::new(0.01, 0.01, 0.01) // Default small size
        } else if site.size.len() == 1 {
            let s = site.size[0];
            Vector3::new(s, s, s)
        } else {
            Vector3::new(
                site.size[0],
                site.size.get(1).copied().unwrap_or(site.size[0]),
                site.size.get(2).copied().unwrap_or(site.size[0]),
            )
        };
        self.site_size.push(size);

        self.site_name.push(if site.name.is_empty() {
            None
        } else {
            Some(site.name.clone())
        });

        Ok(site_id)
    }

    fn process_actuator(
        &mut self,
        actuator: &MjcfActuator,
    ) -> std::result::Result<usize, ModelConversionError> {
        let act_id = self.actuator_trntype.len();

        // Determine transmission type and target
        // MuJoCo supports Joint, Tendon, Site, Body transmissions
        // We currently only support Joint; others require site/tendon arrays in Model
        let (trntype, trnid) = if let Some(ref joint_name) = actuator.joint {
            let jnt_id =
                self.joint_name_to_id
                    .get(joint_name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!("Actuator references unknown joint: {joint_name}"),
                    })?;
            (ActuatorTransmission::Joint, *jnt_id)
        } else if let Some(ref tendon_name) = actuator.tendon {
            // Tendon transmission not yet implemented in Model
            // Requires tendon arrays (ntendon, tendon_*, etc.)
            return Err(ModelConversionError {
                message: format!(
                    "Actuator '{}' uses tendon transmission '{}' which is not yet supported. \
                     Use joint transmission instead.",
                    actuator.name, tendon_name
                ),
            });
        } else if let Some(ref site_name) = actuator.site {
            let site_id =
                self.site_name_to_id
                    .get(site_name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!("Actuator references unknown site: {site_name}"),
                    })?;
            (ActuatorTransmission::Site, *site_id)
        } else if let Some(ref body_name) = actuator.body {
            // Body transmission (adhesion actuators) not yet implemented
            return Err(ModelConversionError {
                message: format!(
                    "Actuator '{}' uses body transmission '{}' which is not yet supported.",
                    actuator.name, body_name
                ),
            });
        } else {
            return Err(ModelConversionError {
                message: format!(
                    "Actuator '{}' has no transmission target (joint, tendon, site, or body)",
                    actuator.name
                ),
            });
        };

        // Determine dynamics type
        // MuJoCo semantics:
        // - Motor/General/Damper/Adhesion: Direct force, no activation state
        // - Position/Velocity: First-order filter dynamics
        // - Muscle: Muscle activation dynamics (2 states)
        // - Cylinder: Integrator dynamics (pneumatic)
        let dyntype = match actuator.actuator_type {
            MjcfActuatorType::Motor
            | MjcfActuatorType::General
            | MjcfActuatorType::Damper
            | MjcfActuatorType::Adhesion => ActuatorDynamics::None,
            MjcfActuatorType::Position | MjcfActuatorType::Velocity => ActuatorDynamics::Filter,
            MjcfActuatorType::Muscle => ActuatorDynamics::Muscle,
            MjcfActuatorType::Cylinder => ActuatorDynamics::Integrator,
        };

        self.actuator_trntype.push(trntype);
        self.actuator_dyntype.push(dyntype);
        self.actuator_trnid.push(trnid);
        self.actuator_gear.push(actuator.gear);
        self.actuator_ctrlrange
            .push(actuator.ctrlrange.unwrap_or((-1.0, 1.0)));
        self.actuator_forcerange.push(
            actuator
                .forcerange
                .unwrap_or((f64::NEG_INFINITY, f64::INFINITY)),
        );
        self.actuator_name.push(if actuator.name.is_empty() {
            None
        } else {
            Some(actuator.name.clone())
        });

        // Compute activation state count based on dynamics type
        // MuJoCo semantics: None=0, Filter/Integrator=1, Muscle=2
        let act_num = match dyntype {
            ActuatorDynamics::None => 0,
            ActuatorDynamics::Filter | ActuatorDynamics::Integrator => 1,
            ActuatorDynamics::Muscle => 2,
        };

        self.actuator_act_adr.push(self.na);
        self.actuator_act_num.push(act_num);
        self.na += act_num;

        Ok(act_id)
    }

    fn build(self) -> Model {
        let njnt = self.jnt_type.len();
        let nbody = self.body_parent.len();
        let ngeom = self.geom_type.len();
        let nsite = self.site_body.len();
        let nu = self.actuator_trntype.len();

        let mut model = Model {
            nq: self.nq,
            nv: self.nv,
            nbody,
            njnt,
            ngeom,
            nsite,
            nu,
            na: self.na,

            body_parent: self.body_parent,
            body_rootid: self.body_rootid,
            body_jnt_adr: self.body_jnt_adr,
            body_jnt_num: self.body_jnt_num,
            body_dof_adr: self.body_dof_adr,
            body_dof_num: self.body_dof_num,
            body_geom_adr: self.body_geom_adr,
            body_geom_num: self.body_geom_num,
            body_pos: self.body_pos,
            body_quat: self.body_quat,
            body_ipos: self.body_ipos,
            body_iquat: self.body_iquat,
            body_mass: self.body_mass,
            body_inertia: self.body_inertia,
            body_name: self.body_name,
            body_subtreemass: vec![0.0; nbody], // Computed after model construction

            jnt_type: self.jnt_type,
            jnt_body: self.jnt_body,
            jnt_qpos_adr: self.jnt_qpos_adr,
            jnt_dof_adr: self.jnt_dof_adr,
            jnt_pos: self.jnt_pos,
            jnt_axis: self.jnt_axis,
            jnt_limited: self.jnt_limited,
            jnt_range: self.jnt_range,
            jnt_stiffness: self.jnt_stiffness,
            jnt_damping: self.jnt_damping,
            jnt_armature: self.jnt_armature,
            jnt_name: self.jnt_name,

            dof_body: self.dof_body,
            dof_jnt: self.dof_jnt,
            dof_parent: self.dof_parent,
            dof_armature: self.dof_armature,
            dof_damping: self.dof_damping,
            dof_frictionloss: vec![0.0; self.nv], // Default: no friction loss

            geom_type: self.geom_type,
            geom_body: self.geom_body,
            geom_pos: self.geom_pos,
            geom_quat: self.geom_quat,
            geom_size: self.geom_size,
            geom_friction: self.geom_friction,
            geom_contype: self.geom_contype,
            geom_conaffinity: self.geom_conaffinity,
            geom_margin: vec![0.0; ngeom], // Default margin
            geom_gap: vec![0.0; ngeom],    // Default gap
            geom_solimp: vec![[0.9, 0.95, 0.001, 0.5, 2.0]; ngeom], // MuJoCo defaults
            geom_solref: vec![[0.02, 1.0]; ngeom], // MuJoCo defaults
            geom_name: self.geom_name,

            site_body: self.site_body,
            site_type: self.site_type,
            site_pos: self.site_pos,
            site_quat: self.site_quat,
            site_size: self.site_size,
            site_name: self.site_name,

            // Sensors (empty for now - will be populated from MJCF sensor definitions)
            nsensor: 0,
            nsensordata: 0,
            sensor_type: vec![],
            sensor_datatype: vec![],
            sensor_objtype: vec![],
            sensor_objid: vec![],
            sensor_reftype: vec![],
            sensor_refid: vec![],
            sensor_adr: vec![],
            sensor_dim: vec![],
            sensor_noise: vec![],
            sensor_cutoff: vec![],
            sensor_name: vec![],

            actuator_trntype: self.actuator_trntype,
            actuator_dyntype: self.actuator_dyntype,
            actuator_trnid: self.actuator_trnid,
            actuator_gear: self.actuator_gear,
            actuator_ctrlrange: self.actuator_ctrlrange,
            actuator_forcerange: self.actuator_forcerange,
            actuator_name: self.actuator_name,
            actuator_act_adr: self.actuator_act_adr,
            actuator_act_num: self.actuator_act_num,

            // Tendons (empty for now - will be populated from MJCF tendon definitions)
            ntendon: 0,
            nwrap: 0,
            tendon_range: vec![],
            tendon_limited: vec![],
            tendon_stiffness: vec![],
            tendon_damping: vec![],
            tendon_lengthspring: vec![],
            tendon_length0: vec![],
            tendon_num: vec![],
            tendon_adr: vec![],
            tendon_name: vec![],
            wrap_type: vec![],
            wrap_objid: vec![],
            wrap_prm: vec![],

            // Equality constraints (empty for now - will be populated from MJCF equality definitions)
            neq: 0,
            eq_type: vec![],
            eq_obj1id: vec![],
            eq_obj2id: vec![],
            eq_data: vec![],
            eq_active: vec![],
            eq_solimp: vec![],
            eq_solref: vec![],
            eq_name: vec![],

            timestep: self.timestep,
            gravity: self.gravity,
            qpos0: DVector::from_vec(self.qpos0_values),
            wind: self.wind,
            magnetic: self.magnetic,
            density: self.density,
            viscosity: self.viscosity,
            solver_iterations: self.solver_iterations,
            solver_tolerance: self.solver_tolerance,
            impratio: self.impratio,
            cone: self.cone,
            disableflags: self.disableflags,
            enableflags: self.enableflags,
            integrator: self.integrator,

            // Pre-computed kinematic data (will be populated by compute_ancestors)
            body_ancestor_joints: vec![vec![]; nbody],
            body_ancestor_mask: vec![vec![]; nbody], // Multi-word bitmask, computed by compute_ancestors
        };

        // Pre-compute ancestor lists for O(n) CRBA/RNE
        model.compute_ancestors();
        model
    }
}

/// Convert MJCF quaternion (w, x, y, z) to `UnitQuaternion`.
fn quat_from_wxyz(q: nalgebra::Vector4<f64>) -> UnitQuaternion<f64> {
    UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(q[0], q[1], q[2], q[3]))
}

/// Extract inertial properties from MjcfInertial with full MuJoCo semantics.
///
/// Handles both `diaginertia` and `fullinertia` specifications.
/// When `fullinertia` is specified, diagonalizes via eigendecomposition
/// and returns the principal axis orientation in `iquat`.
fn extract_inertial_properties(
    inertial: &MjcfInertial,
) -> (f64, Vector3<f64>, Vector3<f64>, UnitQuaternion<f64>) {
    let mass = inertial.mass;
    let ipos = inertial.pos;

    // Priority: fullinertia > diaginertia > default
    if let Some(full) = inertial.fullinertia {
        // Full inertia tensor [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
        // Build symmetric matrix and diagonalize
        let inertia_matrix = Matrix3::new(
            full[0], full[3], full[4], // Ixx, Ixy, Ixz
            full[3], full[1], full[5], // Ixy, Iyy, Iyz
            full[4], full[5], full[2], // Ixz, Iyz, Izz
        );

        // Eigendecomposition to get principal axes
        let eigen = inertia_matrix.symmetric_eigen();
        let principal_inertia = Vector3::new(
            eigen.eigenvalues[0].abs(),
            eigen.eigenvalues[1].abs(),
            eigen.eigenvalues[2].abs(),
        );

        // Eigenvectors form rotation matrix to principal axes
        // Ensure right-handed coordinate system
        let mut rot = eigen.eigenvectors;
        if rot.determinant() < 0.0 {
            // Flip one column to make it right-handed
            rot.set_column(2, &(-rot.column(2)));
        }

        let iquat = UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix(&rot));

        // Combine with any existing inertial frame orientation
        let base_iquat = quat_from_wxyz(inertial.quat);
        let final_iquat = base_iquat * iquat;

        (mass, principal_inertia, ipos, final_iquat)
    } else if let Some(diag) = inertial.diaginertia {
        // Diagonal inertia already in principal axes
        (mass, diag, ipos, quat_from_wxyz(inertial.quat))
    } else {
        // Default: small uniform inertia
        (
            mass,
            Vector3::new(0.001, 0.001, 0.001),
            ipos,
            quat_from_wxyz(inertial.quat),
        )
    }
}

/// Compute inertia from geoms (fallback when no explicit inertial).
fn compute_inertia_from_geoms(
    geoms: &[MjcfGeom],
) -> (f64, Vector3<f64>, Vector3<f64>, UnitQuaternion<f64>) {
    if geoms.is_empty() {
        // No geoms: use small default mass
        return (
            0.001,
            Vector3::new(0.001, 0.001, 0.001),
            Vector3::zeros(),
            UnitQuaternion::identity(),
        );
    }

    let mut total_mass = 0.0;
    let mut com = Vector3::zeros();

    // First pass: compute total mass and COM
    for geom in geoms {
        let geom_mass = compute_geom_mass(geom);
        total_mass += geom_mass;
        com += geom.pos * geom_mass;
    }

    if total_mass > 1e-10 {
        com /= total_mass;
    }

    // Second pass: compute inertia about COM using parallel axis theorem
    let mut inertia = Vector3::zeros();
    for geom in geoms {
        let geom_mass = compute_geom_mass(geom);
        let geom_inertia = compute_geom_inertia(geom);

        // Parallel axis: I_com = I_geom + m * d^2
        let d = geom.pos - com;
        let d_sq = d.component_mul(&d);

        // For diagonal inertia, add off-axis contributions
        inertia.x += geom_inertia.x + geom_mass * (d_sq.y + d_sq.z);
        inertia.y += geom_inertia.y + geom_mass * (d_sq.x + d_sq.z);
        inertia.z += geom_inertia.z + geom_mass * (d_sq.x + d_sq.y);
    }

    (total_mass, inertia, com, UnitQuaternion::identity())
}

/// Compute mass of a single geom.
fn compute_geom_mass(geom: &MjcfGeom) -> f64 {
    if let Some(mass) = geom.mass {
        return mass;
    }

    let volume = match geom.geom_type {
        MjcfGeomType::Sphere => {
            let r = geom.size.first().copied().unwrap_or(0.1);
            (4.0 / 3.0) * std::f64::consts::PI * r.powi(3)
        }
        MjcfGeomType::Box => {
            let x = geom.size.first().copied().unwrap_or(0.1);
            let y = geom.size.get(1).copied().unwrap_or(x);
            let z = geom.size.get(2).copied().unwrap_or(y);
            8.0 * x * y * z // size is half-extents
        }
        MjcfGeomType::Capsule => {
            let r = geom.size.first().copied().unwrap_or(0.1);
            let h = geom.size.get(1).copied().unwrap_or(0.1);
            std::f64::consts::PI * r.powi(2) * (h * 2.0 + (4.0 / 3.0) * r)
        }
        MjcfGeomType::Cylinder => {
            let r = geom.size.first().copied().unwrap_or(0.1);
            let h = geom.size.get(1).copied().unwrap_or(0.1);
            std::f64::consts::PI * r.powi(2) * h * 2.0
        }
        _ => 0.001, // Default small volume for other types
    };

    geom.density * volume
}

/// Compute diagonal inertia of a single geom about its center.
///
/// Uses exact formulas matching MuJoCo's computation:
/// - Sphere: I = (2/5) m r²
/// - Box: I_x = (1/12) m (y² + z²), etc.
/// - Cylinder: I_x = (1/12) m (3r² + h²), I_z = (1/2) m r²
/// - Capsule: Exact formula including hemispherical end caps
fn compute_geom_inertia(geom: &MjcfGeom) -> Vector3<f64> {
    let mass = compute_geom_mass(geom);

    match geom.geom_type {
        MjcfGeomType::Sphere => {
            let r = geom.size.first().copied().unwrap_or(0.1);
            let i = 0.4 * mass * r.powi(2); // (2/5) m r²
            Vector3::new(i, i, i)
        }
        MjcfGeomType::Box => {
            // Full dimensions (size is half-extents)
            let x = geom.size.first().copied().unwrap_or(0.1) * 2.0;
            let y = geom.size.get(1).copied().unwrap_or(0.1) * 2.0;
            let z = geom.size.get(2).copied().unwrap_or(0.1) * 2.0;
            let c = mass / 12.0;
            Vector3::new(
                c * (y * y + z * z),
                c * (x * x + z * z),
                c * (x * x + y * y),
            )
        }
        MjcfGeomType::Cylinder => {
            let r = geom.size.first().copied().unwrap_or(0.1);
            let h = geom.size.get(1).copied().unwrap_or(0.1) * 2.0; // Full height
            // Solid cylinder about center
            let ix = mass * (3.0 * r.powi(2) + h.powi(2)) / 12.0;
            let iz = 0.5 * mass * r.powi(2);
            Vector3::new(ix, ix, iz)
        }
        MjcfGeomType::Capsule => {
            // Exact capsule inertia (cylinder + two hemispheres)
            // Reference: https://www.gamedev.net/articles/programming/math-and-physics/capsule-inertia-tensor-r3856/
            let r = geom.size.first().copied().unwrap_or(0.1);
            let h = geom.size.get(1).copied().unwrap_or(0.1) * 2.0; // Cylinder height (not including caps)

            // Volume components
            let v_cyl = std::f64::consts::PI * r.powi(2) * h;
            let v_sphere = (4.0 / 3.0) * std::f64::consts::PI * r.powi(3);
            let v_total = v_cyl + v_sphere;

            // Mass components (assuming uniform density)
            let m_cyl = mass * v_cyl / v_total;
            let m_hemi = mass * v_sphere / (2.0 * v_total); // Each hemisphere

            // Cylinder inertia about its own center
            let i_cyl_x = m_cyl * (3.0 * r.powi(2) + h.powi(2)) / 12.0;
            let i_cyl_z = 0.5 * m_cyl * r.powi(2);

            // Hemisphere inertia about its own center (half of sphere)
            let i_hemi_own = 0.4 * m_hemi * r.powi(2);

            // Distance from capsule center to hemisphere center
            let d = h / 2.0 + (3.0 / 8.0) * r; // Center of hemisphere from cylinder end

            // Parallel axis theorem for hemispheres
            let i_hemi_x = i_hemi_own + m_hemi * d.powi(2);
            let i_hemi_z = i_hemi_own; // No parallel axis for z (axial)

            // Total: cylinder + 2 hemispheres
            let ix = i_cyl_x + 2.0 * i_hemi_x;
            let iz = i_cyl_z + 2.0 * i_hemi_z;

            Vector3::new(ix, ix, iz)
        }
        MjcfGeomType::Ellipsoid => {
            // Ellipsoid inertia: I_x = (1/5) m (b² + c²), etc.
            let a = geom.size.first().copied().unwrap_or(0.1);
            let b = geom.size.get(1).copied().unwrap_or(a);
            let c = geom.size.get(2).copied().unwrap_or(b);
            let coeff = mass / 5.0;
            Vector3::new(
                coeff * (b * b + c * c),
                coeff * (a * a + c * c),
                coeff * (a * a + b * b),
            )
        }
        _ => Vector3::new(0.001, 0.001, 0.001), // Default small inertia
    }
}

/// Compute pose from fromto specification.
fn compute_fromto_pose(
    fromto: [f64; 6],
    size: &[f64],
) -> (Vector3<f64>, UnitQuaternion<f64>, Vector3<f64>) {
    let from = Vector3::new(fromto[0], fromto[1], fromto[2]);
    let to = Vector3::new(fromto[3], fromto[4], fromto[5]);

    let center = (from + to) / 2.0;
    let axis = to - from;
    let half_length = axis.norm() / 2.0;
    let radius = size.first().copied().unwrap_or(0.05);

    // Compute rotation to align Z with axis
    let quat = if axis.norm() > 1e-10 {
        let axis_normalized = axis.normalize();
        let z_axis = Vector3::z();

        if (axis_normalized - z_axis).norm() < 1e-10 {
            UnitQuaternion::identity()
        } else if (axis_normalized + z_axis).norm() < 1e-10 {
            UnitQuaternion::from_axis_angle(
                &nalgebra::Unit::new_normalize(Vector3::x()),
                std::f64::consts::PI,
            )
        } else {
            let rot_axis = z_axis.cross(&axis_normalized);
            // Clamp to avoid NaN from floating-point precision issues
            let angle = z_axis.dot(&axis_normalized).clamp(-1.0, 1.0).acos();
            UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(rot_axis), angle)
        }
    } else {
        UnitQuaternion::identity()
    };

    (center, quat, Vector3::new(radius, radius, half_length))
}

/// Convert geom size array to `Vector3`.
fn geom_size_to_vec3(size: &[f64], geom_type: GeomType) -> Vector3<f64> {
    match geom_type {
        GeomType::Sphere => {
            let r = size.first().copied().unwrap_or(0.1);
            Vector3::new(r, r, r)
        }
        GeomType::Box => {
            let x = size.first().copied().unwrap_or(0.1);
            let y = size.get(1).copied().unwrap_or(x);
            let z = size.get(2).copied().unwrap_or(y);
            Vector3::new(x, y, z)
        }
        GeomType::Capsule | GeomType::Cylinder => {
            let r = size.first().copied().unwrap_or(0.1);
            let h = size.get(1).copied().unwrap_or(0.1);
            Vector3::new(r, r, h)
        }
        _ => Vector3::new(0.1, 0.1, 0.1),
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn test_simple_pendulum() {
        let model = load_model(
            r#"
            <mujoco model="pendulum">
                <option timestep="0.001" gravity="0 0 -9.81"/>
                <worldbody>
                    <body name="link1" pos="0 0 1">
                        <joint name="j1" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.1 0.1 0.01"/>
                        <geom type="capsule" size="0.05 0.5"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nbody, 2); // world + link1
        assert_eq!(model.njnt, 1);
        assert_eq!(model.nq, 1); // hinge has 1 qpos
        assert_eq!(model.nv, 1); // hinge has 1 DOF
        assert_eq!(model.ngeom, 1);

        // Check body tree
        assert_eq!(model.body_parent[1], 0); // link1's parent is world

        // Check joint
        assert_eq!(model.jnt_type[0], MjJointType::Hinge);
        assert_eq!(model.jnt_body[0], 1); // joint on link1

        // Check options
        assert!((model.timestep - 0.001).abs() < 1e-10);
        assert!((model.gravity.z - (-9.81)).abs() < 1e-10);
    }

    #[test]
    fn test_double_pendulum() {
        let model = load_model(
            r#"
            <mujoco model="double_pendulum">
                <worldbody>
                    <body name="link1" pos="0 0 1">
                        <joint name="j1" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.1 0.1 0.01"/>
                        <geom type="capsule" size="0.05 0.25"/>
                        <body name="link2" pos="0 0 -0.5">
                            <joint name="j2" type="hinge" axis="0 1 0"/>
                            <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.1 0.1 0.01"/>
                            <geom type="capsule" size="0.05 0.25"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nbody, 3); // world + link1 + link2
        assert_eq!(model.njnt, 2);
        assert_eq!(model.nq, 2);
        assert_eq!(model.nv, 2);

        // Check body tree
        assert_eq!(model.body_parent[1], 0); // link1's parent is world
        assert_eq!(model.body_parent[2], 1); // link2's parent is link1
    }

    #[test]
    fn test_with_actuator() {
        let model = load_model(
            r#"
            <mujoco model="actuated">
                <worldbody>
                    <body name="arm" pos="0 0 1">
                        <joint name="shoulder" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <motor name="motor1" joint="shoulder" gear="100"/>
                </actuator>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nu, 1);
        assert_eq!(model.actuator_trntype[0], ActuatorTransmission::Joint);
        assert_eq!(model.actuator_trnid[0], 0); // First joint
        assert!((model.actuator_gear[0] - 100.0).abs() < 1e-10);
    }

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
    /// For a 2-link pendulum (world → link1 → link2), each with 1 hinge:
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
    /// world → link1 (ball, 3 DOF) → link2 (hinge, 1 DOF)
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
    /// world → base (hinge) ─┬→ left_arm (hinge)
    ///                       └→ right_arm (hinge)
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
    /// world → link (hinge1, hinge2)
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

    /// Test that tendon actuators return proper error.
    #[test]
    fn test_tendon_actuator_error() {
        let result = load_model(
            r#"
            <mujoco model="tendon_test">
                <worldbody>
                    <body name="link" pos="0 0 1">
                        <joint name="j1" type="hinge"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <motor name="m1" tendon="some_tendon" gear="1"/>
                </actuator>
            </mujoco>
        "#,
        );

        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(err.to_string().contains("tendon transmission"));
    }

    /// Test capsule inertia computation is physically reasonable.
    #[test]
    fn test_capsule_inertia() {
        // A capsule should have I_z < I_x = I_y (axially symmetric, longer than wide)
        let geom = MjcfGeom {
            name: None,
            class: None,
            geom_type: MjcfGeomType::Capsule,
            pos: Vector3::zeros(),
            quat: nalgebra::Vector4::new(1.0, 0.0, 0.0, 0.0),
            size: vec![0.1, 0.5], // radius=0.1, half-height=0.5
            fromto: None,
            density: 1000.0,
            mass: None,
            friction: Vector3::new(1.0, 0.005, 0.0001),
            rgba: nalgebra::Vector4::new(0.5, 0.5, 0.5, 1.0),
            contype: 1,
            conaffinity: 1,
            condim: 3,
            mesh: None,
        };

        let inertia = compute_geom_inertia(&geom);

        // Ix = Iy (axially symmetric)
        assert!((inertia.x - inertia.y).abs() < 1e-10);

        // Iz < Ix (thin cylinder is easier to spin about long axis)
        assert!(inertia.z < inertia.x);

        // All positive
        assert!(inertia.x > 0.0);
        assert!(inertia.y > 0.0);
        assert!(inertia.z > 0.0);
    }

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

    /// Test site actuator transmission.
    #[test]
    fn test_site_actuator() {
        let model = load_model(
            r#"
            <mujoco model="site_actuator_test">
                <worldbody>
                    <body name="arm" pos="0 0 1">
                        <joint name="j1" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <site name="end_effector" pos="0 0 -1"/>
                    </body>
                </worldbody>
                <actuator>
                    <general name="site_act" site="end_effector" gear="50"/>
                </actuator>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nu, 1);
        assert_eq!(model.actuator_trntype[0], ActuatorTransmission::Site);
        assert_eq!(model.actuator_trnid[0], 0); // First site
        assert!((model.actuator_gear[0] - 50.0).abs() < 1e-10);
    }

    /// Test actuator activation states (na) computation.
    #[test]
    fn test_actuator_activation_states() {
        let model = load_model(
            r#"
            <mujoco model="activation_test">
                <worldbody>
                    <body name="arm" pos="0 0 1">
                        <joint name="j1" type="hinge" axis="0 1 0"/>
                        <joint name="j2" type="hinge" axis="1 0 0"/>
                        <joint name="j3" type="hinge" axis="0 0 1"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <motor name="motor1" joint="j1" gear="10"/>
                    <position name="pos1" joint="j2" kp="100"/>
                    <velocity name="vel1" joint="j3" kv="10"/>
                </actuator>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nu, 3);

        // Motor has no dynamics -> 0 activation states
        assert_eq!(model.actuator_dyntype[0], ActuatorDynamics::None);
        assert_eq!(model.actuator_act_adr[0], 0);
        assert_eq!(model.actuator_act_num[0], 0);

        // Position servo has filter dynamics -> 1 activation state
        assert_eq!(model.actuator_dyntype[1], ActuatorDynamics::Filter);
        assert_eq!(model.actuator_act_adr[1], 0);
        assert_eq!(model.actuator_act_num[1], 1);

        // Velocity servo has filter dynamics -> 1 activation state
        assert_eq!(model.actuator_dyntype[2], ActuatorDynamics::Filter);
        assert_eq!(model.actuator_act_adr[2], 1); // Starts after position servo's 1 state
        assert_eq!(model.actuator_act_num[2], 1);

        // Total activation states
        assert_eq!(model.na, 2); // 0 + 1 + 1 = 2
    }
}
