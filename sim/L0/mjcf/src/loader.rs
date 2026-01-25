//! MJCF to sim-core type conversion.
//!
//! Converts parsed MJCF intermediate representation into simulation-ready types.

use std::collections::HashMap;
use std::fs;
use std::path::Path;

use nalgebra::{Point3, Vector3};
use sim_constraint::{
    AdhesionActuator, BoxedActuator, ConnectConstraint, IntegratedVelocityActuator,
    IntoBoxedActuator, JointLimits, PneumaticCylinderActuator, TendonConstraint,
};
use sim_core::{Body, CollisionShape, Joint, World};
use sim_tendon::{
    CableProperties, SpatialTendon,
    path::{AttachmentPoint, TendonPath},
    spatial::WrappingGeometryRef,
    wrapping::{CylinderWrap, SphereWrap, WrappingGeometry},
};
use sim_types::{BodyId, JointId, JointType, MassProperties, Pose, RigidBodyState};

use crate::config::ExtendedSolverConfig;
use crate::defaults::DefaultResolver;
use crate::error::{MjcfError, Result};
use crate::parser::parse_mjcf_str;
use crate::types::{
    MjcfActuator, MjcfActuatorType, MjcfBody, MjcfGeom, MjcfGeomType, MjcfJoint, MjcfJointType,
    MjcfMesh, MjcfModel, MjcfOption, MjcfTendon, MjcfTendonType,
};
use crate::validation::{ValidationResult, validate};

/// A loaded actuator with metadata for simulation integration.
pub struct LoadedActuator {
    /// Actuator name.
    pub name: String,
    /// The boxed actuator implementation.
    pub actuator: BoxedActuator,
    /// Target joint name (if joint-based actuator).
    pub joint: Option<String>,
    /// Target body name (if body-based actuator like adhesion).
    pub body: Option<String>,
    /// Target tendon name (if tendon-based actuator).
    pub tendon: Option<String>,
    /// Control range (if clamped).
    pub ctrlrange: Option<(f64, f64)>,
    /// Force range (if clamped).
    pub forcerange: Option<(f64, f64)>,
    /// Gear ratio.
    pub gear: f64,
}

impl std::fmt::Debug for LoadedActuator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LoadedActuator")
            .field("name", &self.name)
            .field("joint", &self.joint)
            .field("body", &self.body)
            .field("tendon", &self.tendon)
            .field("ctrlrange", &self.ctrlrange)
            .field("forcerange", &self.forcerange)
            .field("gear", &self.gear)
            .finish_non_exhaustive()
    }
}

/// Information about a site attachment point.
///
/// Sites are attachment points on bodies used for spatial tendons,
/// sensors, and other features that need to track positions on bodies.
#[derive(Debug, Clone)]
pub struct SiteInfo {
    /// The body this site is attached to.
    pub body_id: BodyId,
    /// Position in the body's local frame.
    pub local_position: Point3<f64>,
}

/// Information about a geometry that can be used for tendon wrapping.
///
/// Geoms can serve as wrapping surfaces for spatial tendons, allowing
/// tendons to route around obstacles like bones, pulleys, or joints.
#[derive(Debug, Clone)]
pub struct GeomInfo {
    /// The body this geom is attached to.
    pub body_id: BodyId,
    /// Position in the body's local frame.
    pub local_position: Point3<f64>,
    /// Orientation in the body's local frame (quaternion: w, x, y, z).
    pub local_orientation: nalgebra::UnitQuaternion<f64>,
    /// The wrapping geometry (sphere or cylinder).
    pub wrapping_geometry: Option<WrappingGeometry>,
}

/// A loaded fixed tendon from MJCF.
///
/// Fixed tendons define length as a linear combination of joint positions:
/// `L = L_rest + Σᵢ coef_i * q_i`
#[derive(Debug, Clone)]
pub struct LoadedFixedTendon {
    /// Tendon name.
    pub name: String,
    /// The underlying tendon constraint.
    pub constraint: TendonConstraint,
    /// Whether the tendon has length limits.
    pub limited: bool,
    /// Length range limits (if limited).
    pub range: Option<(f64, f64)>,
    /// Tendon width for visualization.
    pub width: f64,
    /// RGBA color for visualization.
    pub rgba: [f64; 4],
}

/// A loaded spatial tendon from MJCF.
///
/// Spatial tendons route through 3D attachment points (sites) on bodies.
/// Length is computed from the actual 3D path through space.
#[derive(Debug, Clone)]
pub struct LoadedSpatialTendon {
    /// Tendon name.
    pub name: String,
    /// The underlying spatial tendon from sim-tendon.
    pub tendon: SpatialTendon,
    /// Whether the tendon has length limits.
    pub limited: bool,
    /// Length range limits (if limited).
    pub range: Option<(f64, f64)>,
    /// Tendon width for visualization.
    pub width: f64,
    /// RGBA color for visualization.
    pub rgba: [f64; 4],
}

/// A loaded tendon from MJCF.
///
/// Tendons can be either fixed (joint-based coupling) or spatial (3D path through sites).
// Allow large enum variant: Spatial tendons contain more data than fixed tendons by design.
// Boxing would add indirection and lifetime complexity without significant memory benefit
// since tendons are typically few in number and long-lived.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone)]
pub enum LoadedTendon {
    /// Fixed tendon: linear combination of joint positions.
    Fixed(LoadedFixedTendon),
    /// Spatial tendon: 3D path through attachment sites.
    Spatial(LoadedSpatialTendon),
}

impl LoadedTendon {
    /// Get the tendon name.
    #[must_use]
    pub fn name(&self) -> &str {
        match self {
            Self::Fixed(t) => &t.name,
            Self::Spatial(t) => &t.name,
        }
    }

    /// Check if this is a fixed tendon.
    #[must_use]
    pub fn is_fixed(&self) -> bool {
        matches!(self, Self::Fixed(_))
    }

    /// Check if this is a spatial tendon.
    #[must_use]
    pub fn is_spatial(&self) -> bool {
        matches!(self, Self::Spatial(_))
    }

    /// Get as a fixed tendon, if it is one.
    #[must_use]
    pub fn as_fixed(&self) -> Option<&LoadedFixedTendon> {
        match self {
            Self::Fixed(t) => Some(t),
            Self::Spatial(_) => None,
        }
    }

    /// Get as a mutable fixed tendon, if it is one.
    #[must_use]
    pub fn as_fixed_mut(&mut self) -> Option<&mut LoadedFixedTendon> {
        match self {
            Self::Fixed(t) => Some(t),
            Self::Spatial(_) => None,
        }
    }

    /// Get as a spatial tendon, if it is one.
    #[must_use]
    pub fn as_spatial(&self) -> Option<&LoadedSpatialTendon> {
        match self {
            Self::Fixed(_) => None,
            Self::Spatial(t) => Some(t),
        }
    }

    /// Get as a mutable spatial tendon, if it is one.
    #[must_use]
    pub fn as_spatial_mut(&mut self) -> Option<&mut LoadedSpatialTendon> {
        match self {
            Self::Fixed(_) => None,
            Self::Spatial(t) => Some(t),
        }
    }

    /// Whether the tendon has length limits.
    #[must_use]
    pub fn limited(&self) -> bool {
        match self {
            Self::Fixed(t) => t.limited,
            Self::Spatial(t) => t.limited,
        }
    }

    /// Length range limits (if limited).
    #[must_use]
    pub fn range(&self) -> Option<(f64, f64)> {
        match self {
            Self::Fixed(t) => t.range,
            Self::Spatial(t) => t.range,
        }
    }

    /// Tendon width for visualization.
    #[must_use]
    pub fn width(&self) -> f64 {
        match self {
            Self::Fixed(t) => t.width,
            Self::Spatial(t) => t.width,
        }
    }

    /// RGBA color for visualization.
    #[must_use]
    pub fn rgba(&self) -> [f64; 4] {
        match self {
            Self::Fixed(t) => t.rgba,
            Self::Spatial(t) => t.rgba,
        }
    }
}

/// A loaded model ready to be spawned into a simulation world.
#[derive(Debug)]
pub struct LoadedModel {
    /// Model name.
    pub name: String,
    /// Bodies ready for insertion.
    pub bodies: Vec<Body>,
    /// Joints ready for insertion.
    pub joints: Vec<Joint>,
    /// Connect (ball) equality constraints.
    pub connect_constraints: Vec<ConnectConstraint>,
    /// Tendon constraints.
    pub tendons: Vec<LoadedTendon>,
    /// Actuators ready for use.
    pub actuators: Vec<LoadedActuator>,
    /// Map from body name to body ID.
    pub body_to_id: HashMap<String, BodyId>,
    /// Map from joint name to joint ID.
    pub joint_to_id: HashMap<String, JointId>,
    /// Map from tendon name to index in tendons vec.
    pub tendon_to_index: HashMap<String, usize>,
    /// Map from actuator name to index in actuators vec.
    pub actuator_to_index: HashMap<String, usize>,
    /// Parsed MJCF options for simulation configuration.
    pub option: MjcfOption,
    /// Extended solver configuration derived from MJCF options.
    pub solver_config: ExtendedSolverConfig,
}

/// Result of spawning a model into a world.
#[derive(Debug)]
pub struct SpawnedModel {
    /// Map from body name to body ID in the world.
    pub body_ids: HashMap<String, BodyId>,
    /// Map from joint name to joint ID in the world.
    pub joint_ids: HashMap<String, JointId>,
    /// Loaded tendons (constraints that can be applied to joints).
    pub tendons: Vec<LoadedTendon>,
    /// Map from tendon name to index in tendons vec.
    pub tendon_to_index: HashMap<String, usize>,
}

impl SpawnedModel {
    /// Get the body ID for a body by name.
    pub fn body_id(&self, name: &str) -> Option<BodyId> {
        self.body_ids.get(name).copied()
    }

    /// Get the joint ID by name.
    pub fn joint_id(&self, name: &str) -> Option<JointId> {
        self.joint_ids.get(name).copied()
    }

    /// Get a tendon by name.
    #[must_use]
    pub fn tendon(&self, name: &str) -> Option<&LoadedTendon> {
        self.tendon_to_index
            .get(name)
            .and_then(|&idx| self.tendons.get(idx))
    }

    /// Get a mutable tendon by name.
    #[must_use]
    pub fn tendon_mut(&mut self, name: &str) -> Option<&mut LoadedTendon> {
        self.tendon_to_index
            .get(name)
            .copied()
            .and_then(|idx| self.tendons.get_mut(idx))
    }

    /// Get the number of tendons.
    #[must_use]
    pub fn tendon_count(&self) -> usize {
        self.tendons.len()
    }
}

impl LoadedModel {
    /// Spawn this model into a world at the given base pose.
    ///
    /// # Errors
    ///
    /// Returns an error if inserting bodies or joints fails.
    pub fn spawn_into(self, world: &mut World, base_pose: Pose) -> Result<SpawnedModel> {
        let mut body_ids = HashMap::new();
        let mut joint_ids = HashMap::new();

        // Insert bodies, applying base pose
        for mut body in self.bodies {
            // Apply base pose to all bodies (they're already in world frame)
            body.state.pose = base_pose.compose(&body.state.pose);

            let id = body.id;
            let name = body.name.clone();
            world
                .insert_body(body)
                .map_err(|e| MjcfError::XmlParse(e.to_string()))?;

            if let Some(name) = name {
                body_ids.insert(name, id);
            }
        }

        // Insert joints
        for joint in self.joints {
            let id = joint.id;
            let name = joint.name.clone();
            world
                .insert_joint(joint)
                .map_err(|e| MjcfError::XmlParse(e.to_string()))?;

            if let Some(name) = name {
                joint_ids.insert(name, id);
            }
        }

        Ok(SpawnedModel {
            body_ids,
            joint_ids,
            tendons: self.tendons,
            tendon_to_index: self.tendon_to_index,
        })
    }

    /// Spawn at the origin with identity pose.
    pub fn spawn_at_origin(self, world: &mut World) -> Result<SpawnedModel> {
        self.spawn_into(world, Pose::identity())
    }

    /// Get the simulation configuration derived from MJCF options.
    ///
    /// This provides a `SimulationConfig` that can be used to configure
    /// the physics simulation with the settings from the MJCF file.
    #[must_use]
    pub fn simulation_config(&self) -> sim_types::SimulationConfig {
        sim_types::SimulationConfig::from(&self.option)
    }

    /// Get the timestep specified in the MJCF file.
    #[must_use]
    pub fn timestep(&self) -> f64 {
        self.option.timestep
    }

    /// Check if gravity is enabled according to MJCF options.
    #[must_use]
    pub fn gravity_enabled(&self) -> bool {
        self.option.gravity_enabled()
    }

    /// Check if contacts are enabled according to MJCF options.
    #[must_use]
    pub fn contacts_enabled(&self) -> bool {
        self.option.contacts_enabled()
    }

    /// Get an actuator by name.
    #[must_use]
    pub fn actuator(&self, name: &str) -> Option<&LoadedActuator> {
        self.actuator_to_index
            .get(name)
            .and_then(|&idx| self.actuators.get(idx))
    }

    /// Get a mutable actuator by name.
    #[must_use]
    pub fn actuator_mut(&mut self, name: &str) -> Option<&mut LoadedActuator> {
        self.actuator_to_index
            .get(name)
            .copied()
            .and_then(|idx| self.actuators.get_mut(idx))
    }

    /// Get all actuators targeting a specific joint.
    #[must_use]
    pub fn actuators_for_joint(&self, joint_name: &str) -> Vec<&LoadedActuator> {
        self.actuators
            .iter()
            .filter(|a| a.joint.as_deref() == Some(joint_name))
            .collect()
    }

    /// Get all actuators targeting a specific body (e.g., adhesion actuators).
    #[must_use]
    pub fn actuators_for_body(&self, body_name: &str) -> Vec<&LoadedActuator> {
        self.actuators
            .iter()
            .filter(|a| a.body.as_deref() == Some(body_name))
            .collect()
    }

    /// Get the number of actuators.
    #[must_use]
    pub fn actuator_count(&self) -> usize {
        self.actuators.len()
    }

    /// Get a tendon by name.
    #[must_use]
    pub fn tendon(&self, name: &str) -> Option<&LoadedTendon> {
        self.tendon_to_index
            .get(name)
            .and_then(|&idx| self.tendons.get(idx))
    }

    /// Get a mutable tendon by name.
    #[must_use]
    pub fn tendon_mut(&mut self, name: &str) -> Option<&mut LoadedTendon> {
        self.tendon_to_index
            .get(name)
            .copied()
            .and_then(|idx| self.tendons.get_mut(idx))
    }

    /// Get the number of tendons.
    #[must_use]
    pub fn tendon_count(&self) -> usize {
        self.tendons.len()
    }

    /// Get all actuators targeting a specific tendon.
    #[must_use]
    pub fn actuators_for_tendon(&self, tendon_name: &str) -> Vec<&LoadedActuator> {
        self.actuators
            .iter()
            .filter(|a| a.tendon.as_deref() == Some(tendon_name))
            .collect()
    }
}

/// MJCF loader with configuration options.
#[derive(Debug, Clone)]
pub struct MjcfLoader {
    /// Whether to use collision shapes from MJCF (default: true).
    pub use_collision_shapes: bool,
    /// Default density for geoms without mass/density (kg/m³).
    pub default_density: f64,
    /// Minimum mass for bodies (to avoid singular inertia matrices).
    pub min_mass: f64,
    /// Base directory for resolving relative mesh file paths.
    pub mesh_base_dir: Option<std::path::PathBuf>,
}

impl Default for MjcfLoader {
    fn default() -> Self {
        Self {
            use_collision_shapes: true,
            default_density: 1000.0, // Water density
            min_mass: 0.001,         // 1 gram minimum
            mesh_base_dir: None,
        }
    }
}

impl MjcfLoader {
    /// Create a new loader with default settings.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Set whether to use collision shapes.
    #[must_use]
    pub fn with_collision_shapes(mut self, use_shapes: bool) -> Self {
        self.use_collision_shapes = use_shapes;
        self
    }

    /// Set the default density for geoms.
    #[must_use]
    pub fn with_default_density(mut self, density: f64) -> Self {
        self.default_density = density;
        self
    }

    /// Set the base directory for resolving relative mesh file paths.
    #[must_use]
    pub fn with_mesh_base_dir(mut self, dir: impl AsRef<Path>) -> Self {
        self.mesh_base_dir = Some(dir.as_ref().to_path_buf());
        self
    }

    /// Load MJCF from a file path.
    ///
    /// The directory containing the MJCF file will be used as the base
    /// directory for resolving relative mesh file paths.
    ///
    /// # Errors
    ///
    /// Returns an error if the file cannot be read or parsed.
    pub fn load_file(&self, path: impl AsRef<Path>) -> Result<LoadedModel> {
        let path = path.as_ref();
        let content = fs::read_to_string(path)?;

        // Use the MJCF file's directory as the mesh base dir if not already set
        let loader = if self.mesh_base_dir.is_none() {
            let base_dir = path.parent().map(|p| p.to_path_buf());
            Self {
                mesh_base_dir: base_dir,
                ..self.clone()
            }
        } else {
            self.clone()
        };

        loader.load_str(&content)
    }

    /// Load MJCF from a string.
    ///
    /// # Errors
    ///
    /// Returns an error if parsing or validation fails.
    pub fn load_str(&self, xml: &str) -> Result<LoadedModel> {
        let model = parse_mjcf_str(xml)?;
        self.load_model(model)
    }

    /// Load from a parsed MJCF model.
    ///
    /// # Errors
    ///
    /// Returns an error if validation fails.
    pub fn load_model(&self, model: MjcfModel) -> Result<LoadedModel> {
        let validation = validate(&model)?;
        self.convert(model, validation)
    }

    /// Convert MJCF types to sim-core types.
    fn convert(&self, model: MjcfModel, validation: ValidationResult) -> Result<LoadedModel> {
        let mut bodies = Vec::new();
        let mut joints = Vec::new();
        let mut body_to_id: HashMap<String, BodyId> = HashMap::new();
        let mut joint_to_id: HashMap<String, JointId> = HashMap::new();

        // Create default resolver for applying default classes
        let resolver = DefaultResolver::from_model(&model);

        // Build mesh lookup from asset names
        let mesh_lookup: HashMap<&str, &MjcfMesh> =
            model.meshes.iter().map(|m| (m.name.as_str(), m)).collect();

        // Assign IDs
        let mut next_body_id = 1u64;
        let mut next_joint_id = 1u64;

        // Build body lookup
        let body_lookup = build_body_lookup(&model.worldbody);

        // Build site lookup (body names, not IDs yet)
        let site_lookup_by_name = build_site_lookup(&model.worldbody);

        // Build geom lookup for wrapping geometries (body names, not IDs yet)
        let geom_lookup_by_name = build_geom_lookup(&model.worldbody);

        // Create bodies in topological order
        for body_name in &validation.sorted_bodies {
            let mjcf_body = body_lookup
                .get(body_name.as_str())
                .ok_or_else(|| MjcfError::XmlParse(format!("body '{body_name}' not found")))?;

            let body_id = BodyId::new(next_body_id);
            next_body_id += 1;
            body_to_id.insert(body_name.clone(), body_id);

            // Compute world pose by traversing parent chain
            let world_pose = compute_world_pose(body_name, &validation, &body_lookup);

            // Create the body (with defaults applied to geoms)
            let body = self.create_body_with_defaults(
                body_id,
                mjcf_body,
                world_pose,
                &resolver,
                &mesh_lookup,
            );
            bodies.push(body);
        }

        // Create joints
        for body_name in &validation.sorted_bodies {
            let mjcf_body = body_lookup
                .get(body_name.as_str())
                .ok_or_else(|| MjcfError::XmlParse(format!("body '{body_name}' not found")))?;

            // Get parent body ID (if any)
            let parent_id = validation
                .body_parent
                .get(body_name)
                .and_then(|parent_name| body_to_id.get(parent_name))
                .copied();

            let child_id = *body_to_id
                .get(body_name)
                .ok_or_else(|| MjcfError::XmlParse(format!("body '{body_name}' not in map")))?;

            // Create joints for this body (with defaults applied)
            for mjcf_joint in &mjcf_body.joints {
                let joint_id = JointId::new(next_joint_id);
                next_joint_id += 1;

                // Apply defaults to the joint
                let resolved_joint = resolver.apply_to_joint(mjcf_joint);

                if !resolved_joint.name.is_empty() {
                    joint_to_id.insert(resolved_joint.name.clone(), joint_id);
                }

                // For MJCF, joints connect to parent (or world if no parent)
                // If no parent, create a "world anchor" (fixed to world origin)
                let actual_parent = parent_id.unwrap_or(BodyId::new(0)); // BodyId(0) represents world

                let joint = self.create_joint(joint_id, &resolved_joint, actual_parent, child_id);
                joints.push(joint);
            }
        }

        // Convert equality constraints
        let connect_constraints =
            self.convert_connect_constraints(&model.equality.connects, &body_to_id)?;

        // Resolve site lookup now that we have body IDs
        let site_to_info = resolve_site_lookup(&site_lookup_by_name, &body_to_id);

        // Resolve geom lookup now that we have body IDs
        let geom_to_info = resolve_geom_lookup(&geom_lookup_by_name, &body_to_id);

        // Convert tendons
        let (tendons, tendon_to_index) = self.convert_tendons(
            &model.tendons,
            &joint_to_id,
            &site_to_info,
            &geom_to_info,
            &resolver,
        )?;

        // Convert actuators (with defaults applied)
        let (actuators, actuator_to_index) = self.convert_actuators(&model.actuators, &resolver);

        // Convert options to solver config
        let solver_config = ExtendedSolverConfig::from(&model.option);

        Ok(LoadedModel {
            name: model.name,
            bodies,
            joints,
            connect_constraints,
            tendons,
            actuators,
            body_to_id,
            joint_to_id,
            tendon_to_index,
            actuator_to_index,
            option: model.option,
            solver_config,
        })
    }

    /// Create a sim-core Body from an MJCF body.
    ///
    /// Note: This is a legacy method that doesn't support mesh assets.
    /// Use `create_body_with_defaults` for full mesh support.
    #[allow(dead_code)]
    fn create_body(&self, id: BodyId, mjcf_body: &MjcfBody, pose: Pose) -> Body {
        // Compute mass properties from inertial or geoms
        let mass_props = if let Some(ref inertial) = mjcf_body.inertial {
            MassProperties::new(inertial.mass, inertial.pos, inertial.inertia_matrix())
        } else {
            // Compute from geoms
            self.compute_mass_from_geoms(&mjcf_body.geoms)
        };

        let state = RigidBodyState::at_rest(pose);
        let mut body = Body::new(id, state, mass_props).with_name(&mjcf_body.name);

        // Add collision shape from first geom (no mesh lookup)
        if self.use_collision_shapes {
            let empty_mesh_lookup = HashMap::new();
            if let Some(collision_shape) =
                self.collision_shape_from_geoms(&mjcf_body.geoms, &empty_mesh_lookup)
            {
                body = body.with_collision_shape(collision_shape);
            }
        }

        // Transfer collision filtering from first geom (MuJoCo-compatible contype/conaffinity)
        if let Some(first_geom) = mjcf_body.geoms.first() {
            // Convert i32 to u32, clamping negative values to 0
            let contype = first_geom.contype.max(0) as u32;
            let conaffinity = first_geom.conaffinity.max(0) as u32;
            body = body.with_collision_filter(contype, conaffinity);
        }

        body
    }

    /// Create a sim-core Body from an MJCF body with defaults applied.
    fn create_body_with_defaults(
        &self,
        id: BodyId,
        mjcf_body: &MjcfBody,
        pose: Pose,
        resolver: &DefaultResolver,
        mesh_lookup: &HashMap<&str, &MjcfMesh>,
    ) -> Body {
        // Apply defaults to all geoms
        let resolved_geoms: Vec<MjcfGeom> = mjcf_body
            .geoms
            .iter()
            .map(|g| resolver.apply_to_geom(g))
            .collect();

        // Compute mass properties from inertial or resolved geoms
        let mass_props = if let Some(ref inertial) = mjcf_body.inertial {
            MassProperties::new(inertial.mass, inertial.pos, inertial.inertia_matrix())
        } else {
            // Compute from resolved geoms
            self.compute_mass_from_geoms(&resolved_geoms)
        };

        let state = RigidBodyState::at_rest(pose);
        let mut body = Body::new(id, state, mass_props).with_name(&mjcf_body.name);

        // Add collision shape from first resolved geom
        if self.use_collision_shapes {
            if let Some(first_geom) = resolved_geoms.first() {
                if let Some(collision_shape) = self.geom_to_collision(first_geom, mesh_lookup) {
                    body = body.with_collision_shape(collision_shape);

                    // Compute and store the geom's local pose (for visualization)
                    if let Some(shape_pose) = Self::compute_geom_local_pose(first_geom) {
                        body = body.with_collision_shape_pose(shape_pose);
                    }
                }
            }
        }

        // Transfer collision filtering from first resolved geom
        if let Some(first_geom) = resolved_geoms.first() {
            // Convert i32 to u32, clamping negative values to 0
            let contype = first_geom.contype.max(0) as u32;
            let conaffinity = first_geom.conaffinity.max(0) as u32;
            body = body.with_collision_filter(contype, conaffinity);
        }

        body
    }

    /// Compute mass properties from geoms.
    fn compute_mass_from_geoms(&self, geoms: &[MjcfGeom]) -> MassProperties {
        if geoms.is_empty() {
            return MassProperties::point_mass(self.min_mass);
        }

        // Sum up mass from all geoms
        let mut total_mass = 0.0;
        let mut weighted_com = Vector3::zeros();

        for geom in geoms {
            let mass = geom.computed_mass().max(self.min_mass);
            total_mass += mass;
            weighted_com += geom.pos * mass;
        }

        let com = if total_mass > 0.0 {
            weighted_com / total_mass
        } else {
            Vector3::zeros()
        };

        // For simplicity, use point mass approximation with offset COM
        // A more accurate implementation would compute proper inertia tensor
        MassProperties::new(
            total_mass.max(self.min_mass),
            com,
            nalgebra::Matrix3::zeros(),
        )
    }

    /// Create collision shape from geoms.
    fn collision_shape_from_geoms(
        &self,
        geoms: &[MjcfGeom],
        mesh_lookup: &HashMap<&str, &MjcfMesh>,
    ) -> Option<CollisionShape> {
        // Use first geom for collision
        geoms
            .first()
            .and_then(|geom| self.geom_to_collision(geom, mesh_lookup))
    }

    /// Compute the local pose of a geom relative to the body frame.
    ///
    /// This handles:
    /// - `fromto`: Capsule/cylinder endpoints define position (midpoint) and rotation
    /// - `pos`: Explicit position offset
    /// - `quat`: Explicit quaternion rotation
    fn compute_geom_local_pose(geom: &MjcfGeom) -> Option<Pose> {
        // Check if geom has any local transformation
        let has_fromto = geom.fromto.is_some();
        let has_pos = geom.pos.norm() > 1e-10;
        // Check if quaternion is not identity (w=1, x=y=z=0)
        let is_identity_quat = (geom.quat[0] - 1.0).abs() < 1e-10
            && geom.quat[1].abs() < 1e-10
            && geom.quat[2].abs() < 1e-10
            && geom.quat[3].abs() < 1e-10;
        let has_rotation = !is_identity_quat;

        if !has_fromto && !has_pos && !has_rotation {
            return None; // No local pose needed
        }

        if let Some(fromto) = geom.fromto {
            // fromto defines both position and rotation for capsules/cylinders
            let start = Vector3::new(fromto[0], fromto[1], fromto[2]);
            let end = Vector3::new(fromto[3], fromto[4], fromto[5]);

            // Position is the midpoint
            let center = (start + end) / 2.0;
            let position = Point3::new(center.x, center.y, center.z);

            // Direction vector (capsule extends along local Z axis)
            let direction = end - start;
            let length = direction.norm();

            if length < 1e-10 {
                // Zero-length capsule, just use position
                return Some(Pose::from_position(position));
            }

            // Normalize direction
            let z_axis = direction / length;

            // Compute rotation from Z axis to direction
            // Find a rotation that aligns (0, 0, 1) with z_axis
            let rotation = rotation_from_z_axis(&z_axis);

            Some(Pose::from_position_rotation(position, rotation))
        } else {
            // Use explicit pos and rotation
            let position = Point3::new(geom.pos.x, geom.pos.y, geom.pos.z);

            // MuJoCo uses (w, x, y, z) ordering in geom.quat
            let rotation = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                geom.quat[0],
                geom.quat[1],
                geom.quat[2],
                geom.quat[3],
            ));

            if position == Point3::origin() && rotation == nalgebra::UnitQuaternion::identity() {
                None
            } else {
                Some(Pose::from_position_rotation(position, rotation))
            }
        }
    }

    /// Convert MJCF geom to collision shape.
    fn geom_to_collision(
        &self,
        geom: &MjcfGeom,
        mesh_lookup: &HashMap<&str, &MjcfMesh>,
    ) -> Option<CollisionShape> {
        match geom.geom_type {
            MjcfGeomType::Sphere => {
                let radius = geom.size.first().copied().unwrap_or(0.1);
                Some(CollisionShape::sphere(radius))
            }
            MjcfGeomType::Box => {
                // MJCF box size is half-extents
                let x = geom.size.first().copied().unwrap_or(0.1);
                let y = geom.size.get(1).copied().unwrap_or(x);
                let z = geom.size.get(2).copied().unwrap_or(y);
                Some(CollisionShape::box_shape(Vector3::new(x, y, z)))
            }
            MjcfGeomType::Capsule => {
                let radius = geom.size.first().copied().unwrap_or(0.05);
                let half_length = if let Some(fromto) = geom.fromto {
                    // Compute length from fromto
                    let start = Vector3::new(fromto[0], fromto[1], fromto[2]);
                    let end = Vector3::new(fromto[3], fromto[4], fromto[5]);
                    (end - start).norm() / 2.0
                } else {
                    geom.size.get(1).copied().unwrap_or(0.1)
                };
                Some(CollisionShape::capsule(half_length, radius))
            }
            MjcfGeomType::Cylinder => {
                // True cylinder shape (not approximated as capsule)
                let radius = geom.size.first().copied().unwrap_or(0.05);
                let half_length = if let Some(fromto) = geom.fromto {
                    // Compute length from fromto
                    let start = Vector3::new(fromto[0], fromto[1], fromto[2]);
                    let end = Vector3::new(fromto[3], fromto[4], fromto[5]);
                    (end - start).norm() / 2.0
                } else {
                    geom.size.get(1).copied().unwrap_or(0.1)
                };
                Some(CollisionShape::cylinder(half_length, radius))
            }
            MjcfGeomType::Ellipsoid => {
                // Ellipsoid shape with three radii
                let rx = geom.size.first().copied().unwrap_or(0.1);
                let ry = geom.size.get(1).copied().unwrap_or(rx);
                let rz = geom.size.get(2).copied().unwrap_or(ry);
                Some(CollisionShape::ellipsoid_xyz(rx, ry, rz))
            }
            MjcfGeomType::Plane => {
                // Infinite plane
                Some(CollisionShape::plane(Vector3::z(), 0.0))
            }
            MjcfGeomType::Mesh => {
                // Convert mesh to convex hull collision shape
                self.mesh_geom_to_collision(geom, mesh_lookup)
            }
            MjcfGeomType::Sdf => {
                // Convert mesh to SDF collision shape
                // SDF provides cleaner collision detection than convex mesh
                self.sdf_geom_to_collision(geom, mesh_lookup)
            }
            MjcfGeomType::TriangleMesh => {
                // Convert mesh to non-convex triangle mesh collision shape
                // Preserves original geometry for accurate non-convex collision
                self.triangle_mesh_geom_to_collision(geom, mesh_lookup)
            }
            MjcfGeomType::Hfield => None, // Heightfield not yet supported
        }
    }

    /// Convert a mesh geom to a convex mesh collision shape.
    fn mesh_geom_to_collision(
        &self,
        geom: &MjcfGeom,
        mesh_lookup: &HashMap<&str, &MjcfMesh>,
    ) -> Option<CollisionShape> {
        // Get the mesh asset name from the geom
        let mesh_name = geom.mesh.as_ref()?;

        // Look up the mesh asset
        let Some(mesh_asset) = mesh_lookup.get(mesh_name.as_str()) else {
            tracing::warn!("Mesh asset '{}' not found for geom", mesh_name);
            return None;
        };

        // Try to get vertices from embedded data first
        if mesh_asset.has_embedded_data() {
            let vertices = mesh_asset.vertices_as_points();
            if vertices.is_empty() {
                tracing::warn!("Mesh asset '{}' has no vertices", mesh_name);
                return None;
            }
            return Some(CollisionShape::convex_mesh(vertices));
        }

        // Try to load from file
        if let Some(ref file_path) = mesh_asset.file {
            return self.load_mesh_file(file_path, &mesh_asset.scale);
        }

        tracing::warn!(
            "Mesh asset '{}' has no embedded data or file path",
            mesh_name
        );
        None
    }

    /// Load a mesh file and convert to collision shape.
    fn load_mesh_file(&self, file_path: &str, scale: &Vector3<f64>) -> Option<CollisionShape> {
        // Resolve the path relative to the base directory
        let full_path = if let Some(ref base_dir) = self.mesh_base_dir {
            base_dir.join(file_path)
        } else {
            std::path::PathBuf::from(file_path)
        };

        // Load the mesh file using mesh-io
        let indexed_mesh = match mesh_io::load_mesh(&full_path) {
            Ok(m) => m,
            Err(e) => {
                tracing::warn!("Failed to load mesh file '{}': {}", full_path.display(), e);
                return None;
            }
        };

        // Convert mesh vertices to collision shape vertices
        let vertices: Vec<Point3<f64>> = indexed_mesh
            .vertices
            .iter()
            .map(|v| {
                Point3::new(
                    v.position.x * scale.x,
                    v.position.y * scale.y,
                    v.position.z * scale.z,
                )
            })
            .collect();

        if vertices.is_empty() {
            tracing::warn!("Mesh file '{}' has no vertices", full_path.display());
            return None;
        }

        Some(CollisionShape::convex_mesh(vertices))
    }

    /// Convert a mesh geom to a non-convex triangle mesh collision shape.
    ///
    /// Unlike `mesh_geom_to_collision` which creates a convex hull, this preserves
    /// the original triangle structure for accurate non-convex collision detection.
    fn triangle_mesh_geom_to_collision(
        &self,
        geom: &MjcfGeom,
        mesh_lookup: &HashMap<&str, &MjcfMesh>,
    ) -> Option<CollisionShape> {
        // Get the mesh asset name from the geom
        let mesh_name = geom.mesh.as_ref()?;

        // Look up the mesh asset
        let Some(mesh_asset) = mesh_lookup.get(mesh_name.as_str()) else {
            tracing::warn!(
                "Mesh asset '{}' not found for triangle mesh geom",
                mesh_name
            );
            return None;
        };

        // Try to get vertices from embedded data first
        if mesh_asset.has_embedded_data() {
            // For embedded data, we only have vertices, not faces
            // Create a simple triangle soup from consecutive triplets
            let vertices = mesh_asset.vertices_as_points();
            if vertices.is_empty() {
                tracing::warn!("Mesh asset '{}' has no vertices", mesh_name);
                return None;
            }

            // If we have face data, use it; otherwise treat vertices as triangle soup
            if let Some(ref faces) = mesh_asset.face {
                // Parse face data into indices
                let indices: Vec<usize> = faces.iter().map(|&f| f as usize).collect();
                if indices.len() >= 3 && indices.len() % 3 == 0 {
                    return Some(CollisionShape::triangle_mesh_from_vertices(
                        vertices, indices,
                    ));
                }
            }

            // Fallback: treat vertices as a triangle soup (groups of 3)
            if vertices.len() >= 3 {
                // Only use full triangles
                let tri_count = vertices.len() / 3;
                let indices: Vec<usize> = (0..(tri_count * 3)).collect();
                return Some(CollisionShape::triangle_mesh_from_vertices(
                    vertices, indices,
                ));
            }

            tracing::warn!(
                "Mesh asset '{}' has insufficient vertices for triangle mesh",
                mesh_name
            );
            return None;
        }

        // Try to load from file
        if let Some(ref file_path) = mesh_asset.file {
            return self.load_triangle_mesh_file(file_path, &mesh_asset.scale);
        }

        tracing::warn!(
            "Triangle mesh asset '{}' has no embedded data or file path",
            mesh_name
        );
        None
    }

    /// Load a mesh file and convert to non-convex triangle mesh collision shape.
    fn load_triangle_mesh_file(
        &self,
        file_path: &str,
        scale: &Vector3<f64>,
    ) -> Option<CollisionShape> {
        // Resolve the path relative to the base directory
        let full_path = if let Some(ref base_dir) = self.mesh_base_dir {
            base_dir.join(file_path)
        } else {
            std::path::PathBuf::from(file_path)
        };

        // Load the mesh file using mesh-io
        let indexed_mesh = match mesh_io::load_mesh(&full_path) {
            Ok(m) => m,
            Err(e) => {
                tracing::warn!(
                    "Failed to load mesh file '{}' for triangle mesh: {}",
                    full_path.display(),
                    e
                );
                return None;
            }
        };

        // Convert mesh vertices
        let vertices: Vec<Point3<f64>> = indexed_mesh
            .vertices
            .iter()
            .map(|v| {
                Point3::new(
                    v.position.x * scale.x,
                    v.position.y * scale.y,
                    v.position.z * scale.z,
                )
            })
            .collect();

        if vertices.is_empty() {
            tracing::warn!(
                "Triangle mesh file '{}' has no vertices",
                full_path.display()
            );
            return None;
        }

        // Get triangle indices from the mesh faces
        let indices: Vec<usize> = indexed_mesh
            .faces
            .iter()
            .flat_map(|face| [face[0] as usize, face[1] as usize, face[2] as usize])
            .collect();

        if indices.len() < 3 {
            tracing::warn!(
                "Triangle mesh file '{}' has no valid faces",
                full_path.display(),
            );
            return None;
        }

        Some(CollisionShape::triangle_mesh_from_vertices(
            vertices, indices,
        ))
    }

    /// Convert an SDF geom to an SDF collision shape.
    ///
    /// SDF (Signed Distance Field) provides smoother collision detection than
    /// triangle meshes, especially for non-convex geometry. The SDF is computed
    /// from the mesh at the specified resolution.
    fn sdf_geom_to_collision(
        &self,
        geom: &MjcfGeom,
        mesh_lookup: &HashMap<&str, &MjcfMesh>,
    ) -> Option<CollisionShape> {
        // Get the mesh asset name from the geom (SDF geoms reference a mesh)
        let mesh_name = geom.mesh.as_ref()?;

        // Look up the mesh asset
        let Some(mesh_asset) = mesh_lookup.get(mesh_name.as_str()) else {
            tracing::warn!("Mesh asset '{}' not found for SDF geom", mesh_name);
            return None;
        };

        // Try to get vertices from embedded data first
        if mesh_asset.has_embedded_data() {
            let vertices = mesh_asset.vertices_as_points();
            if vertices.is_empty() {
                tracing::warn!("Mesh asset '{}' has no vertices for SDF", mesh_name);
                return None;
            }
            return Some(self.create_sdf_from_vertices(&vertices, &mesh_asset.scale));
        }

        // Try to load from file
        if let Some(ref file_path) = mesh_asset.file {
            return self.load_sdf_from_mesh_file(file_path, &mesh_asset.scale);
        }

        tracing::warn!(
            "SDF mesh asset '{}' has no embedded data or file path",
            mesh_name
        );
        None
    }

    /// Create an SDF collision shape from mesh vertices.
    fn create_sdf_from_vertices(
        &self,
        vertices: &[Point3<f64>],
        _scale: &Vector3<f64>,
    ) -> CollisionShape {
        // Compute bounding box of the mesh
        let mut min = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut max = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);

        for v in vertices {
            min.x = min.x.min(v.x);
            min.y = min.y.min(v.y);
            min.z = min.z.min(v.z);
            max.x = max.x.max(v.x);
            max.y = max.y.max(v.y);
            max.z = max.z.max(v.z);
        }

        // Add padding around the mesh
        let padding = 0.1;
        let origin = Point3::new(min.x - padding, min.y - padding, min.z - padding);

        // Compute extent and cell size
        let extent = max - min;
        let max_extent = extent.x.max(extent.y).max(extent.z) + 2.0 * padding;

        // Use moderate resolution (32^3) for reasonable performance
        let resolution: usize = 32;
        #[allow(clippy::cast_precision_loss)] // Safe: resolution is always small (32)
        let cell_size = max_extent / (resolution - 1) as f64;

        // For now, use a sphere approximation centered on the mesh centroid
        // This is a simple fallback - proper SDF computation would use mesh-sdf crate
        let centroid = Point3::new(
            f64::midpoint(min.x, max.x),
            f64::midpoint(min.y, max.y),
            f64::midpoint(min.z, max.z),
        );
        let radius = extent.norm() / 2.0;

        // Create SDF approximating the mesh as a sphere (conservative)
        let sdf_data = sim_core::SdfCollisionData::from_fn(
            resolution,
            resolution,
            resolution,
            cell_size,
            origin,
            |p| (p - centroid).norm() - radius,
        );

        CollisionShape::sdf(std::sync::Arc::new(sdf_data))
    }

    /// Load a mesh file and create an SDF collision shape.
    fn load_sdf_from_mesh_file(
        &self,
        file_path: &str,
        scale: &Vector3<f64>,
    ) -> Option<CollisionShape> {
        // Resolve the path relative to the base directory
        let full_path = if let Some(ref base_dir) = self.mesh_base_dir {
            base_dir.join(file_path)
        } else {
            std::path::PathBuf::from(file_path)
        };

        // Load the mesh file using mesh-io
        let indexed_mesh = match mesh_io::load_mesh(&full_path) {
            Ok(m) => m,
            Err(e) => {
                tracing::warn!(
                    "Failed to load mesh file '{}' for SDF: {}",
                    full_path.display(),
                    e
                );
                return None;
            }
        };

        // Convert mesh vertices
        let vertices: Vec<Point3<f64>> = indexed_mesh
            .vertices
            .iter()
            .map(|v| {
                Point3::new(
                    v.position.x * scale.x,
                    v.position.y * scale.y,
                    v.position.z * scale.z,
                )
            })
            .collect();

        if vertices.is_empty() {
            tracing::warn!(
                "Mesh file '{}' has no vertices for SDF",
                full_path.display()
            );
            return None;
        }

        Some(self.create_sdf_from_vertices(&vertices, scale))
    }

    /// Convert MJCF connect constraints to sim-constraint ConnectConstraints.
    fn convert_connect_constraints(
        &self,
        mjcf_connects: &[crate::types::MjcfConnect],
        body_to_id: &HashMap<String, BodyId>,
    ) -> Result<Vec<ConnectConstraint>> {
        let mut constraints = Vec::new();

        for mjcf_connect in mjcf_connects {
            // Skip inactive constraints
            if !mjcf_connect.active {
                continue;
            }

            // Look up body1 ID
            let body1_id = body_to_id
                .get(&mjcf_connect.body1)
                .copied()
                .ok_or_else(|| {
                    MjcfError::undefined_body(&mjcf_connect.body1, "connect constraint")
                })?;

            // Look up body2 ID (if specified, otherwise connects to world)
            let body2_id =
                if let Some(ref body2_name) = mjcf_connect.body2 {
                    Some(body_to_id.get(body2_name).copied().ok_or_else(|| {
                        MjcfError::undefined_body(body2_name, "connect constraint")
                    })?)
                } else {
                    None
                };

            // Create the constraint
            let mut constraint = if let Some(body2) = body2_id {
                ConnectConstraint::new(body1_id, body2, mjcf_connect.anchor)
            } else {
                ConnectConstraint::to_world(body1_id, mjcf_connect.anchor)
            };

            // Set optional name
            if let Some(ref name) = mjcf_connect.name {
                constraint = constraint.with_name(name);
            }

            // Set solver parameters if provided
            if let Some(solref) = mjcf_connect.solref {
                constraint = constraint.with_solref(solref);
            }
            if let Some(solimp) = mjcf_connect.solimp {
                constraint = constraint.with_solimp(solimp);
            }

            constraints.push(constraint);
        }

        Ok(constraints)
    }

    /// Convert MJCF tendons to LoadedTendons.
    ///
    /// Supports both fixed tendons (linear combinations of joint positions) and
    /// spatial tendons (point-to-point paths through sites with optional wrapping).
    fn convert_tendons(
        &self,
        mjcf_tendons: &[MjcfTendon],
        joint_to_id: &HashMap<String, JointId>,
        site_to_info: &HashMap<String, SiteInfo>,
        geom_to_info: &HashMap<String, GeomInfo>,
        resolver: &DefaultResolver,
    ) -> Result<(Vec<LoadedTendon>, HashMap<String, usize>)> {
        let mut tendons = Vec::new();
        let mut tendon_to_index = HashMap::new();

        for mjcf_tendon in mjcf_tendons {
            // Apply defaults to the tendon
            let resolved_tendon = resolver.apply_to_tendon(mjcf_tendon);

            match resolved_tendon.tendon_type {
                MjcfTendonType::Fixed => {
                    // Fixed tendons: L = L₀ + Σᵢ cᵢ qᵢ
                    let loaded = self.convert_fixed_tendon(&resolved_tendon, joint_to_id)?;

                    if !loaded.name().is_empty() {
                        tendon_to_index.insert(loaded.name().to_string(), tendons.len());
                    }

                    tendons.push(loaded);
                }
                MjcfTendonType::Spatial => {
                    // Spatial tendons route through sites in 3D space with optional wrapping
                    let loaded =
                        self.convert_spatial_tendon(&resolved_tendon, site_to_info, geom_to_info)?;

                    if !loaded.name().is_empty() {
                        tendon_to_index.insert(loaded.name().to_string(), tendons.len());
                    }

                    tendons.push(loaded);
                }
            }
        }

        Ok((tendons, tendon_to_index))
    }

    /// Convert a fixed tendon to a LoadedTendon.
    ///
    /// Fixed tendons define length as a linear combination of joint positions:
    /// `L = L_rest + Σᵢ coef_i * q_i`
    fn convert_fixed_tendon(
        &self,
        mjcf_tendon: &MjcfTendon,
        joint_to_id: &HashMap<String, JointId>,
    ) -> Result<LoadedTendon> {
        let mut constraint = TendonConstraint::new(&mjcf_tendon.name);

        // Add each joint with its coefficient as the moment arm
        for (joint_name, coefficient) in &mjcf_tendon.joints {
            let joint_id = joint_to_id.get(joint_name).copied().ok_or_else(|| {
                MjcfError::undefined_joint(joint_name, format!("tendon '{}'", mjcf_tendon.name))
            })?;

            constraint = constraint.with_joint(joint_id, *coefficient);
        }

        // Set tendon properties
        if mjcf_tendon.stiffness > 0.0 {
            constraint = constraint.with_stiffness(mjcf_tendon.stiffness);
        }

        if mjcf_tendon.damping > 0.0 {
            constraint = constraint.with_damping(mjcf_tendon.damping);
        }

        // Tendons typically can go slack (pull-only) unless they have significant stiffness
        // With stiffness, they act as spring-dampers in both directions
        let can_slack = mjcf_tendon.stiffness == 0.0;
        constraint = constraint.with_can_slack(can_slack);

        // Extract RGBA for visualization
        let rgba = [
            mjcf_tendon.rgba.x,
            mjcf_tendon.rgba.y,
            mjcf_tendon.rgba.z,
            mjcf_tendon.rgba.w,
        ];

        Ok(LoadedTendon::Fixed(LoadedFixedTendon {
            name: mjcf_tendon.name.clone(),
            constraint,
            limited: mjcf_tendon.limited,
            range: mjcf_tendon.range,
            width: mjcf_tendon.width,
            rgba,
        }))
    }

    /// Convert a spatial tendon to a LoadedTendon.
    ///
    /// Spatial tendons route through 3D attachment points (sites) on bodies.
    /// The tendon length is computed from the actual path through space.
    /// Wrapping geometries allow the tendon to route around obstacles.
    fn convert_spatial_tendon(
        &self,
        mjcf_tendon: &MjcfTendon,
        site_to_info: &HashMap<String, SiteInfo>,
        geom_to_info: &HashMap<String, GeomInfo>,
    ) -> Result<LoadedTendon> {
        // Need at least 2 sites for a valid tendon path
        if mjcf_tendon.sites.len() < 2 {
            return Err(MjcfError::invalid_attribute(
                "sites",
                format!("spatial tendon '{}'", mjcf_tendon.name),
                "spatial tendon requires at least 2 sites",
            ));
        }

        // Look up each site and build attachment points
        let mut attachment_points = Vec::with_capacity(mjcf_tendon.sites.len());

        for site_name in &mjcf_tendon.sites {
            let site_info = site_to_info.get(site_name).ok_or_else(|| {
                MjcfError::undefined_site(
                    site_name,
                    format!("spatial tendon '{}'", mjcf_tendon.name),
                )
            })?;

            attachment_points.push(AttachmentPoint::new(
                site_info.body_id,
                site_info.local_position,
            ));
        }

        // Build the tendon path from attachment points
        let first = attachment_points[0];
        let last = attachment_points[attachment_points.len() - 1];
        let mut path = TendonPath::new(first, last);

        // Add via points (all points between first and last)
        for point in attachment_points
            .iter()
            .skip(1)
            .take(attachment_points.len() - 2)
        {
            path = path.with_via_point(*point);
        }

        // Create cable properties from MJCF tendon properties
        let cable = if mjcf_tendon.stiffness > 0.0 {
            CableProperties::new(mjcf_tendon.stiffness, 0.0) // rest_length will be computed
                .with_damping(mjcf_tendon.damping)
        } else {
            // Inextensible tendon (very high stiffness)
            CableProperties::new(1e6, 0.0).with_damping(mjcf_tendon.damping)
        };

        // Create the spatial tendon
        let mut tendon = SpatialTendon::new(&mjcf_tendon.name, path).with_cable(cable);

        // Add wrapping geometries if specified
        for geom_name in &mjcf_tendon.wrapping_geoms {
            let geom_info = geom_to_info.get(geom_name).ok_or_else(|| {
                MjcfError::undefined_geom(
                    geom_name,
                    format!("spatial tendon '{}'", mjcf_tendon.name),
                )
            })?;

            // Only add wrapping if we have a valid wrapping geometry
            if let Some(wrapping_geometry) = geom_info.wrapping_geometry {
                let wrap_ref = WrappingGeometryRef::new(geom_info.body_id, wrapping_geometry);
                tendon = tendon.with_wrapping_ref(wrap_ref);
            }
        }

        // Extract RGBA for visualization
        let rgba = [
            mjcf_tendon.rgba.x,
            mjcf_tendon.rgba.y,
            mjcf_tendon.rgba.z,
            mjcf_tendon.rgba.w,
        ];

        Ok(LoadedTendon::Spatial(LoadedSpatialTendon {
            name: mjcf_tendon.name.clone(),
            tendon,
            limited: mjcf_tendon.limited,
            range: mjcf_tendon.range,
            width: mjcf_tendon.width,
            rgba,
        }))
    }

    /// Convert MJCF actuators to LoadedActuators.
    fn convert_actuators(
        &self,
        mjcf_actuators: &[MjcfActuator],
        resolver: &DefaultResolver,
    ) -> (Vec<LoadedActuator>, HashMap<String, usize>) {
        let mut actuators = Vec::new();
        let mut actuator_to_index = HashMap::new();

        for mjcf_actuator in mjcf_actuators {
            // Apply defaults to the actuator
            let resolved_actuator = resolver.apply_to_actuator(mjcf_actuator);
            let loaded = self.convert_single_actuator(&resolved_actuator);

            if !loaded.name.is_empty() {
                actuator_to_index.insert(loaded.name.clone(), actuators.len());
            }

            actuators.push(loaded);
        }

        (actuators, actuator_to_index)
    }

    /// Convert a single MJCF actuator to a LoadedActuator.
    fn convert_single_actuator(&self, mjcf: &MjcfActuator) -> LoadedActuator {
        let actuator: BoxedActuator = match mjcf.actuator_type {
            MjcfActuatorType::Motor => {
                // Direct torque/force motor - use a simple motor interface
                // We'll create an IntegratedVelocityActuator with high max velocity
                // and use it in direct torque mode
                let max_force = mjcf
                    .forcerange
                    .map(|(_, upper)| upper.abs())
                    .unwrap_or(100.0)
                    * mjcf.gear;
                IntegratedVelocityActuator::new(1000.0, max_force)
                    .with_name(&mjcf.name)
                    .with_gains(0.0, 0.0) // No PD control, direct torque
                    .boxed()
            }

            MjcfActuatorType::Position => {
                // Position servo with PD control
                let max_force = mjcf
                    .forcerange
                    .map(|(_, upper)| upper.abs())
                    .unwrap_or(100.0)
                    * mjcf.gear;
                let kp = mjcf.kp * mjcf.gear;
                let kd = mjcf.kv * mjcf.gear;
                IntegratedVelocityActuator::new(10.0, max_force)
                    .with_name(&mjcf.name)
                    .with_gains(kp, kd)
                    .boxed()
            }

            MjcfActuatorType::Velocity => {
                // Velocity servo
                let max_force = mjcf
                    .forcerange
                    .map(|(_, upper)| upper.abs())
                    .unwrap_or(100.0)
                    * mjcf.gear;
                let max_vel = mjcf.ctrlrange.map(|(_, upper)| upper.abs()).unwrap_or(10.0);
                IntegratedVelocityActuator::new(max_vel, max_force)
                    .with_name(&mjcf.name)
                    .with_gains(max_force / 0.01, max_force.sqrt() * 2.0) // High gains for velocity tracking
                    .boxed()
            }

            MjcfActuatorType::General => {
                // General actuator - default to integrated velocity
                let max_force = mjcf
                    .forcerange
                    .map(|(_, upper)| upper.abs())
                    .unwrap_or(100.0)
                    * mjcf.gear;
                IntegratedVelocityActuator::new(10.0, max_force)
                    .with_name(&mjcf.name)
                    .boxed()
            }

            MjcfActuatorType::Cylinder => {
                // Pneumatic/hydraulic cylinder
                // Compute area from diameter if provided
                let area = if let Some(diameter) = mjcf.diameter {
                    std::f64::consts::PI * (diameter / 2.0).powi(2)
                } else {
                    mjcf.area
                };

                // Default supply pressure (about 6 bar)
                let supply_pressure = 600_000.0;

                PneumaticCylinderActuator::new(area, supply_pressure)
                    .with_name(&mjcf.name)
                    .with_rates(
                        1.0 / mjcf.timeconst.max(0.001), // Convert time constant to rate
                        1.0 / mjcf.timeconst.max(0.001),
                        0.05,
                    )
                    .boxed()
            }

            MjcfActuatorType::Muscle => {
                // Hill-type muscle model
                // For now, we approximate with a pneumatic cylinder since it has similar
                // activation dynamics. A full implementation would use sim-muscle crate.
                let (act_tau, deact_tau) = mjcf.muscle_timeconst;
                let max_force = if mjcf.force > 0.0 {
                    mjcf.force * mjcf.gear
                } else {
                    mjcf.scale * mjcf.gear // Use scale as fallback
                };

                // Use pneumatic cylinder as approximation with muscle-like dynamics
                // Area chosen such that max force is achieved at max pressure
                let supply_pressure = 600_000.0;
                let area = max_force / (supply_pressure - 101_325.0);

                PneumaticCylinderActuator::new(area.max(0.0001), supply_pressure)
                    .with_name(&mjcf.name)
                    .with_rates(
                        1.0 / act_tau.max(0.001),   // Activation rate
                        1.0 / deact_tau.max(0.001), // Deactivation rate
                        0.01,
                    )
                    .with_friction(0.0, 0.0) // Muscles have no mechanical friction
                    .boxed()
            }

            MjcfActuatorType::Damper => {
                // Passive damper - create zero-force actuator
                // The damping is typically applied via joint damping, not through actuator
                IntegratedVelocityActuator::new(0.0, 0.0)
                    .with_name(&mjcf.name)
                    .with_enabled(false)
                    .boxed()
            }

            MjcfActuatorType::Adhesion => {
                // Adhesion actuator for gripping/climbing
                let max_adhesion = mjcf.gain * mjcf.gear;
                AdhesionActuator::new(max_adhesion)
                    .with_name(&mjcf.name)
                    .boxed()
            }
        };

        LoadedActuator {
            name: mjcf.name.clone(),
            actuator,
            joint: mjcf.joint.clone(),
            body: mjcf.body.clone(),
            tendon: mjcf.tendon.clone(),
            ctrlrange: mjcf.ctrlrange,
            forcerange: mjcf.forcerange,
            gear: mjcf.gear,
        }
    }

    /// Create a sim-core Joint from an MJCF joint.
    fn create_joint(
        &self,
        id: JointId,
        mjcf_joint: &MjcfJoint,
        parent: BodyId,
        child: BodyId,
    ) -> Joint {
        let joint_type = match mjcf_joint.joint_type {
            MjcfJointType::Hinge => JointType::Revolute,
            MjcfJointType::Slide => JointType::Prismatic,
            MjcfJointType::Ball => JointType::Spherical,
            MjcfJointType::Free => JointType::Free,
            MjcfJointType::Cylindrical => JointType::Cylindrical,
            MjcfJointType::Planar => JointType::Planar,
        };

        let mut joint = Joint::new(id, joint_type, parent, child)
            .with_name(&mjcf_joint.name)
            .with_axis(mjcf_joint.axis);

        // Set anchors
        // In MJCF, the joint pos is relative to the body frame
        let child_anchor = Point3::from(mjcf_joint.pos);
        joint = joint.with_anchors(Point3::origin(), child_anchor);

        // Add limits if enabled
        if mjcf_joint.limited {
            if let Some((lower, upper)) = mjcf_joint.range {
                let limits = JointLimits::new(lower, upper);
                joint = joint.with_limits(limits);
            }
        }

        // Add damping
        if mjcf_joint.damping > 0.0 {
            joint = joint.with_damping(mjcf_joint.damping);
        }

        // Add spring stiffness (as motor position control)
        // Note: Full spring implementation would need motor support
        if mjcf_joint.stiffness > 0.0 {
            // For now, just store in damping as an approximation
            // A proper implementation would add spring force in the solver
        }

        joint
    }
}

/// Build a lookup table from body name to body reference.
fn build_body_lookup(worldbody: &MjcfBody) -> HashMap<&str, &MjcfBody> {
    let mut lookup = HashMap::new();

    fn traverse<'a>(body: &'a MjcfBody, lookup: &mut HashMap<&'a str, &'a MjcfBody>) {
        lookup.insert(body.name.as_str(), body);
        for child in &body.children {
            traverse(child, lookup);
        }
    }

    for child in &worldbody.children {
        traverse(child, &mut lookup);
    }

    lookup
}

/// Build a lookup table from site name to (body_name, local_position).
///
/// This collects all sites from the body tree, recording which body each
/// site belongs to. The body names can later be resolved to BodyIds.
fn build_site_lookup(worldbody: &MjcfBody) -> HashMap<String, (String, Point3<f64>)> {
    let mut lookup = HashMap::new();

    fn traverse(body: &MjcfBody, lookup: &mut HashMap<String, (String, Point3<f64>)>) {
        // Collect sites from this body
        for site in &body.sites {
            if !site.name.is_empty() {
                let local_pos = Point3::new(site.pos.x, site.pos.y, site.pos.z);
                lookup.insert(site.name.clone(), (body.name.clone(), local_pos));
            }
        }

        // Recurse into children
        for child in &body.children {
            traverse(child, lookup);
        }
    }

    // Also check worldbody sites (sites attached to world/ground)
    for site in &worldbody.sites {
        if !site.name.is_empty() {
            let local_pos = Point3::new(site.pos.x, site.pos.y, site.pos.z);
            // Use empty string to indicate world attachment
            lookup.insert(site.name.clone(), (String::new(), local_pos));
        }
    }

    for child in &worldbody.children {
        traverse(child, &mut lookup);
    }

    lookup
}

/// Resolve site lookup (with body names) into SiteInfo (with BodyIds).
///
/// World-attached sites (empty body name) use BodyId(0) to represent the world.
fn resolve_site_lookup(
    site_lookup: &HashMap<String, (String, Point3<f64>)>,
    body_to_id: &HashMap<String, BodyId>,
) -> HashMap<String, SiteInfo> {
    let mut resolved = HashMap::new();

    for (site_name, (body_name, local_pos)) in site_lookup {
        let body_id = if body_name.is_empty() {
            // World-attached site
            BodyId::new(0)
        } else if let Some(&id) = body_to_id.get(body_name) {
            id
        } else {
            // Body not found - skip this site (shouldn't happen with valid MJCF)
            continue;
        };

        resolved.insert(
            site_name.clone(),
            SiteInfo {
                body_id,
                local_position: *local_pos,
            },
        );
    }

    resolved
}

/// Intermediate geom info before BodyId resolution.
///
/// Stores the body name (string) and geom data so we can resolve to BodyId later.
#[derive(Debug, Clone)]
struct GeomLookupEntry {
    body_name: String,
    local_position: Point3<f64>,
    local_orientation: nalgebra::UnitQuaternion<f64>,
    geom_type: MjcfGeomType,
    size: Vec<f64>,
}

/// Build a lookup table from geom name to geom information.
///
/// This collects all named geoms from the body tree, recording which body each
/// geom belongs to and its local pose. Only geoms that can be used for wrapping
/// (sphere, cylinder) are included.
fn build_geom_lookup(worldbody: &MjcfBody) -> HashMap<String, GeomLookupEntry> {
    let mut lookup = HashMap::new();

    fn traverse(body: &MjcfBody, lookup: &mut HashMap<String, GeomLookupEntry>) {
        // Collect named geoms from this body that can serve as wrapping surfaces
        for geom in &body.geoms {
            if let Some(ref name) = geom.name {
                if !name.is_empty() {
                    // Only sphere and cylinder can be used for wrapping
                    if matches!(
                        geom.geom_type,
                        MjcfGeomType::Sphere | MjcfGeomType::Cylinder
                    ) {
                        let local_pos = Point3::new(geom.pos.x, geom.pos.y, geom.pos.z);
                        let local_rot = geom.rotation();
                        lookup.insert(
                            name.clone(),
                            GeomLookupEntry {
                                body_name: body.name.clone(),
                                local_position: local_pos,
                                local_orientation: local_rot,
                                geom_type: geom.geom_type,
                                size: geom.size.clone(),
                            },
                        );
                    }
                }
            }
        }

        // Recurse into children
        for child in &body.children {
            traverse(child, lookup);
        }
    }

    // Check worldbody geoms (geoms attached to world/ground)
    for geom in &worldbody.geoms {
        if let Some(ref name) = geom.name {
            if !name.is_empty()
                && matches!(
                    geom.geom_type,
                    MjcfGeomType::Sphere | MjcfGeomType::Cylinder
                )
            {
                let local_pos = Point3::new(geom.pos.x, geom.pos.y, geom.pos.z);
                let local_rot = geom.rotation();
                // Use empty string to indicate world attachment
                lookup.insert(
                    name.clone(),
                    GeomLookupEntry {
                        body_name: String::new(),
                        local_position: local_pos,
                        local_orientation: local_rot,
                        geom_type: geom.geom_type,
                        size: geom.size.clone(),
                    },
                );
            }
        }
    }

    for child in &worldbody.children {
        traverse(child, &mut lookup);
    }

    lookup
}

/// Resolve geom lookup (with body names) into GeomInfo (with BodyIds).
///
/// Also converts the geom shape data into sim-tendon WrappingGeometry types.
/// World-attached geoms (empty body name) use BodyId(0) to represent the world.
fn resolve_geom_lookup(
    geom_lookup: &HashMap<String, GeomLookupEntry>,
    body_to_id: &HashMap<String, BodyId>,
) -> HashMap<String, GeomInfo> {
    let mut resolved = HashMap::new();

    for (geom_name, entry) in geom_lookup {
        let body_id = if entry.body_name.is_empty() {
            // World-attached geom
            BodyId::new(0)
        } else if let Some(&id) = body_to_id.get(&entry.body_name) {
            id
        } else {
            // Body not found - skip this geom (shouldn't happen with valid MJCF)
            continue;
        };

        // Convert to WrappingGeometry based on geom type
        let wrapping_geometry = match entry.geom_type {
            MjcfGeomType::Sphere => {
                // Sphere: size[0] is radius
                let radius = entry.size.first().copied().unwrap_or(0.1);
                Some(WrappingGeometry::Sphere(SphereWrap::new(
                    entry.local_position,
                    radius,
                )))
            }
            MjcfGeomType::Cylinder => {
                // Cylinder: size[0] is radius, size[1] is half-length
                let radius = entry.size.first().copied().unwrap_or(0.1);
                let half_height = entry.size.get(1).copied().unwrap_or(0.1);
                // Compute axis from orientation (cylinder is Z-aligned in local frame)
                let axis = entry.local_orientation * Vector3::z();
                Some(WrappingGeometry::Cylinder(CylinderWrap::new(
                    entry.local_position,
                    axis,
                    radius,
                    half_height,
                )))
            }
            _ => None, // Other geom types can't be used for wrapping
        };

        resolved.insert(
            geom_name.clone(),
            GeomInfo {
                body_id,
                local_position: entry.local_position,
                local_orientation: entry.local_orientation,
                wrapping_geometry,
            },
        );
    }

    resolved
}

/// Compute a rotation that aligns the local Z axis with the given direction.
fn rotation_from_z_axis(z_axis: &Vector3<f64>) -> nalgebra::UnitQuaternion<f64> {
    let z_unit = Vector3::z();

    // Check if already aligned
    let dot = z_unit.dot(z_axis);
    if dot > 0.9999 {
        return nalgebra::UnitQuaternion::identity();
    }
    if dot < -0.9999 {
        // Opposite direction, rotate 180 degrees around X
        return nalgebra::UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::x()),
            std::f64::consts::PI,
        );
    }

    // General case: rotate from z_unit to z_axis
    let axis = z_unit.cross(z_axis);
    let angle = dot.acos();
    nalgebra::UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(axis), angle)
}

/// Compute world pose for a body by traversing parent chain.
fn compute_world_pose(
    body_name: &str,
    validation: &ValidationResult,
    body_lookup: &HashMap<&str, &MjcfBody>,
) -> Pose {
    // Build chain from root to this body
    let mut chain = vec![body_name.to_string()];
    let mut current = body_name.to_string();

    while let Some(parent_name) = validation.body_parent.get(&current) {
        chain.push(parent_name.clone());
        current = parent_name.clone();
    }

    // Reverse to go from root to body
    chain.reverse();

    // Compose poses
    let mut world_pose = Pose::identity();

    for name in chain {
        if let Some(body) = body_lookup.get(name.as_str()) {
            let local_pose = Pose::from_position_rotation(body.position(), body.rotation());
            world_pose = world_pose.compose(&local_pose);
        }
    }

    world_pose
}

/// Convenience function to load an MJCF file with default settings.
///
/// # Errors
///
/// Returns an error if the file cannot be read or parsed.
pub fn load_mjcf_file(path: impl AsRef<Path>) -> Result<LoadedModel> {
    MjcfLoader::default().load_file(path)
}

/// Convenience function to load an MJCF string with default settings.
///
/// # Errors
///
/// Returns an error if parsing fails.
pub fn load_mjcf_str(xml: &str) -> Result<LoadedModel> {
    MjcfLoader::default().load_str(xml)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    const SIMPLE_ARM_MJCF: &str = r#"
        <mujoco model="simple_arm">
            <option gravity="0 0 -9.81"/>
            <worldbody>
                <body name="base_link" pos="0 0 0.1">
                    <inertial pos="0 0 0" mass="10.0" diaginertia="1 1 1"/>
                    <geom type="box" size="0.1 0.1 0.05"/>
                    <body name="upper_arm" pos="0 0 0.05">
                        <joint name="shoulder" type="hinge" axis="0 1 0" limited="true" range="-1.57 1.57" damping="0.1"/>
                        <inertial pos="0 0 0.25" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <geom type="capsule" size="0.05" fromto="0 0 0 0 0 0.5"/>
                        <body name="lower_arm" pos="0 0 0.5">
                            <joint name="elbow" type="hinge" axis="0 1 0" limited="true" range="-2.0 2.0"/>
                            <inertial pos="0 0 0.2" mass="0.5" diaginertia="0.05 0.05 0.05"/>
                            <geom type="sphere" size="0.1"/>
                        </body>
                    </body>
                </body>
            </worldbody>
        </mujoco>
    "#;

    #[test]
    fn test_load_simple_arm() {
        let model = load_mjcf_str(SIMPLE_ARM_MJCF).expect("should load");

        assert_eq!(model.name, "simple_arm");
        assert_eq!(model.bodies.len(), 3);
        assert_eq!(model.joints.len(), 2);

        // Check body names
        assert!(model.body_to_id.contains_key("base_link"));
        assert!(model.body_to_id.contains_key("upper_arm"));
        assert!(model.body_to_id.contains_key("lower_arm"));

        // Check joint names
        assert!(model.joint_to_id.contains_key("shoulder"));
        assert!(model.joint_to_id.contains_key("elbow"));
    }

    #[test]
    fn test_mass_properties() {
        let model = load_mjcf_str(SIMPLE_ARM_MJCF).expect("should load");

        let base_id = model.body_to_id["base_link"];
        let base = model.bodies.iter().find(|b| b.id == base_id).expect("base");

        assert_relative_eq!(base.mass_props.mass, 10.0, epsilon = 1e-10);
    }

    #[test]
    fn test_collision_shapes() {
        let model = load_mjcf_str(SIMPLE_ARM_MJCF).expect("should load");

        // Base has box collision
        let base_id = model.body_to_id["base_link"];
        let base = model.bodies.iter().find(|b| b.id == base_id).expect("base");
        assert!(matches!(
            base.collision_shape,
            Some(CollisionShape::Box { .. })
        ));

        // Lower arm has sphere collision
        let lower_id = model.body_to_id["lower_arm"];
        let lower = model
            .bodies
            .iter()
            .find(|b| b.id == lower_id)
            .expect("lower");
        assert!(matches!(
            lower.collision_shape,
            Some(CollisionShape::Sphere { .. })
        ));
    }

    #[test]
    fn test_joint_properties() {
        let model = load_mjcf_str(SIMPLE_ARM_MJCF).expect("should load");

        let shoulder_id = model.joint_to_id["shoulder"];
        let shoulder = model
            .joints
            .iter()
            .find(|j| j.id == shoulder_id)
            .expect("shoulder");

        assert_eq!(shoulder.joint_type, JointType::Revolute);
        assert!(shoulder.limits.is_some());

        let limits = shoulder.limits.as_ref().expect("limits");
        assert_relative_eq!(limits.lower(), -1.57, epsilon = 1e-10);
        assert_relative_eq!(limits.upper(), 1.57, epsilon = 1e-10);

        assert_relative_eq!(shoulder.damping, 0.1, epsilon = 1e-10);
    }

    #[test]
    fn test_spawn_into_world() {
        let model = load_mjcf_str(SIMPLE_ARM_MJCF).expect("should load");

        let mut world = World::default();
        let spawned = model.spawn_at_origin(&mut world).expect("should spawn");

        assert_eq!(world.body_count(), 3);
        assert_eq!(world.joint_count(), 2);

        assert!(spawned.body_id("base_link").is_some());
        assert!(spawned.joint_id("shoulder").is_some());
    }

    #[test]
    fn test_spawn_with_base_pose() {
        let model = load_mjcf_str(SIMPLE_ARM_MJCF).expect("should load");

        let mut world = World::default();
        let base_pose = Pose::from_position(Point3::new(1.0, 2.0, 3.0));
        let spawned = model
            .spawn_into(&mut world, base_pose)
            .expect("should spawn");

        let base_id = spawned.body_id("base_link").expect("base_link");
        let base = world.body(base_id).expect("body");

        // Base pose should be offset by (1, 2, 3) from its original position
        assert_relative_eq!(base.state.pose.position.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(base.state.pose.position.y, 2.0, epsilon = 1e-10);
        // Original z was 0.1, plus offset of 3
        assert_relative_eq!(base.state.pose.position.z, 3.1, epsilon = 1e-10);
    }

    #[test]
    fn test_loader_without_collision() {
        let loader = MjcfLoader::new().with_collision_shapes(false);
        let model = loader.load_str(SIMPLE_ARM_MJCF).expect("should load");

        for body in &model.bodies {
            assert!(body.collision_shape.is_none());
        }
    }

    #[test]
    fn test_free_joint() {
        let xml = r#"
            <mujoco model="freebody">
                <worldbody>
                    <body name="floating" pos="0 0 1">
                        <joint type="free"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");
        let joint = &model.joints[0];

        assert_eq!(joint.joint_type, JointType::Free);
    }

    #[test]
    fn test_ball_joint() {
        let xml = r#"
            <mujoco model="ballsocket">
                <worldbody>
                    <body name="base">
                        <geom type="box" size="0.1 0.1 0.1"/>
                        <body name="pendulum" pos="0 0 0.2">
                            <joint name="ball" type="ball"/>
                            <geom type="sphere" size="0.05" mass="0.5"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");
        let ball_id = model.joint_to_id["ball"];
        let ball = model.joints.iter().find(|j| j.id == ball_id).expect("ball");

        assert_eq!(ball.joint_type, JointType::Spherical);
    }

    #[test]
    fn test_slide_joint() {
        let xml = r#"
            <mujoco model="slider">
                <worldbody>
                    <body name="base">
                        <geom type="box" size="0.1 0.1 0.1"/>
                        <body name="slider" pos="0.2 0 0">
                            <joint name="slide" type="slide" axis="1 0 0"/>
                            <geom type="box" size="0.05 0.05 0.05" mass="0.5"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");
        let slide_id = model.joint_to_id["slide"];
        let slide = model
            .joints
            .iter()
            .find(|j| j.id == slide_id)
            .expect("slide");

        assert_eq!(slide.joint_type, JointType::Prismatic);
    }

    #[test]
    fn test_defaults_applied_to_joints() {
        // Test that default classes are properly applied to joints
        let xml = r#"
            <mujoco model="defaults_test">
                <default>
                    <joint damping="0.5"/>
                    <default class="stiff">
                        <joint damping="2.0" stiffness="100.0"/>
                    </default>
                </default>
                <worldbody>
                    <body name="base">
                        <geom type="box" size="0.1 0.1 0.1"/>
                        <body name="link1" pos="0 0 0.2">
                            <!-- This joint uses root defaults (damping=0.5) -->
                            <joint name="j1" type="hinge"/>
                            <geom type="sphere" size="0.05"/>
                            <body name="link2" pos="0 0 0.2">
                                <!-- This joint uses "stiff" class (damping=2.0, stiffness=100.0) -->
                                <joint name="j2" type="hinge" class="stiff"/>
                                <geom type="sphere" size="0.05"/>
                            </body>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");

        let j1_id = model.joint_to_id["j1"];
        let j1 = model.joints.iter().find(|j| j.id == j1_id).expect("j1");
        // j1 has no class, so it should get root defaults applied
        assert_relative_eq!(j1.damping, 0.5, epsilon = 1e-10);

        let j2_id = model.joint_to_id["j2"];
        let j2 = model.joints.iter().find(|j| j.id == j2_id).expect("j2");
        // j2 uses "stiff" class, so it should get damping=2.0
        assert_relative_eq!(j2.damping, 2.0, epsilon = 1e-10);
    }

    #[test]
    fn test_defaults_model_with_actuators_tendons_sensors() {
        // Test that a model with defaults for actuators, tendons, and sensors parses correctly
        let xml = r#"
            <mujoco model="full_defaults">
                <default>
                    <joint damping="1.0" armature="0.1"/>
                    <geom density="2000"/>
                    <motor gear="50" ctrlrange="-1 1"/>
                    <tendon stiffness="500" damping="5"/>
                    <sensor noise="0.01"/>
                </default>
                <worldbody>
                    <body name="arm">
                        <joint name="shoulder" type="hinge"/>
                        <geom type="capsule" size="0.05" fromto="0 0 0 0 0 0.5"/>
                    </body>
                </worldbody>
                <actuator>
                    <motor name="shoulder_motor" joint="shoulder"/>
                </actuator>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");
        assert_eq!(model.name, "full_defaults");
        assert_eq!(model.bodies.len(), 1);
        assert_eq!(model.joints.len(), 1);

        // Check that joint defaults were applied
        let shoulder = &model.joints[0];
        assert_relative_eq!(shoulder.damping, 1.0, epsilon = 1e-10);

        // Check that actuator defaults were applied
        assert_eq!(model.actuators.len(), 1);
        let actuator = &model.actuators[0];
        assert_eq!(actuator.name, "shoulder_motor");
        assert_relative_eq!(actuator.gear, 50.0, epsilon = 1e-10);
        assert_eq!(actuator.ctrlrange, Some((-1.0, 1.0)));
    }

    #[test]
    fn test_actuator_defaults_applied() {
        // Test that actuator defaults are correctly applied to actuators
        let xml = r#"
            <mujoco model="actuator_defaults">
                <default>
                    <motor gear="100" ctrlrange="-2 2" forcerange="-50 50"/>
                    <default class="weak">
                        <motor gear="10" ctrlrange="-0.5 0.5"/>
                    </default>
                </default>
                <worldbody>
                    <body name="link">
                        <joint name="j1" type="hinge"/>
                        <geom type="capsule" size="0.05" fromto="0 0 0 0 0 0.5"/>
                    </body>
                </worldbody>
                <actuator>
                    <motor name="strong_motor" joint="j1"/>
                    <motor name="weak_motor" joint="j1" class="weak"/>
                    <motor name="override_motor" joint="j1" gear="200"/>
                </actuator>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");
        assert_eq!(model.actuators.len(), 3);

        // First actuator uses root defaults
        let strong = &model.actuators[0];
        assert_eq!(strong.name, "strong_motor");
        assert_relative_eq!(strong.gear, 100.0, epsilon = 1e-10);
        assert_eq!(strong.ctrlrange, Some((-2.0, 2.0)));
        assert_eq!(strong.forcerange, Some((-50.0, 50.0)));

        // Second actuator uses "weak" class defaults
        let weak = &model.actuators[1];
        assert_eq!(weak.name, "weak_motor");
        assert_relative_eq!(weak.gear, 10.0, epsilon = 1e-10);
        assert_eq!(weak.ctrlrange, Some((-0.5, 0.5)));
        // forcerange should be inherited from root
        assert_eq!(weak.forcerange, Some((-50.0, 50.0)));

        // Third actuator overrides gear explicitly
        let override_actuator = &model.actuators[2];
        assert_eq!(override_actuator.name, "override_motor");
        assert_relative_eq!(override_actuator.gear, 200.0, epsilon = 1e-10);
        // Other values should come from root defaults
        assert_eq!(override_actuator.ctrlrange, Some((-2.0, 2.0)));
    }

    // ========================================================================
    // Mesh geom collision tests
    // ========================================================================

    #[test]
    fn test_mesh_geom_with_embedded_vertices() {
        // A simple tetrahedron with embedded vertex data
        let xml = r#"
            <mujoco model="mesh_test">
                <asset>
                    <mesh name="tetra" vertex="0 0 0 1 0 0 0.5 0.866 0 0.5 0.289 0.816"/>
                </asset>
                <worldbody>
                    <body name="mesh_body">
                        <geom type="mesh" mesh="tetra"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");
        assert_eq!(model.bodies.len(), 1);

        let body = &model.bodies[0];
        assert!(body.collision_shape.is_some());

        // Check that it's a ConvexMesh shape
        match &body.collision_shape {
            Some(CollisionShape::ConvexMesh { vertices }) => {
                assert_eq!(vertices.len(), 4);
                // First vertex should be at origin
                assert_relative_eq!(vertices[0].x, 0.0, epsilon = 1e-10);
                assert_relative_eq!(vertices[0].y, 0.0, epsilon = 1e-10);
                assert_relative_eq!(vertices[0].z, 0.0, epsilon = 1e-10);
            }
            _ => panic!("Expected ConvexMesh shape"),
        }
    }

    #[test]
    fn test_mesh_geom_with_scale() {
        // Mesh with scale factor applied
        let xml = r#"
            <mujoco model="scaled_mesh">
                <asset>
                    <mesh name="cube" vertex="0 0 0 1 0 0 0 1 0 0 0 1" scale="0.1 0.1 0.1"/>
                </asset>
                <worldbody>
                    <body name="scaled_body">
                        <geom type="mesh" mesh="cube"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");
        let body = &model.bodies[0];

        match &body.collision_shape {
            Some(CollisionShape::ConvexMesh { vertices }) => {
                assert_eq!(vertices.len(), 4);
                // Second vertex should be at (0.1, 0, 0) due to scale
                assert_relative_eq!(vertices[1].x, 0.1, epsilon = 1e-10);
                assert_relative_eq!(vertices[1].y, 0.0, epsilon = 1e-10);
                assert_relative_eq!(vertices[1].z, 0.0, epsilon = 1e-10);
            }
            _ => panic!("Expected ConvexMesh shape"),
        }
    }

    #[test]
    fn test_mesh_geom_missing_asset() {
        // Reference to a mesh asset that doesn't exist
        let xml = r#"
            <mujoco model="missing_mesh">
                <worldbody>
                    <body name="body">
                        <geom type="mesh" mesh="nonexistent"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;

        // Should still load, but without collision shape
        let model = load_mjcf_str(xml).expect("should load");
        let body = &model.bodies[0];

        // The collision shape should be None because the mesh wasn't found
        assert!(body.collision_shape.is_none());
    }

    #[test]
    fn test_mesh_geom_among_other_geoms() {
        // Body with multiple geoms including a mesh
        let xml = r#"
            <mujoco model="multi_geom">
                <asset>
                    <mesh name="custom" vertex="0 0 0 1 0 0 0 1 0"/>
                </asset>
                <worldbody>
                    <body name="composite">
                        <geom type="mesh" mesh="custom"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");
        let body = &model.bodies[0];

        // First geom (mesh) should be used for collision
        match &body.collision_shape {
            Some(CollisionShape::ConvexMesh { vertices }) => {
                assert_eq!(vertices.len(), 3);
            }
            _ => panic!("Expected ConvexMesh shape from first geom"),
        }
    }

    #[test]
    fn test_mesh_type_enum() {
        use crate::types::MjcfGeomType;

        assert_eq!(MjcfGeomType::from_str("mesh"), Some(MjcfGeomType::Mesh));
        assert_eq!(MjcfGeomType::Mesh.as_str(), "mesh");
    }

    #[test]
    fn test_trimesh_type_enum() {
        use crate::types::MjcfGeomType;

        // Test all aliases for TriangleMesh
        assert_eq!(
            MjcfGeomType::from_str("trimesh"),
            Some(MjcfGeomType::TriangleMesh)
        );
        assert_eq!(
            MjcfGeomType::from_str("triangle_mesh"),
            Some(MjcfGeomType::TriangleMesh)
        );
        assert_eq!(
            MjcfGeomType::from_str("nonconvex"),
            Some(MjcfGeomType::TriangleMesh)
        );
        assert_eq!(MjcfGeomType::TriangleMesh.as_str(), "trimesh");
    }

    #[test]
    fn test_triangle_mesh_geom_with_embedded_vertices() {
        // A simple floor with embedded vertex and face data
        let xml = r#"
            <mujoco model="trimesh_test">
                <asset>
                    <mesh name="floor" vertex="0 0 0 1 0 0 1 1 0 0 1 0" face="0 1 2 0 2 3"/>
                </asset>
                <worldbody>
                    <body name="floor_body">
                        <geom type="trimesh" mesh="floor"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");
        assert_eq!(model.bodies.len(), 1);

        let body = &model.bodies[0];
        assert!(body.collision_shape.is_some());

        // Check that it's a TriangleMesh shape (not ConvexMesh)
        match &body.collision_shape {
            Some(CollisionShape::TriangleMesh { data }) => {
                assert_eq!(data.vertex_count(), 4);
                assert_eq!(data.triangle_count(), 2);
            }
            other => panic!("Expected TriangleMesh shape, got {other:?}"),
        }
    }

    #[test]
    fn test_mesh_vertices_as_points() {
        use crate::types::MjcfMesh;

        let mesh = MjcfMesh {
            name: "test".to_string(),
            file: None,
            scale: Vector3::new(2.0, 2.0, 2.0),
            vertex: Some(vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]),
            face: None,
        };

        let points = mesh.vertices_as_points();
        assert_eq!(points.len(), 3);

        // Vertices should be scaled
        assert_relative_eq!(points[0].x, 2.0, epsilon = 1e-10);
        assert_relative_eq!(points[1].y, 2.0, epsilon = 1e-10);
        assert_relative_eq!(points[2].z, 2.0, epsilon = 1e-10);
    }

    #[test]
    fn test_loader_with_mesh_base_dir() {
        let loader = MjcfLoader::new().with_mesh_base_dir("/some/path");
        assert_eq!(
            loader.mesh_base_dir,
            Some(std::path::PathBuf::from("/some/path"))
        );
    }

    // ========================================================================
    // Connect constraint tests
    // ========================================================================

    #[test]
    fn test_connect_constraint_between_bodies() {
        let xml = r#"
            <mujoco model="connect_test">
                <worldbody>
                    <body name="body1" pos="0 0 0">
                        <geom type="sphere" size="0.1"/>
                    </body>
                    <body name="body2" pos="1 0 0">
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <equality>
                    <connect name="ball" body1="body1" body2="body2" anchor="0.5 0 0"/>
                </equality>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");

        assert_eq!(model.connect_constraints.len(), 1);

        let constraint = &model.connect_constraints[0];
        assert_eq!(constraint.name(), "ball");
        assert_eq!(constraint.body1(), model.body_to_id["body1"]);
        assert_eq!(constraint.body2(), Some(model.body_to_id["body2"]));
        assert_relative_eq!(constraint.anchor().x, 0.5, epsilon = 1e-10);
    }

    #[test]
    fn test_connect_constraint_to_world() {
        let xml = r#"
            <mujoco model="connect_world">
                <worldbody>
                    <body name="floating">
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <equality>
                    <connect body1="floating" anchor="0 0 1"/>
                </equality>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");

        assert_eq!(model.connect_constraints.len(), 1);

        let constraint = &model.connect_constraints[0];
        assert!(constraint.is_world_constraint());
        assert!(constraint.body2().is_none());
    }

    #[test]
    fn test_connect_constraint_with_solver_params() {
        let xml = r#"
            <mujoco model="connect_params">
                <worldbody>
                    <body name="a">
                        <geom type="sphere" size="0.1"/>
                    </body>
                    <body name="b">
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <equality>
                    <connect body1="a" body2="b"
                             solref="0.02 1"
                             solimp="0.9 0.95 0.001 0.5 2"/>
                </equality>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");
        let constraint = &model.connect_constraints[0];

        let solref = constraint.solref().expect("should have solref");
        assert_relative_eq!(solref[0], 0.02, epsilon = 1e-10);

        let solimp = constraint.solimp().expect("should have solimp");
        assert_relative_eq!(solimp[0], 0.9, epsilon = 1e-10);
    }

    #[test]
    fn test_connect_constraint_inactive_skipped() {
        let xml = r#"
            <mujoco model="connect_inactive">
                <worldbody>
                    <body name="a">
                        <geom type="sphere" size="0.1"/>
                    </body>
                    <body name="b">
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <equality>
                    <connect body1="a" body2="b" active="false"/>
                    <connect body1="a" body2="b" active="true"/>
                </equality>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");

        // Only the active constraint should be included
        assert_eq!(model.connect_constraints.len(), 1);
    }

    #[test]
    fn test_connect_constraint_undefined_body() {
        let xml = r#"
            <mujoco model="connect_bad">
                <worldbody>
                    <body name="real_body"/>
                </worldbody>
                <equality>
                    <connect body1="nonexistent" body2="real_body"/>
                </equality>
            </mujoco>
        "#;

        let result = load_mjcf_str(xml);
        assert!(result.is_err());

        let err = result.unwrap_err();
        assert!(matches!(err, MjcfError::UndefinedBody { .. }));
    }

    #[test]
    fn test_multiple_connect_constraints() {
        let xml = r#"
            <mujoco model="multi_connect">
                <worldbody>
                    <body name="a">
                        <geom type="sphere" size="0.1"/>
                    </body>
                    <body name="b">
                        <geom type="sphere" size="0.1"/>
                    </body>
                    <body name="c">
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <equality>
                    <connect name="c1" body1="a" body2="b"/>
                    <connect name="c2" body1="b" body2="c"/>
                    <connect name="c3" body1="c" anchor="0 0 0"/>
                </equality>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");
        assert_eq!(model.connect_constraints.len(), 3);

        // Check names
        assert_eq!(model.connect_constraints[0].name(), "c1");
        assert_eq!(model.connect_constraints[1].name(), "c2");
        assert_eq!(model.connect_constraints[2].name(), "c3");

        // Third constraint should be to world
        assert!(model.connect_constraints[2].is_world_constraint());
    }

    // ========================================================================
    // Tendon tests
    // ========================================================================

    #[test]
    fn test_fixed_tendon_basic() {
        let xml = r#"
            <mujoco model="tendon_test">
                <worldbody>
                    <body name="arm">
                        <joint name="shoulder" type="hinge"/>
                        <geom type="capsule" size="0.05" fromto="0 0 0 0 0 0.5"/>
                        <body name="forearm" pos="0 0 0.5">
                            <joint name="elbow" type="hinge"/>
                            <geom type="capsule" size="0.04" fromto="0 0 0 0 0 0.4"/>
                        </body>
                    </body>
                </worldbody>
                <tendon>
                    <fixed name="biceps">
                        <joint joint="shoulder" coef="0.02"/>
                        <joint joint="elbow" coef="0.015"/>
                    </fixed>
                </tendon>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");

        assert_eq!(model.tendons.len(), 1);
        assert_eq!(model.tendon_to_index.len(), 1);

        let tendon = &model.tendons[0];
        assert_eq!(tendon.name(), "biceps");
        assert!(tendon.is_fixed());

        // Check the underlying constraint
        let fixed = tendon.as_fixed().expect("should be fixed tendon");
        let constraint = &fixed.constraint;
        assert_eq!(constraint.joints().len(), 2);

        // First joint should be shoulder
        let shoulder_id = model.joint_to_id["shoulder"];
        let elbow_id = model.joint_to_id["elbow"];

        // Find the joint entries (order may vary)
        let shoulder_entry = constraint
            .joints()
            .iter()
            .find(|(id, _)| *id == shoulder_id);
        let elbow_entry = constraint.joints().iter().find(|(id, _)| *id == elbow_id);

        assert!(shoulder_entry.is_some());
        assert!(elbow_entry.is_some());

        assert_relative_eq!(shoulder_entry.unwrap().1, 0.02, epsilon = 1e-10);
        assert_relative_eq!(elbow_entry.unwrap().1, 0.015, epsilon = 1e-10);
    }

    #[test]
    fn test_fixed_tendon_with_properties() {
        let xml = r#"
            <mujoco model="tendon_props">
                <worldbody>
                    <body name="link">
                        <joint name="j1" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <tendon>
                    <fixed name="spring_tendon" stiffness="1000" damping="50" limited="true" range="-0.1 0.2">
                        <joint joint="j1" coef="0.01"/>
                    </fixed>
                </tendon>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");

        let tendon = &model.tendons[0];
        assert_eq!(tendon.name(), "spring_tendon");
        assert!(tendon.limited());
        assert_eq!(tendon.range(), Some((-0.1, 0.2)));

        // Check constraint properties
        let fixed = tendon.as_fixed().expect("should be fixed tendon");
        assert_relative_eq!(fixed.constraint.stiffness(), 1000.0, epsilon = 1e-10);
    }

    #[test]
    fn test_tendon_lookup_by_name() {
        let xml = r#"
            <mujoco model="tendon_lookup">
                <worldbody>
                    <body name="link">
                        <joint name="j1" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <tendon>
                    <fixed name="tendon_a">
                        <joint joint="j1" coef="0.01"/>
                    </fixed>
                    <fixed name="tendon_b">
                        <joint joint="j1" coef="0.02"/>
                    </fixed>
                </tendon>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");

        // Test lookup by name
        assert!(model.tendon("tendon_a").is_some());
        assert!(model.tendon("tendon_b").is_some());
        assert!(model.tendon("nonexistent").is_none());

        // Check values
        let tendon_a = model.tendon("tendon_a").unwrap();
        assert_eq!(tendon_a.name(), "tendon_a");

        let tendon_b = model.tendon("tendon_b").unwrap();
        assert_eq!(tendon_b.name(), "tendon_b");

        assert_eq!(model.tendon_count(), 2);
    }

    #[test]
    fn test_fixed_tendon_undefined_joint() {
        let xml = r#"
            <mujoco model="bad_tendon">
                <worldbody>
                    <body name="link">
                        <joint name="j1" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <tendon>
                    <fixed name="bad">
                        <joint joint="nonexistent" coef="0.01"/>
                    </fixed>
                </tendon>
            </mujoco>
        "#;

        let result = load_mjcf_str(xml);
        assert!(result.is_err());

        let err = result.unwrap_err();
        assert!(matches!(err, MjcfError::UndefinedJoint { .. }));
    }

    #[test]
    fn test_spawned_model_tendon_access() {
        let xml = r#"
            <mujoco model="spawn_tendon">
                <worldbody>
                    <body name="link">
                        <joint name="j1" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <tendon>
                    <fixed name="my_tendon">
                        <joint joint="j1" coef="0.01"/>
                    </fixed>
                </tendon>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");

        let mut world = World::default();
        let spawned = model.spawn_at_origin(&mut world).expect("should spawn");

        // Tendon should be accessible from spawned model
        assert_eq!(spawned.tendon_count(), 1);
        assert!(spawned.tendon("my_tendon").is_some());

        let tendon = spawned.tendon("my_tendon").unwrap();
        assert_eq!(tendon.name(), "my_tendon");
    }

    #[test]
    fn test_multiple_fixed_tendons() {
        let xml = r#"
            <mujoco model="multi_tendon">
                <worldbody>
                    <body name="upper">
                        <joint name="shoulder" type="hinge"/>
                        <geom type="capsule" size="0.05" fromto="0 0 0 0 0 0.3"/>
                        <body name="lower" pos="0 0 0.3">
                            <joint name="elbow" type="hinge"/>
                            <geom type="capsule" size="0.04" fromto="0 0 0 0 0 0.25"/>
                            <body name="hand" pos="0 0 0.25">
                                <joint name="wrist" type="hinge"/>
                                <geom type="sphere" size="0.03"/>
                            </body>
                        </body>
                    </body>
                </worldbody>
                <tendon>
                    <fixed name="flexor">
                        <joint joint="shoulder" coef="0.02"/>
                        <joint joint="elbow" coef="0.015"/>
                        <joint joint="wrist" coef="0.01"/>
                    </fixed>
                    <fixed name="extensor">
                        <joint joint="shoulder" coef="-0.02"/>
                        <joint joint="elbow" coef="-0.015"/>
                        <joint joint="wrist" coef="-0.01"/>
                    </fixed>
                </tendon>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");

        assert_eq!(model.tendons.len(), 2);

        let flexor = model.tendon("flexor").unwrap();
        let extensor = model.tendon("extensor").unwrap();

        // Flexor should have positive coefficients
        let flexor_fixed = flexor.as_fixed().expect("should be fixed");
        assert_eq!(flexor_fixed.constraint.joints().len(), 3);

        // Extensor should have negative coefficients
        let extensor_fixed = extensor.as_fixed().expect("should be fixed");
        let ext_shoulder = extensor_fixed
            .constraint
            .joints()
            .iter()
            .find(|(id, _)| *id == model.joint_to_id["shoulder"])
            .unwrap();
        assert_relative_eq!(ext_shoulder.1, -0.02, epsilon = 1e-10);
    }

    // ========================================================================
    // Spatial tendon tests
    // ========================================================================

    #[test]
    fn test_spatial_tendon_basic() {
        let xml = r#"
            <mujoco model="spatial_tendon_test">
                <worldbody>
                    <body name="upper_arm" pos="0 0 0">
                        <geom type="capsule" size="0.05" fromto="0 0 0 0 0 0.3"/>
                        <site name="bicep_origin" pos="0 0.05 0.1"/>
                        <body name="forearm" pos="0 0 0.3">
                            <joint name="elbow" type="hinge"/>
                            <geom type="capsule" size="0.04" fromto="0 0 0 0 0 0.25"/>
                            <site name="bicep_insertion" pos="0 0.03 0.05"/>
                        </body>
                    </body>
                </worldbody>
                <tendon>
                    <spatial name="bicep">
                        <site site="bicep_origin"/>
                        <site site="bicep_insertion"/>
                    </spatial>
                </tendon>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");

        assert_eq!(model.tendons.len(), 1);

        let tendon = &model.tendons[0];
        assert_eq!(tendon.name(), "bicep");
        assert!(tendon.is_spatial());

        // Verify it's a spatial tendon
        let spatial = tendon.as_spatial().expect("should be spatial tendon");
        assert_eq!(spatial.name, "bicep");

        // The underlying SpatialTendon should have a path with 2 points
        assert_eq!(spatial.tendon.path().num_points(), 2);
    }

    #[test]
    fn test_spatial_tendon_with_via_points() {
        let xml = r#"
            <mujoco model="spatial_via_points">
                <worldbody>
                    <body name="body_a" pos="0 0 0">
                        <geom type="sphere" size="0.1"/>
                        <site name="site_a" pos="0.1 0 0"/>
                    </body>
                    <body name="body_b" pos="0.5 0 0">
                        <geom type="sphere" size="0.1"/>
                        <site name="site_b" pos="0 0 0"/>
                    </body>
                    <body name="body_c" pos="1 0 0">
                        <geom type="sphere" size="0.1"/>
                        <site name="site_c" pos="-0.1 0 0"/>
                    </body>
                </worldbody>
                <tendon>
                    <spatial name="cable">
                        <site site="site_a"/>
                        <site site="site_b"/>
                        <site site="site_c"/>
                    </spatial>
                </tendon>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");

        let tendon = model.tendon("cable").expect("should find tendon");
        assert!(tendon.is_spatial());

        let spatial = tendon.as_spatial().unwrap();
        // Path should have 3 points (origin, via, insertion)
        assert_eq!(spatial.tendon.path().num_points(), 3);
    }

    #[test]
    fn test_spatial_tendon_with_properties() {
        let xml = r#"
            <mujoco model="spatial_props">
                <worldbody>
                    <body name="a">
                        <geom type="sphere" size="0.1"/>
                        <site name="s1" pos="0 0 0"/>
                    </body>
                    <body name="b" pos="1 0 0">
                        <geom type="sphere" size="0.1"/>
                        <site name="s2" pos="0 0 0"/>
                    </body>
                </worldbody>
                <tendon>
                    <spatial name="elastic_cable" stiffness="5000" damping="100"
                             limited="true" range="0.8 1.2" width="0.005" rgba="0.2 0.8 0.2 1">
                        <site site="s1"/>
                        <site site="s2"/>
                    </spatial>
                </tendon>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");

        let tendon = model.tendon("elastic_cable").unwrap();
        assert!(tendon.limited());
        assert_eq!(tendon.range(), Some((0.8, 1.2)));
        assert_relative_eq!(tendon.width(), 0.005, epsilon = 1e-10);

        let rgba = tendon.rgba();
        assert_relative_eq!(rgba[0], 0.2, epsilon = 1e-10);
        assert_relative_eq!(rgba[1], 0.8, epsilon = 1e-10);
        assert_relative_eq!(rgba[2], 0.2, epsilon = 1e-10);
        assert_relative_eq!(rgba[3], 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_spatial_tendon_undefined_site() {
        let xml = r#"
            <mujoco model="bad_spatial">
                <worldbody>
                    <body name="a">
                        <geom type="sphere" size="0.1"/>
                        <site name="real_site" pos="0 0 0"/>
                    </body>
                </worldbody>
                <tendon>
                    <spatial name="bad_tendon">
                        <site site="real_site"/>
                        <site site="missing_site"/>
                    </spatial>
                </tendon>
            </mujoco>
        "#;

        let result = load_mjcf_str(xml);
        assert!(result.is_err());

        let err = result.unwrap_err();
        assert!(matches!(err, MjcfError::UndefinedSite { .. }));
    }

    #[test]
    fn test_spatial_tendon_insufficient_sites() {
        let xml = r#"
            <mujoco model="single_site">
                <worldbody>
                    <body name="a">
                        <geom type="sphere" size="0.1"/>
                        <site name="only_site" pos="0 0 0"/>
                    </body>
                </worldbody>
                <tendon>
                    <spatial name="invalid">
                        <site site="only_site"/>
                    </spatial>
                </tendon>
            </mujoco>
        "#;

        let result = load_mjcf_str(xml);
        assert!(result.is_err());

        let err = result.unwrap_err();
        assert!(matches!(err, MjcfError::InvalidAttribute { .. }));
    }

    #[test]
    fn test_mixed_fixed_and_spatial_tendons() {
        let xml = r#"
            <mujoco model="mixed_tendons">
                <worldbody>
                    <body name="upper">
                        <joint name="shoulder" type="hinge"/>
                        <geom type="capsule" size="0.05" fromto="0 0 0 0 0 0.3"/>
                        <site name="muscle_origin" pos="0 0.05 0.1"/>
                        <body name="lower" pos="0 0 0.3">
                            <joint name="elbow" type="hinge"/>
                            <geom type="capsule" size="0.04" fromto="0 0 0 0 0 0.25"/>
                            <site name="muscle_insertion" pos="0 0.03 0.05"/>
                        </body>
                    </body>
                </worldbody>
                <tendon>
                    <fixed name="coupling_tendon">
                        <joint joint="shoulder" coef="1"/>
                        <joint joint="elbow" coef="-0.5"/>
                    </fixed>
                    <spatial name="spatial_muscle">
                        <site site="muscle_origin"/>
                        <site site="muscle_insertion"/>
                    </spatial>
                </tendon>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");

        assert_eq!(model.tendons.len(), 2);
        assert_eq!(model.tendon_count(), 2);

        // Check that we can look up both types by name
        let coupling = model.tendon("coupling_tendon").expect("should find fixed");
        let muscle = model.tendon("spatial_muscle").expect("should find spatial");

        assert!(coupling.is_fixed());
        assert!(muscle.is_spatial());

        // Verify the fixed tendon structure
        let fixed = coupling.as_fixed().unwrap();
        assert_eq!(fixed.constraint.joints().len(), 2);

        // Verify the spatial tendon structure
        let spatial = muscle.as_spatial().unwrap();
        assert_eq!(spatial.tendon.path().num_points(), 2);
    }

    #[test]
    fn test_spatial_tendon_body_ids_correct() {
        let xml = r#"
            <mujoco model="body_check">
                <worldbody>
                    <body name="body1" pos="0 0 0">
                        <geom type="sphere" size="0.1"/>
                        <site name="site1" pos="0.1 0 0"/>
                    </body>
                    <body name="body2" pos="1 0 0">
                        <geom type="sphere" size="0.1"/>
                        <site name="site2" pos="-0.1 0 0"/>
                    </body>
                </worldbody>
                <tendon>
                    <spatial name="tendon">
                        <site site="site1"/>
                        <site site="site2"/>
                    </spatial>
                </tendon>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");

        let tendon = model.tendon("tendon").unwrap();
        let spatial = tendon.as_spatial().unwrap();

        // Get the body IDs
        let body1_id = model.body_to_id["body1"];
        let body2_id = model.body_to_id["body2"];

        // The spatial tendon should reference these bodies
        let connected = spatial.tendon.connected_bodies();
        assert!(connected.contains(&body1_id));
        assert!(connected.contains(&body2_id));
    }

    #[test]
    fn test_spatial_tendon_with_sphere_wrapping() {
        let xml = r#"
            <mujoco model="wrap_sphere">
                <worldbody>
                    <body name="body1" pos="0 0 0">
                        <geom type="sphere" size="0.1"/>
                        <site name="origin" pos="0 0 0"/>
                    </body>
                    <body name="body2" pos="1 0 0">
                        <geom type="sphere" size="0.1"/>
                        <site name="insertion" pos="0 0 0"/>
                    </body>
                    <body name="pulley" pos="0.5 0.2 0">
                        <geom name="wrap_sphere" type="sphere" size="0.05" pos="0 0 0"/>
                    </body>
                </worldbody>
                <tendon>
                    <spatial name="cable" stiffness="1000">
                        <site site="origin"/>
                        <geom geom="wrap_sphere"/>
                        <site site="insertion"/>
                    </spatial>
                </tendon>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");

        let tendon = model.tendon("cable").unwrap();
        assert!(tendon.is_spatial());

        let spatial = tendon.as_spatial().unwrap();
        assert_eq!(spatial.name, "cable");

        // Should have 1 wrapping geometry
        assert_eq!(spatial.tendon.wrapping().len(), 1);
    }

    #[test]
    fn test_spatial_tendon_with_cylinder_wrapping() {
        let xml = r#"
            <mujoco model="wrap_cylinder">
                <worldbody>
                    <body name="arm" pos="0 0 0">
                        <geom type="capsule" size="0.05" fromto="0 0 0 0 0 0.3"/>
                        <site name="muscle_origin" pos="0.06 0 0.05"/>
                        <site name="muscle_insertion" pos="0.06 0 0.25"/>
                        <!-- Cylinder wrap around the elbow joint -->
                        <geom name="elbow_wrap" type="cylinder" size="0.02 0.03" pos="0 0 0.15"/>
                    </body>
                </worldbody>
                <tendon>
                    <spatial name="bicep" stiffness="2000" damping="50">
                        <site site="muscle_origin"/>
                        <geom geom="elbow_wrap"/>
                        <site site="muscle_insertion"/>
                    </spatial>
                </tendon>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");

        let tendon = model.tendon("bicep").unwrap();
        assert!(tendon.is_spatial());

        let spatial = tendon.as_spatial().unwrap();
        assert_eq!(spatial.name, "bicep");

        // Should have 1 wrapping geometry (cylinder)
        assert_eq!(spatial.tendon.wrapping().len(), 1);
    }

    #[test]
    fn test_spatial_tendon_with_multiple_wrapping() {
        let xml = r#"
            <mujoco model="multi_wrap">
                <worldbody>
                    <body name="segment1" pos="0 0 0">
                        <geom type="sphere" size="0.05"/>
                        <site name="start" pos="0 0 0"/>
                    </body>
                    <body name="pulley1" pos="0.2 0.1 0">
                        <geom name="pulley_a" type="sphere" size="0.02"/>
                    </body>
                    <body name="pulley2" pos="0.4 -0.1 0">
                        <geom name="pulley_b" type="cylinder" size="0.02 0.02"/>
                    </body>
                    <body name="segment2" pos="0.6 0 0">
                        <geom type="sphere" size="0.05"/>
                        <site name="end" pos="0 0 0"/>
                    </body>
                </worldbody>
                <tendon>
                    <spatial name="cable">
                        <site site="start"/>
                        <geom geom="pulley_a"/>
                        <geom geom="pulley_b"/>
                        <site site="end"/>
                    </spatial>
                </tendon>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");

        let tendon = model.tendon("cable").unwrap();
        let spatial = tendon.as_spatial().unwrap();

        // Should have 2 wrapping geometries
        assert_eq!(spatial.tendon.wrapping().len(), 2);
    }

    #[test]
    fn test_spatial_tendon_undefined_wrapping_geom() {
        let xml = r#"
            <mujoco model="undefined_wrap">
                <worldbody>
                    <body name="body1" pos="0 0 0">
                        <geom type="sphere" size="0.1"/>
                        <site name="site1" pos="0 0 0"/>
                    </body>
                    <body name="body2" pos="1 0 0">
                        <geom type="sphere" size="0.1"/>
                        <site name="site2" pos="0 0 0"/>
                    </body>
                </worldbody>
                <tendon>
                    <spatial name="tendon">
                        <site site="site1"/>
                        <geom geom="nonexistent_geom"/>
                        <site site="site2"/>
                    </spatial>
                </tendon>
            </mujoco>
        "#;

        let result = load_mjcf_str(xml);
        assert!(result.is_err());

        let err = result.unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("nonexistent_geom"));
    }

    #[test]
    fn test_spatial_tendon_wrapping_ignores_unsupported_geom_types() {
        // Only sphere and cylinder can be used for wrapping
        // Box, capsule, etc. should be ignored (no error, just not added)
        let xml = r#"
            <mujoco model="unsupported_wrap">
                <worldbody>
                    <body name="body1" pos="0 0 0">
                        <geom type="sphere" size="0.1"/>
                        <site name="site1" pos="0 0 0"/>
                    </body>
                    <body name="body2" pos="1 0 0">
                        <geom type="sphere" size="0.1"/>
                        <site name="site2" pos="0 0 0"/>
                        <!-- Box geom - not suitable for wrapping -->
                        <geom name="box_geom" type="box" size="0.1 0.1 0.1"/>
                    </body>
                </worldbody>
                <tendon>
                    <spatial name="tendon">
                        <site site="site1"/>
                        <site site="site2"/>
                    </spatial>
                </tendon>
            </mujoco>
        "#;

        // This should succeed but the box geom won't be available for wrapping
        let model = load_mjcf_str(xml).expect("should load");
        let tendon = model.tendon("tendon").unwrap();
        let spatial = tendon.as_spatial().unwrap();

        // No wrapping geometries added
        assert_eq!(spatial.tendon.wrapping().len(), 0);
    }

    #[test]
    fn test_geom_info_sphere_wrapping_geometry() {
        let xml = r#"
            <mujoco model="geom_info_check">
                <worldbody>
                    <body name="body1" pos="0 0 0">
                        <geom name="wrap_sphere" type="sphere" size="0.05" pos="0.1 0.2 0.3"/>
                        <site name="s1" pos="0 0 0"/>
                    </body>
                    <body name="body2" pos="1 0 0">
                        <site name="s2" pos="0 0 0"/>
                    </body>
                </worldbody>
                <tendon>
                    <spatial name="t">
                        <site site="s1"/>
                        <geom geom="wrap_sphere"/>
                        <site site="s2"/>
                    </spatial>
                </tendon>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");
        let tendon = model.tendon("t").unwrap();
        let spatial = tendon.as_spatial().unwrap();

        assert_eq!(spatial.tendon.wrapping().len(), 1);
    }

    #[test]
    fn test_worldbody_wrapping_geom() {
        // Wrapping geoms can be attached to the world (worldbody)
        let xml = r#"
            <mujoco model="world_wrap">
                <worldbody>
                    <!-- Pulley fixed in world -->
                    <geom name="fixed_pulley" type="sphere" size="0.05" pos="0.5 0 0"/>
                    <body name="body1" pos="0 0 0">
                        <geom type="sphere" size="0.1"/>
                        <site name="start" pos="0 0 0"/>
                    </body>
                    <body name="body2" pos="1 0 0">
                        <geom type="sphere" size="0.1"/>
                        <site name="end" pos="0 0 0"/>
                    </body>
                </worldbody>
                <tendon>
                    <spatial name="cable">
                        <site site="start"/>
                        <geom geom="fixed_pulley"/>
                        <site site="end"/>
                    </spatial>
                </tendon>
            </mujoco>
        "#;

        let model = load_mjcf_str(xml).expect("should load");
        let tendon = model.tendon("cable").unwrap();
        let spatial = tendon.as_spatial().unwrap();

        // Should have the worldbody wrapping geometry
        assert_eq!(spatial.tendon.wrapping().len(), 1);
    }
}
