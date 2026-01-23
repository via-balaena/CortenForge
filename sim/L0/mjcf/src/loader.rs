//! MJCF to sim-core type conversion.
//!
//! Converts parsed MJCF intermediate representation into simulation-ready types.

use std::collections::HashMap;
use std::fs;
use std::path::Path;

use nalgebra::{Point3, Vector3};
use sim_constraint::{
    AdhesionActuator, BoxedActuator, ConnectConstraint, IntegratedVelocityActuator,
    IntoBoxedActuator, JointLimits, PneumaticCylinderActuator,
};
use sim_core::{Body, CollisionShape, Joint, World};
use sim_types::{BodyId, JointId, JointType, MassProperties, Pose, RigidBodyState};

use crate::config::ExtendedSolverConfig;
use crate::defaults::DefaultResolver;
use crate::error::{MjcfError, Result};
use crate::parser::parse_mjcf_str;
use crate::types::{
    MjcfActuator, MjcfActuatorType, MjcfBody, MjcfGeom, MjcfGeomType, MjcfJoint, MjcfJointType,
    MjcfMesh, MjcfModel, MjcfOption,
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
    /// Actuators ready for use.
    pub actuators: Vec<LoadedActuator>,
    /// Map from body name to body ID.
    pub body_to_id: HashMap<String, BodyId>,
    /// Map from joint name to joint ID.
    pub joint_to_id: HashMap<String, JointId>,
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

        // Convert actuators (with defaults applied)
        let (actuators, actuator_to_index) = self.convert_actuators(&model.actuators, &resolver);

        // Convert options to solver config
        let solver_config = ExtendedSolverConfig::from(&model.option);

        Ok(LoadedModel {
            name: model.name,
            bodies,
            joints,
            connect_constraints,
            actuators,
            body_to_id,
            joint_to_id,
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
            if let Some(collision_shape) =
                self.collision_shape_from_geoms(&resolved_geoms, mesh_lookup)
            {
                body = body.with_collision_shape(collision_shape);
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
}
