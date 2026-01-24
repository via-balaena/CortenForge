//! URDF to sim-core type conversion.
//!
//! Converts parsed URDF intermediate representation into simulation-ready types.

use std::collections::HashMap;
use std::fs;
use std::path::Path;

use nalgebra::{Point3, Vector3};
use sim_constraint::JointLimits;
use sim_core::{Body, CollisionShape, Joint, World};
use sim_types::{BodyId, JointId, JointType, MassProperties, Pose, RigidBodyState};

use crate::error::{Result, UrdfError};
use crate::parser::parse_urdf_str;
use crate::types::{UrdfGeometry, UrdfJoint, UrdfJointType, UrdfLink, UrdfOrigin, UrdfRobot};
use crate::validation::{ValidationResult, validate};

/// A loaded robot ready to be spawned into a simulation world.
#[derive(Debug)]
pub struct LoadedRobot {
    /// Robot name.
    pub name: String,
    /// Bodies (links) ready for insertion.
    pub bodies: Vec<Body>,
    /// Joints ready for insertion.
    pub joints: Vec<Joint>,
    /// The root link body ID.
    pub root_body_id: BodyId,
    /// Map from link name to body ID.
    pub link_to_body: HashMap<String, BodyId>,
    /// Map from joint name to joint ID.
    pub joint_to_id: HashMap<String, JointId>,
}

/// Result of spawning a robot into a world.
#[derive(Debug)]
pub struct SpawnedRobot {
    /// Map from link name to body ID in the world.
    pub link_ids: HashMap<String, BodyId>,
    /// Map from joint name to joint ID in the world.
    pub joint_ids: HashMap<String, JointId>,
    /// The root body ID.
    pub root_id: BodyId,
}

impl SpawnedRobot {
    /// Get the body ID for a link by name.
    pub fn link_id(&self, name: &str) -> Option<BodyId> {
        self.link_ids.get(name).copied()
    }

    /// Get the joint ID by name.
    pub fn joint_id(&self, name: &str) -> Option<JointId> {
        self.joint_ids.get(name).copied()
    }
}

impl LoadedRobot {
    /// Spawn this robot into a world at the given base pose.
    ///
    /// # Errors
    ///
    /// Returns an error if inserting bodies or joints fails.
    pub fn spawn_into(self, world: &mut World, base_pose: Pose) -> Result<SpawnedRobot> {
        let mut link_ids = HashMap::new();
        let mut joint_ids = HashMap::new();

        // Insert bodies, applying base pose to root
        for mut body in self.bodies {
            let is_root = body.id == self.root_body_id;

            if is_root {
                // Apply base pose to root body
                body.state.pose = base_pose.compose(&body.state.pose);
            }

            let id = body.id;
            let name = body.name.clone();
            world
                .insert_body(body)
                .map_err(|e| UrdfError::XmlParse(e.to_string()))?;

            if let Some(name) = name {
                link_ids.insert(name, id);
            }
        }

        // Insert joints
        for joint in self.joints {
            let id = joint.id;
            let name = joint.name.clone();
            world
                .insert_joint(joint)
                .map_err(|e| UrdfError::XmlParse(e.to_string()))?;

            if let Some(name) = name {
                joint_ids.insert(name, id);
            }
        }

        Ok(SpawnedRobot {
            link_ids,
            joint_ids,
            root_id: self.root_body_id,
        })
    }

    /// Spawn at the origin with identity pose.
    pub fn spawn_at_origin(self, world: &mut World) -> Result<SpawnedRobot> {
        self.spawn_into(world, Pose::identity())
    }
}

/// URDF loader with configuration options.
#[derive(Debug, Clone)]
pub struct UrdfLoader {
    /// Whether to use collision shapes from URDF (default: true).
    pub use_collision_shapes: bool,
    /// Default mass for links without inertial (default: 1.0).
    pub default_mass: f64,
    /// Whether fixed links should be static bodies (default: false).
    pub fixed_links_static: bool,
}

impl Default for UrdfLoader {
    fn default() -> Self {
        Self {
            use_collision_shapes: true,
            default_mass: 1.0,
            fixed_links_static: false,
        }
    }
}

impl UrdfLoader {
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

    /// Set the default mass for links without inertial data.
    #[must_use]
    pub fn with_default_mass(mut self, mass: f64) -> Self {
        self.default_mass = mass;
        self
    }

    /// Load URDF from a file path.
    ///
    /// # Errors
    ///
    /// Returns an error if the file cannot be read or parsed.
    pub fn load_file(&self, path: impl AsRef<Path>) -> Result<LoadedRobot> {
        let content = fs::read_to_string(path)?;
        self.load_str(&content)
    }

    /// Load URDF from a string.
    ///
    /// # Errors
    ///
    /// Returns an error if parsing or validation fails.
    pub fn load_str(&self, xml: &str) -> Result<LoadedRobot> {
        let robot = parse_urdf_str(xml)?;
        self.load_robot(robot)
    }

    /// Load from a parsed URDF robot.
    ///
    /// # Errors
    ///
    /// Returns an error if validation fails.
    pub fn load_robot(&self, robot: UrdfRobot) -> Result<LoadedRobot> {
        let validation = validate(&robot)?;
        self.convert(robot, validation)
    }

    /// Convert URDF types to sim-core types.
    fn convert(&self, robot: UrdfRobot, validation: ValidationResult) -> Result<LoadedRobot> {
        let mut bodies = Vec::new();
        let mut joints = Vec::new();
        let mut link_to_body: HashMap<String, BodyId> = HashMap::new();
        let mut joint_to_id: HashMap<String, JointId> = HashMap::new();

        // Build link name -> UrdfLink lookup
        let link_map: HashMap<&str, &UrdfLink> =
            robot.links.iter().map(|l| (l.name.as_str(), l)).collect();

        // Build joint name -> UrdfJoint lookup
        let joint_map: HashMap<&str, &UrdfJoint> =
            robot.joints.iter().map(|j| (j.name.as_str(), j)).collect();

        // Assign body IDs in topological order
        let mut next_body_id = 1u64;
        let mut next_joint_id = 1u64;

        // Create bodies for each link
        for link_name in &validation.sorted_links {
            let link = link_map
                .get(link_name.as_str())
                .ok_or_else(|| UrdfError::XmlParse(format!("link '{link_name}' not found")))?;

            let body_id = BodyId::new(next_body_id);
            next_body_id += 1;
            link_to_body.insert(link_name.clone(), body_id);

            // Compute pose relative to world
            // For root, pose is identity (base pose applied at spawn time)
            // For children, pose is computed from parent pose + joint origin
            let pose = if link_name == &validation.root_link {
                Pose::identity()
            } else {
                // Find parent joint
                let parent_joint_name =
                    validation.link_parent_joint.get(link_name).ok_or_else(|| {
                        UrdfError::XmlParse(format!("no parent joint for {link_name}"))
                    })?;

                let parent_joint = joint_map.get(parent_joint_name.as_str()).ok_or_else(|| {
                    UrdfError::XmlParse(format!("joint '{parent_joint_name}' not found"))
                })?;

                // Get parent body pose
                let parent_body_id = link_to_body.get(&parent_joint.parent).ok_or_else(|| {
                    UrdfError::XmlParse(format!("parent link '{}' not found", parent_joint.parent))
                })?;

                // Find parent body to get its pose
                let parent_pose = bodies
                    .iter()
                    .find(|b: &&Body| b.id == *parent_body_id)
                    .map(|b| b.state.pose)
                    .unwrap_or_else(Pose::identity);

                // Compose parent pose with joint origin
                let joint_pose = origin_to_pose(&parent_joint.origin);
                parent_pose.compose(&joint_pose)
            };

            // Create body
            let body = self.create_body(body_id, link, pose);
            bodies.push(body);
        }

        // Create joints
        for urdf_joint in &robot.joints {
            let joint_id = JointId::new(next_joint_id);
            next_joint_id += 1;
            joint_to_id.insert(urdf_joint.name.clone(), joint_id);

            let parent_id = *link_to_body
                .get(&urdf_joint.parent)
                .ok_or_else(|| UrdfError::undefined_link(&urdf_joint.parent, &urdf_joint.name))?;

            let child_id = *link_to_body
                .get(&urdf_joint.child)
                .ok_or_else(|| UrdfError::undefined_link(&urdf_joint.child, &urdf_joint.name))?;

            let joint = self.create_joint(joint_id, urdf_joint, parent_id, child_id);
            joints.push(joint);
        }

        let root_body_id = *link_to_body
            .get(&validation.root_link)
            .ok_or_else(|| UrdfError::NoRootLink)?;

        Ok(LoadedRobot {
            name: robot.name,
            bodies,
            joints,
            root_body_id,
            link_to_body,
            joint_to_id,
        })
    }

    /// Create a sim-core Body from a URDF link.
    fn create_body(&self, id: BodyId, link: &UrdfLink, pose: Pose) -> Body {
        // Get mass properties
        let mass_props = if let Some(ref inertial) = link.inertial {
            // Transform inertia from inertial frame to link frame if needed
            // For now, assume inertial origin is at link origin
            MassProperties::new(
                inertial.mass,
                inertial.origin.xyz,
                inertial.inertia.to_matrix(),
            )
        } else {
            // Default mass for links without inertial
            MassProperties::point_mass(self.default_mass)
        };

        let state = RigidBodyState::at_rest(pose);
        let mut body = Body::new(id, state, mass_props).with_name(&link.name);

        // Add collision shape if available
        if self.use_collision_shapes {
            if let Some(collision_shape) = self.collision_shape_for_link(link) {
                body = body.with_collision_shape(collision_shape);
            }
        }

        body
    }

    /// Create a collision shape from URDF collision geometry.
    fn collision_shape_for_link(&self, link: &UrdfLink) -> Option<CollisionShape> {
        // Use first collision geometry if available
        link.collisions.first().and_then(|c| {
            match &c.geometry {
                UrdfGeometry::Box { size } => {
                    Some(CollisionShape::box_shape(*size / 2.0)) // URDF uses full size, sim uses half-extents
                }
                UrdfGeometry::Sphere { radius } => Some(CollisionShape::sphere(*radius)),
                UrdfGeometry::Cylinder { radius, length } => {
                    // Approximate cylinder as a box (sim-core doesn't have cylinder)
                    // Could also use sphere with radius = max(radius, length/2)
                    Some(CollisionShape::box_shape(Vector3::new(
                        *radius,
                        *radius,
                        length / 2.0,
                    )))
                }
                UrdfGeometry::Mesh { .. } => {
                    // Mesh collision not supported in sim-core primitive shapes
                    None
                }
            }
        })
    }

    /// Create a sim-core Joint from a URDF joint.
    fn create_joint(
        &self,
        id: JointId,
        urdf_joint: &UrdfJoint,
        parent: BodyId,
        child: BodyId,
    ) -> Joint {
        let joint_type = match urdf_joint.joint_type {
            UrdfJointType::Fixed => JointType::Fixed,
            UrdfJointType::Revolute | UrdfJointType::Continuous => JointType::Revolute,
            UrdfJointType::Prismatic => JointType::Prismatic,
            UrdfJointType::Floating => JointType::Free,
            UrdfJointType::Planar => JointType::Planar,
        };

        let mut joint = Joint::new(id, joint_type, parent, child)
            .with_name(&urdf_joint.name)
            .with_axis(urdf_joint.axis);

        // Set anchors from joint origin
        // Parent anchor is at the joint origin in parent frame
        // Child anchor is at origin of child frame
        let parent_anchor = Point3::from(urdf_joint.origin.xyz);
        joint = joint.with_anchors(parent_anchor, Point3::origin());

        // Add limits if present
        if let Some(ref limit) = urdf_joint.limit {
            // Continuous joints don't have position limits
            if urdf_joint.joint_type != UrdfJointType::Continuous {
                // Note: JointLimits from sim-constraint only supports position limits
                // Velocity and effort limits from URDF are not currently used
                let joint_limits = JointLimits::new(limit.lower, limit.upper);
                joint = joint.with_limits(joint_limits);
            }
        }

        // Add damping if specified
        if let Some(ref dynamics) = urdf_joint.dynamics {
            joint = joint.with_damping(dynamics.damping);
        }

        joint
    }
}

/// Convert URDF origin to sim-types Pose.
fn origin_to_pose(origin: &UrdfOrigin) -> Pose {
    Pose::from_position_rotation(origin.position(), origin.rotation())
}

/// Convenience function to load a URDF file with default settings.
///
/// # Errors
///
/// Returns an error if the file cannot be read or parsed.
pub fn load_urdf_file(path: impl AsRef<Path>) -> Result<LoadedRobot> {
    UrdfLoader::default().load_file(path)
}

/// Convenience function to load a URDF string with default settings.
///
/// # Errors
///
/// Returns an error if parsing fails.
pub fn load_urdf_str(xml: &str) -> Result<LoadedRobot> {
    UrdfLoader::default().load_str(xml)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    const SIMPLE_ARM_URDF: &str = r#"
        <robot name="simple_arm">
            <link name="base_link">
                <inertial>
                    <mass value="10.0"/>
                    <inertia ixx="1" iyy="1" izz="1"/>
                </inertial>
                <collision>
                    <geometry>
                        <box size="0.2 0.2 0.1"/>
                    </geometry>
                </collision>
            </link>
            <link name="upper_arm">
                <inertial>
                    <mass value="1.0"/>
                    <inertia ixx="0.1" iyy="0.1" izz="0.1"/>
                </inertial>
                <collision>
                    <geometry>
                        <cylinder radius="0.05" length="0.5"/>
                    </geometry>
                </collision>
            </link>
            <link name="lower_arm">
                <inertial>
                    <mass value="0.5"/>
                    <inertia ixx="0.05" iyy="0.05" izz="0.05"/>
                </inertial>
                <collision>
                    <geometry>
                        <sphere radius="0.1"/>
                    </geometry>
                </collision>
            </link>
            <joint name="shoulder" type="revolute">
                <parent link="base_link"/>
                <child link="upper_arm"/>
                <origin xyz="0 0 0.1" rpy="0 0 0"/>
                <axis xyz="0 1 0"/>
                <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
                <dynamics damping="0.1"/>
            </joint>
            <joint name="elbow" type="revolute">
                <parent link="upper_arm"/>
                <child link="lower_arm"/>
                <origin xyz="0 0 0.5"/>
                <axis xyz="0 1 0"/>
                <limit lower="-2.0" upper="2.0" effort="50" velocity="2"/>
            </joint>
        </robot>
    "#;

    #[test]
    fn test_load_simple_arm() {
        let robot = load_urdf_str(SIMPLE_ARM_URDF).expect("should load");

        assert_eq!(robot.name, "simple_arm");
        assert_eq!(robot.bodies.len(), 3);
        assert_eq!(robot.joints.len(), 2);

        // Check body names
        assert!(robot.link_to_body.contains_key("base_link"));
        assert!(robot.link_to_body.contains_key("upper_arm"));
        assert!(robot.link_to_body.contains_key("lower_arm"));

        // Check joint names
        assert!(robot.joint_to_id.contains_key("shoulder"));
        assert!(robot.joint_to_id.contains_key("elbow"));
    }

    #[test]
    fn test_mass_properties() {
        let robot = load_urdf_str(SIMPLE_ARM_URDF).expect("should load");

        let base_id = robot.link_to_body["base_link"];
        let base = robot.bodies.iter().find(|b| b.id == base_id).expect("base");

        assert_relative_eq!(base.mass_props.mass, 10.0, epsilon = 1e-10);
    }

    #[test]
    fn test_collision_shapes() {
        let robot = load_urdf_str(SIMPLE_ARM_URDF).expect("should load");

        // Base has box collision
        let base_id = robot.link_to_body["base_link"];
        let base = robot.bodies.iter().find(|b| b.id == base_id).expect("base");
        assert!(matches!(
            base.collision_shape,
            Some(CollisionShape::Box { .. })
        ));

        // Lower arm has sphere collision
        let lower_id = robot.link_to_body["lower_arm"];
        let lower = robot
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
        let robot = load_urdf_str(SIMPLE_ARM_URDF).expect("should load");

        let shoulder_id = robot.joint_to_id["shoulder"];
        let shoulder = robot
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
        let robot = load_urdf_str(SIMPLE_ARM_URDF).expect("should load");

        let mut world = World::default();
        let spawned = robot.spawn_at_origin(&mut world).expect("should spawn");

        assert_eq!(world.body_count(), 3);
        assert_eq!(world.joint_count(), 2);

        assert!(spawned.link_id("base_link").is_some());
        assert!(spawned.joint_id("shoulder").is_some());
    }

    #[test]
    fn test_spawn_with_base_pose() {
        let robot = load_urdf_str(SIMPLE_ARM_URDF).expect("should load");

        let mut world = World::default();
        let base_pose = Pose::from_position(Point3::new(1.0, 2.0, 3.0));
        let spawned = robot
            .spawn_into(&mut world, base_pose)
            .expect("should spawn");

        let base_id = spawned.link_id("base_link").expect("base_link");
        let base = world.body(base_id).expect("body");

        assert_relative_eq!(base.state.pose.position.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(base.state.pose.position.y, 2.0, epsilon = 1e-10);
        assert_relative_eq!(base.state.pose.position.z, 3.0, epsilon = 1e-10);
    }

    #[test]
    fn test_loader_without_collision() {
        let loader = UrdfLoader::new().with_collision_shapes(false);
        let robot = loader.load_str(SIMPLE_ARM_URDF).expect("should load");

        for body in &robot.bodies {
            assert!(body.collision_shape.is_none());
        }
    }

    #[test]
    fn test_continuous_joint() {
        let xml = r#"
            <robot name="wheel">
                <link name="base"/>
                <link name="wheel"/>
                <joint name="axle" type="continuous">
                    <parent link="base"/>
                    <child link="wheel"/>
                    <axis xyz="1 0 0"/>
                </joint>
            </robot>
        "#;

        let robot = load_urdf_str(xml).expect("should load");
        let axle_id = robot.joint_to_id["axle"];
        let axle = robot.joints.iter().find(|j| j.id == axle_id).expect("axle");

        assert_eq!(axle.joint_type, JointType::Revolute);
        assert!(axle.limits.is_none()); // Continuous joints have no limits
    }

    #[test]
    fn test_fixed_joint() {
        let xml = r#"
            <robot name="fixture">
                <link name="base"/>
                <link name="attached"/>
                <joint name="weld" type="fixed">
                    <parent link="base"/>
                    <child link="attached"/>
                </joint>
            </robot>
        "#;

        let robot = load_urdf_str(xml).expect("should load");
        let weld_id = robot.joint_to_id["weld"];
        let weld = robot.joints.iter().find(|j| j.id == weld_id).expect("weld");

        assert_eq!(weld.joint_type, JointType::Fixed);
    }
}
