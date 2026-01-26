//! URDF robot description parser for physics simulation.
//!
//! This crate parses [URDF](http://wiki.ros.org/urdf) (Unified Robot Description Format)
//! files and converts them into simulation-ready types for use with `sim-core`.
//!
//! # Features
//!
//! - Parse URDF XML from files or strings
//! - Convert links to rigid bodies with mass properties
//! - Convert joints to sim-core joint constraints
//! - Support for primitive collision shapes (box, sphere, cylinder)
//! - Kinematic tree validation
//!
//! # Layer 0
//!
//! This is a Layer 0 crate with **zero Bevy dependencies**. It can be used in:
//!
//! - Headless training environments
//! - Hardware control systems
//! - Analysis tools
//! - Other game engines
//!
//! # Example
//!
//! ```
//! use sim_urdf::{load_urdf_str, UrdfLoader};
//! use sim_core::World;
//! use sim_types::Pose;
//!
//! // Load a simple robot from URDF
//! let urdf = r#"
//!     <robot name="simple">
//!         <link name="base_link">
//!             <inertial>
//!                 <mass value="1.0"/>
//!                 <inertia ixx="0.1" iyy="0.1" izz="0.1"/>
//!             </inertial>
//!         </link>
//!     </robot>
//! "#;
//!
//! let robot = load_urdf_str(urdf).expect("should parse");
//! assert_eq!(robot.name, "simple");
//!
//! // Spawn into a world
//! let mut world = World::default();
//! let spawned = robot.spawn_at_origin(&mut world).expect("should spawn");
//! assert_eq!(world.body_count(), 1);
//! ```
//!
//! # Supported URDF Elements
//!
//! ## Links
//!
//! - `<link name="...">` - Rigid body definition
//! - `<inertial>` - Mass, center of mass, inertia tensor
//! - `<collision>` - Collision geometry (primitives only)
//! - `<visual>` - Parsed but not used (Layer 0 has no rendering)
//!
//! ## Joints
//!
//! - `<joint name="..." type="...">` - Joint constraint
//! - Supported types: `fixed`, `revolute`, `continuous`, `prismatic`, `floating`, `planar`
//! - `<parent>`, `<child>` - Connected links
//! - `<origin>` - Joint frame relative to parent
//! - `<axis>` - Joint axis for revolute/prismatic
//! - `<limit>` - Position, velocity, effort limits
//! - `<dynamics>` - Damping and friction
//!
//! ## Geometry
//!
//! - `<box size="x y z"/>` - Box collision shape
//! - `<sphere radius="r"/>` - Sphere collision shape
//! - `<cylinder radius="r" length="l"/>` - Approximated as box (sim-core limitation)
//! - `<mesh filename="..."/>` - Parsed but not loaded (no mesh collision support)
//!
//! # Limitations
//!
//! - Mesh collision shapes are not supported (logged as warning)
//! - `<mimic>` joints are not supported
//! - `<gazebo>` extensions are ignored
//! - Kinematic loops are not supported (tree structures only)
//!
//! # Coordinate System
//!
//! URDF uses a right-handed coordinate system which is converted to match
//! the CortenForge convention:
//!
//! - X: right
//! - Y: forward
//! - Z: up

#![doc(html_root_url = "https://docs.rs/sim-urdf/0.7.0")]
#![deny(clippy::unwrap_used, clippy::expect_used)]
#![warn(missing_docs)]
#![allow(
    clippy::missing_const_for_fn,
    clippy::module_name_repetitions,
    clippy::unnested_or_patterns,
    clippy::similar_names,
    clippy::must_use_candidate,
    clippy::missing_errors_doc,
    clippy::missing_panics_doc,
    clippy::unnecessary_wraps,
    clippy::redundant_closure_for_method_calls,
    clippy::should_implement_trait,
    clippy::items_after_statements,
    clippy::unnecessary_lazy_evaluations,
    clippy::needless_pass_by_value,
    clippy::map_unwrap_or,
    clippy::option_if_let_else,
    clippy::unused_self,
    clippy::redundant_pattern_matching
)]

mod converter;
mod error;
mod loader;
mod parser;
mod types;
mod validation;

// Re-export main types
pub use converter::{robot_to_mjcf, urdf_to_mjcf};
pub use error::{Result, UrdfError};
pub use loader::{LoadedRobot, SpawnedRobot, UrdfLoader, load_urdf_file, load_urdf_str};
pub use parser::parse_urdf_str;
pub use types::{
    UrdfCollision, UrdfGeometry, UrdfInertia, UrdfInertial, UrdfJoint, UrdfJointDynamics,
    UrdfJointLimit, UrdfJointType, UrdfLink, UrdfOrigin, UrdfRobot, UrdfVisual,
};
pub use validation::{ValidationResult, validate};

/// Load a URDF file and convert it directly to a MuJoCo-aligned Model.
///
/// This is the recommended way to load URDF robots for simulation. It:
/// 1. Parses the URDF XML
/// 2. Converts to MJCF XML (single compilation path)
/// 3. Loads via `sim_mjcf::load_model()`
///
/// # Example
///
/// ```ignore
/// use sim_urdf::load_urdf_model;
///
/// let urdf = r#"<robot name="arm">...</robot>"#;
/// let model = load_urdf_model(urdf)?;
/// let mut data = model.make_data();
/// data.step(&model);
/// ```
pub fn load_urdf_model(urdf_xml: &str) -> Result<sim_core::Model> {
    let mjcf = urdf_to_mjcf(urdf_xml)?;
    sim_mjcf::load_model(&mjcf)
        .map_err(|e| UrdfError::Unsupported(format!("MJCF conversion error: {e}")))
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use sim_core::World;

    /// Integration test with a more complex robot.
    #[test]
    fn test_two_link_arm() {
        let urdf = r#"
            <robot name="two_link_arm">
                <link name="base_link">
                    <inertial>
                        <mass value="5.0"/>
                        <inertia ixx="0.5" iyy="0.5" izz="0.5"/>
                    </inertial>
                </link>

                <link name="link1">
                    <inertial>
                        <origin xyz="0 0 0.25"/>
                        <mass value="1.0"/>
                        <inertia ixx="0.1" iyy="0.1" izz="0.01"/>
                    </inertial>
                    <collision>
                        <origin xyz="0 0 0.25"/>
                        <geometry>
                            <cylinder radius="0.05" length="0.5"/>
                        </geometry>
                    </collision>
                </link>

                <link name="link2">
                    <inertial>
                        <origin xyz="0 0 0.2"/>
                        <mass value="0.5"/>
                        <inertia ixx="0.05" iyy="0.05" izz="0.005"/>
                    </inertial>
                    <collision>
                        <origin xyz="0 0 0.2"/>
                        <geometry>
                            <sphere radius="0.08"/>
                        </geometry>
                    </collision>
                </link>

                <joint name="joint1" type="revolute">
                    <parent link="base_link"/>
                    <child link="link1"/>
                    <origin xyz="0 0 0.1"/>
                    <axis xyz="0 1 0"/>
                    <limit lower="-3.14" upper="3.14" effort="100" velocity="2"/>
                    <dynamics damping="0.5" friction="0.1"/>
                </joint>

                <joint name="joint2" type="revolute">
                    <parent link="link1"/>
                    <child link="link2"/>
                    <origin xyz="0 0 0.5"/>
                    <axis xyz="0 1 0"/>
                    <limit lower="-2.0" upper="2.0" effort="50" velocity="3"/>
                    <dynamics damping="0.2"/>
                </joint>
            </robot>
        "#;

        // Load and validate
        let robot = load_urdf_str(urdf).expect("should load");
        assert_eq!(robot.name, "two_link_arm");
        assert_eq!(robot.bodies.len(), 3);
        assert_eq!(robot.joints.len(), 2);

        // Spawn into world
        let mut world = World::default();
        let spawned = robot.spawn_at_origin(&mut world).expect("should spawn");

        // Verify world state
        assert_eq!(world.body_count(), 3);
        assert_eq!(world.joint_count(), 2);

        // Check we can access by name
        let base_id = spawned.link_id("base_link").expect("base_link");
        let joint1_id = spawned.joint_id("joint1").expect("joint1");

        assert!(world.body(base_id).is_some());
        assert!(world.joint(joint1_id).is_some());
    }

    /// Test error handling for invalid URDF.
    #[test]
    fn test_invalid_urdf() {
        // Missing robot element
        let result = parse_urdf_str("<link name='test'/>");
        assert!(result.is_err());

        // Invalid joint type
        let result = parse_urdf_str(
            r#"
            <robot name="test">
                <link name="a"/>
                <link name="b"/>
                <joint name="j" type="invalid">
                    <parent link="a"/>
                    <child link="b"/>
                </joint>
            </robot>
        "#,
        );
        assert!(matches!(result, Err(UrdfError::UnknownJointType(_))));
    }

    /// Test validation catches kinematic issues.
    #[test]
    fn test_validation_errors() {
        // Undefined link reference
        let robot = UrdfRobot::new("test")
            .with_link(UrdfLink::new("base"))
            .with_joint(UrdfJoint::new(
                "j1",
                UrdfJointType::Fixed,
                "base",
                "nonexistent",
            ));

        let result = validate(&robot);
        assert!(matches!(result, Err(UrdfError::UndefinedLink { .. })));
    }
}
