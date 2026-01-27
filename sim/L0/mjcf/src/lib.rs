//! MJCF (MuJoCo XML Format) model loader for physics simulation.
//!
//! This crate parses [MJCF](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
//! (MuJoCo XML Format) files and converts them into simulation-ready types for use
//! with `sim-core`.
//!
//! # Features
//!
//! - Parse MJCF XML from files or strings
//! - Convert to MuJoCo-aligned Model/Data architecture
//! - Support for primitive collision shapes (sphere, box, capsule, plane, cylinder, ellipsoid)
//! - Support for mesh collision shapes (convex hull from embedded data or external files)
//! - Support for actuators (motors, position/velocity servos)
//! - Default class inheritance system
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
//! use sim_mjcf::load_model;
//!
//! // Load a simple model from MJCF
//! let mjcf = r#"
//!     <mujoco model="simple">
//!         <worldbody>
//!             <body name="base">
//!                 <geom type="sphere" size="0.1" mass="1.0"/>
//!             </body>
//!         </worldbody>
//!     </mujoco>
//! "#;
//!
//! let model = load_model(mjcf).expect("should parse");
//! assert_eq!(model.name, "simple");
//!
//! // Create simulation data
//! let mut data = model.make_data();
//!
//! // Step the simulation
//! for _ in 0..100 {
//!     data.step(&model);
//! }
//! ```
//!
//! # Supported MJCF Elements
//!
//! ## Model Structure
//!
//! - `<mujoco model="...">` - Root element, model name parsing
//! - `<option>` - Global simulation options (gravity, timestep)
//! - `<default>` - Default parameter classes
//! - `<asset>` - Asset definitions (meshes, textures, materials)
//! - `<worldbody>` - Root of the body tree
//!
//! ## Bodies
//!
//! - `<body name="...">` - Hierarchical body definition
//! - `<inertial>` - Mass properties with pos, mass, diaginertia/fullinertia
//! - `<joint>` - Joint attached to body
//! - `<geom>` - Collision/visual geometry attached to body
//! - `<site>` - Marker/sensor attachment point - parsed but not used
//!
//! ## Joints
//!
//! - `type="hinge"` - Single-axis rotation (revolute)
//! - `type="slide"` - Single-axis translation (prismatic)
//! - `type="ball"` - Ball-and-socket (spherical, 3 DOF)
//! - `type="free"` - 6 DOF floating joint
//! - `limited="true"`, `range="..."` - Position limits
//! - `damping`, `stiffness` - Joint dynamics
//!
//! ## Geometry
//!
//! - `<geom type="sphere" size="r"/>` - Sphere collision shape
//! - `<geom type="box" size="x y z"/>` - Box collision shape (half-extents)
//! - `<geom type="capsule" size="r l"/>` - Capsule collision shape
//! - `<geom type="cylinder" size="r l"/>` - Cylinder collision shape
//! - `<geom type="ellipsoid" size="rx ry rz"/>` - Ellipsoid collision shape
//! - `<geom type="plane" size="x y z"/>` - Infinite plane
//! - `<geom type="mesh" mesh="asset_name"/>` - Mesh collision shape (convex hull)
//!
//! ## Mesh Assets
//!
//! - `<mesh name="..." file="path.stl"/>` - Load mesh from STL, OBJ, PLY, or 3MF file
//! - `<mesh name="..." vertex="x y z ..."/>` - Embedded vertex data
//! - `<mesh name="..." scale="sx sy sz"/>` - Scale factor for mesh vertices
//!
//! ## Actuators
//!
//! - `<motor>` - Direct torque/force actuator
//! - `<position>` - Position servo with PD control
//! - `<velocity>` - Velocity servo
//! - `<general>` - General actuator (limited support)
//!
//! ## Contact
//!
//! - `<contact>` - Contact pair filtering (parsed but not implemented)
//!
//! # MJB Binary Format
//!
//! When the `mjb` feature is enabled, this crate also supports MuJoCo's binary
//! format (MJB) for faster model loading:
//!
//! ```ignore
//! use sim_mjcf::{load_model, load_mjb_file, save_mjb_file};
//!
//! // Parse MJCF and save as binary
//! let model = load_model("<mujoco><worldbody/></mujoco>").unwrap();
//! save_mjb_file(&model, "model.mjb").unwrap();
//!
//! // Later, load the binary format (much faster)
//! let loaded = load_mjb_file("model.mjb").unwrap();
//! ```
//!
//! # Limitations
//!
//! - Non-convex meshes are converted to convex hulls (may not match original geometry)
//! - Height fields (hfield) and signed distance fields (sdf) are not supported
//! - Composite bodies are not supported
//! - Include files are not supported
//!
//! # Coordinate System
//!
//! MJCF uses a right-handed coordinate system (Z-up by default) which is
//! converted to match the CortenForge convention:
//!
//! - X: right
//! - Y: forward
//! - Z: up

#![doc(html_root_url = "https://docs.rs/sim-mjcf/0.7.0")]
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
    clippy::redundant_pattern_matching,
    clippy::doc_markdown,
    clippy::cast_sign_loss,
    clippy::field_reassign_with_default,
    clippy::suboptimal_flops,
    clippy::derivable_impls,
    clippy::too_many_lines,
    clippy::too_many_arguments,
    clippy::struct_field_names,
    clippy::use_self
)]

mod config;
mod defaults;
mod error;
#[cfg(feature = "mjb")]
mod mjb;
mod model_builder;
mod parser;
mod types;
mod validation;

// Re-export main types
pub use config::ExtendedSolverConfig;
pub use defaults::DefaultResolver;
pub use error::{MjcfError, Result};
pub use parser::parse_mjcf_str;
pub use types::{
    MjcfActuator, MjcfActuatorDefaults, MjcfActuatorType, MjcfBody, MjcfConeType, MjcfConnect,
    MjcfDefault, MjcfDistance, MjcfEquality, MjcfFlag, MjcfGeom, MjcfGeomDefaults, MjcfGeomType,
    MjcfInertial, MjcfIntegrator, MjcfJacobianType, MjcfJoint, MjcfJointDefaults,
    MjcfJointEquality, MjcfJointType, MjcfMesh, MjcfMeshDefaults, MjcfModel, MjcfOption,
    MjcfSensorDefaults, MjcfSite, MjcfSiteDefaults, MjcfSolverType, MjcfTendonDefaults, MjcfWeld,
};
pub use validation::{ValidationResult, validate};

// MuJoCo-aligned Model conversion (primary API)
pub use model_builder::{ModelConversionError, load_model, model_from_mjcf};

// MJB binary format support (requires "mjb" feature)
#[cfg(feature = "mjb")]
pub use mjb::{
    MJB_HEADER_SIZE, MJB_MAGIC, MJB_VERSION, MjbHeader, is_mjb_bytes, is_mjb_file, load_mjb_bytes,
    load_mjb_file, load_mjb_reader, save_mjb_bytes, save_mjb_file, save_mjb_writer,
};

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    /// Test Model/Data API with a two-link arm.
    #[test]
    fn test_two_link_arm_model_data() {
        let mjcf = r#"
            <mujoco model="two_link_arm">
                <option gravity="0 0 -9.81"/>
                <worldbody>
                    <body name="base_link" pos="0 0 0.1">
                        <inertial pos="0 0 0" mass="5.0" diaginertia="0.5 0.5 0.5"/>
                        <geom type="box" size="0.1 0.1 0.05"/>
                        <body name="link1" pos="0 0 0.05">
                            <joint name="joint1" type="hinge" axis="0 1 0" limited="true" range="-3.14 3.14" damping="0.5"/>
                            <inertial pos="0 0 0.25" mass="1.0" diaginertia="0.1 0.1 0.01"/>
                            <geom type="capsule" size="0.05" fromto="0 0 0 0 0 0.5"/>
                            <body name="link2" pos="0 0 0.5">
                                <joint name="joint2" type="hinge" axis="0 1 0" limited="true" range="-2.0 2.0" damping="0.2"/>
                                <inertial pos="0 0 0.2" mass="0.5" diaginertia="0.05 0.05 0.005"/>
                                <geom type="sphere" size="0.08"/>
                            </body>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#;

        // Load into Model
        let model = load_model(mjcf).expect("should load");
        assert_eq!(model.name, "two_link_arm");

        // Create Data and step
        let mut data = model.make_data();
        data.forward(&model);

        // Verify structure
        assert!(model.nbody >= 3, "Should have at least 3 bodies");
        assert_eq!(model.njnt, 2, "Should have 2 joints");
    }

    /// Test error handling for invalid MJCF.
    #[test]
    fn test_invalid_mjcf() {
        // Missing mujoco element
        let result = parse_mjcf_str("<body name='test'/>");
        assert!(result.is_err());

        // Invalid joint type
        let result = parse_mjcf_str(
            r#"
            <mujoco model="test">
                <worldbody>
                    <body name="a">
                        <joint name="j" type="invalid"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        );
        assert!(matches!(result, Err(MjcfError::UnknownJointType(_))));
    }

    /// Test validation catches kinematic issues.
    #[test]
    fn test_validation_errors() {
        // Empty model (no bodies) - should validate fine
        let model = MjcfModel::new("test");
        let result = validate(&model);
        assert!(result.is_ok());
    }

    /// Test simple sphere body with Model/Data API.
    #[test]
    fn test_simple_sphere_model_data() {
        let mjcf = r#"
            <mujoco model="sphere">
                <worldbody>
                    <body name="ball" pos="0 0 1">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;

        let model = load_model(mjcf).expect("should load");
        assert_eq!(model.name, "sphere");

        let mut data = model.make_data();
        data.forward(&model);

        // Step a few times
        for _ in 0..10 {
            data.step(&model);
        }
    }
}
