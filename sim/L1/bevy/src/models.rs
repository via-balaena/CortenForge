//! Model loading utilities for MJCF and URDF files.
//!
//! **Deprecated**: This module spawns models into the deprecated `SimulationHandle`
//! (World API). For new code, use `sim_mjcf::load_model()` to get a `Model` directly,
//! then use [`PhysicsModel`](crate::model_data::PhysicsModel) and
//! [`PhysicsData`](crate::model_data::PhysicsData).
//!
//! This module provides integration between sim-bevy and the Layer 0
//! model loaders (sim-mjcf and sim-urdf). It handles loading model files
//! and spawning them into the physics world for visualization.
//!
//! # Design
//!
//! The model loading workflow is:
//! 1. Load the model file using sim-mjcf or sim-urdf
//! 2. Spawn the model into the sim-core World
//! 3. The existing `sync_physics_entities` system automatically creates Bevy entities
//!
//! This module provides helper types and systems to simplify this workflow.
//!
//! # Example
//!
//! ```no_run,ignore
//! use bevy::prelude::*;
//! use sim_bevy::prelude::*;
//! use sim_bevy::models::{MjcfModel, UrdfModel};
//!
//! fn setup(mut commands: Commands, mut sim_handle: ResMut<SimulationHandle>) {
//!     // Load and spawn an MJCF model
//!     let model = MjcfModel::from_file("models/humanoid.xml").unwrap();
//!     let spawned = model.spawn_into(&mut sim_handle).unwrap();
//!
//!     // Access spawned body/joint IDs by name
//!     if let Some(torso_id) = spawned.body_id("torso") {
//!         println!("Torso body ID: {:?}", torso_id);
//!     }
//! }
//! ```

#![allow(deprecated)] // This module uses deprecated World API types

use std::path::Path;

use bevy::prelude::*;
use sim_mjcf::{LoadedModel as MjcfLoadedModel, SpawnedModel as MjcfSpawnedModel};
use sim_types::{BodyId, JointId, Pose};
use sim_urdf::{LoadedRobot as UrdfLoadedRobot, SpawnedRobot as UrdfSpawnedRobot};
use thiserror::Error;

#[allow(deprecated)]
use crate::resources::SimulationHandle;

/// Errors that can occur during model loading.
#[derive(Error, Debug)]
pub enum ModelError {
    /// Failed to load MJCF file.
    #[error("MJCF loading error: {0}")]
    Mjcf(#[from] sim_mjcf::MjcfError),

    /// Failed to load URDF file.
    #[error("URDF loading error: {0}")]
    Urdf(#[from] sim_urdf::UrdfError),

    /// No physics world available.
    #[error("No physics world available in SimulationHandle")]
    NoWorld,

    /// File not found.
    #[error("File not found: {0}")]
    FileNotFound(String),

    /// IO error.
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
}

/// Result type for model operations.
pub type ModelResult<T> = Result<T, ModelError>;

/// An MJCF model ready to be spawned.
///
/// This wraps a `LoadedModel` from sim-mjcf and provides methods
/// for spawning it into a sim-bevy `SimulationHandle`.
#[derive(Debug)]
pub struct MjcfModel {
    loaded: MjcfLoadedModel,
}

impl MjcfModel {
    /// Load an MJCF model from a file path.
    ///
    /// # Errors
    ///
    /// Returns an error if the file cannot be read or parsed.
    pub fn from_file<P: AsRef<Path>>(path: P) -> ModelResult<Self> {
        let path = path.as_ref();
        if !path.exists() {
            return Err(ModelError::FileNotFound(path.display().to_string()));
        }
        let loaded = sim_mjcf::load_mjcf_file(path)?;
        Ok(Self { loaded })
    }

    /// Load an MJCF model from an XML string.
    ///
    /// # Errors
    ///
    /// Returns an error if the XML cannot be parsed.
    pub fn from_xml(xml: &str) -> ModelResult<Self> {
        let loaded = sim_mjcf::load_mjcf_str(xml)?;
        Ok(Self { loaded })
    }

    /// Create from an already-loaded model.
    #[must_use]
    pub fn from_loaded(loaded: MjcfLoadedModel) -> Self {
        Self { loaded }
    }

    /// Get the model name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.loaded.name
    }

    /// Get the number of bodies in the model.
    #[must_use]
    pub fn body_count(&self) -> usize {
        self.loaded.bodies.len()
    }

    /// Get the number of joints in the model.
    #[must_use]
    pub fn joint_count(&self) -> usize {
        self.loaded.joints.len()
    }

    /// Spawn the model into the physics world at the origin.
    ///
    /// # Errors
    ///
    /// Returns an error if no world is available or spawning fails.
    pub fn spawn_into(self, sim_handle: &mut SimulationHandle) -> ModelResult<SpawnedMjcf> {
        self.spawn_at(sim_handle, Pose::identity())
    }

    /// Spawn the model into the physics world at a specific pose.
    ///
    /// # Errors
    ///
    /// Returns an error if no world is available or spawning fails.
    pub fn spawn_at(
        self,
        sim_handle: &mut SimulationHandle,
        base_pose: Pose,
    ) -> ModelResult<SpawnedMjcf> {
        let world = sim_handle.world_mut().ok_or(ModelError::NoWorld)?;
        let model_name = self.loaded.name.clone();
        let spawned = self.loaded.spawn_into(world, base_pose)?;
        Ok(SpawnedMjcf {
            inner: spawned,
            model_name,
        })
    }

    /// Get access to the underlying loaded model.
    #[must_use]
    pub fn loaded(&self) -> &MjcfLoadedModel {
        &self.loaded
    }

    /// Consume and return the underlying loaded model.
    #[must_use]
    pub fn into_loaded(self) -> MjcfLoadedModel {
        self.loaded
    }
}

/// A spawned MJCF model with body and joint ID lookups.
#[derive(Debug)]
pub struct SpawnedMjcf {
    inner: MjcfSpawnedModel,
    model_name: String,
}

impl SpawnedMjcf {
    /// Get the body ID for a named body.
    #[must_use]
    pub fn body_id(&self, name: &str) -> Option<BodyId> {
        self.inner.body_id(name)
    }

    /// Get the joint ID for a named joint.
    #[must_use]
    pub fn joint_id(&self, name: &str) -> Option<JointId> {
        self.inner.joint_id(name)
    }

    /// Get the model name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.model_name
    }

    /// Get access to the underlying spawned model.
    #[must_use]
    pub fn inner(&self) -> &MjcfSpawnedModel {
        &self.inner
    }

    /// Consume and return the underlying spawned model.
    #[must_use]
    pub fn into_inner(self) -> MjcfSpawnedModel {
        self.inner
    }
}

/// A URDF robot model ready to be spawned.
///
/// This wraps a `LoadedRobot` from sim-urdf and provides methods
/// for spawning it into a sim-bevy `SimulationHandle`.
#[derive(Debug)]
pub struct UrdfModel {
    loaded: UrdfLoadedRobot,
}

impl UrdfModel {
    /// Load a URDF model from a file path.
    ///
    /// # Errors
    ///
    /// Returns an error if the file cannot be read or parsed.
    pub fn from_file<P: AsRef<Path>>(path: P) -> ModelResult<Self> {
        let path = path.as_ref();
        if !path.exists() {
            return Err(ModelError::FileNotFound(path.display().to_string()));
        }
        let loaded = sim_urdf::load_urdf_file(path)?;
        Ok(Self { loaded })
    }

    /// Load a URDF model from an XML string.
    ///
    /// # Errors
    ///
    /// Returns an error if the XML cannot be parsed.
    pub fn from_xml(xml: &str) -> ModelResult<Self> {
        let loaded = sim_urdf::load_urdf_str(xml)?;
        Ok(Self { loaded })
    }

    /// Create from an already-loaded robot.
    #[must_use]
    pub fn from_loaded(loaded: UrdfLoadedRobot) -> Self {
        Self { loaded }
    }

    /// Get the robot name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.loaded.name
    }

    /// Get the number of links (bodies) in the robot.
    #[must_use]
    pub fn link_count(&self) -> usize {
        self.loaded.bodies.len()
    }

    /// Get the number of joints in the robot.
    #[must_use]
    pub fn joint_count(&self) -> usize {
        self.loaded.joints.len()
    }

    /// Spawn the robot into the physics world at the origin.
    ///
    /// # Errors
    ///
    /// Returns an error if no world is available or spawning fails.
    pub fn spawn_into(self, sim_handle: &mut SimulationHandle) -> ModelResult<SpawnedUrdf> {
        self.spawn_at(sim_handle, Pose::identity())
    }

    /// Spawn the robot into the physics world at a specific pose.
    ///
    /// # Errors
    ///
    /// Returns an error if no world is available or spawning fails.
    pub fn spawn_at(
        self,
        sim_handle: &mut SimulationHandle,
        base_pose: Pose,
    ) -> ModelResult<SpawnedUrdf> {
        let world = sim_handle.world_mut().ok_or(ModelError::NoWorld)?;
        let robot_name = self.loaded.name.clone();
        let spawned = self.loaded.spawn_into(world, base_pose)?;
        Ok(SpawnedUrdf {
            inner: spawned,
            robot_name,
        })
    }

    /// Get access to the underlying loaded robot.
    #[must_use]
    pub fn loaded(&self) -> &UrdfLoadedRobot {
        &self.loaded
    }

    /// Consume and return the underlying loaded robot.
    #[must_use]
    pub fn into_loaded(self) -> UrdfLoadedRobot {
        self.loaded
    }
}

/// A spawned URDF robot with link and joint ID lookups.
#[derive(Debug)]
pub struct SpawnedUrdf {
    inner: UrdfSpawnedRobot,
    robot_name: String,
}

impl SpawnedUrdf {
    /// Get the body ID for a named link.
    #[must_use]
    pub fn link_id(&self, name: &str) -> Option<BodyId> {
        self.inner.link_id(name)
    }

    /// Get the joint ID for a named joint.
    #[must_use]
    pub fn joint_id(&self, name: &str) -> Option<JointId> {
        self.inner.joint_id(name)
    }

    /// Get the robot name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.robot_name
    }

    /// Get access to the underlying spawned robot.
    #[must_use]
    pub fn inner(&self) -> &UrdfSpawnedRobot {
        &self.inner
    }

    /// Consume and return the underlying spawned robot.
    #[must_use]
    pub fn into_inner(self) -> UrdfSpawnedRobot {
        self.inner
    }
}

/// Bevy component to track which model a body belongs to.
///
/// This is useful for identifying bodies when multiple models are loaded.
#[derive(Component, Debug, Clone)]
pub struct ModelSource {
    /// Name of the model this body came from.
    pub model_name: String,
    /// Type of model (MJCF or URDF).
    pub model_type: ModelType,
    /// Original name of this body/link in the model.
    pub body_name: Option<String>,
}

/// Type of model format.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ModelType {
    /// `MuJoCo` XML format.
    Mjcf,
    /// Unified Robot Description Format.
    Urdf,
}

impl ModelSource {
    /// Create a new MJCF model source.
    #[must_use]
    pub fn mjcf(model_name: impl Into<String>) -> Self {
        Self {
            model_name: model_name.into(),
            model_type: ModelType::Mjcf,
            body_name: None,
        }
    }

    /// Create a new URDF model source.
    #[must_use]
    pub fn urdf(model_name: impl Into<String>) -> Self {
        Self {
            model_name: model_name.into(),
            model_type: ModelType::Urdf,
            body_name: None,
        }
    }

    /// Set the body name.
    #[must_use]
    pub fn with_body_name(mut self, name: impl Into<String>) -> Self {
        self.body_name = Some(name.into());
        self
    }
}

#[cfg(test)]
#[allow(clippy::expect_used)]
mod tests {
    use super::*;

    const SIMPLE_MJCF: &str = r#"
        <mujoco model="test">
            <worldbody>
                <body name="box1" pos="0 0 1">
                    <geom type="box" size="0.5 0.5 0.5"/>
                    <joint type="free"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    const SIMPLE_URDF: &str = r#"
        <?xml version="1.0"?>
        <robot name="test_robot">
            <link name="base_link">
                <inertial>
                    <mass value="1.0"/>
                    <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
                </inertial>
                <collision>
                    <geometry>
                        <box size="1 1 1"/>
                    </geometry>
                </collision>
            </link>
        </robot>
    "#;

    #[test]
    fn mjcf_model_from_str() {
        let model = MjcfModel::from_xml(SIMPLE_MJCF);
        assert!(model.is_ok());
        let model = model.expect("should parse");
        assert_eq!(model.name(), "test");
        assert!(model.body_count() > 0);
    }

    #[test]
    fn urdf_model_from_str() {
        let model = UrdfModel::from_xml(SIMPLE_URDF);
        assert!(model.is_ok());
        let model = model.expect("should parse");
        assert_eq!(model.name(), "test_robot");
        assert_eq!(model.link_count(), 1);
    }

    #[test]
    fn mjcf_spawn_into_world() {
        let model = MjcfModel::from_xml(SIMPLE_MJCF).expect("should parse");
        let world = sim_core::World::default();
        let mut handle = SimulationHandle::new(world);

        let spawned = model.spawn_into(&mut handle);
        assert!(spawned.is_ok());

        let spawned = spawned.expect("should spawn");
        assert_eq!(spawned.name(), "test");
        assert!(spawned.body_id("box1").is_some());
    }

    #[test]
    fn urdf_spawn_into_world() {
        let model = UrdfModel::from_xml(SIMPLE_URDF).expect("should parse");
        let world = sim_core::World::default();
        let mut handle = SimulationHandle::new(world);

        let spawned = model.spawn_into(&mut handle);
        assert!(spawned.is_ok());

        let spawned = spawned.expect("should spawn");
        assert_eq!(spawned.name(), "test_robot");
        assert!(spawned.link_id("base_link").is_some());
    }

    #[test]
    fn spawn_fails_without_world() {
        let model = MjcfModel::from_xml(SIMPLE_MJCF).expect("should parse");
        let mut handle = SimulationHandle::empty();

        let result = model.spawn_into(&mut handle);
        assert!(matches!(result, Err(ModelError::NoWorld)));
    }

    #[test]
    fn model_source_component() {
        let source = ModelSource::mjcf("humanoid").with_body_name("torso");
        assert_eq!(source.model_name, "humanoid");
        assert_eq!(source.model_type, ModelType::Mjcf);
        assert_eq!(source.body_name, Some("torso".to_string()));
    }

    #[test]
    fn file_not_found_error() {
        let result = MjcfModel::from_file("/nonexistent/path/model.xml");
        assert!(matches!(result, Err(ModelError::FileNotFound(_))));
    }
}
