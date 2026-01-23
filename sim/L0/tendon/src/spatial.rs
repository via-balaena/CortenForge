//! Spatial tendons that route through 3D space.
//!
//! Spatial tendons represent physical cables that route through attachment
//! points on bodies. Unlike fixed tendons which use linear coupling coefficients,
//! spatial tendons compute their length from the actual 3D path through space.
//!
//! # MuJoCo Spatial Tendons
//!
//! MuJoCo's spatial tendons route through:
//! - **Sites**: Fixed attachment points on bodies
//! - **Pulleys**: Force direction change without path length change
//! - **Wrapping objects**: Spheres and cylinders the tendon can wrap around
//!
//! # Example
//!
//! ```
//! use sim_tendon::{SpatialTendon, SpatialTendonConfig, CableProperties};
//! use sim_tendon::path::{TendonPath, AttachmentPoint};
//! use sim_types::BodyId;
//! use nalgebra::Point3;
//!
//! // Create a spatial tendon from upper arm to forearm
//! let path = TendonPath::straight(
//!     BodyId::new(0),
//!     Point3::new(0.0, 0.0, 0.1),  // Upper arm attachment
//!     BodyId::new(1),
//!     Point3::new(0.0, 0.0, -0.1), // Forearm attachment
//! );
//!
//! let tendon = SpatialTendon::new("biceps_tendon", path)
//!     .with_cable(CableProperties::biological_tendon(20e-6)); // 20mm² cross-section
//! ```

use crate::{
    TendonActuator,
    cable::CableProperties,
    path::TendonPath,
    wrapping::{WrapResult, WrappingGeometry},
};
use nalgebra::{Isometry3, Vector3};
use sim_types::BodyId;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Configuration for a spatial tendon.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SpatialTendonConfig {
    /// Cable/tendon material properties.
    pub cable: CableProperties,

    /// Width of friction cone for via points (radians).
    ///
    /// Via points with small direction changes (< friction_cone) don't
    /// contribute friction. Larger changes cause friction losses.
    pub friction_cone: f64,

    /// Coefficient of friction at via points.
    pub friction_coefficient: f64,

    /// Whether the tendon can only pull (not push).
    pub one_way: bool,

    /// Range limits (min, max) for tendon length.
    pub range: Option<(f64, f64)>,
}

impl Default for SpatialTendonConfig {
    fn default() -> Self {
        Self {
            cable: CableProperties::default(),
            friction_cone: 0.1, // ~6 degrees
            friction_coefficient: 0.0,
            one_way: true,
            range: None,
        }
    }
}

impl SpatialTendonConfig {
    /// Create a configuration for a biological tendon.
    #[must_use]
    pub fn biological(cross_section_area: f64) -> Self {
        Self {
            cable: CableProperties::biological_tendon(cross_section_area),
            friction_cone: 0.2,
            friction_coefficient: 0.1,
            one_way: true,
            range: None,
        }
    }

    /// Create a configuration for a steel cable.
    #[must_use]
    pub fn steel(diameter: f64) -> Self {
        Self {
            cable: CableProperties::steel_cable(diameter),
            friction_cone: 0.05,
            friction_coefficient: 0.05,
            one_way: true,
            range: None,
        }
    }

    /// Set cable properties.
    #[must_use]
    pub fn with_cable(mut self, cable: CableProperties) -> Self {
        self.cable = cable;
        self
    }

    /// Set friction parameters.
    #[must_use]
    pub fn with_friction(mut self, cone: f64, coefficient: f64) -> Self {
        self.friction_cone = cone;
        self.friction_coefficient = coefficient;
        self
    }

    /// Set range limits.
    #[must_use]
    pub fn with_range(mut self, min: f64, max: f64) -> Self {
        self.range = Some((min, max));
        self
    }
}

/// A spatial tendon routing through 3D space.
///
/// Spatial tendons represent physical cables that route through attachment
/// points on bodies. The tendon length is computed from the actual 3D path.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SpatialTendon {
    /// Name of the tendon.
    name: String,

    /// The 3D path of the tendon.
    path: TendonPath,

    /// Configuration.
    config: SpatialTendonConfig,

    /// Wrapping geometries the tendon can wrap around.
    wrapping: Vec<WrappingGeometryRef>,

    /// Cached state.
    cached_length: f64,
    cached_velocity: f64,
}

/// Reference to a wrapping geometry with its body.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct WrappingGeometryRef {
    /// The body the wrapping geometry is attached to.
    pub body: BodyId,

    /// The wrapping geometry in the body's local frame.
    pub geometry: WrappingGeometry,

    /// Which path segment(s) this geometry can affect.
    ///
    /// If None, affects all segments.
    pub segment_range: Option<(usize, usize)>,
}

impl WrappingGeometryRef {
    /// Create a new wrapping geometry reference.
    #[must_use]
    pub fn new(body: BodyId, geometry: WrappingGeometry) -> Self {
        Self {
            body,
            geometry,
            segment_range: None,
        }
    }

    /// Set the segment range.
    #[must_use]
    pub fn with_segment_range(mut self, start: usize, end: usize) -> Self {
        self.segment_range = Some((start, end));
        self
    }
}

impl SpatialTendon {
    /// Create a new spatial tendon.
    #[must_use]
    pub fn new(name: impl Into<String>, path: TendonPath) -> Self {
        Self {
            name: name.into(),
            path,
            config: SpatialTendonConfig::default(),
            wrapping: Vec::new(),
            cached_length: 0.0,
            cached_velocity: 0.0,
        }
    }

    /// Set the configuration.
    #[must_use]
    pub fn with_config(mut self, config: SpatialTendonConfig) -> Self {
        self.config = config;
        self
    }

    /// Set cable properties.
    #[must_use]
    pub fn with_cable(mut self, cable: CableProperties) -> Self {
        self.config.cable = cable;
        self
    }

    /// Add a wrapping geometry.
    #[must_use]
    pub fn with_wrapping(mut self, body: BodyId, geometry: WrappingGeometry) -> Self {
        self.wrapping.push(WrappingGeometryRef::new(body, geometry));
        self
    }

    /// Add a wrapping geometry reference.
    #[must_use]
    pub fn with_wrapping_ref(mut self, wrap_ref: WrappingGeometryRef) -> Self {
        self.wrapping.push(wrap_ref);
        self
    }

    /// Get the tendon name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get the path.
    #[must_use]
    pub fn path(&self) -> &TendonPath {
        &self.path
    }

    /// Get the path mutably.
    #[must_use]
    pub fn path_mut(&mut self) -> &mut TendonPath {
        &mut self.path
    }

    /// Get the configuration.
    #[must_use]
    pub fn config(&self) -> &SpatialTendonConfig {
        &self.config
    }

    /// Get the wrapping geometries.
    #[must_use]
    pub fn wrapping(&self) -> &[WrappingGeometryRef] {
        &self.wrapping
    }

    /// Get all bodies this tendon connects to.
    #[must_use]
    pub fn connected_bodies(&self) -> Vec<BodyId> {
        let mut bodies = self.path.bodies();

        for wrap in &self.wrapping {
            if !bodies.contains(&wrap.body) {
                bodies.push(wrap.body);
            }
        }

        bodies
    }

    /// Update the tendon state from body transforms and velocities.
    ///
    /// This must be called each simulation step before computing forces.
    pub fn update<F, G>(&mut self, get_transform: F, get_velocity: G)
    where
        F: Fn(BodyId) -> Isometry3<f64>,
        G: Fn(BodyId) -> (Vector3<f64>, Vector3<f64>),
    {
        // Compute path length (with wrapping if applicable)
        self.cached_length = self.compute_wrapped_length(&get_transform);
        self.cached_velocity = self.path.compute_velocity(&get_transform, &get_velocity);
    }

    /// Compute the path length including wrapping.
    fn compute_wrapped_length<F>(&mut self, get_transform: &F) -> f64
    where
        F: Fn(BodyId) -> Isometry3<f64>,
    {
        if self.wrapping.is_empty() {
            // No wrapping - just straight path
            return self.path.compute_length(get_transform);
        }

        // With wrapping, we need to check each segment against wrapping geometries
        // This is a simplified implementation - full implementation would need
        // to compute actual wrap paths

        let base_length = self.path.compute_length(get_transform);

        // For each wrapping geometry, check if the path wraps around it
        let mut additional_length = 0.0;

        for wrap_ref in &self.wrapping {
            let wrap_transform = get_transform(wrap_ref.body);
            let world_geometry = wrap_ref.geometry.transform(&wrap_transform);

            // Check each segment
            for (i, segment) in self.path.segments().iter().enumerate() {
                // Skip if not in segment range
                if let Some((start, end)) = wrap_ref.segment_range {
                    if i < start || i > end {
                        continue;
                    }
                }

                // Get segment endpoints (from cached path computation)
                if let (Some(start_point), Some(end_point)) = (
                    self.path.point(segment.start_index),
                    self.path.point(segment.end_index),
                ) {
                    let start_world = start_point.world_position(&get_transform(start_point.body));
                    let end_world = end_point.world_position(&get_transform(end_point.body));

                    // Compute wrap
                    let wrap_result = world_geometry.compute_wrap(&start_world, &end_world);

                    if let WrapResult::Wrapped { arc_length, .. } = wrap_result {
                        // Replace straight segment with wrap path
                        let straight_length = (end_world - start_world).norm();
                        additional_length += arc_length - straight_length;
                    }
                }
            }
        }

        base_length + additional_length
    }

    /// Compute the tendon tension from body states.
    ///
    /// This is the primary method for computing tendon force in simulations
    /// where body transforms are available.
    ///
    /// # Arguments
    ///
    /// * `get_transform` - Function to get body transforms
    /// * `get_velocity` - Function to get body velocities
    ///
    /// # Returns
    ///
    /// Tendon tension in Newtons.
    pub fn compute_tension<F, G>(&mut self, get_transform: F, get_velocity: G) -> f64
    where
        F: Fn(BodyId) -> Isometry3<f64>,
        G: Fn(BodyId) -> (Vector3<f64>, Vector3<f64>),
    {
        self.update(&get_transform, &get_velocity);

        // Compute cable force
        let force = self
            .config
            .cable
            .compute_force(self.cached_length, self.cached_velocity);

        // Apply one-way constraint
        if self.config.one_way {
            force.max(0.0)
        } else {
            force
        }
    }

    /// Compute forces on all connected bodies.
    ///
    /// # Arguments
    ///
    /// * `tension` - Tendon tension in Newtons
    /// * `get_transform` - Function to get body transforms
    ///
    /// # Returns
    ///
    /// Vector of (body_id, force, torque) tuples.
    #[must_use]
    pub fn compute_body_forces<F>(
        &self,
        tension: f64,
        get_transform: F,
    ) -> Vec<(BodyId, Vector3<f64>, Vector3<f64>)>
    where
        F: Fn(BodyId) -> Isometry3<f64>,
    {
        self.path.compute_forces(tension, get_transform)
    }

    /// Get the cached length.
    #[must_use]
    pub fn cached_length(&self) -> f64 {
        self.cached_length
    }

    /// Get the cached velocity.
    #[must_use]
    pub fn cached_velocity(&self) -> f64 {
        self.cached_velocity
    }
}

impl TendonActuator for SpatialTendon {
    fn rest_length(&self) -> f64 {
        self.config.cable.rest_length
    }

    fn compute_length(&self, _joint_positions: &[f64]) -> f64 {
        // For spatial tendons, length depends on body poses, not joint positions directly
        // Return cached length (must call update() first)
        self.cached_length
    }

    fn compute_velocity(&self, _joint_positions: &[f64], _joint_velocities: &[f64]) -> f64 {
        // Return cached velocity
        self.cached_velocity
    }

    fn compute_force(&self, _joint_positions: &[f64], _joint_velocities: &[f64]) -> f64 {
        // Compute force from cached state
        let force = self
            .config
            .cable
            .compute_force(self.cached_length, self.cached_velocity);

        if self.config.one_way {
            force.max(0.0)
        } else {
            force
        }
    }

    fn jacobian(&self, _joint_positions: &[f64]) -> Vec<f64> {
        // For spatial tendons, the Jacobian depends on the current configuration
        // and must be computed numerically or through the path geometry.
        // For now, return empty (must use compute_body_forces instead)
        Vec::new()
    }

    fn num_joints(&self) -> usize {
        // Spatial tendons don't directly couple to joints
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::path::AttachmentPoint;
    use approx::assert_relative_eq;
    use nalgebra::Point3;

    fn identity_transform(_: BodyId) -> Isometry3<f64> {
        Isometry3::identity()
    }

    fn zero_velocity(_: BodyId) -> (Vector3<f64>, Vector3<f64>) {
        (Vector3::zeros(), Vector3::zeros())
    }

    #[test]
    fn test_spatial_tendon_creation() {
        let path = TendonPath::straight(
            BodyId::new(0),
            Point3::origin(),
            BodyId::new(1),
            Point3::new(1.0, 0.0, 0.0),
        );

        let tendon =
            SpatialTendon::new("test", path).with_cable(CableProperties::new(10000.0, 1.0));

        assert_eq!(tendon.name(), "test");
        assert_eq!(tendon.path().num_points(), 2);
    }

    #[test]
    fn test_spatial_tendon_length() {
        let path = TendonPath::straight(
            BodyId::new(0),
            Point3::origin(),
            BodyId::new(1),
            Point3::new(1.0, 0.0, 0.0),
        );

        let mut tendon =
            SpatialTendon::new("test", path).with_cable(CableProperties::new(10000.0, 1.0));

        tendon.update(identity_transform, zero_velocity);

        assert_relative_eq!(tendon.cached_length(), 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_spatial_tendon_force() {
        let path = TendonPath::straight(
            BodyId::new(0),
            Point3::origin(),
            BodyId::new(1),
            Point3::new(1.0, 0.0, 0.0),
        );

        // Rest length = 0.5, current length = 1.0, stretch = 0.5
        // Force = 10000 * 0.5 = 5000 N
        let mut tendon = SpatialTendon::new("test", path)
            .with_cable(CableProperties::new(10000.0, 0.5).with_damping(0.0));

        let force = tendon.compute_tension(identity_transform, zero_velocity);

        assert_relative_eq!(force, 5000.0, epsilon = 1e-10);
    }

    #[test]
    fn test_spatial_tendon_slack() {
        let path = TendonPath::straight(
            BodyId::new(0),
            Point3::origin(),
            BodyId::new(1),
            Point3::new(0.3, 0.0, 0.0),
        );

        // Rest length = 0.5, current length = 0.3 -> slack
        let mut tendon =
            SpatialTendon::new("test", path).with_cable(CableProperties::new(10000.0, 0.5));

        let force = tendon.compute_tension(identity_transform, zero_velocity);

        assert_relative_eq!(force, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_spatial_tendon_with_via_point() {
        let path = TendonPath::straight(
            BodyId::new(0),
            Point3::origin(),
            BodyId::new(2),
            Point3::new(2.0, 0.0, 0.0),
        )
        .with_via_point(AttachmentPoint::new(
            BodyId::new(1),
            Point3::new(1.0, 1.0, 0.0),
        ));

        let mut tendon = SpatialTendon::new("test", path);

        tendon.update(identity_transform, zero_velocity);

        // Path: (0,0,0) -> (1,1,0) -> (2,0,0)
        // Length: sqrt(2) + sqrt(2) = 2*sqrt(2) ≈ 2.828
        let expected = 2.0 * std::f64::consts::SQRT_2;
        assert_relative_eq!(tendon.cached_length(), expected, epsilon = 1e-10);
    }

    #[test]
    fn test_connected_bodies() {
        let path = TendonPath::straight(
            BodyId::new(0),
            Point3::origin(),
            BodyId::new(1),
            Point3::new(1.0, 0.0, 0.0),
        );

        let tendon = SpatialTendon::new("test", path)
            .with_wrapping(BodyId::new(2), WrappingGeometry::sphere(0.1));

        let bodies = tendon.connected_bodies();

        assert!(bodies.contains(&BodyId::new(0)));
        assert!(bodies.contains(&BodyId::new(1)));
        assert!(bodies.contains(&BodyId::new(2)));
    }

    #[test]
    fn test_spatial_tendon_config() {
        let config = SpatialTendonConfig::biological(20e-6);

        assert!(config.cable.stiffness > 0.0);
        assert!(config.friction_coefficient > 0.0);
        assert!(config.one_way);
    }
}
