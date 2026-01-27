//! Pulley systems for cable-driven robots.
//!
//! Pulleys redirect cable forces and can provide mechanical advantage.
//! This module provides components for building pulley systems.
//!
//! # Pulley Types
//!
//! - **Idler pulley**: Redirects cable without changing tension
//! - **Fixed pulley**: Mounted on world/ground, changes force direction
//! - **Moving pulley**: Attached to a load, provides 2:1 mechanical advantage
//! - **Compound pulley**: Multiple pulleys for higher mechanical advantage
//!
//! # Example
//!
//! ```
//! use sim_tendon::pulley::{Pulley, PulleySystem, PulleyConfig};
//! use sim_types::BodyId;
//! use nalgebra::{Point3, Vector3};
//!
//! // Create a simple 2:1 pulley system
//! let system = PulleySystem::new("hoist")
//!     .with_fixed_pulley(Point3::new(0.0, 0.0, 2.0), Vector3::z(), 0.05)
//!     .with_moving_pulley(BodyId::new(1), Point3::origin(), Vector3::z(), 0.05);
//! ```

use nalgebra::{Isometry3, Point3, Vector3};
use sim_types::BodyId;

/// Safe axis normalization with Z fallback.
#[inline]
fn safe_normalize_axis(v: Vector3<f64>) -> Vector3<f64> {
    let n = v.norm();
    if n > 1e-10 { v / n } else { Vector3::z() }
}

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Configuration for a single pulley.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PulleyConfig {
    /// Radius of the pulley sheave.
    pub radius: f64,

    /// Coefficient of friction at the bearing.
    pub bearing_friction: f64,

    /// Mass of the pulley (for dynamics).
    pub mass: f64,

    /// Moment of inertia about the axis.
    pub inertia: f64,

    /// Wrapping friction coefficient (cable-to-sheave).
    pub wrap_friction: f64,
}

impl Default for PulleyConfig {
    fn default() -> Self {
        Self {
            radius: 0.05, // 5cm radius
            bearing_friction: 0.01,
            mass: 0.1,          // 100g
            inertia: 0.0001,    // Small inertia
            wrap_friction: 0.0, // Ideal pulley
        }
    }
}

impl PulleyConfig {
    /// Create a new pulley configuration.
    #[must_use]
    pub fn new(radius: f64) -> Self {
        Self {
            radius,
            ..Self::default()
        }
    }

    /// Set the bearing friction.
    #[must_use]
    pub fn with_bearing_friction(mut self, friction: f64) -> Self {
        self.bearing_friction = friction.max(0.0);
        self
    }

    /// Set the mass.
    #[must_use]
    pub fn with_mass(mut self, mass: f64) -> Self {
        self.mass = mass.max(0.0);
        // Update inertia assuming solid cylinder
        self.inertia = 0.5 * mass * self.radius * self.radius;
        self
    }

    /// Set the wrap friction (Capstan effect).
    #[must_use]
    pub fn with_wrap_friction(mut self, friction: f64) -> Self {
        self.wrap_friction = friction.max(0.0);
        self
    }

    /// Compute friction losses through the pulley.
    ///
    /// # Arguments
    ///
    /// * `tension_in` - Incoming cable tension
    /// * `wrap_angle` - Angle the cable wraps around the pulley (radians)
    ///
    /// # Returns
    ///
    /// Outgoing cable tension after friction losses.
    #[must_use]
    pub fn apply_friction(&self, tension_in: f64, wrap_angle: f64) -> f64 {
        if self.wrap_friction <= 0.0 {
            return tension_in;
        }

        // Capstan equation: T_out = T_in * e^(-μθ)
        let friction_factor = (-self.wrap_friction * wrap_angle.abs()).exp();
        tension_in * friction_factor
    }
}

/// A single pulley in a pulley system.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Pulley {
    /// Configuration.
    pub config: PulleyConfig,

    /// Whether this is a fixed (world) or moving (body-attached) pulley.
    pub pulley_type: PulleyType,

    /// Position in the attachment frame (world for fixed, local for moving).
    pub position: Point3<f64>,

    /// Rotation axis of the pulley.
    pub axis: Vector3<f64>,
}

/// Type of pulley attachment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum PulleyType {
    /// Fixed to the world.
    Fixed,

    /// Attached to a body.
    Moving(BodyId),
}

impl Pulley {
    /// Create a fixed pulley.
    #[must_use]
    pub fn fixed(position: Point3<f64>, axis: Vector3<f64>, radius: f64) -> Self {
        Self {
            config: PulleyConfig::new(radius),
            pulley_type: PulleyType::Fixed,
            position,
            axis: safe_normalize_axis(axis),
        }
    }

    /// Create a moving pulley attached to a body.
    #[must_use]
    pub fn moving(body: BodyId, local_pos: Point3<f64>, axis: Vector3<f64>, radius: f64) -> Self {
        Self {
            config: PulleyConfig::new(radius),
            pulley_type: PulleyType::Moving(body),
            position: local_pos,
            axis: safe_normalize_axis(axis),
        }
    }

    /// Set the configuration.
    #[must_use]
    pub fn with_config(mut self, config: PulleyConfig) -> Self {
        self.config = config;
        self
    }

    /// Get the world position of the pulley center.
    ///
    /// # Arguments
    ///
    /// * `get_transform` - Function to get body transforms
    #[must_use]
    pub fn world_position<F>(&self, get_transform: &F) -> Point3<f64>
    where
        F: Fn(BodyId) -> Isometry3<f64>,
    {
        match self.pulley_type {
            PulleyType::Fixed => self.position,
            PulleyType::Moving(body) => {
                let transform = get_transform(body);
                transform * self.position
            }
        }
    }

    /// Get the world axis of the pulley.
    #[must_use]
    pub fn world_axis<F>(&self, get_transform: &F) -> Vector3<f64>
    where
        F: Fn(BodyId) -> Isometry3<f64>,
    {
        match self.pulley_type {
            PulleyType::Fixed => self.axis,
            PulleyType::Moving(body) => {
                let transform = get_transform(body);
                transform.rotation * self.axis
            }
        }
    }

    /// Compute the wrap angle for a cable passing over this pulley.
    ///
    /// # Arguments
    ///
    /// * `incoming_dir` - Direction cable arrives from (toward pulley)
    /// * `outgoing_dir` - Direction cable leaves to (away from pulley)
    ///
    /// # Returns
    ///
    /// Wrap angle in radians.
    #[must_use]
    pub fn compute_wrap_angle(
        &self,
        incoming_dir: &Vector3<f64>,
        outgoing_dir: &Vector3<f64>,
    ) -> f64 {
        // Project directions onto plane perpendicular to axis
        let proj_in = incoming_dir - self.axis * incoming_dir.dot(&self.axis);
        let proj_out = outgoing_dir - self.axis * outgoing_dir.dot(&self.axis);

        let in_norm = proj_in.norm();
        let out_norm = proj_out.norm();

        if in_norm < 1e-10 || out_norm < 1e-10 {
            return std::f64::consts::PI; // Assume half wrap if degenerate
        }

        let cos_angle = (proj_in.dot(&proj_out)) / (in_norm * out_norm);
        let angle = cos_angle.clamp(-1.0, 1.0).acos();

        // Wrap angle is π minus the angle between directions
        std::f64::consts::PI - angle
    }

    /// Get the body this pulley is attached to (None if fixed).
    #[must_use]
    pub fn attached_body(&self) -> Option<BodyId> {
        match self.pulley_type {
            PulleyType::Fixed => None,
            PulleyType::Moving(body) => Some(body),
        }
    }

    /// Check if this is a fixed pulley.
    #[must_use]
    pub fn is_fixed(&self) -> bool {
        matches!(self.pulley_type, PulleyType::Fixed)
    }

    /// Check if this is a moving pulley.
    #[must_use]
    pub fn is_moving(&self) -> bool {
        matches!(self.pulley_type, PulleyType::Moving(_))
    }
}

/// A complete pulley system with multiple pulleys.
///
/// Pulley systems model the interaction of cables with multiple pulleys,
/// computing the effective mechanical advantage and cable routing.
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PulleySystem {
    /// Name of the system.
    name: String,

    /// Pulleys in order from input to output.
    pulleys: Vec<Pulley>,

    /// Anchor point for the input (fixed end of cable).
    input_anchor: Option<Point3<f64>>,

    /// Anchor point for the output (load attachment).
    output_anchor: Option<(BodyId, Point3<f64>)>,
}

impl PulleySystem {
    /// Create a new pulley system.
    #[must_use]
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            pulleys: Vec::new(),
            input_anchor: None,
            output_anchor: None,
        }
    }

    /// Add a fixed pulley.
    #[must_use]
    pub fn with_fixed_pulley(
        mut self,
        position: Point3<f64>,
        axis: Vector3<f64>,
        radius: f64,
    ) -> Self {
        self.pulleys.push(Pulley::fixed(position, axis, radius));
        self
    }

    /// Add a moving pulley.
    #[must_use]
    pub fn with_moving_pulley(
        mut self,
        body: BodyId,
        local_pos: Point3<f64>,
        axis: Vector3<f64>,
        radius: f64,
    ) -> Self {
        self.pulleys
            .push(Pulley::moving(body, local_pos, axis, radius));
        self
    }

    /// Add a pulley.
    #[must_use]
    pub fn with_pulley(mut self, pulley: Pulley) -> Self {
        self.pulleys.push(pulley);
        self
    }

    /// Set the input anchor (where the cable starts).
    #[must_use]
    pub fn with_input_anchor(mut self, position: Point3<f64>) -> Self {
        self.input_anchor = Some(position);
        self
    }

    /// Set the output anchor (where the cable attaches to the load).
    #[must_use]
    pub fn with_output_anchor(mut self, body: BodyId, local_pos: Point3<f64>) -> Self {
        self.output_anchor = Some((body, local_pos));
        self
    }

    /// Get the system name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get the pulleys.
    #[must_use]
    pub fn pulleys(&self) -> &[Pulley] {
        &self.pulleys
    }

    /// Get the number of pulleys.
    #[must_use]
    pub fn num_pulleys(&self) -> usize {
        self.pulleys.len()
    }

    /// Compute the mechanical advantage of the system.
    ///
    /// For ideal pulleys:
    /// - Each fixed pulley provides 1:1 advantage
    /// - Each moving pulley provides 2:1 advantage
    #[must_use]
    pub fn mechanical_advantage(&self) -> f64 {
        let mut advantage = 1.0;

        for pulley in &self.pulleys {
            if pulley.is_moving() {
                advantage *= 2.0;
            }
        }

        advantage
    }

    /// Compute the total cable length through the system.
    ///
    /// # Arguments
    ///
    /// * `get_transform` - Function to get body transforms
    ///
    /// # Returns
    ///
    /// Total cable length in meters.
    pub fn compute_cable_length<F>(&self, get_transform: F) -> f64
    where
        F: Fn(BodyId) -> Isometry3<f64>,
    {
        if self.pulleys.is_empty() {
            return 0.0;
        }

        let mut total_length = 0.0;
        let mut prev_point: Option<Point3<f64>> = self.input_anchor;

        // Length through pulleys
        for pulley in &self.pulleys {
            let pulley_center = pulley.world_position(&get_transform);

            if let Some(prev) = prev_point {
                // Segment to pulley
                total_length += (pulley_center - prev).norm();

                // Add wrap length around pulley (approximate)
                // In reality this depends on neighboring segments
                total_length += pulley.config.radius * std::f64::consts::PI;
            }

            prev_point = Some(pulley_center);
        }

        // Length to output
        if let (Some(prev), Some((body, local_pos))) = (prev_point, &self.output_anchor) {
            let output_world = get_transform(*body) * local_pos;
            total_length += (output_world - prev).norm();
        }

        total_length
    }

    /// Compute the output force given input force.
    ///
    /// # Arguments
    ///
    /// * `input_force` - Force applied to the input cable
    ///
    /// # Returns
    ///
    /// Force transmitted to the output, accounting for friction.
    #[must_use]
    pub fn compute_output_force(&self, input_force: f64) -> f64 {
        let mut force = input_force;

        // Apply friction losses through each pulley
        for pulley in &self.pulleys {
            // Assume 180 degree wrap for simplicity
            force = pulley.config.apply_friction(force, std::f64::consts::PI);

            // Moving pulleys multiply force
            if pulley.is_moving() {
                force *= 2.0;
            }
        }

        force
    }

    /// Get all bodies this system connects to.
    #[must_use]
    pub fn connected_bodies(&self) -> Vec<BodyId> {
        let mut bodies = Vec::new();

        for pulley in &self.pulleys {
            if let Some(body) = pulley.attached_body() {
                if !bodies.contains(&body) {
                    bodies.push(body);
                }
            }
        }

        if let Some((body, _)) = &self.output_anchor {
            if !bodies.contains(body) {
                bodies.push(*body);
            }
        }

        bodies
    }
}

/// Builder for creating common pulley configurations.
pub struct PulleyBuilder;

impl PulleyBuilder {
    /// Create a simple 2:1 block and tackle.
    ///
    /// One fixed pulley and one moving pulley.
    #[must_use]
    pub fn block_and_tackle_2_1(
        fixed_pos: Point3<f64>,
        moving_body: BodyId,
        moving_local_pos: Point3<f64>,
        radius: f64,
    ) -> PulleySystem {
        PulleySystem::new("block_and_tackle_2_1")
            .with_fixed_pulley(fixed_pos, Vector3::z(), radius)
            .with_moving_pulley(moving_body, moving_local_pos, Vector3::z(), radius)
    }

    /// Create a 4:1 compound pulley system.
    ///
    /// Two fixed and two moving pulleys.
    #[must_use]
    pub fn compound_4_1(
        fixed_pos_1: Point3<f64>,
        fixed_pos_2: Point3<f64>,
        moving_body: BodyId,
        moving_local_pos_1: Point3<f64>,
        moving_local_pos_2: Point3<f64>,
        radius: f64,
    ) -> PulleySystem {
        PulleySystem::new("compound_4_1")
            .with_fixed_pulley(fixed_pos_1, Vector3::z(), radius)
            .with_moving_pulley(moving_body, moving_local_pos_1, Vector3::z(), radius)
            .with_fixed_pulley(fixed_pos_2, Vector3::z(), radius)
            .with_moving_pulley(moving_body, moving_local_pos_2, Vector3::z(), radius)
    }

    /// Create a direction-change pulley (fixed, no mechanical advantage).
    #[must_use]
    pub fn direction_change(
        position: Point3<f64>,
        axis: Vector3<f64>,
        radius: f64,
    ) -> PulleySystem {
        PulleySystem::new("direction_change").with_fixed_pulley(position, axis, radius)
    }
}

#[cfg(test)]
#[allow(dead_code)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn identity_transform(_: BodyId) -> Isometry3<f64> {
        Isometry3::identity()
    }

    #[test]
    fn test_pulley_config() {
        let config = PulleyConfig::new(0.05)
            .with_bearing_friction(0.02)
            .with_mass(0.2);

        assert_relative_eq!(config.radius, 0.05, epsilon = 1e-10);
        assert_relative_eq!(config.bearing_friction, 0.02, epsilon = 1e-10);
        assert_relative_eq!(config.mass, 0.2, epsilon = 1e-10);
    }

    #[test]
    fn test_fixed_pulley() {
        let pulley = Pulley::fixed(Point3::new(0.0, 0.0, 2.0), Vector3::z(), 0.05);

        assert!(pulley.is_fixed());
        assert!(!pulley.is_moving());
        assert_eq!(pulley.attached_body(), None);
    }

    #[test]
    fn test_moving_pulley() {
        let pulley = Pulley::moving(BodyId::new(1), Point3::origin(), Vector3::z(), 0.05);

        assert!(!pulley.is_fixed());
        assert!(pulley.is_moving());
        assert_eq!(pulley.attached_body(), Some(BodyId::new(1)));
    }

    #[test]
    fn test_pulley_world_position() {
        let pulley = Pulley::moving(
            BodyId::new(0),
            Point3::new(0.0, 0.0, 0.1),
            Vector3::z(),
            0.05,
        );

        let get_transform = |_| Isometry3::translation(1.0, 0.0, 0.0);

        let world_pos = pulley.world_position(&get_transform);
        assert_relative_eq!(world_pos.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(world_pos.z, 0.1, epsilon = 1e-10);
    }

    #[test]
    fn test_wrap_angle() {
        let pulley = Pulley::fixed(Point3::origin(), Vector3::z(), 0.05);

        // Straight through - no wrap (0 degrees)
        // incoming = -X, outgoing = +X, same direction -> angle = π, wrap = π - π = 0
        let angle =
            pulley.compute_wrap_angle(&Vector3::new(-1.0, 0.0, 0.0), &Vector3::new(1.0, 0.0, 0.0));
        assert_relative_eq!(angle, 0.0, epsilon = 1e-10);

        // 90 degree turn (90 degree wrap)
        // incoming = -X, outgoing = +Y, perpendicular -> angle = π/2, wrap = π - π/2 = π/2
        let angle =
            pulley.compute_wrap_angle(&Vector3::new(-1.0, 0.0, 0.0), &Vector3::new(0.0, 1.0, 0.0));
        assert_relative_eq!(angle, std::f64::consts::FRAC_PI_2, epsilon = 1e-10);

        // U-turn (180 degree wrap)
        // incoming = -X, outgoing = -X (same direction), angle = 0, wrap = π - 0 = π
        let angle =
            pulley.compute_wrap_angle(&Vector3::new(-1.0, 0.0, 0.0), &Vector3::new(-1.0, 0.0, 0.0));
        assert_relative_eq!(angle, std::f64::consts::PI, epsilon = 1e-10);
    }

    #[test]
    fn test_pulley_system_mechanical_advantage() {
        let system = PulleySystem::new("test")
            .with_fixed_pulley(Point3::new(0.0, 0.0, 2.0), Vector3::z(), 0.05)
            .with_moving_pulley(BodyId::new(1), Point3::origin(), Vector3::z(), 0.05);

        // 1 moving pulley = 2:1 advantage
        assert_relative_eq!(system.mechanical_advantage(), 2.0, epsilon = 1e-10);
    }

    #[test]
    fn test_compound_advantage() {
        let system = PulleyBuilder::compound_4_1(
            Point3::new(0.0, 0.0, 2.0),
            Point3::new(0.2, 0.0, 2.0),
            BodyId::new(1),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.2, 0.0, 0.0),
            0.05,
        );

        // 2 moving pulleys = 4:1 advantage
        assert_relative_eq!(system.mechanical_advantage(), 4.0, epsilon = 1e-10);
    }

    #[test]
    fn test_output_force_ideal() {
        let system = PulleySystem::new("test").with_moving_pulley(
            BodyId::new(1),
            Point3::origin(),
            Vector3::z(),
            0.05,
        );

        // 2:1 advantage with no friction
        let output = system.compute_output_force(100.0);
        assert_relative_eq!(output, 200.0, epsilon = 1e-10);
    }

    #[test]
    fn test_friction_loss() {
        let config = PulleyConfig::new(0.05).with_wrap_friction(0.1);

        // Capstan equation: T_out = T_in * e^(-μθ)
        // For 180 degree wrap (π radians) and μ=0.1:
        // T_out = 100 * e^(-0.1 * π) ≈ 73.0
        let output = config.apply_friction(100.0, std::f64::consts::PI);
        assert!(output < 100.0);
        assert!(output > 70.0);
    }

    #[test]
    fn test_connected_bodies() {
        let system = PulleySystem::new("test")
            .with_moving_pulley(BodyId::new(1), Point3::origin(), Vector3::z(), 0.05)
            .with_moving_pulley(BodyId::new(2), Point3::origin(), Vector3::z(), 0.05)
            .with_output_anchor(BodyId::new(3), Point3::origin());

        let bodies = system.connected_bodies();

        assert_eq!(bodies.len(), 3);
        assert!(bodies.contains(&BodyId::new(1)));
        assert!(bodies.contains(&BodyId::new(2)));
        assert!(bodies.contains(&BodyId::new(3)));
    }
}
