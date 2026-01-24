//! Cable properties and state for tendon simulation.
//!
//! This module provides material properties and state tracking for cables/tendons.
//!
//! # Cable Model
//!
//! Cables are modeled as one-way springs that can only pull, not push:
//!
//! ```text
//! F = {
//!     k(L - L₀) + c·v    if L > L₀ (stretched)
//!     0                   if L ≤ L₀ (slack)
//! }
//! ```
//!
//! Where:
//! - `k` is the stiffness (N/m)
//! - `L` is current length
//! - `L₀` is rest length
//! - `c` is damping coefficient
//! - `v` is elongation velocity

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Material and mechanical properties of a cable/tendon.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CableProperties {
    /// Stiffness (spring constant) in N/m.
    ///
    /// Typical values:
    /// - Steel cable: 100,000 - 200,000 N/m per mm² cross-section
    /// - Dyneema: 50,000 - 100,000 N/m per mm²
    /// - Biological tendon: 1,000 - 2,000 N/m per mm²
    pub stiffness: f64,

    /// Damping coefficient in N·s/m.
    ///
    /// Provides viscous damping for numerical stability.
    /// Typical: 1-10% of critical damping.
    pub damping: f64,

    /// Rest length (unstretched length) in meters.
    ///
    /// The length at which the cable produces zero force.
    pub rest_length: f64,

    /// Maximum tension before failure, in Newtons.
    ///
    /// Set to `f64::INFINITY` for indestructible cables.
    pub max_tension: f64,

    /// Minimum length (for slack prevention), in meters.
    ///
    /// If the cable shortens below this length, it's considered slack
    /// and produces no force.
    pub min_length: f64,

    /// Cable diameter in meters (for visualization/wrapping).
    pub diameter: f64,
}

impl Default for CableProperties {
    fn default() -> Self {
        Self {
            stiffness: 10_000.0, // 10 kN/m - moderate stiffness
            damping: 100.0,      // 100 N·s/m - light damping
            rest_length: 0.5,    // 50cm default
            max_tension: f64::INFINITY,
            min_length: 0.0,
            diameter: 0.002, // 2mm diameter
        }
    }
}

impl CableProperties {
    /// Create cable properties with specified stiffness and rest length.
    #[must_use]
    pub fn new(stiffness: f64, rest_length: f64) -> Self {
        Self {
            stiffness,
            rest_length,
            ..Self::default()
        }
    }

    /// Create properties for a steel cable.
    ///
    /// # Arguments
    ///
    /// * `diameter` - Cable diameter in meters
    #[must_use]
    pub fn steel_cable(diameter: f64) -> Self {
        // Steel: E ≈ 200 GPa, typical cable is 60-80% of solid
        // Effective E ≈ 120-160 GPa
        let area = std::f64::consts::PI * (diameter / 2.0).powi(2);
        let young_modulus = 140e9; // 140 GPa effective modulus
        let reference_length = 1.0; // per meter of length

        Self {
            stiffness: young_modulus * area / reference_length,
            damping: 0.01 * (2.0 * (young_modulus * area).sqrt()), // 1% critical damping
            rest_length: 0.5,
            max_tension: area * 1200e6, // ~1200 MPa yield strength
            min_length: 0.0,
            diameter,
        }
    }

    /// Create properties for a Dyneema (UHMWPE) cable.
    ///
    /// Dyneema is stronger than steel per unit weight.
    #[must_use]
    pub fn dyneema_cable(diameter: f64) -> Self {
        let area = std::f64::consts::PI * (diameter / 2.0).powi(2);
        let young_modulus = 100e9; // 100 GPa

        Self {
            stiffness: young_modulus * area,
            damping: 0.02 * (2.0 * (young_modulus * area).sqrt()), // 2% critical damping
            rest_length: 0.5,
            max_tension: area * 3000e6, // ~3 GPa strength
            min_length: 0.0,
            diameter,
        }
    }

    /// Create properties for a biological tendon.
    ///
    /// Biological tendons have nonlinear stiffness; this uses a linearized
    /// approximation around typical operating strain.
    #[must_use]
    pub fn biological_tendon(cross_section_area: f64) -> Self {
        // Tendon: E ≈ 1-2 GPa in linear region
        let young_modulus = 1.5e9;

        Self {
            stiffness: young_modulus * cross_section_area,
            damping: 0.05 * (2.0 * (young_modulus * cross_section_area).sqrt()),
            rest_length: 0.1,                        // 10cm default
            max_tension: cross_section_area * 100e6, // ~100 MPa ultimate stress
            min_length: 0.0,
            diameter: 2.0 * (cross_section_area / std::f64::consts::PI).sqrt(),
        }
    }

    /// Create properties for a soft/compliant cable.
    ///
    /// Useful for soft robotics applications.
    #[must_use]
    pub fn soft_cable(stiffness: f64, rest_length: f64) -> Self {
        Self {
            stiffness,
            damping: 0.1 * (2.0 * stiffness.sqrt()), // 10% critical damping
            rest_length,
            max_tension: f64::INFINITY,
            min_length: 0.0,
            diameter: 0.003,
        }
    }

    /// Set the rest length.
    #[must_use]
    pub fn with_rest_length(mut self, rest_length: f64) -> Self {
        self.rest_length = rest_length;
        self
    }

    /// Set the damping coefficient.
    #[must_use]
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.damping = damping.max(0.0);
        self
    }

    /// Set the maximum tension.
    #[must_use]
    pub fn with_max_tension(mut self, max_tension: f64) -> Self {
        self.max_tension = max_tension;
        self
    }

    /// Compute the cable force given length and velocity.
    ///
    /// # Arguments
    ///
    /// * `length` - Current cable length (m)
    /// * `velocity` - Rate of length change (m/s, positive = lengthening)
    ///
    /// # Returns
    ///
    /// Cable tension in Newtons (always non-negative).
    #[must_use]
    pub fn compute_force(&self, length: f64, velocity: f64) -> f64 {
        // Check if cable is slack
        if length <= self.rest_length || length <= self.min_length {
            return 0.0;
        }

        // Spring force
        let stretch = length - self.rest_length;
        let spring_force = self.stiffness * stretch;

        // Damping force (only when lengthening to avoid negative tension)
        let damping_force = self.damping * velocity.max(0.0);

        // Total force, clamped to non-negative and max tension
        (spring_force + damping_force)
            .max(0.0)
            .min(self.max_tension)
    }

    /// Compute the cable force with full damping (including during shortening).
    ///
    /// This version includes damping during shortening, which can result in
    /// reduced tension but still non-negative.
    #[must_use]
    pub fn compute_force_with_full_damping(&self, length: f64, velocity: f64) -> f64 {
        if length <= self.rest_length || length <= self.min_length {
            return 0.0;
        }

        let stretch = length - self.rest_length;
        let spring_force = self.stiffness * stretch;
        let damping_force = self.damping * velocity;

        // Ensure tension is non-negative (cables can't push)
        (spring_force + damping_force)
            .max(0.0)
            .min(self.max_tension)
    }

    /// Check if the cable is slack at the given length.
    #[must_use]
    pub fn is_slack(&self, length: f64) -> bool {
        length <= self.rest_length || length <= self.min_length
    }

    /// Compute the strain (relative stretch) at the given length.
    #[must_use]
    pub fn strain(&self, length: f64) -> f64 {
        if self.rest_length > 0.0 {
            (length - self.rest_length) / self.rest_length
        } else {
            0.0
        }
    }
}

/// Current state of a cable/tendon.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CableState {
    /// Current length in meters.
    pub length: f64,

    /// Current velocity (rate of length change) in m/s.
    pub velocity: f64,

    /// Current tension in Newtons.
    pub tension: f64,

    /// Whether the cable is slack.
    pub is_slack: bool,
}

impl CableState {
    /// Create a new cable state.
    #[must_use]
    pub const fn new(length: f64, velocity: f64, tension: f64, is_slack: bool) -> Self {
        Self {
            length,
            velocity,
            tension,
            is_slack,
        }
    }

    /// Create a state from properties and current length/velocity.
    #[must_use]
    pub fn from_properties(props: &CableProperties, length: f64, velocity: f64) -> Self {
        let is_slack = props.is_slack(length);
        let tension = props.compute_force(length, velocity);

        Self {
            length,
            velocity,
            tension,
            is_slack,
        }
    }
}

/// Result of computing cable tension.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct TensionResult {
    /// Cable tension in Newtons.
    pub tension: f64,

    /// Whether the cable is in tension (not slack).
    pub in_tension: bool,

    /// Whether the tension limit was exceeded.
    pub limit_exceeded: bool,

    /// Current strain (stretch / `rest_length`).
    pub strain: f64,
}

impl TensionResult {
    /// Create a tension result for a slack cable.
    #[must_use]
    pub fn slack() -> Self {
        Self {
            tension: 0.0,
            in_tension: false,
            limit_exceeded: false,
            strain: 0.0,
        }
    }

    /// Create a tension result with the given values.
    #[must_use]
    pub const fn new(tension: f64, in_tension: bool, limit_exceeded: bool, strain: f64) -> Self {
        Self {
            tension,
            in_tension,
            limit_exceeded,
            strain,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_cable_properties_default() {
        let props = CableProperties::default();
        assert!(props.stiffness > 0.0);
        assert!(props.damping >= 0.0);
        assert!(props.rest_length > 0.0);
    }

    #[test]
    fn test_cable_slack() {
        let props = CableProperties::new(10000.0, 0.5);

        // At rest length, no force
        let force = props.compute_force(0.5, 0.0);
        assert_relative_eq!(force, 0.0, epsilon = 1e-10);

        // Below rest length, no force (slack)
        let force = props.compute_force(0.4, 0.0);
        assert_relative_eq!(force, 0.0, epsilon = 1e-10);

        assert!(props.is_slack(0.4));
        assert!(props.is_slack(0.5));
        assert!(!props.is_slack(0.6));
    }

    #[test]
    fn test_cable_tension() {
        let props = CableProperties::new(10000.0, 0.5);

        // Stretched by 0.1m at 10000 N/m = 1000N
        let force = props.compute_force(0.6, 0.0);
        assert_relative_eq!(force, 1000.0, epsilon = 1e-10);
    }

    #[test]
    fn test_cable_damping() {
        let props = CableProperties::new(10000.0, 0.5).with_damping(100.0);

        // Stretched by 0.1m + velocity of 1 m/s
        let force = props.compute_force(0.6, 1.0);
        // Spring: 1000N, Damping: 100N
        assert_relative_eq!(force, 1100.0, epsilon = 1e-10);

        // Damping during shortening is zero in default mode
        let force = props.compute_force(0.6, -1.0);
        assert_relative_eq!(force, 1000.0, epsilon = 1e-10);
    }

    #[test]
    fn test_cable_max_tension() {
        let props = CableProperties::new(10000.0, 0.5).with_max_tension(500.0);

        // Would be 1000N but clamped to 500N
        let force = props.compute_force(0.6, 0.0);
        assert_relative_eq!(force, 500.0, epsilon = 1e-10);
    }

    #[test]
    fn test_steel_cable() {
        let props = CableProperties::steel_cable(0.002); // 2mm cable

        // Should have high stiffness
        assert!(props.stiffness > 100_000.0);
        assert!(props.max_tension > 1000.0);
    }

    #[test]
    fn test_cable_strain() {
        let props = CableProperties::new(10000.0, 0.5);

        assert_relative_eq!(props.strain(0.5), 0.0, epsilon = 1e-10);
        assert_relative_eq!(props.strain(0.55), 0.1, epsilon = 1e-10);
        assert_relative_eq!(props.strain(0.6), 0.2, epsilon = 1e-10);
    }

    #[test]
    fn test_cable_state() {
        let props = CableProperties::new(10000.0, 0.5);
        let state = CableState::from_properties(&props, 0.6, 0.0);

        assert_relative_eq!(state.length, 0.6, epsilon = 1e-10);
        assert_relative_eq!(state.tension, 1000.0, epsilon = 1e-10);
        assert!(!state.is_slack);
    }
}
