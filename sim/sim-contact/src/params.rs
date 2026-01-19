//! Contact parameters and domain randomization.
//!
//! This module provides configurable contact parameters that map to physical
//! material properties. The parameters can be randomized for sim-to-real transfer.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Physical parameters for the contact model.
///
/// These parameters control the force response when two bodies are in contact.
/// They are designed to map to real-world material properties for sim-to-real transfer.
///
/// # MuJoCo-style Contact Model
///
/// The normal contact force follows:
/// ```text
/// F_n = stiffness * penetration^stiffness_power + damping * penetration_velocity
/// ```
///
/// The tangential friction force follows:
/// ```text
/// F_t = -friction_coefficient * |F_n| * normalize(v_tangent)  (for sliding)
/// ```
///
/// # Example
///
/// ```
/// use sim_contact::ContactParams;
///
/// // Use a preset for common material pairs
/// let rubber = ContactParams::rubber_on_concrete();
///
/// // Or customize for your specific materials
/// let custom = ContactParams::default()
///     .with_stiffness(50_000.0)
///     .with_friction(0.8);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ContactParams {
    /// Stiffness coefficient (N/m^p where p is stiffness_power).
    ///
    /// Higher values = harder materials, less penetration.
    /// Typical range: 1,000 - 1,000,000 N/m
    pub stiffness: f64,

    /// Stiffness power exponent.
    ///
    /// - 1.0 = linear spring (Hooke's law)
    /// - 1.5 = Hertzian contact (spheres)
    /// - 2.0 = stronger nonlinearity
    ///
    /// MuJoCo uses values around 1.0-2.0
    pub stiffness_power: f64,

    /// Damping coefficient (N·s/m).
    ///
    /// Dissipates energy during contact. Higher = less bouncy.
    /// Typical range: 10 - 10,000 N·s/m
    pub damping: f64,

    /// Coulomb friction coefficient (dimensionless).
    ///
    /// Maximum tangential force = friction_coefficient * normal_force.
    /// - 0.0 = frictionless (ice)
    /// - 0.3-0.5 = typical materials
    /// - 0.7-1.0 = rubber, high friction
    /// - >1.0 = sticky materials
    pub friction_coefficient: f64,

    /// Rolling friction coefficient (m).
    ///
    /// Resists rolling motion. Typical range: 0.001 - 0.1 m
    /// Set to 0 to disable rolling friction.
    pub rolling_friction: f64,

    /// Torsional friction coefficient (m).
    ///
    /// Resists spinning in place. Typical range: 0.001 - 0.1 m
    /// Set to 0 to disable torsional friction.
    pub torsional_friction: f64,

    /// Minimum penetration depth before forces are applied (m).
    ///
    /// Helps with numerical stability by creating a small "skin" around objects.
    /// Typical range: 0.0001 - 0.001 m
    pub contact_margin: f64,

    /// Coefficient of restitution (dimensionless, 0-1).
    ///
    /// Controls bounciness during collisions.
    /// - 0.0 = perfectly inelastic (no bounce)
    /// - 1.0 = perfectly elastic (full bounce)
    ///
    /// Note: In compliant contact models, this is approximated through
    /// the damping coefficient, but we expose it for convenience.
    pub restitution: f64,
}

impl Default for ContactParams {
    fn default() -> Self {
        Self {
            stiffness: 100_000.0,      // 100 kN/m - moderate stiffness
            stiffness_power: 1.0,      // Linear spring
            damping: 1_000.0,          // 1 kN·s/m - moderate damping
            friction_coefficient: 0.5, // Typical friction
            rolling_friction: 0.01,    // Small rolling resistance
            torsional_friction: 0.01,  // Small torsional resistance
            contact_margin: 0.0001,    // 0.1 mm margin
            restitution: 0.3,          // Slight bounce
        }
    }
}

impl ContactParams {
    /// Create parameters for rubber on concrete (high friction, low bounce).
    #[must_use]
    pub fn rubber_on_concrete() -> Self {
        Self {
            stiffness: 50_000.0,
            stiffness_power: 1.3,
            damping: 2_000.0,
            friction_coefficient: 0.9,
            rolling_friction: 0.02,
            torsional_friction: 0.02,
            contact_margin: 0.0002,
            restitution: 0.1,
        }
    }

    /// Create parameters for metal on metal (stiff, moderate friction).
    #[must_use]
    pub fn metal_on_metal() -> Self {
        Self {
            stiffness: 500_000.0,
            stiffness_power: 1.0,
            damping: 5_000.0,
            friction_coefficient: 0.4,
            rolling_friction: 0.005,
            torsional_friction: 0.005,
            contact_margin: 0.00005,
            restitution: 0.5,
        }
    }

    /// Create parameters for plastic on plastic (moderate everything).
    #[must_use]
    pub fn plastic_on_plastic() -> Self {
        Self {
            stiffness: 80_000.0,
            stiffness_power: 1.2,
            damping: 1_500.0,
            friction_coefficient: 0.35,
            rolling_friction: 0.01,
            torsional_friction: 0.01,
            contact_margin: 0.0001,
            restitution: 0.4,
        }
    }

    /// Create parameters for wood on wood.
    #[must_use]
    pub fn wood_on_wood() -> Self {
        Self {
            stiffness: 100_000.0,
            stiffness_power: 1.1,
            damping: 2_000.0,
            friction_coefficient: 0.5,
            rolling_friction: 0.015,
            torsional_friction: 0.015,
            contact_margin: 0.0001,
            restitution: 0.3,
        }
    }

    /// Create very soft parameters (for testing/debugging).
    #[must_use]
    pub fn soft() -> Self {
        Self {
            stiffness: 10_000.0,
            stiffness_power: 1.0,
            damping: 500.0,
            friction_coefficient: 0.3,
            rolling_friction: 0.02,
            torsional_friction: 0.02,
            contact_margin: 0.001,
            restitution: 0.5,
        }
    }

    /// Create stiff parameters (for hard surfaces).
    #[must_use]
    pub fn stiff() -> Self {
        Self {
            stiffness: 1_000_000.0,
            stiffness_power: 1.0,
            damping: 10_000.0,
            friction_coefficient: 0.5,
            rolling_friction: 0.005,
            torsional_friction: 0.005,
            contact_margin: 0.00001,
            restitution: 0.2,
        }
    }

    /// Create frictionless parameters (ice-like).
    #[must_use]
    pub fn frictionless() -> Self {
        Self {
            friction_coefficient: 0.0,
            rolling_friction: 0.0,
            torsional_friction: 0.0,
            ..Default::default()
        }
    }

    /// Set the stiffness coefficient.
    #[must_use]
    pub fn with_stiffness(mut self, stiffness: f64) -> Self {
        self.stiffness = stiffness;
        self
    }

    /// Set the stiffness power.
    #[must_use]
    pub fn with_stiffness_power(mut self, power: f64) -> Self {
        self.stiffness_power = power;
        self
    }

    /// Set the damping coefficient.
    #[must_use]
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.damping = damping;
        self
    }

    /// Set the friction coefficient.
    #[must_use]
    pub fn with_friction(mut self, friction: f64) -> Self {
        self.friction_coefficient = friction;
        self
    }

    /// Set the restitution (bounciness).
    #[must_use]
    pub fn with_restitution(mut self, restitution: f64) -> Self {
        self.restitution = restitution.clamp(0.0, 1.0);
        self
    }

    /// Compute the critical damping coefficient for a given mass.
    ///
    /// Critical damping prevents oscillation. Use this to set damping
    /// based on expected contact mass.
    #[must_use]
    pub fn critical_damping(stiffness: f64, mass: f64) -> f64 {
        2.0 * (stiffness * mass).sqrt()
    }

    /// Validate the parameters are physically reasonable.
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.stiffness <= 0.0 {
            return Err("stiffness must be positive");
        }
        if self.stiffness_power < 0.5 || self.stiffness_power > 3.0 {
            return Err("stiffness_power should be in [0.5, 3.0]");
        }
        if self.damping < 0.0 {
            return Err("damping cannot be negative");
        }
        if self.friction_coefficient < 0.0 {
            return Err("friction_coefficient cannot be negative");
        }
        if self.contact_margin < 0.0 {
            return Err("contact_margin cannot be negative");
        }
        if self.restitution < 0.0 || self.restitution > 1.0 {
            return Err("restitution must be in [0, 1]");
        }
        Ok(())
    }

    /// Compute effective parameters for a pair of materials.
    ///
    /// Uses harmonic mean for stiffness/damping and geometric mean for friction.
    #[must_use]
    pub fn combine(a: &Self, b: &Self) -> Self {
        Self {
            // Harmonic mean for stiffness (springs in series)
            stiffness: 2.0 * a.stiffness * b.stiffness / (a.stiffness + b.stiffness),
            // Average for power
            stiffness_power: (a.stiffness_power + b.stiffness_power) * 0.5,
            // Harmonic mean for damping
            damping: 2.0 * a.damping * b.damping / (a.damping + b.damping + 1e-10),
            // Geometric mean for friction
            friction_coefficient: (a.friction_coefficient * b.friction_coefficient).sqrt(),
            rolling_friction: (a.rolling_friction * b.rolling_friction).sqrt(),
            torsional_friction: (a.torsional_friction * b.torsional_friction).sqrt(),
            // Max margin
            contact_margin: a.contact_margin.max(b.contact_margin),
            // Geometric mean for restitution
            restitution: (a.restitution * b.restitution).sqrt(),
        }
    }
}

/// Surface material type for automatic parameter lookup.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum SurfaceMaterial {
    /// Generic default material.
    Default,
    /// Rubber (high friction, low bounce).
    Rubber,
    /// Metal (stiff, moderate friction).
    Metal,
    /// Plastic (moderate properties).
    Plastic,
    /// Wood (moderate properties).
    Wood,
    /// Concrete (stiff, moderate friction).
    Concrete,
    /// Ice (low friction).
    Ice,
    /// Glass (stiff, low friction).
    Glass,
}

impl SurfaceMaterial {
    /// Get the default contact parameters for this material.
    #[must_use]
    pub fn params(self) -> ContactParams {
        match self {
            Self::Default => ContactParams::default(),
            Self::Rubber => ContactParams::rubber_on_concrete(),
            Self::Metal => ContactParams::metal_on_metal(),
            Self::Plastic => ContactParams::plastic_on_plastic(),
            Self::Wood => ContactParams::wood_on_wood(),
            Self::Concrete => ContactParams::stiff().with_friction(0.6),
            Self::Ice => ContactParams::frictionless().with_restitution(0.1),
            Self::Glass => ContactParams::stiff()
                .with_friction(0.2)
                .with_restitution(0.5),
        }
    }
}

/// A material pair key for looking up combined contact parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MaterialPair {
    /// First material (ordered so that a <= b for consistent hashing).
    a: SurfaceMaterial,
    /// Second material.
    b: SurfaceMaterial,
}

impl MaterialPair {
    /// Create a new material pair (order-independent).
    #[must_use]
    pub fn new(mat_a: SurfaceMaterial, mat_b: SurfaceMaterial) -> Self {
        // Canonical ordering for consistent hashing
        if (mat_a as u8) <= (mat_b as u8) {
            Self { a: mat_a, b: mat_b }
        } else {
            Self { a: mat_b, b: mat_a }
        }
    }

    /// Get the combined contact parameters for this material pair.
    #[must_use]
    pub fn params(&self) -> ContactParams {
        ContactParams::combine(&self.a.params(), &self.b.params())
    }
}

/// Configuration for domain randomization of contact parameters.
///
/// Domain randomization is a technique for sim-to-real transfer where you
/// train with randomized simulation parameters to make the policy robust
/// to real-world variations.
///
/// # Example
///
/// ```
/// use sim_contact::{ContactParams, DomainRandomization};
///
/// let base_params = ContactParams::rubber_on_concrete();
/// let randomizer = DomainRandomization::default();
///
/// // In your training loop, sample randomized parameters:
/// // let randomized = randomizer.sample(&base_params, &mut rng);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct DomainRandomization {
    /// Range for stiffness multiplier (relative to base).
    pub stiffness_range: (f64, f64),

    /// Range for damping multiplier (relative to base).
    pub damping_range: (f64, f64),

    /// Range for friction coefficient (absolute values).
    pub friction_range: (f64, f64),

    /// Range for stiffness power (absolute values).
    pub stiffness_power_range: (f64, f64),

    /// Range for restitution (absolute values).
    pub restitution_range: (f64, f64),

    /// Whether to correlate stiffness and damping (physically realistic).
    pub correlate_stiffness_damping: bool,
}

impl Default for DomainRandomization {
    fn default() -> Self {
        Self {
            // ±50% stiffness variation
            stiffness_range: (0.5, 1.5),
            // ±50% damping variation
            damping_range: (0.5, 1.5),
            // Friction from 0.2 to 1.0
            friction_range: (0.2, 1.0),
            // Stiffness power 0.8 to 1.5
            stiffness_power_range: (0.8, 1.5),
            // Restitution 0.0 to 0.6
            restitution_range: (0.0, 0.6),
            // Correlate by default (more realistic)
            correlate_stiffness_damping: true,
        }
    }
}

impl DomainRandomization {
    /// Create mild randomization (±20% variation).
    #[must_use]
    pub fn mild() -> Self {
        Self {
            stiffness_range: (0.8, 1.2),
            damping_range: (0.8, 1.2),
            friction_range: (0.3, 0.7),
            stiffness_power_range: (0.9, 1.2),
            restitution_range: (0.1, 0.5),
            correlate_stiffness_damping: true,
        }
    }

    /// Create aggressive randomization for maximum robustness.
    #[must_use]
    pub fn aggressive() -> Self {
        Self {
            stiffness_range: (0.3, 2.0),
            damping_range: (0.3, 2.0),
            friction_range: (0.1, 1.2),
            stiffness_power_range: (0.7, 2.0),
            restitution_range: (0.0, 0.8),
            correlate_stiffness_damping: false, // Allow uncorrelated for more variety
        }
    }

    /// No randomization (deterministic parameters).
    #[must_use]
    pub fn none() -> Self {
        Self {
            stiffness_range: (1.0, 1.0),
            damping_range: (1.0, 1.0),
            friction_range: (0.5, 0.5), // Will need to match base
            stiffness_power_range: (1.0, 1.0),
            restitution_range: (0.3, 0.3),
            correlate_stiffness_damping: true,
        }
    }

    /// Sample randomized parameters from the given base.
    ///
    /// # Arguments
    ///
    /// * `base` - Base contact parameters to randomize
    /// * `uniform_samples` - 5 uniform random values in \[0, 1\] for:
    ///   - Index 0: stiffness
    ///   - Index 1: damping (ignored if correlate_stiffness_damping is true)
    ///   - Index 2: friction
    ///   - Index 3: stiffness_power
    ///   - Index 4: restitution
    #[must_use]
    pub fn sample(&self, base: &ContactParams, uniform_samples: [f64; 5]) -> ContactParams {
        let stiffness_mult = lerp(
            self.stiffness_range.0,
            self.stiffness_range.1,
            uniform_samples[0],
        );
        let damping_mult = if self.correlate_stiffness_damping {
            // Damping should scale with sqrt(stiffness) for critical damping
            stiffness_mult.sqrt()
        } else {
            lerp(
                self.damping_range.0,
                self.damping_range.1,
                uniform_samples[1],
            )
        };

        ContactParams {
            stiffness: base.stiffness * stiffness_mult,
            stiffness_power: lerp(
                self.stiffness_power_range.0,
                self.stiffness_power_range.1,
                uniform_samples[3],
            ),
            damping: base.damping * damping_mult,
            friction_coefficient: lerp(
                self.friction_range.0,
                self.friction_range.1,
                uniform_samples[2],
            ),
            rolling_friction: base.rolling_friction * stiffness_mult.sqrt(), // Scale with main friction
            torsional_friction: base.torsional_friction * stiffness_mult.sqrt(),
            contact_margin: base.contact_margin,
            restitution: lerp(
                self.restitution_range.0,
                self.restitution_range.1,
                uniform_samples[4],
            ),
        }
    }

    /// Check if a parameter set is within the randomization bounds.
    #[must_use]
    pub fn contains(&self, params: &ContactParams, base: &ContactParams) -> bool {
        let stiffness_mult = params.stiffness / base.stiffness;
        let damping_mult = params.damping / base.damping;

        stiffness_mult >= self.stiffness_range.0
            && stiffness_mult <= self.stiffness_range.1
            && damping_mult >= self.damping_range.0
            && damping_mult <= self.damping_range.1
            && params.friction_coefficient >= self.friction_range.0
            && params.friction_coefficient <= self.friction_range.1
            && params.stiffness_power >= self.stiffness_power_range.0
            && params.stiffness_power <= self.stiffness_power_range.1
            && params.restitution >= self.restitution_range.0
            && params.restitution <= self.restitution_range.1
    }
}

/// Linear interpolation.
fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_default_params_valid() {
        let params = ContactParams::default();
        assert!(params.validate().is_ok());
    }

    #[test]
    fn test_preset_params_valid() {
        assert!(ContactParams::rubber_on_concrete().validate().is_ok());
        assert!(ContactParams::metal_on_metal().validate().is_ok());
        assert!(ContactParams::plastic_on_plastic().validate().is_ok());
        assert!(ContactParams::wood_on_wood().validate().is_ok());
        assert!(ContactParams::soft().validate().is_ok());
        assert!(ContactParams::stiff().validate().is_ok());
        assert!(ContactParams::frictionless().validate().is_ok());
    }

    #[test]
    fn test_params_combine() {
        let rubber = ContactParams::rubber_on_concrete();
        let metal = ContactParams::metal_on_metal();

        let combined = ContactParams::combine(&rubber, &metal);

        // Combined friction should be geometric mean
        let expected_friction = (rubber.friction_coefficient * metal.friction_coefficient).sqrt();
        assert_relative_eq!(
            combined.friction_coefficient,
            expected_friction,
            epsilon = 1e-10
        );

        // Combined stiffness should be harmonic mean (less than both)
        assert!(combined.stiffness < rubber.stiffness.max(metal.stiffness));
    }

    #[test]
    fn test_material_pair_symmetric() {
        let pair1 = MaterialPair::new(SurfaceMaterial::Rubber, SurfaceMaterial::Metal);
        let pair2 = MaterialPair::new(SurfaceMaterial::Metal, SurfaceMaterial::Rubber);

        assert_eq!(pair1, pair2);
    }

    #[test]
    fn test_domain_randomization_sample() {
        let base = ContactParams::default();
        let randomizer = DomainRandomization::default();

        // Middle of ranges
        let samples = [0.5, 0.5, 0.5, 0.5, 0.5];
        let randomized = randomizer.sample(&base, samples);

        // Should be valid
        assert!(randomized.validate().is_ok());

        // Should be within bounds
        assert!(randomizer.contains(&randomized, &base));
    }

    #[test]
    fn test_domain_randomization_extremes() {
        let base = ContactParams::default();
        let randomizer = DomainRandomization::default();

        // Minimum values
        let min_samples = [0.0, 0.0, 0.0, 0.0, 0.0];
        let min_params = randomizer.sample(&base, min_samples);
        assert!(min_params.validate().is_ok());

        // Maximum values
        let max_samples = [1.0, 1.0, 1.0, 1.0, 1.0];
        let max_params = randomizer.sample(&base, max_samples);
        assert!(max_params.validate().is_ok());
    }

    #[test]
    fn test_critical_damping() {
        let stiffness = 100_000.0;
        let mass = 1.0;

        let critical = ContactParams::critical_damping(stiffness, mass);

        // Critical damping = 2 * sqrt(k * m)
        let expected = 2.0 * (stiffness * mass).sqrt();
        assert_relative_eq!(critical, expected, epsilon = 1e-10);
    }

    #[test]
    fn test_builder_pattern() {
        let params = ContactParams::default()
            .with_stiffness(200_000.0)
            .with_friction(0.8)
            .with_restitution(0.2);

        assert_relative_eq!(params.stiffness, 200_000.0, epsilon = 1e-10);
        assert_relative_eq!(params.friction_coefficient, 0.8, epsilon = 1e-10);
        assert_relative_eq!(params.restitution, 0.2, epsilon = 1e-10);
    }
}
