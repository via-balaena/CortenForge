//! Force-length and force-velocity curves for muscle modeling.
//!
//! # Force-Length Relationship
//!
//! Muscle force depends on the length of the contractile element (sarcomere).
//! At optimal length, maximum force can be generated. At shorter or longer
//! lengths, force capacity is reduced.
//!
//! The active force-length curve is typically bell-shaped, while the passive
//! force-length curve rises exponentially at long lengths.
//!
//! # Force-Velocity Relationship
//!
//! During concentric (shortening) contraction, force decreases as velocity
//! increases - it's easier to move slowly. During eccentric (lengthening)
//! contraction, force can exceed isometric maximum.
//!
//! # References
//!
//! - Hill, A.V. (1938). The heat of shortening and dynamic constants of muscle.
//! - Zajac, F.E. (1989). Muscle and tendon: properties, models, scaling, and
//!   application to biomechanics and motor control.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Parameters for the active force-length curve.
///
/// Uses a Gaussian-like bell curve centered at optimal fiber length.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ActiveForceLengthCurve {
    /// Width of the ascending limb (normalized length units).
    /// Controls how quickly force drops at short lengths.
    pub width_ascending: f64,

    /// Width of the descending limb (normalized length units).
    /// Controls how quickly force drops at long lengths.
    pub width_descending: f64,

    /// Minimum normalized length where any force can be produced.
    pub min_normalized_length: f64,

    /// Maximum normalized length where any force can be produced.
    pub max_normalized_length: f64,
}

impl Default for ActiveForceLengthCurve {
    fn default() -> Self {
        Self {
            width_ascending: 0.45,  // Typical value from literature
            width_descending: 0.56, // Asymmetric curve
            min_normalized_length: 0.5,
            max_normalized_length: 1.6,
        }
    }
}

impl ActiveForceLengthCurve {
    /// Create a new active force-length curve with custom parameters.
    #[must_use]
    pub fn new(width_asc: f64, width_desc: f64) -> Self {
        Self {
            width_ascending: width_asc.max(0.1),
            width_descending: width_desc.max(0.1),
            ..Default::default()
        }
    }

    /// Evaluate the active force-length multiplier.
    ///
    /// # Arguments
    ///
    /// * `normalized_length` - Fiber length divided by optimal fiber length (L/L_opt)
    ///
    /// # Returns
    ///
    /// Force multiplier in range [0, 1], where 1 is at optimal length.
    #[must_use]
    pub fn evaluate(&self, normalized_length: f64) -> f64 {
        let l = normalized_length;

        // Outside valid range: no force
        if l <= self.min_normalized_length || l >= self.max_normalized_length {
            return 0.0;
        }

        // Use asymmetric Gaussian with different widths for ascending/descending
        let width = if l < 1.0 {
            self.width_ascending
        } else {
            self.width_descending
        };

        // Gaussian bell curve centered at 1.0 (optimal length)
        let exponent = -((l - 1.0) / width).powi(2);
        exponent.exp()
    }

    /// Evaluate the derivative of the force-length curve.
    ///
    /// Useful for computing fiber velocity from tendon velocity.
    #[must_use]
    pub fn derivative(&self, normalized_length: f64) -> f64 {
        let l = normalized_length;

        if l <= self.min_normalized_length || l >= self.max_normalized_length {
            return 0.0;
        }

        let width = if l < 1.0 {
            self.width_ascending
        } else {
            self.width_descending
        };

        let fl = self.evaluate(l);
        let dfl = -2.0 * (l - 1.0) / (width * width) * fl;

        dfl
    }
}

/// Parameters for the passive force-length curve.
///
/// Models the elastic resistance of passive tissue (titin, collagen)
/// that resists stretching beyond optimal length.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PassiveForceLengthCurve {
    /// Normalized length at which passive force begins.
    /// Typically around 1.0 (optimal length).
    pub slack_length: f64,

    /// Exponential shape factor.
    /// Higher values make the curve steeper.
    pub shape_factor: f64,

    /// Scale factor for passive force relative to max isometric force.
    pub scale: f64,
}

impl Default for PassiveForceLengthCurve {
    fn default() -> Self {
        Self {
            slack_length: 1.0, // Passive force starts at optimal length
            shape_factor: 4.0, // Standard exponential steepness
            scale: 1.0,        // Full contribution
        }
    }
}

impl PassiveForceLengthCurve {
    /// Create a new passive force-length curve.
    #[must_use]
    pub fn new(slack_length: f64, shape_factor: f64) -> Self {
        Self {
            slack_length: slack_length.max(0.5),
            shape_factor: shape_factor.max(1.0),
            scale: 1.0,
        }
    }

    /// Set the scale factor.
    #[must_use]
    pub fn with_scale(mut self, scale: f64) -> Self {
        self.scale = scale.max(0.0);
        self
    }

    /// Evaluate the passive force-length multiplier.
    ///
    /// # Arguments
    ///
    /// * `normalized_length` - Fiber length / optimal fiber length
    ///
    /// # Returns
    ///
    /// Passive force multiplier (can exceed 1.0 at very long lengths).
    #[must_use]
    pub fn evaluate(&self, normalized_length: f64) -> f64 {
        let l = normalized_length;

        // No passive force below slack length
        if l <= self.slack_length {
            return 0.0;
        }

        // Exponential rise above slack length
        let strain = (l - self.slack_length) / (1.5 - self.slack_length);
        let fp = self.scale * (strain.exp().powi(self.shape_factor as i32) - 1.0)
            / (core::f64::consts::E.powi(self.shape_factor as i32) - 1.0);

        fp.max(0.0)
    }

    /// Evaluate the derivative of the passive force-length curve.
    #[must_use]
    pub fn derivative(&self, normalized_length: f64) -> f64 {
        let l = normalized_length;

        if l <= self.slack_length {
            return 0.0;
        }

        let strain = (l - self.slack_length) / (1.5 - self.slack_length);
        let n = self.shape_factor as i32;
        let denom = 1.5 - self.slack_length;
        let exp_n = core::f64::consts::E.powi(n) - 1.0;

        self.scale * n as f64 * strain.exp().powi(n - 1) * strain.exp() / (denom * exp_n)
    }
}

/// Parameters for the force-velocity curve.
///
/// Models the Hill hyperbolic relationship between force and shortening velocity,
/// and the enhanced force during eccentric (lengthening) contractions.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ForceVelocityCurve {
    /// Maximum shortening velocity (lengths per second).
    /// Typical values: 5-15 L_opt/s for skeletal muscle.
    pub max_shortening_velocity: f64,

    /// Curvature of the concentric (shortening) portion.
    /// Higher values make the curve more curved. Typical: 0.25.
    pub curvature_concentric: f64,

    /// Maximum eccentric force multiplier.
    /// Force during lengthening can exceed isometric max. Typical: 1.4-1.8.
    pub eccentric_force_max: f64,

    /// Curvature of the eccentric (lengthening) portion.
    /// Controls the transition from isometric to max eccentric force.
    pub curvature_eccentric: f64,
}

impl Default for ForceVelocityCurve {
    fn default() -> Self {
        Self {
            max_shortening_velocity: 10.0, // 10 L_opt/s
            curvature_concentric: 0.25,
            eccentric_force_max: 1.5, // 150% of isometric max
            curvature_eccentric: 0.25,
        }
    }
}

impl ForceVelocityCurve {
    /// Create a new force-velocity curve.
    #[must_use]
    pub fn new(max_velocity: f64, curvature: f64) -> Self {
        Self {
            max_shortening_velocity: max_velocity.max(1.0),
            curvature_concentric: curvature.clamp(0.01, 1.0),
            ..Default::default()
        }
    }

    /// Set the eccentric parameters.
    #[must_use]
    pub fn with_eccentric(mut self, max_force: f64, curvature: f64) -> Self {
        self.eccentric_force_max = max_force.clamp(1.0, 2.5);
        self.curvature_eccentric = curvature.clamp(0.01, 1.0);
        self
    }

    /// Evaluate the force-velocity multiplier.
    ///
    /// # Arguments
    ///
    /// * `normalized_velocity` - Fiber velocity / max shortening velocity
    ///   - Negative = shortening (concentric)
    ///   - Positive = lengthening (eccentric)
    ///
    /// # Returns
    ///
    /// Force multiplier:
    /// - 0 at max shortening velocity
    /// - 1 at zero velocity (isometric)
    /// - Up to `eccentric_force_max` during lengthening
    #[must_use]
    pub fn evaluate(&self, normalized_velocity: f64) -> f64 {
        let v = normalized_velocity;

        if v <= -1.0 {
            // At or beyond max shortening velocity: no force
            return 0.0;
        }

        if v < 0.0 {
            // Concentric (shortening): Hill hyperbolic equation
            // F = F_max * (1 + v) / (1 - v/a)
            // Simplified form using curvature parameter
            let a = self.curvature_concentric;
            (1.0 + v) / (1.0 - v / a) * a / (a + 1.0)
        } else if v > 0.0 {
            // Eccentric (lengthening): enhanced force
            let a = self.curvature_eccentric;
            let f_max = self.eccentric_force_max;
            let asymptote = f_max;

            // Smooth transition from 1.0 (isometric) to f_max (asymptote)
            1.0 + (asymptote - 1.0) * v / (v + a)
        } else {
            // Isometric (v = 0)
            1.0
        }
    }

    /// Compute the inverse: given force, find velocity.
    ///
    /// Only valid for concentric (shortening) region.
    #[must_use]
    pub fn inverse_concentric(&self, force_multiplier: f64) -> f64 {
        let f = force_multiplier.clamp(0.0, 1.0);
        let a = self.curvature_concentric;

        if f >= 1.0 {
            return 0.0;
        }

        // Solve for v from the concentric equation
        // f = a * (1 + v) / (a - v + 1)
        // Rearranging: v = (f * (a + 1) - a) / (f + a)
        let numer = f * (a + 1.0) - a;
        let denom = f + a;

        if denom.abs() < 1e-10 {
            return 0.0;
        }

        (numer / denom).clamp(-1.0, 0.0)
    }
}

/// Combined force curves for a complete muscle model.
///
/// Provides convenient access to all three curves and combined evaluations.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MuscleForceCurves {
    /// Active force-length relationship.
    pub active_fl: ActiveForceLengthCurve,

    /// Passive force-length relationship.
    pub passive_fl: PassiveForceLengthCurve,

    /// Force-velocity relationship.
    pub fv: ForceVelocityCurve,
}

impl Default for MuscleForceCurves {
    fn default() -> Self {
        Self {
            active_fl: ActiveForceLengthCurve::default(),
            passive_fl: PassiveForceLengthCurve::default(),
            fv: ForceVelocityCurve::default(),
        }
    }
}

impl MuscleForceCurves {
    /// Create muscle force curves with custom parameters.
    #[must_use]
    pub fn new(
        active_fl: ActiveForceLengthCurve,
        passive_fl: PassiveForceLengthCurve,
        fv: ForceVelocityCurve,
    ) -> Self {
        Self {
            active_fl,
            passive_fl,
            fv,
        }
    }

    /// Evaluate the total fiber force multiplier.
    ///
    /// # Arguments
    ///
    /// * `activation` - Muscle activation level (0 to 1)
    /// * `normalized_length` - Fiber length / optimal fiber length
    /// * `normalized_velocity` - Fiber velocity / max shortening velocity
    ///
    /// # Returns
    ///
    /// Total force multiplier (active + passive components).
    #[must_use]
    pub fn evaluate(
        &self,
        activation: f64,
        normalized_length: f64,
        normalized_velocity: f64,
    ) -> f64 {
        let fl_active = self.active_fl.evaluate(normalized_length);
        let fl_passive = self.passive_fl.evaluate(normalized_length);
        let fv = self.fv.evaluate(normalized_velocity);

        // Active force = activation * force-length * force-velocity
        let active = activation * fl_active * fv;

        // Total = active + passive
        active + fl_passive
    }

    /// Evaluate only the active force component.
    #[must_use]
    pub fn evaluate_active(
        &self,
        activation: f64,
        normalized_length: f64,
        normalized_velocity: f64,
    ) -> f64 {
        let fl = self.active_fl.evaluate(normalized_length);
        let fv = self.fv.evaluate(normalized_velocity);

        activation * fl * fv
    }

    /// Evaluate only the passive force component.
    #[must_use]
    pub fn evaluate_passive(&self, normalized_length: f64) -> f64 {
        self.passive_fl.evaluate(normalized_length)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    // Active Force-Length Tests
    #[test]
    fn test_active_fl_optimal_length() {
        let curve = ActiveForceLengthCurve::default();

        // Maximum force at optimal length (L/L_opt = 1.0)
        let force = curve.evaluate(1.0);
        assert_relative_eq!(force, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_active_fl_short_length() {
        let curve = ActiveForceLengthCurve::default();

        // Less force at short length
        let force = curve.evaluate(0.7);
        assert!(force < 1.0);
        assert!(force > 0.0);
    }

    #[test]
    fn test_active_fl_long_length() {
        let curve = ActiveForceLengthCurve::default();

        // Less force at long length
        let force = curve.evaluate(1.3);
        assert!(force < 1.0);
        assert!(force > 0.0);
    }

    #[test]
    fn test_active_fl_outside_range() {
        let curve = ActiveForceLengthCurve::default();

        // No force outside valid range
        assert_relative_eq!(curve.evaluate(0.4), 0.0, epsilon = 1e-10);
        assert_relative_eq!(curve.evaluate(1.7), 0.0, epsilon = 1e-10);
    }

    // Passive Force-Length Tests
    #[test]
    fn test_passive_fl_below_slack() {
        let curve = PassiveForceLengthCurve::default();

        // No passive force below slack length
        assert_relative_eq!(curve.evaluate(0.9), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_passive_fl_above_slack() {
        let curve = PassiveForceLengthCurve::default();

        // Passive force rises above slack length
        let f1 = curve.evaluate(1.1);
        let f2 = curve.evaluate(1.3);

        assert!(f1 > 0.0);
        assert!(f2 > f1); // Increases with length
    }

    // Force-Velocity Tests
    #[test]
    fn test_fv_isometric() {
        let curve = ForceVelocityCurve::default();

        // At zero velocity (isometric), force = 1.0
        let force = curve.evaluate(0.0);
        assert_relative_eq!(force, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_fv_concentric() {
        let curve = ForceVelocityCurve::default();

        // Shortening reduces force
        let force = curve.evaluate(-0.5);
        assert!(force < 1.0);
        assert!(force > 0.0);
    }

    #[test]
    fn test_fv_max_shortening() {
        let curve = ForceVelocityCurve::default();

        // At max shortening velocity, force approaches 0
        let force = curve.evaluate(-1.0);
        assert_relative_eq!(force, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_fv_eccentric() {
        let curve = ForceVelocityCurve::default();

        // Lengthening increases force above isometric
        let force = curve.evaluate(0.5);
        assert!(force > 1.0);
        assert!(force < curve.eccentric_force_max);
    }

    // Combined Curves Tests
    #[test]
    fn test_combined_at_optimal_isometric() {
        let curves = MuscleForceCurves::default();

        // Full activation, optimal length, isometric
        let force = curves.evaluate(1.0, 1.0, 0.0);

        // Should be close to 1.0 (plus any passive component)
        assert!(force >= 1.0);
        assert!(force < 1.5);
    }

    #[test]
    fn test_combined_partial_activation() {
        let curves = MuscleForceCurves::default();

        // Half activation at optimal length
        let force = curves.evaluate(0.5, 1.0, 0.0);

        // Active should be ~0.5, plus small passive
        assert!(force >= 0.5);
        assert!(force < 0.8);
    }

    #[test]
    fn test_combined_zero_activation() {
        let curves = MuscleForceCurves::default();

        // Zero activation, but stretched
        let force = curves.evaluate(0.0, 1.2, 0.0);

        // Only passive force
        let passive = curves.evaluate_passive(1.2);
        assert_relative_eq!(force, passive, epsilon = 1e-10);
    }
}
