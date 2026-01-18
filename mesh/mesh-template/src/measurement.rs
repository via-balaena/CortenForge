//! Measurement types for dimensional constraints.
//!
//! Measurements define target dimensions that the fitting process should achieve,
//! such as circumference, width, height, or depth at specific locations.

/// Type of measurement to extract or constrain.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum MeasurementType {
    /// Circumference around a cross-section perpendicular to the normal.
    ///
    /// Computed by projecting vertices onto the measurement plane,
    /// finding the convex hull, and computing its perimeter.
    Circumference,

    /// Width extent perpendicular to the measurement normal.
    ///
    /// Measures the maximum extent in one direction perpendicular
    /// to the normal vector.
    Width,

    /// Height extent along the measurement normal.
    ///
    /// Measures the maximum extent along the normal direction.
    Height,

    /// Depth extent perpendicular to both normal and width.
    ///
    /// Measures the maximum extent in the direction perpendicular
    /// to both the normal and the width direction.
    Depth,
}

/// A dimensional measurement constraint.
///
/// Measurements can be:
/// - **Exact**: Target a specific value within tolerance
/// - **Minimum**: Ensure the dimension is at least this value
///
/// # Examples
///
/// ```
/// use mesh_template::Measurement;
///
/// // Exact 100mm circumference with default 1mm tolerance
/// let exact = Measurement::exact(100.0);
///
/// // Exact value with custom 5mm tolerance
/// let tolerant = Measurement::with_tolerance(100.0, 5.0);
///
/// // Minimum 50mm width (can be larger)
/// let minimum = Measurement::minimum(50.0);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Measurement {
    /// Target measurement value (in mesh units, typically mm).
    pub value: f64,

    /// Tolerance for the measurement (default: 1.0).
    ///
    /// The fit is considered acceptable if the actual measurement
    /// is within `value ± tolerance`.
    pub tolerance: f64,

    /// Whether this is a minimum constraint (inequality).
    ///
    /// If `true`, the actual measurement must be >= `value`.
    /// If `false`, the actual measurement should equal `value ± tolerance`.
    pub is_minimum: bool,
}

impl Measurement {
    /// Default tolerance for measurements (1mm).
    pub const DEFAULT_TOLERANCE: f64 = 1.0;

    /// Creates an exact measurement constraint with default tolerance.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::Measurement;
    ///
    /// let m = Measurement::exact(100.0);
    /// assert!((m.value - 100.0).abs() < 1e-10);
    /// assert!((m.tolerance - 1.0).abs() < 1e-10);
    /// assert!(!m.is_minimum);
    /// ```
    #[must_use]
    pub const fn exact(value: f64) -> Self {
        Self {
            value,
            tolerance: Self::DEFAULT_TOLERANCE,
            is_minimum: false,
        }
    }

    /// Creates an exact measurement constraint with custom tolerance.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::Measurement;
    ///
    /// let m = Measurement::with_tolerance(100.0, 5.0);
    /// assert!((m.tolerance - 5.0).abs() < 1e-10);
    /// ```
    #[must_use]
    pub const fn with_tolerance(value: f64, tolerance: f64) -> Self {
        Self {
            value,
            tolerance,
            is_minimum: false,
        }
    }

    /// Creates a minimum measurement constraint.
    ///
    /// The actual measurement must be at least this value.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::Measurement;
    ///
    /// let m = Measurement::minimum(50.0);
    /// assert!(m.is_minimum);
    /// ```
    #[must_use]
    pub const fn minimum(value: f64) -> Self {
        Self {
            value,
            tolerance: Self::DEFAULT_TOLERANCE,
            is_minimum: true,
        }
    }

    /// Checks if an actual measurement satisfies this constraint.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::Measurement;
    ///
    /// let exact = Measurement::exact(100.0);
    /// assert!(exact.is_satisfied(100.5)); // Within tolerance
    /// assert!(!exact.is_satisfied(105.0)); // Outside tolerance
    ///
    /// let minimum = Measurement::minimum(50.0);
    /// assert!(minimum.is_satisfied(60.0)); // Above minimum
    /// assert!(!minimum.is_satisfied(40.0)); // Below minimum
    /// ```
    #[must_use]
    pub fn is_satisfied(&self, actual: f64) -> bool {
        if self.is_minimum {
            actual >= self.value - self.tolerance
        } else {
            (actual - self.value).abs() <= self.tolerance
        }
    }

    /// Computes the error between an actual measurement and this constraint.
    ///
    /// Returns 0.0 if the constraint is satisfied, otherwise returns
    /// the distance from the acceptable range.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::Measurement;
    ///
    /// let exact = Measurement::with_tolerance(100.0, 1.0);
    /// assert!((exact.error(100.0)).abs() < 1e-10); // On target
    /// assert!((exact.error(100.5)).abs() < 1e-10); // Within tolerance
    /// assert!((exact.error(105.0) - 4.0).abs() < 1e-10); // 4mm over tolerance
    /// ```
    #[must_use]
    pub fn error(&self, actual: f64) -> f64 {
        if self.is_minimum {
            if actual >= self.value - self.tolerance {
                0.0
            } else {
                self.value - self.tolerance - actual
            }
        } else {
            let diff = (actual - self.value).abs();
            if diff <= self.tolerance {
                0.0
            } else {
                diff - self.tolerance
            }
        }
    }

    /// Computes the scale factor needed to achieve this measurement.
    ///
    /// Returns the ratio `target / actual` that would scale the actual
    /// measurement to the target value.
    ///
    /// Returns `None` if the actual measurement is zero or negative.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::Measurement;
    ///
    /// let m = Measurement::exact(100.0);
    /// let scale = m.scale_factor(80.0).unwrap();
    /// assert!((scale - 1.25).abs() < 1e-10); // 100/80 = 1.25
    /// ```
    #[must_use]
    pub fn scale_factor(&self, actual: f64) -> Option<f64> {
        if actual <= 0.0 {
            return None;
        }

        if self.is_minimum && actual >= self.value {
            // Already satisfies minimum constraint
            Some(1.0)
        } else {
            Some(self.value / actual)
        }
    }
}

impl Default for Measurement {
    fn default() -> Self {
        Self::exact(0.0)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_exact_measurement() {
        let m = Measurement::exact(100.0);
        assert_relative_eq!(m.value, 100.0);
        assert_relative_eq!(m.tolerance, 1.0);
        assert!(!m.is_minimum);
    }

    #[test]
    fn test_with_tolerance() {
        let m = Measurement::with_tolerance(50.0, 2.5);
        assert_relative_eq!(m.value, 50.0);
        assert_relative_eq!(m.tolerance, 2.5);
    }

    #[test]
    fn test_minimum_measurement() {
        let m = Measurement::minimum(30.0);
        assert_relative_eq!(m.value, 30.0);
        assert!(m.is_minimum);
    }

    #[test]
    fn test_is_satisfied_exact() {
        let m = Measurement::with_tolerance(100.0, 2.0);

        assert!(m.is_satisfied(100.0)); // Exact
        assert!(m.is_satisfied(101.0)); // Within tolerance
        assert!(m.is_satisfied(99.0)); // Within tolerance
        assert!(m.is_satisfied(102.0)); // Edge of tolerance
        assert!(!m.is_satisfied(103.0)); // Outside tolerance
        assert!(!m.is_satisfied(97.0)); // Outside tolerance
    }

    #[test]
    fn test_is_satisfied_minimum() {
        let m = Measurement::minimum(50.0);

        assert!(m.is_satisfied(50.0)); // Exactly at minimum
        assert!(m.is_satisfied(60.0)); // Above minimum
        assert!(m.is_satisfied(49.5)); // Within tolerance of minimum
        assert!(!m.is_satisfied(48.0)); // Below minimum - tolerance
    }

    #[test]
    fn test_error_exact() {
        let m = Measurement::with_tolerance(100.0, 2.0);

        assert_relative_eq!(m.error(100.0), 0.0); // On target
        assert_relative_eq!(m.error(101.0), 0.0); // Within tolerance
        assert_relative_eq!(m.error(105.0), 3.0); // 3 over tolerance edge
        assert_relative_eq!(m.error(95.0), 3.0); // 3 under tolerance edge
    }

    #[test]
    fn test_error_minimum() {
        let m = Measurement::minimum(50.0);

        assert_relative_eq!(m.error(60.0), 0.0); // Above minimum
        assert_relative_eq!(m.error(50.0), 0.0); // At minimum
        assert_relative_eq!(m.error(49.0), 0.0); // Within tolerance
        assert_relative_eq!(m.error(45.0), 4.0); // 4 below tolerance edge
    }

    #[test]
    fn test_scale_factor() {
        let m = Measurement::exact(100.0);

        assert_relative_eq!(m.scale_factor(100.0).unwrap(), 1.0);
        assert_relative_eq!(m.scale_factor(50.0).unwrap(), 2.0);
        assert_relative_eq!(m.scale_factor(200.0).unwrap(), 0.5);
    }

    #[test]
    fn test_scale_factor_minimum() {
        let m = Measurement::minimum(50.0);

        // Already above minimum, no scaling needed
        assert_relative_eq!(m.scale_factor(60.0).unwrap(), 1.0);

        // Below minimum, need to scale up
        assert_relative_eq!(m.scale_factor(40.0).unwrap(), 1.25);
    }

    #[test]
    fn test_scale_factor_zero() {
        let m = Measurement::exact(100.0);
        assert!(m.scale_factor(0.0).is_none());
        assert!(m.scale_factor(-1.0).is_none());
    }

    #[test]
    fn test_default() {
        let m = Measurement::default();
        assert_relative_eq!(m.value, 0.0);
    }

    #[test]
    fn test_measurement_type_debug() {
        // Ensure all types are debuggable
        let types = [
            MeasurementType::Circumference,
            MeasurementType::Width,
            MeasurementType::Height,
            MeasurementType::Depth,
        ];
        for t in types {
            let _ = format!("{t:?}");
        }
    }

    #[test]
    fn test_measurement_type_equality() {
        assert_eq!(MeasurementType::Circumference, MeasurementType::Circumference);
        assert_ne!(MeasurementType::Width, MeasurementType::Height);
    }
}
