//! Manufacturing clearance profile.
//!
//! A [`PrintProfile`] captures the minimum feature sizes and clearances for a
//! target manufacturing process. Used by [`super::validate`] to check
//! mechanism geometry against manufacturing constraints.

/// Manufacturing clearance profile for a target process.
///
/// All dimensions are in millimeters. Values must be positive and finite.
///
/// # Example
///
/// ```
/// use cf_design::PrintProfile;
///
/// // Typical FDM profile.
/// let fdm = PrintProfile::new(0.3, 0.8, 1.5);
///
/// // High-resolution SLA profile.
/// let sla = PrintProfile::new(0.15, 0.4, 0.5);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct PrintProfile {
    /// Default clearance between mating surfaces (mm).
    /// FDM typical: 0.3, SLA typical: 0.15.
    pub clearance: f64,
    /// Minimum wall thickness the process can produce (mm).
    pub min_wall: f64,
    /// Minimum hole diameter the process can produce (mm).
    pub min_hole: f64,
}

impl PrintProfile {
    /// Create a print profile with validated parameters.
    ///
    /// # Panics
    ///
    /// Panics if any parameter is not positive and finite.
    #[must_use]
    pub fn new(clearance: f64, min_wall: f64, min_hole: f64) -> Self {
        assert!(
            clearance > 0.0 && clearance.is_finite(),
            "clearance must be positive and finite, got {clearance}"
        );
        assert!(
            min_wall > 0.0 && min_wall.is_finite(),
            "min_wall must be positive and finite, got {min_wall}"
        );
        assert!(
            min_hole > 0.0 && min_hole.is_finite(),
            "min_hole must be positive and finite, got {min_hole}"
        );
        Self {
            clearance,
            min_wall,
            min_hole,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn print_profile_valid() {
        let p = PrintProfile::new(0.3, 0.8, 1.5);
        assert!((p.clearance - 0.3).abs() < f64::EPSILON);
        assert!((p.min_wall - 0.8).abs() < f64::EPSILON);
        assert!((p.min_hole - 1.5).abs() < f64::EPSILON);
    }

    #[test]
    #[should_panic(expected = "clearance must be positive")]
    fn print_profile_rejects_zero_clearance() {
        let _pp = PrintProfile::new(0.0, 0.8, 1.5);
    }

    #[test]
    #[should_panic(expected = "clearance must be positive")]
    fn print_profile_rejects_negative_clearance() {
        let _pp = PrintProfile::new(-0.1, 0.8, 1.5);
    }

    #[test]
    #[should_panic(expected = "min_wall must be positive")]
    fn print_profile_rejects_zero_min_wall() {
        let _pp = PrintProfile::new(0.3, 0.0, 1.5);
    }

    #[test]
    #[should_panic(expected = "min_hole must be positive")]
    fn print_profile_rejects_negative_min_hole() {
        let _pp = PrintProfile::new(0.3, 0.8, -1.0);
    }

    #[test]
    #[should_panic(expected = "clearance must be positive")]
    fn print_profile_rejects_nan() {
        let _pp = PrintProfile::new(f64::NAN, 0.8, 1.5);
    }

    #[test]
    #[should_panic(expected = "min_wall must be positive")]
    fn print_profile_rejects_inf_min_wall() {
        let _pp = PrintProfile::new(0.3, f64::INFINITY, 1.5);
    }
}
