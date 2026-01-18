//! Parameters for mesh decimation.

use std::f64::consts::FRAC_PI_6;

/// Parameters for mesh decimation.
#[derive(Debug, Clone)]
pub struct DecimateParams {
    /// Target number of triangles. If None, uses `target_ratio` instead.
    pub target_triangles: Option<usize>,

    /// Target ratio of triangles to keep (0.0 to 1.0). Default: 0.5
    pub target_ratio: f64,

    /// Whether to preserve boundary edges (edges with only one adjacent face).
    /// Default: true
    pub preserve_boundary: bool,

    /// Whether to preserve sharp features (edges with high dihedral angle).
    /// Default: false
    pub preserve_sharp_features: bool,

    /// Dihedral angle threshold in radians for sharp feature detection.
    /// Edges with angle above this are considered sharp. Default: π/6 (30 degrees)
    pub sharp_angle_threshold: f64,

    /// Maximum error allowed for edge collapse. If None, no limit.
    pub max_error: Option<f64>,

    /// Penalty multiplier for boundary edges when `preserve_boundary` is false.
    /// Higher values make boundary edges less likely to collapse. Default: 10.0
    pub boundary_penalty: f64,
}

impl Default for DecimateParams {
    fn default() -> Self {
        Self {
            target_triangles: None,
            target_ratio: 0.5,
            preserve_boundary: true,
            preserve_sharp_features: false,
            sharp_angle_threshold: FRAC_PI_6, // 30 degrees
            max_error: None,
            boundary_penalty: 10.0,
        }
    }
}

impl DecimateParams {
    /// Create params targeting a specific triangle count.
    #[must_use]
    pub fn with_target_triangles(count: usize) -> Self {
        Self {
            target_triangles: Some(count),
            ..Default::default()
        }
    }

    /// Create params targeting a ratio of original triangles.
    #[must_use]
    pub fn with_target_ratio(ratio: f64) -> Self {
        Self {
            target_ratio: ratio.clamp(0.0, 1.0),
            ..Default::default()
        }
    }

    /// Create aggressive decimation params (more simplification).
    #[must_use]
    pub fn aggressive() -> Self {
        Self {
            target_ratio: 0.25,
            preserve_boundary: false,
            preserve_sharp_features: false,
            boundary_penalty: 1.0,
            ..Default::default()
        }
    }

    /// Create conservative decimation params (preserve more detail).
    #[must_use]
    pub fn conservative() -> Self {
        Self {
            target_ratio: 0.75,
            preserve_boundary: true,
            preserve_sharp_features: true,
            sharp_angle_threshold: 0.3491, // 20 degrees
            ..Default::default()
        }
    }

    /// Set preserve boundary option.
    #[must_use]
    pub const fn with_preserve_boundary(mut self, preserve: bool) -> Self {
        self.preserve_boundary = preserve;
        self
    }

    /// Set preserve sharp features option.
    #[must_use]
    pub const fn with_preserve_sharp_features(mut self, preserve: bool) -> Self {
        self.preserve_sharp_features = preserve;
        self
    }

    /// Set maximum error threshold.
    #[must_use]
    pub const fn with_max_error(mut self, max_error: f64) -> Self {
        self.max_error = Some(max_error);
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_params() {
        let params = DecimateParams::default();
        assert!((params.target_ratio - 0.5).abs() < 0.001);
        assert!(params.preserve_boundary);
        assert!(!params.preserve_sharp_features);
    }

    #[test]
    fn test_target_triangles() {
        let params = DecimateParams::with_target_triangles(1000);
        assert_eq!(params.target_triangles, Some(1000));
    }

    #[test]
    fn test_target_ratio() {
        let params = DecimateParams::with_target_ratio(0.3);
        assert!((params.target_ratio - 0.3).abs() < 0.001);
    }

    #[test]
    fn test_ratio_clamping() {
        let params = DecimateParams::with_target_ratio(1.5);
        assert!((params.target_ratio - 1.0).abs() < 0.001);

        let params = DecimateParams::with_target_ratio(-0.5);
        assert!((params.target_ratio).abs() < 0.001);
    }

    #[test]
    fn test_aggressive() {
        let params = DecimateParams::aggressive();
        assert!((params.target_ratio - 0.25).abs() < 0.001);
        assert!(!params.preserve_boundary);
    }

    #[test]
    fn test_conservative() {
        let params = DecimateParams::conservative();
        assert!((params.target_ratio - 0.75).abs() < 0.001);
        assert!(params.preserve_boundary);
        assert!(params.preserve_sharp_features);
    }

    #[test]
    fn test_builder() {
        let params = DecimateParams::default()
            .with_preserve_boundary(false)
            .with_preserve_sharp_features(true)
            .with_max_error(0.01);

        assert!(!params.preserve_boundary);
        assert!(params.preserve_sharp_features);
        assert_eq!(params.max_error, Some(0.01));
    }
}
