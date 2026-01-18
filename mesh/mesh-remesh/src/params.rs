//! Remeshing parameters.

/// Parameters for isotropic mesh remeshing.
#[derive(Debug, Clone)]
#[allow(clippy::struct_excessive_bools)]
pub struct RemeshParams {
    /// Target edge length for uniform remeshing.
    pub target_edge_length: f64,

    /// Number of iterations to perform.
    pub iterations: u32,

    /// Whether to preserve boundary edges.
    pub preserve_boundaries: bool,

    /// Whether to preserve sharp features based on angle threshold.
    pub preserve_sharp_features: bool,

    /// Angle threshold for sharp feature detection in radians.
    pub sharp_angle_threshold: f64,

    /// Enable edge splitting (split long edges).
    pub enable_split: bool,

    /// Enable edge collapsing (collapse short edges).
    pub enable_collapse: bool,

    /// Enable edge flipping (flip edges to improve quality).
    pub enable_flip: bool,

    /// Enable tangential smoothing.
    pub enable_smooth: bool,

    /// Minimum edge length ratio (edges shorter than target * ratio will be collapsed).
    pub min_edge_ratio: f64,

    /// Maximum edge length ratio (edges longer than target * ratio will be split).
    pub max_edge_ratio: f64,
}

impl Default for RemeshParams {
    fn default() -> Self {
        Self {
            target_edge_length: 1.0,
            iterations: 10,
            preserve_boundaries: true,
            preserve_sharp_features: true,
            sharp_angle_threshold: std::f64::consts::FRAC_PI_4, // 45 degrees
            enable_split: true,
            enable_collapse: true,
            enable_flip: true,
            enable_smooth: true,
            min_edge_ratio: 0.8,  // Collapse if < 80% of target
            max_edge_ratio: 1.33, // Split if > 133% of target (4/3)
        }
    }
}

impl RemeshParams {
    /// Create new parameters with default values.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Create parameters with a specific target edge length.
    #[must_use]
    pub fn with_edge_length(target_edge_length: f64) -> Self {
        Self {
            target_edge_length,
            ..Self::default()
        }
    }

    /// Create parameters for quick/coarse remeshing.
    #[must_use]
    pub fn quick() -> Self {
        Self {
            iterations: 3,
            ..Self::default()
        }
    }

    /// Create parameters for high-quality remeshing.
    #[must_use]
    pub fn high_quality() -> Self {
        Self {
            iterations: 20,
            min_edge_ratio: 0.7,
            max_edge_ratio: 1.5,
            ..Self::default()
        }
    }

    /// Set target edge length.
    #[must_use]
    pub const fn with_target_length(mut self, length: f64) -> Self {
        self.target_edge_length = length;
        self
    }

    /// Set number of iterations.
    #[must_use]
    pub const fn with_iterations(mut self, iterations: u32) -> Self {
        self.iterations = iterations;
        self
    }

    /// Set whether to preserve boundaries.
    #[must_use]
    pub const fn with_preserve_boundaries(mut self, preserve: bool) -> Self {
        self.preserve_boundaries = preserve;
        self
    }

    /// Set whether to preserve sharp features.
    #[must_use]
    pub const fn with_preserve_sharp_features(mut self, preserve: bool) -> Self {
        self.preserve_sharp_features = preserve;
        self
    }

    /// Set sharp angle threshold.
    #[must_use]
    pub const fn with_sharp_angle(mut self, angle_radians: f64) -> Self {
        self.sharp_angle_threshold = angle_radians;
        self
    }

    /// Enable or disable edge splitting.
    #[must_use]
    pub const fn with_split(mut self, enable: bool) -> Self {
        self.enable_split = enable;
        self
    }

    /// Enable or disable edge collapsing.
    #[must_use]
    pub const fn with_collapse(mut self, enable: bool) -> Self {
        self.enable_collapse = enable;
        self
    }

    /// Enable or disable edge flipping.
    #[must_use]
    pub const fn with_flip(mut self, enable: bool) -> Self {
        self.enable_flip = enable;
        self
    }

    /// Enable or disable smoothing.
    #[must_use]
    pub const fn with_smooth(mut self, enable: bool) -> Self {
        self.enable_smooth = enable;
        self
    }

    /// Get the minimum edge length for collapse.
    #[must_use]
    pub fn min_edge_length(&self) -> f64 {
        self.target_edge_length * self.min_edge_ratio
    }

    /// Get the maximum edge length for split.
    #[must_use]
    pub fn max_edge_length(&self) -> f64 {
        self.target_edge_length * self.max_edge_ratio
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_params() {
        let params = RemeshParams::default();
        assert!((params.target_edge_length - 1.0).abs() < 0.001);
        assert_eq!(params.iterations, 10);
        assert!(params.preserve_boundaries);
        assert!(params.enable_split);
        assert!(params.enable_collapse);
        assert!(params.enable_flip);
        assert!(params.enable_smooth);
    }

    #[test]
    fn test_with_edge_length() {
        let params = RemeshParams::with_edge_length(2.5);
        assert!((params.target_edge_length - 2.5).abs() < 0.001);
    }

    #[test]
    fn test_builder() {
        let params = RemeshParams::new()
            .with_target_length(0.5)
            .with_iterations(5)
            .with_preserve_boundaries(false);

        assert!((params.target_edge_length - 0.5).abs() < 0.001);
        assert_eq!(params.iterations, 5);
        assert!(!params.preserve_boundaries);
    }

    #[test]
    fn test_edge_length_thresholds() {
        let params = RemeshParams::with_edge_length(10.0);
        assert!((params.min_edge_length() - 8.0).abs() < 0.001);
        assert!((params.max_edge_length() - 13.3).abs() < 0.1);
    }

    #[test]
    fn test_presets() {
        let quick = RemeshParams::quick();
        assert_eq!(quick.iterations, 3);

        let hq = RemeshParams::high_quality();
        assert_eq!(hq.iterations, 20);
    }
}
