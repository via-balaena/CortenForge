//! Subdivision parameters.

/// Subdivision algorithm to use.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum SubdivisionMethod {
    /// Midpoint subdivision - splits each triangle into 4 by adding edge midpoints.
    /// Fast but doesn't smooth the mesh.
    #[default]
    Midpoint,

    /// Loop subdivision - smoothing subdivision for triangle meshes.
    /// Produces smooth surfaces by averaging vertex positions.
    Loop,

    /// Flat subdivision - like midpoint but keeps original positions.
    /// Useful for increasing face count without changing geometry.
    Flat,
}

impl SubdivisionMethod {
    /// Check if this method produces a smoothed result.
    #[must_use]
    pub const fn is_smoothing(&self) -> bool {
        matches!(self, Self::Loop)
    }
}

/// Parameters for mesh subdivision.
#[derive(Debug, Clone)]
pub struct SubdivideParams {
    /// Subdivision method to use.
    pub method: SubdivisionMethod,

    /// Number of subdivision iterations.
    pub iterations: u32,

    /// Maximum faces allowed in result (prevents memory issues).
    pub max_faces: usize,

    /// Preserve boundary edges (don't smooth boundary vertices).
    pub preserve_boundaries: bool,

    /// Preserve sharp edges based on angle threshold.
    pub preserve_sharp_edges: bool,

    /// Angle threshold for sharp edges in radians.
    pub sharp_angle_threshold: f64,
}

impl Default for SubdivideParams {
    fn default() -> Self {
        Self {
            method: SubdivisionMethod::default(),
            iterations: 1,
            max_faces: 10_000_000, // 10M faces max
            preserve_boundaries: true,
            preserve_sharp_edges: false,
            sharp_angle_threshold: std::f64::consts::FRAC_PI_4, // 45 degrees
        }
    }
}

impl SubdivideParams {
    /// Create new parameters with default values.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Create parameters for Loop subdivision.
    #[must_use]
    pub fn loop_subdivision() -> Self {
        Self {
            method: SubdivisionMethod::Loop,
            ..Self::default()
        }
    }

    /// Create parameters for midpoint subdivision.
    #[must_use]
    pub fn midpoint() -> Self {
        Self {
            method: SubdivisionMethod::Midpoint,
            ..Self::default()
        }
    }

    /// Create parameters for flat subdivision.
    #[must_use]
    pub fn flat() -> Self {
        Self {
            method: SubdivisionMethod::Flat,
            ..Self::default()
        }
    }

    /// Set subdivision method.
    #[must_use]
    pub const fn with_method(mut self, method: SubdivisionMethod) -> Self {
        self.method = method;
        self
    }

    /// Set number of iterations.
    #[must_use]
    pub const fn with_iterations(mut self, iterations: u32) -> Self {
        self.iterations = iterations;
        self
    }

    /// Set maximum faces allowed.
    #[must_use]
    pub const fn with_max_faces(mut self, max_faces: usize) -> Self {
        self.max_faces = max_faces;
        self
    }

    /// Set whether to preserve boundaries.
    #[must_use]
    pub const fn with_preserve_boundaries(mut self, preserve: bool) -> Self {
        self.preserve_boundaries = preserve;
        self
    }

    /// Set whether to preserve sharp edges.
    #[must_use]
    pub const fn with_preserve_sharp_edges(mut self, preserve: bool) -> Self {
        self.preserve_sharp_edges = preserve;
        self
    }

    /// Set sharp angle threshold.
    #[must_use]
    pub const fn with_sharp_angle(mut self, angle_radians: f64) -> Self {
        self.sharp_angle_threshold = angle_radians;
        self
    }

    /// Calculate expected face count after subdivision.
    ///
    /// Each subdivision iteration multiplies face count by 4.
    #[must_use]
    pub const fn expected_faces(&self, current_faces: usize) -> usize {
        let mut faces = current_faces;
        let mut i = 0;
        while i < self.iterations {
            faces *= 4;
            i += 1;
        }
        faces
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_params() {
        let params = SubdivideParams::default();
        assert_eq!(params.method, SubdivisionMethod::Midpoint);
        assert_eq!(params.iterations, 1);
        assert!(params.preserve_boundaries);
    }

    #[test]
    fn test_loop_subdivision() {
        let params = SubdivideParams::loop_subdivision();
        assert_eq!(params.method, SubdivisionMethod::Loop);
        assert!(params.method.is_smoothing());
    }

    #[test]
    fn test_builder() {
        let params = SubdivideParams::new()
            .with_method(SubdivisionMethod::Loop)
            .with_iterations(2)
            .with_max_faces(1_000_000);

        assert_eq!(params.method, SubdivisionMethod::Loop);
        assert_eq!(params.iterations, 2);
        assert_eq!(params.max_faces, 1_000_000);
    }

    #[test]
    fn test_expected_faces() {
        let params = SubdivideParams::new().with_iterations(1);
        assert_eq!(params.expected_faces(100), 400);

        let params = SubdivideParams::new().with_iterations(2);
        assert_eq!(params.expected_faces(100), 1600);

        let params = SubdivideParams::new().with_iterations(3);
        assert_eq!(params.expected_faces(12), 768); // 12 * 4^3
    }

    #[test]
    fn test_is_smoothing() {
        assert!(!SubdivisionMethod::Midpoint.is_smoothing());
        assert!(SubdivisionMethod::Loop.is_smoothing());
        assert!(!SubdivisionMethod::Flat.is_smoothing());
    }
}
