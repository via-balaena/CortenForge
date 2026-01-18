//! Parameters for wall thickness analysis.

/// Parameters for wall thickness analysis.
///
/// # Example
///
/// ```
/// use mesh_thickness::ThicknessParams;
///
/// // Use default parameters (1mm minimum thickness)
/// let params = ThicknessParams::default();
/// assert!((params.min_thickness - 1.0).abs() < 1e-10);
///
/// // Use FDM printing parameters (0.8mm minimum)
/// let fdm = ThicknessParams::for_printing();
/// assert!((fdm.min_thickness - 0.8).abs() < 1e-10);
///
/// // Use SLA printing parameters (0.4mm minimum)
/// let sla = ThicknessParams::for_sla();
/// assert!((sla.min_thickness - 0.4).abs() < 1e-10);
/// ```
#[derive(Debug, Clone)]
pub struct ThicknessParams {
    /// Minimum acceptable wall thickness in mesh units (typically mm).
    /// Regions thinner than this will be flagged.
    pub min_thickness: f64,

    /// Maximum ray distance to trace. Set to 0 for unlimited.
    pub max_ray_distance: f64,

    /// Epsilon for ray-triangle intersection tests.
    pub epsilon: f64,

    /// Maximum number of thin regions to report.
    pub max_regions: usize,

    /// Whether to skip vertices without normals.
    pub require_normals: bool,
}

impl Default for ThicknessParams {
    fn default() -> Self {
        Self {
            min_thickness: 1.0,       // 1mm default minimum
            max_ray_distance: 1000.0, // 1m max ray distance
            epsilon: 1e-8,
            max_regions: 1000,
            require_normals: false,
        }
    }
}

impl ThicknessParams {
    /// Create params for FDM 3D printing analysis.
    ///
    /// Uses typical FDM printer requirements (0.8mm minimum wall).
    #[must_use]
    pub const fn for_printing() -> Self {
        Self {
            min_thickness: 0.8, // 0.8mm minimum for FDM
            max_ray_distance: 500.0,
            epsilon: 1e-8,
            max_regions: 1000,
            require_normals: false,
        }
    }

    /// Create params for SLA/resin printing.
    ///
    /// SLA can handle thinner walls (0.4mm minimum).
    #[must_use]
    pub const fn for_sla() -> Self {
        Self {
            min_thickness: 0.4, // 0.4mm minimum for SLA
            max_ray_distance: 500.0,
            epsilon: 1e-8,
            max_regions: 1000,
            require_normals: false,
        }
    }

    /// Create params with a custom minimum thickness.
    #[must_use]
    pub const fn with_min_thickness(min_thickness: f64) -> Self {
        Self {
            min_thickness,
            max_ray_distance: 1000.0,
            epsilon: 1e-8,
            max_regions: 1000,
            require_normals: false,
        }
    }

    /// Set the minimum thickness.
    #[must_use]
    pub const fn min_thickness(mut self, thickness: f64) -> Self {
        self.min_thickness = thickness;
        self
    }

    /// Set the maximum ray distance.
    #[must_use]
    pub const fn max_ray_distance(mut self, distance: f64) -> Self {
        self.max_ray_distance = distance;
        self
    }

    /// Set the maximum number of thin regions to report.
    #[must_use]
    pub const fn max_regions(mut self, count: usize) -> Self {
        self.max_regions = count;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_params() {
        let params = ThicknessParams::default();
        assert!((params.min_thickness - 1.0).abs() < f64::EPSILON);
        assert!((params.max_ray_distance - 1000.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_fdm_params() {
        let params = ThicknessParams::for_printing();
        assert!((params.min_thickness - 0.8).abs() < 0.01);
    }

    #[test]
    fn test_sla_params() {
        let params = ThicknessParams::for_sla();
        assert!((params.min_thickness - 0.4).abs() < 0.01);
    }

    #[test]
    fn test_custom_params() {
        let params = ThicknessParams::with_min_thickness(1.5);
        assert!((params.min_thickness - 1.5).abs() < 0.01);
    }

    #[test]
    fn test_builder_pattern() {
        let params = ThicknessParams::default()
            .min_thickness(2.0)
            .max_ray_distance(500.0)
            .max_regions(100);

        assert!((params.min_thickness - 2.0).abs() < f64::EPSILON);
        assert!((params.max_ray_distance - 500.0).abs() < f64::EPSILON);
        assert_eq!(params.max_regions, 100);
    }
}
