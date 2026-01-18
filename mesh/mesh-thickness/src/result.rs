//! Result types for thickness analysis.

use mesh_types::Point3;

/// A region identified as having thin walls.
#[derive(Debug, Clone)]
pub struct ThinRegion {
    /// Vertex index where thin wall was detected.
    pub vertex_index: usize,
    /// Position of the vertex.
    pub position: Point3<f64>,
    /// Measured wall thickness at this point.
    pub thickness: f64,
    /// Face index that the inward ray hit.
    pub hit_face: Option<usize>,
}

/// Result of wall thickness analysis.
///
/// # Example
///
/// ```
/// use mesh_thickness::AnalysisResult;
///
/// // Check if the analysis found thin regions
/// let result = AnalysisResult::default();
/// assert!(!result.has_thin_regions());
/// ```
#[derive(Debug, Clone, Default)]
pub struct AnalysisResult {
    /// Per-vertex thickness measurements (`f64::INFINITY` if no hit).
    pub vertex_thickness: Vec<f64>,
    /// Minimum thickness found across all vertices.
    pub min_thickness: f64,
    /// Maximum thickness found across all vertices.
    pub max_thickness: f64,
    /// Average thickness (excluding infinite values).
    pub avg_thickness: f64,
    /// List of thin regions (below threshold).
    pub thin_regions: Vec<ThinRegion>,
    /// Number of vertices analyzed.
    pub vertices_analyzed: usize,
    /// Number of vertices with valid thickness (ray hit mesh).
    pub vertices_with_hits: usize,
    /// Number of vertices skipped (no normal or no hit).
    pub vertices_skipped: usize,
    /// Whether search was truncated due to `max_regions`.
    pub truncated: bool,
}

impl AnalysisResult {
    /// Create an empty result for an empty mesh.
    #[must_use]
    pub fn empty() -> Self {
        Self {
            min_thickness: f64::INFINITY,
            max_thickness: f64::NEG_INFINITY,
            ..Default::default()
        }
    }

    /// Check if the mesh has any thin regions.
    #[must_use]
    pub fn has_thin_regions(&self) -> bool {
        !self.thin_regions.is_empty()
    }

    /// Check if all analyzed vertices are above the minimum thickness.
    #[must_use]
    pub fn is_thick_enough(&self, min_thickness: f64) -> bool {
        self.min_thickness >= min_thickness
    }

    /// Get vertices below a given thickness threshold.
    #[must_use]
    pub fn vertices_below_thickness(&self, threshold: f64) -> Vec<usize> {
        self.vertex_thickness
            .iter()
            .enumerate()
            .filter(|&(_, t)| *t < threshold && t.is_finite())
            .map(|(i, _)| i)
            .collect()
    }

    /// Get the percentage of vertices with valid thickness measurements.
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn coverage_percent(&self) -> f64 {
        if self.vertices_analyzed == 0 {
            0.0
        } else {
            100.0 * (self.vertices_with_hits as f64) / (self.vertices_analyzed as f64)
        }
    }
}

impl std::fmt::Display for AnalysisResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Wall Thickness Analysis:")?;
        writeln!(f, "  Vertices analyzed: {}", self.vertices_analyzed)?;
        writeln!(f, "  Vertices with hits: {}", self.vertices_with_hits)?;
        writeln!(f, "  Coverage: {:.1}%", self.coverage_percent())?;
        writeln!(f, "  Min thickness: {:.3}", self.min_thickness)?;
        writeln!(f, "  Max thickness: {:.3}", self.max_thickness)?;
        writeln!(f, "  Avg thickness: {:.3}", self.avg_thickness)?;
        writeln!(f, "  Thin regions: {}", self.thin_regions.len())?;
        if self.truncated {
            writeln!(f, "  (truncated, more thin regions exist)")?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_result() {
        let result = AnalysisResult::empty();
        assert!(!result.has_thin_regions());
        assert_eq!(result.vertices_analyzed, 0);
    }

    #[test]
    fn test_has_thin_regions() {
        let mut result = AnalysisResult::empty();
        assert!(!result.has_thin_regions());

        result.thin_regions.push(ThinRegion {
            vertex_index: 0,
            position: Point3::origin(),
            thickness: 0.5,
            hit_face: Some(0),
        });
        assert!(result.has_thin_regions());
    }

    #[test]
    fn test_is_thick_enough() {
        let mut result = AnalysisResult::empty();
        result.min_thickness = 2.0;

        assert!(result.is_thick_enough(1.0));
        assert!(result.is_thick_enough(2.0));
        assert!(!result.is_thick_enough(3.0));
    }

    #[test]
    fn test_vertices_below_thickness() {
        let result = AnalysisResult {
            vertex_thickness: vec![0.5, 1.5, 2.5, f64::INFINITY],
            min_thickness: 0.5,
            max_thickness: 2.5,
            avg_thickness: 1.5,
            thin_regions: Vec::new(),
            vertices_analyzed: 4,
            vertices_with_hits: 3,
            vertices_skipped: 1,
            truncated: false,
        };

        let below_1 = result.vertices_below_thickness(1.0);
        assert_eq!(below_1, vec![0]);

        let below_2 = result.vertices_below_thickness(2.0);
        assert_eq!(below_2, vec![0, 1]);

        let below_3 = result.vertices_below_thickness(3.0);
        assert_eq!(below_3, vec![0, 1, 2]);
    }

    #[test]
    fn test_coverage_percent() {
        let result = AnalysisResult {
            vertices_analyzed: 100,
            vertices_with_hits: 80,
            ..AnalysisResult::empty()
        };

        assert!((result.coverage_percent() - 80.0).abs() < 0.1);
    }

    #[test]
    fn test_display() {
        let result = AnalysisResult {
            vertex_thickness: vec![1.0, 2.0, 3.0],
            min_thickness: 1.0,
            max_thickness: 3.0,
            avg_thickness: 2.0,
            thin_regions: vec![ThinRegion {
                vertex_index: 0,
                position: Point3::origin(),
                thickness: 1.0,
                hit_face: Some(0),
            }],
            vertices_analyzed: 3,
            vertices_with_hits: 3,
            vertices_skipped: 0,
            truncated: false,
        };

        let output = format!("{result}");
        assert!(output.contains("Wall Thickness"));
        assert!(output.contains("Min thickness: 1.000"));
        assert!(output.contains("Thin regions: 1"));
    }
}
