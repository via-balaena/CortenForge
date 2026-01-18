//! Result types for decimation operations.

// Triangle counts don't overflow in practice
#![allow(clippy::cast_precision_loss)]

use mesh_types::IndexedMesh;

/// Result of mesh decimation.
#[derive(Debug, Clone)]
pub struct DecimationResult {
    /// The decimated mesh.
    pub mesh: IndexedMesh,

    /// Number of triangles in original mesh.
    pub original_triangles: usize,

    /// Number of triangles in decimated mesh.
    pub final_triangles: usize,

    /// Number of edge collapses performed.
    pub collapses_performed: usize,

    /// Number of edge collapses rejected (e.g., would create non-manifold).
    pub collapses_rejected: usize,
}

impl DecimationResult {
    /// Get the reduction ratio (final / original).
    #[must_use]
    pub fn reduction_ratio(&self) -> f64 {
        if self.original_triangles == 0 {
            1.0
        } else {
            self.final_triangles as f64 / self.original_triangles as f64
        }
    }

    /// Get the percentage of triangles removed.
    #[must_use]
    pub fn reduction_percent(&self) -> f64 {
        (1.0 - self.reduction_ratio()) * 100.0
    }

    /// Check if any decimation occurred.
    #[must_use]
    pub const fn was_decimated(&self) -> bool {
        self.collapses_performed > 0
    }
}

impl std::fmt::Display for DecimationResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Decimation: {} → {} triangles ({:.1}% reduction, {} collapses)",
            self.original_triangles,
            self.final_triangles,
            self.reduction_percent(),
            self.collapses_performed
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_reduction_ratio() {
        let result = DecimationResult {
            mesh: IndexedMesh::new(),
            original_triangles: 1000,
            final_triangles: 500,
            collapses_performed: 250,
            collapses_rejected: 10,
        };

        assert!((result.reduction_ratio() - 0.5).abs() < 0.001);
        assert!((result.reduction_percent() - 50.0).abs() < 0.1);
    }

    #[test]
    fn test_was_decimated() {
        let result = DecimationResult {
            mesh: IndexedMesh::new(),
            original_triangles: 1000,
            final_triangles: 1000,
            collapses_performed: 0,
            collapses_rejected: 0,
        };

        assert!(!result.was_decimated());

        let result2 = DecimationResult {
            mesh: IndexedMesh::new(),
            original_triangles: 1000,
            final_triangles: 500,
            collapses_performed: 250,
            collapses_rejected: 0,
        };

        assert!(result2.was_decimated());
    }

    #[test]
    fn test_display() {
        let result = DecimationResult {
            mesh: IndexedMesh::new(),
            original_triangles: 1000,
            final_triangles: 500,
            collapses_performed: 250,
            collapses_rejected: 10,
        };

        let display = format!("{result}");
        assert!(display.contains("1000"));
        assert!(display.contains("500"));
        assert!(display.contains("50.0%"));
    }
}
