//! Result types for remeshing operations.

// Face/edge counts don't overflow in practice
#![allow(clippy::cast_precision_loss)]

use mesh_types::IndexedMesh;

/// Statistics about edge lengths in a mesh.
#[derive(Debug, Clone, Copy, Default)]
pub struct EdgeStatistics {
    /// Minimum edge length.
    pub min_length: f64,
    /// Maximum edge length.
    pub max_length: f64,
    /// Average edge length.
    pub avg_length: f64,
    /// Standard deviation of edge lengths.
    pub std_dev: f64,
    /// Total number of edges.
    pub edge_count: usize,
}

/// Result of mesh remeshing.
#[derive(Debug, Clone)]
pub struct RemeshResult {
    /// The remeshed mesh.
    pub mesh: IndexedMesh,

    /// Number of faces in original mesh.
    pub original_faces: usize,

    /// Number of faces in remeshed mesh.
    pub final_faces: usize,

    /// Number of vertices in original mesh.
    pub original_vertices: usize,

    /// Number of vertices in remeshed mesh.
    pub final_vertices: usize,

    /// Number of iterations performed.
    pub iterations: u32,

    /// Number of edge splits performed.
    pub splits_performed: usize,

    /// Number of edge collapses performed.
    pub collapses_performed: usize,

    /// Number of edge flips performed.
    pub flips_performed: usize,

    /// Edge statistics for the original mesh.
    pub original_edge_stats: EdgeStatistics,

    /// Edge statistics for the remeshed mesh.
    pub final_edge_stats: EdgeStatistics,
}

impl RemeshResult {
    /// Get the face count change ratio.
    #[must_use]
    pub fn face_ratio(&self) -> f64 {
        if self.original_faces == 0 {
            1.0
        } else {
            self.final_faces as f64 / self.original_faces as f64
        }
    }

    /// Get the vertex count change ratio.
    #[must_use]
    pub fn vertex_ratio(&self) -> f64 {
        if self.original_vertices == 0 {
            1.0
        } else {
            self.final_vertices as f64 / self.original_vertices as f64
        }
    }

    /// Check if any remeshing operations were performed.
    #[must_use]
    pub const fn was_remeshed(&self) -> bool {
        self.splits_performed > 0 || self.collapses_performed > 0 || self.flips_performed > 0
    }

    /// Get the total number of operations performed.
    #[must_use]
    pub const fn total_operations(&self) -> usize {
        self.splits_performed + self.collapses_performed + self.flips_performed
    }

    /// Get the edge length uniformity improvement.
    /// Returns how much the standard deviation improved (positive = better).
    #[must_use]
    pub fn uniformity_improvement(&self) -> f64 {
        if self.original_edge_stats.std_dev > 0.0 {
            (self.original_edge_stats.std_dev - self.final_edge_stats.std_dev)
                / self.original_edge_stats.std_dev
        } else {
            0.0
        }
    }
}

impl std::fmt::Display for RemeshResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Remesh: {} → {} faces ({:.1}x), {} splits, {} collapses, {} flips",
            self.original_faces,
            self.final_faces,
            self.face_ratio(),
            self.splits_performed,
            self.collapses_performed,
            self.flips_performed
        )
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::redundant_clone
)]
mod tests {
    use super::*;

    #[test]
    fn test_face_ratio() {
        let result = RemeshResult {
            mesh: IndexedMesh::new(),
            original_faces: 100,
            final_faces: 200,
            original_vertices: 50,
            final_vertices: 100,
            iterations: 5,
            splits_performed: 100,
            collapses_performed: 10,
            flips_performed: 50,
            original_edge_stats: EdgeStatistics::default(),
            final_edge_stats: EdgeStatistics::default(),
        };

        assert!((result.face_ratio() - 2.0).abs() < 0.001);
        assert!((result.vertex_ratio() - 2.0).abs() < 0.001);
    }

    #[test]
    fn test_was_remeshed() {
        let result = RemeshResult {
            mesh: IndexedMesh::new(),
            original_faces: 100,
            final_faces: 100,
            original_vertices: 50,
            final_vertices: 50,
            iterations: 0,
            splits_performed: 0,
            collapses_performed: 0,
            flips_performed: 0,
            original_edge_stats: EdgeStatistics::default(),
            final_edge_stats: EdgeStatistics::default(),
        };

        assert!(!result.was_remeshed());

        let result2 = RemeshResult {
            splits_performed: 10,
            ..result.clone()
        };
        assert!(result2.was_remeshed());
    }

    #[test]
    fn test_total_operations() {
        let result = RemeshResult {
            mesh: IndexedMesh::new(),
            original_faces: 100,
            final_faces: 200,
            original_vertices: 50,
            final_vertices: 100,
            iterations: 5,
            splits_performed: 100,
            collapses_performed: 10,
            flips_performed: 50,
            original_edge_stats: EdgeStatistics::default(),
            final_edge_stats: EdgeStatistics::default(),
        };

        assert_eq!(result.total_operations(), 160);
    }

    #[test]
    fn test_display() {
        let result = RemeshResult {
            mesh: IndexedMesh::new(),
            original_faces: 100,
            final_faces: 200,
            original_vertices: 50,
            final_vertices: 100,
            iterations: 5,
            splits_performed: 100,
            collapses_performed: 10,
            flips_performed: 50,
            original_edge_stats: EdgeStatistics::default(),
            final_edge_stats: EdgeStatistics::default(),
        };

        let display = format!("{result}");
        assert!(display.contains("100"));
        assert!(display.contains("200"));
        assert!(display.contains("2.0x"));
    }
}
