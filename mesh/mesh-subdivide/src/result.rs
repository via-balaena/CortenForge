//! Result types for subdivision operations.

// Face counts don't overflow in practice
#![allow(clippy::cast_precision_loss)]

use mesh_types::IndexedMesh;

use crate::params::SubdivisionMethod;

/// Result of mesh subdivision.
#[derive(Debug, Clone)]
pub struct SubdivisionResult {
    /// The subdivided mesh.
    pub mesh: IndexedMesh,

    /// Number of faces in original mesh.
    pub original_faces: usize,

    /// Number of faces in subdivided mesh.
    pub final_faces: usize,

    /// Number of vertices in original mesh.
    pub original_vertices: usize,

    /// Number of vertices in subdivided mesh.
    pub final_vertices: usize,

    /// Number of iterations performed.
    pub iterations: u32,

    /// Subdivision method used.
    pub method: SubdivisionMethod,
}

impl SubdivisionResult {
    /// Get the face multiplication factor.
    #[must_use]
    pub fn face_ratio(&self) -> f64 {
        if self.original_faces == 0 {
            1.0
        } else {
            self.final_faces as f64 / self.original_faces as f64
        }
    }

    /// Get the vertex multiplication factor.
    #[must_use]
    pub fn vertex_ratio(&self) -> f64 {
        if self.original_vertices == 0 {
            1.0
        } else {
            self.final_vertices as f64 / self.original_vertices as f64
        }
    }

    /// Check if any subdivision occurred.
    #[must_use]
    pub const fn was_subdivided(&self) -> bool {
        self.iterations > 0 && self.final_faces > self.original_faces
    }
}

impl std::fmt::Display for SubdivisionResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Subdivision ({:?}): {} → {} faces ({:.1}x), {} iterations",
            self.method,
            self.original_faces,
            self.final_faces,
            self.face_ratio(),
            self.iterations
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_face_ratio() {
        let result = SubdivisionResult {
            mesh: IndexedMesh::new(),
            original_faces: 100,
            final_faces: 400,
            original_vertices: 50,
            final_vertices: 200,
            iterations: 1,
            method: SubdivisionMethod::Midpoint,
        };

        assert!((result.face_ratio() - 4.0).abs() < 0.001);
        assert!((result.vertex_ratio() - 4.0).abs() < 0.001);
    }

    #[test]
    fn test_was_subdivided() {
        let result = SubdivisionResult {
            mesh: IndexedMesh::new(),
            original_faces: 100,
            final_faces: 100,
            original_vertices: 50,
            final_vertices: 50,
            iterations: 0,
            method: SubdivisionMethod::Midpoint,
        };

        assert!(!result.was_subdivided());

        let result2 = SubdivisionResult {
            mesh: IndexedMesh::new(),
            original_faces: 100,
            final_faces: 400,
            original_vertices: 50,
            final_vertices: 200,
            iterations: 1,
            method: SubdivisionMethod::Loop,
        };

        assert!(result2.was_subdivided());
    }

    #[test]
    fn test_display() {
        let result = SubdivisionResult {
            mesh: IndexedMesh::new(),
            original_faces: 100,
            final_faces: 400,
            original_vertices: 50,
            final_vertices: 200,
            iterations: 1,
            method: SubdivisionMethod::Midpoint,
        };

        let display = format!("{result}");
        assert!(display.contains("100"));
        assert!(display.contains("400"));
        assert!(display.contains("4.0x"));
        assert!(display.contains("Midpoint"));
    }
}
