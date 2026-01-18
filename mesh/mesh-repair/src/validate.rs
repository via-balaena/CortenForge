//! Mesh validation and health reporting.
//!
//! Checks meshes for common issues that can cause problems in downstream processing.

use mesh_types::IndexedMesh;

use crate::adjacency::MeshAdjacency;

/// Report of mesh validation results.
///
/// Contains counts of various mesh issues and methods to check
/// overall mesh health.
#[derive(Debug, Clone, Default)]
pub struct MeshReport {
    /// Total number of vertices.
    pub vertex_count: usize,
    /// Total number of faces.
    pub face_count: usize,
    /// Total number of edges.
    pub edge_count: usize,

    /// Number of boundary edges (edges with only one adjacent face).
    pub boundary_edge_count: usize,
    /// Number of non-manifold edges (edges with more than two adjacent faces).
    pub non_manifold_edge_count: usize,
    /// Number of degenerate faces (zero or near-zero area).
    pub degenerate_face_count: usize,
    /// Number of duplicate faces.
    pub duplicate_face_count: usize,

    /// Whether the mesh is watertight (no boundary edges).
    pub is_watertight: bool,
    /// Whether the mesh is manifold (no non-manifold edges).
    pub is_manifold: bool,
    /// Whether the mesh appears to be inside-out (majority of volume is negative).
    pub is_inside_out: bool,
}

impl MeshReport {
    /// Check if the mesh is ready for 3D printing.
    ///
    /// A printable mesh must be watertight, manifold, and have correct winding.
    #[must_use]
    pub fn is_printable(&self) -> bool {
        self.is_watertight && self.is_manifold && !self.is_inside_out
    }

    /// Check if the mesh has any issues.
    #[must_use]
    pub fn has_issues(&self) -> bool {
        self.boundary_edge_count > 0
            || self.non_manifold_edge_count > 0
            || self.degenerate_face_count > 0
            || self.duplicate_face_count > 0
    }

    /// Get a count of total issues found.
    #[must_use]
    pub fn issue_count(&self) -> usize {
        self.boundary_edge_count
            + self.non_manifold_edge_count
            + self.degenerate_face_count
            + self.duplicate_face_count
    }
}

impl std::fmt::Display for MeshReport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Mesh Report:")?;
        writeln!(f, "  Vertices: {}", self.vertex_count)?;
        writeln!(f, "  Faces: {}", self.face_count)?;
        writeln!(f, "  Edges: {}", self.edge_count)?;
        writeln!(f)?;
        writeln!(f, "  Status:")?;
        writeln!(
            f,
            "    Watertight: {}",
            if self.is_watertight { "Yes" } else { "No" }
        )?;
        writeln!(
            f,
            "    Manifold: {}",
            if self.is_manifold { "Yes" } else { "No" }
        )?;
        writeln!(
            f,
            "    Winding: {}",
            if self.is_inside_out {
                "Inside-out"
            } else {
                "Correct"
            }
        )?;

        if self.has_issues() {
            writeln!(f)?;
            writeln!(f, "  Issues:")?;
            if self.boundary_edge_count > 0 {
                writeln!(f, "    Boundary edges: {}", self.boundary_edge_count)?;
            }
            if self.non_manifold_edge_count > 0 {
                writeln!(
                    f,
                    "    Non-manifold edges: {}",
                    self.non_manifold_edge_count
                )?;
            }
            if self.degenerate_face_count > 0 {
                writeln!(f, "    Degenerate faces: {}", self.degenerate_face_count)?;
            }
            if self.duplicate_face_count > 0 {
                writeln!(f, "    Duplicate faces: {}", self.duplicate_face_count)?;
            }
        }

        Ok(())
    }
}

/// Options for mesh validation.
#[derive(Debug, Clone)]
pub struct ValidationOptions {
    /// Area threshold below which a face is considered degenerate.
    pub degenerate_area_threshold: f64,
    /// Whether to check for inside-out meshes.
    pub check_winding: bool,
}

impl Default for ValidationOptions {
    fn default() -> Self {
        Self {
            degenerate_area_threshold: 1e-12,
            check_winding: true,
        }
    }
}

/// Validate a mesh and return a report of any issues.
///
/// # Arguments
///
/// * `mesh` - The mesh to validate
///
/// # Example
///
/// ```
/// use mesh_types::{IndexedMesh, Vertex};
/// use mesh_repair::validate_mesh;
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
///
/// let report = validate_mesh(&mesh);
/// assert_eq!(report.face_count, 1);
/// assert_eq!(report.boundary_edge_count, 3); // Single triangle has 3 boundary edges
/// ```
#[must_use]
pub fn validate_mesh(mesh: &IndexedMesh) -> MeshReport {
    validate_mesh_with_options(mesh, &ValidationOptions::default())
}

/// Validate a mesh with custom options.
///
/// # Arguments
///
/// * `mesh` - The mesh to validate
/// * `options` - Validation options
#[must_use]
pub fn validate_mesh_with_options(mesh: &IndexedMesh, options: &ValidationOptions) -> MeshReport {
    let adjacency = MeshAdjacency::build(&mesh.faces);

    let degenerate_face_count = count_degenerate_faces(mesh, options.degenerate_area_threshold);
    let duplicate_face_count = count_duplicate_faces(&mesh.faces);
    let is_inside_out = if options.check_winding {
        check_inside_out(mesh)
    } else {
        false
    };

    MeshReport {
        vertex_count: mesh.vertices.len(),
        face_count: mesh.faces.len(),
        edge_count: adjacency.edge_count(),
        boundary_edge_count: adjacency.boundary_edge_count(),
        non_manifold_edge_count: adjacency.non_manifold_edge_count(),
        degenerate_face_count,
        duplicate_face_count,
        is_watertight: adjacency.is_watertight(),
        is_manifold: adjacency.is_manifold(),
        is_inside_out,
    }
}

/// Count faces with area below the threshold.
fn count_degenerate_faces(mesh: &IndexedMesh, area_threshold: f64) -> usize {
    mesh.faces
        .iter()
        .filter(|face| {
            let v0 = &mesh.vertices[face[0] as usize].position;
            let v1 = &mesh.vertices[face[1] as usize].position;
            let v2 = &mesh.vertices[face[2] as usize].position;

            let e1 = *v1 - *v0;
            let e2 = *v2 - *v0;
            let cross = e1.cross(&e2);
            let area = cross.norm() * 0.5;

            area < area_threshold
        })
        .count()
}

/// Count duplicate faces.
fn count_duplicate_faces(faces: &[[u32; 3]]) -> usize {
    use hashbrown::HashSet;

    let mut seen: HashSet<[u32; 3]> = HashSet::new();
    let mut duplicates = 0;

    for face in faces {
        let normalized = normalize_face(*face);
        let reversed = normalize_face([face[0], face[2], face[1]]);

        if seen.contains(&normalized) || seen.contains(&reversed) {
            duplicates += 1;
        } else {
            seen.insert(normalized);
        }
    }

    duplicates
}

/// Normalize a face so the smallest vertex index comes first.
fn normalize_face(face: [u32; 3]) -> [u32; 3] {
    let min_idx = if face[0] <= face[1] && face[0] <= face[2] {
        0
    } else if face[1] <= face[2] {
        1
    } else {
        2
    };

    [
        face[min_idx],
        face[(min_idx + 1) % 3],
        face[(min_idx + 2) % 3],
    ]
}

/// Check if the mesh appears to be inside-out based on signed volume.
fn check_inside_out(mesh: &IndexedMesh) -> bool {
    if mesh.faces.is_empty() {
        return false;
    }

    // Compute signed volume using the divergence theorem
    let mut volume = 0.0;

    for face in &mesh.faces {
        let v0 = &mesh.vertices[face[0] as usize].position;
        let v1 = &mesh.vertices[face[1] as usize].position;
        let v2 = &mesh.vertices[face[2] as usize].position;

        // Signed volume of tetrahedron formed with origin
        volume += v0.x * (v1.y * v2.z - v2.y * v1.z)
            + v1.x * (v2.y * v0.z - v0.y * v2.z)
            + v2.x * (v0.y * v1.z - v1.y * v0.z);
    }

    volume / 6.0 < 0.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    fn simple_triangle() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 10.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh
    }

    fn unit_tetrahedron() -> IndexedMesh {
        // A closed tetrahedron with correct winding
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 0.866, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 0.289, 0.816));

        // CCW winding when viewed from outside
        mesh.faces.push([0, 2, 1]); // bottom
        mesh.faces.push([0, 1, 3]); // front
        mesh.faces.push([1, 2, 3]); // right
        mesh.faces.push([2, 0, 3]); // left
        mesh
    }

    #[test]
    fn validate_single_triangle() {
        let mesh = simple_triangle();
        let report = validate_mesh(&mesh);

        assert_eq!(report.vertex_count, 3);
        assert_eq!(report.face_count, 1);
        assert_eq!(report.boundary_edge_count, 3);
        assert!(!report.is_watertight);
    }

    #[test]
    fn validate_tetrahedron() {
        let mesh = unit_tetrahedron();
        let report = validate_mesh(&mesh);

        assert_eq!(report.vertex_count, 4);
        assert_eq!(report.face_count, 4);
        assert_eq!(report.boundary_edge_count, 0);
        assert!(report.is_watertight);
        assert!(report.is_manifold);
    }

    #[test]
    fn detect_degenerate_faces() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(2.0, 0.0, 0.0)); // Collinear!
        mesh.faces.push([0, 1, 2]);

        let report = validate_mesh(&mesh);
        assert_eq!(report.degenerate_face_count, 1);
    }

    #[test]
    fn detect_duplicate_faces() {
        let mut mesh = simple_triangle();
        mesh.faces.push([0, 1, 2]); // Duplicate

        let report = validate_mesh(&mesh);
        assert_eq!(report.duplicate_face_count, 1);
    }

    #[test]
    fn detect_duplicate_faces_reversed() {
        let mut mesh = simple_triangle();
        mesh.faces.push([0, 2, 1]); // Same face, reversed winding

        let report = validate_mesh(&mesh);
        assert_eq!(report.duplicate_face_count, 1);
    }

    #[test]
    fn printability_check() {
        let mesh = unit_tetrahedron();
        let report = validate_mesh(&mesh);

        assert!(report.is_printable());
    }

    #[test]
    fn printability_fails_with_holes() {
        let mesh = simple_triangle();
        let report = validate_mesh(&mesh);

        assert!(!report.is_printable()); // Has holes
    }

    #[test]
    fn report_display() {
        let mesh = simple_triangle();
        let report = validate_mesh(&mesh);
        let display = format!("{}", report);

        assert!(display.contains("Vertices: 3"));
        assert!(display.contains("Watertight: No"));
    }

    #[test]
    fn has_issues_empty_mesh() {
        let report = MeshReport::default();
        assert!(!report.has_issues());
    }

    #[test]
    fn issue_count() {
        let report = MeshReport {
            boundary_edge_count: 3,
            degenerate_face_count: 2,
            ..Default::default()
        };

        assert_eq!(report.issue_count(), 5);
    }
}
