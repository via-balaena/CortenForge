//! Shell validation utilities.
//!
//! Validates shell meshes to ensure they are suitable for 3D printing.

use mesh_repair::validate_mesh;
use mesh_types::IndexedMesh;
use tracing::{debug, info, warn};

/// Result of shell validation.
#[derive(Debug, Clone)]
pub struct ShellValidationResult {
    /// Whether the shell is watertight (no boundary edges).
    pub is_watertight: bool,
    /// Whether the shell is manifold (no edges with >2 faces).
    pub is_manifold: bool,
    /// Whether the shell has consistent winding order.
    pub has_consistent_winding: bool,
    /// Number of boundary edges (should be 0 for printable shell).
    pub boundary_edge_count: usize,
    /// Number of non-manifold edges (should be 0 for printable shell).
    pub non_manifold_edge_count: usize,
    /// Total vertex count.
    pub vertex_count: usize,
    /// Total face count.
    pub face_count: usize,
    /// List of validation issues found.
    pub issues: Vec<ShellIssue>,
}

impl ShellValidationResult {
    /// Check if the shell passes all validation checks.
    #[must_use]
    pub const fn is_valid(&self) -> bool {
        self.is_watertight && self.is_manifold && self.has_consistent_winding
    }

    /// Check if the shell is suitable for 3D printing.
    #[must_use]
    pub const fn is_printable(&self) -> bool {
        self.is_watertight && self.is_manifold
    }

    /// Get the total number of issues found.
    #[must_use]
    pub const fn issue_count(&self) -> usize {
        self.issues.len()
    }
}

impl std::fmt::Display for ShellValidationResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Shell Validation Result:")?;
        writeln!(f, "  Vertices: {}", self.vertex_count)?;
        writeln!(f, "  Faces: {}", self.face_count)?;
        writeln!(
            f,
            "  Watertight: {} (boundary edges: {})",
            if self.is_watertight { "yes" } else { "NO" },
            self.boundary_edge_count
        )?;
        writeln!(
            f,
            "  Manifold: {} (non-manifold edges: {})",
            if self.is_manifold { "yes" } else { "NO" },
            self.non_manifold_edge_count
        )?;
        writeln!(
            f,
            "  Consistent winding: {}",
            if self.has_consistent_winding {
                "yes"
            } else {
                "NO"
            }
        )?;
        writeln!(
            f,
            "  Printable: {}",
            if self.is_printable() { "yes" } else { "NO" }
        )?;

        if !self.issues.is_empty() {
            writeln!(f, "  Issues ({}):", self.issues.len())?;
            for issue in &self.issues {
                writeln!(f, "    - {issue}")?;
            }
        }

        Ok(())
    }
}

/// Issues that can be found during shell validation.
#[derive(Debug, Clone)]
pub enum ShellIssue {
    /// Shell has boundary edges (not watertight).
    NotWatertight {
        /// Number of boundary edges.
        boundary_edge_count: usize,
    },
    /// Shell has non-manifold edges.
    NonManifold {
        /// Number of non-manifold edges.
        non_manifold_edge_count: usize,
    },
    /// Shell has inconsistent face winding.
    InconsistentWinding,
    /// Shell has zero faces.
    EmptyShell,
    /// Shell has degenerate triangles.
    DegenerateTriangles {
        /// Number of degenerate triangles.
        count: usize,
    },
}

impl std::fmt::Display for ShellIssue {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NotWatertight {
                boundary_edge_count,
            } => {
                write!(
                    f,
                    "Shell is not watertight ({boundary_edge_count} boundary edges)"
                )
            }
            Self::NonManifold {
                non_manifold_edge_count,
            } => {
                write!(
                    f,
                    "Shell is not manifold ({non_manifold_edge_count} non-manifold edges)"
                )
            }
            Self::InconsistentWinding => {
                write!(f, "Shell has inconsistent face winding order")
            }
            Self::EmptyShell => {
                write!(f, "Shell is empty (no faces)")
            }
            Self::DegenerateTriangles { count } => {
                write!(f, "Shell has {count} degenerate triangles")
            }
        }
    }
}

/// Validate a shell mesh for 3D printing suitability.
///
/// Checks:
/// - Watertightness (no boundary edges)
/// - Manifoldness (no edges with >2 adjacent faces)
/// - Consistent winding order
///
/// # Arguments
/// * `shell` - The shell mesh to validate
///
/// # Returns
/// A `ShellValidationResult` with detailed validation information.
#[must_use]
pub fn validate_shell(shell: &IndexedMesh) -> ShellValidationResult {
    info!(
        "Validating shell mesh ({} vertices, {} faces)",
        shell.vertices.len(),
        shell.faces.len()
    );

    let mut issues = Vec::new();

    // Check for empty shell
    if shell.faces.is_empty() {
        issues.push(ShellIssue::EmptyShell);
        return ShellValidationResult {
            is_watertight: false,
            is_manifold: false,
            has_consistent_winding: false,
            boundary_edge_count: 0,
            non_manifold_edge_count: 0,
            vertex_count: shell.vertices.len(),
            face_count: 0,
            issues,
        };
    }

    // Use mesh-repair's validation to check topology
    let mesh_report = validate_mesh(shell);

    let boundary_edge_count = mesh_report.boundary_edge_count;
    let non_manifold_edge_count = mesh_report.non_manifold_edge_count;

    // Check watertightness
    let is_watertight = boundary_edge_count == 0;
    if !is_watertight {
        issues.push(ShellIssue::NotWatertight {
            boundary_edge_count,
        });
        warn!(
            "Shell is not watertight: {} boundary edges",
            boundary_edge_count
        );
    }

    // Check manifoldness
    let is_manifold = non_manifold_edge_count == 0;
    if !is_manifold {
        issues.push(ShellIssue::NonManifold {
            non_manifold_edge_count,
        });
        warn!(
            "Shell is not manifold: {} non-manifold edges",
            non_manifold_edge_count
        );
    }

    // Check winding consistency
    let has_consistent_winding = check_winding_consistency(shell);
    if !has_consistent_winding {
        issues.push(ShellIssue::InconsistentWinding);
        warn!("Shell has inconsistent winding order");
    }

    // Check for degenerate triangles
    let degenerate_count = count_degenerate_triangles(shell);
    if degenerate_count > 0 {
        issues.push(ShellIssue::DegenerateTriangles {
            count: degenerate_count,
        });
        warn!("Shell has {} degenerate triangles", degenerate_count);
    }

    let result = ShellValidationResult {
        is_watertight,
        is_manifold,
        has_consistent_winding,
        boundary_edge_count,
        non_manifold_edge_count,
        vertex_count: shell.vertices.len(),
        face_count: shell.faces.len(),
        issues,
    };

    if result.is_printable() {
        info!("Shell validation passed - mesh is printable");
    } else {
        warn!("Shell validation found {} issue(s)", result.issue_count());
    }

    debug!("{}", result);

    result
}

/// Check if the mesh has consistent winding order.
///
/// For a valid closed mesh, adjacent faces should have opposite winding
/// along their shared edge (so normals point consistently outward).
fn check_winding_consistency(mesh: &IndexedMesh) -> bool {
    use hashbrown::HashMap;

    // Build edge to faces mapping ourselves
    let mut edge_to_faces: HashMap<(u32, u32), Vec<usize>> = HashMap::new();

    for (face_idx, face) in mesh.faces.iter().enumerate() {
        for i in 0..3 {
            let v0 = face[i];
            let v1 = face[(i + 1) % 3];
            // Normalize edge direction for grouping
            let edge = if v0 < v1 { (v0, v1) } else { (v1, v0) };
            edge_to_faces.entry(edge).or_default().push(face_idx);
        }
    }

    // For each edge with exactly 2 adjacent faces, check winding consistency
    for (&edge, face_indices) in &edge_to_faces {
        if face_indices.len() != 2 {
            // Skip boundary or non-manifold edges
            continue;
        }

        let face_a = mesh.faces[face_indices[0]];
        let face_b = mesh.faces[face_indices[1]];

        // Find the shared edge orientation in each face
        let edge_in_a = find_edge_direction(&face_a, edge);
        let edge_in_b = find_edge_direction(&face_b, edge);

        // For consistent winding, the edge should appear in opposite directions
        // in the two adjacent faces
        if edge_in_a == edge_in_b {
            return false;
        }
    }

    true
}

/// Find the direction of an edge in a face.
/// Returns true if edge goes v0->v1 in the face's winding order, false if v1->v0.
fn find_edge_direction(face: &[u32; 3], edge: (u32, u32)) -> bool {
    let (v0, v1) = edge;

    // Check all three edges of the triangle
    for i in 0..3 {
        let a = face[i];
        let b = face[(i + 1) % 3];

        if a == v0 && b == v1 {
            return true; // Forward direction
        }
        if a == v1 && b == v0 {
            return false; // Reverse direction
        }
    }

    // Edge not found in face (shouldn't happen with valid adjacency)
    true
}

/// Count degenerate triangles in the mesh.
fn count_degenerate_triangles(mesh: &IndexedMesh) -> usize {
    const DEGENERATE_THRESHOLD: f64 = 1e-10;

    mesh.faces
        .iter()
        .filter(|face| {
            let v0 = &mesh.vertices[face[0] as usize].position;
            let v1 = &mesh.vertices[face[1] as usize].position;
            let v2 = &mesh.vertices[face[2] as usize].position;

            let edge1 = v1 - v0;
            let edge2 = v2 - v0;
            let cross = edge1.cross(&edge2);
            let area = cross.norm() / 2.0;

            area < DEGENERATE_THRESHOLD
        })
        .count()
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    fn create_watertight_tetrahedron() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(5.0, 10.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(5.0, 5.0, 10.0));

        // Faces with consistent outward winding
        mesh.faces.push([0, 2, 1]); // Bottom
        mesh.faces.push([0, 1, 3]); // Front
        mesh.faces.push([1, 2, 3]); // Right
        mesh.faces.push([2, 0, 3]); // Left

        mesh
    }

    fn create_open_box() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();

        // 8 vertices
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 10.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 10.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 10.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 10.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 10.0, 10.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 10.0, 10.0));

        // 5 faces (open top)
        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([0, 3, 2]);
        mesh.faces.push([0, 1, 5]);
        mesh.faces.push([0, 5, 4]);
        mesh.faces.push([2, 3, 7]);
        mesh.faces.push([2, 7, 6]);
        mesh.faces.push([0, 4, 7]);
        mesh.faces.push([0, 7, 3]);
        mesh.faces.push([1, 2, 6]);
        mesh.faces.push([1, 6, 5]);

        mesh
    }

    #[test]
    fn test_validate_watertight_shell() {
        let shell = create_watertight_tetrahedron();
        let result = validate_shell(&shell);

        assert!(result.is_watertight);
        assert!(result.is_manifold);
        assert!(result.is_printable());
        assert_eq!(result.boundary_edge_count, 0);
        assert_eq!(result.non_manifold_edge_count, 0);
    }

    #[test]
    fn test_validate_open_shell() {
        let shell = create_open_box();
        let result = validate_shell(&shell);

        assert!(!result.is_watertight);
        assert!(result.is_manifold);
        assert!(!result.is_printable());
        assert!(result.boundary_edge_count > 0);
        assert!(
            result
                .issues
                .iter()
                .any(|i| matches!(i, ShellIssue::NotWatertight { .. }))
        );
    }

    #[test]
    fn test_validate_empty_shell() {
        let shell = IndexedMesh::new();
        let result = validate_shell(&shell);

        assert!(!result.is_valid());
        assert!(!result.is_printable());
        assert!(
            result
                .issues
                .iter()
                .any(|i| matches!(i, ShellIssue::EmptyShell))
        );
    }

    #[test]
    fn test_shell_validation_result_display() {
        let shell = create_watertight_tetrahedron();
        let result = validate_shell(&shell);
        let output = format!("{result}");

        assert!(output.contains("Vertices:"));
        assert!(output.contains("Faces:"));
        assert!(output.contains("Watertight: yes"));
        assert!(output.contains("Manifold: yes"));
        assert!(output.contains("Printable: yes"));
    }

    #[test]
    fn test_shell_issue_display() {
        let issue = ShellIssue::NotWatertight {
            boundary_edge_count: 4,
        };
        let output = format!("{issue}");
        assert!(output.contains("watertight"));
        assert!(output.contains('4'));

        let issue = ShellIssue::NonManifold {
            non_manifold_edge_count: 2,
        };
        let output = format!("{issue}");
        assert!(output.contains("manifold"));
        assert!(output.contains('2'));

        let issue = ShellIssue::InconsistentWinding;
        let output = format!("{issue}");
        assert!(output.contains("winding"));

        let issue = ShellIssue::EmptyShell;
        let output = format!("{issue}");
        assert!(output.contains("empty"));

        let issue = ShellIssue::DegenerateTriangles { count: 5 };
        let output = format!("{issue}");
        assert!(output.contains("degenerate"));
        assert!(output.contains('5'));
    }

    #[test]
    fn test_degenerate_triangle_detection() {
        let mut mesh = create_watertight_tetrahedron();

        // Add a degenerate triangle (all vertices at same position)
        let idx = mesh.vertices.len() as u32;
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.faces.push([idx, idx + 1, idx + 2]);

        let count = count_degenerate_triangles(&mesh);
        assert_eq!(count, 1);
    }
}
