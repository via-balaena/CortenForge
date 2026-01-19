//! Main validation logic for printability analysis.
//!
//! Provides the primary validation functionality to check meshes
//! against printer constraints.

use mesh_types::{IndexedMesh, MeshTopology, Point3, Vector3};

use crate::config::PrinterConfig;
use crate::error::{PrintabilityError, PrintabilityResult};
use crate::issues::{IssueSeverity, PrintIssue, PrintIssueType};
use crate::regions::{OverhangRegion, SupportRegion, ThinWallRegion};

/// Result of validating a mesh for printing.
///
/// Contains all issues found, their severity, and recommendations
/// for improving printability.
#[derive(Debug, Clone)]
pub struct PrintValidation {
    /// Printer configuration used for validation.
    pub config: PrinterConfig,

    /// List of issues found.
    pub issues: Vec<PrintIssue>,

    /// Thin wall regions detected.
    pub thin_walls: Vec<ThinWallRegion>,

    /// Overhang regions detected.
    pub overhangs: Vec<OverhangRegion>,

    /// Regions needing support structures.
    pub support_regions: Vec<SupportRegion>,

    /// Estimated print time in minutes (if available).
    pub estimated_print_time: Option<f64>,

    /// Estimated material usage in cubic mm.
    pub estimated_material_volume: Option<f64>,
}

impl PrintValidation {
    /// Create a new validation result.
    #[must_use]
    pub fn new(config: PrinterConfig) -> Self {
        Self {
            config,
            issues: Vec::new(),
            thin_walls: Vec::new(),
            overhangs: Vec::new(),
            support_regions: Vec::new(),
            estimated_print_time: None,
            estimated_material_volume: None,
        }
    }

    /// Check if the mesh is printable (no critical issues).
    #[must_use]
    pub fn is_printable(&self) -> bool {
        !self.issues.iter().any(PrintIssue::is_critical)
    }

    /// Get the number of critical issues.
    #[must_use]
    pub fn critical_count(&self) -> usize {
        self.issues.iter().filter(|i| i.is_critical()).count()
    }

    /// Get the number of warnings.
    #[must_use]
    pub fn warning_count(&self) -> usize {
        self.issues.iter().filter(|i| i.is_warning()).count()
    }

    /// Get total estimated support volume in cubic mm.
    #[must_use]
    pub fn total_support_volume(&self) -> f64 {
        self.support_regions.iter().map(|r| r.volume).sum()
    }

    /// Get a summary of the validation result.
    #[must_use]
    pub fn summary(&self) -> String {
        if self.issues.is_empty() {
            return "Mesh is ready for printing".to_string();
        }

        let critical = self.critical_count();
        let warnings = self.warning_count();
        let info = self.issues.len() - critical - warnings;

        let mut parts = Vec::new();
        if critical > 0 {
            parts.push(format!("{critical} critical issue(s)"));
        }
        if warnings > 0 {
            parts.push(format!("{warnings} warning(s)"));
        }
        if info > 0 {
            parts.push(format!("{info} info"));
        }

        if critical > 0 {
            format!("Not printable: {}", parts.join(", "))
        } else {
            format!("Printable with issues: {}", parts.join(", "))
        }
    }
}

/// Validate a mesh for printability.
///
/// Checks the mesh against printer constraints and returns a detailed
/// validation result with all issues found.
///
/// # Errors
///
/// Returns an error if the mesh is empty or has no faces.
///
/// # Example
///
/// ```
/// use mesh_types::IndexedMesh;
/// use mesh_printability::{validate_for_printing, PrinterConfig};
///
/// let mesh = IndexedMesh::new();
/// // Note: This would fail since the mesh is empty
/// // let result = validate_for_printing(&mesh, &PrinterConfig::fdm_default());
/// ```
pub fn validate_for_printing(
    mesh: &IndexedMesh,
    config: &PrinterConfig,
) -> PrintabilityResult<PrintValidation> {
    if mesh.vertices.is_empty() {
        return Err(PrintabilityError::EmptyMesh);
    }

    if mesh.faces.is_empty() {
        return Err(PrintabilityError::NoFaces);
    }

    let mut validation = PrintValidation::new(config.clone());

    // Check build volume
    check_build_volume(mesh, config, &mut validation);

    // Check overhangs
    check_overhangs(mesh, config, &mut validation);

    // Check manifold (basic check)
    check_basic_manifold(mesh, &mut validation);

    Ok(validation)
}

/// Check if mesh fits within the build volume.
fn check_build_volume(
    mesh: &IndexedMesh,
    config: &PrinterConfig,
    validation: &mut PrintValidation,
) {
    let (min, max) = compute_bounds(mesh);

    let size_x = max.x - min.x;
    let size_y = max.y - min.y;
    let size_z = max.z - min.z;

    let (build_x, build_y, build_z) = config.build_volume;

    if size_x > build_x || size_y > build_y || size_z > build_z {
        let issue = PrintIssue::new(
            PrintIssueType::ExceedsBuildVolume,
            IssueSeverity::Critical,
            format!(
                "Mesh dimensions ({:.1} x {:.1} x {:.1} mm) exceed build volume ({:.1} x {:.1} x {:.1} mm)",
                size_x, size_y, size_z, build_x, build_y, build_z
            ),
        );
        validation.issues.push(issue);
    }

    // Estimate material volume (bounding box approximation)
    validation.estimated_material_volume = Some(size_x * size_y * size_z * 0.3); // Rough 30% fill estimate
}

/// Check for excessive overhangs.
fn check_overhangs(mesh: &IndexedMesh, config: &PrinterConfig, validation: &mut PrintValidation) {
    // Skip overhang check for technologies that don't need supports
    if !config.technology.requires_supports() {
        return;
    }

    let max_angle_rad = config.max_overhang_angle.to_radians();
    let up = Vector3::new(0.0, 0.0, 1.0);

    let mut overhang_faces = Vec::new();
    let mut total_overhang_area = 0.0;

    let num_triangles = mesh.face_count();
    for i in 0..num_triangles {
        let face = mesh.faces[i];
        let idx0 = face[0] as usize;
        let idx1 = face[1] as usize;
        let idx2 = face[2] as usize;

        if idx0 >= mesh.vertices.len() || idx1 >= mesh.vertices.len() || idx2 >= mesh.vertices.len()
        {
            continue;
        }

        let v0 = mesh.vertices[idx0].position;
        let v1 = mesh.vertices[idx1].position;
        let v2 = mesh.vertices[idx2].position;

        // Compute face normal
        let edge1 = Vector3::new(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
        let edge2 = Vector3::new(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
        let normal = edge1.cross(&edge2);

        let len = (normal.x * normal.x + normal.y * normal.y + normal.z * normal.z).sqrt();
        if len < 1e-10 {
            continue;
        }

        let normal = Vector3::new(normal.x / len, normal.y / len, normal.z / len);

        // Check if face is pointing downward (overhang)
        let dot = normal.x * up.x + normal.y * up.y + normal.z * up.z;
        let angle = dot.acos();

        // Face is overhanging if normal points mostly downward
        // and angle from vertical exceeds threshold
        if dot < 0.0 {
            let overhang_angle = std::f64::consts::PI - angle;
            if overhang_angle > max_angle_rad {
                #[allow(clippy::cast_possible_truncation)]
                overhang_faces.push(i as u32);

                // Compute triangle area
                let area = len / 2.0;
                total_overhang_area += area;
            }
        }
    }

    if !overhang_faces.is_empty() {
        let center = compute_face_centroid(mesh, overhang_faces[0] as usize);
        let overhang_angle_deg = config.max_overhang_angle + 10.0; // Approximate

        let region = OverhangRegion::new(center, overhang_angle_deg, total_overhang_area)
            .with_faces(overhang_faces.clone());
        validation.overhangs.push(region);

        // Create support region estimate
        let support_volume = total_overhang_area * 5.0; // Rough estimate
        let support_region = SupportRegion::new(center, support_volume, 10.0) // Rough height
            .with_faces(overhang_faces.clone());
        validation.support_regions.push(support_region);

        let severity = if total_overhang_area > 1000.0 {
            IssueSeverity::Warning
        } else {
            IssueSeverity::Info
        };

        let issue = PrintIssue::new(
            PrintIssueType::ExcessiveOverhang,
            severity,
            format!(
                "{} faces with overhangs exceeding {:.1}° (total area: {:.1} mm²)",
                overhang_faces.len(),
                config.max_overhang_angle,
                total_overhang_area
            ),
        )
        .with_location(center)
        .with_affected_elements(overhang_faces);

        validation.issues.push(issue);
    }
}

/// Basic manifold check (edge usage).
fn check_basic_manifold(mesh: &IndexedMesh, validation: &mut PrintValidation) {
    use hashbrown::HashMap;

    // Count edge usage
    let mut edge_count: HashMap<(u32, u32), u32> = HashMap::new();

    for face in &mesh.faces {
        let idx0 = face[0];
        let idx1 = face[1];
        let idx2 = face[2];

        // Add edges (always store with smaller index first for consistency)
        let edges = [
            (idx0.min(idx1), idx0.max(idx1)),
            (idx1.min(idx2), idx1.max(idx2)),
            (idx2.min(idx0), idx2.max(idx0)),
        ];

        for edge in &edges {
            *edge_count.entry(*edge).or_insert(0) += 1;
        }
    }

    // In a manifold mesh, each edge should be shared by exactly 2 faces
    let boundary_edges: Vec<_> = edge_count
        .iter()
        .filter(|(_, count)| **count != 2)
        .collect();

    if !boundary_edges.is_empty() {
        let non_manifold_count = boundary_edges.iter().filter(|(_, c)| **c > 2).count();
        let open_edge_count = boundary_edges.iter().filter(|(_, c)| **c == 1).count();

        if non_manifold_count > 0 {
            let issue = PrintIssue::new(
                PrintIssueType::NonManifold,
                IssueSeverity::Critical,
                format!("{non_manifold_count} non-manifold edge(s) detected"),
            );
            validation.issues.push(issue);
        }

        if open_edge_count > 0 {
            let issue = PrintIssue::new(
                PrintIssueType::NotWatertight,
                IssueSeverity::Critical,
                format!("{open_edge_count} open edge(s) detected (mesh not watertight)"),
            );
            validation.issues.push(issue);
        }
    }
}

/// Compute the bounding box of a mesh.
fn compute_bounds(mesh: &IndexedMesh) -> (Point3<f64>, Point3<f64>) {
    let mut min = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);

    for v in &mesh.vertices {
        min.x = min.x.min(v.position.x);
        min.y = min.y.min(v.position.y);
        min.z = min.z.min(v.position.z);
        max.x = max.x.max(v.position.x);
        max.y = max.y.max(v.position.y);
        max.z = max.z.max(v.position.z);
    }

    (min, max)
}

/// Compute the centroid of a face.
fn compute_face_centroid(mesh: &IndexedMesh, face_idx: usize) -> Point3<f64> {
    let face = mesh.faces[face_idx];
    let idx0 = face[0] as usize;
    let idx1 = face[1] as usize;
    let idx2 = face[2] as usize;

    let v0 = &mesh.vertices[idx0].position;
    let v1 = &mesh.vertices[idx1].position;
    let v2 = &mesh.vertices[idx2].position;

    Point3::new(
        (v0.x + v1.x + v2.x) / 3.0,
        (v0.y + v1.y + v2.y) / 3.0,
        (v0.z + v1.z + v2.z) / 3.0,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    fn create_cube_mesh() -> IndexedMesh {
        // Simple cube mesh (not watertight for testing)
        let vertices = vec![
            Vertex::from_coords(0.0, 0.0, 0.0),
            Vertex::from_coords(10.0, 0.0, 0.0),
            Vertex::from_coords(10.0, 10.0, 0.0),
            Vertex::from_coords(0.0, 10.0, 0.0),
            Vertex::from_coords(0.0, 0.0, 10.0),
            Vertex::from_coords(10.0, 0.0, 10.0),
            Vertex::from_coords(10.0, 10.0, 10.0),
            Vertex::from_coords(0.0, 10.0, 10.0),
        ];

        // Bottom and top faces only (not complete cube)
        let faces = vec![
            [0, 1, 2],
            [0, 2, 3], // Bottom
            [4, 6, 5],
            [4, 7, 6], // Top
        ];

        IndexedMesh::from_parts(vertices, faces)
    }

    fn create_watertight_cube() -> IndexedMesh {
        let vertices = vec![
            Vertex::from_coords(0.0, 0.0, 0.0),
            Vertex::from_coords(10.0, 0.0, 0.0),
            Vertex::from_coords(10.0, 10.0, 0.0),
            Vertex::from_coords(0.0, 10.0, 0.0),
            Vertex::from_coords(0.0, 0.0, 10.0),
            Vertex::from_coords(10.0, 0.0, 10.0),
            Vertex::from_coords(10.0, 10.0, 10.0),
            Vertex::from_coords(0.0, 10.0, 10.0),
        ];

        // All 6 faces (12 triangles)
        let faces = vec![
            // Bottom (Z=0)
            [0, 2, 1],
            [0, 3, 2],
            // Top (Z=10)
            [4, 5, 6],
            [4, 6, 7],
            // Front (Y=0)
            [0, 1, 5],
            [0, 5, 4],
            // Back (Y=10)
            [3, 6, 2],
            [3, 7, 6],
            // Left (X=0)
            [0, 4, 7],
            [0, 7, 3],
            // Right (X=10)
            [1, 2, 6],
            [1, 6, 5],
        ];

        IndexedMesh::from_parts(vertices, faces)
    }

    #[test]
    fn test_empty_mesh_error() {
        let mesh = IndexedMesh::new();
        let config = PrinterConfig::fdm_default();
        let result = validate_for_printing(&mesh, &config);
        assert!(matches!(result, Err(PrintabilityError::EmptyMesh)));
    }

    #[test]
    fn test_no_faces_error() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        let config = PrinterConfig::fdm_default();
        let result = validate_for_printing(&mesh, &config);
        assert!(matches!(result, Err(PrintabilityError::NoFaces)));
    }

    #[test]
    fn test_build_volume_check() {
        let mesh = create_cube_mesh();
        let config = PrinterConfig::fdm_default().with_build_volume(5.0, 5.0, 5.0);

        let result = validate_for_printing(&mesh, &config).expect("Should succeed");
        assert!(
            result
                .issues
                .iter()
                .any(|i| i.issue_type == PrintIssueType::ExceedsBuildVolume)
        );
        assert!(!result.is_printable());
    }

    #[test]
    fn test_build_volume_ok() {
        let mesh = create_watertight_cube();
        let config = PrinterConfig::fdm_default().with_build_volume(100.0, 100.0, 100.0);

        let result = validate_for_printing(&mesh, &config).expect("Should succeed");
        assert!(
            !result
                .issues
                .iter()
                .any(|i| i.issue_type == PrintIssueType::ExceedsBuildVolume)
        );
    }

    #[test]
    fn test_not_watertight_detection() {
        let mesh = create_cube_mesh(); // Incomplete cube
        let config = PrinterConfig::fdm_default();

        let result = validate_for_printing(&mesh, &config).expect("Should succeed");
        assert!(
            result
                .issues
                .iter()
                .any(|i| i.issue_type == PrintIssueType::NotWatertight)
        );
    }

    #[test]
    fn test_watertight_mesh() {
        let mesh = create_watertight_cube();
        let config = PrinterConfig::fdm_default();

        let result = validate_for_printing(&mesh, &config).expect("Should succeed");
        assert!(
            !result
                .issues
                .iter()
                .any(|i| i.issue_type == PrintIssueType::NotWatertight)
        );
    }

    #[test]
    fn test_validation_summary() {
        let config = PrinterConfig::fdm_default();
        let mut validation = PrintValidation::new(config);

        assert_eq!(validation.summary(), "Mesh is ready for printing");

        validation.issues.push(PrintIssue::new(
            PrintIssueType::ThinWall,
            IssueSeverity::Warning,
            "test",
        ));

        assert!(validation.summary().contains("Printable with issues"));

        validation.issues.push(PrintIssue::new(
            PrintIssueType::NotWatertight,
            IssueSeverity::Critical,
            "test",
        ));

        assert!(validation.summary().contains("Not printable"));
    }

    #[test]
    fn test_issue_counts() {
        let config = PrinterConfig::fdm_default();
        let mut validation = PrintValidation::new(config);

        validation.issues.push(PrintIssue::new(
            PrintIssueType::ThinWall,
            IssueSeverity::Warning,
            "",
        ));
        validation.issues.push(PrintIssue::new(
            PrintIssueType::ThinWall,
            IssueSeverity::Warning,
            "",
        ));
        validation.issues.push(PrintIssue::new(
            PrintIssueType::NotWatertight,
            IssueSeverity::Critical,
            "",
        ));
        validation.issues.push(PrintIssue::new(
            PrintIssueType::Other,
            IssueSeverity::Info,
            "",
        ));

        assert_eq!(validation.critical_count(), 1);
        assert_eq!(validation.warning_count(), 2);
        assert!(!validation.is_printable());
    }

    #[test]
    fn test_sls_no_overhang_check() {
        let mesh = create_watertight_cube();
        let config = PrinterConfig::sls_default();

        let result = validate_for_printing(&mesh, &config).expect("Should succeed");
        // SLS doesn't check overhangs
        assert!(
            !result
                .issues
                .iter()
                .any(|i| i.issue_type == PrintIssueType::ExcessiveOverhang)
        );
    }

    #[test]
    fn test_support_volume_calculation() {
        let config = PrinterConfig::fdm_default();
        let mut validation = PrintValidation::new(config);

        validation.support_regions.push(SupportRegion::new(
            Point3::new(0.0, 0.0, 0.0),
            100.0,
            10.0,
        ));
        validation.support_regions.push(SupportRegion::new(
            Point3::new(0.0, 0.0, 0.0),
            200.0,
            20.0,
        ));

        assert!((validation.total_support_volume() - 300.0).abs() < f64::EPSILON);
    }
}
