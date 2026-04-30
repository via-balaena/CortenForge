//! Main validation logic for printability analysis.
//!
//! Provides the primary validation functionality to check meshes
//! against printer constraints.

use mesh_types::{IndexedMesh, Point3, Vector3};

use crate::config::PrinterConfig;
use crate::error::{PrintabilityError, PrintabilityResult};
use crate::issues::{IssueSeverity, PrintIssue, PrintIssueType};
use crate::regions::{OverhangRegion, SupportRegion, ThinWallRegion};

/// General geometric tie-breaking tolerance in millimetres (per spec §4.2).
/// Used by the build-plate filter in `check_overhangs` to identify faces
/// resting on the build plate.
const EPS_GEOMETRIC: f64 = 1e-9;

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
    pub const fn new(config: PrinterConfig) -> Self {
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
/// use mesh_types::{IndexedMesh, Point3};
/// use mesh_printability::{validate_for_printing, PrinterConfig};
///
/// let mesh = IndexedMesh::from_parts(
///     vec![
///         Point3::new(0.0, 0.0, 0.0),
///         Point3::new(10.0, 0.0, 0.0),
///         Point3::new(5.0, 10.0, 0.0),
///     ],
///     vec![[0, 1, 2]],
/// );
///
/// let result = validate_for_printing(&mesh, &PrinterConfig::fdm_default()).unwrap();
/// assert!(result.issues.len() > 0); // Single triangle isn't watertight
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
                "Mesh dimensions ({size_x:.1} x {size_y:.1} x {size_z:.1} mm) exceed build volume ({build_x:.1} x {build_y:.1} x {build_z:.1} mm)"
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

    // Build-plate filter (M.2): a face is "on the build plate" iff its
    // minimum projection along `up` is within EPS_GEOMETRIC of the mesh
    // minimum. Hoisted before the face loop so the O(n_vertices) reduction
    // runs once, not per-face. Pattern shared with §6.2 LongBridge.
    let mesh_min_along_up = mesh
        .vertices
        .iter()
        .map(|v| v.coords.dot(&up))
        .fold(f64::INFINITY, f64::min);

    let mut overhang_faces = Vec::new();
    let mut total_overhang_area = 0.0;
    // §5.2 Gap B: track the actual maximum `overhang_angle` (in radians)
    // observed across flagged faces; replaces v0.7's hardcoded
    // `config.max_overhang_angle + 10.0` placeholder. Read at region
    // creation below; the `overhang_faces.is_empty()` guard prevents the
    // zero-init from leaking into a region.
    let mut max_overhang_angle_rad: f64 = 0.0;

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

        let v0 = mesh.vertices[idx0];
        let v1 = mesh.vertices[idx1];
        let v2 = mesh.vertices[idx2];

        // Compute face normal
        let edge1 = Vector3::new(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
        let edge2 = Vector3::new(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
        let normal = edge1.cross(&edge2);

        // FP-bit preserved: rewriting `(x*x + y*y + z*z)` into nested
        // `mul_add` calls would shift the normalized face-normal cross-
        // platform and thereby shift which faces flag at the overhang
        // threshold. Bit-exactness deferred — see CHANGELOG.md
        // `[Unreleased] / v0.9 candidates`.
        #[allow(clippy::suboptimal_flops)]
        let len = (normal.x * normal.x + normal.y * normal.y + normal.z * normal.z).sqrt();
        if len < 1e-10 {
            continue;
        }

        let normal = Vector3::new(normal.x / len, normal.y / len, normal.z / len);

        // FP-bit preserved: rewriting `(a*b + c*d + e*f)` into `mul_add`
        // chains would shift the final FP bit of the overhang predicate's
        // dot product, changing which faces flag at the threshold boundary
        // cross-platform. Bit-exactness deferred — see CHANGELOG.md
        // `[Unreleased] / v0.9 candidates`.
        #[allow(clippy::suboptimal_flops)]
        let dot = normal.x * up.x + normal.y * up.y + normal.z * up.z;

        // §5.9 FDM-convention overhang predicate: `overhang_angle` is the
        // signed deviation of the face normal from the build-plane plane.
        // Flags downward-tilted faces; vertical walls (dot=0) sit at the
        // strict-greater-than boundary and are not flagged.
        let angle_from_up = dot.acos();
        let overhang_angle = angle_from_up - std::f64::consts::FRAC_PI_2;

        if overhang_angle > max_angle_rad {
            // Build-plate filter (M.2): a face touching the build plate is
            // supported by the plate itself; not an overhang concern.
            let face_min_along_up = [v0, v1, v2]
                .iter()
                .map(|v| v.coords.dot(&up))
                .fold(f64::INFINITY, f64::min);
            if (face_min_along_up - mesh_min_along_up) < EPS_GEOMETRIC {
                continue;
            }

            // Mesh face index i fits in u32 (mesh size bounded well below 2^32).
            #[allow(clippy::cast_possible_truncation)]
            overhang_faces.push(i as u32);
            max_overhang_angle_rad = max_overhang_angle_rad.max(overhang_angle);

            // Compute triangle area
            let area = len / 2.0;
            total_overhang_area += area;
        }
    }

    if !overhang_faces.is_empty() {
        let center = compute_face_centroid(mesh, overhang_faces[0] as usize);
        let overhang_angle_deg = max_overhang_angle_rad.to_degrees();

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
        min.x = min.x.min(v.x);
        min.y = min.y.min(v.y);
        min.z = min.z.min(v.z);
        max.x = max.x.max(v.x);
        max.y = max.y.max(v.y);
        max.z = max.z.max(v.z);
    }

    (min, max)
}

/// Compute the centroid of a face.
fn compute_face_centroid(mesh: &IndexedMesh, face_idx: usize) -> Point3<f64> {
    let face = mesh.faces[face_idx];
    let idx0 = face[0] as usize;
    let idx1 = face[1] as usize;
    let idx2 = face[2] as usize;

    let v0 = &mesh.vertices[idx0];
    let v1 = &mesh.vertices[idx1];
    let v2 = &mesh.vertices[idx2];

    Point3::new(
        (v0.x + v1.x + v2.x) / 3.0,
        (v0.y + v1.y + v2.y) / 3.0,
        (v0.z + v1.z + v2.z) / 3.0,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Point3;

    fn create_cube_mesh() -> IndexedMesh {
        // Simple cube mesh (not watertight for testing)
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(10.0, 10.0, 0.0),
            Point3::new(0.0, 10.0, 0.0),
            Point3::new(0.0, 0.0, 10.0),
            Point3::new(10.0, 0.0, 10.0),
            Point3::new(10.0, 10.0, 10.0),
            Point3::new(0.0, 10.0, 10.0),
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
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(10.0, 10.0, 0.0),
            Point3::new(0.0, 10.0, 0.0),
            Point3::new(0.0, 0.0, 10.0),
            Point3::new(10.0, 0.0, 10.0),
            Point3::new(10.0, 10.0, 10.0),
            Point3::new(0.0, 10.0, 10.0),
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
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        let config = PrinterConfig::fdm_default();
        let result = validate_for_printing(&mesh, &config);
        assert!(matches!(result, Err(PrintabilityError::NoFaces)));
    }

    // The five `validate_for_printing(...).expect("Should succeed")` calls in the
    // tests below operate on fixtures constructed inline (no I/O, no fallible
    // setup): an `expect` failure here would indicate a regression in the
    // detector itself, not a malformed fixture. Each site is annotated
    // individually so the lint stays active for any future statements added
    // inside these test bodies.
    #[test]
    fn test_build_volume_check() {
        let mesh = create_cube_mesh();
        let config = PrinterConfig::fdm_default().with_build_volume(5.0, 5.0, 5.0);

        #[allow(clippy::expect_used)]
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

        #[allow(clippy::expect_used)]
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

        #[allow(clippy::expect_used)]
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

        #[allow(clippy::expect_used)]
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

        #[allow(clippy::expect_used)]
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

    // ---- §5.9 Gap M overhang predicate tests ----------------------------
    //
    // Each `validate_for_printing(...).expect(...)` site below carries an
    // explicit per-site `#[allow(clippy::expect_used)]`. The fixtures are
    // hand-built inline (no I/O, no fallible setup): an `expect` failure
    // would indicate a regression in the detector itself, not a malformed
    // fixture. The shared `panic!`-via-`let-else` pattern was avoided
    // because the workspace lints `clippy::panic` at warn (which becomes
    // an error under the `-D warnings` clippy gate), and the existing
    // `validation.rs::tests` already uses the same per-site `expect_used`
    // pattern as of commit #1b.
    //
    // TODO(commit #8 / Gap L §5.6): land
    // `test_overhang_borderline_via_y_up_orientation` here. It exercises
    // `PrinterConfig::with_build_up_direction(Vector3::new(0, 1, 0))`,
    // which lands with Gap L (commit #8). The test slot is reserved at
    // §5.9 of the v0.8 fix arc spec; the body lands once the API exists.
    // Keeping the 9 §5.9 tests here in commit #2 is achieved by splitting
    // the FP-fragile `test_overhang_45deg_tilt_borderline_not_flagged`
    // into a `just_below_45deg_not_flagged` / `just_above_45deg_flagged`
    // pair (1° margin on either side keeps both bit-stable across f64
    // rounding of the strict-greater-than boundary).

    /// Build a fixture for Gap-M unit tests: a top-facing ground-anchor
    /// triangle at z=0 and a test face at z=`z_offset` tilted by
    /// `beta_rad` from horizontal in the `+Y` / `-Z` plane. The test face
    /// has normal `(0, cos(beta), -sin(beta))`, so it points
    /// downward when `beta` > 0 (with `overhang_angle = beta`), is
    /// horizontal at `beta` = 0 (vertical wall, `dot` = 0), and points
    /// upward when `beta` < 0. The anchor at z=0 keeps
    /// `mesh_min_along_up` < `face_min_along_up` so the build-plate
    /// filter does not block the predicate's flag/not-flag decision for
    /// any `beta` in (-π/2, π/2).
    fn make_overhang_fixture(beta_rad: f64, z_offset: f64) -> IndexedMesh {
        let vertices = vec![
            // Ground anchor at z=0 (top-facing, normal = (0, 0, +1)):
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            // Test face at z_offset, tilted by beta in the Y-Z plane:
            Point3::new(0.0, 0.0, z_offset),
            Point3::new(1.0, 0.0, z_offset),
            Point3::new(0.0, beta_rad.sin(), z_offset + beta_rad.cos()),
        ];
        let faces = vec![[0, 1, 2], [3, 5, 4]];
        IndexedMesh::from_parts(vertices, faces)
    }

    #[test]
    fn test_overhang_roof_flagged() {
        // Pure roof: face normal = (0, 0, -1), overhang_angle = 90°.
        // Anchor at z=0 ensures the build-plate filter does not apply.
        let mesh = make_overhang_fixture(std::f64::consts::FRAC_PI_2, 5.0);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the roof fixture");

        assert_eq!(
            result.overhangs.len(),
            1,
            "pure roof (overhang_angle = 90°) should flag under FDM max=45°"
        );
    }

    #[test]
    fn test_overhang_vertical_wall_not_flagged() {
        // Vertical wall: face normal horizontal, dot = 0,
        // overhang_angle = 0°. Strict-greater-than check rejects.
        let mesh = make_overhang_fixture(0.0, 5.0);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the vertical-wall fixture");

        assert_eq!(
            result.overhangs.len(),
            0,
            "vertical wall (overhang_angle = 0°) must not flag under any positive threshold"
        );
    }

    #[test]
    fn test_overhang_top_face_not_flagged() {
        // Top-facing face: normal = (0, 0, +1), dot = +1,
        // overhang_angle = -90°. Predicate rejects.
        let mesh = make_overhang_fixture(-std::f64::consts::FRAC_PI_2, 5.0);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the top-face fixture");

        assert_eq!(
            result.overhangs.len(),
            0,
            "top-facing face (overhang_angle = -90°) must not flag"
        );
    }

    // The §5.9 boundary-case test was authored as a single test at exactly
    // 45°, but the f64 subtraction `acos(dot) - FRAC_PI_2` carries ~2 ULP
    // of upward rounding at exactly 45° tilt: a hand-rolled exact-45°
    // fixture would actually flag, contradicting the test's intent. The
    // pair below brackets the threshold at 44° / 46° (1° margin = ~17 orders
    // of magnitude above f64 ULP), so both halves are bit-stable across
    // platforms and bound the implementation's flag boundary in (44°, 46°).
    // The strict-greater-than convention itself is locked in by the
    // predicate's `>` operator and called out in the predicate's
    // doc-comment in `check_overhangs`.

    #[test]
    fn test_overhang_just_below_45deg_not_flagged() {
        // Tilt 44° (just below the FDM max=45° threshold).
        let mesh = make_overhang_fixture(44.0_f64.to_radians(), 5.0);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the 44° fixture");

        assert_eq!(
            result.overhangs.len(),
            0,
            "44° tilt (just below max=45°) must not flag"
        );
    }

    #[test]
    fn test_overhang_just_above_45deg_flagged() {
        // Tilt 46° (just above the FDM max=45° threshold).
        let mesh = make_overhang_fixture(46.0_f64.to_radians(), 5.0);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the 46° fixture");

        assert_eq!(
            result.overhangs.len(),
            1,
            "46° tilt (just above max=45°) should flag"
        );
    }

    #[test]
    fn test_overhang_60deg_tilt_flagged() {
        // 60° tilt: well above 45° threshold, robust to FP drift.
        let mesh = make_overhang_fixture(60.0_f64.to_radians(), 5.0);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the 60° fixture");

        assert_eq!(result.overhangs.len(), 1, "60° tilt should flag");
    }

    #[test]
    fn test_overhang_30deg_tilt_not_flagged() {
        // 30° tilt: well below 45° threshold, robust to FP drift.
        let mesh = make_overhang_fixture(30.0_f64.to_radians(), 5.0);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the 30° fixture");

        assert_eq!(result.overhangs.len(), 0, "30° tilt must not flag");
    }

    #[test]
    fn test_overhang_build_plate_face_not_flagged() {
        // Watertight cube on the build plate (z ∈ [0, 10]). The cube's
        // bottom face has overhang_angle = 90° but face_min == mesh_min
        // → build-plate filter applies → not flagged.
        let mesh = create_watertight_cube();
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the cube-on-plate fixture");

        assert_eq!(
            result.overhangs.len(),
            0,
            "cube-on-plate bottom must be filtered by the build-plate check"
        );
    }

    #[test]
    fn test_overhang_suspended_roof_flagged() {
        // Locks in mesh-min-relativity of the build-plate filter via a
        // contrast pair: the SAME roof face flags or does not flag based
        // purely on whether a separate ground anchor is present at z=0.
        let config = PrinterConfig::fdm_default();

        // Variant A: roof alone at z=20, no anchor. mesh_min == face_min
        // == 20 → build-plate filter applies → not flagged.
        let mesh_no_anchor = IndexedMesh::from_parts(
            vec![
                Point3::new(0.0, 0.0, 20.0),
                Point3::new(0.0, 1.0, 20.0),
                Point3::new(1.0, 0.0, 20.0),
            ],
            vec![[0, 1, 2]],
        );
        #[allow(clippy::expect_used)]
        let no_anchor = validate_for_printing(&mesh_no_anchor, &config)
            .expect("validation should succeed for the no-anchor variant");
        assert_eq!(
            no_anchor.overhangs.len(),
            0,
            "roof alone (mesh_min == face_min) is filtered as build-plate"
        );

        // Variant B: same roof + small ground anchor at z=0. mesh_min = 0,
        // face_min = 20. (20 - 0) ≫ EPS_GEOMETRIC → not filtered → flagged.
        let mesh_anchored = IndexedMesh::from_parts(
            vec![
                Point3::new(-1.0, -1.0, 0.0),
                Point3::new(-2.0, -1.0, 0.0),
                Point3::new(-1.5, -2.0, 0.0),
                Point3::new(0.0, 0.0, 20.0),
                Point3::new(0.0, 1.0, 20.0),
                Point3::new(1.0, 0.0, 20.0),
            ],
            vec![[0, 1, 2], [3, 4, 5]],
        );
        #[allow(clippy::expect_used)]
        let anchored = validate_for_printing(&mesh_anchored, &config)
            .expect("validation should succeed for the anchored variant");
        assert_eq!(
            anchored.overhangs.len(),
            1,
            "anchored roof (mesh_min = 0, face_min = 20) flags — filter is mesh-min-relative, not face-z-absolute"
        );
    }

    // ---- §5.2 Gap B max-angle tracking tests ---------------------------
    //
    // Lock in the post-§5.2 semantic: `OverhangRegion.angle` is the actual
    // maximum of `overhang_angle` (in degrees) across the flagged faces.
    // Pre-Gap-B v0.7 reported `config.max_overhang_angle + 10.0` (a
    // geometry-independent constant); §5.2 replaces this with
    // `max_overhang_angle_rad.to_degrees()`. Composes with Gap M (§5.9):
    // post-v0.8 `overhang_angle ∈ [0°, 90°]` for downward-facing flagged
    // faces. Tolerance epsilon = 1e-6 per §4.5 / §5.2 acceptance.

    #[test]
    fn test_overhang_angle_tracks_max() {
        // Single-face fixture at β = 60° tilt: per the
        // `make_overhang_fixture` contract, the test face has
        // `overhang_angle = β` (face normal = (0, cos β, -sin β),
        // dot = -sin β, overhang_angle = acos(-sin β) - π/2 = β).
        let mesh = make_overhang_fixture(60.0_f64.to_radians(), 5.0);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the 60° fixture");

        assert_eq!(result.overhangs.len(), 1, "60° tilt must flag");
        approx::assert_relative_eq!(result.overhangs[0].angle, 60.0, epsilon = 1e-6);
    }

    #[test]
    fn test_overhang_angle_uses_steepest_face() {
        // Two test faces at β = 50° and β = 70°, plus a ground anchor at
        // z = 0. Both flag under FDM max = 45°; the reported angle is the
        // max (~70°). Critical lock-in for the "max-of" semantic: pre-Gap-B
        // v0.7 would report 55° here regardless of geometry.
        let beta1 = 50.0_f64.to_radians();
        let beta2 = 70.0_f64.to_radians();
        let z_offset = 5.0_f64;
        let vertices = vec![
            // Ground anchor at z = 0 (top-facing, not flagged):
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(0.0, 10.0, 0.0),
            // Test face 1 at β = 50°, x = 0:
            Point3::new(0.0, 0.0, z_offset),
            Point3::new(1.0, 0.0, z_offset),
            Point3::new(0.0, beta1.sin(), z_offset + beta1.cos()),
            // Test face 2 at β = 70°, x = 5:
            Point3::new(5.0, 0.0, z_offset),
            Point3::new(6.0, 0.0, z_offset),
            Point3::new(5.0, beta2.sin(), z_offset + beta2.cos()),
        ];
        let faces = vec![[0, 1, 2], [3, 5, 4], [6, 8, 7]];
        let mesh = IndexedMesh::from_parts(vertices, faces);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the two-face fixture");

        // Pre-Gap-D: all flagged faces lump into a single OverhangRegion.
        // The reported angle is the max observed across flagged faces.
        assert_eq!(
            result.overhangs.len(),
            1,
            "two flagged faces lump into one region pre-Gap-D"
        );
        approx::assert_relative_eq!(result.overhangs[0].angle, 70.0, epsilon = 1e-6);
    }

    #[test]
    fn test_overhang_no_overhang_no_region() {
        // Vertical-wall fixture (β = 0): overhang_angle = 0, no faces
        // flag. The `overhang_faces.is_empty()` guard prevents the
        // zero-init `max_overhang_angle_rad` from leaking out as a region
        // with `angle = 0` (Gap B risk row 3 lock-in, §8.1).
        let mesh = make_overhang_fixture(0.0, 5.0);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the vertical-wall fixture");

        assert!(
            result.overhangs.is_empty(),
            "no flagged faces → no region (Gap B empty-case guard)"
        );
    }
}
