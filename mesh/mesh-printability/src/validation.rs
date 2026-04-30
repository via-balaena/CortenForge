//! Main validation logic for printability analysis.
//!
//! Provides the primary validation functionality to check meshes
//! against printer constraints.

use hashbrown::{HashMap, HashSet};
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

/// Per-face metadata captured during the overhang flag-collection loop.
///
/// Used post-loop for per-component aggregation in the §5.3 Gap D
/// connected-region partition: each component's `OverhangRegion`
/// reports the max `overhang_angle_rad` (converted to degrees) and the
/// summed `area` over its member faces.
#[derive(Clone, Copy)]
struct FlaggedFaceMeta {
    overhang_angle_rad: f64,
    area: f64,
}

/// Check for excessive overhangs.
fn check_overhangs(mesh: &IndexedMesh, config: &PrinterConfig, validation: &mut PrintValidation) {
    if !config.technology.requires_supports() {
        return;
    }

    let max_angle_rad = config.max_overhang_angle.to_radians();
    let up = Vector3::new(0.0, 0.0, 1.0);

    let flagged = flag_overhang_faces(mesh, max_angle_rad, up);
    if flagged.is_empty() {
        return;
    }

    let components = partition_flagged_into_components(mesh, &flagged);

    // Per component: one OverhangRegion + one SupportRegion + one
    // ExcessiveOverhang issue. The 1:1
    // `support_regions.len() == overhangs.len()` invariant is preserved
    // at component granularity (was: 1:1 globally pre-Gap-D). The
    // ExcessiveOverhang issue per region is what Gap E (commit #6) will
    // re-classify by per-region max angle.
    for component in components {
        emit_overhang_component(mesh, config, &flagged, component, validation);
    }
}

/// Walk every face and collect those whose normal exceeds
/// `config.max_overhang_angle` from the build-up direction (FDM
/// convention; §5.9 Gap M predicate). The build-plate filter (M.2)
/// rejects faces resting on the mesh-min plane. Returns per-face
/// `overhang_angle` + `area` keyed by face index for downstream
/// connected-region partition (§5.3 Gap D).
fn flag_overhang_faces(
    mesh: &IndexedMesh,
    max_angle_rad: f64,
    up: Vector3<f64>,
) -> HashMap<u32, FlaggedFaceMeta> {
    // Build-plate filter (M.2): a face is "on the build plate" iff its
    // minimum projection along `up` is within EPS_GEOMETRIC of the mesh
    // minimum. Hoisted before the face loop so the O(n_vertices) reduction
    // runs once, not per-face. Pattern shared with §6.2 LongBridge.
    let mesh_min_along_up = mesh
        .vertices
        .iter()
        .map(|v| v.coords.dot(&up))
        .fold(f64::INFINITY, f64::min);

    let mut flagged: HashMap<u32, FlaggedFaceMeta> = HashMap::new();

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

            let area = len / 2.0;
            // Mesh face index i fits in u32 (mesh size bounded well below 2^32).
            #[allow(clippy::cast_possible_truncation)]
            let face_idx = i as u32;
            flagged.insert(
                face_idx,
                FlaggedFaceMeta {
                    overhang_angle_rad: overhang_angle,
                    area,
                },
            );
        }
    }

    flagged
}

/// Partition flagged faces into edge-connected components (§5.3 Gap D).
///
/// Adjacency contract: only manifold edges (incident on exactly 2
/// faces) contribute. Non-manifold edges (>2 incident faces) and open
/// edges (1 incident face) do NOT — two flagged faces are
/// "geometrically adjacent" iff they share a manifold edge in the FDM
/// sense; vertex-only sharing produces disjoint regions.
///
/// Output ordering is deterministic across `HashMap` iteration
/// permutations: components emerge in min-face-idx order (each
/// component's seed is its smallest `face_idx`; smaller seeds are
/// visited first), and faces within each component are sorted
/// ascending.
fn partition_flagged_into_components(
    mesh: &IndexedMesh,
    flagged: &HashMap<u32, FlaggedFaceMeta>,
) -> Vec<Vec<u32>> {
    let edge_to_faces = build_edge_to_faces(mesh);
    let mut adjacency: HashMap<u32, Vec<u32>> = HashMap::new();
    for faces in edge_to_faces.values() {
        if faces.len() == 2 {
            let a = faces[0];
            let b = faces[1];
            if flagged.contains_key(&a) && flagged.contains_key(&b) {
                adjacency.entry(a).or_default().push(b);
                adjacency.entry(b).or_default().push(a);
            }
        }
    }

    let mut sorted_seeds: Vec<u32> = flagged.keys().copied().collect();
    sorted_seeds.sort_unstable();
    let mut visited: HashSet<u32> = HashSet::new();
    let mut components: Vec<Vec<u32>> = Vec::new();

    for &seed in &sorted_seeds {
        if !visited.insert(seed) {
            continue;
        }
        let mut component: Vec<u32> = vec![seed];
        let mut stack: Vec<u32> = vec![seed];
        while let Some(face_idx) = stack.pop() {
            if let Some(neighbors) = adjacency.get(&face_idx) {
                for &n in neighbors {
                    if visited.insert(n) {
                        component.push(n);
                        stack.push(n);
                    }
                }
            }
        }
        component.sort_unstable();
        components.push(component);
    }

    components
}

/// Classify `ExcessiveOverhang` severity per §4.3 of the v0.8 fix arc spec.
///
/// Maps the per-region max overhang angle (in degrees, tilt-from-vertical)
/// to `IssueSeverity` via three bands relative to the printer's
/// `max_overhang_angle` threshold (also in degrees):
///
/// - `observed > threshold + 30°` → `Critical` (e.g., 80° on a 45° FDM threshold)
/// - `threshold + 15° < observed ≤ threshold + 30°` → `Warning`
/// - `threshold < observed ≤ threshold + 15°` → `Info`
///
/// The lowest-band entry condition `observed > threshold` is the
/// upstream `flag_overhang_faces` invariant — only faces that strictly
/// exceed the threshold reach this classifier — but the explicit upper
/// bounds stay for self-documentation and to centralize the policy in
/// one place.
///
/// Boundary fixtures should bracket each band edge by ≥5° to absorb
/// the ~ULP drift that `acos`-based normal-tilt computation accumulates
/// near `π/2` (see §5.4 / §8.1 Gap E risk row 2). Single source of
/// policy: the only call site is `emit_overhang_component`. Future
/// detectors with their own bands declare their own classifiers.
fn classify_overhang_severity(observed_angle_deg: f64, threshold_deg: f64) -> IssueSeverity {
    if observed_angle_deg > threshold_deg + 30.0 {
        IssueSeverity::Critical
    } else if observed_angle_deg > threshold_deg + 15.0 {
        IssueSeverity::Warning
    } else {
        IssueSeverity::Info
    }
}

/// Aggregate one connected component's per-face metadata and emit an
/// `OverhangRegion`, a `SupportRegion`, and an `ExcessiveOverhang`
/// `PrintIssue` (§5.3 Gap D per-component emission).
///
/// The component's reported `angle` is the per-component max
/// `overhang_angle` (in degrees); `area` is the summed area; the
/// `center` is the mean of per-face centroids. Severity is the
/// §4.3 angle-based classification of the per-component max angle
/// (see `classify_overhang_severity`).
fn emit_overhang_component(
    mesh: &IndexedMesh,
    config: &PrinterConfig,
    flagged: &HashMap<u32, FlaggedFaceMeta>,
    component: Vec<u32>,
    validation: &mut PrintValidation,
) {
    let mut max_overhang_angle_rad: f64 = 0.0;
    let mut total_area: f64 = 0.0;
    let mut centroid_sum_x: f64 = 0.0;
    let mut centroid_sum_y: f64 = 0.0;
    let mut centroid_sum_z: f64 = 0.0;

    for &face_idx in &component {
        // Lookup is infallible by construction: face_idx came from the
        // DFS over `flagged.keys()`. Skip on a (impossible) miss to
        // keep the code panic-free per the workspace
        // `clippy::expect_used` policy.
        if let Some(meta) = flagged.get(&face_idx) {
            max_overhang_angle_rad = max_overhang_angle_rad.max(meta.overhang_angle_rad);
            total_area += meta.area;
        }
        let face_centroid = compute_face_centroid(mesh, face_idx as usize);
        centroid_sum_x += face_centroid.x;
        centroid_sum_y += face_centroid.y;
        centroid_sum_z += face_centroid.z;
    }

    // `component.len()` is bounded by `mesh.face_count()`, which fits
    // in u32 (asserted by the upstream `i as u32` cast); usize → f64
    // is exact for any value below 2^53, well above the u32 range.
    #[allow(clippy::cast_precision_loss)]
    let n = component.len() as f64;
    let center = Point3::new(centroid_sum_x / n, centroid_sum_y / n, centroid_sum_z / n);

    let region = OverhangRegion::new(center, max_overhang_angle_rad.to_degrees(), total_area)
        .with_faces(component.clone());
    validation.overhangs.push(region);

    let support_volume = total_area * 5.0; // Rough estimate
    let support_region =
        SupportRegion::new(center, support_volume, 10.0).with_faces(component.clone());
    validation.support_regions.push(support_region);

    let max_overhang_angle_deg = max_overhang_angle_rad.to_degrees();
    let severity = classify_overhang_severity(max_overhang_angle_deg, config.max_overhang_angle);

    let issue = PrintIssue::new(
        PrintIssueType::ExcessiveOverhang,
        severity,
        format!(
            "{} face(s) with max overhang {:.1}° (threshold {:.1}°, region area: {:.1} mm²)",
            component.len(),
            max_overhang_angle_deg,
            config.max_overhang_angle,
            total_area
        ),
    )
    .with_location(center)
    .with_affected_elements(component);

    validation.issues.push(issue);
}

/// Build a map from undirected edges to incident face indices.
///
/// Keys are `(min, max)`-normalized vertex pairs; values are the list of
/// face indices that reference the edge, in mesh-iteration order. Used by
/// `check_basic_manifold` for edge-count classification (open vs
/// non-manifold) and by `check_overhangs`' region-split logic
/// (§5.3 Gap D, lands at the next commit).
fn build_edge_to_faces(mesh: &IndexedMesh) -> HashMap<(u32, u32), Vec<u32>> {
    let mut edge_to_faces: HashMap<(u32, u32), Vec<u32>> = HashMap::new();

    for (face_idx, face) in mesh.faces.iter().enumerate() {
        let idx0 = face[0];
        let idx1 = face[1];
        let idx2 = face[2];

        // Add edges (always store with smaller index first for consistency)
        let edges = [
            (idx0.min(idx1), idx0.max(idx1)),
            (idx1.min(idx2), idx1.max(idx2)),
            (idx2.min(idx0), idx2.max(idx0)),
        ];

        // Mesh face index fits in u32 (mesh size bounded well below 2^32);
        // pattern shared with the cast in `check_overhangs`.
        #[allow(clippy::cast_possible_truncation)]
        let face_idx = face_idx as u32;
        for edge in &edges {
            edge_to_faces.entry(*edge).or_default().push(face_idx);
        }
    }

    edge_to_faces
}

/// Basic manifold check (edge usage).
fn check_basic_manifold(mesh: &IndexedMesh, validation: &mut PrintValidation) {
    let edge_to_faces = build_edge_to_faces(mesh);

    // In a manifold mesh, each edge should be shared by exactly 2 faces
    let boundary_edges: Vec<_> = edge_to_faces
        .iter()
        .filter(|(_, faces)| faces.len() != 2)
        .collect();

    if !boundary_edges.is_empty() {
        let non_manifold_count = boundary_edges.iter().filter(|(_, f)| f.len() > 2).count();
        let open_edge_count = boundary_edges.iter().filter(|(_, f)| f.len() == 1).count();

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

        // Post-Gap-D: the two flagged faces share no edge or vertex
        // (one at x = 0, one at x = 5), so they split into two disjoint
        // regions. Each region has one face, so its `angle` field equals
        // that single face's overhang angle. Assertions use min/max over
        // the regions to be independent of HashMap iteration order in
        // the partition output.
        assert_eq!(
            result.overhangs.len(),
            2,
            "two flagged faces with no shared edge or vertex split into two regions post-Gap-D"
        );
        let max_angle = result
            .overhangs
            .iter()
            .map(|r| r.angle)
            .fold(0.0_f64, f64::max);
        let min_angle = result
            .overhangs
            .iter()
            .map(|r| r.angle)
            .fold(f64::INFINITY, f64::min);
        approx::assert_relative_eq!(max_angle, 70.0, epsilon = 1e-6);
        approx::assert_relative_eq!(min_angle, 50.0, epsilon = 1e-6);
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

    // ---- §5.3 Gap D connected-region partition tests --------------------
    //
    // Lock in the post-§5.3 semantic: `check_overhangs` partitions
    // flagged faces into edge-connected components via the
    // `build_edge_to_faces` helper (commit #4). Each component emits
    // one `OverhangRegion` (mean of face centroids, max overhang_angle
    // in degrees, summed area) and one matching `SupportRegion`
    // (volume = component_area × 5.0, height = 10.0 mm placeholder).
    // Pre-Gap-D v0.7 lumped ALL flagged faces into one region using
    // `overhang_faces[0]`'s centroid (geometry-blind).
    //
    // Adjacency contract: two flagged faces share a component IFF they
    // share a manifold edge (incident on exactly 2 faces). Non-manifold
    // edges (>2 incident faces) and open edges (1 incident face) do
    // NOT contribute adjacency. Faces sharing only a vertex are NOT
    // adjacent.
    //
    // Invariants:
    //   1. `validation.support_regions.len() == validation.overhangs.len()`
    //      preserved at component granularity.
    //   2. Σ(component.area) == total_overhang_area pre-Gap-D
    //      (partition is exhaustive — every flagged face lands in
    //      exactly one component).
    //   3. Components emerge in min-face-idx order; faces within each
    //      component sorted ascending. Independent of HashMap
    //      iteration order.

    #[test]
    fn test_overhang_single_connected_region() {
        // Two coplanar roof triangles sharing the diagonal edge (3, 5):
        // both have downward normal (0, 0, -1), both flag, and they
        // form a single connected component → one OverhangRegion.
        let mesh = IndexedMesh::from_parts(
            vec![
                // Anchor at z=0 (top-facing, not flagged):
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(10.0, 0.0, 0.0),
                Point3::new(0.0, 10.0, 0.0),
                // Two roof triangles forming a 1×1 square at z=5:
                Point3::new(0.0, 0.0, 5.0), // 3
                Point3::new(0.0, 1.0, 5.0), // 4
                Point3::new(1.0, 0.0, 5.0), // 5
                Point3::new(1.0, 1.0, 5.0), // 6
            ],
            vec![[0, 1, 2], [3, 4, 5], [4, 6, 5]],
        );
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the connected-roof fixture");

        assert_eq!(
            result.overhangs.len(),
            1,
            "two flagged faces sharing a manifold edge form one connected region"
        );
        assert_eq!(
            result.overhangs[0].faces.len(),
            2,
            "the single region contains both flagged faces"
        );
        assert_eq!(
            result.support_regions.len(),
            result.overhangs.len(),
            "1:1 support/overhang invariant preserved at component granularity"
        );
    }

    #[test]
    fn test_overhang_two_disjoint_regions() {
        // Two roof triangles at separated x positions sharing no
        // vertices (geometric gap). Both flag. Faces are disjoint:
        // every face index appears in exactly one region's
        // `faces` list (partition is exhaustive and non-overlapping).
        let mesh = IndexedMesh::from_parts(
            vec![
                // Anchor at z=0:
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(20.0, 0.0, 0.0),
                Point3::new(0.0, 5.0, 0.0),
                // Roof 1 at x ∈ [0, 1]:
                Point3::new(0.0, 0.0, 5.0),
                Point3::new(0.0, 1.0, 5.0),
                Point3::new(1.0, 0.0, 5.0),
                // Roof 2 at x ∈ [10, 11] (no shared vertices):
                Point3::new(10.0, 0.0, 5.0),
                Point3::new(10.0, 1.0, 5.0),
                Point3::new(11.0, 0.0, 5.0),
            ],
            vec![[0, 1, 2], [3, 4, 5], [6, 7, 8]],
        );
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the two-disjoint-roofs fixture");

        assert_eq!(
            result.overhangs.len(),
            2,
            "two flagged faces with no shared geometry form two regions"
        );
        // Face-disjoint membership: union of region face lists has the
        // same length as the sum of region face counts (no duplicates).
        let mut all_faces: Vec<u32> = result
            .overhangs
            .iter()
            .flat_map(|r| r.faces.iter().copied())
            .collect();
        let total_count = all_faces.len();
        all_faces.sort_unstable();
        all_faces.dedup();
        assert_eq!(
            all_faces.len(),
            total_count,
            "regions are face-disjoint (partition is non-overlapping)"
        );
        assert_eq!(
            result.support_regions.len(),
            result.overhangs.len(),
            "1:1 support/overhang invariant preserved at component granularity"
        );
    }

    #[test]
    fn test_overhang_region_centroid_is_component_centroid() {
        // Single triangular overhang at known position.
        // Triangle vertices (1, 0, 5), (1, 1, 5), (2, 0, 5) →
        // analytical centroid ((1+1+2)/3, (0+1+0)/3, (5+5+5)/3)
        //                   = (4/3, 1/3, 5).
        // Single-face component → component centroid = face centroid.
        let mesh = IndexedMesh::from_parts(
            vec![
                // Anchor at z=0:
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(10.0, 0.0, 0.0),
                Point3::new(0.0, 10.0, 0.0),
                // Single overhang at known position:
                Point3::new(1.0, 0.0, 5.0),
                Point3::new(1.0, 1.0, 5.0),
                Point3::new(2.0, 0.0, 5.0),
            ],
            vec![[0, 1, 2], [3, 4, 5]],
        );
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the single-face-overhang fixture");

        assert_eq!(result.overhangs.len(), 1);
        let center = result.overhangs[0].center;
        approx::assert_relative_eq!(center.x, 4.0_f64 / 3.0, epsilon = 1e-6);
        approx::assert_relative_eq!(center.y, 1.0_f64 / 3.0, epsilon = 1e-6);
        approx::assert_relative_eq!(center.z, 5.0, epsilon = 1e-6);
    }

    #[test]
    fn test_overhang_no_overhangs() {
        // Watertight cube on the build plate: bottom faces are
        // filtered by the M.2 build-plate filter; all other faces are
        // top-facing or vertical. Result: zero overhangs and zero
        // support regions. Locks the §5.3 acceptance bullet
        // "no overhangs → no support_regions" at component
        // granularity.
        let mesh = create_watertight_cube();
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the cube-on-plate fixture");

        assert!(
            result.overhangs.is_empty(),
            "no overhangs after build-plate filter on cube-on-plate"
        );
        assert!(
            result.support_regions.is_empty(),
            "no support regions when no overhangs (1:1 invariant at zero)"
        );
    }

    #[test]
    fn test_overhang_face_adjacency_via_shared_edge() {
        // Locks the manifold-edge adjacency contract: two flagged
        // faces share a component IFF they share a manifold edge.
        // Vertex-only sharing produces disjoint components. Two
        // contrast variants in one test for direct comparison.
        let config = PrinterConfig::fdm_default();

        // Variant A — edge-shared: two roof triangles share edge (4, 5).
        // Both have downward normal (0, 0, -1) → both flag → ONE region.
        let mesh_edge_shared = IndexedMesh::from_parts(
            vec![
                // Anchor at z=0:
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(10.0, 0.0, 0.0),
                Point3::new(0.0, 10.0, 0.0),
                // Two coplanar roofs at z=5:
                Point3::new(0.0, 0.0, 5.0), // 3
                Point3::new(0.0, 1.0, 5.0), // 4
                Point3::new(1.0, 0.0, 5.0), // 5
                Point3::new(1.0, 1.0, 5.0), // 6
            ],
            vec![[0, 1, 2], [3, 4, 5], [4, 6, 5]],
        );
        #[allow(clippy::expect_used)]
        let edge_shared = validate_for_printing(&mesh_edge_shared, &config)
            .expect("validation should succeed for the edge-shared variant");
        assert_eq!(
            edge_shared.overhangs.len(),
            1,
            "two faces sharing a manifold edge form one connected region"
        );

        // Variant B — vertex-only: the second roof shares only vertex 5
        // with the first (no shared edge). Both flag → TWO regions.
        let mesh_vertex_only = IndexedMesh::from_parts(
            vec![
                // Anchor at z=0:
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(10.0, 0.0, 0.0),
                Point3::new(0.0, 10.0, 0.0),
                // Roof 1:
                Point3::new(0.0, 0.0, 5.0), // 3
                Point3::new(0.0, 1.0, 5.0), // 4
                Point3::new(1.0, 0.0, 5.0), // 5 (shared with roof 2)
                // Roof 2 — shares only vertex 5:
                Point3::new(1.0, 1.0, 5.0), // 6
                Point3::new(2.0, 0.0, 5.0), // 7
            ],
            vec![[0, 1, 2], [3, 4, 5], [5, 6, 7]],
        );
        #[allow(clippy::expect_used)]
        let vertex_only = validate_for_printing(&mesh_vertex_only, &config)
            .expect("validation should succeed for the vertex-only variant");
        assert_eq!(
            vertex_only.overhangs.len(),
            2,
            "two faces sharing only a vertex form two disjoint regions"
        );
    }

    // ---- §5.4 Gap E severity classifier tests ---------------------------
    //
    // Lock in the post-§5.4 semantic: ExcessiveOverhang severity is
    // classified by the per-region max overhang angle relative to
    // `config.max_overhang_angle` per the §4.3 angle bands:
    //
    //   - observed > threshold + 30°               → Critical
    //   - threshold + 15° < observed ≤ threshold + 30°   → Warning
    //   - threshold       < observed ≤ threshold + 15°   → Info
    //
    // Pre-Gap-E v0.7 used an area-based ternary
    // (`if total_area > 1000.0 { Warning } else { Info }`), capping
    // severity at Warning even for near-90° roofs and silently allowing
    // `is_printable()` to return true on slicer-rejected meshes.
    // Post-v0.8 a Critical overhang flips `is_printable()` to false,
    // matching FDM-slicer convention (PrusaSlicer / Cura).
    //
    // FP-fragility: the test fixtures bracket each band edge by ≥5° to
    // stay clear of `make_overhang_fixture`'s acos-based ULP drift at
    // exact boundaries (see §8.1 Gap E risk row 2). Per the
    // `make_overhang_fixture` contract (§5.9), the test face's
    // `overhang_angle` equals β by construction, so an 80° β produces
    // an 80° flagged-face angle ± a few ULPs — well within any band's
    // interior at the chosen 50° / 65° / 80° fixture set.

    #[test]
    fn test_overhang_severity_critical_at_steep() {
        // 80° on FDM threshold 45°: 80 > 75 (= 45 + 30) → Critical.
        let mesh = make_overhang_fixture(80.0_f64.to_radians(), 5.0);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the 80° fixture");

        #[allow(clippy::expect_used)]
        let overhang_issue = result
            .issues
            .iter()
            .find(|i| i.issue_type == PrintIssueType::ExcessiveOverhang)
            .expect("80° overhang must produce an ExcessiveOverhang issue");
        assert_eq!(overhang_issue.severity, IssueSeverity::Critical);
    }

    #[test]
    fn test_overhang_severity_warning_at_medium() {
        // 65° on FDM threshold 45°: 60 (= 45 + 15) < 65 ≤ 75 → Warning.
        let mesh = make_overhang_fixture(65.0_f64.to_radians(), 5.0);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the 65° fixture");

        #[allow(clippy::expect_used)]
        let overhang_issue = result
            .issues
            .iter()
            .find(|i| i.issue_type == PrintIssueType::ExcessiveOverhang)
            .expect("65° overhang must produce an ExcessiveOverhang issue");
        assert_eq!(overhang_issue.severity, IssueSeverity::Warning);
    }

    #[test]
    fn test_overhang_severity_info_at_borderline() {
        // 50° on FDM threshold 45°: 45 < 50 ≤ 60 (= 45 + 15) → Info.
        let mesh = make_overhang_fixture(50.0_f64.to_radians(), 5.0);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the 50° fixture");

        #[allow(clippy::expect_used)]
        let overhang_issue = result
            .issues
            .iter()
            .find(|i| i.issue_type == PrintIssueType::ExcessiveOverhang)
            .expect("50° overhang must produce an ExcessiveOverhang issue");
        assert_eq!(overhang_issue.severity, IssueSeverity::Info);
    }

    /// Build a closed (watertight, manifold) overhang fixture for the
    /// §5.4 `is_printable()` polarity tests below. Cross-section is an
    /// "L" extruded along +y; the L's underside-of-arm segment is
    /// tilted to land at `target_overhang_angle_deg`
    /// (tilt-from-vertical, FDM convention). The L's leg sits on the
    /// build plate (z=0); the slanted underside of the arm hovers
    /// above (`face_min_along_up` = 1.0 > `mesh_min` = 0.0), so the
    /// build-plate filter does not block the overhang flag. The
    /// remaining five side walls are vertical, horizontal-top, or on
    /// the build plate, so the only flagged region is the slanted
    /// underside (2 triangles forming one connected component).
    ///
    /// Construction rationale: a *closed* mesh is required for these
    /// tests because the simpler 2-triangle `make_overhang_fixture`
    /// emits a `NotWatertight` Critical issue (six open edges) that
    /// would taint `is_printable()` independently of the overhang
    /// severity under test.
    fn make_closed_overhang_fixture(target_overhang_angle_deg: f64) -> IndexedMesh {
        let theta_rad = target_overhang_angle_deg.to_radians();
        let sin_t = theta_rad.sin();
        let cos_t = theta_rad.cos();

        let vertices = vec![
            // Front cross-section (y = 0):
            Point3::new(0.0, 0.0, 0.0), // 0  P1 — leg bottom-left
            Point3::new(1.0, 0.0, 0.0), // 1  P2 — leg bottom-right
            Point3::new(1.0, 0.0, 1.0), // 2  P3 — leg top-right (start of overhang)
            Point3::new(1.0 + sin_t, 0.0, 1.0 + cos_t), // 3  P4 — overhang outer end
            Point3::new(1.0 + sin_t, 0.0, 2.0 + cos_t), // 4  P5 — arm top-right
            Point3::new(0.0, 0.0, 2.0 + cos_t), // 5  P6 — arm top-left
            // Back cross-section (y = 1):
            Point3::new(0.0, 1.0, 0.0),                 // 6  P1'
            Point3::new(1.0, 1.0, 0.0),                 // 7  P2'
            Point3::new(1.0, 1.0, 1.0),                 // 8  P3'
            Point3::new(1.0 + sin_t, 1.0, 1.0 + cos_t), // 9  P4'
            Point3::new(1.0 + sin_t, 1.0, 2.0 + cos_t), // 10 P5'
            Point3::new(0.0, 1.0, 2.0 + cos_t),         // 11 P6'
        ];
        let faces = vec![
            // Bottom of leg (outward -z; build-plate filtered):
            [0, 6, 7],
            [0, 7, 1],
            // Right side of leg (outward +x; vertical wall, not flagged):
            [1, 7, 8],
            [1, 8, 2],
            // Slanted underside of arm — the OVERHANG
            // (outward (cos θ, 0, -sin θ); flagged at angle θ):
            [2, 8, 9],
            [2, 9, 3],
            // Right side of arm (outward +x; vertical wall):
            [3, 9, 10],
            [3, 10, 4],
            // Top of arm (outward +z; top face):
            [4, 10, 11],
            [4, 11, 5],
            // Left side (outward -x; vertical wall):
            [5, 11, 6],
            [5, 6, 0],
            // Front face (outward -y; hexagon fanned from P1):
            [0, 1, 2],
            [0, 2, 3],
            [0, 3, 4],
            [0, 4, 5],
            // Back face (outward +y; hexagon fanned from P1'):
            [6, 11, 10],
            [6, 10, 9],
            [6, 9, 8],
            [6, 8, 7],
        ];
        IndexedMesh::from_parts(vertices, faces)
    }

    #[test]
    fn test_is_printable_blocks_critical_overhang() {
        // Gap E primary regression: an 80° single-region overhang on
        // FDM threshold 45° must flip `is_printable()` to false. The
        // closed L-prism keeps the only Critical-issue surface to the
        // overhang itself (avoids the `NotWatertight` Critical that
        // would taint the polarity check). Pre-Gap-E v0.7's area-based
        // heuristic capped severity at Warning for small-area faces,
        // so `is_printable()` was true even on near-roof overhangs —
        // the bug Gap E fixes.
        let mesh = make_closed_overhang_fixture(80.0);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the closed 80° fixture");

        assert!(
            !result.is_printable(),
            "80° overhang on 45° FDM threshold must be Critical and block is_printable()"
        );
    }

    #[test]
    fn test_is_printable_allows_borderline_overhang() {
        // 50° single-region overhang on FDM threshold 45°: Info
        // severity, does not block `is_printable()` (only Critical
        // does). Pairs with `test_is_printable_blocks_critical_overhang`
        // to lock the band-to-printable mapping at both polarities;
        // the closed L-prism keeps `NotWatertight` from clouding the
        // result.
        let mesh = make_closed_overhang_fixture(50.0);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the closed 50° fixture");

        assert!(
            result.is_printable(),
            "50° overhang on 45° FDM threshold is Info; does not block is_printable()"
        );
    }
}
