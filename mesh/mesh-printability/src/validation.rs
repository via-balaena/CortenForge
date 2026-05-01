//! Main validation logic for printability analysis.
//!
//! Provides the primary validation functionality to check meshes
//! against printer constraints.

use std::collections::VecDeque;

use hashbrown::{HashMap, HashSet};
use mesh_types::{IndexedMesh, Point3, Vector3};

use crate::config::{PrintTechnology, PrinterConfig};
use crate::error::{PrintabilityError, PrintabilityResult};
use crate::issues::{IssueSeverity, PrintIssue, PrintIssueType};
use crate::regions::{
    LongBridgeRegion, OverhangRegion, SupportRegion, ThinWallRegion, TrappedVolumeRegion,
};

/// General geometric tie-breaking tolerance in millimetres (per spec §4.2).
/// Two call sites:
/// - `check_overhangs` build-plate filter — identifies faces resting on
///   the build plate by comparing per-face min projection against
///   mesh-min along the build-up axis.
/// - `moller_trumbore` (§6.1) — both the determinant magnitude check
///   (ray co-planar with triangle) and the `t > EPS` ray-parameter
///   guard (positive ray direction).
const EPS_GEOMETRIC: f64 = 1e-9;

/// Ray-origin inward offset for the §6.1 `ThinWall` ray-cast (mm).
///
/// Offsets the ray's starting point inward by 1 µm along `-N_F` so the
/// Möller-Trumbore intersection at `t > EPS_GEOMETRIC` cannot self-hit
/// the originating face. Three orders of magnitude above
/// `EPS_GEOMETRIC` (1e-9 mm) — adequate margin per §8.1 Gap C row 1.
const EPS_RAY_OFFSET: f64 = 1e-6;

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

    /// Long-bridge regions detected (§6.2 Gap G).
    pub long_bridges: Vec<LongBridgeRegion>,

    /// Trapped-volume regions detected (§6.3 Gap H).
    pub trapped_volumes: Vec<TrappedVolumeRegion>,

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
            long_bridges: Vec::new(),
            trapped_volumes: Vec::new(),
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

    // §6.1 Gap C — thin walls via inward ray-cast. Runs after manifold so
    // its precondition check (watertight + consistent winding) reflects
    // the same edge analysis the manifold detector just performed.
    check_thin_walls(mesh, config, &mut validation);

    // §6.2 Gap G — long bridges via boundary-edge span analysis. Runs
    // after `check_thin_walls`; FDM/SLA-only (silent skip on SLS/MJF
    // per `requires_supports()`).
    check_long_bridges(mesh, config, &mut validation);

    // §6.3 Gap H — trapped volumes via voxel-based exterior flood-fill.
    // Watertight precondition (NOT consistent winding, distinct from
    // ThinWall) — `§9.1` row 11 documents that TrappedVolume tolerates
    // inconsistent winding.
    check_trapped_volumes(mesh, config, &mut validation);

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

/// Per-face metadata captured during the §6.1 `ThinWall` flag-collection
/// loop. Used post-loop for per-cluster aggregation: each cluster's
/// `ThinWallRegion` reports the *minimum* observed `thickness` (the
/// worst-case constraint) and the summed `area` over its member faces.
#[derive(Clone, Copy)]
struct ThinWallFlagMeta {
    thickness: f64,
    area: f64,
}

/// Per-face metadata captured during the §6.2 `LongBridge` flag-collection
/// loop. `area` is summed per cluster for the issue-description's
/// `region area: …` line (parity with `ThinWall` + `ExcessiveOverhang`).
/// `angle_from_neg_up_rad` is preserved for v0.9 diagnostic surface
/// (per-cluster "most-horizontal face" reporting); the v0.8 emit path
/// reads it via `meta.angle_from_neg_up_rad` only when the field is
/// referenced explicitly, so the dead-code lint allow below is the
/// minimal escape until v0.9.
#[derive(Clone, Copy)]
#[allow(dead_code)]
struct LongBridgeFlagMeta {
    angle_from_neg_up_rad: f64,
    area: f64,
}

/// Maximum tilt-from-horizontal (radians) for `LongBridge` candidate
/// faces (§6.2 — `ANGLE_TOL_HORIZONTAL = 30°`). A face whose outward
/// normal sits within 30° of `-up` is "near-horizontal downward" — the
/// FDM/SLA bridge-candidate predicate.
const LONG_BRIDGE_ANGLE_TOL_RAD: f64 = std::f64::consts::FRAC_PI_6;

/// Check for excessive overhangs.
fn check_overhangs(mesh: &IndexedMesh, config: &PrinterConfig, validation: &mut PrintValidation) {
    if !config.technology.requires_supports() {
        return;
    }

    let max_angle_rad = config.max_overhang_angle.to_radians();
    let up = config.build_up_direction;

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
fn partition_flagged_into_components<T>(
    mesh: &IndexedMesh,
    flagged: &HashMap<u32, T>,
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
///
/// Two passes run side-by-side:
/// 1. **Undirected edge sharing** — each edge `(min, max)` should be incident
///    on exactly 2 faces; deviations emit `NonManifold` (>2 faces) or
///    `NotWatertight` (1 face) `Critical` issues.
/// 2. **Directed edge orientation (Gap F, §5.5)** — for each face, record
///    the three traversals `(face[0], face[1])`, `(face[1], face[2])`,
///    `(face[2], face[0])`. In a consistently-wound surface, each directed
///    pair appears at most once: shared edges are traversed in opposite
///    directions by the two incident faces. Any directed pair appearing
///    more than once means two faces traverse that edge in the *same*
///    direction (one is "inside out" relative to the other) — flagged as
///    `NonManifold` `Critical` with description containing
///    "winding inconsistency" so callers can discriminate from open-edge
///    and non-manifold-edge cases by description string.
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

    // §5.5 Gap F — directed-edge winding-orientation pass.
    let mut directed_edges: HashMap<(u32, u32), u32> = HashMap::new();
    for face in &mesh.faces {
        let edges = [(face[0], face[1]), (face[1], face[2]), (face[2], face[0])];
        for edge in &edges {
            *directed_edges.entry(*edge).or_insert(0) += 1;
        }
    }
    let inconsistent_count = directed_edges.values().filter(|&&c| c > 1).count();
    if inconsistent_count > 0 {
        let issue = PrintIssue::new(
            PrintIssueType::NonManifold,
            IssueSeverity::Critical,
            format!(
                "{inconsistent_count} edge(s) traversed in same direction by multiple faces (winding inconsistency)"
            ),
        );
        validation.issues.push(issue);
    }
}

// ===== §6.1 Gap C — ThinWall detector via inward ray-cast =================

/// Möller-Trumbore ray-triangle intersection (§6.1).
///
/// Returns `Some(t)` for ray parameter `t > EPS_GEOMETRIC` if the ray hits
/// the triangle's interior; `None` otherwise. Standard textbook form
/// (Möller & Trumbore 1997). Early-out conditions:
///
/// - ray is co-planar with the triangle (`a.abs() < EPS_GEOMETRIC`),
/// - barycentric `u`, `v` fall outside `[0, 1]` or `u + v > 1`,
/// - parameter `t` is at or behind the ray origin (`t <= EPS_GEOMETRIC`).
///
/// `t > EPS_GEOMETRIC` (1e-9 mm) is three orders above the
/// `EPS_RAY_OFFSET` (1e-6 mm) that `flag_thin_wall_faces` applies, so
/// any back-reflection from the originating face is rejected.
// Single-character names are the canonical Möller-Trumbore notation
// (h = direction × edge2; a = determinant; f = 1/a; s, q = barycentric
// intermediates; u, v = barycentric coords; t = ray parameter).
// Renaming obscures the textbook reading — same naming choice most
// ray-tracer references use.
#[allow(clippy::many_single_char_names)]
fn moller_trumbore(
    origin: Point3<f64>,
    direction: Vector3<f64>,
    v0: Point3<f64>,
    v1: Point3<f64>,
    v2: Point3<f64>,
) -> Option<f64> {
    let edge1 = Vector3::new(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
    let edge2 = Vector3::new(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
    let h = direction.cross(&edge2);
    let a = edge1.dot(&h);
    if a.abs() < EPS_GEOMETRIC {
        return None;
    }
    let f = 1.0 / a;
    let s = Vector3::new(origin.x - v0.x, origin.y - v0.y, origin.z - v0.z);
    let u = f * s.dot(&h);
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = s.cross(&edge1);
    let v = f * direction.dot(&q);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = f * edge2.dot(&q);
    if t > EPS_GEOMETRIC { Some(t) } else { None }
}

/// Classify `ThinWall` severity per §6.1 (two-band).
///
/// `Critical` if `thickness < min_wall_thickness / 2`, `Warning` otherwise
/// (the lower-band entry condition `thickness < min_wall_thickness` is
/// `flag_thin_wall_faces`'s invariant). Single source of severity policy
/// — only call site is `emit_thin_wall_component`.
fn classify_thin_wall_severity(thickness: f64, min_wall: f64) -> IssueSeverity {
    if thickness < min_wall / 2.0 {
        IssueSeverity::Critical
    } else {
        IssueSeverity::Warning
    }
}

/// Watertight + consistent-winding precondition for §6.1 `ThinWall`.
///
/// Returns `true` iff every undirected edge is incident on exactly two
/// faces (watertight) AND every directed edge appears in at most one
/// face (consistent winding). Mirrors `check_basic_manifold`'s edge
/// passes; computed independently so the precondition is encoded in
/// the detector itself rather than relying on `validation.issues`
/// inspection.
fn is_watertight_and_consistent_winding(mesh: &IndexedMesh) -> bool {
    let edge_to_faces = build_edge_to_faces(mesh);
    if edge_to_faces.values().any(|faces| faces.len() != 2) {
        return false;
    }
    let mut directed: HashMap<(u32, u32), u32> = HashMap::new();
    for face in &mesh.faces {
        let edges = [(face[0], face[1]), (face[1], face[2]), (face[2], face[0])];
        for edge in &edges {
            *directed.entry(*edge).or_insert(0) += 1;
        }
    }
    directed.values().all(|&c| c <= 1)
}

/// Walk every face and ray-cast inward; collect those whose nearest
/// opposite face is closer than `config.min_wall_thickness`. Returns
/// per-face `thickness` + `area` keyed by face index for downstream
/// edge-adjacency clustering and per-cluster emission.
fn flag_thin_wall_faces(
    mesh: &IndexedMesh,
    config: &PrinterConfig,
) -> HashMap<u32, ThinWallFlagMeta> {
    let mut flagged: HashMap<u32, ThinWallFlagMeta> = HashMap::new();
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
        let edge1 = Vector3::new(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
        let edge2 = Vector3::new(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
        let raw_normal = edge1.cross(&edge2);

        // FP-bit preserved: same rationale as `flag_overhang_faces`'s
        // `len = sqrt(x*x + y*y + z*z)` site — keeping the explicit form
        // ensures threshold-boundary determinism cross-platform.
        #[allow(clippy::suboptimal_flops)]
        let len = (raw_normal.x * raw_normal.x
            + raw_normal.y * raw_normal.y
            + raw_normal.z * raw_normal.z)
            .sqrt();
        if len < 1e-10 {
            continue;
        }
        let normal = Vector3::new(raw_normal.x / len, raw_normal.y / len, raw_normal.z / len);
        let area = len / 2.0;

        let centroid = compute_face_centroid(mesh, i);
        // FP-bit preserved: rewriting `centroid - EPS_RAY_OFFSET * normal`
        // into `mul_add(-normal, centroid)` would shift the ray-origin's
        // FP bits cross-platform, which in turn shifts which faces flag
        // at the `min_wall_thickness` threshold. Same precedent as the
        // `flag_overhang_faces` `len = sqrt(...)` site. Bit-exactness
        // deferred — see CHANGELOG.md `[Unreleased] / v0.9 candidates`.
        #[allow(clippy::suboptimal_flops)]
        let origin = Point3::new(
            centroid.x - EPS_RAY_OFFSET * normal.x,
            centroid.y - EPS_RAY_OFFSET * normal.y,
            centroid.z - EPS_RAY_OFFSET * normal.z,
        );
        let direction = Vector3::new(-normal.x, -normal.y, -normal.z);

        let mut min_dist = f64::INFINITY;
        for (j, other) in mesh.faces.iter().enumerate() {
            if j == i {
                continue;
            }
            let oj0 = other[0] as usize;
            let oj1 = other[1] as usize;
            let oj2 = other[2] as usize;
            if oj0 >= mesh.vertices.len()
                || oj1 >= mesh.vertices.len()
                || oj2 >= mesh.vertices.len()
            {
                continue;
            }
            if let Some(t) = moller_trumbore(
                origin,
                direction,
                mesh.vertices[oj0],
                mesh.vertices[oj1],
                mesh.vertices[oj2],
            ) && t < min_dist
            {
                min_dist = t;
            }
        }

        // Reported thickness is min_dist + EPS_RAY_OFFSET — the geometric
        // wall thickness from outer face surface to inner face surface,
        // accounting for the ray's inward starting offset. Without this
        // correction, every reported thickness is off by 1 µm and a
        // wall at exactly `min_wall_thickness` flags spuriously
        // (`min_dist = 0.999999 < 1.0` at thickness = 1.0).
        let thickness = min_dist + EPS_RAY_OFFSET;
        if thickness < config.min_wall_thickness {
            // Mesh face index `i` fits in u32 (same bound as
            // `flag_overhang_faces`'s `i as u32` cast).
            #[allow(clippy::cast_possible_truncation)]
            let face_idx = i as u32;
            flagged.insert(face_idx, ThinWallFlagMeta { thickness, area });
        }
    }

    flagged
}

/// Aggregate one cluster's per-face metadata and emit a `ThinWallRegion`
/// + a `ThinWall` `PrintIssue` (§6.1 per-cluster emission).
///
/// Per cluster: `thickness` is the minimum observed (worst-case
/// constraint), `area` is the summed area, `center` is the mean of
/// per-face centroids. Severity per `classify_thin_wall_severity`.
fn emit_thin_wall_component(
    mesh: &IndexedMesh,
    config: &PrinterConfig,
    flagged: &HashMap<u32, ThinWallFlagMeta>,
    component: Vec<u32>,
    validation: &mut PrintValidation,
) {
    let mut min_thickness = f64::INFINITY;
    let mut total_area = 0.0_f64;
    let mut centroid_sum_x = 0.0_f64;
    let mut centroid_sum_y = 0.0_f64;
    let mut centroid_sum_z = 0.0_f64;

    for &face_idx in &component {
        if let Some(meta) = flagged.get(&face_idx) {
            if meta.thickness < min_thickness {
                min_thickness = meta.thickness;
            }
            total_area += meta.area;
        }
        let face_centroid = compute_face_centroid(mesh, face_idx as usize);
        centroid_sum_x += face_centroid.x;
        centroid_sum_y += face_centroid.y;
        centroid_sum_z += face_centroid.z;
    }

    // `component.len()` is bounded by `mesh.face_count()` (u32-fittable);
    // usize → f64 is exact below 2^53. Same precision rationale as
    // `emit_overhang_component`.
    #[allow(clippy::cast_precision_loss)]
    let n = component.len() as f64;
    let center = Point3::new(centroid_sum_x / n, centroid_sum_y / n, centroid_sum_z / n);

    let region = ThinWallRegion::new(center, min_thickness)
        .with_area(total_area)
        .with_faces(component.clone());
    validation.thin_walls.push(region);

    let severity = classify_thin_wall_severity(min_thickness, config.min_wall_thickness);
    let issue = PrintIssue::new(
        PrintIssueType::ThinWall,
        severity,
        format!(
            "{} face(s) with min thickness {:.3} mm (threshold {:.3} mm, region area: {:.1} mm²)",
            component.len(),
            min_thickness,
            config.min_wall_thickness,
            total_area
        ),
    )
    .with_location(center)
    .with_affected_elements(component);
    validation.issues.push(issue);
}

/// Detect thin walls via inward ray-cast (§6.1 Gap C).
///
/// Preconditions: watertight + consistent winding. On precondition
/// failure: emit one `Info` `PrintIssue` of type `DetectorSkipped` and
/// return without populating `validation.thin_walls`.
///
/// Algorithm:
///
/// 1. For each face F: ray-cast inward from
///    `centroid(F) - EPS_RAY_OFFSET * N_F` along `-N_F`; record `min_dist`
///    to the nearest opposite face via `moller_trumbore`.
/// 2. Flag F if `min_dist < config.min_wall_thickness`.
/// 3. Cluster flagged faces by edge-adjacency (using
///    `partition_flagged_into_components`, which reuses
///    `build_edge_to_faces`).
/// 4. Per cluster: emit one `ThinWallRegion` + one `ThinWall`
///    `PrintIssue`.
///
/// Per-face complexity is O(n) ray-tri intersections; total is O(n²).
/// Documented v0.9 BVH followup at >10k tris (see §6.1 of the v0.8 spec).
fn check_thin_walls(mesh: &IndexedMesh, config: &PrinterConfig, validation: &mut PrintValidation) {
    if !is_watertight_and_consistent_winding(mesh) {
        validation.issues.push(PrintIssue::new(
            PrintIssueType::DetectorSkipped,
            IssueSeverity::Info,
            "ThinWall detection requires watertight mesh with consistent winding (skipped)",
        ));
        return;
    }

    let flagged = flag_thin_wall_faces(mesh, config);
    if flagged.is_empty() {
        return;
    }

    let components = partition_flagged_into_components(mesh, &flagged);
    for component in components {
        emit_thin_wall_component(mesh, config, &flagged, component, validation);
    }
}

/// Construct an orthonormal basis `(e1, e2)` of the plane perpendicular
/// to `up` (which is assumed unit-length per `with_build_up_direction`'s
/// normalize-on-construct contract).
///
/// Reference vector picking avoids the degenerate case where the chosen
/// reference is collinear with `up`: pick `(1, 0, 0)` unless `up` is
/// nearly parallel to it (`|up.x| > 0.9`), in which case pick `(0, 1, 0)`.
/// `0.9` chosen so the reference is at least ~26° from `up` — avoiding
/// FP loss in the Gram-Schmidt subtraction `ref - (ref·up)*up`.
///
/// For default `+Z` up the basis falls out as `e1 = (1, 0, 0)`,
/// `e2 = (0, 1, 0)` — the natural world-X/Y projection. For `+Y` up the
/// basis is `e1 = (1, 0, 0)`, `e2 = (0, 0, -1)` — symmetric span.
fn perpendicular_plane_basis(up: Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    let reference = if up.x.abs() > 0.9 {
        Vector3::new(0.0, 1.0, 0.0)
    } else {
        Vector3::new(1.0, 0.0, 0.0)
    };
    // Gram-Schmidt: subtract the up-component of `reference` to land in
    // the perpendicular plane, then normalize.
    let dot = reference.dot(&up);
    let raw_e1 = reference - up * dot;
    let e1 = raw_e1.normalize();
    let e2 = up.cross(&e1);
    (e1, e2)
}

/// Classify `LongBridge` severity per §6.2 (two-band; no Info).
///
/// `Critical` if `span > max_span * 1.5`; `Warning` otherwise. The lower-
/// band entry condition `span > max_span` is `flag_long_bridge_faces`'s
/// downstream guard — only clusters whose span strictly exceeds
/// `max_bridge_span` reach this classifier. No Info band: bridges that
/// don't exceed the threshold don't flag at all (the FDM/SLA decision is
/// binary at the printer-config level).
const fn classify_long_bridge_severity(span: f64, max_span: f64) -> IssueSeverity {
    if span > max_span * 1.5 {
        IssueSeverity::Critical
    } else {
        IssueSeverity::Warning
    }
}

/// Walk every face and collect those whose outward normal lies within
/// `LONG_BRIDGE_ANGLE_TOL_RAD` (30°) of `-up` — "near-horizontal
/// downward" candidates. Build-plate filter (mesh-min-relative, same
/// pattern as `flag_overhang_faces`'s M.2) rejects faces resting on the
/// build plate.
fn flag_long_bridge_faces(
    mesh: &IndexedMesh,
    up: Vector3<f64>,
) -> HashMap<u32, LongBridgeFlagMeta> {
    let mesh_min_along_up = mesh
        .vertices
        .iter()
        .map(|v| v.coords.dot(&up))
        .fold(f64::INFINITY, f64::min);

    let neg_up = -up;
    let mut flagged: HashMap<u32, LongBridgeFlagMeta> = HashMap::new();

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
        let edge1 = Vector3::new(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
        let edge2 = Vector3::new(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
        let raw_normal = edge1.cross(&edge2);

        // FP-bit preserved: same rationale as `flag_overhang_faces`'s
        // `len = sqrt(...)` site — keeping the explicit form ensures
        // threshold-boundary determinism cross-platform.
        #[allow(clippy::suboptimal_flops)]
        let len = (raw_normal.x * raw_normal.x
            + raw_normal.y * raw_normal.y
            + raw_normal.z * raw_normal.z)
            .sqrt();
        if len < 1e-10 {
            continue;
        }
        let normal = Vector3::new(raw_normal.x / len, raw_normal.y / len, raw_normal.z / len);

        // Tilt from `-up` (the "downward horizontal" reference). A face
        // whose normal is exactly `-up` has angle 0; a vertical wall has
        // angle π/2. Flag iff angle < ANGLE_TOL_HORIZONTAL.
        #[allow(clippy::suboptimal_flops)]
        let dot_neg_up = normal.x * neg_up.x + normal.y * neg_up.y + normal.z * neg_up.z;
        let angle_from_neg_up = dot_neg_up.clamp(-1.0, 1.0).acos();
        if angle_from_neg_up >= LONG_BRIDGE_ANGLE_TOL_RAD {
            continue;
        }

        // Build-plate filter (mesh-min-relative): a face touching the
        // build plate is supported by the plate, not a bridge.
        let face_min_along_up = [v0, v1, v2]
            .iter()
            .map(|v| v.coords.dot(&up))
            .fold(f64::INFINITY, f64::min);
        if (face_min_along_up - mesh_min_along_up) < EPS_GEOMETRIC {
            continue;
        }

        let area = len / 2.0;
        // Mesh face index `i` fits in u32 (same bound as
        // `flag_overhang_faces`'s `i as u32` cast).
        #[allow(clippy::cast_possible_truncation)]
        let face_idx = i as u32;
        flagged.insert(
            face_idx,
            LongBridgeFlagMeta {
                angle_from_neg_up_rad: angle_from_neg_up,
                area,
            },
        );
    }

    flagged
}

/// Axis-aligned bounding box of a `LongBridge` cluster's vertices,
/// projected onto the plane perpendicular to `up`. Stored as struct
/// fields (rather than four bindings) to keep the call site readable
/// and avoid `clippy::similar_names` triggers on any pair like
/// `min_u`/`min_w`.
struct ClusterPerpBbox {
    min_along_e1: f64,
    max_along_e1: f64,
    min_along_e2: f64,
    max_along_e2: f64,
}

impl ClusterPerpBbox {
    const fn extent_e1(&self) -> f64 {
        self.max_along_e1 - self.min_along_e1
    }
    const fn extent_e2(&self) -> f64 {
        self.max_along_e2 - self.min_along_e2
    }
    const fn mid_e1(&self) -> f64 {
        f64::midpoint(self.min_along_e1, self.max_along_e1)
    }
    const fn mid_e2(&self) -> f64 {
        f64::midpoint(self.min_along_e2, self.max_along_e2)
    }
}

/// Project a `LongBridge` cluster's unique vertices onto the plane
/// spanned by `(e1, e2)` and compute the axis-aligned bbox of the
/// projection. Vertex de-dup via `HashSet<u32>` so shared cluster-
/// interior vertices are only sampled once.
fn cluster_perpendicular_bbox(
    mesh: &IndexedMesh,
    component: &[u32],
    e1: Vector3<f64>,
    e2: Vector3<f64>,
) -> ClusterPerpBbox {
    let mut vertex_indices: HashSet<u32> = HashSet::new();
    for &face_idx in component {
        let face = mesh.faces[face_idx as usize];
        vertex_indices.insert(face[0]);
        vertex_indices.insert(face[1]);
        vertex_indices.insert(face[2]);
    }

    let mut bbox = ClusterPerpBbox {
        min_along_e1: f64::INFINITY,
        max_along_e1: f64::NEG_INFINITY,
        min_along_e2: f64::INFINITY,
        max_along_e2: f64::NEG_INFINITY,
    };
    for &vi in &vertex_indices {
        let v = mesh.vertices[vi as usize];
        let proj_e1 = v.coords.dot(&e1);
        let proj_e2 = v.coords.dot(&e2);
        bbox.min_along_e1 = bbox.min_along_e1.min(proj_e1);
        bbox.max_along_e1 = bbox.max_along_e1.max(proj_e1);
        bbox.min_along_e2 = bbox.min_along_e2.min(proj_e2);
        bbox.max_along_e2 = bbox.max_along_e2.max(proj_e2);
    }
    bbox
}

/// Identify a cluster's boundary edges: edges shared with at least one
/// face that is NOT in the flagged set (per §6.2 line 988). Output is
/// `(min, max)`-normalized vertex pairs, sorted ascending for
/// determinism across `HashMap` iteration permutations.
fn cluster_boundary_edges(
    mesh: &IndexedMesh,
    component: &[u32],
    edge_to_faces: &HashMap<(u32, u32), Vec<u32>>,
    flagged: &HashMap<u32, LongBridgeFlagMeta>,
) -> Vec<(u32, u32)> {
    let mut boundary_edges: Vec<(u32, u32)> = Vec::new();
    let mut seen: HashSet<(u32, u32)> = HashSet::new();
    for &face_idx in component {
        let face = mesh.faces[face_idx as usize];
        let raw_edges = [(face[0], face[1]), (face[1], face[2]), (face[2], face[0])];
        for (a, b) in raw_edges {
            let key = (a.min(b), a.max(b));
            if !seen.insert(key) {
                continue;
            }
            if let Some(faces) = edge_to_faces.get(&key) {
                let on_boundary = faces.iter().any(|f| !flagged.contains_key(f));
                if on_boundary {
                    boundary_edges.push(key);
                }
            }
        }
    }
    boundary_edges.sort_unstable();
    boundary_edges
}

/// Project a cluster's vertices onto the plane perpendicular to `up`
/// (using basis `(e1, e2)`), compute the axis-aligned bounding box of
/// projections, and emit a `LongBridgeRegion` + `LongBridge` `PrintIssue`
/// if the max bbox extent exceeds `config.max_bridge_span`.
///
/// `start` and `end` lift the longest-axis endpoints back to 3D using
/// the cluster centroid's elevation along `up`; the issue location is
/// the midpoint.
fn emit_long_bridge_component(
    mesh: &IndexedMesh,
    config: &PrinterConfig,
    edge_to_faces: &HashMap<(u32, u32), Vec<u32>>,
    flagged: &HashMap<u32, LongBridgeFlagMeta>,
    component: Vec<u32>,
    validation: &mut PrintValidation,
) {
    let up = config.build_up_direction;
    let (e1, e2) = perpendicular_plane_basis(up);
    let bbox = cluster_perpendicular_bbox(mesh, &component, e1, e2);

    let extent_e1 = bbox.extent_e1();
    let extent_e2 = bbox.extent_e2();
    let span = extent_e1.max(extent_e2);
    if span <= config.max_bridge_span {
        return;
    }

    // Cluster centroid: mean of per-face centroids (matches
    // `emit_overhang_component` + `emit_thin_wall_component`). `total_area`
    // is the per-cluster sum used in the issue description (parity with
    // sister detectors' `region area: …` field).
    let mut centroid_sum_x = 0.0_f64;
    let mut centroid_sum_y = 0.0_f64;
    let mut centroid_sum_z = 0.0_f64;
    let mut total_area = 0.0_f64;
    for &face_idx in &component {
        if let Some(meta) = flagged.get(&face_idx) {
            total_area += meta.area;
        }
        let c = compute_face_centroid(mesh, face_idx as usize);
        centroid_sum_x += c.x;
        centroid_sum_y += c.y;
        centroid_sum_z += c.z;
    }
    // `component.len()` is bounded by `mesh.face_count()` (u32-fittable);
    // usize → f64 is exact below 2^53. Same precision rationale as
    // `emit_overhang_component`.
    #[allow(clippy::cast_precision_loss)]
    let n = component.len() as f64;
    let cluster_centroid = Point3::new(centroid_sum_x / n, centroid_sum_y / n, centroid_sum_z / n);

    // Pick the longest perpendicular-plane axis; lift the bbox center
    // back to 3D from its (e1, e2, up) projections.
    let (axis, long_proj, perp_axis, perp_proj) = if extent_e1 >= extent_e2 {
        (e1, bbox.mid_e1(), e2, bbox.mid_e2())
    } else {
        (e2, bbox.mid_e2(), e1, bbox.mid_e1())
    };
    let center_along_up = cluster_centroid.coords.dot(&up);
    let center_vec = axis * long_proj + perp_axis * perp_proj + up * center_along_up;
    let center = Point3::new(center_vec.x, center_vec.y, center_vec.z);

    let half_span = span / 2.0;
    let start_vec = center_vec - axis * half_span;
    let end_vec = center_vec + axis * half_span;
    let start = Point3::new(start_vec.x, start_vec.y, start_vec.z);
    let end = Point3::new(end_vec.x, end_vec.y, end_vec.z);

    let boundary_edges = cluster_boundary_edges(mesh, &component, edge_to_faces, flagged);

    let region = LongBridgeRegion::new(start, end, span)
        .with_edges(boundary_edges)
        .with_faces(component.clone());
    validation.long_bridges.push(region);

    let severity = classify_long_bridge_severity(span, config.max_bridge_span);
    let issue = PrintIssue::new(
        PrintIssueType::LongBridge,
        severity,
        format!(
            "{} face(s) span {:.1} mm (limit {:.1} mm, region area: {:.1} mm²)",
            component.len(),
            span,
            config.max_bridge_span,
            total_area
        ),
    )
    .with_location(center)
    .with_affected_elements(component);
    validation.issues.push(issue);
}

/// Detect long bridges via boundary-edge span analysis (§6.2 Gap G).
///
/// Preconditions: `config.technology.requires_supports()` must be true
/// (FDM/SLA). SLS/MJF skip **silently** — no `DetectorSkipped` issue
/// (bridges aren't applicable to powder-bed-supported processes per
/// §6.2 line 996).
///
/// Algorithm:
///
/// 1. Pre-check: `requires_supports()`; silent early return otherwise.
/// 2. Per face: classify as bridge candidate iff the outward normal
///    sits within 30° of `-up`. Build-plate filter (mesh-min-relative)
///    rejects faces on the plate.
/// 3. Cluster candidates by edge-adjacency
///    (`partition_flagged_into_components`).
/// 4. Per cluster: project vertices onto the plane perpendicular to
///    `up`, compute axis-aligned bbox; if `max(extent_u, extent_w) >
///    config.max_bridge_span`, emit a `LongBridgeRegion` (with start +
///    end at the longest-axis endpoints, lifted to 3D at the cluster
///    centroid's `up`-elevation) and a `LongBridge` `PrintIssue`.
///
/// **v0.8 limitations** (locked by §9.2.4 stress fixtures, v0.9
/// followups documented in `CHANGELOG.md`):
///
/// - Cantilever-as-bridge: a one-end-anchored horizontal face flags
///   even though it isn't a true bridge. v0.9: support-end analysis to
///   distinguish.
/// - Diagonal underflag: bbox is axis-aligned in the perpendicular
///   plane, so a 14×14 mm patch (true diagonal ≈ 19.8 mm) underflags
///   when `max_bridge_span = 15`. v0.9: OBB-based span.
fn check_long_bridges(
    mesh: &IndexedMesh,
    config: &PrinterConfig,
    validation: &mut PrintValidation,
) {
    if !config.technology.requires_supports() {
        return;
    }

    let up = config.build_up_direction;
    let flagged = flag_long_bridge_faces(mesh, up);
    if flagged.is_empty() {
        return;
    }

    let edge_to_faces = build_edge_to_faces(mesh);
    let components = partition_flagged_into_components(mesh, &flagged);
    let regions_before = validation.long_bridges.len();
    for component in components {
        emit_long_bridge_component(
            mesh,
            config,
            &edge_to_faces,
            &flagged,
            component,
            validation,
        );
    }

    // §4.4 ordering: sort `long_bridges` by `(start.x, start.y, start.z)`.
    // `f64::total_cmp` gives a total order on all `f64` (including
    // signed zero / NaN), avoiding the `partial_cmp` panic risk
    // flagged by `clippy::expect_used` and matching §4.4's
    // "deterministic across runs" promise. Sort range is restricted
    // to the regions emitted by this detector so a future hoist of
    // the sort to `validate_for_printing` doesn't have to undo
    // anything per-detector.
    validation.long_bridges[regions_before..].sort_by(|a, b| {
        a.start
            .x
            .total_cmp(&b.start.x)
            .then_with(|| a.start.y.total_cmp(&b.start.y))
            .then_with(|| a.start.z.total_cmp(&b.start.z))
    });
}

// ===== §6.3 Gap H — TrappedVolume detector via exterior flood-fill =======

/// Per-row scanline ray-origin offsets in y and z (mm).
///
/// Applied to the §6.3 voxel inside-test scanline ray's origin so no
/// row passes exactly through a vertex / edge. The y and z offsets
/// are intentionally **different** so the ray's `(y, z)` coordinates
/// never lie on a triangle's symmetry line: equal jitters fail on
/// axis-aligned cube fixtures whose face diagonals bisect the face
/// along `y = z` (or `y + z = const`), where the ray re-strikes the
/// shared diagonal edge of two triangles and the parity flips by 2
/// (no net effect — voxels never get marked `VOXEL_INSIDE`). The
/// 1.0e-5 / 1.7e-5 split breaks both `y = z` and `y + z = const`
/// coincidences; magnitudes still sit an order of magnitude above
/// `EPS_RAY_OFFSET` (1e-6 mm) so the per-row offset and the
/// `moller_trumbore` self-hit guard don't interact at threshold
/// boundaries. Per §6.3 line 1096 (with v0.8 implementation refinement
/// — the spec text "a single uniform jitter suffices" is incorrect for
/// axis-aligned cube fixtures; v0.9 spec edit will document the
/// asymmetric requirement).
const ROW_JITTER_Y: f64 = 1.0e-5;
const ROW_JITTER_Z: f64 = 1.7e-5;

/// Voxel-grid memory budget per §6.3 step 4.5 amendment (surfaced by
/// §9.2.5 risk-mitigation review): 1 byte per voxel (`u8` voxel state),
/// cap at 1 GB. Pathological inputs above this cap (e.g. a 250×200×220
/// mm full FDM build volume at 0.1 mm voxel — 11 GB) emit a
/// `DetectorSkipped` Info instead of allocating; mitigates the §8.4
/// Gap H grid-memory blowup risk.
const VOXEL_GRID_BYTE_CAP: u64 = 1_000_000_000;

/// Voxel state encoding for the `TrappedVolume` detector's grid:
/// 0 unknown, 1 inside-mesh, 2 exterior, 3 trapped, 4 trapped-visited.
const VOXEL_UNKNOWN: u8 = 0;
const VOXEL_INSIDE: u8 = 1;
const VOXEL_EXTERIOR: u8 = 2;
const VOXEL_TRAPPED: u8 = 3;
const VOXEL_TRAPPED_VISITED: u8 = 4;

/// Watertight check (§6.3 precondition; NO consistent-winding requirement).
///
/// Returns `true` iff no edge is incident on exactly one face (i.e. no
/// `open_edge_count` per `check_basic_manifold`'s pass). Distinct from
/// `is_watertight_and_consistent_winding`: the `TrappedVolume` detector
/// tolerates inconsistent winding (`§9.1` row 11), so the check is
/// open-edge-count-only.
fn is_watertight(mesh: &IndexedMesh) -> bool {
    let edge_to_faces = build_edge_to_faces(mesh);
    !edge_to_faces.values().any(|faces| faces.len() == 1)
}

/// Classify `TrappedVolume` severity per §6.3 (technology-aware).
///
/// `Info` if `volume < min_feature_size³` (below the printer's
/// resolution; not actionable). Otherwise `Critical` for SLA / SLS / MJF
/// (trapped uncured / unsintered material is a hard failure mode);
/// `Info` for FDM / Other (sealed cavities print fine on extrusion).
fn classify_trapped_volume_severity(
    volume: f64,
    tech: PrintTechnology,
    config: &PrinterConfig,
) -> IssueSeverity {
    let res_volume = config.min_feature_size.powi(3);
    if volume < res_volume {
        return IssueSeverity::Info;
    }
    match tech {
        PrintTechnology::Sla | PrintTechnology::Sls | PrintTechnology::Mjf => {
            IssueSeverity::Critical
        }
        PrintTechnology::Fdm | PrintTechnology::Other => IssueSeverity::Info,
    }
}

/// Voxel grid for the §6.3 `TrappedVolume` detector.
///
/// `dims` is `(nx, ny, nz)`. `origin` is the grid's min corner.
/// `voxel_size` is the per-axis voxel size in mm (isotropic).
/// `states` is `nx × ny × nz` bytes, x-major (linear index
/// `(z * ny + y) * nx + x`).
struct VoxelGrid {
    dims: (u32, u32, u32),
    origin: Point3<f64>,
    voxel_size: f64,
    states: Vec<u8>,
}

impl VoxelGrid {
    /// Linear index for `(x, y, z)`, x-major. Caller verifies bounds.
    const fn linear_index(&self, x: u32, y: u32, z: u32) -> usize {
        let nx = self.dims.0 as usize;
        let ny = self.dims.1 as usize;
        ((z as usize * ny) + y as usize) * nx + x as usize
    }

    /// Center of voxel `(x, y, z)` in mesh coordinates (mm).
    // FP-bit preserved: the explicit `(coord + 0.5) * voxel_size` form
    // keeps the parity-flip threshold deterministic across platforms.
    // `mul_add` would shift FP bits and break cross-os voxel inside-
    // test parity per §8.4 row 3.
    #[allow(clippy::suboptimal_flops)]
    fn voxel_center(&self, x: u32, y: u32, z: u32) -> Point3<f64> {
        Point3::new(
            self.origin.x + (f64::from(x) + 0.5) * self.voxel_size,
            self.origin.y + (f64::from(y) + 0.5) * self.voxel_size,
            self.origin.z + (f64::from(z) + 0.5) * self.voxel_size,
        )
    }
}

/// Mark inside voxels via per-row scanline + Möller-Trumbore parity
/// (§6.3 step 6).
///
/// For each `(y, z)` row, cast a `+X` ray from one voxel left of the grid
/// at `(grid_x_min − voxel_size, y_center + ROW_JITTER_Y, z_center +
/// ROW_JITTER_Z)`. Collect all `moller_trumbore` t-intersections, sort
/// by t. Sweep voxels left-to-right: at each voxel midpoint, if the
/// parity of crossings with `t < midpoint_t` is odd, mark voxel
/// `VOXEL_INSIDE`. Per row the cost is `O(n_faces + nx)`; per grid
/// `O(ny × nz × (n_faces + nx))`.
///
// FP-bit preserved on the `(coord + 0.5) * voxel_size` arithmetic: same
// FP-determinism rationale as `flag_overhang_faces`'s `len = sqrt(...)`
// site — `mul_add` would shift FP bits and break cross-os voxel
// parity per §8.4 row 3.
#[allow(clippy::suboptimal_flops)]
fn mark_inside_voxels(grid: &mut VoxelGrid, mesh: &IndexedMesh) {
    let (nx, ny, nz) = grid.dims;
    let v = grid.voxel_size;
    let origin_x = grid.origin.x;
    let origin_y = grid.origin.y;
    let origin_z = grid.origin.z;

    // +X ray direction (unit, axis-aligned for FP determinism).
    let direction = Vector3::new(1.0, 0.0, 0.0);

    for z in 0..nz {
        let z_center = origin_z + (f64::from(z) + 0.5) * v + ROW_JITTER_Z;
        for y in 0..ny {
            let y_center = origin_y + (f64::from(y) + 0.5) * v + ROW_JITTER_Y;
            // Ray origin one voxel-width outside the grid; voxel-midpoint
            // t-values then fall in `[v + 0.5 v, v + (nx − 0.5) v]` —
            // strictly positive, so `moller_trumbore`'s `t > EPS_GEOMETRIC`
            // guard never rejects a legitimate crossing.
            let row_origin = Point3::new(origin_x - v, y_center, z_center);

            let mut crossings: Vec<f64> = Vec::new();
            for face in &mesh.faces {
                let i0 = face[0] as usize;
                let i1 = face[1] as usize;
                let i2 = face[2] as usize;
                if i0 >= mesh.vertices.len()
                    || i1 >= mesh.vertices.len()
                    || i2 >= mesh.vertices.len()
                {
                    continue;
                }
                if let Some(t) = moller_trumbore(
                    row_origin,
                    direction,
                    mesh.vertices[i0],
                    mesh.vertices[i1],
                    mesh.vertices[i2],
                ) {
                    crossings.push(t);
                }
            }
            crossings.sort_by(f64::total_cmp);

            let mut inside = false;
            let mut next = 0_usize;
            for x in 0..nx {
                let voxel_mid_t = v + (f64::from(x) + 0.5) * v;
                while next < crossings.len() && crossings[next] < voxel_mid_t {
                    inside = !inside;
                    next += 1;
                }
                if inside {
                    let idx = grid.linear_index(x, y, z);
                    grid.states[idx] = VOXEL_INSIDE;
                }
            }
        }
    }
}

/// 6-connected BFS flood-fill from `seed`.
///
/// Visits every voxel reachable from `seed` whose current state matches
/// `from_state`; marks visited voxels with `to_state`. Uses an explicit
/// `VecDeque` queue (FIFO / BFS) — no recursion, so even an 8M-voxel
/// (200³) grid cannot stack-overflow.
fn flood_fill_6_connected(
    grid: &mut VoxelGrid,
    seed: (u32, u32, u32),
    from_state: u8,
    to_state: u8,
) {
    let (nx, ny, nz) = grid.dims;
    let seed_idx = grid.linear_index(seed.0, seed.1, seed.2);
    if grid.states[seed_idx] != from_state {
        return;
    }
    grid.states[seed_idx] = to_state;
    let mut queue: VecDeque<(u32, u32, u32)> = VecDeque::new();
    queue.push_back(seed);
    while let Some((x, y, z)) = queue.pop_front() {
        // 6-connected neighbors via i64 to handle 0 − 1 underflow safely.
        let neighbors: [(i64, i64, i64); 6] = [
            (i64::from(x) - 1, i64::from(y), i64::from(z)),
            (i64::from(x) + 1, i64::from(y), i64::from(z)),
            (i64::from(x), i64::from(y) - 1, i64::from(z)),
            (i64::from(x), i64::from(y) + 1, i64::from(z)),
            (i64::from(x), i64::from(y), i64::from(z) - 1),
            (i64::from(x), i64::from(y), i64::from(z) + 1),
        ];
        for (nxi, nyi, nzi) in neighbors {
            if nxi < 0 || nyi < 0 || nzi < 0 {
                continue;
            }
            // i64 → u32 cast is bounded: positive-checked above; upper
            // bound enforced by the next `>=` guard against grid dims.
            #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
            let neighbor = (nxi as u32, nyi as u32, nzi as u32);
            if neighbor.0 >= nx || neighbor.1 >= ny || neighbor.2 >= nz {
                continue;
            }
            let n_idx = grid.linear_index(neighbor.0, neighbor.1, neighbor.2);
            if grid.states[n_idx] != from_state {
                continue;
            }
            grid.states[n_idx] = to_state;
            queue.push_back(neighbor);
        }
    }
}

/// Connected-component label trapped voxels (state == `VOXEL_TRAPPED`)
/// via 6-connected BFS. Returns one entry per component.
///
/// Marks visited voxels as `VOXEL_TRAPPED_VISITED` (state 4) so the
/// outer triple-loop doesn't re-seed the same component. Component
/// scan order is x-major (`for z { for y { for x }}`), giving
/// deterministic component-discovery order; per-component voxel order
/// is BFS expansion order from the first-discovered seed.
fn trapped_components(grid: &mut VoxelGrid) -> Vec<Vec<(u32, u32, u32)>> {
    let mut components: Vec<Vec<(u32, u32, u32)>> = Vec::new();
    let (nx, ny, nz) = grid.dims;
    for z in 0..nz {
        for y in 0..ny {
            for x in 0..nx {
                let idx = grid.linear_index(x, y, z);
                if grid.states[idx] != VOXEL_TRAPPED {
                    continue;
                }
                let mut component: Vec<(u32, u32, u32)> = Vec::new();
                let mut queue: VecDeque<(u32, u32, u32)> = VecDeque::new();
                grid.states[idx] = VOXEL_TRAPPED_VISITED;
                queue.push_back((x, y, z));
                while let Some((cx, cy, cz)) = queue.pop_front() {
                    component.push((cx, cy, cz));
                    let neighbors: [(i64, i64, i64); 6] = [
                        (i64::from(cx) - 1, i64::from(cy), i64::from(cz)),
                        (i64::from(cx) + 1, i64::from(cy), i64::from(cz)),
                        (i64::from(cx), i64::from(cy) - 1, i64::from(cz)),
                        (i64::from(cx), i64::from(cy) + 1, i64::from(cz)),
                        (i64::from(cx), i64::from(cy), i64::from(cz) - 1),
                        (i64::from(cx), i64::from(cy), i64::from(cz) + 1),
                    ];
                    for (nxi, nyi, nzi) in neighbors {
                        if nxi < 0 || nyi < 0 || nzi < 0 {
                            continue;
                        }
                        // Same bounded-cast rationale as in `flood_fill_6_connected`.
                        #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
                        let nb = (nxi as u32, nyi as u32, nzi as u32);
                        if nb.0 >= nx || nb.1 >= ny || nb.2 >= nz {
                            continue;
                        }
                        let n_idx = grid.linear_index(nb.0, nb.1, nb.2);
                        if grid.states[n_idx] != VOXEL_TRAPPED {
                            continue;
                        }
                        grid.states[n_idx] = VOXEL_TRAPPED_VISITED;
                        queue.push_back(nb);
                    }
                }
                components.push(component);
            }
        }
    }
    components
}

/// Detect trapped volumes via voxel-based exterior flood-fill (§6.3 Gap H).
///
/// Preconditions: `is_watertight(mesh)` (no `open_edge_count`). On
/// failure: emit one `Info` `PrintIssue` of type `DetectorSkipped` and
/// return without populating `validation.trapped_volumes`. Inconsistent
/// winding is tolerated (`§9.1` row 11).
///
/// Algorithm:
///
/// 1. Watertight pre-check (open-edge count only).
/// 2. `voxel_size = min(min_feature_size, layer_height) / 2`.
/// 3. Mesh AABB padded by 2 voxel widths on every side; integer
///    `dims = ceil(extent / voxel_size)`.
/// 4. **Memory pre-flight (§6.3 step 4.5 / §9.2.5 amendment)**:
///    if `nx × ny × nz > VOXEL_GRID_BYTE_CAP` (1 GB at 1 byte / voxel),
///    emit `DetectorSkipped` Info and return; the grid is never
///    allocated. Mitigates §8.4 grid-memory blowup.
/// 5. Allocate `Vec<u8>` initialized to `VOXEL_UNKNOWN`; mark inside
///    voxels via per-row scanline + `moller_trumbore` parity (§6.3
///    step 6). FP-drift mitigated cross-platform by the per-row
///    `ROW_JITTER` offset + the §9.2.5
///    `stress_h_sphere_inside_cube_volume_within_10pct` 10 % volume
///    tolerance + the §10.4.1 cross-os CI matrix extension.
/// 6. Exterior flood-fill from grid corner `(0, 0, 0)` over
///    `VOXEL_UNKNOWN` voxels → mark `VOXEL_EXTERIOR`.
/// 7. Trap-label remaining `VOXEL_UNKNOWN` voxels → `VOXEL_TRAPPED`.
/// 8. Connected-component label trapped voxels via 6-connected BFS;
///    per component emit one `TrappedVolumeRegion` + one
///    `PrintIssue` (severity per `classify_trapped_volume_severity`).
/// 9. §4.4 sort `validation.trapped_volumes` by
///    `(center.x, center.y, center.z)` via `f64::total_cmp`.
///
/// **v0.8 limitations** (locked by §9.2.5 stress fixtures, v0.9
/// followups documented in `CHANGELOG.md`):
///
/// - Sub-voxel pinhole leaks: a cavity-to-exterior channel narrower
///   than `voxel_size` lets the flood-fill reach the cavity, so it is
///   not flagged as trapped. Documented intentional behavior for v0.8
///   (printer drainage capability is a separate concern). v0.9:
///   drainage-path simulation along `up` direction.
/// - Adaptive voxel sizing: parts much larger than `min_feature_size`
///   pay an `O((part_size / voxel_size)³)` memory bill. v0.9: per-region
///   adaptive voxel sizing.
//
// `clippy::too_many_lines`: the §6.3 algorithm is inherently 11-step
// (precondition → voxel sizing → memory pre-flight → grid alloc →
// inside-mark → exterior flood-fill → trap label → connected-component
// label → per-component emit → §4.4 sort), and helper extraction would
// cross algorithm phase boundaries (e.g., the per-component emit loop
// reads voxel-grid state, builds `TrappedVolumeRegion`, runs severity
// classifier, and pushes both region + issue — splitting it would
// require a 6-field tuple intermediate). Keeping the algorithm linear
// in one function preserves §6.3 step-numbered traceability.
#[allow(clippy::too_many_lines)]
fn check_trapped_volumes(
    mesh: &IndexedMesh,
    config: &PrinterConfig,
    validation: &mut PrintValidation,
) {
    if !is_watertight(mesh) {
        validation.issues.push(PrintIssue::new(
            PrintIssueType::DetectorSkipped,
            IssueSeverity::Info,
            "TrappedVolume detection requires watertight mesh (skipped)",
        ));
        return;
    }

    let voxel_size = config.min_feature_size.min(config.layer_height) / 2.0;
    if !voxel_size.is_finite() || voxel_size <= 0.0 {
        // Pathological config (zero / negative / NaN feature size). Tolerate
        // silently — `validate_for_printing`'s upstream `check_build_volume`
        // is the authoritative handler for nonsensical configs.
        return;
    }

    let (mesh_min, mesh_max) = compute_bounds(mesh);
    let pad = 2.0 * voxel_size;
    let grid_min = Point3::new(mesh_min.x - pad, mesh_min.y - pad, mesh_min.z - pad);
    let grid_max = Point3::new(mesh_max.x + pad, mesh_max.y + pad, mesh_max.z + pad);
    let extent_x = grid_max.x - grid_min.x;
    let extent_y = grid_max.y - grid_min.y;
    let extent_z = grid_max.z - grid_min.z;

    // Per-axis dim: `ceil(extent / voxel_size).max(1)`. f64 → u64 saturates
    // NaN to 0 and Inf to u64::MAX (Rust 1.45+ semantics); the subsequent
    // `checked_mul` step 4 detects the latter and skips.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let nx = (extent_x / voxel_size).ceil().max(1.0) as u64;
    // Same f64 → u64 saturation rationale as `nx`.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let ny = (extent_y / voxel_size).ceil().max(1.0) as u64;
    // Same f64 → u64 saturation rationale as `nx`.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let nz = (extent_z / voxel_size).ceil().max(1.0) as u64;

    // Step 4.5 — §9.2.5 memory pre-flight amendment.
    let total_bytes = nx.checked_mul(ny).and_then(|p| p.checked_mul(nz));
    let Some(total_bytes) = total_bytes else {
        validation.issues.push(PrintIssue::new(
            PrintIssueType::DetectorSkipped,
            IssueSeverity::Info,
            format!(
                "TrappedVolume detection skipped: voxel grid {nx}×{ny}×{nz} at {voxel_size:.4} mm voxel would exceed 1 GB memory budget (u64 overflow)"
            ),
        ));
        return;
    };
    if total_bytes > VOXEL_GRID_BYTE_CAP {
        validation.issues.push(PrintIssue::new(
            PrintIssueType::DetectorSkipped,
            IssueSeverity::Info,
            format!(
                "TrappedVolume detection skipped: voxel grid {nx}×{ny}×{nz} at {voxel_size:.4} mm voxel would exceed 1 GB memory budget"
            ),
        ));
        return;
    }

    // Past the cap → each dim ≤ total_bytes ≤ 1e9, fits in u32 (u32::MAX ≈ 4.3e9).
    #[allow(clippy::cast_possible_truncation)]
    let dims = (nx as u32, ny as u32, nz as u32);
    // total_bytes ≤ 1e9 fits in usize on every 32-bit-or-larger target.
    #[allow(clippy::cast_possible_truncation)]
    let total_voxels_usize = total_bytes as usize;

    let mut grid = VoxelGrid {
        dims,
        origin: grid_min,
        voxel_size,
        states: vec![VOXEL_UNKNOWN; total_voxels_usize],
    };

    mark_inside_voxels(&mut grid, mesh);
    flood_fill_6_connected(&mut grid, (0, 0, 0), VOXEL_UNKNOWN, VOXEL_EXTERIOR);
    for state in &mut grid.states {
        if *state == VOXEL_UNKNOWN {
            *state = VOXEL_TRAPPED;
        }
    }

    let regions_before = validation.trapped_volumes.len();
    let voxel_volume = voxel_size.powi(3);
    let half_voxel = voxel_size / 2.0;
    let components = trapped_components(&mut grid);
    for component in components {
        let voxel_count = component.len();
        // `component.len()` is bounded by `total_voxels_usize ≤ 1e9 < u32::MAX`.
        #[allow(clippy::cast_possible_truncation)]
        let voxel_count_u32 = voxel_count as u32;

        let mut sum_x = 0.0_f64;
        let mut sum_y = 0.0_f64;
        let mut sum_z = 0.0_f64;
        let mut bbox_x = (f64::INFINITY, f64::NEG_INFINITY);
        let mut bbox_y = (f64::INFINITY, f64::NEG_INFINITY);
        let mut bbox_z = (f64::INFINITY, f64::NEG_INFINITY);
        for &(vx, vy, vz) in &component {
            let c = grid.voxel_center(vx, vy, vz);
            sum_x += c.x;
            sum_y += c.y;
            sum_z += c.z;
            bbox_x = (bbox_x.0.min(c.x), bbox_x.1.max(c.x));
            bbox_y = (bbox_y.0.min(c.y), bbox_y.1.max(c.y));
            bbox_z = (bbox_z.0.min(c.z), bbox_z.1.max(c.z));
        }
        // `voxel_count` ≤ 1e9 < 2^53; usize → f64 is exact.
        #[allow(clippy::cast_precision_loss)]
        let n = voxel_count as f64;
        let center = Point3::new(sum_x / n, sum_y / n, sum_z / n);
        let bbox_min = Point3::new(
            bbox_x.0 - half_voxel,
            bbox_y.0 - half_voxel,
            bbox_z.0 - half_voxel,
        );
        let bbox_max = Point3::new(
            bbox_x.1 + half_voxel,
            bbox_y.1 + half_voxel,
            bbox_z.1 + half_voxel,
        );
        // Same precision rationale.
        #[allow(clippy::cast_precision_loss)]
        let volume = (voxel_count as f64) * voxel_volume;

        let region =
            TrappedVolumeRegion::new(center, volume, (bbox_min, bbox_max), voxel_count_u32);
        validation.trapped_volumes.push(region);

        let severity = classify_trapped_volume_severity(volume, config.technology, config);
        let issue = PrintIssue::new(
            PrintIssueType::TrappedVolume,
            severity,
            format!(
                "{voxel_count} trapped voxel(s); volume {volume:.2} mm³ at voxel size {voxel_size:.3} mm"
            ),
        )
        .with_location(center);
        validation.issues.push(issue);
    }

    // §4.4 sort: `(center.x, center.y, center.z)` via `f64::total_cmp`.
    // Range-restricted to this detector's emissions so a future hoist
    // of the sort to `validate_for_printing` doesn't have to undo it
    // per-detector. Same precedent as `check_long_bridges` at
    // `validation.rs:1335`.
    validation.trapped_volumes[regions_before..].sort_by(|a, b| {
        a.center
            .x
            .total_cmp(&b.center.x)
            .then_with(|| a.center.y.total_cmp(&b.center.y))
            .then_with(|| a.center.z.total_cmp(&b.center.z))
    });
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

    // §5.5 Gap F — directed-edge winding-orientation cluster.
    //
    // `check_basic_manifold` runs two side-by-side passes: the undirected
    // edge-sharing pass (existing v0.7 logic) and the directed-edge pass
    // (Gap F). Both push under `PrintIssueType::NonManifold` with distinct
    // descriptions, so these tests discriminate the new winding-inconsistency
    // signal by string-matching `"winding inconsistency"` in the issue
    // description — the load-bearing anchor convention from §5.5.
    //
    // Fixture polarity:
    // - `test_winding_inconsistent_two_same_direction_faces` is the positive
    //   case (issue must fire).
    // - `test_winding_consistent_watertight_cube` is the regression-guard
    //   that pre-flight verified `create_watertight_cube` has consistent
    //   CCW-from-outside winding (§8.1 Gap F risk row 1 mitigation).
    // - `test_winding_consistent_disjoint_faces` confirms the detector
    //   doesn't false-positive on faces that share only a vertex.
    // - `test_winding_inconsistent_does_not_break_open_edge_check` verifies
    //   the directed-edge pass doesn't perturb the existing open-edge
    //   detector when winding is consistent.

    #[test]
    fn test_winding_inconsistent_two_same_direction_faces() {
        // Two triangles sharing edge (0,1), both traversing it in the same
        // direction: face [0,1,2] has directed edge (0,1); face [0,1,3]
        // also has directed edge (0,1). One face is "inside out" relative
        // to the other — Gap F's positive case.
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ];
        let faces = vec![[0, 1, 2], [0, 1, 3]];
        let mesh = IndexedMesh::from_parts(vertices, faces);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the same-direction-faces fixture");

        assert!(
            result
                .issues
                .iter()
                .any(|i| i.issue_type == PrintIssueType::NonManifold
                    && i.description.contains("winding inconsistency")),
            "directed-edge pass must flag NonManifold Critical with 'winding inconsistency' description on a same-direction shared edge"
        );
    }

    #[test]
    fn test_winding_consistent_watertight_cube() {
        // Regression guard: the existing `create_watertight_cube` fixture
        // is wound CCW-from-outside (verified by directed-edge inspection
        // in §8.1 Gap F risk row 1 pre-flight). The directed pass must NOT
        // emit a winding-inconsistency issue on it. If this test fails,
        // the cube fixture has a silent winding defect that v0.7 missed —
        // surface as plan-contradicting evidence.
        let mesh = create_watertight_cube();
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the watertight cube");

        assert!(
            !result
                .issues
                .iter()
                .any(|i| i.issue_type == PrintIssueType::NonManifold
                    && i.description.contains("winding inconsistency")),
            "watertight cube has consistent CCW-from-outside winding; no winding-inconsistency issue should fire"
        );
    }

    #[test]
    fn test_winding_consistent_disjoint_faces() {
        // Two triangles sharing only a single vertex (vertex 2): no shared
        // edge exists, so no directed-edge collision is possible. The
        // detector must not false-positive on vertex-only adjacency.
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 1.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
        ];
        let faces = vec![[0, 1, 2], [2, 3, 4]];
        let mesh = IndexedMesh::from_parts(vertices, faces);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the disjoint-faces fixture");

        assert!(
            !result
                .issues
                .iter()
                .any(|i| i.issue_type == PrintIssueType::NonManifold
                    && i.description.contains("winding inconsistency")),
            "two faces sharing only a vertex have no shared edge; no winding-inconsistency issue should fire"
        );
    }

    // -- Gap L (§5.6): build_up_direction parametrization --------------------

    /// Rotate a mesh -90° around the +X axis. Maps each vertex
    /// `(x, y, z)` to `(x, z, -y)`, sending the mesh's `+Z` axis to
    /// world `+Y`. Used to exercise the `+Y` build-up direction
    /// symmetric with the default `+Z` configuration.
    fn rotate_neg_90_about_x(mesh: &IndexedMesh) -> IndexedMesh {
        let vertices: Vec<Point3<f64>> = mesh
            .vertices
            .iter()
            .map(|v| Point3::new(v.x, v.z, -v.y))
            .collect();
        let faces = mesh.faces.clone();
        IndexedMesh::from_parts(vertices, faces)
    }

    #[test]
    fn test_overhang_with_y_up_orientation() {
        // Equivalence: the same overhang fixture flagged under +Z up
        // must flag the same number of regions under +Y up after the
        // mesh is rotated -90° around +X (which sends mesh +Z → world +Y).
        // Both branches dispatch through `check_overhangs`'s build-up
        // direction read, so any failure here means propagation broke.
        let mesh_z = make_overhang_fixture(std::f64::consts::FRAC_PI_2, 5.0);
        let mesh_y = rotate_neg_90_about_x(&mesh_z);

        let config_z = PrinterConfig::fdm_default();
        let config_y =
            PrinterConfig::fdm_default().with_build_up_direction(Vector3::new(0.0, 1.0, 0.0));

        #[allow(clippy::expect_used)]
        let result_z = validate_for_printing(&mesh_z, &config_z)
            .expect("validation should succeed for +Z-up roof fixture");
        #[allow(clippy::expect_used)]
        let result_y = validate_for_printing(&mesh_y, &config_y)
            .expect("validation should succeed for +Y-up rotated roof fixture");

        assert_eq!(result_z.overhangs.len(), 1, "+Z up should flag the roof");
        assert_eq!(
            result_z.overhangs.len(),
            result_y.overhangs.len(),
            "overhang count must be symmetric across +Z and +Y build-up directions"
        );
    }

    #[test]
    fn test_overhang_with_oblique_up() {
        // Builder normalizes a non-axis-aligned input. (0, 1, 1) →
        // (0, 1/√2, 1/√2). The validator must accept any unit vector
        // as the build-up direction; the dot-product math is fully
        // direction-agnostic.
        let config =
            PrinterConfig::fdm_default().with_build_up_direction(Vector3::new(0.0, 1.0, 1.0));
        let s = std::f64::consts::FRAC_1_SQRT_2;
        assert!((config.build_up_direction.x - 0.0).abs() < f64::EPSILON);
        assert!((config.build_up_direction.y - s).abs() < f64::EPSILON);
        assert!((config.build_up_direction.z - s).abs() < f64::EPSILON);

        // Validation should run without panic on a fixture under the
        // oblique up vector (no NaN / Inf escapes).
        let mesh = make_overhang_fixture(std::f64::consts::FRAC_PI_2, 5.0);
        #[allow(clippy::expect_used)]
        let _ = validate_for_printing(&mesh, &config)
            .expect("validation should succeed with oblique build-up direction");
    }

    #[test]
    fn test_winding_inconsistent_does_not_break_open_edge_check() {
        // `create_cube_mesh` is an incomplete cube (bottom + top only)
        // with consistent winding within each face. The directed-edge
        // pass must not perturb the undirected open-edge check: result
        // still reports `NotWatertight` Critical, and no winding-
        // inconsistency issue fires.
        let mesh = create_cube_mesh();
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the incomplete-cube fixture");

        assert!(
            result
                .issues
                .iter()
                .any(|i| i.issue_type == PrintIssueType::NotWatertight),
            "incomplete cube must still flag NotWatertight (open edges)"
        );
        assert!(
            !result
                .issues
                .iter()
                .any(|i| i.issue_type == PrintIssueType::NonManifold
                    && i.description.contains("winding inconsistency")),
            "incomplete cube has consistent winding; no winding-inconsistency issue should fire"
        );
    }

    // -- §6.1: ThinWall detector ---------------------------------------

    /// Build a closed 10×10×`thickness` cuboid centred on the +XY quadrant
    /// at z ∈ [0, thickness], wound CCW-from-outside. Watertight +
    /// consistently wound; the §6.1 `ThinWall` detector preconditions
    /// hold. The cuboid's top + bottom faces oppose each other at
    /// distance `thickness` along the build-up axis; the four side faces
    /// oppose each other at distance 10 mm in the orthogonal axes.
    fn make_thin_slab(thickness: f64) -> IndexedMesh {
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(10.0, 10.0, 0.0),
            Point3::new(0.0, 10.0, 0.0),
            Point3::new(0.0, 0.0, thickness),
            Point3::new(10.0, 0.0, thickness),
            Point3::new(10.0, 10.0, thickness),
            Point3::new(0.0, 10.0, thickness),
        ];
        let faces = vec![
            [0, 2, 1],
            [0, 3, 2], // Bottom (-z normal)
            [4, 5, 6],
            [4, 6, 7], // Top (+z normal)
            [0, 1, 5],
            [0, 5, 4], // Front (-y)
            [3, 6, 2],
            [3, 7, 6], // Back (+y)
            [0, 4, 7],
            [0, 7, 3], // Left (-x)
            [1, 2, 6],
            [1, 6, 5], // Right (+x)
        ];
        IndexedMesh::from_parts(vertices, faces)
    }

    #[test]
    fn test_thin_wall_detected_on_thin_slab() {
        // 10×10×0.4 mm slab, FDM `min_wall_thickness = 1.0`. The top and
        // bottom faces oppose each other across 0.4 mm; their inward
        // rays hit the opposite face at 0.4 mm — flagged. Side faces
        // oppose at 10 mm — not flagged. By edge-adjacency the top and
        // bottom triangles cluster *separately* (closed-shell topology:
        // top tris share no edge with bottom tris), so the assertion is
        // exactly 2 clusters, both reporting thickness ≈ 0.4 mm. Same
        // topological pattern as the §7.1 hollow box (2 clusters per
        // pre-flight hand-trace).
        let mesh = make_thin_slab(0.4);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the thin-slab fixture");

        assert_eq!(
            result.thin_walls.len(),
            2,
            "thin slab: top + bottom flagged faces cluster into 2 disjoint regions"
        );
        for region in &result.thin_walls {
            assert!(
                (region.thickness - 0.4).abs() < 1e-5,
                "expected thickness ≈ 0.4 mm, got {}",
                region.thickness
            );
        }
        let critical_thin = result
            .issues
            .iter()
            .filter(|i| {
                i.issue_type == PrintIssueType::ThinWall && i.severity == IssueSeverity::Critical
            })
            .count();
        assert_eq!(
            critical_thin, 2,
            "0.4 < 1.0/2 = 0.5 → both clusters Critical"
        );
    }

    #[test]
    fn test_thin_wall_no_issue_on_thick_cube() {
        // A 10 mm watertight cube under FDM `min_wall_thickness = 1.0`:
        // every face's inward ray hits the opposite face at 10 mm,
        // 10 ≫ 1.0 → no `ThinWall` regions. Verifies the detector
        // doesn't false-flag effectively-solid geometry.
        let mesh = create_watertight_cube();
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the watertight-cube fixture");

        assert_eq!(
            result.thin_walls.len(),
            0,
            "10 mm walls under min_wall=1.0 must not flag"
        );
    }

    #[test]
    fn test_thin_wall_severity_critical_at_quarter_min() {
        // 0.25 mm slab, min_wall=1.0 → 0.25 < 1.0/2 = 0.5 → Critical.
        let mesh = make_thin_slab(0.25);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the 0.25 mm slab");

        assert!(!result.thin_walls.is_empty(), "0.25 mm slab must flag");
        let any_critical = result.issues.iter().any(|i| {
            i.issue_type == PrintIssueType::ThinWall && i.severity == IssueSeverity::Critical
        });
        assert!(any_critical, "0.25 < 0.5 must classify as Critical");
    }

    #[test]
    fn test_thin_wall_severity_warning_at_three_quarter_min() {
        // 0.75 mm slab, min_wall=1.0 → 0.5 ≤ 0.75 < 1.0 → Warning.
        let mesh = make_thin_slab(0.75);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the 0.75 mm slab");

        assert!(!result.thin_walls.is_empty(), "0.75 mm slab must flag");
        let any_warning = result.issues.iter().any(|i| {
            i.issue_type == PrintIssueType::ThinWall && i.severity == IssueSeverity::Warning
        });
        let any_critical = result.issues.iter().any(|i| {
            i.issue_type == PrintIssueType::ThinWall && i.severity == IssueSeverity::Critical
        });
        assert!(any_warning, "0.75 ≥ 0.5 must classify as Warning");
        assert!(!any_critical, "0.75 ≥ 0.5 must NOT classify as Critical");
    }

    #[test]
    fn test_thin_wall_borderline_no_issue() {
        // Strictly-less predicate (`min_dist < min_wall_thickness`):
        // 1.0 mm slab under min_wall=1.0 must NOT flag.
        let mesh = make_thin_slab(1.0);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the borderline 1.0 mm slab");

        assert_eq!(
            result.thin_walls.len(),
            0,
            "1.0 mm under min_wall=1.0 must not flag (strict-less predicate)"
        );
    }

    #[test]
    fn test_thin_wall_skipped_on_open_mesh() {
        // Open mesh (top of slab removed → 4 open edges along the top
        // perimeter): watertight precondition fails → `DetectorSkipped`
        // emitted, no `ThinWallRegion`s populated.
        let mut mesh = make_thin_slab(0.4);
        // Drop the top tris (faces 2 + 3 in the slab face list). Drain
        // and rebuild the face list omitting indices 2..4.
        let faces: Vec<[u32; 3]> = mesh
            .faces
            .iter()
            .enumerate()
            .filter(|(idx, _)| *idx != 2 && *idx != 3)
            .map(|(_, f)| *f)
            .collect();
        mesh = IndexedMesh::from_parts(mesh.vertices, faces);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the open-mesh fixture");

        let any_skipped = result.issues.iter().any(|i| {
            i.issue_type == PrintIssueType::DetectorSkipped
                && i.description.contains("ThinWall")
                && i.description.contains("watertight")
        });
        assert!(
            any_skipped,
            "open mesh must emit DetectorSkipped with ThinWall + watertight in description"
        );
        assert_eq!(
            result.thin_walls.len(),
            0,
            "open mesh must not populate thin_walls"
        );
    }

    #[test]
    fn test_thin_wall_skipped_on_inconsistent_winding() {
        // Slab with one top tri's vertex order flipped → directed edge
        // (4, 6) appears in BOTH the flipped face and the unflipped
        // sister face → consistent-winding precondition fails →
        // `DetectorSkipped` emitted.
        let mesh = make_thin_slab(0.4);
        // Flip face index 2 from [4, 5, 6] to [4, 6, 5].
        let mut faces: Vec<[u32; 3]> = mesh.faces.clone();
        faces[2] = [4, 6, 5];
        let mesh = IndexedMesh::from_parts(mesh.vertices, faces);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the flipped-winding fixture");

        let any_skipped = result.issues.iter().any(|i| {
            i.issue_type == PrintIssueType::DetectorSkipped && i.description.contains("ThinWall")
        });
        assert!(
            any_skipped,
            "inconsistent winding must emit DetectorSkipped from ThinWall"
        );
        assert_eq!(
            result.thin_walls.len(),
            0,
            "winding-inconsistent mesh must not populate thin_walls"
        );
    }

    #[test]
    fn test_thin_wall_two_disjoint_clusters() {
        // Two slabs offset along +X by 30 mm: each slab independently
        // produces 2 clusters (top + bottom topologically disjoint per
        // closed-shell hand-trace). Total: 4 clusters across the two
        // disjoint components.
        let slab_a = make_thin_slab(0.4);
        let mut vertices: Vec<Point3<f64>> = slab_a.vertices.clone();
        let mut faces: Vec<[u32; 3]> = slab_a.faces.clone();

        let offset_x = 30.0;
        // Hand-built fixture: an `expect` failure here would indicate
        // the slab helper itself was malformed, not a detector regression.
        #[allow(clippy::expect_used)]
        let base = u32::try_from(slab_a.vertices.len()).expect("slab vertex count fits in u32");
        for v in &slab_a.vertices {
            vertices.push(Point3::new(v.x + offset_x, v.y, v.z));
        }
        for f in &slab_a.faces {
            faces.push([f[0] + base, f[1] + base, f[2] + base]);
        }
        let mesh = IndexedMesh::from_parts(vertices, faces);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for the two-slab fixture");

        assert_eq!(
            result.thin_walls.len(),
            4,
            "two disjoint slabs each contribute 2 clusters → 4 total"
        );
        for region in &result.thin_walls {
            assert!(
                (region.thickness - 0.4).abs() < 1e-5,
                "all clusters report thickness ≈ 0.4 mm"
            );
        }
    }

    #[test]
    fn test_thin_wall_sort_stable_across_runs() {
        // Same input, two runs: cluster ordering matches deterministically.
        // Verifies §4.4 sort stability — `partition_flagged_into_components`
        // emerges in min-face-idx order regardless of `HashMap` iteration
        // permutation.
        let mesh = make_thin_slab(0.4);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let r1 = validate_for_printing(&mesh, &config).expect("validation 1");
        #[allow(clippy::expect_used)]
        let r2 = validate_for_printing(&mesh, &config).expect("validation 2");

        assert_eq!(r1.thin_walls.len(), r2.thin_walls.len());
        for (a, b) in r1.thin_walls.iter().zip(r2.thin_walls.iter()) {
            assert_eq!(a.faces, b.faces, "cluster face order must match");
            assert!(
                (a.thickness - b.thickness).abs() < f64::EPSILON,
                "cluster thickness must match bit-for-bit"
            );
        }
    }

    // ---- §6.2 Gap G LongBridge tests --------------------------------------
    //
    // Closed-mesh bridge fixtures: a tiny ground-anchor cuboid at `z=0`
    // plus an elevated slab. Two disjoint cuboids are jointly watertight
    // + consistently wound (each component closes on its own; vertices
    // are disjoint so no edge crosses components). The slab's bottom
    // face is the bridge candidate; the anchor pins `mesh_min_along_up = 0`
    // so the slab's bottom isn't filtered. Slab thickness ≥ 1 mm to
    // avoid co-flagging as `ThinWall` under default FDM
    // `min_wall_thickness = 1.0`.

    /// Append a closed cuboid (CCW-from-outside) to `(vertices, faces)`.
    /// `min`/`max` are diagonally-opposite corners; six rectangular
    /// faces / twelve triangles. Mirrors the winding of
    /// `cube_vertices_and_faces` in `tests/stress_inputs.rs`.
    fn append_closed_cuboid(
        vertices: &mut Vec<Point3<f64>>,
        faces: &mut Vec<[u32; 3]>,
        min: Point3<f64>,
        max: Point3<f64>,
    ) {
        // `vertices.len()` is bounded by the test fixtures' small tri
        // counts (≤ 100); usize → u32 is safe at this scale.
        #[allow(clippy::cast_possible_truncation)]
        let base: u32 = vertices.len() as u32;
        vertices.extend_from_slice(&[
            Point3::new(min.x, min.y, min.z),
            Point3::new(max.x, min.y, min.z),
            Point3::new(max.x, max.y, min.z),
            Point3::new(min.x, max.y, min.z),
            Point3::new(min.x, min.y, max.z),
            Point3::new(max.x, min.y, max.z),
            Point3::new(max.x, max.y, max.z),
            Point3::new(min.x, max.y, max.z),
        ]);
        let cf = |a: u32, b: u32, c: u32| [base + a, base + b, base + c];
        faces.extend_from_slice(&[
            cf(0, 2, 1),
            cf(0, 3, 2),
            cf(4, 5, 6),
            cf(4, 6, 7),
            cf(0, 1, 5),
            cf(0, 5, 4),
            cf(3, 6, 2),
            cf(3, 7, 6),
            cf(0, 4, 7),
            cf(0, 7, 3),
            cf(1, 2, 6),
            cf(1, 6, 5),
        ]);
    }

    /// Build a closed bridge fixture: a 2×2×2 anchor cuboid at
    /// `z ∈ [0, 2]` (sets `mesh_min_along_up = 0`) plus a slab of
    /// `x_extent × y_extent × slab_thickness` at `z ∈ [elevation,
    /// elevation + slab_thickness]`. The slab is x-offset by `+10` so
    /// its vertices are disjoint from the anchor's.
    fn make_closed_bridge_fixture(
        x_extent: f64,
        y_extent: f64,
        slab_thickness: f64,
        elevation: f64,
    ) -> IndexedMesh {
        let mut vertices: Vec<Point3<f64>> = Vec::new();
        let mut faces: Vec<[u32; 3]> = Vec::new();
        // Anchor at (-3..-1, -3..-1, 0..2) — x/y-offset to avoid overlap
        // with the slab's bbox.
        append_closed_cuboid(
            &mut vertices,
            &mut faces,
            Point3::new(-3.0, -3.0, 0.0),
            Point3::new(-1.0, -1.0, 2.0),
        );
        // Slab at (0..x_extent, 0..y_extent, elevation..elevation+thickness).
        append_closed_cuboid(
            &mut vertices,
            &mut faces,
            Point3::new(0.0, 0.0, elevation),
            Point3::new(x_extent, y_extent, elevation + slab_thickness),
        );
        IndexedMesh::from_parts(vertices, faces)
    }

    #[test]
    fn test_long_bridge_horizontal_slab_exceeds_span() {
        // 20×5 slab at z=10, max_bridge_span = 10 (FDM default).
        // span = max(20, 5) = 20; flagged at Critical (20 > 10*1.5).
        let mesh = make_closed_bridge_fixture(20.0, 5.0, 1.5, 10.0);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

        assert_eq!(
            result.long_bridges.len(),
            1,
            "20×5 slab elevated above plate must produce 1 LongBridge region"
        );
        let region = &result.long_bridges[0];
        assert!(
            (region.span - 20.0).abs() < 1e-6,
            "span must equal the bbox max-extent (20 mm); got {}",
            region.span
        );
        // Issue exists at LongBridge type with Critical severity.
        let critical = result
            .issues
            .iter()
            .filter(|i| {
                i.issue_type == PrintIssueType::LongBridge && i.severity == IssueSeverity::Critical
            })
            .count();
        assert_eq!(critical, 1, "20 mm > 10*1.5 → Critical");
    }

    #[test]
    fn test_long_bridge_short_span_no_issue() {
        // 5×5 slab, max_bridge_span = 10. span = 5 ≤ 10 → no flag.
        let mesh = make_closed_bridge_fixture(5.0, 5.0, 1.5, 10.0);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

        assert_eq!(
            result.long_bridges.len(),
            0,
            "5×5 slab span (5) ≤ max_bridge_span (10); no LongBridge region"
        );
    }

    #[test]
    fn test_long_bridge_borderline_warning() {
        // 12×5 slab, max=10. span = 12; 10 < 12 ≤ 10*1.5 = 15 → Warning.
        let mesh = make_closed_bridge_fixture(12.0, 5.0, 1.5, 10.0);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

        assert_eq!(result.long_bridges.len(), 1);
        let warning = result
            .issues
            .iter()
            .filter(|i| {
                i.issue_type == PrintIssueType::LongBridge && i.severity == IssueSeverity::Warning
            })
            .count();
        assert_eq!(warning, 1, "span 12 in (10, 15] → Warning");
    }

    #[test]
    fn test_long_bridge_well_above_critical() {
        // 20×5 slab, max=10. 20 > 10*1.5 = 15 → Critical (boundary
        // bracketed by 5 mm above the Warning/Critical edge).
        let mesh = make_closed_bridge_fixture(20.0, 5.0, 1.5, 10.0);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

        assert_eq!(result.long_bridges.len(), 1);
        let critical = result
            .issues
            .iter()
            .filter(|i| {
                i.issue_type == PrintIssueType::LongBridge && i.severity == IssueSeverity::Critical
            })
            .count();
        assert_eq!(critical, 1, "span 20 > 10*1.5 → Critical");
    }

    #[test]
    fn test_long_bridge_skipped_for_sls() {
        // SLS config: `requires_supports() == false` → silent skip.
        // No `LongBridge` regions, no `DetectorSkipped` issue (per §6.2
        // line 996, distinct from `ThinWall`'s `DetectorSkipped` on
        // non-watertight).
        let mesh = make_closed_bridge_fixture(20.0, 5.0, 1.5, 10.0);
        let config = PrinterConfig::sls_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

        assert_eq!(result.long_bridges.len(), 0, "SLS skips bridges silently");
        let skipped_naming_bridge = result.issues.iter().any(|i| {
            i.issue_type == PrintIssueType::DetectorSkipped
                && i.description.to_lowercase().contains("bridge")
        });
        assert!(
            !skipped_naming_bridge,
            "SLS skip is silent — no DetectorSkipped issue naming bridges"
        );
    }

    #[test]
    fn test_long_bridge_bottom_face_not_flagged() {
        // 20×5 slab at z=0 (sitting on build plate). The slab's bottom
        // face has `face_min = 0 = mesh_min` → build-plate filter
        // excludes; no bridge. Single closed cuboid, no anchor needed
        // (its own bottom is mesh-min).
        let mut vertices: Vec<Point3<f64>> = Vec::new();
        let mut faces: Vec<[u32; 3]> = Vec::new();
        append_closed_cuboid(
            &mut vertices,
            &mut faces,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(20.0, 5.0, 1.5),
        );
        let mesh = IndexedMesh::from_parts(vertices, faces);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

        assert_eq!(
            result.long_bridges.len(),
            0,
            "slab on build plate: bottom filtered, no LongBridge"
        );
    }

    #[test]
    fn test_long_bridge_two_disjoint_bridges() {
        // Two parallel 20×5 slabs separated by 30 mm along +X plus a
        // single ground anchor. Three disjoint closed cuboids; each
        // slab's bottom is its own cluster. Two LongBridge regions.
        let mut vertices: Vec<Point3<f64>> = Vec::new();
        let mut faces: Vec<[u32; 3]> = Vec::new();
        append_closed_cuboid(
            &mut vertices,
            &mut faces,
            Point3::new(-3.0, -3.0, 0.0),
            Point3::new(-1.0, -1.0, 2.0),
        );
        append_closed_cuboid(
            &mut vertices,
            &mut faces,
            Point3::new(0.0, 0.0, 10.0),
            Point3::new(20.0, 5.0, 11.5),
        );
        append_closed_cuboid(
            &mut vertices,
            &mut faces,
            Point3::new(30.0, 0.0, 10.0),
            Point3::new(50.0, 5.0, 11.5),
        );
        let mesh = IndexedMesh::from_parts(vertices, faces);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

        assert_eq!(
            result.long_bridges.len(),
            2,
            "two disjoint slab bottoms → 2 LongBridge regions"
        );
        for region in &result.long_bridges {
            assert!(
                (region.span - 20.0).abs() < 1e-6,
                "each slab spans 20 mm; got {}",
                region.span
            );
        }
    }

    #[test]
    fn test_long_bridge_with_y_up_orientation() {
        // Same 20×5 slab geometry, rotated 90° around +X (y → z, z → -y),
        // validated with `+Y up`. The bridge face's outward normal in
        // the rotated frame is (0, -1, 0) = -up; flagged identically.
        // Anchor at (-3..-1, 0..2, 1..3) (y plays z-role).
        let mut vertices: Vec<Point3<f64>> = Vec::new();
        let mut faces: Vec<[u32; 3]> = Vec::new();
        // Anchor in the y-up frame: y ∈ [0, 2] is "elevation 0..2", and
        // x/z extents define the footprint.
        append_closed_cuboid(
            &mut vertices,
            &mut faces,
            Point3::new(-3.0, 0.0, -3.0),
            Point3::new(-1.0, 2.0, -1.0),
        );
        // Slab in the y-up frame: y ∈ [10, 11.5] (1.5 mm thick), x ∈
        // [0, 20], z ∈ [0, 5]. Span = max(20, 5) = 20 in the
        // (e1, e2) = ((1,0,0), (0,0,-1)) basis.
        append_closed_cuboid(
            &mut vertices,
            &mut faces,
            Point3::new(0.0, 10.0, 0.0),
            Point3::new(20.0, 11.5, 5.0),
        );
        let mesh = IndexedMesh::from_parts(vertices, faces);
        let config =
            PrinterConfig::fdm_default().with_build_up_direction(Vector3::new(0.0, 1.0, 0.0));

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

        assert_eq!(
            result.long_bridges.len(),
            1,
            "+Y up rotation produces same flag count as +Z up"
        );
        let region = &result.long_bridges[0];
        assert!(
            (region.span - 20.0).abs() < 1e-6,
            "span identical to +Z fixture; got {}",
            region.span
        );
    }

    #[test]
    fn test_long_bridge_cantilever_currently_flagged() {
        // 20-mm cantilever face anchored on one end. v0.8 cannot
        // distinguish a cantilever from a bridge: both produce a single
        // cluster of horizontal-down faces above the build plate.
        // Locks the v0.8 limitation; v0.9 followup adds support-end
        // analysis. Fixture is identical to
        // `test_long_bridge_well_above_critical`: the v0.8 detector
        // sees both shapes the same way.
        let mesh = make_closed_bridge_fixture(20.0, 5.0, 1.5, 10.0);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

        assert_eq!(
            result.long_bridges.len(),
            1,
            "cantilever currently flags as a bridge (v0.8 limitation; v0.9 followup)"
        );
    }

    #[test]
    fn test_long_bridge_diagonal_underflagged_documented() {
        // 14×14 horizontal patch at z=10, `max_bridge_span = 15`.
        // span = max(14, 14) = 14 < 15 → no flag, even though the true
        // Euclidean diagonal (14·√2 ≈ 19.8 mm) exceeds the limit. v0.8
        // axis-aligned bbox is conservative for diagonals; v0.9 OBB
        // followup will catch this case.
        let mesh = make_closed_bridge_fixture(14.0, 14.0, 1.5, 10.0);
        let mut config = PrinterConfig::fdm_default();
        config.max_bridge_span = 15.0;

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

        assert_eq!(
            result.long_bridges.len(),
            0,
            "diagonal patch: bbox extent (14) < max (15); v0.8 underflags by design"
        );
    }

    #[test]
    fn test_long_bridge_regions_sorted_by_start_xyz() {
        // §4.4 ordering: `long_bridges` sorted by `(start.x, start.y,
        // start.z)`. Two parallel slabs appended in REVERSE x-order
        // (high-x slab first → lower face indices). Without §4.4 sort,
        // emission order would be high-x then low-x. With the sort,
        // low-x must precede high-x.
        let mut vertices: Vec<Point3<f64>> = Vec::new();
        let mut faces: Vec<[u32; 3]> = Vec::new();
        append_closed_cuboid(
            &mut vertices,
            &mut faces,
            Point3::new(-3.0, -3.0, 0.0),
            Point3::new(-1.0, -1.0, 2.0),
        );
        // High-x slab appended FIRST (gets lower face indices).
        append_closed_cuboid(
            &mut vertices,
            &mut faces,
            Point3::new(30.0, 0.0, 10.0),
            Point3::new(50.0, 5.0, 11.5),
        );
        // Low-x slab appended SECOND.
        append_closed_cuboid(
            &mut vertices,
            &mut faces,
            Point3::new(0.0, 0.0, 10.0),
            Point3::new(20.0, 5.0, 11.5),
        );
        let mesh = IndexedMesh::from_parts(vertices, faces);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

        assert_eq!(result.long_bridges.len(), 2);
        let first = &result.long_bridges[0];
        let second = &result.long_bridges[1];
        assert!(
            first.start.x < second.start.x,
            "§4.4: long_bridges must sort by start.x; got [{}, {}]",
            first.start.x,
            second.start.x
        );
    }

    #[test]
    fn test_long_bridge_co_flags_with_overhang() {
        // §6.2 edge case: cluster overlapping with overhang region —
        // both detectors flag independently (not a duplicate-flag bug).
        // The slab bottom has overhang_angle = 90° (>> FDM 45°
        // threshold) AND span 20 > max=10. Both regions emit.
        let mesh = make_closed_bridge_fixture(20.0, 5.0, 1.5, 10.0);
        let config = PrinterConfig::fdm_default();

        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

        assert_eq!(result.long_bridges.len(), 1);
        assert!(
            !result.overhangs.is_empty(),
            "slab bottom (90° overhang) must also flag as ExcessiveOverhang"
        );
    }

    // ---- §6.3 Gap H — TrappedVolume detector tests --------------------------
    //
    // The §6.3 spec calls the load-bearing fixture "sphere_inside_cube" but
    // the implementation here uses a cube-cavity-in-cube. Rationale: the
    // detector is winding- and curvature-agnostic (operates on voxelized
    // parity); a cube cavity is sufficient to exercise every code path. The
    // §9.2.5 stress fixture `stress_h_sphere_inside_cube_volume_within_10pct`
    // (in `tests/stress_inputs.rs`) is the geometry-faithful sphere variant.

    /// Build a watertight outer cube with a single inner-cube cavity.
    /// Outer cube vertices `[0..8]` are wound CCW-from-outside (mirrors
    /// `create_watertight_cube`). Inner cavity vertices `[8..16]` are wound
    /// CCW-from-inside the cavity (normals point into the cavity). The
    /// detector's parity-based inside-test is winding-agnostic so either
    /// inner winding works; this convention matches the implicit-surface
    /// "body is the inside set" mental model.
    fn make_cube_with_inner_cavity(outer_size: f64, inner_size: f64) -> IndexedMesh {
        let cs = (outer_size - inner_size) / 2.0;
        let i_max = cs + inner_size;
        let vertices = vec![
            // Outer cube vertices [0..8]
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(outer_size, 0.0, 0.0),
            Point3::new(outer_size, outer_size, 0.0),
            Point3::new(0.0, outer_size, 0.0),
            Point3::new(0.0, 0.0, outer_size),
            Point3::new(outer_size, 0.0, outer_size),
            Point3::new(outer_size, outer_size, outer_size),
            Point3::new(0.0, outer_size, outer_size),
            // Inner cavity vertices [8..16]
            Point3::new(cs, cs, cs),
            Point3::new(i_max, cs, cs),
            Point3::new(i_max, i_max, cs),
            Point3::new(cs, i_max, cs),
            Point3::new(cs, cs, i_max),
            Point3::new(i_max, cs, i_max),
            Point3::new(i_max, i_max, i_max),
            Point3::new(cs, i_max, i_max),
        ];
        let faces = vec![
            // Outer cube — CCW-from-outside.
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [3, 6, 2],
            [3, 7, 6],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
            // Inner cavity — wound CCW-from-INSIDE the cavity.
            [8, 9, 10],
            [8, 10, 11],
            [12, 14, 13],
            [12, 15, 14],
            [8, 13, 9],
            [8, 12, 13],
            [11, 10, 14],
            [11, 14, 15],
            [8, 15, 12],
            [8, 11, 15],
            [9, 14, 10],
            [9, 13, 14],
        ];
        IndexedMesh::from_parts(vertices, faces)
    }

    /// Build a watertight outer cube with two disjoint inner-cube cavities
    /// at `(cavity_size/2 + offset_a, …)` and `(outer_size − cavity_size/2 −
    /// offset_a, …)` along the diagonal — well-separated so flood-fill
    /// labels them as distinct components. 24 vertices, 36 faces.
    fn make_cube_with_two_inner_cavities(
        outer_size: f64,
        cavity_size: f64,
        offset_a: f64,
    ) -> IndexedMesh {
        let outer_min = Point3::new(0.0, 0.0, 0.0);
        let outer_max = Point3::new(outer_size, outer_size, outer_size);
        let a_lo = offset_a;
        let a_hi = a_lo + cavity_size;
        let b_lo = outer_size - offset_a - cavity_size;
        let b_hi = b_lo + cavity_size;
        let vertices = vec![
            // Outer cube [0..8]
            outer_min,
            Point3::new(outer_max.x, 0.0, 0.0),
            Point3::new(outer_max.x, outer_max.y, 0.0),
            Point3::new(0.0, outer_max.y, 0.0),
            Point3::new(0.0, 0.0, outer_max.z),
            Point3::new(outer_max.x, 0.0, outer_max.z),
            outer_max,
            Point3::new(0.0, outer_max.y, outer_max.z),
            // Cavity A [8..16]
            Point3::new(a_lo, a_lo, a_lo),
            Point3::new(a_hi, a_lo, a_lo),
            Point3::new(a_hi, a_hi, a_lo),
            Point3::new(a_lo, a_hi, a_lo),
            Point3::new(a_lo, a_lo, a_hi),
            Point3::new(a_hi, a_lo, a_hi),
            Point3::new(a_hi, a_hi, a_hi),
            Point3::new(a_lo, a_hi, a_hi),
            // Cavity B [16..24]
            Point3::new(b_lo, b_lo, b_lo),
            Point3::new(b_hi, b_lo, b_lo),
            Point3::new(b_hi, b_hi, b_lo),
            Point3::new(b_lo, b_hi, b_lo),
            Point3::new(b_lo, b_lo, b_hi),
            Point3::new(b_hi, b_lo, b_hi),
            Point3::new(b_hi, b_hi, b_hi),
            Point3::new(b_lo, b_hi, b_hi),
        ];
        let faces = vec![
            // Outer cube — CCW-from-outside.
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [3, 6, 2],
            [3, 7, 6],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
            // Cavity A — CCW-from-INSIDE the cavity.
            [8, 9, 10],
            [8, 10, 11],
            [12, 14, 13],
            [12, 15, 14],
            [8, 13, 9],
            [8, 12, 13],
            [11, 10, 14],
            [11, 14, 15],
            [8, 15, 12],
            [8, 11, 15],
            [9, 14, 10],
            [9, 13, 14],
            // Cavity B — same winding pattern, offset by +8.
            [16, 17, 18],
            [16, 18, 19],
            [20, 22, 21],
            [20, 23, 22],
            [16, 21, 17],
            [16, 20, 21],
            [19, 18, 22],
            [19, 22, 23],
            [16, 23, 20],
            [16, 19, 23],
            [17, 22, 18],
            [17, 21, 22],
        ];
        IndexedMesh::from_parts(vertices, faces)
    }

    /// Coarse-voxel `PrinterConfig` for fast unit tests: voxel = 0.2 mm
    /// regardless of technology (`min_feature_size = 0.8`,
    /// `layer_height = 0.4`). At a 6 mm outer cube, this produces a 34³ ≈
    /// 39 k-voxel grid that finishes in <100 ms in debug mode while still
    /// exercising every algorithmic path. The technology field still
    /// drives `classify_trapped_volume_severity`'s SLA / SLS / MJF /
    /// FDM branch.
    fn coarse_voxel_config(tech: PrintTechnology) -> PrinterConfig {
        let mut c = match tech {
            PrintTechnology::Sla => PrinterConfig::sla_default(),
            PrintTechnology::Sls => PrinterConfig::sls_default(),
            PrintTechnology::Mjf => PrinterConfig::mjf_default(),
            PrintTechnology::Fdm | PrintTechnology::Other => PrinterConfig::fdm_default(),
        };
        c.technology = tech;
        c.min_feature_size = 0.8;
        c.layer_height = 0.4;
        c
    }

    #[test]
    fn test_trapped_volume_no_cavity() {
        // Solid 6 mm cube (no inner cavity) under FDM coarse-voxel config.
        // Flood-fill from grid corner reaches every non-inside voxel →
        // no trapped voxels → 0 regions.
        let mesh = create_watertight_cube();
        let config = coarse_voxel_config(PrintTechnology::Fdm);
        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for solid cube");

        assert_eq!(
            result.trapped_volumes.len(),
            0,
            "solid cube must produce no trapped-volume regions"
        );
        let any_trapped = result
            .issues
            .iter()
            .any(|i| i.issue_type == PrintIssueType::TrappedVolume);
        assert!(!any_trapped, "solid cube must emit no TrappedVolume issues");
    }

    #[test]
    fn test_trapped_volume_skipped_on_open_mesh() {
        // Open mesh (4-face square at z=0; 4 open edges → not watertight)
        // → DetectorSkipped Info; trapped_volumes stays empty.
        let mesh = create_cube_mesh();
        let config = coarse_voxel_config(PrintTechnology::Fdm);
        #[allow(clippy::expect_used)]
        let result =
            validate_for_printing(&mesh, &config).expect("validation should succeed for open mesh");

        let any_skipped = result.issues.iter().any(|i| {
            i.issue_type == PrintIssueType::DetectorSkipped
                && i.description.contains("TrappedVolume")
                && i.description.contains("watertight")
        });
        assert!(
            any_skipped,
            "open mesh must emit DetectorSkipped with TrappedVolume + watertight in description"
        );
        assert_eq!(
            result.trapped_volumes.len(),
            0,
            "open mesh must not populate trapped_volumes"
        );
    }

    #[test]
    fn test_trapped_volume_sphere_inside_cube() {
        // 6 mm outer cube with 2 mm inner cavity (cube cavity used as
        // cavity proxy per module-doc deviation note). Expected analytical
        // cavity volume: 2³ = 8 mm³. At voxel 0.2 mm, the cavity occupies
        // ~10³ = 1000 voxels = 1000 × 0.008 = 8 mm³ exactly. Bbox is
        // approximately 2×2×2 mm (within ±voxel_size on each side).
        let mesh = make_cube_with_inner_cavity(6.0, 2.0);
        let config = coarse_voxel_config(PrintTechnology::Fdm);
        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for cube-cavity fixture");

        assert_eq!(
            result.trapped_volumes.len(),
            1,
            "single inner cavity must produce exactly one trapped-volume region"
        );
        let region = &result.trapped_volumes[0];
        // Cavity center is at outer_size/2 = 3.0 mm in each axis.
        assert!(
            (region.center.x - 3.0).abs() < 0.2 + 1e-9
                && (region.center.y - 3.0).abs() < 0.2 + 1e-9
                && (region.center.z - 3.0).abs() < 0.2 + 1e-9,
            "cavity centroid must be near (3, 3, 3); got {:?}",
            region.center
        );
        let bbox_extent_x = region.bounding_box.1.x - region.bounding_box.0.x;
        let bbox_extent_y = region.bounding_box.1.y - region.bounding_box.0.y;
        let bbox_extent_z = region.bounding_box.1.z - region.bounding_box.0.z;
        assert!(
            (bbox_extent_x - 2.0).abs() < 0.4
                && (bbox_extent_y - 2.0).abs() < 0.4
                && (bbox_extent_z - 2.0).abs() < 0.4,
            "cavity bbox must be ~2×2×2 mm; got {bbox_extent_x:.3} × {bbox_extent_y:.3} × {bbox_extent_z:.3}"
        );
    }

    #[test]
    fn test_trapped_volume_two_disjoint_cavities() {
        // 8 mm outer cube with 2 disjoint 1 mm cubic cavities at offset 1.5
        // from opposite corners. Walls between cavities ≥ 3 mm = 15
        // voxels, so flood-fill labels them as 2 separate components.
        let mesh = make_cube_with_two_inner_cavities(8.0, 1.0, 1.5);
        let config = coarse_voxel_config(PrintTechnology::Fdm);
        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for two-cavity fixture");

        assert_eq!(
            result.trapped_volumes.len(),
            2,
            "two disjoint cavities must produce exactly two trapped-volume regions"
        );
    }

    #[test]
    fn test_trapped_volume_critical_for_sla() {
        // SLA + 2 mm cavity (volume 8 mm³) ≫ res_volume = 0.8³ = 0.512
        // → severity Critical (trapped uncured resin is a hard failure).
        let mesh = make_cube_with_inner_cavity(6.0, 2.0);
        let config = coarse_voxel_config(PrintTechnology::Sla);
        #[allow(clippy::expect_used)]
        let result =
            validate_for_printing(&mesh, &config).expect("validation should succeed for SLA");

        let critical_trapped = result.issues.iter().any(|i| {
            i.issue_type == PrintIssueType::TrappedVolume && i.severity == IssueSeverity::Critical
        });
        assert!(
            critical_trapped,
            "SLA + cavity above resolution → Critical severity"
        );
    }

    #[test]
    fn test_trapped_volume_critical_for_sls() {
        // SLS + 2 mm cavity → severity Critical (trapped unsintered powder).
        let mesh = make_cube_with_inner_cavity(6.0, 2.0);
        let config = coarse_voxel_config(PrintTechnology::Sls);
        #[allow(clippy::expect_used)]
        let result =
            validate_for_printing(&mesh, &config).expect("validation should succeed for SLS");

        let critical_trapped = result.issues.iter().any(|i| {
            i.issue_type == PrintIssueType::TrappedVolume && i.severity == IssueSeverity::Critical
        });
        assert!(
            critical_trapped,
            "SLS + cavity above resolution → Critical severity"
        );
    }

    #[test]
    fn test_trapped_volume_critical_for_mjf() {
        // MJF + 2 mm cavity → severity Critical (trapped unfused powder).
        let mesh = make_cube_with_inner_cavity(6.0, 2.0);
        let config = coarse_voxel_config(PrintTechnology::Mjf);
        #[allow(clippy::expect_used)]
        let result =
            validate_for_printing(&mesh, &config).expect("validation should succeed for MJF");

        let critical_trapped = result.issues.iter().any(|i| {
            i.issue_type == PrintIssueType::TrappedVolume && i.severity == IssueSeverity::Critical
        });
        assert!(
            critical_trapped,
            "MJF + cavity above resolution → Critical severity"
        );
    }

    #[test]
    fn test_trapped_volume_info_for_fdm() {
        // FDM + 2 mm cavity → severity Info (sealed cavity prints fine on
        // extrusion). Cavity is detected; the flag is just informational.
        let mesh = make_cube_with_inner_cavity(6.0, 2.0);
        let config = coarse_voxel_config(PrintTechnology::Fdm);
        #[allow(clippy::expect_used)]
        let result =
            validate_for_printing(&mesh, &config).expect("validation should succeed for FDM");

        let any_trapped_critical = result.issues.iter().any(|i| {
            i.issue_type == PrintIssueType::TrappedVolume && i.severity == IssueSeverity::Critical
        });
        assert!(
            !any_trapped_critical,
            "FDM trapped volume must NOT be Critical"
        );
        let any_trapped_info = result.issues.iter().any(|i| {
            i.issue_type == PrintIssueType::TrappedVolume && i.severity == IssueSeverity::Info
        });
        assert!(any_trapped_info, "FDM trapped volume must be Info");
    }

    #[test]
    fn test_trapped_volume_info_below_min_feature() {
        // Tiny cavity (0.6 mm cube → analytical 0.216 mm³) under config
        // with min_feature_size = 0.8 (res_volume = 0.512 mm³). Volume is
        // below resolution → severity Info regardless of technology.
        let mesh = make_cube_with_inner_cavity(6.0, 0.6);
        let config = coarse_voxel_config(PrintTechnology::Sla);
        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed for sub-resolution cavity");

        // The detector may emit a region (it depends on whether the
        // 0.6 mm cavity at voxel 0.2 mm contains any non-leak voxels).
        // What's load-bearing: any trapped issue reported is Info, never
        // Critical/Warning.
        let any_critical = result.issues.iter().any(|i| {
            i.issue_type == PrintIssueType::TrappedVolume && i.severity == IssueSeverity::Critical
        });
        assert!(
            !any_critical,
            "sub-resolution cavity must NOT be Critical even on SLA"
        );
        for issue in &result.issues {
            if issue.issue_type == PrintIssueType::TrappedVolume {
                assert_eq!(
                    issue.severity,
                    IssueSeverity::Info,
                    "all sub-resolution trapped issues must be Info"
                );
            }
        }
    }

    #[test]
    fn test_trapped_volume_volume_within_10pct_of_analytical() {
        // 6 mm outer cube + 2 mm cube cavity → analytical volume 8 mm³.
        // Voxel-discretized volume must be within ±10% of analytical
        // (per §9.6 + §6.3 line 1136 tolerance band; absorbs voxel
        // discretization noise + cross-platform ULP variance).
        let mesh = make_cube_with_inner_cavity(6.0, 2.0);
        let config = coarse_voxel_config(PrintTechnology::Fdm);
        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

        assert_eq!(result.trapped_volumes.len(), 1);
        let analytical_volume = 2.0_f64.powi(3); // cube cavity volume
        let voxel_volume = result.trapped_volumes[0].volume;
        approx::assert_relative_eq!(voxel_volume, analytical_volume, max_relative = 0.10);
    }

    #[test]
    fn test_trapped_volume_sort_stable_across_runs() {
        // Two disjoint cavities → two regions. §4.4 mandates sort by
        // (center.x, center.y, center.z) via total_cmp; back-to-back
        // runs must produce identical centers in the same order.
        let mesh = make_cube_with_two_inner_cavities(8.0, 1.0, 1.5);
        let config = coarse_voxel_config(PrintTechnology::Fdm);

        #[allow(clippy::expect_used)]
        let r1 = validate_for_printing(&mesh, &config).expect("validation should succeed (run 1)");
        #[allow(clippy::expect_used)]
        let r2 = validate_for_printing(&mesh, &config).expect("validation should succeed (run 2)");

        assert_eq!(r1.trapped_volumes.len(), 2);
        assert_eq!(r2.trapped_volumes.len(), 2);
        for (a, b) in r1.trapped_volumes.iter().zip(r2.trapped_volumes.iter()) {
            assert!((a.center.x - b.center.x).abs() < 1e-9);
            assert!((a.center.y - b.center.y).abs() < 1e-9);
            assert!((a.center.z - b.center.z).abs() < 1e-9);
        }
        // Ascending §4.4 sort: first center has smaller (x, y, z).
        assert!(
            r1.trapped_volumes[0].center.x <= r1.trapped_volumes[1].center.x,
            "§4.4: trapped_volumes must sort by center.x ascending"
        );
    }

    #[test]
    fn test_trapped_volume_voxel_grid_oom_safety_skips() {
        // Pathological config that would request a >1 GB grid: 200 mm cube
        // at FDM-coarsened voxel = 0.2 mm → 1004³ ≈ 10⁹ bytes — at the
        // boundary. Push into oversize via cavity-fixture-with-large-extent:
        // construct a fake fixture whose vertices span (-1000, +1000) on
        // each axis → 2000 mm extent at voxel 0.2 → 10004³ ≈ 10¹² bytes
        // → far above the 1 GB cap → DetectorSkipped emitted before
        // the grid is allocated.
        let outer_extent = 2000.0;
        let mesh = make_cube_with_inner_cavity(outer_extent, outer_extent / 3.0);
        let config = coarse_voxel_config(PrintTechnology::Fdm);
        #[allow(clippy::expect_used)]
        let result = validate_for_printing(&mesh, &config)
            .expect("validation should succeed (oom safety path)");

        let any_skipped = result.issues.iter().any(|i| {
            i.issue_type == PrintIssueType::DetectorSkipped
                && i.description.contains("TrappedVolume")
                && i.description.contains("1 GB")
        });
        assert!(
            any_skipped,
            "200 mm-extent fixture must trigger §6.3 step 4.5 memory pre-flight skip"
        );
        assert_eq!(
            result.trapped_volumes.len(),
            0,
            "memory-cap skip must not populate trapped_volumes"
        );
    }
}
