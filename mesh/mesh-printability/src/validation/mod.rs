//! Main validation logic for printability analysis.
//!
//! Provides the primary validation functionality to check meshes
//! against printer constraints.

use std::collections::VecDeque;

use hashbrown::{HashMap, HashSet};
use mesh_types::{IndexedMesh, Point3, Vector3};
use parry3d_f64::bounding_volume::Aabb;
use parry3d_f64::math::{Point as ParryPoint, Real as ParryReal, Vector as ParryVector};
use parry3d_f64::query::{Ray, RayCast};
use parry3d_f64::shape::TriMesh;

use crate::config::{PrintTechnology, PrinterConfig};
use crate::error::{PrintabilityError, PrintabilityResult};
use crate::issues::{IssueSeverity, PrintIssue, PrintIssueType};
use crate::regions::{
    LongBridgeRegion, OverhangRegion, SelfIntersectingRegion, SmallFeatureRegion, SupportRegion,
    ThinWallRegion, TrappedVolumeRegion,
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

    /// Self-intersecting triangle pairs detected (§6.4 Gap I).
    pub self_intersecting: Vec<SelfIntersectingRegion>,

    /// Small-feature components detected (§6.5 Gap J).
    pub small_features: Vec<SmallFeatureRegion>,

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
            self_intersecting: Vec::new(),
            small_features: Vec::new(),
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

    // Per-check timing instrumentation — gated by env var
    // `MESH_PRINTABILITY_TIMING=1` so production runs are silent.
    // Diagnostic for `docs/CF_CAST_F4_SPATIAL_INDEX_RECON.md` §B-12
    // S3 — identifies which F4 check dominates after the
    // `flag_thin_wall_faces` BVH ship at S1.
    let time_checks = std::env::var_os("MESH_PRINTABILITY_TIMING").is_some();

    macro_rules! timed_check {
        ($name:literal, $call:expr) => {{
            let t = std::time::Instant::now();
            $call;
            if time_checks {
                eprintln!(
                    "[F4-time] {} ({} faces): {:.3}s",
                    $name,
                    mesh.face_count(),
                    t.elapsed().as_secs_f64()
                );
            }
        }};
    }

    // Check build volume
    timed_check!(
        "build_volume",
        check_build_volume(mesh, config, &mut validation)
    );

    // Check overhangs
    timed_check!("overhangs", check_overhangs(mesh, config, &mut validation));

    // Check manifold (basic check)
    timed_check!(
        "basic_manifold",
        check_basic_manifold(mesh, &mut validation)
    );

    // §6.1 Gap C — thin walls via inward ray-cast. Runs after manifold so
    // its precondition check (watertight + consistent winding) reflects
    // the same edge analysis the manifold detector just performed.
    timed_check!(
        "thin_walls",
        check_thin_walls(mesh, config, &mut validation)
    );

    // §6.2 Gap G — long bridges via boundary-edge span analysis. Runs
    // after `check_thin_walls`; FDM/SLA-only (silent skip on SLS/MJF
    // per `requires_supports()`).
    timed_check!(
        "long_bridges",
        check_long_bridges(mesh, config, &mut validation)
    );

    // §6.3 Gap H — trapped volumes via voxel-based exterior flood-fill.
    // Watertight precondition (NOT consistent winding, distinct from
    // ThinWall) — `§9.1` row 11 documents that TrappedVolume tolerates
    // inconsistent winding.
    timed_check!(
        "trapped_volumes",
        check_trapped_volumes(mesh, config, &mut validation)
    );

    // §6.4 Gap I — self-intersecting triangle pairs via mesh-repair
    // re-use. No precondition (mesh-repair handles all input gracefully:
    // single-face / empty / large meshes).
    timed_check!(
        "self_intersecting",
        check_self_intersecting(mesh, &mut validation)
    );

    // §6.5 Gap J — small features via connected-component bbox extent.
    // No precondition (tolerant of any input: empty, open, non-manifold,
    // or NaN-vertex). Catches floating debris, unit-conversion errors,
    // and CAD-leftover burrs.
    timed_check!(
        "small_features",
        check_small_features(mesh, config, &mut validation)
    );

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

/// Build a `parry3d_f64::shape::TriMesh` (BVH-backed) from an
/// [`IndexedMesh`].
///
/// Used by [`flag_thin_wall_faces`]'s BVH-accelerated ray-cast path
/// per `docs/CF_CAST_F4_SPATIAL_INDEX_RECON.md` §B-12 S1. Returns
/// `None` if `parry3d_f64`'s `TriMesh` builder would reject the input
/// (empty / out-of-bounds indices / non-finite vertices); the caller
/// falls back to a no-walls outcome which matches the watertight
/// precondition's behavior.
///
/// **Precision:** uses `parry3d-f64` (the f64 sibling of `parry3d`)
/// because the f32 sibling produces ~1e-7 mm drift in Möller-Trumbore
/// which crosses the `min_wall_thickness` boundary at the 1.0 mm
/// borderline test fixture. f64 keeps the BVH path bit-aligned to
/// the f64 reference path's Möller-Trumbore. Tradeoff: ~2× the
/// parry3d build cost in the workspace (parry3d remains for
/// `mesh-sdf`'s f32 SDF queries which don't hit threshold-boundary
/// precision issues at MC cell scales).
fn build_parry_trimesh(mesh: &IndexedMesh) -> Option<TriMesh> {
    // Defensive preconditions — `parry3d_f64::shape::TriMesh::new`
    // panics on empty vertices / out-of-bounds indices / non-finite
    // coordinates. Production callers reach this fn through
    // `check_thin_walls`'s watertight precondition (which implies
    // all of these), but the degenerate-input regression test
    // directly probes this entry. Return `None` rather than panic.
    if mesh.vertices.is_empty() || mesh.faces.is_empty() {
        return None;
    }
    let n_verts = mesh.vertices.len();
    for face in &mesh.faces {
        if (face[0] as usize) >= n_verts
            || (face[1] as usize) >= n_verts
            || (face[2] as usize) >= n_verts
        {
            return None;
        }
    }
    for v in &mesh.vertices {
        if !v.x.is_finite() || !v.y.is_finite() || !v.z.is_finite() {
            return None;
        }
    }
    let vertices: Vec<ParryPoint<ParryReal>> = mesh
        .vertices
        .iter()
        .map(|v| ParryPoint::new(v.x, v.y, v.z))
        .collect();
    Some(TriMesh::new(vertices, mesh.faces.clone()))
}

/// Walk every face and ray-cast inward; collect those whose nearest
/// opposite face is closer than `config.min_wall_thickness`. Returns
/// per-face `thickness` + `area` keyed by face index for downstream
/// edge-adjacency clustering and per-cluster emission.
///
/// **Algorithmic acceleration (S1 of
/// `docs/CF_CAST_F4_SPATIAL_INDEX_RECON.md`):** the inner ray-cast
/// uses a parry3d BVH built once at function entry, replacing the
/// pure O(face²) Möller-Trumbore double-loop. Per-mesh wall-clock
/// drops from ~540 s to ~1-2 s on production 400 k-face gasket
/// meshes (~300-500× algorithmic speedup). The
/// [`flag_thin_wall_faces_reference`] function preserves the O(n²)
/// implementation for regression testing.
fn flag_thin_wall_faces(
    mesh: &IndexedMesh,
    config: &PrinterConfig,
) -> HashMap<u32, ThinWallFlagMeta> {
    let mut flagged: HashMap<u32, ThinWallFlagMeta> = HashMap::new();
    let num_triangles = mesh.face_count();

    // Build the BVH once. Degenerate input (e.g., NaN vertices that
    // slipped past the watertight precondition) → no flagged faces,
    // same outcome as a no-thin-walls mesh.
    let Some(tri_mesh) = build_parry_trimesh(mesh) else {
        return flagged;
    };

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

        // BVH-accelerated inward ray-cast (replaces the O(face²)
        // Möller-Trumbore double-loop). `solid: false` ⇒ ray
        // starting inside the body is considered to start at the
        // entry surface and must exit the shape — returns the
        // exit-wall toi (the OPPOSITE-side wall thickness), which
        // is exactly what the thin-wall detector wants. The source
        // face's `t = 0` self-hit is implicit in the entry-skip
        // semantic — no explicit `j == i` filter needed.
        let ray = Ray::new(
            ParryPoint::new(origin.x, origin.y, origin.z),
            ParryVector::new(direction.x, direction.y, direction.z),
        );
        let min_dist = tri_mesh
            .cast_local_ray(&ray, ParryReal::MAX, false)
            .unwrap_or(f64::INFINITY);

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

/// O(face²) Möller-Trumbore reference implementation of
/// [`flag_thin_wall_faces`]. Preserved as a test-only path so the
/// BVH-accelerated production path can be regression-tested for
/// flagged-faces + per-face `min_dist` equivalence per
/// `docs/CF_CAST_F4_SPATIAL_INDEX_RECON.md` §B-3 #1.
#[cfg(test)]
fn flag_thin_wall_faces_reference(
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

        let thickness = min_dist + EPS_RAY_OFFSET;
        if thickness < config.min_wall_thickness {
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

/// Per-axis voxel-count cap for the `TrappedVolume` detector grid.
/// Bounds the `O((part_size / voxel_size)³)` memory + scanline cost
/// when the part's extent is much larger than `min_feature_size`.
///
/// For workshop cf-cast parts (200 mm-scale cup pieces) at the
/// default 0.1 mm voxel, the un-capped grid was 2000 × 300 × 300 =
/// 180 million voxels, costing ~50 s per cup half in the inside-
/// mark + flood-fill phase. With this cap at 500 the voxel size
/// scales up for large parts (200 mm → 0.4 mm voxel, ~2.8 M voxels
/// = 65× less work). Small parts (test fixtures ≤ 50 mm) are
/// unaffected — initial voxel size's 0.1 mm gives 500 voxels at
/// exactly 50 mm extent, cap doesn't kick in.
///
/// **Tradeoff:** trapped cavities smaller than the scaled voxel
/// get missed on large parts. Acceptable for cf-cast iter-N use
/// case (cup-piece cavities are gross body-cavity scale, not sub-
/// mm pockets). §6.3 v0.9 followup ("per-region adaptive voxel
/// sizing") is still the canonical fix; this cap is the workshop-
/// scale accelerator until that ships. See
/// `docs/CF_CAST_F4_SPATIAL_INDEX_RECON.md` §B-12 S3 for the
/// production iter-1 regen measurement that motivated the cap.
const MAX_VOXELS_PER_AXIS: f64 = 500.0;

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

/// Reference O(`ny·nz·n_faces`) inside-voxel marker (§6.3 step 6) — each
/// `(y, z)` scanline row brute-forces `moller_trumbore` against **every**
/// face.
///
/// For each `(y, z)` row, cast a `+X` ray from one voxel left of the grid
/// at `(grid_x_min − voxel_size, y_center + ROW_JITTER_Y, z_center +
/// ROW_JITTER_Z)`. Collect all `moller_trumbore` t-intersections, sort
/// by t. Sweep voxels left-to-right: at each voxel midpoint, if the
/// parity of crossings with `t < midpoint_t` is odd, mark voxel
/// `VOXEL_INSIDE`. Per row the cost is `O(n_faces + nx)`; per grid
/// `O(ny × nz × (n_faces + nx))`.
///
/// Preserved as the correctness oracle for the BVH-accelerated
/// [`mark_inside_voxels`] (a regression test asserts the two produce
/// identical `grid.states`) and as that function's fallback when the
/// parry `TriMesh` builder rejects degenerate input.
///
// FP-bit preserved on the `(coord + 0.5) * voxel_size` arithmetic: same
// FP-determinism rationale as `flag_overhang_faces`'s `len = sqrt(...)`
// site — `mul_add` would shift FP bits and break cross-os voxel
// parity per §8.4 row 3.
#[allow(clippy::suboptimal_flops)]
fn mark_inside_voxels_reference(grid: &mut VoxelGrid, mesh: &IndexedMesh) {
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

/// Margin (mm) added to each per-row query AABB before culling candidate
/// faces from the BVH in [`mark_inside_voxels`].
///
/// Loosening the query AABB can only **add** candidate faces — an added
/// face the `+X` ray does not actually pierce yields
/// `moller_trumbore == None` and contributes no crossing — so the
/// retained crossing set (and every resulting voxel state) is invariant
/// to this value for any `margin ≥ 0`. A small positive margin defends
/// against any strict-vs-inclusive AABB-overlap edge case exactly at the
/// cull boundary; it is correctness insurance, not a tuning knob.
const ROW_QUERY_MARGIN_MM: f64 = 1.0e-6;

/// BVH-accelerated inside-voxel marker (S3 of
/// `docs/CF_CAST_F4_SPATIAL_INDEX_RECON.md` — S1 BVH'd `thin_walls`, S2
/// measured `trapped_volumes` as the remaining long pole, S3 is this).
///
/// Produces a **byte-for-byte identical** `grid.states` to
/// [`mark_inside_voxels_reference`], but replaces the per-row
/// `O(n_faces)` Möller-Trumbore sweep with a parry3d BVH cull: each `+X`
/// scanline queries only the triangles whose AABB overlaps the ray, then
/// runs the **same** f64 `moller_trumbore` on those candidates. Per-grid
/// cost drops from `O(ny·nz·n_faces)` to `~O(ny·nz·(log n_faces + k))`
/// where `k` is the per-row candidate count (a handful for typical mesh
/// face sizes). On a 530 k-face cup half this is the §6.3 long pole:
/// ~138 s → single-digit seconds, the bulk of F4.
///
/// **Bit-identity argument.** A `+X` ray at `(y_center, z_center)` can
/// pierce triangle `T` only if `(y_center, z_center)` lies inside `T`'s
/// YZ projection, which is contained in `T`'s YZ AABB. So every face the
/// reference path's `moller_trumbore` accepts is returned by the AABB
/// query (BVH leaf AABB ⊇ triangle AABB ⊇ projection ∋ the ray point,
/// further loosened by [`ROW_QUERY_MARGIN_MM`]). The culled faces are
/// exactly those `moller_trumbore` would reject (`None`). The retained
/// crossings are therefore the same multiset; the sort + parity sweep
/// are unchanged. (Equal-`t` crossings may be ordered differently than
/// the reference's face-order, but two crossings sharing a `t` are both
/// below or both at/above any `voxel_mid_t`, so the per-voxel parity —
/// and thus every voxel state — is invariant to their relative order.)
///
/// The precision-sensitive inside test stays on the f64
/// `moller_trumbore`, **not** parry's ray cast, per §8.4 row 3 (parry's
/// intersection would shift FP bits and break cross-OS voxel parity); the
/// BVH is used **only** to cull candidates.
///
/// Falls back to [`mark_inside_voxels_reference`] when
/// [`build_parry_trimesh`] rejects the mesh (degenerate input the
/// watertight precondition would normally exclude).
//
// FP-bit preserved on the `(coord + 0.5) * voxel_size` arithmetic: same
// rationale as the reference path.
#[allow(clippy::suboptimal_flops)]
fn mark_inside_voxels(grid: &mut VoxelGrid, mesh: &IndexedMesh) {
    // BVH build failure ⇒ exact reference fallback (no behavior change).
    let Some(tri_mesh) = build_parry_trimesh(mesh) else {
        mark_inside_voxels_reference(grid, mesh);
        return;
    };
    let qbvh = tri_mesh.qbvh();

    let (nx, ny, nz) = grid.dims;
    let v = grid.voxel_size;
    let origin_x = grid.origin.x;
    let origin_y = grid.origin.y;
    let origin_z = grid.origin.z;

    // +X ray direction (unit, axis-aligned for FP determinism).
    let direction = Vector3::new(1.0, 0.0, 0.0);

    // X-span of the query AABB: cover the whole row (two voxels past each
    // end, comfortably enclosing the reference ray origin + the last
    // voxel midpoint). The X bounds only widen the cull; they never drop
    // a face the ray could cross.
    let x_lo = origin_x - 2.0 * v;
    let x_hi = origin_x + (f64::from(nx) + 2.0) * v;

    // Reused across rows to avoid per-row reallocation.
    let mut candidates: Vec<u32> = Vec::new();
    let mut crossings: Vec<f64> = Vec::new();

    for z in 0..nz {
        let z_center = origin_z + (f64::from(z) + 0.5) * v + ROW_JITTER_Z;
        for y in 0..ny {
            let y_center = origin_y + (f64::from(y) + 0.5) * v + ROW_JITTER_Y;
            // Ray origin one voxel-width outside the grid (identical to the
            // reference path so the resulting `t` values match exactly).
            let row_origin = Point3::new(origin_x - v, y_center, z_center);

            // Cull to faces whose AABB overlaps this +X row, loosened by
            // ROW_QUERY_MARGIN_MM (only adds candidates; see const doc).
            let query = Aabb::new(
                ParryPoint::new(
                    x_lo,
                    y_center - ROW_QUERY_MARGIN_MM,
                    z_center - ROW_QUERY_MARGIN_MM,
                ),
                ParryPoint::new(
                    x_hi,
                    y_center + ROW_QUERY_MARGIN_MM,
                    z_center + ROW_QUERY_MARGIN_MM,
                ),
            );
            candidates.clear();
            qbvh.intersect_aabb(&query, &mut candidates);

            crossings.clear();
            for &tri in &candidates {
                let face_idx = tri as usize;
                if face_idx >= mesh.faces.len() {
                    continue;
                }
                let face = mesh.faces[face_idx];
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

    let initial_voxel_size = config.min_feature_size.min(config.layer_height) / 2.0;
    if !initial_voxel_size.is_finite() || initial_voxel_size <= 0.0 {
        // Pathological config (zero / negative / NaN feature size). Tolerate
        // silently — `validate_for_printing`'s upstream `check_build_volume`
        // is the authoritative handler for nonsensical configs.
        return;
    }

    let (mesh_min, mesh_max) = compute_bounds(mesh);

    // S3 voxel-axis cap: see `MAX_VOXELS_PER_AXIS` const at module
    // scope below for the algorithmic rationale.
    let max_extent = (mesh_max.x - mesh_min.x)
        .max(mesh_max.y - mesh_min.y)
        .max(mesh_max.z - mesh_min.z);
    let cap_voxel_size = max_extent / MAX_VOXELS_PER_AXIS;
    let voxel_size = initial_voxel_size.max(cap_voxel_size);

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

/// §6.4 Gap I — detect self-intersecting triangle pairs via mesh-repair.
///
/// Re-uses `mesh_repair::detect_self_intersections` with
/// `IntersectionParams::default()` (`max_reported = 100`,
/// `epsilon = 1e-10`, `skip_adjacent = true`). One
/// `SelfIntersectingRegion` is pushed per intersecting pair (capped
/// at 100 by default params; the cap is exposed in the issue
/// description). One summary `PrintIssue` is pushed at
/// `IssueSeverity::Critical` — slicer behavior on self-intersection is
/// undefined, so any pair blocks `is_printable()`.
///
/// Adjacent triangles sharing an edge are skipped via
/// `skip_adjacent = true` and not flagged. Vertex-only contact is not
/// an intersection (interiors do not share points) and is not flagged.
///
/// `face_a` and `face_b` are canonicalized so `face_a < face_b` per
/// §4.4. `approximate_location` is the midpoint of the two face
/// centroids — visual placement only. The §4.4 sort is range-restricted
/// to this detector's emissions so a future hoist of the sort to
/// `validate_for_printing` does not have to undo it per-detector
/// (same precedent as `check_trapped_volumes` at `validation.rs:1867`).
fn check_self_intersecting(mesh: &IndexedMesh, validation: &mut PrintValidation) {
    let result = mesh_repair::intersect::detect_self_intersections(
        mesh,
        &mesh_repair::intersect::IntersectionParams::default(),
    );

    if !result.has_intersections {
        return;
    }

    let regions_before = validation.self_intersecting.len();
    for &(a, b) in &result.intersecting_pairs {
        // mesh-repair's outer/inner-loop construction emits `(i, j)`
        // with `i < j`, so canonicalization is defensive insurance —
        // §6.4 line 1162 documents that mesh-repair does not guarantee
        // canonical order in its output.
        let (face_a, face_b) = if a < b { (a, b) } else { (b, a) };
        let centroid_a = compute_face_centroid(mesh, face_a as usize);
        let centroid_b = compute_face_centroid(mesh, face_b as usize);
        let approximate_location = Point3::new(
            f64::midpoint(centroid_a.x, centroid_b.x),
            f64::midpoint(centroid_a.y, centroid_b.y),
            f64::midpoint(centroid_a.z, centroid_b.z),
        );
        validation
            .self_intersecting
            .push(SelfIntersectingRegion::new(
                face_a,
                face_b,
                approximate_location,
            ));
    }

    let pair_count = result.intersection_count;
    let truncation_suffix = if result.truncated {
        " (search truncated; total may be higher)"
    } else {
        ""
    };
    let issue = PrintIssue::new(
        PrintIssueType::SelfIntersecting,
        IssueSeverity::Critical,
        format!("{pair_count} self-intersecting triangle pair(s){truncation_suffix}"),
    );
    validation.issues.push(issue);

    // §4.4 sort: `(face_a, face_b)` ascending. Range-restricted to
    // this detector's emissions per the precedent above.
    validation.self_intersecting[regions_before..].sort_by(|a, b| {
        a.face_a
            .cmp(&b.face_a)
            .then_with(|| a.face_b.cmp(&b.face_b))
    });
}

/// Partition every face of the mesh into edge-connected components
/// (§6.5 Gap J helper).
///
/// Two faces are adjacent iff they share an edge, regardless of
/// orientation or how many other faces also share that edge. Open
/// edges (1-incident) and non-manifold edges (>2 incident) both
/// contribute to adjacency — every distinct unordered pair of faces
/// sharing an edge gets linked. This is broader than
/// `partition_flagged_into_components`'s manifold-only adjacency
/// (used by Gap D / Gap C / Gap G) because §6.5's small-feature
/// definition is purely topological: a CAD-leftover burr loosely
/// connected to the main body via a single non-manifold edge should
/// still be one component, not split into pieces.
///
/// Every face index `0..mesh.faces.len()` seeds the DFS, including
/// faces with no shared edges (true islands). Components emerge in
/// min-face-idx order: the outer loop visits seeds in ascending
/// index, so the first unvisited face index becomes the seed of the
/// next component. Faces within each component are sorted ascending,
/// matching `partition_flagged_into_components`'s output contract.
fn partition_all_faces_into_components(mesh: &IndexedMesh) -> Vec<Vec<u32>> {
    let edge_to_faces = build_edge_to_faces(mesh);
    let mut adjacency: HashMap<u32, Vec<u32>> = HashMap::new();
    for faces in edge_to_faces.values() {
        if faces.len() < 2 {
            continue;
        }
        for i in 0..faces.len() {
            for j in (i + 1)..faces.len() {
                let a = faces[i];
                let b = faces[j];
                adjacency.entry(a).or_default().push(b);
                adjacency.entry(b).or_default().push(a);
            }
        }
    }

    let face_count = mesh.faces.len();
    let mut visited: HashSet<u32> = HashSet::new();
    let mut components: Vec<Vec<u32>> = Vec::new();

    for face_idx in 0..face_count {
        // Mesh face index fits in u32 (mesh size bounded well below 2^32);
        // pattern shared with `build_edge_to_faces`.
        #[allow(clippy::cast_possible_truncation)]
        let face_idx = face_idx as u32;
        if !visited.insert(face_idx) {
            continue;
        }
        let mut component: Vec<u32> = vec![face_idx];
        let mut stack: Vec<u32> = vec![face_idx];
        while let Some(f) = stack.pop() {
            if let Some(neighbors) = adjacency.get(&f) {
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

/// Signed volume of a face subset via the divergence theorem
/// (§6.5 Gap J helper).
///
/// Sums per-face tetrahedral volumes `(v0 · (v1 × v2)) / 6` against
/// the origin. For a closed, consistently-wound surface this equals
/// the enclosed volume up to sign (positive for outward winding,
/// negative for inward). For open or non-manifold components the
/// result is non-physical; the caller takes `abs(...)` and documents
/// the field as approximate.
///
/// FP semantics: scalar form is preserved verbatim from the spec
/// (§6.5 line 1238) for cross-platform bit-level reproducibility on
/// exact-representable inputs (`stress_j_signed_volume_unit_cube`
/// locks 1.0 mm³ within 1e-6 on a unit cube). `mul_add` substitution
/// would change the rounding profile across platforms.
fn signed_volume(mesh: &IndexedMesh, face_indices: &[u32]) -> f64 {
    let mut vol = 0.0;
    for &fi in face_indices {
        let face = mesh.faces[fi as usize];
        let v0 = &mesh.vertices[face[0] as usize];
        let v1 = &mesh.vertices[face[1] as usize];
        let v2 = &mesh.vertices[face[2] as usize];
        // Per-site `#[allow]` justification: §5.1 deferral pattern —
        // `f64::mul_add` would change rounding bits across platforms,
        // breaking the 1e-6 tolerance assertion in
        // `stress_j_signed_volume_unit_cube`. The `(a*b - c*d)`
        // determinant form is the canonical signed-tetrahedron volume.
        #[allow(clippy::suboptimal_flops)]
        let term = v0.x * (v1.y * v2.z - v1.z * v2.y)
            + v0.y * (v1.z * v2.x - v1.x * v2.z)
            + v0.z * (v1.x * v2.y - v1.y * v2.x);
        vol += term / 6.0;
    }
    vol
}

/// Classify `SmallFeature` severity per §6.5 line 1266.
///
/// - `max_extent < min_feature_size / 2.0` → `IssueSeverity::Warning`
///   (definitely below printer resolution)
/// - otherwise → `IssueSeverity::Info` (borderline; may print)
///
/// No `Critical` band: small features are advisory, not blocking.
/// Even a unit-conversion-mistaken mesh that is entirely below
/// resolution should not block `is_printable()` — the user reading
/// the warning fixes the mesh upstream.
fn classify_small_feature_severity(max_extent: f64, min_feature: f64) -> IssueSeverity {
    if max_extent < min_feature / 2.0 {
        IssueSeverity::Warning
    } else {
        IssueSeverity::Info
    }
}

/// Detect connected components below the printer's minimum feature
/// size (§6.5 Gap J).
///
/// A small feature is any edge-connected face component whose AABB
/// max-extent falls under `config.min_feature_size`. Catches floating
/// debris, isolated tiny protrusions left by CAD/Boolean ops, and
/// unit-conversion errors (a mesh authored in metres looks like a
/// single sub-millimetre fragment under FDM's 0.8 mm threshold).
///
/// No precondition; runs on any input topology. Open/non-manifold
/// components produce approximate volume via `abs(signed_volume)` —
/// non-physical but no-panic, documented in the `volume` field.
///
/// Per qualifying component: one `SmallFeatureRegion` pushed and one
/// `PrintIssue` of type `SmallFeature`. Severity per
/// `classify_small_feature_severity`. The §4.4 sort is range-restricted
/// to this detector's emissions so a future hoist of the sort to
/// `validate_for_printing` does not have to undo it per-detector
/// (same precedent as `check_self_intersecting`).
fn check_small_features(
    mesh: &IndexedMesh,
    config: &PrinterConfig,
    validation: &mut PrintValidation,
) {
    let components = partition_all_faces_into_components(mesh);
    let regions_before = validation.small_features.len();

    for component in components {
        // Collect unique vertex indices from the component's faces.
        let mut vertex_indices: HashSet<u32> = HashSet::new();
        for &fi in &component {
            let face = mesh.faces[fi as usize];
            vertex_indices.insert(face[0]);
            vertex_indices.insert(face[1]);
            vertex_indices.insert(face[2]);
        }

        // AABB over component vertices.
        let mut bbox_min = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut bbox_max = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for &vi in &vertex_indices {
            let v = &mesh.vertices[vi as usize];
            bbox_min.x = bbox_min.x.min(v.x);
            bbox_min.y = bbox_min.y.min(v.y);
            bbox_min.z = bbox_min.z.min(v.z);
            bbox_max.x = bbox_max.x.max(v.x);
            bbox_max.y = bbox_max.y.max(v.y);
            bbox_max.z = bbox_max.z.max(v.z);
        }
        let extent_x = bbox_max.x - bbox_min.x;
        let extent_y = bbox_max.y - bbox_min.y;
        let extent_z = bbox_max.z - bbox_min.z;
        let max_extent = extent_x.max(extent_y).max(extent_z);

        // NaN-tolerant flag gate: a NaN `max_extent` makes
        // `max_extent < threshold` evaluate to false, so NaN-vertex
        // components are silently skipped (no false positive, no panic).
        // Direct positive-form check so the NaN handling is explicit
        // rather than via `!(<)`.
        if !max_extent.is_finite() || max_extent >= config.min_feature_size {
            continue;
        }

        // Centroid: mean of component vertex positions.
        let mut centroid_sum_x: f64 = 0.0;
        let mut centroid_sum_y: f64 = 0.0;
        let mut centroid_sum_z: f64 = 0.0;
        for &vi in &vertex_indices {
            let v = &mesh.vertices[vi as usize];
            centroid_sum_x += v.x;
            centroid_sum_y += v.y;
            centroid_sum_z += v.z;
        }
        // `vertex_indices.len()` ≤ `mesh.vertices.len()`, well below 2^53;
        // usize → f64 cast is exact in this range.
        #[allow(clippy::cast_precision_loss)]
        let n = vertex_indices.len() as f64;
        let center = Point3::new(centroid_sum_x / n, centroid_sum_y / n, centroid_sum_z / n);

        let volume = signed_volume(mesh, &component).abs();
        // `component.len()` ≤ `mesh.faces.len()`, bounded by `u32::MAX`;
        // pattern shared with `build_edge_to_faces`'s face-index cast.
        #[allow(clippy::cast_possible_truncation)]
        let face_count = component.len() as u32;
        let region = SmallFeatureRegion::new(center, max_extent, volume, face_count, component);
        let severity = classify_small_feature_severity(max_extent, config.min_feature_size);

        let issue = PrintIssue::new(
            PrintIssueType::SmallFeature,
            severity,
            format!(
                "{face_count} face(s) form a small feature with max extent {max_extent:.3} mm \
                 (min feature size {:.3} mm; volume {volume:.3} mm³)",
                config.min_feature_size
            ),
        )
        .with_location(center)
        .with_affected_elements(region.faces.clone());

        validation.small_features.push(region);
        validation.issues.push(issue);
    }

    // §4.4 sort: by `min(component_face_indices)` ascending. Each
    // region's `faces` is already sorted ascending (per
    // `partition_all_faces_into_components`), so `faces[0]` IS the
    // min. Range-restricted to this detector's emissions per the
    // precedent in `check_self_intersecting`.
    validation.small_features[regions_before..].sort_by(|a, b| {
        let a_min = a.faces.first().copied().unwrap_or(u32::MAX);
        let b_min = b.faces.first().copied().unwrap_or(u32::MAX);
        a_min.cmp(&b_min)
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
mod tests;
