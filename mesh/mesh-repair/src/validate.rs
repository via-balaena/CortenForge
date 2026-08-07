//! Mesh validation and health reporting.
//!
//! Checks meshes for common issues that can cause problems in downstream processing.

use mesh_types::IndexedMesh;

use crate::adjacency::MeshAdjacency;
use crate::winding::{WindingCensus, winding_census};

/// Report of mesh validation results.
///
/// Contains counts of various mesh issues and methods to check
/// overall mesh health.
///
/// # Reading the two winding instruments
///
/// This report carries **two** orientation readings, and neither subsumes the
/// other. They answer different questions and can disagree without either
/// being wrong.
///
/// * [`Self::winding`] is per-edge and **coordinate-free**. It is the only one
///   that answers "are two neighbouring faces oriented against each other".
/// * [`Self::is_inside_out`] sums origin-apex tetrahedra. ⚠ **That sum is
///   translation-invariant only while the surface is consistently oriented.**
///   Once any face is flipped, the flag depends on the mesh's distance from
///   the world origin: near it the flag **misses** a local flip, far from it a
///   local flip **sets** it. So `is_inside_out == false` is not evidence of
///   correct winding, and `true` may mean "globally reversed" *or* "locally
///   flipped and far from the origin". (Producer:
///   `signed_volume_sees_a_local_flip_once_the_mesh_is_off_origin`.)
///
/// ⚠ **`winding.degenerate_faces` and [`Self::degenerate_face_count`] count
/// different things** and are expected to differ: the census skips faces that
/// list a vertex index twice (a connectivity defect), while
/// `degenerate_face_count` counts faces whose *area* falls below
/// [`ValidationOptions::degenerate_area_threshold`] (a geometric one). A
/// sliver triangle with three distinct indices is counted by the latter only.
///
/// **Report-only.** These are readings, not verdicts; see
/// [`Self::is_printable`] for what is and is not judged.
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
    /// Whether the mesh's origin-apex signed volume is negative.
    ///
    /// ⚠ **A global test, and unsound as a winding check in both directions** —
    /// see the type docs before reading this as "the winding is reversed".
    /// Prefer [`Self::winding`] for local orientation consistency.
    ///
    /// `false` when [`ValidationOptions::check_winding`] is off, which is
    /// indistinguishable from a genuine negative result.
    pub is_inside_out: bool,

    /// Per-edge local orientation consistency — the question
    /// [`Self::is_inside_out`] cannot ask.
    ///
    /// All-zero when [`ValidationOptions::check_winding`] is off. That state is
    /// **self-identifying**: `winding.has_judgeable_edges()` is then `false`,
    /// so a zero `inconsistent_edges` reports "no edge was examined" rather
    /// than "the winding is clean". `is_inside_out` has no such tell.
    pub winding: WindingCensus,
}

impl MeshReport {
    /// Check if the mesh is ready for 3D printing.
    ///
    /// Requires the mesh to be watertight, manifold, and to have a
    /// **non-negative origin-apex signed volume**.
    ///
    /// ⚠ **That last term is not a winding check.** This method's doc
    /// previously claimed it required "correct winding"; it does not, and
    /// cannot, because [`Self::is_inside_out`] is unsound in both directions
    /// (type docs). A mesh with locally flipped faces can pass this, and a
    /// correctly wound mesh far from the origin can fail it.
    ///
    /// ▶ **Deliberately unchanged for now.** [`Self::winding`] is reported but
    /// not yet judged here, because this predicate gates production refusals
    /// (`mesh-shell`'s shell generation) and nothing has yet measured what
    /// in-tree meshes census at. Tightening it is a separate, evidence-led
    /// change — see also [`Self::has_issues`], which excludes winding for the
    /// same reason.
    #[must_use]
    pub const fn is_printable(&self) -> bool {
        self.is_watertight && self.is_manifold && !self.is_inside_out
    }

    /// Check if the mesh has any issues.
    ///
    /// ▶ Counts connectivity and geometry defects only. Inconsistent winding
    /// ([`Self::winding`]) is **excluded**, for the same reason it is excluded
    /// from [`Self::is_printable`]: adding it would change what existing
    /// callers see reported, ahead of any measurement of real inputs.
    #[must_use]
    pub const fn has_issues(&self) -> bool {
        self.boundary_edge_count > 0
            || self.non_manifold_edge_count > 0
            || self.degenerate_face_count > 0
            || self.duplicate_face_count > 0
    }

    /// Get a count of total issues found.
    #[must_use]
    pub const fn issue_count(&self) -> usize {
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
        // ⚠ These are two readings of two different questions, printed
        // separately on purpose. This block used to print a single
        // `Winding: Correct` derived from `is_inside_out` alone — a claim that
        // test could not support (see the type docs).
        writeln!(
            f,
            "    Signed volume: {} (global, origin-apex)",
            if self.is_inside_out {
                "negative"
            } else {
                "non-negative"
            }
        )?;
        if self.winding.has_judgeable_edges() {
            writeln!(
                f,
                "    Local winding: {} of {} interior edges inconsistent",
                self.winding.inconsistent_edges, self.winding.interior_edges
            )?;
        } else {
            writeln!(f, "    Local winding: not examined (no interior edges)")?;
        }

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
    /// Whether to run the two orientation instruments —
    /// [`MeshReport::is_inside_out`] and [`MeshReport::winding`].
    ///
    /// ⚠ **Turning this off is not neutral.** `is_inside_out` becomes a
    /// hardcoded `false`, which is indistinguishable from a mesh that was
    /// checked and found fine — so any gate asserting it did *not* fire is
    /// silently satisfied. [`MeshReport::winding`] does not share the defect:
    /// its `has_judgeable_edges()` reports `false` when it was never run.
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
/// use mesh_types::{IndexedMesh, Point3};
/// use mesh_repair::validate_mesh;
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(0.0, 1.0, 0.0));
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
    let (is_inside_out, winding) = if options.check_winding {
        (check_inside_out(mesh), winding_census(mesh))
    } else {
        (false, WindingCensus::default())
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
        winding,
    }
}

/// Count faces with area below the threshold.
fn count_degenerate_faces(mesh: &IndexedMesh, area_threshold: f64) -> usize {
    mesh.faces
        .iter()
        .filter(|face| {
            let v0 = &mesh.vertices[face[0] as usize];
            let v1 = &mesh.vertices[face[1] as usize];
            let v2 = &mesh.vertices[face[2] as usize];

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
const fn normalize_face(face: [u32; 3]) -> [u32; 3] {
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
        let v0 = &mesh.vertices[face[0] as usize];
        let v1 = &mesh.vertices[face[1] as usize];
        let v2 = &mesh.vertices[face[2] as usize];

        // Signed volume of tetrahedron formed with origin
        volume += v2.x.mul_add(
            v0.y.mul_add(v1.z, -(v1.y * v0.z)),
            v0.x.mul_add(
                v1.y.mul_add(v2.z, -(v2.y * v1.z)),
                v1.x * v2.y.mul_add(v0.z, -(v0.y * v2.z)),
            ),
        );
    }

    volume / 6.0 < 0.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Point3;

    fn simple_triangle() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(10.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 10.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh
    }

    fn unit_tetrahedron() -> IndexedMesh {
        // A closed tetrahedron with correct winding
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, 0.866, 0.0));
        mesh.vertices.push(Point3::new(0.5, 0.289, 0.816));

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
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(2.0, 0.0, 0.0)); // Collinear!
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
        let display = format!("{report}");

        assert!(display.contains("Vertices: 3"));
        assert!(display.contains("Watertight: No"));
    }

    /// Flip one face of a closed tetrahedron and reverse its winding.
    ///
    /// Face 1 (`[0, 1, 3]`) is chosen deliberately: it contains vertex 0, which
    /// sits at the world origin, so its origin-apex tetrahedron has **exactly
    /// zero** volume. Reversing it therefore perturbs the signed-volume sum by
    /// exactly nothing — the global test's blindness here is exact, not
    /// approximate. The general off-origin behaviour (where the flag flips on
    /// distance alone) is measured by `winding`'s
    /// `signed_volume_sees_a_local_flip_once_the_mesh_is_off_origin`.
    fn tetrahedron_with_one_flipped_face() -> IndexedMesh {
        let mut mesh = unit_tetrahedron();
        let f = mesh.faces[1];
        mesh.faces[1] = [f[0], f[2], f[1]];
        mesh
    }

    #[test]
    fn census_sees_a_local_flip_that_the_signed_volume_test_misses() {
        let report = validate_mesh(&tetrahedron_with_one_flipped_face());

        // Fixture precondition: if the global test ever started catching this,
        // the test would stop exercising the gap it exists to demonstrate.
        assert!(
            !report.is_inside_out,
            "the global signed-volume test must MISS this flip"
        );

        // The per-edge census is not blind to it. All three of the flipped
        // face's edges are now traversed the same way by both incident faces.
        assert_eq!(report.winding.inconsistent_edges, 3);
        assert!(report.winding.has_inconsistent_winding());
        assert!(report.winding.has_judgeable_edges());

        // ⇒ the exact claim the old `is_printable` doc made and could not
        //   support: a mesh whose winding is locally broken passes it. Pinned
        //   here so that the deferral is visible rather than assumed, and so
        //   that changing the predicate has to change this test on purpose.
        assert!(
            report.is_printable(),
            "is_printable is deliberately still winding-blind at this rung"
        );
        assert!(
            !report.has_issues(),
            "has_issues likewise excludes winding at this rung"
        );
    }

    #[test]
    fn disabling_the_winding_check_is_self_identifying_in_the_census() {
        let mesh = tetrahedron_with_one_flipped_face();
        let options = ValidationOptions {
            check_winding: false,
            ..Default::default()
        };
        let report = validate_mesh_with_options(&mesh, &options);

        // Both instruments now read "clean" — but only one of them admits that
        // it never ran, which is the whole difference between them.
        assert!(
            !report.is_inside_out,
            "the documented trap: indistinguishable from a genuine result"
        );
        assert_eq!(report.winding.inconsistent_edges, 0);
        assert!(
            !report.winding.has_judgeable_edges(),
            "a zero from an un-run census must report itself as un-examined"
        );

        // The same mesh, actually checked, is not clean — so the zero above is
        // an artefact of the option and not a property of the mesh. Without
        // this the assertions above would pass on a genuinely clean fixture.
        assert_eq!(validate_mesh(&mesh).winding.inconsistent_edges, 3);
    }

    #[test]
    fn display_reports_both_instruments_and_claims_neither_as_correctness() {
        let display = format!("{}", validate_mesh(&unit_tetrahedron()));

        assert!(display.contains("Signed volume: non-negative (global, origin-apex)"));
        assert!(display.contains("Local winding: 0 of 6 interior edges inconsistent"));
        assert!(
            !display.contains("Winding: Correct"),
            "the retired claim must not return: `is_inside_out` cannot support it"
        );
    }

    #[test]
    fn display_marks_an_unjudgeable_census_rather_than_printing_a_clean_zero() {
        // A single triangle has no interior edge, so the census saw nothing.
        let display = format!("{}", validate_mesh(&simple_triangle()));

        assert!(display.contains("Local winding: not examined (no interior edges)"));
        assert!(!display.contains("0 of 0"));
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
