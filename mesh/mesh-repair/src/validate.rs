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
///   of the two that can say whether two neighbouring faces are oriented
///   against each other. See [`WindingCensus`] for precisely which question it
///   answers and the several ways it can be vacuous — that type is the single
///   home for those semantics, and this doc does not restate them.
/// * [`Self::is_inside_out`] sums origin-apex tetrahedra, so it is a *global*
///   quantity and it is **frame-dependent**. Translating the mesh by `t` moves
///   the sum by `(t/3) · Σ A_f n_f`, the area-weighted face-normal sum.
///
/// ⚠ **Two consequences of that formula, both of which are easy to state
/// wrongly:**
///
/// 1. **The flag is translation-invariant exactly when `Σ A_f n_f = 0`** —
///    which holds for a **closed** consistently-oriented surface. Consistent
///    winding *alone* is not enough: `simple_triangle` in this file's tests is
///    perfectly wound, and at `z = -1` its sum is `-100`, so the flag is set.
///    Nor is it necessary — two antipodal flipped faces cancel in `Σ A_f n_f`
///    and leave the sum invariant while the census counts six bad edges.
/// 2. **Where it is not invariant, the flag is set on one side of a
///    half-space in `t` — not "far from the origin".** Translating along a
///    direction orthogonal to `Σ A_f n_f` never changes it at any magnitude,
///    and translating the other way keeps it `false` however far you go, with
///    the flip hidden inside. (Producer for one such traversal, in the `+z`
///    half-space only: `signed_volume_sees_a_local_flip_once_the_mesh_is_off_origin`.)
///
/// ⇒ **Do not read this flag as a winding verdict in either direction.**
/// `false` is not evidence of correct winding, and `true` may mean a global
/// reversal, a local flip, **or** a correctly-wound open surface in an unlucky
/// frame.
///
/// ⚠ **`winding.degenerate_faces` and [`Self::degenerate_face_count`] are
/// nested, not disjoint.** The census counts faces listing a vertex index
/// twice (a connectivity defect); `degenerate_face_count` counts faces whose
/// *area* is below [`ValidationOptions::degenerate_area_threshold`] (a
/// geometric one). An index-repeating face has two coincident corners and so
/// has exactly zero area — it is counted by **both**. For any positive
/// threshold `winding.degenerate_faces <= degenerate_face_count`, so
/// subtracting them to isolate "geometric-only" slivers works, but reading
/// them as independent populations does not.
///
/// **Report-only.** These are readings, not verdicts; see
/// [`Self::is_printable`] for what is and is not judged.
///
/// ⚠ **`#[non_exhaustive]`.** Build one with [`validate_mesh`] rather than a
/// struct literal; match with a `..` rest pattern. Added in 2.0.0 alongside
/// the [`Self::winding`] field, so that the major break is paid once and
/// later fields are additive.
#[derive(Debug, Clone, Default)]
#[non_exhaustive]
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
    /// `false` when [`ValidationOptions::check_winding`] is off —
    /// indistinguishable from a mesh that was checked and found not to be
    /// inside-out.
    pub is_inside_out: bool,

    /// Per-edge local orientation consistency — the question
    /// [`Self::is_inside_out`] cannot ask.
    ///
    /// All-zero when [`ValidationOptions::check_winding`] is off.
    ///
    /// ⚠ **That state is self-identifying for the *verdict* only, and for
    /// nothing else.** `winding.has_judgeable_edges()` is `false`, so a zero
    /// `inconsistent_edges` reports "no edge was examined" rather than "the
    /// winding is clean" — `is_inside_out` has no equivalent tell. But
    /// `has_judgeable_edges()` reads `interior_edges` alone. The census's
    /// other four counters — `boundary_edges`, `non_manifold_edges`,
    /// `degenerate_faces`, `faces_on_inconsistent_edges` — are zeroed by the
    /// option with **no tell at all**, and are indistinguishable from a mesh
    /// genuinely free of those defects. A gate asserting
    /// `winding.non_manifold_edges == 0` is silently satisfied by turning the
    /// option off, exactly as `is_inside_out` is.
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
    /// cannot, because [`Self::is_inside_out`] is frame-dependent and unsound
    /// as a winding verdict in both directions (type docs). A mesh with
    /// locally flipped faces can pass this, and a correctly-wound **open**
    /// mesh can fail it purely because of where it sits in its own frame.
    ///
    /// ▶ **Deliberately unchanged.** [`Self::winding`] is reported here but not
    /// judged. Note what that does *not* rest on: this predicate has **no
    /// caller in this workspace** outside tests, so the in-tree risk of
    /// tightening it is nil. The reasons to defer are that (a) nothing has yet
    /// measured what real meshes census at, and (b) it is a published
    /// predicate, so changing its meaning is a behavioural break that deserves
    /// its own evidence and its own release note rather than riding along with
    /// a structural one. See also [`Self::has_issues`] and
    /// [`Self::issue_count`], which exclude winding for the same reason.
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
    ///
    /// ▶ Excludes inconsistent winding, for the same reason as
    /// [`Self::has_issues`] — the two are deliberately kept in step, so a
    /// change to one belongs to both.
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
        // `Winding: Correct` derived from `is_inside_out` alone — a claim the
        // signed-volume test cannot support (see the type docs).
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
            // ⚠ No cause is stated on purpose. An all-zero census means EITHER
            // the mesh had no judgeable interior edge OR `check_winding` was
            // off, and this report does not record which — so naming either
            // one here would be the same kind of unsupported claim the line
            // above was retired for.
            writeln!(f, "    Local winding: not examined")?;
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
///
/// # Cost
///
/// ⚠ **Two edge maps, not one**, when `check_winding` is on: the
/// [`MeshAdjacency`] built here, and [`winding_census`]'s own. The census
/// needs per-edge traversal *direction*, which `MeshAdjacency` does not
/// record and cannot currently supply. Measured at **~27 % over the 1.0.0
/// single-map version** across 12–5120 faces; see `CHANGELOG.md` for the
/// numbers and for the `build_edges_only` offset that would likely repay it.
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
    /// Face 1 (`[0, 1, 3]`) contains vertex 0, which sits at the world origin,
    /// so its origin-apex tetrahedron has **exactly zero** volume and
    /// reversing it perturbs the signed-volume sum by exactly nothing. The
    /// blindness demonstrated here is therefore exact rather than approximate
    /// — and `the_flip_leaves_the_signed_volume_sum_untouched` pins that,
    /// because it is a property of where the fixture sits, not a law.
    ///
    /// ⚠ Three of the four faces contain vertex 0, so the index is not
    /// uniquely determined; face **2** (`[1, 2, 3]`) is the one that does not,
    /// and flipping *it* sets the flag even at the origin —
    /// `a_local_flip_can_set_the_flag_at_the_origin` covers that case, and is
    /// why this file does not describe the global test as distance-dependent.
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
        // ...out of all six, so the census judged the whole closed surface
        // rather than reaching a verdict from a sliver of it.
        //
        // ⚠ `has_inconsistent_winding()` and `has_judgeable_edges()` are
        // deliberately NOT asserted here: both are *defined* as `> 0` on
        // counters the line above already pins, so beside it neither can fail
        // and asserting them would read as verification while adding none.
        assert_eq!(report.winding.interior_edges, 6);

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

    /// The fixture's stated rationale, pinned rather than asserted in prose.
    ///
    /// Without this, recentring `unit_tetrahedron` — an ordinary tidy-up —
    /// would leave every other test green while silently making the fixture's
    /// "perturbs the sum by exactly nothing" claim false. Compares the flag
    /// across the flip rather than pinning it absolutely, which is the form
    /// that survives the fixture moving.
    #[test]
    fn the_flip_leaves_the_signed_volume_sum_untouched() {
        let before = validate_mesh(&unit_tetrahedron()).is_inside_out;
        let after = validate_mesh(&tetrahedron_with_one_flipped_face()).is_inside_out;

        assert_eq!(
            before, after,
            "the flipped face's origin-apex volume must be exactly zero, or \
             this fixture no longer demonstrates an exact blindness"
        );
    }

    /// The counter-example to "near the origin the flag misses a local flip".
    ///
    /// Face 2 is the only face of `unit_tetrahedron` not containing the origin
    /// vertex, so its determinant is the sum's only non-zero term. Reversing
    /// it takes the total from `+0.7067` to `-0.7067` — the flag fires on a
    /// single local flip, at the origin. The dependence is a half-space in the
    /// translation, not a distance.
    #[test]
    fn a_local_flip_can_set_the_flag_at_the_origin() {
        let mut mesh = unit_tetrahedron();
        let f = mesh.faces[2];
        mesh.faces[2] = [f[0], f[2], f[1]];

        let report = validate_mesh(&mesh);

        assert!(
            report.is_inside_out,
            "a local flip on the one face not touching the origin sets the \
             global flag with no translation at all"
        );
        // Same defect, same magnitude, as the face-1 flip the other tests use
        // — so the two cases differ only in the GLOBAL test's response, which
        // is the point.
        assert_eq!(report.winding.inconsistent_edges, 3);
    }

    /// The other counter-example: consistent winding does not buy invariance.
    ///
    /// `simple_triangle` has zero winding defects and is open. Translating it
    /// to `z = -1` makes the origin-apex sum `-100`, so the flag fires on a
    /// perfectly-wound mesh one unit from the origin. Invariance needs
    /// `sum(A_f * n_f) == 0`, which needs CLOSURE — not just consistency.
    #[test]
    fn a_consistently_wound_open_mesh_can_set_the_flag_by_translation_alone() {
        assert!(
            !validate_mesh(&simple_triangle()).is_inside_out,
            "precondition: at the origin this fixture does not fire"
        );

        let mut moved = simple_triangle();
        for v in &mut moved.vertices {
            v.z -= 1.0;
        }

        let report = validate_mesh(&moved);
        assert!(
            report.is_inside_out,
            "an open, consistently-wound mesh fires purely on its frame"
        );
        assert_eq!(
            report.winding.inconsistent_edges, 0,
            "and the census correctly reports no local defect, so the two \
             instruments disagree without either being wrong"
        );
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

    /// The non-zero render, which the clean-mesh test above cannot exercise.
    ///
    /// On a clean mesh `inconsistent_edges` and `faces_on_inconsistent_edges`
    /// are both 0, so interpolating the wrong one still prints `0 of 6` and
    /// goes unnoticed. The flipped fixture separates them (3 vs 4).
    #[test]
    fn display_renders_the_actual_inconsistent_edge_count() {
        let report = validate_mesh(&tetrahedron_with_one_flipped_face());

        assert_ne!(
            report.winding.inconsistent_edges, report.winding.faces_on_inconsistent_edges,
            "precondition: the two counters must differ, or this test cannot \
             tell which one Display used"
        );
        assert!(format!("{report}").contains("Local winding: 3 of 6 interior edges inconsistent"));
    }

    /// The `is_inside_out == true` arm of the signed-volume line. The whole
    /// `writeln!` was rewritten in this change; pinning one arm is not pinning
    /// the branch.
    #[test]
    fn display_renders_the_negative_signed_volume_arm() {
        let mut moved = simple_triangle();
        for v in &mut moved.vertices {
            v.z -= 1.0;
        }

        let display = format!("{}", validate_mesh(&moved));
        assert!(display.contains("Signed volume: negative (global, origin-apex)"));
    }

    #[test]
    fn display_marks_an_unjudgeable_census_rather_than_printing_a_clean_zero() {
        // A single triangle has no interior edge, so the census saw nothing.
        let display = format!("{}", validate_mesh(&simple_triangle()));

        assert!(display.contains("Local winding: not examined"));
        // Pin the ROUTE, not just the text: this test exists to cover the
        // no-interior-edge cause, and would silently become a duplicate of
        // `display_names_no_cause_for_an_all_zero_census` if the census stopped
        // running here.
        let census = validate_mesh(&simple_triangle()).winding;
        assert_eq!(census.interior_edges, 0);
        assert_eq!(
            census.boundary_edges, 3,
            "the census DID run and saw three boundary edges — the zero above \
             is the mesh's shape, not a skipped check"
        );
    }

    #[test]
    fn display_names_no_cause_for_an_all_zero_census() {
        // The same "not examined" line, reached the OTHER way — a mesh with
        // six perfectly judgeable interior edges, whose census never ran. The
        // line must not claim a cause it cannot distinguish: saying "no
        // interior edges" here would be false.
        let options = ValidationOptions {
            check_winding: false,
            ..Default::default()
        };
        let display = format!(
            "{}",
            validate_mesh_with_options(&unit_tetrahedron(), &options)
        );

        assert!(display.contains("Local winding: not examined"));
        assert!(
            !display.contains("no interior edges"),
            "the mesh has six; the census simply did not run"
        );
        // The cross-check: those six edges are real and judgeable, so the
        // absent cause really would have been a false statement.
        assert_eq!(validate_mesh(&unit_tetrahedron()).winding.interior_edges, 6);
    }

    /// The SYMMETRIC guard. The test above rules out one false cause; without
    /// this one, naming the *other* cause — "`check_winding` disabled" — passes
    /// every test in this file while being false for every single-triangle
    /// mesh, whose census ran and simply had nothing to judge.
    #[test]
    fn display_names_no_cause_for_a_census_that_ran_and_found_nothing() {
        let display = format!("{}", validate_mesh(&simple_triangle()));

        assert!(display.contains("Local winding: not examined"));
        assert!(
            !display.contains("check_winding") && !display.contains("disabled"),
            "the census DID run on this mesh; naming the option as the cause \
             would be exactly the unsupported claim this line was rewritten to \
             avoid"
        );
    }

    /// The two degenerate counters are NESTED, and the type docs say so — this
    /// is the producer for that claim, which otherwise had none.
    ///
    /// No test in this crate previously read both counters on one mesh, so
    /// "they count different things" was checkable only by reading the source.
    #[test]
    fn the_two_degenerate_counters_are_nested_not_disjoint() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(2.0, 0.0, 0.0)); // collinear with the above
        mesh.faces.push([0, 1, 2]); // sliver: distinct indices, zero area
        mesh.faces.push([0, 0, 1]); // index-repeat: ALSO zero area

        let report = validate_mesh(&mesh);

        // The census judges connectivity, so it sees only the index-repeat.
        assert_eq!(report.winding.degenerate_faces, 1);
        // The area test sees BOTH — an index-repeating face has two coincident
        // corners, hence exactly zero area. This is the containment the docs
        // claim, and the reason the two must not be read as disjoint.
        assert_eq!(report.degenerate_face_count, 2);
        assert!(report.winding.degenerate_faces <= report.degenerate_face_count);
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
