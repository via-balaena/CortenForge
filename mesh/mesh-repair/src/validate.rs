//! Mesh validation and health reporting.
//!
//! Checks meshes for common issues that can cause problems in downstream processing.

use mesh_types::IndexedMesh;

use crate::adjacency::MeshAdjacency;
use crate::winding::{WindingCensus, winding_census};

/// Report of mesh validation results.
///
/// Counts of common mesh defects, plus two orientation readings:
///
/// * [`Self::winding`] — whether any face is oriented against a face it shares
///   an edge with. `None` when not requested.
/// * [`Self::is_inside_out`] — the sign of the origin-apex signed volume.
///   Frame-dependent; see that field before trusting it.
///
/// ⚠ **Both can read clean on a mis-oriented mesh.** Neither compares faces
/// that share no edge, so an inverted *component* can escape both: the census
/// never sees across shells, and the volume sum reports whichever way the
/// total lands. [`WindingCensus`]'s "What this does NOT answer" enumerates the
/// cases, and [`crate::split_into_components`] separates the shells so each
/// can be checked on its own — with the cavity caveat on
/// [`crate::fix_winding_order`].
///
/// To repair what the census reports, see [`crate::fix_winding_order`]
/// (per-component) or [`crate::flip_winding`] (unconditional).
///
/// ```
/// use mesh_repair::validate_mesh;
/// # use mesh_types::{IndexedMesh, Point3};
/// # let mut mesh = IndexedMesh::new();
/// # mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
/// # mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
/// # mesh.vertices.push(Point3::new(0.0, 1.0, 0.0));
/// # mesh.faces.push([0, 1, 2]);
/// let report = validate_mesh(&mesh);
///
/// // A census verdict is trustworthy only where it had edges to judge.
/// let locally_clean = report
///     .winding
///     .is_some_and(|c| c.has_judgeable_edges() && !c.has_inconsistent_winding());
/// # let _ = locally_clean;
/// ```
///
/// ⚠ **Cannot be conjured from nothing.** No `Default`, and
/// `#[non_exhaustive]` blocks struct expressions from outside this crate, so a
/// report must start from [`validate_mesh`]. The fields stay `pub` and can be
/// overwritten afterwards — that is a caller misleading themselves, not a
/// value arriving from nowhere. Match with a `..` rest pattern.
#[derive(Debug, Clone)]
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
    /// Whether `V < 0`, for `V = Σ det[v₀,v₁,v₂] / 6` — the origin-apex
    /// tetrahedron sum. Always measured.
    ///
    /// ⚠ **Frame-dependent, and not a winding verdict in either direction.**
    /// This is the single home for that mechanism; other docs link here rather
    /// than restate it. Translating by `t` moves `V` by `(t/3) · Σ A_f n_f`,
    /// the area-weighted face-normal sum. Two consequences:
    ///
    /// 1. `V` is translation-invariant exactly when `Σ A_f n_f = 0`, which
    ///    holds for a **closed** consistently-oriented surface. Consistent
    ///    winding alone does not suffice — an open triangle is perfectly wound
    ///    and its flag still moves with the frame. Nor is it required: two
    ///    antipodal flipped faces cancel in `Σ A_f n_f`, so `V` stays
    ///    translation-invariant while the census counts six bad edges.
    /// 2. Where `V` is not invariant, the flag is set on one side of a
    ///    **half-space** in `t` — not "far from the origin". Translating
    ///    orthogonal to `Σ A_f n_f` never changes it at any magnitude.
    ///
    /// ⇒ `false` is not evidence of correct winding, and `true` may mean a
    /// global reversal, a local flip, **or** a correctly-wound open surface in
    /// an unlucky frame.
    pub is_inside_out: bool,

    /// Per-edge local orientation consistency — the question
    /// [`Self::is_inside_out`] cannot ask. See [`WindingCensus`] for its
    /// semantics and the several ways it can be vacuous.
    ///
    /// `None` when [`ValidationOptions::winding_census`] is off. The `Option`
    /// exists so that "never ran" and "ran and had nothing to judge" stay
    /// distinct — they are different facts, and a zeroed census collapses them.
    ///
    /// ⚠ **Its edge and face counters are NOT the report's, and can disagree.**
    /// The census skips a face listing a vertex twice *whole*; the adjacency
    /// behind [`Self::boundary_edge_count`], [`Self::non_manifold_edge_count`]
    /// and [`Self::is_manifold`] counts that face's traversals. One such face
    /// is enough: the report reads `non_manifold_edge_count == 1` and
    /// `is_manifold == false` while the census reads `non_manifold_edges == 0`.
    /// Likewise `winding.degenerate_faces` (index-repeating) is a subset of
    /// [`Self::degenerate_face_count`] (area below
    /// [`ValidationOptions::degenerate_area_threshold`]) — an index-repeating
    /// face has zero area, so it is counted by both, and the containment holds
    /// at any positive threshold. At exactly `0.0` it inverts, since
    /// `0.0 < 0.0` is false.
    pub winding: Option<WindingCensus>,
}

impl MeshReport {
    /// Check if the mesh is ready for 3D printing.
    ///
    /// Watertight, manifold, and a non-negative origin-apex signed volume.
    ///
    /// ⚠ **That last term is not a winding check** — see
    /// [`Self::is_inside_out`]. A mesh with locally flipped faces passes this,
    /// and on such a mesh the answer is **not stable**: `Σ A_f n_f` is then
    /// non-zero, so translating the mesh can flip it. Check [`Self::winding`]
    /// first.
    /// [`Self::winding`] is reported but not judged here, nor by
    /// [`Self::has_issues`] or [`Self::issue_count`].
    #[must_use]
    pub const fn is_printable(&self) -> bool {
        self.is_watertight && self.is_manifold && !self.is_inside_out
    }

    /// Check if the mesh has any issues.
    ///
    /// Connectivity and geometry defects only; winding is excluded, and
    /// [`Self::issue_count`] is kept in step with this.
    #[must_use]
    pub const fn has_issues(&self) -> bool {
        self.boundary_edge_count > 0
            || self.non_manifold_edge_count > 0
            || self.degenerate_face_count > 0
            || self.duplicate_face_count > 0
    }

    /// Get a count of total issues found. Excludes winding, as
    /// [`Self::has_issues`] does.
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
        // Two readings of two different questions, labelled separately.
        writeln!(
            f,
            "    Signed volume: {} (global, origin-apex)",
            if self.is_inside_out {
                "negative"
            } else {
                "non-negative"
            }
        )?;
        // Three states, each named for what actually happened. The `Option`
        // is what makes the third distinguishable from the second.
        match self.winding {
            Some(c) if c.has_judgeable_edges() => writeln!(
                f,
                "    Local winding: {} of {} interior edges inconsistent",
                c.inconsistent_edges, c.interior_edges
            ),
            Some(_) => writeln!(f, "    Local winding: no judgeable interior edge"),
            None => writeln!(f, "    Local winding: not measured"),
        }?;

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
    /// Whether to run the per-edge winding census.
    ///
    /// It adds ~28 % to `validate_mesh` (see `CHANGELOG.md`). Several other
    /// `O(faces)` passes run regardless — the adjacency build, duplicate-face
    /// and degenerate-face detection — so this is a share of the cost, not all
    /// of it.
    ///
    /// Off ⇒ [`MeshReport::winding`] is `None`. Nothing else is affected:
    /// [`MeshReport::is_inside_out`] is always measured, because it is a
    /// single allocation-free pass and gating it bought nothing but a field
    /// that could not tell you it had been skipped.
    pub winding_census: bool,
}

impl Default for ValidationOptions {
    fn default() -> Self {
        Self {
            degenerate_area_threshold: 1e-12,
            winding_census: true,
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
/// ⚠ **Two edge maps, not one**, when `winding_census` is on: the
/// [`MeshAdjacency`] built here, and [`winding_census`]'s own. The census
/// needs per-edge traversal *direction*, which `MeshAdjacency` does not
/// record and cannot currently supply. Measured at **~28 % over the 1.0.0
/// single-map version** across 12–5120 faces (+0.4 µs at 12 faces, +204 µs at
/// 5120).
#[must_use]
pub fn validate_mesh_with_options(mesh: &IndexedMesh, options: &ValidationOptions) -> MeshReport {
    let adjacency = MeshAdjacency::build(&mesh.faces);

    let degenerate_face_count = count_degenerate_faces(mesh, options.degenerate_area_threshold);
    let duplicate_face_count = count_duplicate_faces(&mesh.faces);
    // ⚠ `is_inside_out` is NOT gated. It is one allocation-free pass, and a
    // gated bool cannot report that it was skipped — the census can, via
    // `None`, which is why only the census is optional.
    let is_inside_out = check_inside_out(mesh);
    let winding = options.winding_census.then(|| winding_census(mesh));

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

    /// The census from a report that requested one. Panics if it did not —
    /// which is the point: a test reading census counters is asserting the
    /// census ran, and should fail loudly rather than silently read zeros.
    ///
    /// The panic is the contract, not an oversight, hence the narrow allow.
    #[allow(clippy::panic, reason = "test helper whose failure mode IS a panic")]
    fn census(report: &MeshReport) -> WindingCensus {
        let Some(c) = report.winding else {
            panic!("this report was built with the census enabled")
        };
        c
    }

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
        assert_eq!(census(&report).inconsistent_edges, 3);
        // ...out of all six, so the census judged the whole closed surface
        // rather than reaching a verdict from a sliver of it.
        //
        // ⚠ `has_inconsistent_winding()` and `has_judgeable_edges()` are
        // deliberately NOT asserted here: both are *defined* as `> 0` on
        // counters the line above already pins, so beside it neither can fail
        // and asserting them would read as verification while adding none.
        assert_eq!(census(&report).interior_edges, 6);

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
        assert_eq!(
            report.issue_count(),
            0,
            "issue_count is documented as kept in step with has_issues; \
             without this, adding winding to it alone fails nothing"
        );
    }

    /// The option gates the census and NOTHING else.
    ///
    /// ⚠ The fixture must be one where the measured value differs from the
    /// value a gate would hardcode. Face 2 flipped makes the mesh genuinely
    /// inside-out, so a resurrected gate returning `false` fails here; on a
    /// fixture that measures `false` anyway, this assert would pass either
    /// way and prove nothing.
    #[test]
    fn the_option_gates_the_census_and_leaves_the_volume_flag_measured() {
        let mut mesh = unit_tetrahedron();
        let f = mesh.faces[2];
        mesh.faces[2] = [f[0], f[2], f[1]];

        let report = validate_mesh_with_options(
            &mesh,
            &ValidationOptions {
                winding_census: false,
                ..Default::default()
            },
        );

        assert!(report.winding.is_none(), "the census was not requested");
        assert!(
            report.is_inside_out,
            "the volume flag is measured regardless of the option — a gate on \
             it would report the hardcoded `false` here"
        );
        // Cross-check that the two answers really do differ on this fixture,
        // so the assert above cannot pass by coincidence.
        assert_eq!(
            census(&validate_mesh(&mesh)).inconsistent_edges,
            3,
            "fixture precondition: this mesh is BOTH inside-out and locally \
             inconsistent, so neither instrument's answer is the default"
        );
    }

    /// The fixture's stated rationale, pinned rather than asserted in prose.
    ///
    /// Without this, recentring `unit_tetrahedron` — an ordinary tidy-up —
    /// would leave every other test green while silently making the fixture's
    /// "perturbs the sum by exactly nothing" claim false.
    ///
    /// ⚠ **Compares the SCALAR, not the flag.** The claim is that a value is
    /// *exactly* unchanged, and a boolean can only witness a sign crossing:
    /// recentring the fixture moves the sum 0.1178 → 0.0589 — destroying the
    /// claim — while both values stay positive and the flag never budges.
    #[test]
    fn the_flip_leaves_the_signed_volume_sum_untouched() {
        // Computed here rather than read from the report: the claim is about
        // the fixture's geometry, so routing it through the flag under
        // discussion would make it circular.
        fn origin_apex_volume(mesh: &IndexedMesh) -> f64 {
            mesh.faces
                .iter()
                .map(|f| {
                    let (a, b, c) = (
                        mesh.vertices[f[0] as usize],
                        mesh.vertices[f[1] as usize],
                        mesh.vertices[f[2] as usize],
                    );
                    a.coords.dot(&b.coords.cross(&c.coords))
                })
                .sum::<f64>()
                / 6.0
        }

        let before = origin_apex_volume(&unit_tetrahedron());
        let after = origin_apex_volume(&tetrahedron_with_one_flipped_face());

        assert_eq!(
            before.to_bits(),
            after.to_bits(),
            "the flipped face's origin-apex term must be exactly zero, or this \
             fixture no longer demonstrates an exact blindness (before {before}, \
             after {after})"
        );
    }

    /// The counter-example to "near the origin the flag misses a local flip".
    ///
    /// Face 2 is the only face of `unit_tetrahedron` not containing the origin
    /// vertex, so its determinant is the sum's only non-zero term. Reversing it
    /// takes `V` from `+0.1178` to `-0.1178` — the flag fires on a single local
    /// flip, at the origin. The dependence is a half-space in the translation,
    /// not a distance.
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
        assert_eq!(census(&report).inconsistent_edges, 3);
    }

    /// The other counter-example: consistent winding does not buy invariance.
    ///
    /// `simple_triangle` has zero winding defects and is open. Translating it
    /// to `z = -1` makes `V = -16.67`, so the flag fires on a perfectly-wound
    /// mesh one unit from the origin. Invariance needs `sum(A_f * n_f) == 0`,
    /// which needs CLOSURE — not just consistency.
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
        // ⚠ NOT `inconsistent_edges == 0` — a single triangle has no interior
        // edge, so that would follow from the counter's definition rather than
        // from the census reaching a verdict. The census ABSTAINS here; it does
        // not agree. Pinning the abstention is the honest claim, and it is the
        // one that makes the contrast with the fired flag meaningful.
        assert!(
            !census(&report).has_judgeable_edges(),
            "the census had nothing to judge, so the flag fired with no local \
             defect present and none ruled out"
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
            census(&report).inconsistent_edges,
            census(&report).faces_on_inconsistent_edges,
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

    /// The three census states must render as three different lines.
    ///
    /// This replaces three earlier tests that each guarded one half of a
    /// wording problem: with a single all-zero census, "ran but had nothing to
    /// judge" and "never ran" were indistinguishable, so any cause the line
    /// named was false in one of the two cases. `Option` makes them distinct,
    /// and this pins that they stay distinct.
    #[test]
    fn display_distinguishes_a_vacuous_census_from_an_absent_one() {
        // (a) ran, and had judgeable edges.
        let judged = format!("{}", validate_mesh(&unit_tetrahedron()));
        // (b) ran, and had nothing to judge — a lone triangle has no interior edge.
        let vacuous = format!("{}", validate_mesh(&simple_triangle()));
        // (c) never ran.
        let absent = format!(
            "{}",
            validate_mesh_with_options(
                &unit_tetrahedron(),
                &ValidationOptions {
                    winding_census: false,
                    ..Default::default()
                },
            )
        );

        assert!(judged.contains("Local winding: 0 of 6 interior edges inconsistent"));
        assert!(vacuous.contains("Local winding: no judgeable interior edge"));
        assert!(absent.contains("Local winding: not measured"));

        // No trailing `assert_ne!` on the two "Local winding" lines: the three
        // asserts above already pin three distinct strings, so distinctness
        // follows from them and a fourth assert would read as verification
        // while adding none.
    }

    /// Producer for the type docs' claim that the census's edge counters and
    /// the report's own can disagree. One index-repeating face is enough.
    #[test]
    fn the_census_and_the_report_disagree_on_non_manifold_edges() {
        let mut mesh = unit_tetrahedron();
        mesh.faces.push([0, 0, 1]); // index-repeat: skipped whole by the census

        let report = validate_mesh(&mesh);

        // Adjacency counts the face's traversals... (`is_manifold` is not
        // asserted beside this: it is `all(len <= 2)` over the same map that
        // produced the count, so it cannot fail while the count holds.)
        assert_eq!(report.non_manifold_edge_count, 1);
        // ...the census skipped it, so it sees no non-manifold edge at all.
        assert_eq!(census(&report).non_manifold_edges, 0);
        assert_eq!(census(&report).degenerate_faces, 1);
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
        assert_eq!(census(&report).degenerate_faces, 1);
        // The area test sees BOTH — an index-repeating face has two coincident
        // corners, hence exactly zero area. This is the containment the docs
        // claim, and the reason the two must not be read as disjoint.
        //
        // ⚠ No trailing `degenerate_faces <= degenerate_face_count` assert: it
        // would follow from the two numbers above and could not fail beside
        // them.
        assert_eq!(report.degenerate_face_count, 2);

        // ...and the documented inversion at a threshold of exactly 0.0,
        // where `area < threshold` admits nothing and the containment flips.
        let strict = validate_mesh_with_options(
            &mesh,
            &ValidationOptions {
                degenerate_area_threshold: 0.0,
                ..Default::default()
            },
        );
        // Only the report's counter can respond to the threshold —
        // `winding_census` takes none, so re-asserting the census here would
        // be the same call on the same mesh as above.
        assert_eq!(strict.degenerate_face_count, 0);
    }

    #[test]
    fn has_issues_empty_mesh() {
        assert!(!validate_mesh(&IndexedMesh::new()).has_issues());
    }

    /// `issue_count` sums the four connectivity/geometry counters.
    ///
    /// ⚠ **The four counters must be PAIRWISE DISTINCT**, and that is asserted
    /// below rather than stated here. With any two equal, substituting one
    /// field for another inside `issue_count` leaves the total unchanged and
    /// the mutation ships green — a public reporting method could silently
    /// stop counting non-manifold edges.
    #[test]
    fn issue_count_sums_the_connectivity_and_geometry_defects() {
        let mut mesh = IndexedMesh::new();
        for p in [
            // two triangles, each duplicated: 2 duplicates, no boundary edges
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (5.0, 0.0, 0.0),
            (6.0, 0.0, 0.0),
            (5.0, 1.0, 0.0),
            // three collinear triples: 3 zero-area faces, 9 boundary edges
            (10.0, 0.0, 0.0),
            (11.0, 0.0, 0.0),
            (12.0, 0.0, 0.0),
            (15.0, 0.0, 0.0),
            (16.0, 0.0, 0.0),
            (17.0, 0.0, 0.0),
            (20.0, 0.0, 0.0),
            (21.0, 0.0, 0.0),
            (22.0, 0.0, 0.0),
            // a three-face fan on one edge: 1 non-manifold edge, 6 boundary
            (30.0, 0.0, 0.0),
            (31.0, 0.0, 0.0),
            (30.0, 1.0, 0.0),
            (30.0, 2.0, 0.0),
            (30.0, 3.0, 0.0),
        ] {
            mesh.vertices.push(Point3::new(p.0, p.1, p.2));
        }
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([3, 4, 5]);
        mesh.faces.push([3, 4, 5]);
        mesh.faces.push([6, 7, 8]);
        mesh.faces.push([9, 10, 11]);
        mesh.faces.push([12, 13, 14]);
        mesh.faces.push([15, 16, 17]);
        mesh.faces.push([15, 16, 18]);
        mesh.faces.push([15, 16, 19]);

        let report = validate_mesh(&mesh);

        let terms = [
            report.boundary_edge_count,
            report.non_manifold_edge_count,
            report.degenerate_face_count,
            report.duplicate_face_count,
        ];
        // Two separate properties, and each kills a different mutant:
        // non-zero kills "a term was dropped from the sum", pairwise-distinct
        // kills "one field was substituted for another". An earlier fixture
        // had each in turn and neither together.
        assert!(terms.iter().all(|&t| t > 0), "every term must be non-zero");
        for i in 0..terms.len() {
            for j in (i + 1)..terms.len() {
                assert_ne!(
                    terms[i], terms[j],
                    "terms {i} and {j} both equal {}; substituting one for the \
                     other in `issue_count` would leave the total unchanged",
                    terms[i]
                );
            }
        }

        assert_eq!(report.issue_count(), terms.iter().sum::<usize>());
        assert_eq!(report.issue_count(), 21);
    }
}
