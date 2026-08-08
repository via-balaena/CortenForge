//! Shell validation utilities.
//!
//! Measures a shell's topology and reports what it finds. It does not *ensure*
//! anything: [`validate_shell`] returns a [`ShellValidationResult`] whose two
//! predicates deliberately answer different questions, and whose `issues` list
//! carries defects that neither predicate consults.

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
    /// Whether every judgeable edge agrees on winding direction.
    ///
    /// Measured by [`mesh_repair::winding_census`]: an edge is judgeable when
    /// exactly two faces meet along it, and it is inconsistent when both
    /// traverse it the same way. Faces listing a vertex twice are skipped
    /// whole, so they neither create nor spoil a judgeable edge.
    ///
    /// ⚠ "Skipped" there means *repeated vertex index*, which is not what
    /// `DegenerateTriangles` counts in this module — that one is a zero-**area**
    /// test. A sliver triangle with three distinct indices is degenerate by area
    /// and still judged here.
    ///
    /// ⚠ On a NON-EMPTY shell this is vacuously `true` when nothing is
    /// judgeable: a mesh with no interior edge has no edge that disagrees.
    /// Ruling that out needs BOTH other counters, not just
    /// `boundary_edge_count` — a mesh whose every edge is non-manifold is
    /// watertight with zero boundary edges and still judges nothing. When
    /// [`Self::is_printable`] holds on a non-empty shell every edge has exactly
    /// two faces, so the verdict is never vacuous there.
    ///
    /// ⚠ The EMPTY shell is the deliberate exception: it reports `false` here,
    /// and for watertight and manifold too, without consulting the census. A
    /// mesh with no faces is refused rather than passed on a technicality.
    ///
    /// ⚠ This is a **local** property. It is `true` for a shell whose faces are
    /// uniformly wound the *wrong* way — a global flip leaves every edge in
    /// agreement. `mesh_repair::MeshReport::is_inside_out` is the instrument for
    /// that, and it is not part of this result.
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
    /// Watertight **and** manifold **and** locally consistent winding.
    ///
    /// ⚠ Not "passes every check". [`Self::issues`] can be non-empty while this
    /// is `true`: `DegenerateTriangles` is reported but not consulted here, so a
    /// closed, manifold, consistently wound shell containing a zero-area face is
    /// `is_valid()`. Inspect `issues` for the complete picture.
    #[must_use]
    pub const fn is_valid(&self) -> bool {
        self.is_watertight && self.is_manifold && self.has_consistent_winding
    }

    /// Watertight **and** manifold. Nothing else.
    ///
    /// ⚠ Deliberately weaker than [`Self::is_valid`] — winding is **not** a
    /// term, so a shell whose faces disagree still reads as printable. Nor are
    /// degenerate triangles. This answers "is the surface closed and
    /// two-manifold", which is what a slicer needs to produce watertight
    /// toolpaths; it is not a statement that the result will print *well*.
    /// The three gates nest — this ⊆ [`Self::is_valid`] ⊆ an empty
    /// [`Self::issues`] — so gate on the issue list when you want the strongest
    /// statement; adding `is_valid()` to it is redundant.
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
/// Checks, each surfacing as a [`ShellIssue`]:
/// - Emptiness (no faces) — returns immediately, refusing everything
/// - Watertightness (no boundary edges)
/// - Manifoldness (no edges with >2 adjacent faces)
/// - Consistent winding order (per-edge, via [`mesh_repair::winding_census`])
/// - Degenerate triangles (zero **area**, which the winding check does not see)
///
/// ⚠ The last of these is reported but is not a term in either
/// [`ShellValidationResult::is_valid`] or
/// [`ShellValidationResult::is_printable`]. Read `issues` for the full picture.
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

    // Check for empty shell. Refuses everything below rather than passing
    // vacuously; see the note on `has_consistent_winding`.
    if shell.faces.is_empty() {
        issues.push(ShellIssue::EmptyShell);
        warn!("Shell has no faces; refusing rather than validating vacuously");
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

    // Check winding consistency. `validate_mesh` already censused the edges
    // above, so read that rather than rebuilding the edge map.
    //
    // `None` means the census was not run. Treat that as NOT consistent: it is
    // an absence of evidence, and reporting it as clean winding is exactly the
    // overclaim `WindingCensus` was introduced to end.
    //
    // ⚠ The two empty cases are deliberately NOT symmetric. `None` is "the
    // instrument never ran" and reads false. A census that ran and found no
    // judgeable edge reads TRUE, because `inconsistent_edges == 0` is then a
    // real measurement over an empty set — no edge disagrees. The field doc
    // names that vacuity so callers can rule it out; `has_judgeable_edges()`
    // is the discriminator if a caller needs to.
    let has_consistent_winding = mesh_report
        .winding
        .is_some_and(|census| census.inconsistent_edges == 0);
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

    // Report on the issue list, not on `is_printable()`. That predicate omits
    // winding and degenerate faces, so branching on it logged "validation
    // passed" for a mis-wound shell while swallowing its issues.
    if result.issues.is_empty() {
        info!("Shell validation found no issues");
    } else {
        warn!(
            "Shell validation found {} issue(s) (printable={}, valid={})",
            result.issue_count(),
            result.is_printable(),
            result.is_valid()
        );
    }

    debug!("{}", result);

    result
}

/// Count degenerate triangles in the mesh.
fn count_degenerate_triangles(mesh: &IndexedMesh) -> usize {
    const DEGENERATE_THRESHOLD: f64 = 1e-10;

    mesh.faces
        .iter()
        .filter(|face| {
            let v0 = &mesh.vertices[face[0] as usize];
            let v1 = &mesh.vertices[face[1] as usize];
            let v2 = &mesh.vertices[face[2] as usize];

            let edge1 = v1 - v0;
            let edge2 = v2 - v0;
            let cross = edge1.cross(&edge2);
            let area = cross.norm() / 2.0;

            area < DEGENERATE_THRESHOLD
        })
        .count()
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_possible_truncation
)]
mod tests {
    use super::*;
    use mesh_types::Point3;

    fn create_watertight_tetrahedron() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(10.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(5.0, 10.0, 0.0));
        mesh.vertices.push(Point3::new(5.0, 5.0, 10.0));

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
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(10.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(10.0, 10.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 10.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 0.0, 10.0));
        mesh.vertices.push(Point3::new(10.0, 0.0, 10.0));
        mesh.vertices.push(Point3::new(10.0, 10.0, 10.0));
        mesh.vertices.push(Point3::new(0.0, 10.0, 10.0));

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
    fn a_single_reversed_face_is_reported_as_inconsistent_winding() {
        // True-positive coverage for the winding verdict: take a
        // consistently-wound watertight tetrahedron and reverse a SINGLE
        // face's winding. The mesh stays watertight + manifold (same edges,
        // each still shared by exactly 2 faces) but the flipped face's three
        // edges now run the SAME direction in both incident faces, so the
        // census must fire. This guards the positive path independently of any
        // example — the `shell-generation` fold's rim-winding fix means no
        // example produces a mis-wound shell anymore.
        let mut shell = create_watertight_tetrahedron();
        let [a, b, c] = shell.faces[0];
        shell.faces[0] = [a, c, b]; // reverse winding of one face

        let result = validate_shell(&shell);

        assert!(
            result.is_watertight,
            "flipping one face keeps it watertight"
        );
        assert!(result.is_manifold, "flipping one face keeps it manifold");
        assert!(
            !result.has_consistent_winding,
            "one reversed face ⇒ inconsistent winding must be detected",
        );
        // `is_printable()` is winding-INDEPENDENT (watertight && manifold),
        // so a mis-wound shell still reads as printable — that was the whole
        // point of the old rim quirk. The stricter `is_valid()` (which adds
        // consistent winding) is what the detector guards.
        assert!(
            result.is_printable(),
            "watertight + manifold ⇒ is_printable() is true regardless of winding",
        );
        assert!(
            !result.is_valid(),
            "inconsistent winding ⇒ NOT valid (is_valid() includes winding)",
        );
        assert!(
            result
                .issues
                .iter()
                .any(|i| matches!(i, ShellIssue::InconsistentWinding)),
            "expected an InconsistentWinding issue; got: {:?}",
            result.issues,
        );
    }

    #[test]
    fn a_degenerate_face_is_reported_as_degenerate_not_as_bad_winding() {
        // Regression guard for the hand-rolled detector this check replaced.
        //
        // That version keyed edges by their sorted vertex pair. A face listing a
        // vertex twice registers the edge `(0,1)` from BOTH its `(0,1)` and
        // `(1,0)` traversals, so the face lands in that edge's bucket TWICE. The
        // bucket then has length 2 — indistinguishable from a normal interior
        // edge — and the detector compared the face against ITSELF, found the
        // directions equal, and reported inconsistent winding.
        //
        // ⚠ The fixture must be the degenerate face ALONE. Adding a second face
        // on `(0,1)` pushes the bucket to length 3, which the old code skipped
        // as non-manifold — so it returned `true` and the guard would be
        // vacuous. Measured, old vs new: `[[0,0,1]]` gives false vs true
        // (discriminates); `[[0,0,1],[0,1,2]]` gives true vs true (does not).
        //
        // `winding_census` skips a face listing a vertex twice whole, so the
        // defect is classified as what it is — a degenerate face.
        let mut shell = IndexedMesh::new();
        shell.vertices.push(Point3::new(0.0, 0.0, 0.0));
        shell.vertices.push(Point3::new(1.0, 0.0, 0.0));
        shell.vertices.push(Point3::new(0.0, 1.0, 0.0));
        shell.faces.push([0, 0, 1]);

        let result = validate_shell(&shell);

        // ⚠ The winding verdict here is VACUOUS, and that is the point worth
        // pinning: the only face is skipped whole, so NOTHING is judgeable.
        // `has_consistent_winding` is true because no edge disagrees, not
        // because winding was affirmatively verified. The old code reached the
        // opposite verdict on this very mesh, which is what makes it a guard.
        let census = mesh_repair::winding_census(&shell);
        assert_eq!(
            census.degenerate_faces, 1,
            "oracle: one repeated-index face"
        );
        assert_eq!(
            census.interior_edges, 0,
            "oracle: no judgeable edge survives, so the verdict below is vacuous",
        );

        assert!(
            result.has_consistent_winding,
            "a repeated-index face is a degenerate face, not a winding defect; got: {:?}",
            result.issues,
        );
        assert!(
            !result
                .issues
                .iter()
                .any(|i| matches!(i, ShellIssue::InconsistentWinding)),
            "no InconsistentWinding issue should be raised; got: {:?}",
            result.issues,
        );
        // Not merely "not misreported" — reported as the defect it actually is.
        // Without this the test would still pass if the degenerate face were
        // dropped from the report altogether.
        assert!(
            result
                .issues
                .iter()
                .any(|i| matches!(i, ShellIssue::DegenerateTriangles { count: 1 })),
            "the repeated-index face must surface as DegenerateTriangles; got: {:?}",
            result.issues,
        );
    }

    #[test]
    fn an_empty_issue_list_implies_both_predicates() {
        // The three gates are NESTED, not independent:
        //   is_printable() ⊆ is_valid() ⊆ issues.is_empty()
        //
        // The crate-level example relies on the outermost implication, so pin
        // it. It holds structurally — each predicate's failure pushes its own
        // issue — and would break the moment a check reported an issue without
        // a corresponding term, or gained a term without an issue.
        //
        // ⚠ One-directional. `is_valid()` does NOT imply an empty issue list;
        // `is_valid_is_true_while_an_issue_is_reported` is the counterexample.
        let mut flipped = create_watertight_tetrahedron();
        flipped.faces[0] = {
            let [a, b, c] = flipped.faces[0];
            [a, c, b]
        };
        let mut duplicated = create_watertight_tetrahedron();
        duplicated.faces = duplicated.faces.iter().flat_map(|f| [*f, *f]).collect();

        let cases: Vec<(&str, IndexedMesh)> = vec![
            ("empty", IndexedMesh::new()),
            ("clean tetrahedron", create_watertight_tetrahedron()),
            ("one face flipped", flipped),
            ("open box", create_open_box()),
            ("every face duplicated", duplicated),
        ];

        for (name, mesh) in cases {
            let result = validate_shell(&mesh);
            assert!(
                !result.issues.is_empty() || result.is_valid(),
                "{name}: empty issue list must imply is_valid(); got issues={:?} is_valid={}",
                result.issues,
                result.is_valid(),
            );
            assert!(
                !result.issues.is_empty() || result.is_printable(),
                "{name}: empty issue list must imply is_printable() too",
            );
        }
    }

    #[test]
    fn is_valid_is_true_while_an_issue_is_reported() {
        // Pins the caveat on `is_valid`: it is NOT "passes every check".
        //
        // A tetrahedron with one vertex nudged nearly onto the opposite edge
        // stays closed, manifold and consistently wound, so all three terms of
        // `is_valid` hold — while `DegenerateTriangles` is reported. A caller
        // treating `is_valid()` as "no issues" would miss it.
        let mut shell = IndexedMesh::new();
        shell.vertices.push(Point3::new(0.0, 0.0, 0.0));
        shell.vertices.push(Point3::new(1.0, 0.0, 0.0));
        shell.vertices.push(Point3::new(0.5, 1e-14, 0.0));
        shell.vertices.push(Point3::new(0.3, 0.4, 1.0));
        shell.faces = vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];

        let result = validate_shell(&shell);

        assert!(result.is_valid(), "all three terms of is_valid hold");
        assert!(
            result
                .issues
                .iter()
                .any(|i| matches!(i, ShellIssue::DegenerateTriangles { .. })),
            "yet a degenerate triangle IS reported; got: {:?}",
            result.issues,
        );
    }

    #[test]
    fn a_watertight_shell_can_still_judge_nothing() {
        // Pins the field doc's caveat: `boundary_edge_count` alone does NOT
        // rule out a vacuous winding verdict.
        //
        // A tetrahedron with every face duplicated has four incident faces on
        // every edge. That is zero boundary edges — watertight — while nothing
        // is judgeable, because the census excludes non-manifold edges.
        let mut shell = create_watertight_tetrahedron();
        shell.faces = shell.faces.iter().flat_map(|f| [*f, *f]).collect();

        let census = mesh_repair::winding_census(&shell);
        let result = validate_shell(&shell);

        assert_eq!(result.boundary_edge_count, 0, "duplicating faces closes it");
        assert!(result.is_watertight, "so it reads as watertight");
        assert_eq!(
            census.interior_edges, 0,
            "yet nothing is judgeable — every edge is non-manifold",
        );
        assert!(
            result.has_consistent_winding,
            "the verdict is therefore vacuously true, not verified",
        );
        // `is_printable` is what actually rules the vacuous case out, and here
        // it correctly refuses.
        assert!(
            !result.is_printable(),
            "not manifold ⇒ not printable, which is the caveat's escape hatch",
        );
    }

    #[test]
    fn the_two_senses_of_degenerate_in_this_module_do_not_coincide() {
        // `DegenerateTriangles` counts zero-AREA faces. The census skips faces
        // that list a vertex twice. Those are different sets, and the field doc
        // on `has_consistent_winding` says so — this pins it.
        //
        // Two slivers sharing edge (0,1): three DISTINCT indices each, area ~0.
        // Degenerate by area, invisible to the census's skip rule, and their
        // shared edge is judged normally.
        let mut shell = IndexedMesh::new();
        shell.vertices.push(Point3::new(0.0, 0.0, 0.0));
        shell.vertices.push(Point3::new(1.0, 0.0, 0.0));
        shell.vertices.push(Point3::new(0.5, 1e-12, 0.0));
        shell.vertices.push(Point3::new(0.5, -1e-12, 0.0));
        shell.faces.push([0, 1, 2]);
        shell.faces.push([1, 0, 3]);

        let census = mesh_repair::winding_census(&shell);
        assert_eq!(
            census.degenerate_faces, 0,
            "distinct indices ⇒ the census does not skip these",
        );
        assert_eq!(
            census.interior_edges, 1,
            "the shared edge is judged despite both faces having ~zero area",
        );
        assert_eq!(
            count_degenerate_triangles(&shell),
            2,
            "both faces ARE degenerate by this module's area test",
        );
    }

    #[test]
    fn the_winding_verdict_comes_from_the_census_not_a_second_edge_map() {
        // The census is computed once, by `validate_mesh`. Guard that this
        // result reports the census's verdict rather than a recomputation.
        //
        // ⚠ BOTH polarities are required. Checking only the flipped mesh lets a
        // substituted sibling field survive: on a flipped tetra `interior_edges`
        // is 6 and `inconsistent_edges` is 3, so `== 0` is false either way. The
        // clean mesh is what separates them — there `inconsistent_edges == 0` is
        // true while `interior_edges == 0` is false. (Measured: the sibling
        // mutant passed this test until the clean case was added.)
        let clean = create_watertight_tetrahedron();
        let clean_census = mesh_repair::winding_census(&clean);
        assert_eq!(
            clean_census.inconsistent_edges, 0,
            "oracle: an unmodified tetrahedron is consistently wound",
        );
        assert!(
            clean_census.interior_edges > 0,
            "the clean case must have judgeable edges, or it cannot separate \
             `inconsistent_edges` from `interior_edges`",
        );
        assert!(
            validate_shell(&clean).has_consistent_winding,
            "clean mesh must report consistent winding",
        );

        let mut flipped = create_watertight_tetrahedron();
        let [a, b, c] = flipped.faces[0];
        flipped.faces[0] = [a, c, b];
        let flipped_census = mesh_repair::winding_census(&flipped);
        assert_eq!(
            flipped_census.inconsistent_edges, 3,
            "oracle: one flipped face disagrees on its three edges",
        );
        assert!(
            !validate_shell(&flipped).has_consistent_winding,
            "flipped mesh must report inconsistent winding",
        );
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
        // Pins the documented EXCEPTION to the vacuity rule. Everywhere else a
        // census with nothing judgeable reports `true` — on an empty mesh the
        // census would too (`inconsistent_edges == 0`, measured). The early
        // return refuses instead, and never consults it. Without this assert the
        // field doc and this path could drift apart unnoticed.
        assert!(
            !result.has_consistent_winding,
            "the empty shell is refused, not passed vacuously",
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
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.faces.push([idx, idx + 1, idx + 2]);

        let count = count_degenerate_triangles(&mesh);
        assert_eq!(count, 1);
    }
}
