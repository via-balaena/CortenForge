//! IV-4 — SDF region-tagging refinement-monotonicity.
//!
//! Phase 4 scope memo §1 IV-4 + §8 commit 10. Verifies the per-tet
//! centroid-sampled `NeoHookean` cache produced by
//! [`SdfMeshedTetMesh::from_sdf`] from a 3-shell `LayeredScalarField`
//! through two independent gates:
//!
//! - **Value-correctness** — every tet's `materials()` entry bit-
//!   equals the expected `NeoHookean` for the centroid's shell, using
//!   a test-side [`shell_at`] reimplementation of the partition rule
//!   that mirrors [`LayeredScalarField`]'s
//!   `partition_point(|&t| t <= phi)`. Cross-implementation gate: any
//!   drift between the mesher's partition pass and the test's
//!   re-derivation fires loud.
//! - **Refinement monotonicity** — the count of "straddler" tets,
//!   those whose four vertex shell labels do not all agree, expressed
//!   as a fraction of total tets, strictly decreases as the BCC
//!   mesher's `cell_size` halves.
//!
//! ## What "straddler" means here
//!
//! For each tet, sample the layered scalar field at every vertex via
//! the same `partition_point(|&t| t <= phi)` rule
//! [`LayeredScalarField`] uses for centroid sampling. If all four
//! vertex labels agree the tet has a **unanimous shell**; if they
//! disagree the tet **straddles** a shell boundary — its centroid
//! lands in one shell but its volume crosses into another, which is
//! exactly the "thin band near each shell boundary" the scope memo
//! §1 IV-4 row names.
//!
//! Vertex-unanimity drives only the refinement-monotonicity gate.
//! Value-correctness uses the centroid's shell label, not the
//! vertex-unanimous shell — the concentric-shell partition is non-
//! convex (the middle shell is an annulus), so "unanimous vertices
//! ⇒ centroid in same shell" holds empirically for BCC tets at our
//! refinement levels but is not a mathematical guarantee. See
//! [`classify`] for the full rationale.
//!
//! ## Monotonicity is on the straddler *fraction*, not the count
//!
//! Scope memo §1 IV-4 row reads "misclassification count decreases
//! monotonically as the mesher's `cell_size` halves" — this test
//! interprets "count" as **fraction** (`straddlers / n_tets`) per the
//! standard 3D interface-scaling reality:
//!
//! - Total tet count scales as `O(1/h³)` (volumetric).
//! - Tets crossed by a fixed 2D shell-boundary surface scale as
//!   `O(A / h²)` (the band area divided by the per-tet face area).
//! - Therefore the **absolute straddler count grows as `1/h²`** under
//!   refinement, while the **straddler fraction shrinks as `h`** and
//!   halves cleanly per refinement level.
//!
//! Both metrics encode the same physical claim ("the interface band
//! shrinks under refinement"); only the fraction strictly decreases
//! per halving. The memo phrasing reads as colloquial — surfacing
//! this to commit 13 as a documentation-fixup candidate so §1 IV-4
//! + IV-6 disambiguate "count" → "fraction".
//!
//! The strict-decrease assertion across `(h, h/2, h/4)` catches
//! mesher regressions that introduce non-monotonic interface
//! behaviour (e.g., a sampling drift, a non-deterministic warp, a
//! coarse-grid quantization at small scales).
//!
//! ## Distinction from IV-6 (commit 12, Decision K)
//!
//! IV-4 and IV-6 are **structurally adjacent but distinct** refinement-
//! monotonicity gates on different rules:
//!
//! - **IV-4 (this file)** — vertex-unanimous shell-label rule. Pure
//!   structural detector for the centroid-sampling pipeline; no
//!   `L_e` (edge-length) dependency. Refinement-monotonic by
//!   construction. Catches per-tet sampling bugs in the mesher (e.g.,
//!   off-by-one centroid formula, swapped shell indices, threshold-list
//!   reversal).
//! - **IV-6 (commit 12, future)** — `|φ_blend(centroid)| < L_e` rule
//!   per Part 7 §02 §01 + scope memo Decision K. Diagnostic-only flag
//!   populated by the mesher and read by Part 11 mesh-quality
//!   reporting. Has explicit `L_e` (mean edge length) dependency.
//!
//! Two independent refinement-monotonicity gates on two different
//! rules, exercising overlapping code paths with no overlapping
//! assertions.
//!
//! ## Material values (deviation from Decision J)
//!
//! Decision J prescribes outer = inner = Ecoflex 00-30 baseline (1×),
//! middle = composite (2× stiffness). Under that symmetry a swap
//! between outer and inner shells would be invisible to the value-
//! correctness check, defeating one of IV-4's load-bearing diagnostic
//! roles.
//!
//! This test deviates by using three distinct values: `μ ∈ {5e4, 1e5,
//! 2e5}`, `λ ∈ {2e5, 4e5, 8e5}`, factors `0.5× / 1× / 2×` of the IV-1
//! baseline. Same compressible regime (`λ = 4 μ` ⇒ `ν = 0.4`) as every
//! prior Phase 1-4 test; discriminates against shell-swap bugs that
//! Decision J's symmetry would mask. Mirrors the IV-3 (commit 8)
//! compressible-regime ν = 0.4 deviation precedent — Decision J is
//! the layered-silicone-device end-to-end commitment, not a per-test
//! requirement.

// `from_sdf` calls `.expect()` to surface meshing failures as test
// panics — the canonical sphere scene either succeeds by construction
// or surfaces a regression worth investigating. Same convention as
// III-1 (`sdf_pipeline_determinism.rs`).
#![allow(clippy::expect_used)]

mod common;

use common::assert_neo_hookean_bit_equal;
use sim_soft::sdf_bridge::{Aabb3, MeshingHints, SdfMeshedTetMesh};
use sim_soft::{
    Field, LayeredScalarField, MaterialField, Mesh, NeoHookean, SphereSdf, TetId, Vec3,
};

// ── Geometry & shell partition ───────────────────────────────────────────

/// Body sphere radius. Shared with III-1 / commit 9 canonical scene.
const BODY_RADIUS: f64 = 0.1;

/// Bbox half-extent — wraps `BODY_RADIUS` with a 0.02 margin (matches
/// the canonical `bbox_half_extent / cell_size = 6.0` ratio at h/2 =
/// 0.02).
const BBOX_HALF_EXTENT: f64 = 0.12;

/// Inner-shell threshold on `phi = ‖x‖ − BODY_RADIUS`. Vertices with
/// `‖x‖ < 0.06` land in the inner shell.
const PHI_INNER_THRESHOLD: f64 = -0.04;

/// Middle-shell upper threshold on `phi`. Vertices with `0.06 ≤ ‖x‖ <
/// 0.08` land in the middle shell; `‖x‖ ≥ 0.08` land in outer.
const PHI_MIDDLE_THRESHOLD: f64 = -0.02;

/// Three refinement levels — `h, h/2, h/4`. Strict halving at every
/// level; `h/2 = 0.02` is the canonical Phase 3 / commit 9 scene cell
/// size, so the middle level reproduces the III-1 determinism scene.
const CELL_SIZE_H: f64 = 0.04;
const CELL_SIZE_H2: f64 = 0.02;
const CELL_SIZE_H4: f64 = 0.01;

// ── Material values per shell — three distinct (μ, λ) pairs ──────────────

const MU_INNER: f64 = 5.0e4;
const LAMBDA_INNER: f64 = 2.0e5;
const MU_MIDDLE: f64 = 1.0e5;
const LAMBDA_MIDDLE: f64 = 4.0e5;
const MU_OUTER: f64 = 2.0e5;
const LAMBDA_OUTER: f64 = 8.0e5;

// ── Helpers ──────────────────────────────────────────────────────────────

fn canonical_bbox() -> Aabb3 {
    Aabb3::new(
        Vec3::new(-BBOX_HALF_EXTENT, -BBOX_HALF_EXTENT, -BBOX_HALF_EXTENT),
        Vec3::new(BBOX_HALF_EXTENT, BBOX_HALF_EXTENT, BBOX_HALF_EXTENT),
    )
}

/// Build the canonical 3-shell `MaterialField`: a single
/// [`SphereSdf`] of radius `BODY_RADIUS` (matching the meshed body)
/// partitions space into three concentric shells via thresholds
/// `[PHI_INNER_THRESHOLD, PHI_MIDDLE_THRESHOLD]`.
fn three_shell_field() -> MaterialField {
    let body = || {
        Box::new(SphereSdf {
            radius: BODY_RADIUS,
        })
    };
    let mu_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        body(),
        vec![PHI_INNER_THRESHOLD, PHI_MIDDLE_THRESHOLD],
        vec![MU_INNER, MU_MIDDLE, MU_OUTER],
    ));
    let lambda_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        body(),
        vec![PHI_INNER_THRESHOLD, PHI_MIDDLE_THRESHOLD],
        vec![LAMBDA_INNER, LAMBDA_MIDDLE, LAMBDA_OUTER],
    ));
    MaterialField::from_fields(mu_field, lambda_field)
}

fn build_at(cell_size: f64) -> SdfMeshedTetMesh {
    SdfMeshedTetMesh::from_sdf(
        &SphereSdf {
            radius: BODY_RADIUS,
        },
        &MeshingHints {
            bbox: canonical_bbox(),
            cell_size,
            material_field: Some(three_shell_field()),
        },
    )
    .expect("canonical sphere scene should mesh successfully")
}

/// Shell label for a single point under the partition rule
/// [`LayeredScalarField`] uses internally
/// (`partition_point(|&t| t <= phi)` ⇒ at exactly `phi == threshold`
/// the point lands in the outer shell). Returns `0` for inner, `1`
/// for middle, `2` for outer — matching the `values` slot indexing of
/// the `LayeredScalarField` constructed in [`three_shell_field`].
fn shell_at(p: Vec3) -> usize {
    let phi = p.norm() - BODY_RADIUS;
    if phi < PHI_INNER_THRESHOLD {
        0
    } else if phi < PHI_MIDDLE_THRESHOLD {
        1
    } else {
        2
    }
}

/// Per-mesh classification report. `unanimous_count[s]` counts tets
/// whose four vertex shell labels all agree on shell `s`.
/// `straddler_count` counts tets whose vertices span two or more
/// shells — the misclassification metric this gate tracks.
struct ClassificationReport {
    n_tets: usize,
    unanimous_count: [usize; 3],
    straddler_count: usize,
}

impl ClassificationReport {
    /// Straddler fraction. `n_tets > 0` is guaranteed at every
    /// refinement level we run by [`build_at`]'s
    /// `MeshingError::EmptyMesh` short-circuit.
    fn straddler_fraction(&self) -> f64 {
        // n_tets cap at i32-safe range per `BccLattice::new`; these
        // refinements are well below f64-precision concerns.
        #[allow(clippy::cast_precision_loss)]
        let frac = self.straddler_count as f64 / self.n_tets as f64;
        frac
    }
}

/// Walk every tet performing two independent checks:
///
/// 1. **Value-correctness** (per-tet): the mesher's centroid-sampled
///    `materials()[tet_id]` must bit-equal the expected `NeoHookean`
///    for `shell_at(centroid)`. This is a cross-implementation gate
///    between the mesher's [`LayeredScalarField`]
///    `partition_point(|&t| t <= phi)` and the test's [`shell_at`]
///    if-else: both implement the same partition rule, so they must
///    agree on every centroid by construction. Any mismatch is a
///    drift between the two implementations or a centroid-formula
///    regression.
/// 2. **Refinement-monotonicity classification** (per-tet): the four
///    vertex shell labels are computed and compared. If they all
///    agree the tet is **unanimous**; if they disagree the tet
///    **straddles** a shell boundary. Returned as aggregate counts
///    that the refinement-monotonicity gate operates on; not used
///    for value-correctness.
///
/// The two concerns are deliberately separated. The mesher samples
/// at the centroid, so the value-correctness ground truth is the
/// centroid's shell label — *not* the vertex-unanimous shell. The
/// concentric-shell partition is non-convex (the middle shell is an
/// annulus), so vertex-unanimity is **not** a sound substitute for
/// centroid-shell at the value-correctness level. (Empirically for
/// BCC tets at our refinement levels the two coincide because the
/// centroid stays close to the local vertex cluster, but pinning
/// value-correctness on vertex-unanimity would be relying on that
/// empirical coincidence rather than the partition rule itself.)
fn classify(mesh: &SdfMeshedTetMesh) -> ClassificationReport {
    let positions = mesh.positions();
    let mut unanimous_count = [0usize; 3];
    let mut straddler_count = 0usize;

    let expected = [
        NeoHookean::from_lame(MU_INNER, LAMBDA_INNER),
        NeoHookean::from_lame(MU_MIDDLE, LAMBDA_MIDDLE),
        NeoHookean::from_lame(MU_OUTER, LAMBDA_OUTER),
    ];

    // n_tets returns usize; TetId is u32. Canonical sphere stays well
    // below u32::MAX even at h/4.
    #[allow(clippy::cast_possible_truncation)]
    let n_tets = mesh.n_tets() as TetId;
    for tet_id in 0..n_tets {
        let vids = mesh.tet_vertices(tet_id);
        let v0 = positions[vids[0] as usize];
        let v1 = positions[vids[1] as usize];
        let v2 = positions[vids[2] as usize];
        let v3 = positions[vids[3] as usize];

        // Value-correctness — cross-impl agreement at the centroid.
        let centroid = (v0 + v1 + v2 + v3) * 0.25;
        let s_centroid = shell_at(centroid);
        assert_neo_hookean_bit_equal(&mesh.materials()[tet_id as usize], &expected[s_centroid]);

        // Refinement-monotonicity classification — vertex-unanimity.
        let s0 = shell_at(v0);
        let s1 = shell_at(v1);
        let s2 = shell_at(v2);
        let s3 = shell_at(v3);
        if s0 == s1 && s1 == s2 && s2 == s3 {
            unanimous_count[s0] += 1;
        } else {
            straddler_count += 1;
        }
    }

    ClassificationReport {
        n_tets: mesh.n_tets(),
        unanimous_count,
        straddler_count,
    }
}

// ── Tests ────────────────────────────────────────────────────────────────

#[test]
fn iv_4_shell_populations_are_non_empty_at_finest() {
    // Scene-sanity guard. At finest refinement (h/4 = 0.01) all three
    // shells must produce at least one unanimous-vertex tet —
    // confirming the geometry / threshold combination admits a non-
    // trivial bulk population for each shell at the resolution the
    // test runs. Otherwise the test scene is degenerate (e.g., a
    // middle annulus too thin to admit any tet whose four vertices
    // all land inside it), and the refinement-monotonicity gate's
    // discriminating power on that shell collapses.
    //
    // The guard runs at `h/4` not coarsest because `h = 0.04` is too
    // coarse for the middle and outer shells to admit unanimous tets:
    // at h, only 48 unanimous-inner survive and the rest of the body's
    // tets straddle at least one shell boundary. At `h/2 = 0.02` (the
    // canonical Phase 3 / commit 9 cell size) all three shells have
    // hundreds of unanimous tets, and at `h/4` they have thousands —
    // a well-resolved scene at the finer levels.
    let mesh = build_at(CELL_SIZE_H4);
    let report = classify(&mesh);

    eprintln!(
        "iv_4 finest (cell_size = {h}, n_tets = {n}): unanimous = [inner: {ui}, middle: \
         {um}, outer: {uo}], straddlers = {s} (fraction = {f:.4})",
        h = CELL_SIZE_H4,
        n = report.n_tets,
        ui = report.unanimous_count[0],
        um = report.unanimous_count[1],
        uo = report.unanimous_count[2],
        s = report.straddler_count,
        f = report.straddler_fraction(),
    );

    assert!(
        report.unanimous_count[0] > 0,
        "expected at least one unanimous-inner tet at finest refinement; got {} (degenerate \
         test scene — inner shell admits no unanimous-vertex tet)",
        report.unanimous_count[0],
    );
    assert!(
        report.unanimous_count[1] > 0,
        "expected at least one unanimous-middle tet at finest refinement; got {}",
        report.unanimous_count[1],
    );
    assert!(
        report.unanimous_count[2] > 0,
        "expected at least one unanimous-outer tet at finest refinement; got {}",
        report.unanimous_count[2],
    );
}

#[test]
fn iv_4_misclassification_fraction_decreases_strictly_under_refinement() {
    // Main IV-4 gate. Three refinement levels (h, h/2, h/4) on the
    // same canonical sphere; assert
    // `frac_h > frac_h2 > frac_h4` strictly, where `frac_k =
    // straddler_count_k / n_tets_k`. Each `classify` call also asserts
    // per-tet value-correctness across every tet in the mesh
    // (mesher's centroid-sampled `materials()` must bit-equal the
    // expected `NeoHookean` for `shell_at(centroid)`) — that's the
    // per-level value-correctness gate, with refinement-monotonicity
    // on the fraction as the structural gate above it.
    //
    // Strict inequality (`<`) over weak (`<=`) is the deliberate
    // tightening: a regression where halving `cell_size` produces an
    // identical straddler fraction (e.g., a future mesher quantizing
    // vertex positions to a coarse grid below some scale, or a warp
    // step that snaps to the lattice on the fine end) would silently
    // pass under `<=`. Strict-decrease pins the structural claim.
    //
    // **Fraction not absolute count** — see module-doc "Monotonicity is
    // on the straddler *fraction*, not the count" for the standard 3D
    // interface-scaling reasoning. Empirically the fraction halves
    // cleanly per refinement level on the canonical sphere, so the
    // strict `<` margin is comfortable.
    let mesh_h = build_at(CELL_SIZE_H);
    let mesh_h2 = build_at(CELL_SIZE_H2);
    let mesh_h4 = build_at(CELL_SIZE_H4);

    let report_h = classify(&mesh_h);
    let report_h2 = classify(&mesh_h2);
    let report_h4 = classify(&mesh_h4);

    let frac_h = report_h.straddler_fraction();
    let frac_h2 = report_h2.straddler_fraction();
    let frac_h4 = report_h4.straddler_fraction();

    eprintln!(
        "iv_4 refinement scan: \
         h = {h:.3} (n_tets = {n_h}, straddlers = {s_h}, frac = {f_h:.4}, unanimous = \
         [{u0_h}, {u1_h}, {u2_h}]); \
         h/2 = {h2:.3} (n_tets = {n_h2}, straddlers = {s_h2}, frac = {f_h2:.4}, unanimous = \
         [{u0_h2}, {u1_h2}, {u2_h2}]); \
         h/4 = {h4:.3} (n_tets = {n_h4}, straddlers = {s_h4}, frac = {f_h4:.4}, unanimous = \
         [{u0_h4}, {u1_h4}, {u2_h4}])",
        h = CELL_SIZE_H,
        h2 = CELL_SIZE_H2,
        h4 = CELL_SIZE_H4,
        n_h = report_h.n_tets,
        n_h2 = report_h2.n_tets,
        n_h4 = report_h4.n_tets,
        s_h = report_h.straddler_count,
        s_h2 = report_h2.straddler_count,
        s_h4 = report_h4.straddler_count,
        f_h = frac_h,
        f_h2 = frac_h2,
        f_h4 = frac_h4,
        u0_h = report_h.unanimous_count[0],
        u1_h = report_h.unanimous_count[1],
        u2_h = report_h.unanimous_count[2],
        u0_h2 = report_h2.unanimous_count[0],
        u1_h2 = report_h2.unanimous_count[1],
        u2_h2 = report_h2.unanimous_count[2],
        u0_h4 = report_h4.unanimous_count[0],
        u1_h4 = report_h4.unanimous_count[1],
        u2_h4 = report_h4.unanimous_count[2],
    );

    assert!(
        frac_h2 < frac_h,
        "h/2 straddler fraction {frac_h2:.4} must be strictly less than h fraction \
         {frac_h:.4}",
    );
    assert!(
        frac_h4 < frac_h2,
        "h/4 straddler fraction {frac_h4:.4} must be strictly less than h/2 fraction \
         {frac_h2:.4}",
    );
}

#[test]
fn iv_4_misclassification_count_is_run_to_run_deterministic() {
    // Decision N determinism — same scene built twice; the
    // `materials()` cache is bit-equal across runs and the
    // misclassification count is identical. Cheap second-pass guard
    // against non-determinism leaking into the centroid-sampling pass
    // (e.g., a `HashMap` iteration order in a future field type, a
    // non-deterministic reduction in a SIMD path). III-1 already
    // gates mesh topology + position determinism; this gate adds
    // `materials()` cache + classification-count determinism.
    let mesh_a = build_at(CELL_SIZE_H2);
    let mesh_b = build_at(CELL_SIZE_H2);

    let report_a = classify(&mesh_a);
    let report_b = classify(&mesh_b);

    assert_eq!(
        report_a.straddler_count,
        report_b.straddler_count,
        "straddler count drift across runs: {a} vs {b}",
        a = report_a.straddler_count,
        b = report_b.straddler_count,
    );
    for s in 0..3 {
        assert_eq!(
            report_a.unanimous_count[s],
            report_b.unanimous_count[s],
            "unanimous count drift at shell {s} across runs: {a} vs {b}",
            a = report_a.unanimous_count[s],
            b = report_b.unanimous_count[s],
        );
    }

    assert_eq!(mesh_a.materials().len(), mesh_b.materials().len());
    for (nh_a, nh_b) in mesh_a.materials().iter().zip(mesh_b.materials().iter()) {
        assert_neo_hookean_bit_equal(nh_a, nh_b);
    }
}
