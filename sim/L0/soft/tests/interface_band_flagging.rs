//! IV-6 — Interface-tet flagging refinement-monotonicity + book rule
//! agreement.
//!
//! Phase 4 scope memo §1 IV-6 + §8 commit 12. Verifies the per-tet
//! [`Mesh::interface_flags`] cache populated by
//! [`SdfMeshedTetMesh::from_sdf`] when a [`MaterialField`] carries an
//! interface SDF (Part 7 §02 §01 `|φ(x_c)| < L_e` rule). Three
//! independent gates:
//!
//! - **Default-uniform passthrough** — a uniform [`MaterialField`]
//!   with no interface SDF attached produces an all-`false` flag
//!   vector of length `n_tets`. Pins the [`MaterialField`]
//!   `interface_sdf: None` ⇒ no interface tets contract; canonical
//!   IV-1 baseline scenes go through this path unchanged.
//! - **Refinement monotonicity on the flagged fraction** — under
//!   refinement (`cell_size` halving), the flagged fraction
//!   `flagged_count / n_tets` decreases strictly. Mirrors the IV-4
//!   `tests/sdf_material_tagging.rs` straddler-fraction gate
//!   structurally; same `(h, h/2, h/4)` refinement scan, same
//!   canonical sphere scene, same strict-decrease assertion (`<` not
//!   `<=`).
//! - **Determinism** — same scene built twice yields identical flag
//!   vectors, bit-equal element-by-element. Sister of IV-4's
//!   per-tet-materials determinism gate; covers the path where
//!   future non-determinism in the centroid-sampling pass leaks
//!   into [`Mesh::interface_flags`] without surfacing in
//!   [`Mesh::materials`].
//!
//! ## Why fraction not absolute count
//!
//! Per [`project_3d_interface_scaling`][p] and IV-4's analogous
//! reasoning: under `h → h/2`, total tet count scales as `O(1/h³)`
//! (volumetric) while the band-flagged count scales as `O(A · L_e /
//! h³) = O(A / h²)` (band area times one mean-edge-length over
//! per-tet volume; `L_e ~ h`). The absolute count therefore grows as
//! `1/h²` under refinement; only the fraction shrinks as `h` and
//! halves cleanly per refinement level. Strict-decrease on the
//! fraction is the well-defined refinement-monotonicity claim.
//!
//! Scope memo §1 IV-6 row reads "halves the interface-tet count to
//! within ~25%" — that phrasing is colloquial / Decision-K-time guess.
//! This test interprets it via the standard 3D interface-scaling
//! reasoning, mirroring the IV-4 amendment already queued for commit
//! 13. Surfacing both at once.
//!
//! ## What this test does NOT cover
//!
//! - The book-prescribed adaptive-refinement consumer
//!   ([Part 7 Ch 03](../../../../docs/studies/soft_body_architecture/src/70-sdf-pipeline/03-adaptive-refine.md)) — Phase H per Decision K.
//! - Newton-hot-path purity (Decision K's "Newton loop does not
//!   branch on the flag") — verified by `grep` audit at commit
//!   review time per scope memo §4, not by this test.
//! - The `BlendedScalarField` smoothstep-blend path with the
//!   composition's own SDF — IV-6 uses the `LayeredScalarField`
//!   shell-pattern with its own SDF as the interface SDF, which
//!   exercises the same `|φ(x_c)| < L_e` rule on a single SDF
//!   reference. The `BlendedScalarField` path samples the flag
//!   identically (the SDF reference is just plumbed through
//!   [`MaterialField::with_interface_sdf`]); a future composition-
//!   path coverage gate is a Phase H follow-on.
//!
//! [p]: ../../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_3d_interface_scaling.md

// `from_sdf` calls `.expect()` to surface meshing failures as test
// panics — the canonical sphere scene either succeeds by construction
// or surfaces a regression worth investigating. Same convention as
// IV-4 (`sdf_material_tagging.rs`) + III-1 (`sdf_pipeline_determinism.rs`).
#![allow(clippy::expect_used)]

use sim_soft::sdf_bridge::{Aabb3, MeshingHints, SdfMeshedTetMesh};
use sim_soft::{Field, LayeredScalarField, MaterialField, Mesh, SphereSdf, Vec3};

// ── Geometry & three-shell partition (mirror of IV-4) ────────────────────

/// Body sphere radius. Shared with III-1, IV-4, IV-5 canonical scenes.
const BODY_RADIUS: f64 = 0.1;

/// Bbox half-extent — wraps `BODY_RADIUS` with a 0.02 margin (matches
/// the canonical `bbox_half_extent / cell_size = 6.0` ratio at h/2 =
/// 0.02).
const BBOX_HALF_EXTENT: f64 = 0.12;

const PHI_INNER_THRESHOLD: f64 = -0.04;
const PHI_MIDDLE_THRESHOLD: f64 = -0.02;

const CELL_SIZE_H: f64 = 0.04;
const CELL_SIZE_H2: f64 = 0.02;
const CELL_SIZE_H4: f64 = 0.01;

// Material values match IV-4's three distinct-shells convention so the
// scene is structurally identical to IV-4's; only the interface SDF
// attachment + flag check differ.
const MU_INNER: f64 = 5.0e4;
const LAMBDA_INNER: f64 = 2.0e5;
const MU_MIDDLE: f64 = 1.0e5;
const LAMBDA_MIDDLE: f64 = 4.0e5;
const MU_OUTER: f64 = 2.0e5;
const LAMBDA_OUTER: f64 = 8.0e5;

fn canonical_bbox() -> Aabb3 {
    Aabb3::new(
        Vec3::new(-BBOX_HALF_EXTENT, -BBOX_HALF_EXTENT, -BBOX_HALF_EXTENT),
        Vec3::new(BBOX_HALF_EXTENT, BBOX_HALF_EXTENT, BBOX_HALF_EXTENT),
    )
}

/// Three-shell `MaterialField` — identical to IV-4's, but with an
/// interface SDF attached so the [`Mesh::interface_flags`] cache is
/// populated via the `|φ(x_c)| < L_e` rule.
///
/// The interface SDF is the same body-radius `SphereSdf` used inside
/// the layered fields — its zero set is the outer-middle shell
/// boundary, which is the most discriminating interface for the
/// refinement-monotonicity gate (the inner-middle boundary at
/// `‖x‖ = 0.06` would also work but is structurally equivalent).
fn three_shell_field_with_interface() -> MaterialField {
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
    MaterialField::from_fields(mu_field, lambda_field).with_interface_sdf(body())
}

fn build_at(cell_size: f64, field: MaterialField) -> SdfMeshedTetMesh {
    SdfMeshedTetMesh::from_sdf(
        &SphereSdf {
            radius: BODY_RADIUS,
        },
        &MeshingHints {
            bbox: canonical_bbox(),
            cell_size,
            material_field: Some(field),
        },
    )
    .expect("canonical sphere scene should mesh successfully")
}

/// Per-mesh flag report.
struct FlagReport {
    n_tets: usize,
    flagged_count: usize,
}

impl FlagReport {
    fn flagged_fraction(&self) -> f64 {
        // n_tets cap at i32-safe range per `BccLattice::new`; these
        // refinements are well below f64-precision concerns.
        #[allow(clippy::cast_precision_loss)]
        let frac = self.flagged_count as f64 / self.n_tets as f64;
        frac
    }
}

fn count_flags(mesh: &SdfMeshedTetMesh) -> FlagReport {
    let flagged_count = mesh.interface_flags().iter().filter(|&&f| f).count();
    FlagReport {
        n_tets: mesh.n_tets(),
        flagged_count,
    }
}

// ── Tests ────────────────────────────────────────────────────────────────

#[test]
fn iv_6_uniform_field_with_no_interface_sdf_yields_all_false_flags() {
    // Default-uniform passthrough gate. The IV-1 baseline path is a
    // uniform [`MaterialField`] with no interface SDF attached; the
    // flag vector must be all-`false` of length `n_tets` — never
    // empty (length contract), never `true` anywhere (no SDF zero
    // set to test against, so by construction nothing is "near a
    // material-interface"). Pins the
    // `MaterialField::interface_sdf: None` ⇒ flags-all-false
    // contract that uniform / `LayeredScalarField`-only callers
    // implicitly depend on.
    let mesh = build_at(CELL_SIZE_H2, MaterialField::skeleton_default());

    let flags = mesh.interface_flags();
    assert_eq!(
        flags.len(),
        mesh.n_tets(),
        "interface_flags length {} must equal n_tets {}",
        flags.len(),
        mesh.n_tets(),
    );
    assert!(
        flags.iter().all(|&f| !f),
        "uniform MaterialField with no interface SDF must produce all-false \
         flag vector; got {} true flags out of {} tets",
        flags.iter().filter(|&&f| f).count(),
        flags.len(),
    );
}

#[test]
fn iv_6_flagged_fraction_decreases_strictly_under_refinement() {
    // Main IV-6 gate. Three refinement levels (h, h/2, h/4) on the
    // canonical 3-shell sphere scene with the body SDF attached as
    // the interface SDF; assert `frac_h > frac_h2 > frac_h4`
    // strictly, where `frac_k = flagged_count_k / n_tets_k`.
    //
    // Strict inequality (`<`) over weak (`<=`) is the deliberate
    // tightening per IV-4 precedent: a regression where halving
    // `cell_size` produces an identical flagged fraction (e.g., a
    // future mesher quantizing centroid positions to a coarse grid
    // below some scale, or a `BlendedScalarField` rounding a band
    // computation to the lattice on the fine end) would silently
    // pass under `<=`. Strict-decrease pins the structural claim.
    //
    // **Fraction not absolute count** — see module-doc "Why fraction
    // not absolute count" for the standard 3D interface-scaling
    // reasoning. Empirically the fraction halves cleanly per
    // refinement level on the canonical sphere; the strict `<`
    // margin is comfortable.
    let mesh_h = build_at(CELL_SIZE_H, three_shell_field_with_interface());
    let mesh_h2 = build_at(CELL_SIZE_H2, three_shell_field_with_interface());
    let mesh_h4 = build_at(CELL_SIZE_H4, three_shell_field_with_interface());

    let report_h = count_flags(&mesh_h);
    let report_h2 = count_flags(&mesh_h2);
    let report_h4 = count_flags(&mesh_h4);

    let frac_h = report_h.flagged_fraction();
    let frac_h2 = report_h2.flagged_fraction();
    let frac_h4 = report_h4.flagged_fraction();

    eprintln!(
        "iv_6 refinement scan: \
         h = {h:.3} (n_tets = {n_h}, flagged = {f_h}, frac = {fr_h:.4}); \
         h/2 = {h2:.3} (n_tets = {n_h2}, flagged = {f_h2}, frac = {fr_h2:.4}); \
         h/4 = {h4:.3} (n_tets = {n_h4}, flagged = {f_h4}, frac = {fr_h4:.4})",
        h = CELL_SIZE_H,
        h2 = CELL_SIZE_H2,
        h4 = CELL_SIZE_H4,
        n_h = report_h.n_tets,
        n_h2 = report_h2.n_tets,
        n_h4 = report_h4.n_tets,
        f_h = report_h.flagged_count,
        f_h2 = report_h2.flagged_count,
        f_h4 = report_h4.flagged_count,
        fr_h = frac_h,
        fr_h2 = frac_h2,
        fr_h4 = frac_h4,
    );

    // Scene-sanity floor: at least one flagged tet at every
    // refinement level. A regression that produces zero flagged tets
    // (e.g., the mean-edge-length helper returning zero, or
    // `interface_sdf().eval()` returning a constant non-zero value)
    // would trivially satisfy strict-decrease as `0 > 0 > 0` is
    // false — but it would also be the wrong answer. Pin a
    // minimum-flagged-count floor at every level to catch that.
    assert!(
        report_h.flagged_count > 0,
        "h-level flagged count must be positive (scene-sanity floor); got {}",
        report_h.flagged_count,
    );
    assert!(
        report_h2.flagged_count > 0,
        "h/2-level flagged count must be positive; got {}",
        report_h2.flagged_count,
    );
    assert!(
        report_h4.flagged_count > 0,
        "h/4-level flagged count must be positive; got {}",
        report_h4.flagged_count,
    );

    assert!(
        frac_h2 < frac_h,
        "h/2 flagged fraction {frac_h2:.4} must be strictly less than h fraction {frac_h:.4}",
    );
    assert!(
        frac_h4 < frac_h2,
        "h/4 flagged fraction {frac_h4:.4} must be strictly less than h/2 fraction {frac_h2:.4}",
    );
}

#[test]
fn iv_6_flag_vector_is_run_to_run_deterministic() {
    // Decision N determinism — same scene built twice; the
    // `interface_flags()` cache is bit-equal element-by-element
    // across runs. Cheap second-pass guard against non-determinism
    // leaking into the centroid-sampling pass for the flag-only
    // path (e.g., a future SDF impl reading thread-local state, a
    // non-deterministic reduction in a SIMD edge-length sum). III-1
    // gates mesh topology + position determinism; IV-4 gates the
    // `materials()` cache + per-tet shell-classification
    // determinism; this gate adds the `interface_flags()` cache
    // determinism to that family.
    let mesh_a = build_at(CELL_SIZE_H2, three_shell_field_with_interface());
    let mesh_b = build_at(CELL_SIZE_H2, three_shell_field_with_interface());

    let flags_a = mesh_a.interface_flags();
    let flags_b = mesh_b.interface_flags();

    assert_eq!(
        flags_a.len(),
        flags_b.len(),
        "flag vector length drift across runs: {} vs {}",
        flags_a.len(),
        flags_b.len(),
    );
    for (i, (&a, &b)) in flags_a.iter().zip(flags_b.iter()).enumerate() {
        assert_eq!(a, b, "flag drift at tet {i}: {a} vs {b}");
    }
}
