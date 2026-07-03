//! Foundation patch for the cortenforge sliding-intruder FEM ramp —
//! verify [`PenaltyRigidContact::with_params_and_interior_cutoff`] +
//! the active-set walk's `sd < -interior_cutoff` filter at
//! [`ActivePairsFor::active_pairs`] and
//! [`PenaltyRigidContact::per_pair_readout`].
//!
//! Motivating consumer + design rationale: cortenforge cf-sim-research
//! sliding-intruder FEM ramp (see
//! `docs/SIM_ARC_SLIDING_INTRUDER_CONTACT_RECON.md`). A `TransformedSdf`
//! of a flood-fill-signed closed-body `GridSdf` can return `sd << 0` at
//! pose-dependent BCC vertex positions that project into the static
//! body interior; without the cutoff, those pairs generate force
//! `κ·(d̂ − sd)·n` with `(d̂ − sd)` in tens of millimeters and break
//! Newton convergence. The cutoff silently excludes such pairs at the
//! active-set walk; elastic restoring forces move the vertex back into
//! the contact band, at which point normal penalty contact resumes.
//!
//! These tests pin the smallest viable surface: cutoff filters deep-
//! interior pairs, leaves in-band pairs alone, and behaves identically
//! on [`ActivePairsFor::active_pairs`] and
//! [`PenaltyRigidContact::per_pair_readout`] (the two active-set walks
//! the cutoff guards).

use sim_soft::{
    ActivePairsFor, ContactPair, MaterialField, PenaltyRigidContact, RigidPlane, SingleTetMesh,
    Vec3,
};

/// Fixture-local penalty stiffness (mirrors `PENALTY_KAPPA_DEFAULT`).
const KAPPA: f64 = 1.0e4;
/// Fixture-local contact band — 1 mm, same as `PENALTY_DHAT_DEFAULT`.
const D_HAT: f64 = 1.0e-3;
/// Interior cutoff — 5 mm. Excludes pairs with `sd < -5 mm`.
const INTERIOR_CUTOFF: f64 = 5.0e-3;

/// A `+z`-normal plane through the origin — vertex at `(_, _, z)` has
/// `sd = z`, so the test fixture controls `sd` directly through the
/// vertex's z-coordinate.
fn z_floor_with_cutoff() -> PenaltyRigidContact {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
    PenaltyRigidContact::with_params_and_interior_cutoff(vec![plane], KAPPA, D_HAT, INTERIOR_CUTOFF)
}

#[test]
fn penalty_with_interior_cutoff_excludes_deep_interior_pair() {
    let c = z_floor_with_cutoff();
    let mesh = SingleTetMesh::new(&MaterialField::skeleton_default());
    // One vertex at z = -10 mm — sd = -10 mm, well past `-INTERIOR_CUTOFF
    // = -5 mm`. Must be excluded.
    let positions = [Vec3::new(0.0, 0.0, -10.0e-3)];
    let pairs = c.active_pairs(&mesh, &positions);
    assert!(
        pairs.is_empty(),
        "interior cutoff at -5 mm must exclude a vertex at sd = -10 mm, got {} pairs",
        pairs.len(),
    );
}

#[test]
fn penalty_with_interior_cutoff_keeps_within_band_pair() {
    let c = z_floor_with_cutoff();
    let mesh = SingleTetMesh::new(&MaterialField::skeleton_default());
    // One vertex at z = -3 mm — sd = -3 mm. In the contact band
    // (sd < d̂ = 1 mm) and above the cutoff (sd > -5 mm). Must be
    // included.
    let positions = [Vec3::new(0.0, 0.0, -3.0e-3)];
    let pairs = c.active_pairs(&mesh, &positions);
    assert_eq!(
        pairs.len(),
        1,
        "interior cutoff at -5 mm must KEEP a vertex at sd = -3 mm, got {} pairs",
        pairs.len(),
    );
    let &ContactPair::Vertex {
        vertex_id,
        primitive_id,
    } = &pairs[0];
    assert_eq!((vertex_id, primitive_id), (0, 0));
}

#[test]
fn penalty_per_pair_readout_respects_interior_cutoff() {
    let c = z_floor_with_cutoff();
    let mesh = SingleTetMesh::new(&MaterialField::skeleton_default());
    // `SingleTetMesh` has 4 vertices, and `per_pair_readout` now computes
    // per-vertex tributary areas over the mesh's boundary faces, so
    // `positions` must cover all 4 (its documented precondition). Pose
    // the three filter-test vertices as before — deep-interior (cutoff
    // excludes), in-band (kept), above-band (d̂-gate filters) — and park
    // the 4th vertex above the band too, so it is filtered and adds no
    // readout. The `z`-normal plane has `sd = z`, so distinct `x`/`y`
    // offsets keep `sd` controlled by `z` alone while making the tet
    // non-degenerate.
    let positions = [
        Vec3::new(0.00, 0.00, -10.0e-3),    // sd = -10 mm — cutoff excludes
        Vec3::new(0.01, 0.00, -3.0e-3),     // sd = -3 mm — kept
        Vec3::new(0.00, 0.01, 5.0 * D_HAT), // sd = +5 mm — d̂-gate filters
        Vec3::new(0.01, 0.01, 5.0 * D_HAT), // 4th tet vertex, parked above band
    ];
    let readouts = c.per_pair_readout(&mesh, &positions);
    assert_eq!(
        readouts.len(),
        1,
        "exactly one readout (the in-band vertex at sd = -3 mm) should survive cutoff + band-gate, got {}",
        readouts.len(),
    );
    let r = &readouts[0];
    let ContactPair::Vertex {
        vertex_id,
        primitive_id,
    } = r.pair;
    assert_eq!((vertex_id, primitive_id), (1, 0));
    assert!(
        (r.sd - (-3.0e-3)).abs() < 1.0e-12,
        "kept vertex sd should be ≈ -3 mm, got {}",
        r.sd,
    );
}

/// Boundary case — vertex exactly at `sd = -cutoff` is KEPT. The
/// filter uses `sd < -c` (strict), so the cutoff value itself is
/// inclusive of the active set. A future regression to `sd <= -c`
/// would silently exclude vertices right at the boundary; this test
/// pins the strict-vs-non-strict semantics.
#[test]
fn penalty_with_interior_cutoff_keeps_pair_exactly_at_boundary() {
    let c = z_floor_with_cutoff();
    let mesh = SingleTetMesh::new(&MaterialField::skeleton_default());
    let positions = [Vec3::new(0.0, 0.0, -INTERIOR_CUTOFF)];
    let pairs = c.active_pairs(&mesh, &positions);
    assert_eq!(
        pairs.len(),
        1,
        "vertex at sd = -interior_cutoff (boundary) must be KEPT (strict < filter), got {} pairs",
        pairs.len(),
    );
}

// ---------------------------------------------------------------------
// Precondition tests for `with_params_and_interior_cutoff` — each
// `#[should_panic]` test pins one branch of the constructor's
// validation assertions.
// ---------------------------------------------------------------------

#[test]
#[should_panic(expected = "interior_cutoff must be positive and finite")]
fn penalty_with_interior_cutoff_panics_on_zero_cutoff() {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
    let _contact =
        PenaltyRigidContact::with_params_and_interior_cutoff(vec![plane], KAPPA, D_HAT, 0.0);
}

#[test]
#[should_panic(expected = "interior_cutoff must be positive and finite")]
fn penalty_with_interior_cutoff_panics_on_nan_cutoff() {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
    let _contact =
        PenaltyRigidContact::with_params_and_interior_cutoff(vec![plane], KAPPA, D_HAT, f64::NAN);
}

#[test]
#[should_panic(expected = "must be > d_hat")]
fn penalty_with_interior_cutoff_panics_when_cutoff_le_d_hat() {
    // cutoff = d̂ collapses the active band [-d̂, d̂) at its
    // boundary; cutoff < d̂ would empty it entirely. Either misuse
    // is a programmer error — locked by panic here.
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
    let _contact =
        PenaltyRigidContact::with_params_and_interior_cutoff(vec![plane], KAPPA, D_HAT, D_HAT);
}
