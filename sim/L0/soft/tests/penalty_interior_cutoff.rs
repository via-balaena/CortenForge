//! Foundation patch for the cortenforge sliding-intruder FEM ramp —
//! verify [`PenaltyRigidContact::with_params_and_interior_cutoff`] +
//! the active-set walk's `sd < -interior_cutoff` filter at
//! [`ActivePairsFor::active_pairs`] and
//! [`PenaltyRigidContact::per_pair_readout`].
//!
//! Motivating consumer + design rationale: cortenforge cf-device-design
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
    // Three vertices: deep-interior (excluded), in-band (kept), above
    // band (filtered by d̂-gate, not the cutoff). The cutoff path and
    // the band-gate path each contribute one filtered vertex; only the
    // middle vertex survives both.
    let positions = [
        Vec3::new(0.0, 0.0, -10.0e-3),    // sd = -10 mm — cutoff excludes
        Vec3::new(0.0, 0.0, -3.0e-3),     // sd = -3 mm — kept
        Vec3::new(0.0, 0.0, 5.0 * D_HAT), // sd = +5 mm — d̂-gate filters
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
