//! Blended-composition path through the mesher — per-tet `materials()` and
//! `interface_flags()` correctness for a [`BlendedScalarField`]-backed
//! [`MaterialField`].
//!
//! Closes two coverage gaps that the existing Phase-4 gates leave open:
//!
//! - **IV-4** (`tests/sdf_material_tagging.rs`) drives the per-tet
//!   `materials()` cache end-to-end, but only through a
//!   [`LayeredScalarField`] (sharp 3-shell partition). The smoothstep
//!   [`BlendedScalarField`] composition — where each tet's `(μ, λ)`
//!   *grades continuously* across a transition band — is never meshed
//!   and checked.
//! - **IV-6** (`tests/interface_band_flagging.rs`) checks the
//!   `interface_flags()` cache only in aggregate (all-false passthrough,
//!   refinement-monotonic flagged *fraction*, run-to-run determinism);
//!   it never asserts the per-tet book rule `|φ(x_c)| < L_e` for any
//!   field, and its module doc explicitly defers the
//!   `BlendedScalarField`-composition path to a follow-on.
//!
//! ## Rule-B oracle discipline
//!
//! These tests read the library outputs (`mesh.materials()`,
//! `mesh.interface_flags()`) and check them against **independent**
//! oracles — the smoothstep *spec* and structural properties — never a
//! byte-for-byte re-implementation of the library's own arithmetic:
//!
//! - **Snap-to-endpoints** ([`blended_materials_snap_to_endpoints_outside_band`])
//!   uses the smoothstep contract "outside the band the blend clamps to
//!   the endpoint field exactly" (`s = 0 ⇒ inside`, `s = 1 ⇒ outside`),
//!   comparing the meshed Lamé values against the raw endpoint constants
//!   — not against a re-derived FMA blend.
//! - **Grading + monotonicity** ([`blended_materials_grade_monotonically_across_band`])
//!   asserts two structural properties of the observed `μ` field: at
//!   least one band tet lands *strictly between* the endpoints (the blend
//!   is graded, not a step — the behavioural distinction from
//!   `LayeredScalarField`), and `μ` is spatially monotone in the signed
//!   distance. Both read `NeoHookean::mu()` directly; the oracle is the
//!   monotone-composition property, not the blend arithmetic.
//! - **Interface book rule** ([`blended_interface_flags_match_book_rule_per_tet`])
//!   re-derives `L_e` with an *independently ordered* six-edge sum and
//!   asserts the per-tet `|φ| < L_e` biconditional away from the
//!   measure-zero `|φ| == L_e` knife edge (where the two summation orders
//!   may disagree by rounding).

// `from_sdf` calls `.expect()` to surface meshing failures as test
// panics — the canonical sphere scene either succeeds by construction or
// surfaces a regression worth investigating. Same convention as IV-4
// (`sdf_material_tagging.rs`) + IV-6 (`interface_band_flagging.rs`).
#![allow(clippy::expect_used)]
// `n_tets()` (usize) is walked as `TetId` (u32); the canonical sphere at
// `cell_size = 0.02` stays far below `u32::MAX`. Standard Mesh-trait API
// tax mirrored across the workspace.
#![allow(clippy::cast_possible_truncation)]

use sim_soft::sdf_bridge::{Aabb3, MeshingHints, SdfMeshedTetMesh};
use sim_soft::{
    BlendedScalarField, ConstantField, Field, MaterialField, Mesh, SphereSdf, TetId, Vec3,
};

// ── Canonical Blended scene ──────────────────────────────────────────────

/// Solid body sphere radius. Shared with III-1 / IV-4 / IV-6 canonical
/// scenes.
const BODY_RADIUS: f64 = 0.1;

/// Bbox half-extent — the canonical `bbox_half_extent / cell_size = 6.0`
/// ratio at `cell_size = h/2 = 0.02`.
const BBOX_HALF_EXTENT: f64 = 0.12;

/// BCC lattice spacing — III-1 / IV-4 h/2 canonical.
const CELL_SIZE: f64 = 0.02;

/// Interior interface radius — the zero set of the smoothstep blend AND
/// the SDF attached for interface flagging. Strictly interior so the band
/// `[R_INTERFACE − BAND, R_INTERFACE + BAND]` fits inside the body.
const R_INTERFACE: f64 = 0.07;

/// Smoothstep transition band half-width. `[0.055, 0.085] ⊂ (0, 0.1)`.
const BAND: f64 = 0.015;

// Endpoint Lamé pairs: `0.5×` inside, `2×` outside the IV-1 baseline
// `(1e5, 4e5)`; both in the `λ = 4μ` (ν = 0.4) compressible regime.
const MU_INNER: f64 = 5.0e4;
const LAMBDA_INNER: f64 = 2.0e5;
const MU_OUTER: f64 = 2.0e5;
const LAMBDA_OUTER: f64 = 8.0e5;

fn canonical_bbox() -> Aabb3 {
    Aabb3::new(
        Vec3::new(-BBOX_HALF_EXTENT, -BBOX_HALF_EXTENT, -BBOX_HALF_EXTENT),
        Vec3::new(BBOX_HALF_EXTENT, BBOX_HALF_EXTENT, BBOX_HALF_EXTENT),
    )
}

/// Two [`BlendedScalarField`]s (one per Lamé parameter) keyed on an
/// interior `SphereSdf { radius: R_INTERFACE }`, blending the inner
/// endpoint into the outer endpoint over `BAND`. The same interior sphere
/// is attached via [`MaterialField::with_interface_sdf`] so the
/// `interface_flags()` cache populates (harmless for the material tests,
/// required for the flag test).
fn blended_field() -> MaterialField {
    let interface = || {
        Box::new(SphereSdf {
            radius: R_INTERFACE,
        })
    };
    let mu_field: Box<dyn Field<f64>> = Box::new(BlendedScalarField::new(
        interface(),
        Box::new(ConstantField::new(MU_INNER)),
        Box::new(ConstantField::new(MU_OUTER)),
        BAND,
    ));
    let lambda_field: Box<dyn Field<f64>> = Box::new(BlendedScalarField::new(
        interface(),
        Box::new(ConstantField::new(LAMBDA_INNER)),
        Box::new(ConstantField::new(LAMBDA_OUTER)),
        BAND,
    ));
    MaterialField::from_fields(mu_field, lambda_field).with_interface_sdf(interface())
}

fn build_blended_mesh() -> SdfMeshedTetMesh {
    SdfMeshedTetMesh::from_sdf(
        &SphereSdf {
            radius: BODY_RADIUS,
        },
        &MeshingHints {
            bbox: canonical_bbox(),
            cell_size: CELL_SIZE,
            material_field: Some(blended_field()),
        },
    )
    .expect("canonical blended-sphere scene should mesh successfully")
}

/// Tet centroid — the canonical evaluation point the mesher samples the
/// material field and interface SDF at (`material_field.rs::cache_walk`,
/// `mesh/mod.rs::interface_flags_from_field`).
fn centroid(mesh: &SdfMeshedTetMesh, tet_id: TetId) -> Vec3 {
    let positions = mesh.positions();
    let [a, b, c, d] = mesh.tet_vertices(tet_id);
    (positions[a as usize] + positions[b as usize] + positions[c as usize] + positions[d as usize])
        * 0.25
}

// ── Tests ────────────────────────────────────────────────────────────────

#[test]
fn blended_materials_snap_to_endpoints_outside_band() {
    // Smoothstep contract: outside the transition band the blend clamps
    // to the endpoint field bit-exactly. For `φ ≤ −BAND` the weight
    // `s = 0` so the mesh material is the inside endpoint; for `φ ≥ +BAND`
    // the weight `s = 1` so it is the outside endpoint. The oracle is the
    // endpoint constants — no smoothstep re-derivation — so a regression
    // in the blend's snap behaviour (a mis-clamp, a swapped endpoint)
    // fires here.
    let mesh = build_blended_mesh();
    let materials = mesh.materials();
    let n_tets = mesh.n_tets() as TetId;

    let mut n_inside = 0usize;
    let mut n_outside = 0usize;
    for tet_id in 0..n_tets {
        let phi = centroid(&mesh, tet_id).norm() - R_INTERFACE;
        let nh = &materials[tet_id as usize];
        if phi <= -BAND {
            assert_eq!(
                nh.mu().to_bits(),
                MU_INNER.to_bits(),
                "tet {tet_id} at φ = {phi} (≤ −BAND) must snap μ to the inside endpoint",
            );
            assert_eq!(
                nh.lambda().to_bits(),
                LAMBDA_INNER.to_bits(),
                "tet {tet_id} at φ = {phi} (≤ −BAND) must snap λ to the inside endpoint",
            );
            n_inside += 1;
        } else if phi >= BAND {
            assert_eq!(
                nh.mu().to_bits(),
                MU_OUTER.to_bits(),
                "tet {tet_id} at φ = {phi} (≥ +BAND) must snap μ to the outside endpoint",
            );
            assert_eq!(
                nh.lambda().to_bits(),
                LAMBDA_OUTER.to_bits(),
                "tet {tet_id} at φ = {phi} (≥ +BAND) must snap λ to the outside endpoint",
            );
            n_outside += 1;
        }
    }

    // Scene-sanity floor: the snap assertions above are vacuous unless
    // both snap regions are populated.
    assert!(
        n_inside > 0,
        "no deep-inside tets (φ ≤ −BAND); snap-to-inside assertion never fired",
    );
    assert!(
        n_outside > 0,
        "no deep-outside tets (φ ≥ +BAND); snap-to-outside assertion never fired",
    );
}

#[test]
fn blended_materials_grade_monotonically_across_band() {
    // Two structural properties of the observed `μ(x)` field that
    // distinguish the smoothstep blend from a sharp `LayeredScalarField`
    // step and pin the "smooth gradient" the composition ships:
    //
    // 1. **Graded, not a step** — at least one band tet carries a μ
    //    strictly between the two endpoints. A `LayeredScalarField` (or a
    //    regressed blend that collapsed to a step) would only ever emit
    //    the endpoint values.
    // 2. **Spatially monotone** — sorting body tets by signed distance φ,
    //    the observed μ is non-decreasing (the blend weight is monotone in
    //    φ and `MU_OUTER > MU_INNER`).
    //
    // Both read `NeoHookean::mu()` off the meshed cache directly; the
    // oracle is the monotone-composition property, never a re-derived FMA
    // blend.
    //
    // The monotonicity check carries a relative FP-noise slack: the
    // Hermite cubic `s²(3 − 2s)` is mathematically monotone but its FP
    // evaluation is flat near the band edges, so two near-degenerate φ
    // (BCC centroid radii differing by ~1 ULP) can round to a sub-ULP μ
    // inversion (empirically ~1–4 ULP). The slack is set ~16× above that
    // wiggle so a benign meshing/toolchain shift cannot flake the assert.
    // It cannot mask a real regression either: a genuine non-monotonicity
    // (a step collapse, a reversed gradient, even a ~0.01 Pa artifact)
    // violates the order by ≥ 10⁷× this floor (the μ span is ~10⁵ Pa),
    // so widening the noise margin costs zero real-regression sensitivity.
    const MONOTONE_ULP_SLACK: f64 = 64.0 * f64::EPSILON;

    let mesh = build_blended_mesh();
    let materials = mesh.materials();
    let n_tets = mesh.n_tets() as TetId;

    let mut samples: Vec<(f64, f64)> = Vec::with_capacity(mesh.n_tets());
    for tet_id in 0..n_tets {
        let phi = centroid(&mesh, tet_id).norm() - R_INTERFACE;
        samples.push((phi, materials[tet_id as usize].mu()));
    }

    let graded = samples
        .iter()
        .any(|&(phi, mu)| phi.abs() < BAND && mu > MU_INNER && mu < MU_OUTER);
    assert!(
        graded,
        "no band tet carries a strictly-intermediate μ ∈ (MU_INNER, MU_OUTER); the blend \
         collapsed to a step instead of grading",
    );

    samples.sort_by(|a, b| {
        a.0.partial_cmp(&b.0)
            .expect("φ finite by construction (centroid + R_INTERFACE both finite)")
    });
    for pair in samples.windows(2) {
        let (phi_prev, mu_prev) = pair[0];
        let (phi_next, mu_next) = pair[1];
        assert!(
            mu_next >= mu_prev - MONOTONE_ULP_SLACK * mu_prev.abs(),
            "observed μ not monotone in φ: μ = {mu_prev} at φ = {phi_prev} then μ = {mu_next} \
             at φ = {phi_next}",
        );
    }
}

#[test]
fn blended_materials_follow_cubic_hermite_shape_not_linear() {
    // The blend kernel is the cubic Hermite smoothstep `w(s) = s²(3 − 2s)`,
    // NOT a linear ramp `w(s) = s`. Both are monotone, both snap at the
    // endpoints, and both pass through the midpoint at `s = 0.5`, so
    // monotonicity / snap / midpoint checks cannot tell them apart. The
    // distinguishing property is the S-CURVE SHAPE: the cubic lies strictly
    // BELOW the linear chord for `s < 0.5` and strictly ABOVE it for
    // `s > 0.5` (`w(s) − s = −s(s − 1)(2s − 1)`).
    //
    // Oracle is that chord, built from `s` — which is a trivial LINEAR
    // function of φ, independent of the cubic arithmetic — so this is not a
    // re-derivation of the kernel. `s` within `SHAPE_MARGIN` of 0.5 is
    // skipped (there the gap to the chord vanishes); both the below and the
    // above populations must be non-empty or the shape claim is vacuous.
    const SHAPE_MARGIN: f64 = 0.1;

    let mesh = build_blended_mesh();
    let materials = mesh.materials();
    let n_tets = mesh.n_tets() as TetId;

    let mut n_below = 0usize;
    let mut n_above = 0usize;
    for tet_id in 0..n_tets {
        let phi = centroid(&mesh, tet_id).norm() - R_INTERFACE;
        if phi.abs() >= BAND {
            continue; // snap region: kernel is clamped, shape not exercised
        }
        // Smoothstep input `s ∈ (0, 1)`, linear in φ.
        let s = (phi + BAND) / (2.0 * BAND);
        if (s - 0.5).abs() < SHAPE_MARGIN {
            continue; // near the inflection the cubic ≈ the chord
        }
        let observed_mu = materials[tet_id as usize].mu();
        let chord_mu = s.mul_add(MU_OUTER - MU_INNER, MU_INNER);
        if s < 0.5 {
            assert!(
                observed_mu < chord_mu,
                "tet {tet_id} (s = {s}) μ = {observed_mu} not strictly below the linear chord \
                 {chord_mu} — the blend is not the cubic Hermite S-curve (a linear ramp would sit \
                 on the chord)",
            );
            n_below += 1;
        } else {
            assert!(
                observed_mu > chord_mu,
                "tet {tet_id} (s = {s}) μ = {observed_mu} not strictly above the linear chord \
                 {chord_mu} — the blend is not the cubic Hermite S-curve",
            );
            n_above += 1;
        }
    }

    assert!(
        n_below > 0,
        "no band tet with s < 0.5 − margin — cannot check the below-chord half of the S-curve",
    );
    assert!(
        n_above > 0,
        "no band tet with s > 0.5 + margin — cannot check the above-chord half of the S-curve",
    );
}

#[test]
fn blended_interface_flags_match_book_rule_per_tet() {
    // Per-tet book rule `|φ(x_c)| < L_e` (Part 7 §02 §01) for the
    // BlendedScalarField-composition interface SDF — the coverage IV-6
    // defers. `L_e` is re-derived with an INDEPENDENTLY ORDERED six-edge
    // sum (not a copy of the library's hand-unrolled accumulation), so the
    // per-tet biconditional is a genuine cross-check. Tets within a
    // relative `FLAG_EPS` of the `|φ| == L_e` boundary are skipped: there
    // the two summation orders may disagree by rounding and the boolean is
    // legitimately ambiguous (a measure-zero set — in practice none).
    const FLAG_EPS: f64 = 1e-9;

    let mesh = build_blended_mesh();
    let flags = mesh.interface_flags();
    let positions = mesh.positions();
    let n_tets = mesh.n_tets() as TetId;

    assert_eq!(
        flags.len(),
        mesh.n_tets(),
        "interface_flags length {} must equal n_tets {}",
        flags.len(),
        mesh.n_tets(),
    );

    let mut n_flagged = 0usize;
    let mut n_unflagged = 0usize;
    for tet_id in 0..n_tets {
        let [a, b, c, d] = mesh.tet_vertices(tet_id);
        let v0 = positions[a as usize];
        let v1 = positions[b as usize];
        let v2 = positions[c as usize];
        let v3 = positions[d as usize];
        let phi = ((v0 + v1 + v2 + v3) * 0.25).norm() - R_INTERFACE;

        // Independently ordered six-edge mean (library sums the pairs
        // 01,02,03,12,13,23; here 01,12,23,02,13,03 into an array).
        let edges = [
            (v1 - v0).norm(),
            (v2 - v1).norm(),
            (v3 - v2).norm(),
            (v2 - v0).norm(),
            (v3 - v1).norm(),
            (v3 - v0).norm(),
        ];
        let l_e = edges.iter().sum::<f64>() / 6.0;

        if (phi.abs() - l_e).abs() <= FLAG_EPS * l_e {
            continue; // knife edge — boolean legitimately order-sensitive
        }

        let expected = phi.abs() < l_e;
        assert_eq!(
            flags[tet_id as usize],
            expected,
            "tet {tet_id}: interface flag {} disagrees with book rule |φ| < L_e \
             (|φ| = {}, L_e = {l_e})",
            flags[tet_id as usize],
            phi.abs(),
        );
        if expected {
            n_flagged += 1;
        } else {
            n_unflagged += 1;
        }
    }

    // Both classes must be non-empty or the biconditional is half-vacuous.
    assert!(
        n_flagged > 0,
        "no flagged tets — the interface band produced no `|φ| < L_e` hits",
    );
    assert!(
        n_unflagged > 0,
        "no unflagged tets — every tet flagged, book rule not discriminating",
    );
}
