//! SDF-projection mesher rung — end-to-end on the canonical hollow sphere.
//!
//! [`Tet10Mesh::with_sdf_projected_boundary`] is the productized form of the
//! test-local `curve_sphere_boundary` closure in `tet10_lame_decision.rs`: it
//! projects the boundary midside nodes onto the true rigid surface so the
//! isoparametric [`Tet10`] element integrates over the real curved geometry
//! rather than the inscribed facet chords ("exact geometry IS the exact
//! physics").
//!
//! The headline gate is [`reproduces_the_closed_form_radial_projection`]: on
//! the canonical layered sphere, the general Newton projector on the body's
//! `DifferenceSdf` must land every boundary midside on the *same* point the
//! hand-written radial snap does.
//!
//! # ⚠⚠ `CURVED_FACET = -0.04323` was measured on a mesh with eight folded
//! elements in it
//!
//! This module used to close the paragraph above with: that closure produced
//! the merged forward measurement `CURVED_FACET = -0.04323`, "so reproducing
//! its node positions inherits that result through the real mesher API".
//!
//! **The inheritance argument no longer holds, and the reason is not that the
//! projector changed.** The mesh those node positions produce was censused
//! and found to carry **8 of 624 elements folded at a reference corner** — the
//! old four-point back-off could not see them. `with_sdf_projected_boundary`
//! now certifies validity exactly, which makes its back-off engage on this
//! fixture (14 of 360 boundary midsides are held short of the surface at
//! `LAYERED_SPHERE_CONFORM_QUALITY_FLOOR`), so the conformed geometry here is
//! **no longer bit-identical** to the geometry `CURVED_FACET` was taken on.
//!
//! ▶ Re-measuring means re-running `tet10_lame_decision`, which is held out of
//! CI for its ~7 GB working set. Until then `CURVED_FACET` stands as a reading
//! whose fixture is known to have contained folded elements — it is not
//! silently re-anchored here, because a number nobody re-ran is not a number
//! this file may adjust.
//!
//! # The two claims, now separated
//!
//! [`reproduces_the_closed_form_radial_projection`] tests the **projector** on
//! the projector's own output, and
//! [`boundary_midsides_land_on_the_sdf_surface_or_are_held_short_along_their_ray`]
//! tests the **back-off**. They used to be one composite gate that attributed
//! every discrepancy to the projector.
//!
//! Element validity of the conformed mesh is gated in
//! `tests/tet10_exact_validity.rs`, which certifies `det J` over each whole
//! element. The Gauss-point spot check that used to live here was strictly
//! weaker — it is the exact rule that let the eight folded elements through.

#![allow(
    // Vertex/tet counts index a `u32` id space (the `Mesh` trait's ids); the
    // canonical sphere holds far fewer than `u32::MAX`. Mirrors the sibling
    // `tet10_lame_decision` convention.
    clippy::cast_possible_truncation,
    // Scene construction and the `Tet10Mesh` midside channel surface a
    // meshing / contract failure as a test panic — the canonical sphere either
    // meshes and surfaces its faces or has regressed worth investigating.
    clippy::expect_used
)]

use sim_soft::{
    DifferenceSdf, LAYERED_SPHERE_CONFORM_QUALITY_FLOOR, LAYERED_SPHERE_R_CAVITY,
    LAYERED_SPHERE_R_OUTER, MaterialField, Mesh, Sdf, SoftScene, SphereSdf, Tet10Mesh, Vec3,
    VertexId, project_point_onto_sdf,
};

/// Shear modulus and pressure are immaterial here (no solve); reuse the
/// decision-harness values so the mesh is byte-for-byte the one the Lamé gate
/// curves.
const MU: f64 = 2.0e5;
const NU: f64 = 0.4;
const PRESSURE: f64 = 5.0e3;
/// Coarsest canonical cell — a real hollow sphere, cheap to build (no solve).
const CELL: f64 = 0.04;

fn lambda_from_nu(nu: f64) -> f64 {
    2.0 * MU * nu / (1.0 - 2.0 * nu)
}

/// The layered-sphere body surface: outer sphere minus the cavity — exactly the
/// `DifferenceSdf` `SoftScene::layered_silicone_sphere` meshes against.
fn body_sdf() -> DifferenceSdf {
    DifferenceSdf::new(
        Box::new(SphereSdf {
            radius: LAYERED_SPHERE_R_OUTER,
        }),
        Box::new(SphereSdf {
            radius: LAYERED_SPHERE_R_CAVITY,
        }),
    )
}

/// The straight (un-projected) Tet10 sphere mesh — enriched from the scene's
/// Tet4 mesh, no solve.
fn straight_sphere_tet10() -> Tet10Mesh {
    let (mesh4, _bc, _initial, _theta) = SoftScene::layered_silicone_sphere(
        MaterialField::uniform(MU, lambda_from_nu(NU)),
        CELL,
        PRESSURE,
    )
    .expect("layered_silicone_sphere meshes at the canonical cell size");
    Tet10Mesh::from_tet4(&mesh4)
}

/// Boundary midside vertex ids (trailing three slots of each 6-node face).
fn boundary_midsides(mesh: &Tet10Mesh) -> Vec<VertexId> {
    let mut b: Vec<VertexId> = mesh
        .boundary_faces6()
        .expect("Tet10Mesh surfaces 6-node boundary faces")
        .iter()
        .flat_map(|f| [f[3], f[4], f[5]])
        .collect();
    b.sort_unstable();
    b.dedup();
    b
}

/// The hand-written radial snap the merged Lamé gate uses: move a boundary
/// midside onto the nearer of the two analytic shells.
fn closed_form_radial(p: Vec3) -> Vec3 {
    let r = p.norm();
    let target = if (r - LAYERED_SPHERE_R_CAVITY).abs() < (r - LAYERED_SPHERE_R_OUTER).abs() {
        LAYERED_SPHERE_R_CAVITY
    } else {
        LAYERED_SPHERE_R_OUTER
    };
    p * (target / r)
}

/// Boundary midsides the certified back-off holds short of the surface at
/// [`LAYERED_SPHERE_CONFORM_QUALITY_FLOOR`], of 360 total, and the largest
/// shortfall. Both are read off `conform_quality_floor_sweep`'s `0.20` row in
/// `tet10_exact_validity.rs`; pinned here so a silent change in either the
/// floor or the projector shows up as a failure rather than as drift.
const EXPECTED_HELD_SHORT: usize = 14;
const EXPECTED_MAX_SHORTFALL: f64 = 1.7e-3;

/// ★ Tests the **projector**, with the back-off deliberately factored out.
///
/// This gate used to compare final node positions against the closed-form
/// snap, which conflated two independent claims — that the Newton projector
/// finds the right point, and that the back-off leaves it there. Once the
/// back-off started engaging (it must, or the mesh ships folded elements)
/// the composite failed at `3.33e-3 m` and read as a projector regression,
/// which it was not. The projector's own accuracy is now measured on the
/// projector's own output.
#[test]
fn reproduces_the_closed_form_radial_projection() {
    let straight = straight_sphere_tet10();
    let boundary = boundary_midsides(&straight);
    assert!(
        boundary.len() > 100,
        "the canonical sphere has a substantial boundary ({} midsides)",
        boundary.len(),
    );
    let sdf = body_sdf();

    let mut max_err = 0.0_f64;
    let mut max_move = 0.0_f64;
    for &m in &boundary {
        let s = straight.positions()[m as usize];
        // The projector's target, before any validity back-off touches it.
        let target = project_point_onto_sdf(&sdf, s, 1e-12, 32);
        max_err = max_err.max((target - closed_form_radial(s)).norm());
        max_move = max_move.max((target - s).norm());
    }
    // The general Newton projector on the DifferenceSdf lands on the same
    // radial point as the closed-form snap (both move along the ray through the
    // origin): agreement to Newton tolerance, far below any physical scale.
    assert!(
        max_err < 1e-9,
        "productized projection must reproduce the closed-form radial snap; \
         max node error = {max_err:e}",
    );
    // Non-vacuous: the inscribed boundary genuinely moves (the largest move is
    // of order 1e-2 m at this cell size), so the gate is testing a real
    // displacement, not a no-op.
    assert!(
        max_move > 1e-5,
        "the projection must genuinely move the inscribed boundary; \
         max move = {max_move:e}",
    );
}

/// ★ Tests the **back-off**: where the mesh does not reach the surface, and by
/// how much.
///
/// ⚠ This gate used to assert every boundary midside lands on the surface, and
/// that was true only because the back-off never engaged — which was itself
/// only true because the old four-point sampling rule could not see the eight
/// elements it was folding. Exact certification makes the back-off engage, so
/// the honest claim is not "all nodes reach the surface" but "every node
/// either reaches the surface or was held short along its own radial ray, and
/// the number and size of those are pinned".
#[test]
fn boundary_midsides_land_on_the_sdf_surface_or_are_held_short_along_their_ray() {
    let sdf = body_sdf();
    let straight = straight_sphere_tet10();
    let boundary = boundary_midsides(&straight);
    let curved = straight
        .clone()
        .with_sdf_projected_boundary(&sdf, LAYERED_SPHERE_CONFORM_QUALITY_FLOOR);

    let (mut on_surface, mut held_short) = (0usize, 0usize);
    let mut max_shortfall = 0.0_f64;
    for &m in &boundary {
        let s = straight.positions()[m as usize];
        let p = curved.positions()[m as usize];
        let eval = sdf.eval(p.into()).abs();

        if eval <= 1e-9 {
            on_surface += 1;
            // It landed on one of the two analytic shells, not the crease.
            let r = p.norm();
            let on_shell = (r - LAYERED_SPHERE_R_CAVITY).abs() < 1e-9
                || (r - LAYERED_SPHERE_R_OUTER).abs() < 1e-9;
            assert!(
                on_shell,
                "midside {m} landed at radius {r}, off both shells"
            );
            continue;
        }

        held_short += 1;
        max_shortfall = max_shortfall.max(eval);
        // A held-short node is still on the straight→target segment, which for
        // this body is the ray through the origin: the back-off shortens the
        // move, it never redirects it.
        let target = closed_form_radial(s);
        let seg = target - s;
        let moved = p - s;
        let t = moved.dot(&seg) / seg.dot(&seg);
        assert!(
            (moved - seg * t).norm() < 1e-9 * seg.norm(),
            "held-short midside {m} left its straight→surface ray",
        );
        assert!(
            (0.0..1.0).contains(&t),
            "held-short midside {m} must sit strictly between straight and target, got t = {t}",
        );
    }

    assert_eq!(
        held_short, EXPECTED_HELD_SHORT,
        "the number of midsides the floor holds short is a pinned measurement, \
         not a free parameter ({on_surface} reached the surface)",
    );
    assert!(
        max_shortfall < EXPECTED_MAX_SHORTFALL,
        "largest shortfall {max_shortfall:e} m exceeds the pinned {EXPECTED_MAX_SHORTFALL:e} m",
    );
    assert!(
        on_surface * 10 > boundary.len() * 9,
        "the overwhelming majority must still reach the surface; {on_surface}/{}",
        boundary.len(),
    );
}

// `projected_sphere_has_no_inverted_elements` lived here and asserted `det J`
// was positive at the four Stroud points of every element. It is deleted
// rather than kept alongside the certified gate: it PASSED on the mesh that
// carried eight folded elements, because those four points are exactly what
// the fold hid between. Keeping a green check that cannot fail for the reason
// it claims is worse than not having it —
// `tests/tet10_exact_validity.rs::the_conformed_sphere_is_certified_element_by_element`
// establishes the property over each whole element and strictly subsumes it.
