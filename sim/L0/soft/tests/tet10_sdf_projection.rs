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
//! hand-written radial snap does. That closure produced the merged forward
//! measurement `CURVED_FACET = -0.04323` (curving the Lamé boundary moves the
//! `Facet` reading ~18 % toward analytic), so reproducing its node positions
//! inherits that result through the real mesher API — without re-running the
//! multi-second solve.
//!
//! The companion gates confirm the two invariants the properties above rest
//! on: every boundary midside actually lands on the SDF surface, and no
//! element inverts (on this gently-curved mesh the back-off never engages —
//! that path is exercised by the `--lib` `inversion_back_off_*` gate on a
//! deliberately pathological projection).

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

use sim_soft::element::{Element, Tet10};
use sim_soft::{
    DifferenceSdf, LAYERED_SPHERE_R_CAVITY, LAYERED_SPHERE_R_OUTER, MaterialField, Mesh, Sdf,
    SoftScene, SphereSdf, Tet10Mesh, Vec3, VertexId,
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

#[test]
fn reproduces_the_closed_form_radial_projection() {
    let straight = straight_sphere_tet10();
    let boundary = boundary_midsides(&straight);
    assert!(
        boundary.len() > 100,
        "the canonical sphere has a substantial boundary ({} midsides)",
        boundary.len(),
    );

    let before: Vec<Vec3> = boundary
        .iter()
        .map(|&m| straight.positions()[m as usize])
        .collect();
    let curved = straight.with_sdf_projected_boundary(&body_sdf());

    let mut max_err = 0.0_f64;
    let mut max_move = 0.0_f64;
    for (&m, &s) in boundary.iter().zip(&before) {
        let got = curved.positions()[m as usize];
        let expected = closed_form_radial(s);
        max_err = max_err.max((got - expected).norm());
        max_move = max_move.max((got - s).norm());
    }
    // The general Newton projector on the DifferenceSdf lands on the same
    // radial point as the closed-form snap (both move along the ray through the
    // origin): agreement to Newton tolerance, far below any physical scale.
    assert!(
        max_err < 1e-9,
        "productized projection must reproduce the closed-form radial snap; \
         max node error = {max_err:e}",
    );
    // Non-vacuous: the inscribed boundary genuinely moves (sagitta ~ 1e-4 m at
    // this cell size), so the gate is testing a real displacement.
    assert!(
        max_move > 1e-5,
        "the projection must genuinely move the inscribed boundary; \
         max move = {max_move:e}",
    );
}

#[test]
fn boundary_midsides_land_on_the_sdf_surface() {
    let sdf = body_sdf();
    let curved = straight_sphere_tet10().with_sdf_projected_boundary(&sdf);
    for &m in &boundary_midsides(&curved) {
        let p = curved.positions()[m as usize];
        assert!(
            sdf.eval(p.into()).abs() <= 1e-9,
            "boundary midside {m} at {p:?} is off the surface (eval = {})",
            sdf.eval(p.into()),
        );
        // And it lands on one of the two analytic shells, not the crease.
        let r = p.norm();
        let on_shell =
            (r - LAYERED_SPHERE_R_CAVITY).abs() < 1e-9 || (r - LAYERED_SPHERE_R_OUTER).abs() < 1e-9;
        assert!(
            on_shell,
            "midside {m} landed at radius {r}, off both shells"
        );
    }
}

#[test]
fn projected_sphere_has_no_inverted_elements() {
    let curved = straight_sphere_tet10().with_sdf_projected_boundary(&body_sdf());
    let element = Tet10;
    let mut checked = 0usize;
    for t in 0..curved.n_tets() as u32 {
        let corners = curved.tet_vertices(t);
        let mids = curved
            .tet_midside_nodes(t)
            .expect("Tet10Mesh surfaces the midside channel");
        let mut nodes = [Vec3::zeros(); 10];
        for (a, &c) in corners.iter().enumerate() {
            nodes[a] = curved.positions()[c as usize];
        }
        for (i, &m) in mids.iter().enumerate() {
            nodes[4 + i] = curved.positions()[m as usize];
        }
        let x_ref = nalgebra::SMatrix::<f64, 10, 3>::from_fn(|a, k| nodes[a][k]);
        for (q, d) in element.rest_jacobian_dets(&x_ref).iter().enumerate() {
            assert!(
                d.is_finite() && *d > 0.0,
                "tet {t} Gauss point {q} inverted: detJ = {d}",
            );
        }
        checked += 1;
    }
    assert!(checked > 0, "the sphere mesh has elements to check");
}
