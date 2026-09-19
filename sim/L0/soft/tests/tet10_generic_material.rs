#![allow(clippy::expect_used)]

//! `Tet10Mesh` is generic over its material type.
//!
//! Before this rung `Tet10Mesh` stored `Vec<NeoHookean>` and implemented
//! `Mesh` at the trait's default material, so `Mesh<Yeoh>` was unsatisfied and
//! a Tet10 scene could not run Yeoh. That mattered because the two capabilities
//! the scan-fit sleeve needs sat on opposite sides of the gap: the rung-8b
//! surface-integrated contact barrier (and with it a face-consistent
//! `peak_contact_pressure`) is emitted only on a mesh exposing six-node
//! boundary faces — i.e. Tet10 — while Neo-Hookean's validity domain trips at
//! `max_disp ≈ 7 mm`, so the 8 mm interference target is Yeoh-only.
//!
//! These gates pin the *carrying* property, not the solve: enrichment copies
//! the source's per-tet materials verbatim, so a Tet10 mesh must report
//! materials bit-identical to the Tet4 mesh it came from, and must still hand
//! `boundary_faces6` to the contact model.

use sim_soft::material::silicone_table::ECOFLEX_00_30;
use sim_soft::{
    Aabb3, ConstantField, Field, Material, MaterialField, Mesh, MeshingHints, SdfMeshedTetMesh,
    SphereSdf, Tet10Mesh, Vec3, Yeoh,
};

const R: f64 = 0.05;
const CELL: f64 = 0.02;
const HALF: f64 = R * 1.6;

/// `mu` graded along z.
///
/// ⚠ A *uniform* field makes the per-index carry gate below vacuous: every
/// material compares equal, so a permutation or an off-by-one in the copy
/// would pass. Grading it means each tet's material is distinguishable and the
/// index mapping is actually under test. Spatial variation is also the case
/// that matters downstream — zoned durometer and filler-fraction fields are
/// exactly this shape.
struct GradedMu;

impl Field<f64> for GradedMu {
    fn sample(&self, x_ref: Vec3) -> f64 {
        ECOFLEX_00_30.mu * 4.0_f64.mul_add((x_ref.z + HALF) / (2.0 * HALF), 1.0)
    }
}

/// Ecoflex 00-30's Yeoh triple, off the `silicone_table` anchor so this test
/// moves with the data sheet rather than restating it, with `mu` graded.
fn yeoh_field() -> MaterialField {
    MaterialField::from_yeoh_fields(
        Box::new(GradedMu),
        Box::new(ConstantField::new(ECOFLEX_00_30.c2)),
        Box::new(ConstantField::new(ECOFLEX_00_30.lambda)),
    )
}

fn tet4_yeoh() -> SdfMeshedTetMesh<Yeoh> {
    let hints = MeshingHints {
        bbox: Aabb3::new(Vec3::new(-HALF, -HALF, -HALF), Vec3::new(HALF, HALF, HALF)),
        cell_size: CELL,
        material_field: Some(yeoh_field()),
    };
    SdfMeshedTetMesh::<Yeoh>::from_sdf_yeoh(&SphereSdf { radius: R }, &hints)
        .expect("sphere must mesh at this cell size")
}

/// A stretch past the small-strain regime, where Yeoh's `C2(I1-3)^2` term is
/// the whole difference from Neo-Hookean.
const fn stretched() -> nalgebra::Matrix3<f64> {
    nalgebra::Matrix3::new(1.8, 0.0, 0.0, 0.0, 0.9, 0.0, 0.0, 0.0, 0.9)
}

#[test]
fn tet10_carries_its_tet4_source_materials_verbatim() {
    let tet4 = tet4_yeoh();
    let tet10 = Tet10Mesh::<Yeoh>::from_tet4(&tet4);

    // Collection first: a zero-tet mesh would pass the comparison below by
    // comparing nothing.
    let src: &[Yeoh] = Mesh::<Yeoh>::materials(&tet4);
    assert!(
        src.len() > 100,
        "expected a non-trivial mesh, got {} tets",
        src.len()
    );
    assert_eq!(
        Mesh::<Yeoh>::n_tets(&tet10),
        Mesh::<Yeoh>::n_tets(&tet4),
        "enrichment adds nodes, never tets",
    );

    let got: &[Yeoh] = Mesh::<Yeoh>::materials(&tet10);
    assert_eq!(got.len(), src.len(), "one material per tet, carried across");

    let f = stretched();

    // The gate is only non-vacuous if the materials actually differ: with a
    // uniform field every comparison below holds under any permutation.
    let mut energies: Vec<f64> = src.iter().map(|m| m.energy(&f)).collect();
    energies.sort_by(f64::total_cmp);
    energies.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    assert!(
        energies.len() > 10,
        "graded field must produce distinguishable materials; got {} distinct",
        energies.len(),
    );

    for (i, (a, b)) in got.iter().zip(src.iter()).enumerate() {
        assert!(
            (a.energy(&f) - b.energy(&f)).abs() < f64::EPSILON,
            "tet {i}: enriched material must be bit-identical to its source",
        );
    }
}

/// Same scene as [`tet4_yeoh`] but with **uniform** `mu`.
///
/// ⚠ The C2 discriminator below needs `mu` held constant. Against the graded
/// field the comparison passes because `mu` differs, not because `C2` does —
/// a mutation zeroing the field's `C2` survived that version of this gate,
/// which is the definition of a test that cannot fail for its stated reason.
fn tet4_yeoh_uniform_mu() -> SdfMeshedTetMesh<Yeoh> {
    let hints = MeshingHints {
        bbox: Aabb3::new(Vec3::new(-HALF, -HALF, -HALF), Vec3::new(HALF, HALF, HALF)),
        cell_size: CELL,
        material_field: Some(MaterialField::from_yeoh_fields(
            Box::new(ConstantField::new(ECOFLEX_00_30.mu)),
            Box::new(ConstantField::new(ECOFLEX_00_30.c2)),
            Box::new(ConstantField::new(ECOFLEX_00_30.lambda)),
        )),
    };
    SdfMeshedTetMesh::<Yeoh>::from_sdf_yeoh(&SphereSdf { radius: R }, &hints)
        .expect("sphere must mesh at this cell size")
}

#[test]
fn the_carried_material_is_yeoh_not_neo_hookean() {
    // Guards the carry gate from passing on a silently-NeoHookean cache: an NH
    // with the same (mu, lambda) would satisfy a verbatim-copy comparison.
    // Yeoh's C2 term is the only discriminator, and it separates the two only
    // away from the rest configuration — hence `stretched()`.
    //
    // `mu` is uniform here ON PURPOSE so C2 is the sole difference between
    // `mat` and `nh_equivalent`.
    let tet10 = Tet10Mesh::<Yeoh>::from_tet4(&tet4_yeoh_uniform_mu());
    let mat = &Mesh::<Yeoh>::materials(&tet10)[0];

    let nh_equivalent = Yeoh::from_lame_and_c2(ECOFLEX_00_30.mu, ECOFLEX_00_30.lambda, 0.0);
    let f = stretched();

    // (An `assert!(ECOFLEX_00_30.c2 > 0.0)` stood here. `c2` is a `const`, so
    // that assertion had a constant value and could never fail at runtime —
    // a vacuous gate. It is unnecessary anyway: if the anchor's C2 were zero
    // the comparison below would find the two energies equal and fire, which
    // a C2 = 0 mutation confirmed.)
    assert!(
        (mat.energy(&f) - nh_equivalent.energy(&f)).abs() > 1.0,
        "C2 must contribute at this stretch — otherwise this is NH wearing a Yeoh type",
    );
}

#[test]
fn tet10_yeoh_still_exposes_six_node_boundary_faces() {
    // This is what routes IPC to the rung-8b surface-integrated barrier:
    // `active_pairs` branches on `boundary_faces6()`, and a `None` here would
    // silently fall back to the per-vertex path.
    let tet10 = Tet10Mesh::<Yeoh>::from_tet4(&tet4_yeoh());

    // `None` here would silently route IPC to the per-vertex path.
    let faces6 = Mesh::<Yeoh>::boundary_faces6(&tet10)
        .expect("a Tet10 mesh must expose six-node boundary faces");
    let faces3 = Mesh::<Yeoh>::boundary_faces(&tet10);

    assert!(!faces6.is_empty(), "boundary must be non-empty");
    assert_eq!(
        faces6.len(),
        faces3.len(),
        "one 6-node face per 3-node face"
    );
    for (i, (f6, f3)) in faces6.iter().zip(faces3.iter()).enumerate() {
        assert_eq!(&f6[..3], &f3[..], "face {i}: corner triple must match");
    }
}
