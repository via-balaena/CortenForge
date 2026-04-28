//! Phase 4 commit 4 — `Mesh::materials` per-tet cache tests.
//!
//! Verifies the new trait surface introduced by commit 4:
//!
//! 1. `cache.len() == n_tets` for every `Mesh` impl that ships in
//!    Phase 4 (`SingleTetMesh`, `HandBuiltTetMesh::{two_isolated_tets,
//!    two_tet_shared_face}`, `SdfMeshedTetMesh`).
//! 2. Uniform-field construction produces a per-tet cache that is
//!    bit-equal to a single canonical `NeoHookean::from_lame(μ, λ)`
//!    everywhere — Decision P regression-net target for IV-1.
//! 3. Layered-field construction reads each tet's centroid into the
//!    correct shell (Decision K centroid-sampling default per Part 7
//!    §02 §00).
//! 4. Centroid formula is pinned: a 1-shell partition with the
//!    threshold tuned so the canonical centroid sits ~1.7e-3 below
//!    the boundary catches plausible off-by-one centroid errors (e.g.
//!    `(v0 + v1 + v2) / 3` would shift the sampled `phi` across the
//!    threshold and trip the bit-equality assertion).

// SDF-meshed cases unwrap a meshing `Result` whose `Err` arm only
// surfaces under construction-time misuse (canonical sphere hints
// always succeed). Allowing `expect` matches `sdf_pipeline_determinism.rs`.
#![allow(clippy::expect_used)]

mod common;

use common::assert_neo_hookean_bit_equal;
use sim_soft::sdf_bridge::{Aabb3, MeshingHints};
use sim_soft::{
    Field, HandBuiltTetMesh, LayeredScalarField, MaterialField, Mesh, NeoHookean, SdfMeshedTetMesh,
    SingleTetMesh, SphereSdf, Vec3,
};

const MU_BASELINE: f64 = 1.0e5;
const LAMBDA_BASELINE: f64 = 4.0e5;

fn baseline_field() -> MaterialField {
    MaterialField::uniform(MU_BASELINE, LAMBDA_BASELINE)
}

const fn canonical_sphere_hints() -> MeshingHints {
    MeshingHints {
        bbox: Aabb3::new(Vec3::new(-0.12, -0.12, -0.12), Vec3::new(0.12, 0.12, 0.12)),
        cell_size: 0.02,
    }
}

// ---------------------------------------------------------------------
// 1. Cache length matches `n_tets` across every Phase-4 mesh impl
// ---------------------------------------------------------------------

#[test]
fn single_tet_cache_has_length_one() {
    let mesh = SingleTetMesh::new(&baseline_field());
    assert_eq!(mesh.materials().len(), mesh.n_tets());
    assert_eq!(mesh.materials().len(), 1);
}

#[test]
fn two_isolated_tets_cache_has_length_two() {
    let mesh = HandBuiltTetMesh::two_isolated_tets(&baseline_field());
    assert_eq!(mesh.materials().len(), mesh.n_tets());
    assert_eq!(mesh.materials().len(), 2);
}

#[test]
fn two_tet_shared_face_cache_has_length_two() {
    let mesh = HandBuiltTetMesh::two_tet_shared_face(&baseline_field());
    assert_eq!(mesh.materials().len(), mesh.n_tets());
    assert_eq!(mesh.materials().len(), 2);
}

#[test]
fn sdf_meshed_cache_length_matches_n_tets() {
    let mesh = SdfMeshedTetMesh::from_sdf(
        &SphereSdf { radius: 0.1 },
        &canonical_sphere_hints(),
        &baseline_field(),
    )
    .expect("canonical sphere should mesh successfully");
    assert_eq!(mesh.materials().len(), mesh.n_tets());
    assert!(
        !mesh.materials().is_empty(),
        "canonical sphere produces a non-empty cache"
    );
}

// ---------------------------------------------------------------------
// 2. Uniform-field bit-equality (IV-1 regression-net early-warning)
// ---------------------------------------------------------------------

#[test]
fn uniform_field_produces_uniform_cache_on_two_isolated_tets() {
    let mesh = HandBuiltTetMesh::two_isolated_tets(&baseline_field());
    let expected = NeoHookean::from_lame(MU_BASELINE, LAMBDA_BASELINE);
    for nh in mesh.materials() {
        assert_neo_hookean_bit_equal(nh, &expected);
    }
}

#[test]
fn uniform_field_produces_uniform_cache_on_sdf_meshed() {
    let mesh = SdfMeshedTetMesh::from_sdf(
        &SphereSdf { radius: 0.1 },
        &canonical_sphere_hints(),
        &baseline_field(),
    )
    .expect("canonical sphere should mesh successfully");
    let expected = NeoHookean::from_lame(MU_BASELINE, LAMBDA_BASELINE);
    for nh in mesh.materials() {
        assert_neo_hookean_bit_equal(nh, &expected);
    }
}

// ---------------------------------------------------------------------
// 3. Layered-field per-tet centroid drives shell selection
// ---------------------------------------------------------------------

#[test]
fn layered_field_samples_per_tet_centroid_on_two_isolated_tets() {
    // `two_isolated_tets`:
    // - tet 0 centroid = (0.025, 0.025, 0.025), ‖·‖ ≈ 0.04330
    // - tet 1 centroid = (0.525, 0.025, 0.025), ‖·‖ ≈ 0.52560
    //
    // `SphereSdf { radius: 0.3 }` + threshold `[0.0]`:
    // - phi(tet 0 centroid) = -0.2567 → inner shell (values[0])
    // - phi(tet 1 centroid) = +0.2256 → outer shell (values[1])
    let sphere = || Box::new(SphereSdf { radius: 0.3 });
    let mu_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        sphere(),
        vec![0.0],
        vec![1.0e5, 2.0e5],
    ));
    let lambda_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        sphere(),
        vec![0.0],
        vec![4.0e5, 8.0e5],
    ));
    let field = MaterialField::from_fields(mu_field, lambda_field);

    let mesh = HandBuiltTetMesh::two_isolated_tets(&field);
    let expected_inner = NeoHookean::from_lame(1.0e5, 4.0e5);
    let expected_outer = NeoHookean::from_lame(2.0e5, 8.0e5);
    assert_neo_hookean_bit_equal(&mesh.materials()[0], &expected_inner);
    assert_neo_hookean_bit_equal(&mesh.materials()[1], &expected_outer);
}

// ---------------------------------------------------------------------
// 4. Centroid formula pinned via threshold-adjacent shell partition
// ---------------------------------------------------------------------

#[test]
fn cache_pins_centroid_formula_via_threshold_neighbor() {
    // `SingleTetMesh` canonical centroid = (0.025, 0.025, 0.025),
    // ‖·‖ ≈ 0.04330. With `SphereSdf { radius: 0.045 }` + threshold
    // `[0.0]`, the canonical centroid lands at `phi = -0.00170`
    // (inner shell). Plausible off-by-one centroid errors fall on the
    // *outer* side of the same threshold:
    //
    // - `(v0 + v1 + v2) / 3 = (0.0333, 0.0333, 0.000)` ⇒ ‖·‖ ≈ 0.04714
    //   ⇒ phi = +0.00214 ⇒ outer shell ⇒ different μ/λ ⇒ bit-equality
    //   trips.
    // - `(v1 + v2 + v3) / 3 = (0.0333, 0.0333, 0.0333)` ⇒ ‖·‖ ≈ 0.05774
    //   ⇒ phi = +0.01274 ⇒ outer shell ⇒ trips.
    let sphere = || Box::new(SphereSdf { radius: 0.045 });
    let mu_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        sphere(),
        vec![0.0],
        vec![1.0e5, 2.0e5],
    ));
    let lambda_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        sphere(),
        vec![0.0],
        vec![4.0e5, 8.0e5],
    ));
    let field = MaterialField::from_fields(mu_field, lambda_field);

    let mesh = SingleTetMesh::new(&field);
    let expected_inner = NeoHookean::from_lame(1.0e5, 4.0e5);
    assert_neo_hookean_bit_equal(&mesh.materials()[0], &expected_inner);
}
