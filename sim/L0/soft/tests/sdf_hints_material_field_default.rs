//! Phase 4 commit 9 — `MeshingHints::material_field` threading gate.
//!
//! Three #[test] blocks cover the new hint-carried material-field
//! surface that commit 9 wires into [`SdfMeshedTetMesh::from_sdf`]:
//!
//! 1. `none_path_matches_explicit_uniform_baseline` — the
//!    `material_field: None` fallback synthesizes a `MaterialField`
//!    that produces a per-tet cache bit-equal to an explicit
//!    `Some(MaterialField::uniform(1.0e5, 4.0e5))`. Self-consistency
//!    bar pinning the contract that `None` is observationally equal to
//!    the IV-1 baseline. Mesh topology + positions must also match
//!    bit-equal — the only thing the field choice changes is the
//!    per-tet `materials()` cache, so this is the strict regression
//!    detector.
//! 2. `none_path_synthesizes_ecoflex_baseline_value` — the per-tet
//!    `materials()` entries on the `None` path are bit-equal to
//!    `NeoHookean::from_lame(1.0e5, 4.0e5)`. Value-correctness bar:
//!    self-consistency alone (block 1) would pass even if both routes
//!    quietly returned a different uniform pair. Pins the documented
//!    `MaterialField::skeleton_default()` constant via the SDF mesher.
//! 3. `non_uniform_layered_field_threads_through_hints` — a
//!    `LayeredScalarField`-backed `MaterialField` passed via
//!    `MeshingHints::material_field` produces per-tet entries whose
//!    Lamé parameters depend on each tet's centroid relative to a
//!    `SphereSdf` partition. The genuinely novel coverage: non-uniform
//!    fields actually flow through hints end-to-end, which is what
//!    enables commit 10's IV-4 SDF region-tagging test.
//!
//! Memo cite: scope memo §8 commit 9 + §0 (`MeshingHints` row) +
//! Decision M (no new γ-locked types) + Decision N (determinism
//! carry-forward).

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

const RADIUS: f64 = 0.1;
const CELL_SIZE: f64 = 0.02;
const BBOX_HALF_EXTENT: f64 = 0.12;

const MU_BASELINE: f64 = 1.0e5;
const LAMBDA_BASELINE: f64 = 4.0e5;

fn canonical_bbox() -> Aabb3 {
    Aabb3::new(
        Vec3::new(-BBOX_HALF_EXTENT, -BBOX_HALF_EXTENT, -BBOX_HALF_EXTENT),
        Vec3::new(BBOX_HALF_EXTENT, BBOX_HALF_EXTENT, BBOX_HALF_EXTENT),
    )
}

fn build_with(material_field: Option<MaterialField>) -> SdfMeshedTetMesh {
    SdfMeshedTetMesh::from_sdf(
        &SphereSdf { radius: RADIUS },
        &MeshingHints {
            bbox: canonical_bbox(),
            cell_size: CELL_SIZE,
            material_field,
        },
    )
    .expect("canonical sphere scene should mesh successfully")
}

// ---------------------------------------------------------------------
// 1. `None` is observationally equal to explicit `Some(uniform(1e5, 4e5))`
// ---------------------------------------------------------------------

#[test]
fn none_path_matches_explicit_uniform_baseline() {
    let mesh_none = build_with(None);
    let mesh_explicit = build_with(Some(MaterialField::uniform(MU_BASELINE, LAMBDA_BASELINE)));

    // Geometry must match strictly: the field choice does not feed the
    // BCC + warp + stuffing pipeline; only `materials()` differs across
    // hint shapes, and only when the field constants differ.
    assert_eq!(mesh_none.n_vertices(), mesh_explicit.n_vertices());
    assert_eq!(mesh_none.n_tets(), mesh_explicit.n_tets());

    let pa = mesh_none.positions();
    let pb = mesh_explicit.positions();
    for v in 0..mesh_none.n_vertices() {
        for axis in 0..3 {
            assert_eq!(
                pa[v][axis].to_bits(),
                pb[v][axis].to_bits(),
                "position bit drift at vertex {v} axis {axis} between None and \
                 explicit-uniform paths",
            );
        }
    }

    // n_tets returns usize; TetId is u32. Canonical sphere stays well
    // below u32::MAX.
    #[allow(clippy::cast_possible_truncation)]
    let n_tets = mesh_none.n_tets() as TetId;
    for tet_id in 0..n_tets {
        assert_eq!(
            mesh_none.tet_vertices(tet_id),
            mesh_explicit.tet_vertices(tet_id),
            "tet {tet_id} vertex IDs drifted between None and explicit-uniform paths",
        );
    }

    let ma = mesh_none.materials();
    let mb = mesh_explicit.materials();
    assert_eq!(ma.len(), mb.len());
    for (nh_none, nh_explicit) in ma.iter().zip(mb.iter()) {
        assert_neo_hookean_bit_equal(nh_none, nh_explicit);
    }
}

// ---------------------------------------------------------------------
// 2. Value correctness — `None` synthesizes the documented Ecoflex baseline
// ---------------------------------------------------------------------

#[test]
fn none_path_synthesizes_ecoflex_baseline_value() {
    let mesh = build_with(None);
    let expected = NeoHookean::from_lame(MU_BASELINE, LAMBDA_BASELINE);
    assert!(
        !mesh.materials().is_empty(),
        "canonical sphere should produce a non-empty cache",
    );
    for nh in mesh.materials() {
        assert_neo_hookean_bit_equal(nh, &expected);
    }
}

// ---------------------------------------------------------------------
// 3. Non-uniform field flows through hints end-to-end
// ---------------------------------------------------------------------

#[test]
fn non_uniform_layered_field_threads_through_hints() {
    // 2-shell concentric `SphereSdf`-partitioned field on a r = 0.1 m
    // sphere. Threshold = 0.0 (the SDF zero set is implicitly inside
    // the meshed body for `radius < 0.1` and outside for the bbox
    // exterior; we partition on the inner shell at half-radius).
    //
    // Inner shell (`phi < THRESHOLD`): μ = 5.0e4, λ = 2.0e5.
    // Outer shell (`phi ≥ THRESHOLD`): μ = 2.0e5, λ = 8.0e5.
    //
    // Use a SECOND `SphereSdf` (smaller radius) as the partition: a
    // tet centroid c with `‖c‖ < 0.05` lands in the inner shell;
    // `‖c‖ ≥ 0.05` lands in the outer shell. The body's geometric
    // SDF (radius 0.1) is the meshing surface; the partition SDF
    // (radius 0.05) is the material discontinuity.
    const PARTITION_RADIUS: f64 = 0.05;
    const MU_INNER: f64 = 5.0e4;
    const LAMBDA_INNER: f64 = 2.0e5;
    const MU_OUTER: f64 = 2.0e5;
    const LAMBDA_OUTER: f64 = 8.0e5;

    let partition = || {
        Box::new(SphereSdf {
            radius: PARTITION_RADIUS,
        })
    };
    let mu_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        partition(),
        vec![0.0],
        vec![MU_INNER, MU_OUTER],
    ));
    let lambda_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        partition(),
        vec![0.0],
        vec![LAMBDA_INNER, LAMBDA_OUTER],
    ));
    let field = MaterialField::from_fields(mu_field, lambda_field);

    let mesh = build_with(Some(field));

    // Non-emptiness check — the meshed sphere has tets at the centre
    // (inner shell) and at the rim (outer shell); both populations
    // exist by construction.
    assert!(
        mesh.materials().len() > 100,
        "canonical sphere should produce many tets; got {}",
        mesh.materials().len(),
    );

    let expected_inner = NeoHookean::from_lame(MU_INNER, LAMBDA_INNER);
    let expected_outer = NeoHookean::from_lame(MU_OUTER, LAMBDA_OUTER);

    let positions = mesh.positions();
    let mut n_inner = 0usize;
    let mut n_outer = 0usize;

    // n_tets() returns usize; TetId is u32. Canonical sphere stays well
    // below u32::MAX.
    #[allow(clippy::cast_possible_truncation)]
    let n_tets = mesh.n_tets() as TetId;
    for tet_id in 0..n_tets {
        let vids = mesh.tet_vertices(tet_id);
        let v0 = positions[vids[0] as usize];
        let v1 = positions[vids[1] as usize];
        let v2 = positions[vids[2] as usize];
        let v3 = positions[vids[3] as usize];
        let centroid = (v0 + v1 + v2 + v3) * 0.25;
        let phi = centroid.norm() - PARTITION_RADIUS;

        let nh = &mesh.materials()[tet_id as usize];
        if phi < 0.0 {
            // Inner shell — strict bit-equality against expected pair.
            // `LayeredScalarField` uses `partition_point(|&t| t <= phi)`
            // so `phi < 0.0` lands in `values[0]` deterministically.
            assert_neo_hookean_bit_equal(nh, &expected_inner);
            n_inner += 1;
        } else {
            assert_neo_hookean_bit_equal(nh, &expected_outer);
            n_outer += 1;
        }
    }

    // Both shell populations must be non-empty — the partition radius
    // 0.05 sits well inside the meshed sphere of radius 0.1, so
    // centroids near origin land inner and centroids near the rim land
    // outer. A regression that drops one population would silently make
    // the test pass for the surviving population only.
    assert!(n_inner > 0, "expected at least one inner-shell tet");
    assert!(n_outer > 0, "expected at least one outer-shell tet");
}
