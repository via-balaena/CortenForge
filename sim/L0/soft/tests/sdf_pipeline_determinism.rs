//! III-1 — SDF→tet pipeline determinism.
//!
//! Two consecutive runs of the placeholder mesher on the canonical
//! sphere scene (radius `0.1` m, bbox `[-0.12, 0.12]³`, `cell_size`
//! `0.02`) produce mesh-equal output: same vertex count, same tet
//! count, bit-equal vertex positions, bit-equal per-tet vertex IDs,
//! and bit-equal per-tet `QualityMetrics` (`signed_volume`,
//! `aspect_ratio`, `dihedral_min`, `dihedral_max`).
//!
//! The bit-equality bar is strict on every f64 the mesher writes,
//! including the derivation chain — `signed_volume` via `det / 6`,
//! `aspect_ratio` via `r_ins / r_circ` involving `sqrt`, `dihedral_*`
//! via `acos` of normalized dot products. f64 `sqrt` and `acos` are
//! deterministic on a fixed hardware/libm pair (Phase 1+2 already
//! rely on this); the strict bar catches any future drift toward
//! non-determinism in the mesh-construction or quality-kernel paths.
//!
//! Scope: III-1 is a **non-determinism regression net**. It surfaces
//! any future code change that introduces run-to-run output drift —
//! a `HashMap` whose iteration order leaks into the numeric path,
//! `rayon` parallelism without deterministic reduction order,
//! uninitialized memory reads, ASLR-dependent operations. It does
//! NOT verify design correctness: a deterministic D-8 / D-9 / D-10
//! / stencil regression would produce the same wrong output on both
//! runs and pass III-1 vacuously. Those are caught at other layers
//! — see `sdf_meshed_tet_mesh.rs`'s inline
//! `diagonal_edge_cuts_dedup_to_unique_vertex_ids_per_unique_edge`
//! test (D-9), the `case_index_zero_sdf_classified_as_inside` test
//! in `marching_tet` (D-8), and the downstream III-2 quality bounds
//! + III-3 gradcheck for the rest.

// `build()` panics on unexpected meshing failure via `.expect()` —
// that IS the test failure signal under the inline-test convention.
// The canonical sphere scene either succeeds by construction or
// surfaces a regression worth investigating.
#![allow(clippy::expect_used)]

use sim_soft::sdf_bridge::Aabb3;
use sim_soft::sdf_bridge::{MeshingHints, SdfMeshedTetMesh, SphereSdf};
use sim_soft::{Mesh, TetId, VertexId};

const RADIUS: f64 = 0.1;
const CELL_SIZE: f64 = 0.02;
const BBOX_HALF_EXTENT: f64 = 0.12;

fn build() -> SdfMeshedTetMesh {
    let hints = MeshingHints {
        bbox: Aabb3::new(
            sim_soft::Vec3::new(-BBOX_HALF_EXTENT, -BBOX_HALF_EXTENT, -BBOX_HALF_EXTENT),
            sim_soft::Vec3::new(BBOX_HALF_EXTENT, BBOX_HALF_EXTENT, BBOX_HALF_EXTENT),
        ),
        cell_size: CELL_SIZE,
    };
    SdfMeshedTetMesh::from_sdf(&SphereSdf { radius: RADIUS }, &hints)
        .expect("canonical sphere scene should mesh successfully")
}

#[test]
fn sphere_mesh_construction_succeeds_with_non_trivial_size() {
    // Sanity floor for the rest of the suite — a degenerate empty
    // mesh would silently make every following bit-equality
    // assertion vacuous.
    let mesh = build();
    assert!(
        mesh.n_tets() > 100,
        "canonical sphere scene should produce a substantial mesh; got {} tets",
        mesh.n_tets(),
    );
    assert!(mesh.n_vertices() > 100);
}

#[test]
fn vertex_and_tet_counts_are_run_to_run_stable() {
    let a = build();
    let b = build();
    assert_eq!(a.n_vertices(), b.n_vertices());
    assert_eq!(a.n_tets(), b.n_tets());
}

#[test]
fn vertex_positions_are_bit_equal_across_runs() {
    // Bit-equality via f64::to_bits — the strictest possible run-to-
    // run guard, catching any FP-rounding drift the BTreeMap-walk
    // path could leak.
    let a = build();
    let b = build();
    assert_eq!(a.n_vertices(), b.n_vertices());
    let pa = a.positions();
    let pb = b.positions();
    for v in 0..a.n_vertices() {
        for axis in 0..3 {
            assert_eq!(
                pa[v][axis].to_bits(),
                pb[v][axis].to_bits(),
                "position bit drift at vertex {v} axis {axis}: {} vs {}",
                pa[v][axis],
                pb[v][axis],
            );
        }
    }
}

#[test]
fn tet_vertex_indices_are_run_to_run_stable() {
    let a = build();
    let b = build();
    assert_eq!(a.n_tets(), b.n_tets());
    // n_tets returns usize; TetId is u32. Mesher output stays well
    // below u32::MAX for any plausible scene (canonical III-1 has
    // ~10k tets pre-filter).
    #[allow(clippy::cast_possible_truncation)]
    let n_tets = a.n_tets() as TetId;
    for tet_id in 0..n_tets {
        let va = a.tet_vertices(tet_id);
        let vb = b.tet_vertices(tet_id);
        assert_eq!(va, vb, "tet {tet_id} vertex IDs drifted: {va:?} vs {vb:?}");
    }
}

#[test]
fn quality_metrics_fields_are_bit_equal_across_runs() {
    // Strict bit-equality on every f64 in QualityMetrics catches
    // future non-determinism in the kernel chain (det / sqrt / acos)
    // at this layer, before it leaks into III-2 or III-3.
    let a = build();
    let b = build();
    let qa = a.quality();
    let qb = b.quality();
    assert_eq!(qa.signed_volume.len(), qb.signed_volume.len());
    for tet_id in 0..qa.signed_volume.len() {
        assert_eq!(
            qa.signed_volume[tet_id].to_bits(),
            qb.signed_volume[tet_id].to_bits(),
            "signed_volume bit drift at tet {tet_id}",
        );
        assert_eq!(
            qa.aspect_ratio[tet_id].to_bits(),
            qb.aspect_ratio[tet_id].to_bits(),
            "aspect_ratio bit drift at tet {tet_id}",
        );
        assert_eq!(
            qa.dihedral_min[tet_id].to_bits(),
            qb.dihedral_min[tet_id].to_bits(),
            "dihedral_min bit drift at tet {tet_id}",
        );
        assert_eq!(
            qa.dihedral_max[tet_id].to_bits(),
            qb.dihedral_max[tet_id].to_bits(),
            "dihedral_max bit drift at tet {tet_id}",
        );
    }
}

#[test]
fn output_mesh_is_strictly_right_handed_post_filter() {
    // III-1's strict-positivity bar on every output sub-tet is the
    // structural-bug detector: any algorithm path producing a left-
    // handed sub-tet (and thereby getting orientation-swapped
    // post-hoc to RH) would still pass; any path producing a sub-tet
    // with absolute volume below EPSILON_VOLUME (1e-15) would be
    // filtered. So `signed_volume > 0` strict is exactly the
    // contract D-10 names.
    let mesh = build();
    for (tet_id, &v) in mesh.quality().signed_volume.iter().enumerate() {
        assert!(
            v > 0.0,
            "tet {tet_id} has non-positive signed volume {v:e} — contract violation",
        );
    }
}

#[test]
fn mesh_equals_structurally_to_a_repeat_run() {
    // A weaker but more-readable cross-check: equals_structurally
    // asserts (n_vertices, n_tets, per-tet VertexIds) match. The
    // earlier strict-bit-equality tests subsume this, but
    // equals_structurally is the load-bearing API for downstream
    // change-detection (Mesh trait claim 3 in Part 11 Ch 00 §02);
    // exercising it on the SDF-meshed path holds it accountable.
    let a = build();
    let b = build();
    assert!(a.equals_structurally(&b));
}

#[test]
fn vertex_id_zero_is_referenced_by_at_least_one_tet() {
    // Sanity — orphan grid corners can exist (lattice-on-sphere
    // coincidences via D-10 filter) but VertexId 0 is the first
    // emitted vertex by the deterministic walk, and every standard
    // case path that emits sub-tets does so referencing some early
    // VertexIds. If the first VertexId is orphaned, that signals
    // a deeper case-table or vertex-cache bug worth investigating
    // before downstream III-3 trips on it.
    let mesh = build();
    let mut found = false;
    // n_tets returns usize; TetId is u32. III-1 mesh stays below
    // u32::MAX tets.
    #[allow(clippy::cast_possible_truncation)]
    let n_tets = mesh.n_tets() as VertexId;
    for tet_id in 0..n_tets {
        if mesh.tet_vertices(tet_id).contains(&0) {
            found = true;
            break;
        }
    }
    assert!(
        found,
        "VertexId 0 is orphaned — unexpected for the canonical sphere scene",
    );
}
