//! Tet10 ladder rung 3a — enriched-and-midside-pinned solve is
//! bit-identical to the un-enriched Tet4 solve.
//!
//! This is the *SOLVE* gate for rung 3a (not merely "constructs"): the
//! whole rung-3a-as-safe-plumbing rationale rests on this bit-identity.
//! A [`Tet10Mesh`] adds the six edge-midpoint nodes to a linear mesh's
//! positions but keeps [`Mesh::tet_vertices`] at four corners, so a Tet4
//! solver leaves every midside node tet-unreferenced and the
//! construction-time orphan auto-pin Dirichlet-clamps them. Dirichlet DOFs
//! are condensed out of the free block, and because enrichment appends the
//! midside ids *after* the corners (`positions[..n_corners]` bit-identical),
//! the corner free-index numbering — hence faer's fill-reducing permutation
//! over the free block, the symbolic pattern, and the lumped mass — is
//! unchanged. So the corner block of `x_final` must match the un-enriched
//! Tet4 solve bit-for-bit.
//!
//! Two fixtures exercise complementary halves of that claim:
//! - `two_tet_shared_face` (5 corners + 9 midsides, 42 DOF) — leaves a
//!   single free vertex, and its reference solve is cross-checked against
//!   the frozen `SHARED_FACE_X_FINAL` golden (the same capture pinned by
//!   `invariant_iv_1_uniform_passthrough.rs` / `contact_passthrough.rs`),
//!   so the bit-identity is anchored to the canonical baseline rather than
//!   a locally-recomputed number.
//! - a clamped-base `uniform_block(2)` cube — leaves a whole coupled layer
//!   of interior free vertices, so the free block is large enough that
//!   faer's fill-reducing permutation genuinely reorders. This is the
//!   fixture that stresses the "permutation over the free block is
//!   unchanged when midsides append after corners" link directly.
//!
//! Both are fast dense-FEM scenes, auto-discovered and run in CI by the
//! tests-debug harness. Bit-equality on this dense path is the contract
//! Decision P wants; see the two golden fixtures' module docs for the full
//! toolchain-fragility / re-capture protocol.

#![allow(
    // Bit-equality via `to_bits()` (u64) is the entire point of this gate.
    clippy::float_cmp,
    // `n_vertices() as VertexId` is the Mesh-trait API tax (usize count ->
    // u32 id): the test meshes hold ~dozens of vertices, far below u32::MAX.
    clippy::cast_possible_truncation
)]

use sim_ml_chassis::Tensor;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, CpuTet4NHSolver, HandBuiltTetMesh, LoadAxis,
    MaterialField, Mesh, NullContact, Solver, SolverConfig, Tet4, Tet10Mesh, VertexId,
};

/// Stage-1 θ magnitude — matches the golden fixtures' `run_shared_face`.
const THETA: f64 = 10.0;

/// Ecoflex-class Lamé parameters — matches the golden fixtures.
const MU: f64 = 1.0e5;
const LAMBDA: f64 = 4.0e5;

/// 2-tet shared-face reference: 5 vertices × 3 DOFs. Copied verbatim from
/// `invariant_iv_1_uniform_passthrough.rs::SHARED_FACE_X_FINAL` (captured at
/// main `c3729d4a`). The rung-3a bit-identity is anchored to this frozen
/// baseline — NEVER re-bake these values to make the test green (see that
/// fixture's re-capture failure-mode protocol).
const SHARED_FACE_X_FINAL: [u64; 15] = [
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x3fb9_9999_9999_999a,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x3fb9_9999_9999_999a,
    0x0000_0000_0000_0000,
    0x3f26_4080_b691_bfb7,
    0x3f26_4080_b691_bfb8,
    0x3fb9_c967_368a_3cc5,
    0x3fb4_7ae1_47ae_147b,
    0x3fb4_7ae1_47ae_147b,
    0x3fb4_7ae1_47ae_147b,
];

/// Flatten a mesh's rest positions into a `3·n_vertices` DOF vector.
fn rest_x_prev(mesh: &dyn Mesh) -> Vec<f64> {
    let positions = mesh.positions();
    let mut x = vec![0.0; 3 * positions.len()];
    for (v, pos) in positions.iter().enumerate() {
        x[3 * v] = pos.x;
        x[3 * v + 1] = pos.y;
        x[3 * v + 2] = pos.z;
    }
    x
}

/// Solve `tet4` and its [`Tet10Mesh`] enrichment under identical boundary
/// conditions, assert the enriched solve's corner block matches the Tet4
/// solve bit-for-bit and every pinned midside stays exactly at rest, then
/// return the Tet4 reference `x_final` (so a caller can additionally anchor
/// it to a frozen golden).
///
/// The corner id-space is preserved by enrichment, so the same `pinned` /
/// `loaded` vertex ids apply to both meshes verbatim; every midside id
/// (`>= n_corners`) is tet-unreferenced under `tet_vertices` and auto-pinned
/// at construction.
fn assert_enriched_bit_identical(
    tet4: HandBuiltTetMesh,
    pinned: &[VertexId],
    loaded: &[(VertexId, LoadAxis)],
) -> Vec<f64> {
    let cfg = SolverConfig::skeleton();
    let theta = Tensor::from_slice(&[THETA], &[1]);
    let bc = || BoundaryConditions {
        pinned_vertices: pinned.to_vec(),
        roller_vertices: Vec::new(),
        loaded_vertices: loaded.to_vec(),
    };
    let n_corner_dof = 3 * tet4.n_vertices();

    // Enrich BEFORE `tet4` is moved into the reference solver.
    let tet10 = Tet10Mesh::from_tet4(&tet4);
    assert_eq!(
        tet10.n_corners(),
        tet4.n_vertices(),
        "corner id-space must be preserved",
    );

    // Reference: un-enriched Tet4 solve.
    let x_prev_4 = Tensor::from_slice(&rest_x_prev(&tet4), &[n_corner_dof]);
    let v_prev_4 = Tensor::zeros(&[n_corner_dof]);
    let solver_4: CpuTet4NHSolver<HandBuiltTetMesh> =
        CpuNewtonSolver::new(Tet4, tet4, NullContact, cfg, bc());
    let ref_x_final = solver_4
        .replay_step(&x_prev_4, &v_prev_4, &theta, cfg.dt)
        .x_final;

    // Enriched-and-midside-pinned solve, same BCs.
    let enriched_rest = rest_x_prev(&tet10);
    let n_enriched_dof = enriched_rest.len();
    let x_prev_10 = Tensor::from_slice(&enriched_rest, &[n_enriched_dof]);
    let v_prev_10 = Tensor::zeros(&[n_enriched_dof]);
    let solver_10: CpuTet4NHSolver<Tet10Mesh> =
        CpuNewtonSolver::new(Tet4, tet10, NullContact, cfg, bc());
    let enriched_x_final = solver_10
        .replay_step(&x_prev_10, &v_prev_10, &theta, cfg.dt)
        .x_final;
    assert_eq!(
        enriched_x_final.len(),
        n_enriched_dof,
        "enriched solve returns one DOF per enriched vertex",
    );

    // The corner block must match the un-enriched Tet4 solve bit-for-bit.
    for (i, (&corner_val, &ref_val)) in enriched_x_final[..n_corner_dof]
        .iter()
        .zip(ref_x_final.iter())
        .enumerate()
    {
        assert_eq!(
            corner_val.to_bits(),
            ref_val.to_bits(),
            "corner DOF {i} diverged between the Tet4 solve ({ref_val:e}) and the \
             enriched midside-pinned Tet10Mesh solve ({corner_val:e}) — rung 3a is NOT \
             bit-identical Tet4. A midside DOF leaked into the free block (check the \
             orphan auto-pin) or enrichment reordered the corner id-space.",
        );
    }

    // The pinned midside DOFs must remain exactly at their rest positions
    // (inert clamped points) — a moved midside means it was not auto-pinned.
    for (i, (&got, &rest)) in enriched_x_final[n_corner_dof..]
        .iter()
        .zip(enriched_rest[n_corner_dof..].iter())
        .enumerate()
    {
        assert_eq!(
            got.to_bits(),
            rest.to_bits(),
            "midside DOF {i} moved from its rest position ({rest:e} -> {got:e}) — it was \
             not Dirichlet-clamped by the orphan auto-pin",
        );
    }

    ref_x_final
}

#[test]
fn enriched_midside_pinned_solve_is_bit_identical_tet4() {
    // Single-free-vertex fixture, anchored to the frozen golden.
    let field = MaterialField::uniform(MU, LAMBDA);
    let tet4 = HandBuiltTetMesh::two_tet_shared_face(&field);
    // Pins corners {0, 1, 2, 4}, loads corner 3 in +ẑ — the golden-capture BC.
    let ref_x_final = assert_enriched_bit_identical(tet4, &[0, 1, 2, 4], &[(3, LoadAxis::AxisZ)]);

    // Anchor the reference to the frozen golden baseline (provenance): a
    // vacuous bit-identity — two identically-wrong solves — is ruled out.
    assert_eq!(ref_x_final.len(), SHARED_FACE_X_FINAL.len());
    for (i, (val, &exp)) in ref_x_final
        .iter()
        .zip(SHARED_FACE_X_FINAL.iter())
        .enumerate()
    {
        assert_eq!(
            val.to_bits(),
            exp,
            "reference Tet4 x_final[{i}] drifted from the frozen SHARED_FACE golden — \
             diagnose per invariant_iv_1_uniform_passthrough.rs before touching Tet10",
        );
    }
}

#[test]
fn enriched_bit_identity_holds_on_a_multi_free_vertex_block() {
    // Clamped-base cube: pin the bottom face, load the top face in +ẑ, leaving
    // the middle layer of vertices free. The coupled free block is large
    // enough that faer's fill-reducing permutation genuinely reorders, which
    // is the specific link the single-free-vertex fixture above cannot stress.
    let edge = 0.1;
    let field = MaterialField::uniform(MU, LAMBDA);
    let cube = HandBuiltTetMesh::uniform_block(2, edge, &field);
    let positions = cube.positions();
    let n = cube.n_vertices() as VertexId;

    // Bottom layer (z ≈ 0) pinned; top layer (z ≈ edge) loaded. The middle
    // layer (z ≈ edge/2) is free and interior-coupled.
    let pinned: Vec<VertexId> = (0..n)
        .filter(|&v| positions[v as usize].z < 0.25 * edge)
        .collect();
    let loaded: Vec<(VertexId, LoadAxis)> = (0..n)
        .filter(|&v| positions[v as usize].z > 0.75 * edge)
        .map(|v| (v, LoadAxis::AxisZ))
        .collect();
    assert!(
        !pinned.is_empty() && !loaded.is_empty(),
        "fixture must have both clamped and loaded vertices",
    );

    // No golden anchor needed: the first test proves the Tet4 solve path is
    // the canonical baseline; this one asserts the enriched solve reproduces
    // whatever the Tet4 solve produces on a permutation-stressing block.
    let _ = assert_enriched_bit_identical(cube, &pinned, &loaded);
}
