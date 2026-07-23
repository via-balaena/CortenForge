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
//! The reference Tet4 solve is additionally cross-checked against the
//! frozen `SHARED_FACE_X_FINAL` golden bits (the same capture pinned by
//! `invariant_iv_1_uniform_passthrough.rs` and `contact_passthrough.rs`),
//! so the bit-identity below is anchored to the canonical baseline, not to
//! a locally-recomputed number.
//!
//! Fast dense-FEM scene (5 corners + 9 midsides, 42 DOF) — auto-discovered
//! and run in CI by the tests-debug harness. Bit-equality on this dense
//! path is the contract Decision P wants; see the two golden fixtures'
//! module docs for the full toolchain-fragility / re-capture protocol.

#![allow(
    // Bit-equality via `to_bits()` (u64) is the entire point of this gate.
    clippy::float_cmp
)]

use sim_ml_chassis::Tensor;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, CpuTet4NHSolver, HandBuiltTetMesh, LoadAxis,
    MaterialField, Mesh, NullContact, Solver, SolverConfig, Tet4, Tet10Mesh,
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

/// Boundary conditions shared by both solves — identical because the corner
/// id-space is preserved by enrichment. Pins corners {0, 1, 2, 4}, loads
/// corner 3 in `+ẑ`; every midside id (>= 5) is tet-unreferenced under
/// `tet_vertices` and auto-pinned at construction.
fn shared_face_bc() -> BoundaryConditions {
    BoundaryConditions {
        pinned_vertices: vec![0, 1, 2, 4],
        roller_vertices: Vec::new(),
        loaded_vertices: vec![(3, LoadAxis::AxisZ)],
    }
}

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

#[test]
fn enriched_midside_pinned_solve_is_bit_identical_tet4() {
    let cfg = SolverConfig::skeleton();
    let field = MaterialField::uniform(MU, LAMBDA);
    let theta = Tensor::from_slice(&[THETA], &[1]);

    // ── Reference: un-enriched Tet4 solve ────────────────────────────────
    let tet4 = HandBuiltTetMesh::two_tet_shared_face(&field);
    let n_corner_dof = 3 * tet4.n_vertices();
    let x_prev_4 = Tensor::from_slice(&rest_x_prev(&tet4), &[n_corner_dof]);
    let v_prev_4 = Tensor::zeros(&[n_corner_dof]);
    let solver_4: CpuTet4NHSolver<HandBuiltTetMesh> =
        CpuNewtonSolver::new(Tet4, tet4, NullContact, cfg, shared_face_bc());
    let ref_x_final = solver_4
        .replay_step(&x_prev_4, &v_prev_4, &theta, cfg.dt)
        .x_final;

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

    // ── Tet10Mesh enrichment of the same scene, same BCs ─────────────────
    let tet10 = Tet10Mesh::from_tet4(&HandBuiltTetMesh::two_tet_shared_face(&field));
    let n_corners = tet10.n_corners();
    assert_eq!(
        n_corners,
        n_corner_dof / 3,
        "corner id-space must be preserved"
    );
    let enriched_rest = rest_x_prev(&tet10);
    let n_enriched_dof = enriched_rest.len();
    let x_prev_10 = Tensor::from_slice(&enriched_rest, &[n_enriched_dof]);
    let v_prev_10 = Tensor::zeros(&[n_enriched_dof]);
    let solver_10: CpuTet4NHSolver<Tet10Mesh> =
        CpuNewtonSolver::new(Tet4, tet10, NullContact, cfg, shared_face_bc());
    let enriched_x_final = solver_10
        .replay_step(&x_prev_10, &v_prev_10, &theta, cfg.dt)
        .x_final;

    // The enriched solve carries the appended midsides, so its DOF vector is
    // longer; the corner block must match the reference bit-for-bit.
    assert_eq!(
        enriched_x_final.len(),
        n_enriched_dof,
        "enriched solve returns one DOF per enriched vertex",
    );
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
}
