//! IV-2 — interface continuity on a 2-region bilayer hand-built mesh.
//!
//! Phase 4 scope memo §6 IV-2 + §8 commit 7. Verifies the multi-material
//! Hessian assembly produces the structurally correct shared-vertex
//! displacement when **Region A** (Ecoflex 00-30 baseline, μ=1.0e5,
//! λ=4.0e5) and **Region B** (Ecoflex 00-30 + 15 wt% carbon-black at
//! 2.0× stiffness per Decision J, μ=2.0e5, λ=8.0e5) are assigned to the
//! two tets of [`HandBuiltTetMesh::two_tet_shared_face`] via a
//! [`SphereSdf`]-partitioned [`LayeredScalarField`] with radius 0.06 —
//! strictly between the two tets' centroid norms (0.04330 m and
//! 0.07794 m).
//!
//! **Scene reuse.** `two_tet_shared_face` already accepts `&MaterialField`
//! per Phase 4 commit 4; commit 7 ships pure-test (zero production
//! touches), composing the bilayer field inline rather than adding a
//! redundant `with_material_field` constructor that would have an
//! identical body to the existing one. Mirrors the IV-1 (commit 6)
//! pure-test discipline.
//!
//! **Three lenses, three `#[test]` blocks:**
//!
//! 1. [`iv_2_bilayer_mesh_has_distinct_per_tet_materials`] — pre-flight
//!    that the partition actually produces a heterogeneous cache (lens
//!    γ). Catches misconfigured thresholds that degenerate to uniform.
//! 2. [`iv_2_shared_vertex_displacement_continuity_holds`] — read each
//!    shared vertex's displacement via tet 0's connectivity and via
//!    tet 1's, assert bit-equal (lens α). In correct FEM this is a
//!    tautology (`x_final` is global; both lookups index the same
//!    `Vec`); the assertion documents the contract and would catch
//!    contrived bugs (per-tet `x_final` shadows, off-by-one indexing).
//!    Plus non-triviality: shared vertex `v_3` must displace measurably
//!    under the `THETA = 10.0` load.
//! 3. [`iv_2_bilayer_v3_displacement_lies_between_uniform_bounds`] —
//!    the discriminating physical assertion (lens β). `v_3` is the
//!    only free + loaded vertex, governed by Hessian contributions
//!    from both tet 0 (region A in bilayer) and tet 1 (region B in
//!    bilayer). Three scenes are run — uniform-A everywhere (both
//!    tets soft) gives the largest `|d_A|`; uniform-B everywhere
//!    (both tets stiff) gives the smallest `|d_B|`; the bilayer (A in
//!    tet 0, B in tet 1) gives a `|d_bilayer|` strictly between the
//!    two. Catches dropped-tet-contribution / swapped-materials /
//!    mis-assigned-material bugs — all of which would push
//!    `d_bilayer` to one of the bounds (or outside).
//!
//! **Boundary conditions** mirror IV-1's `two_tet_shared_face` capture
//! exactly: `pinned_vertices = [0, 1, 2, 4]`, `loaded_vertices = [(3,
//! AxisZ)]`. `v_3` is simultaneously the only free vertex AND the only
//! loaded one AND a shared interface vertex — the geometric coincidence
//! makes the test discriminating: every assertion reads displacement at
//! a shared interface vertex governed by both regions' stiffnesses
//! through the global Hessian.

#![allow(
    // Expect-on-Option for Vec::position lookups inside the test loop is
    // a contract violation if it fires — every shared vertex must appear
    // in both tets by construction. Panicking is the right test failure.
    clippy::expect_used,
    // Direct displacement comparisons drive lens β's discriminating
    // inequality assertions; bit-equal slice reads in lens α are also
    // intentional.
    clippy::float_cmp
)]

mod common;

use common::assert_neo_hookean_bit_equal;
use sim_ml_chassis::Tensor;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, CpuTet4NHSolver, Field, HandBuiltTetMesh,
    LayeredScalarField, LoadAxis, MaterialField, Mesh, NeoHookean, NullContact, Solver,
    SolverConfig, SphereSdf, Tet4,
};

// ── Material parameters per scope memo Decision J ─────────────────────────

/// Region A: Ecoflex 00-30 baseline (`μ = 1.0e5`, `λ = 4.0e5`). Same
/// `(μ, λ)` as IV-1's MU/LAMBDA constants — region A is the softer of
/// the two regions.
const MU_A: f64 = 1.0e5;
const LAMBDA_A: f64 = 4.0e5;

/// Region B: Ecoflex 00-30 + 15 wt% carbon-black at 2.0× stiffness per
/// Decision J (Part 1 §04 §02 carbon-black correlation, mechanical-only
/// at this phase). 2× the baseline `(μ, λ)`.
const MU_B: f64 = 2.0e5;
const LAMBDA_B: f64 = 8.0e5;

// ── Geometry partition ────────────────────────────────────────────────────

/// [`SphereSdf`] radius placing tet 0 strictly inside (region A) and
/// tet 1 strictly outside (region B).
///
/// Centroid norms on `two_tet_shared_face`:
///
/// - tet 0 centroid `(0.025, 0.025, 0.025)`, norm = √3 × 0.025 ≈ 0.04330
/// - tet 1 centroid `(0.045, 0.045, 0.045)`, norm = √3 × 0.045 ≈ 0.07794
///
/// Radius 0.06 is strictly between the two — `LayeredScalarField`'s
/// `partition_point(|&t| t <= phi)` boundary rule clears either centroid
/// by ≥ 0.0167 m, so the bilayer cache is robust to floating-point
/// drift in centroid evaluation.
const PARTITION_RADIUS: f64 = 0.06;

// ── Load magnitude ────────────────────────────────────────────────────────

/// Stage-1 θ magnitude — same value as IV-1's `THETA` and
/// `solver_convergence::stage_1_traction_converges`. Small-strain regime
/// at this load on the decimeter-scale tets.
const THETA: f64 = 10.0;

// ── Helpers ───────────────────────────────────────────────────────────────

fn bilayer_field() -> MaterialField {
    let mu_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        Box::new(SphereSdf {
            radius: PARTITION_RADIUS,
        }),
        vec![0.0],
        vec![MU_A, MU_B],
    ));
    let lambda_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        Box::new(SphereSdf {
            radius: PARTITION_RADIUS,
        }),
        vec![0.0],
        vec![LAMBDA_A, LAMBDA_B],
    ));
    MaterialField::from_fields(mu_field, lambda_field)
}

fn shared_face_bc() -> BoundaryConditions {
    BoundaryConditions {
        pinned_vertices: vec![0, 1, 2, 4],
        loaded_vertices: vec![(3, LoadAxis::AxisZ)],
    }
}

fn run_step_with_field(field: &MaterialField) -> Vec<f64> {
    let cfg = SolverConfig::skeleton();
    let mesh = HandBuiltTetMesh::two_tet_shared_face(field);
    let positions = mesh.positions();
    let n_dof = 3 * positions.len();
    let mut x_prev_flat = vec![0.0; n_dof];
    for (v, pos) in positions.iter().enumerate() {
        x_prev_flat[3 * v] = pos.x;
        x_prev_flat[3 * v + 1] = pos.y;
        x_prev_flat[3 * v + 2] = pos.z;
    }
    let x_prev = Tensor::from_slice(&x_prev_flat, &[n_dof]);
    let v_prev = Tensor::zeros(&[n_dof]);
    let solver: CpuTet4NHSolver<HandBuiltTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, shared_face_bc());
    let theta_tensor = Tensor::from_slice(&[THETA], &[1]);
    let step = solver.replay_step(&x_prev, &v_prev, &theta_tensor, cfg.dt);
    step.x_final
}

// ── Tests ─────────────────────────────────────────────────────────────────

#[test]
fn iv_2_bilayer_mesh_has_distinct_per_tet_materials() {
    // Lens γ pre-flight: confirm the SphereSdf partition actually
    // produces a heterogeneous bilayer cache, not a degenerate uniform
    // cache from a misconfigured threshold. NeoHookean's scalar fields
    // are private (commit 1 surface lock), so the comparison goes
    // through `assert_neo_hookean_bit_equal` (Material-trait surface,
    // first_piola at F = diag(2, 1, 1) — necessary and sufficient for
    // (μ, λ) bit-equality per the helper's docstring).
    //
    // Distinctness follows transitively: bit-equal-to-A and
    // bit-equal-to-B with MU_A ≠ MU_B implies the two tets carry
    // distinct materials.
    let mesh = HandBuiltTetMesh::two_tet_shared_face(&bilayer_field());
    assert_eq!(mesh.materials().len(), 2);
    let expected_a = NeoHookean::from_lame(MU_A, LAMBDA_A);
    let expected_b = NeoHookean::from_lame(MU_B, LAMBDA_B);
    assert_neo_hookean_bit_equal(&mesh.materials()[0], &expected_a);
    assert_neo_hookean_bit_equal(&mesh.materials()[1], &expected_b);
}

#[test]
fn iv_2_shared_vertex_displacement_continuity_holds() {
    // Lens α — read each shared vertex's displacement via tet 0's
    // connectivity and via tet 1's, assert bit-equal. In correct FEM
    // this is a tautology (x_final is global; both lookups index the
    // same Vec). The assertion documents the contract and would catch
    // contrived bugs (per-tet x_final shadows, off-by-one indexing).
    //
    // Plus non-triviality: shared vertex v_3 is the only loaded + free
    // vertex; it must displace measurably under THETA = 10.0 load.
    // Without this assertion the tautology test could pass on a
    // degenerate scene where x_final is bit-equal to x_prev everywhere.
    let x_final = run_step_with_field(&bilayer_field());
    let mesh = HandBuiltTetMesh::two_tet_shared_face(&bilayer_field());
    let tet0 = mesh.tet_vertices(0); // [0, 1, 2, 3]
    let tet1 = mesh.tet_vertices(1); // [1, 2, 3, 4]

    for shared_v in [1u32, 2, 3] {
        let pos_in_tet0 = tet0
            .iter()
            .position(|&v| v == shared_v)
            .expect("shared vertex must appear in tet 0");
        let pos_in_tet1 = tet1
            .iter()
            .position(|&v| v == shared_v)
            .expect("shared vertex must appear in tet 1");
        let global_v_from_tet0 = tet0[pos_in_tet0];
        let global_v_from_tet1 = tet1[pos_in_tet1];
        assert_eq!(
            global_v_from_tet0, global_v_from_tet1,
            "tautology: same global vertex id from either tet's connectivity"
        );
        let from_tet0 =
            &x_final[3 * global_v_from_tet0 as usize..3 * global_v_from_tet0 as usize + 3];
        let from_tet1 =
            &x_final[3 * global_v_from_tet1 as usize..3 * global_v_from_tet1 as usize + 3];
        assert_eq!(
            from_tet0, from_tet1,
            "tautology: x_final at shared vertex {shared_v} must read identically from \
             either tet's connectivity"
        );
    }

    // Non-triviality: v_3 rest position is (0, 0, 0.1); under +AxisZ
    // load it must displace upward by a measurable amount.
    let v3_z = x_final[3 * 3 + 2];
    let rest_v3_z = 0.1;
    let displacement = v3_z - rest_v3_z;
    assert!(
        displacement > 1e-6,
        "v_3 must displace measurably under +AxisZ load (got {displacement:e}); a near-zero \
         displacement signals either degenerate stiffness or a misconfigured BC"
    );
}

#[test]
fn iv_2_bilayer_v3_displacement_lies_between_uniform_bounds() {
    // Lens β — the discriminating assertion. v_3 is the only free +
    // loaded vertex, governed by Hessian contributions from both tet 0
    // (region A in bilayer) and tet 1 (region B in bilayer).
    //
    // Three scenes:
    //   - uniform-A everywhere (both tets soft)   → |d_A| largest
    //   - uniform-B everywhere (both tets stiff)  → |d_B| smallest
    //   - bilayer (A in tet 0, B in tet 1)        → |d_bilayer| between
    //
    // Catches dropped-tet-contribution / swapped-materials /
    // mis-assigned-material bugs — all of which would push d_bilayer
    // to one of the bounds (or outside).
    let x_uniform_a = run_step_with_field(&MaterialField::uniform(MU_A, LAMBDA_A));
    let x_uniform_b = run_step_with_field(&MaterialField::uniform(MU_B, LAMBDA_B));
    let x_bilayer = run_step_with_field(&bilayer_field());

    let rest_v3_z = 0.1;
    let d_a = x_uniform_a[3 * 3 + 2] - rest_v3_z;
    let d_b = x_uniform_b[3 * 3 + 2] - rest_v3_z;
    let d_bilayer = x_bilayer[3 * 3 + 2] - rest_v3_z;

    assert!(
        d_a > 0.0,
        "uniform-A v_3 must displace upward (got {d_a:e})"
    );
    assert!(
        d_b > 0.0,
        "uniform-B v_3 must displace upward (got {d_b:e})"
    );
    assert!(
        d_bilayer > 0.0,
        "bilayer v_3 must displace upward (got {d_bilayer:e})"
    );

    // Region B (μ=2e5, λ=8e5) is 2× stiffer than region A → uniform-B's
    // compliance is smaller → |d_B| < |d_A|. Sanity check that the
    // uniform bounds are themselves ordered as expected before asserting
    // the bilayer falls between them.
    assert!(
        d_b < d_a,
        "uniform-B (stiffer) must displace less than uniform-A (softer); \
         d_b={d_b:e}, d_a={d_a:e}"
    );

    // The bilayer's Hessian at v_3 has contributions from tet 0 (soft
    // region A) AND tet 1 (stiff region B). The aggregated stiffness
    // is strictly between uniform-A and uniform-B → bilayer
    // displacement is strictly between |d_A| and |d_B|.
    assert!(
        d_b < d_bilayer,
        "bilayer displacement must exceed uniform-B's (one of the two tets is the softer \
         region A); d_bilayer={d_bilayer:e}, d_b={d_b:e}"
    );
    assert!(
        d_bilayer < d_a,
        "bilayer displacement must fall short of uniform-A's (one of the two tets is the \
         stiffer region B); d_bilayer={d_bilayer:e}, d_a={d_a:e}"
    );
}
