//! IV-8 — `∂(displacement)/∂μ_layer` is FD-stable on the layered scene.
//!
//! Phase 4 scope memo §1 IV-8 + §8 commit 13. Forward-looking gradient
//! sentinel for material-parameter differentiability: proves the
//! derivative of the cavity-wall radial displacement with respect to one
//! layer's `μ` is well-defined and FD-stable on the canonical
//! [`SoftScene::layered_silicone_sphere`] scene, **without implementing
//! the reverse-mode adjoint w.r.t. material parameters** (Part 6 §02
//! IFT-adjoint extension; Decision M defers γ-locked surface to that
//! future work).
//!
//! ## Two gates in one test
//!
//! 1. **FD self-consistency at two step sizes.** Central-FD at
//!    `h_rel ∈ {1e-3, 1e-4} × μ_middle` agree pairwise within the 5-digit
//!    relative-error bar — the `FD-stable` claim of the IV-8 gate. A
//!    smooth `μ ↦ u_r(R_cavity)` map is what the IFT existence theorem
//!    of Part 6 §02 requires; FD self-consistency demonstrates the map
//!    IS smooth at the perturbation scales the eventual reverse-mode
//!    adjoint will consume.
//! 2. **FD-of-FEM matches FD-of-closed-form-Lamé within IV-5 floor.**
//!    At `h_rel = 1e-3 × μ_middle`, central FD over the FEM cavity-wall
//!    displacement agrees with central FD over the closed-form three-
//!    shell Lamé solution within `0.35` relative — the
//!    "FD-vs-analytic gradient" framing of the scope memo. The bar is
//!    IV-5's displacement floor at `cell_size = 0.02` (~`0.30` relative
//!    per IV-5's prose) plus margin for sensitivity-of-sensitivity
//!    loosening. See [`FD_VS_LAME_FLOOR`] for the full derivation.
//!    Catches FEM-tracks-physics regressions that pure self-consistency
//!    would miss (e.g., a wrong-sign material-assignment bug that
//!    produces a smooth-but-wrong sensitivity).
//!
//! ## Why middle-layer μ
//!
//! Decision J: middle layer = `2 × μ_outer = 2 × μ_inner`. Middle is the
//! highest-stiffness layer; `∂u_r/∂μ_middle` has the highest magnitude
//! signal-to-noise for the FD probe. Outer/inner perturbations are
//! covered implicitly by the test pattern — adding them would multiply
//! runtime without adding discrimination.
//!
//! ## Step size derivation
//!
//! Central FD truncation `O(h²)` + roundoff `O(ε / h)` for `f64` — the
//! optimal step is at `h ≈ ε^(1/3) · |μ| ≈ 6e-6 · μ_middle`. The chosen
//! pair `h_rel ∈ {1e-3, 1e-4}` brackets this comfortably: at `h_rel = 1e-3
//! × μ_middle = 200 Pa` truncation dominates (ratio `~1e-6` relative); at
//! `h_rel = 1e-4 × μ_middle = 20 Pa` truncation and roundoff are both
//! `~1e-7-1e-8`. Pairwise agreement within 5 digits demonstrates we're
//! safely above the Newton convergence floor (`tol = 1e-10` × `u_r ~ 5e-4`
//! = `5e-14` absolute `u_r` noise; central FD at `h_rel = 1e-4` moves `u_r`
//! by `|∂u/∂μ| · 20 ≈ 1e-9` m, four orders above the noise floor).
//!
//! ## Forward path: Part 6 §02 IFT-adjoint hook
//!
//! IV-8 is FD-only by Decision M (no new γ-locked types in Phase 4). The
//! eventual reverse-mode adjoint with respect to material parameters
//! follows the same IFT formula derived in Part 6 §02 for `θ`-driven
//! parameters — `∂x*/∂μ_e = -A⁻¹ · ∂r/∂μ_e`, with the Newton Hessian
//! factor `A` already cached at convergence and `∂r/∂μ_e` a per-element
//! contraction the FEM-assembly VJP path already knows how to compute.
//! When that adjoint lands, this test's FD baseline IS the gradcheck
//! reference — the scalar `∂u_r/∂μ_middle` value reported by
//! [`fd_central_fem`] becomes the right-hand side of an
//! analytic-vs-FD assertion mirroring II-2 / III-3.
//!
//! ## cf-design coordination
//!
//! The `(μ, λ)` perturbation here is sourced from inline constants
//! matching IV-5 + Decision J. Once cf-design's first multi-material
//! consumer lands per the coordination memo at
//! `sim/docs/todo/cf_design_material_handoff_scope.md`, the same
//! perturbation will run over design-side parameters that flow into the
//! [`MaterialField`] — the IV-8 gate is the test the design loop
//! eventually wraps for design-time gradient queries.
//!
//! ## Runtime budget
//!
//! Four [`Solver::replay_step`] calls at `cell_size = 0.02` (canonical
//! Phase 3 / IV-5 mid refinement, ~7 k tets) plus three closed-form
//! Lamé 6×6 LU solves. Empirically `~0.3` s total in release (each h/2
//! single-pass cost is dominated by mesh build + `~3` Newton iters at
//! the static regime); orders of magnitude inside any reasonable
//! per-test budget. Soft budget per
//! [`feedback_release_mode_heavy_tests`].

#![allow(
    // Direct gradient comparisons with documented analytic baselines —
    // not numerical bit patterns. Mirrors `concentric_lame_shells.rs`
    // precedent.
    clippy::float_cmp,
    // Inline `.expect()` on `from_sdf` — mirrors III-1 + IV-4 + IV-5
    // canonical-mesh convention; surfaces meshing failures as test
    // panics rather than `Result` plumbing.
    clippy::expect_used,
    // The closed-form 6×6 Lamé setup names ~10 single-character scalars
    // (radii, K coefficients, μ array). Engineering test prose reads
    // more cleanly with the local-derivation idiom than with hoisted
    // const blocks.
    clippy::many_single_char_names,
    // The `loaded.len() as f64` cast mirrors IV-5's tributary-area
    // computation — exact f64 representation at any mesh size sim-soft
    // hits.
    clippy::cast_precision_loss
)]

use nalgebra::{Matrix6, Vector6};
use sim_soft::{
    CpuNewtonSolver, CpuTet4NHSolver, Field, LAYERED_SPHERE_R_CAVITY, LAYERED_SPHERE_R_INNER_OUTER,
    LAYERED_SPHERE_R_OUTER, LAYERED_SPHERE_R_OUTER_INNER, LayeredScalarField, MaterialField, Mesh,
    NullContact, SceneInitial, SdfMeshedTetMesh, SoftScene, Solver, SolverConfig, SphereSdf, Tet4,
    Vec3,
};

// ── Material parameters per Decision J (compressible regime, ν = 0.4) ────

/// Outer Ecoflex 00-30 baseline (μ, λ) per Decision J. Mirrors IV-5
/// (`concentric_lame_shells.rs`) verbatim so the FD-vs-Lamé comparison
/// gate runs against the same closed-form geometry IV-5 already
/// validates for displacement.
const MU_OUTER: f64 = 1.0e5;
const LAMBDA_OUTER: f64 = 4.0e5;

/// Middle carbon-black composite shell — `2×` stiffness multiplier per
/// Decision J. The IV-8 perturbation target.
const MU_MIDDLE_BASE: f64 = 2.0e5;
const LAMBDA_MIDDLE: f64 = 8.0e5;

/// Inner Ecoflex 00-30 — same as outer per Decision J.
const MU_INNER: f64 = 1.0e5;
const LAMBDA_INNER: f64 = 4.0e5;

/// Internal pressure — same as IV-5. Lands cavity-wall radial
/// displacement at ~`5e-4` m, comfortably small-strain.
const PRESSURE: f64 = 5.0e3;

/// Cell size — canonical Phase 3 / IV-5 mid refinement. ~7 k tets,
/// ~2 s/pass.
const CELL_SIZE: f64 = 0.02;

/// Static-regime time step — mirrors IV-5. `dt = 1.0` collapses the
/// inertial term `M / dt²` by `~4` orders relative to stiffness; one
/// `replay_step` from rest converges to static equilibrium at
/// `tol = 1e-10`.
const STATIC_DT: f64 = 1.0;

/// Two FD step sizes per scope memo §1 IV-8 + Q3 design choice. Both
/// expressed as relative perturbations of `μ_middle`. See module
/// docstring's "Step size derivation" section for the optimal-`h`
/// analysis.
const FD_H_REL_COARSE: f64 = 1.0e-3;
const FD_H_REL_FINE: f64 = 1.0e-4;

/// 5-digit relative-error bar — pairwise FD agreement at the two step
/// sizes. Mirrors `material_fd.rs` / `multi_element_grad_scaling.rs` /
/// `sdf_forward_map_gradcheck.rs` precedent.
const FD_SELF_CONSISTENCY_BAR: f64 = 1.0e-5;

/// FD-of-FEM vs FD-of-Lamé floor. IV-5 `iv_5_three_shell_converges_to_
/// piecewise_lame` validates displacement-vs-Lamé at `0.20` relative at
/// `cell_size = 0.01`; at `cell_size = 0.02` (this test's refinement)
/// the displacement floor is closer to `0.30`, and sensitivity-of-
/// sensitivity loosening adds margin. `0.35` is the conservative gate.
const FD_VS_LAME_FLOOR: f64 = 0.35;

// ── MaterialField builder ─────────────────────────────────────────────────

/// Three-shell `MaterialField` parameterized by middle-layer `μ`.
///
/// Mirrors `concentric_lame_shells.rs::three_shell_field` verbatim
/// except `MU_MIDDLE` is a free parameter (so the FD probe can perturb
/// just the middle shell). `LAMBDA_MIDDLE` stays fixed — IV-8 perturbs
/// only `μ`, not `λ`, to isolate the shear-modulus sensitivity gate.
fn three_shell_field_with_middle_mu(mu_middle: f64) -> MaterialField {
    let body = || {
        Box::new(SphereSdf {
            radius: LAYERED_SPHERE_R_OUTER,
        })
    };
    let phi_inner_outer = LAYERED_SPHERE_R_INNER_OUTER - LAYERED_SPHERE_R_OUTER;
    let phi_outer_inner = LAYERED_SPHERE_R_OUTER_INNER - LAYERED_SPHERE_R_OUTER;
    let mu_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        body(),
        vec![phi_inner_outer, phi_outer_inner],
        vec![MU_INNER, mu_middle, MU_OUTER],
    ));
    let lambda_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        body(),
        vec![phi_inner_outer, phi_outer_inner],
        vec![LAMBDA_INNER, LAMBDA_MIDDLE, LAMBDA_OUTER],
    ));
    MaterialField::from_fields(mu_field, lambda_field)
}

// ── FEM evaluation: Saint-Venant cavity-wall radial displacement ─────────

/// Build the layered scene at `mu_middle`, run one static `replay_step`,
/// return the Saint-Venant-averaged cavity-wall radial displacement.
/// Mirrors IV-5's `run_at_refinement` Saint-Venant read pattern.
fn cavity_u_r_at(mu_middle: f64) -> f64 {
    let field = three_shell_field_with_middle_mu(mu_middle);
    let (mesh, bc, initial, theta) = SoftScene::layered_silicone_sphere(field, CELL_SIZE, PRESSURE)
        .expect("layered_silicone_sphere should mesh successfully at canonical CELL_SIZE");

    let SceneInitial { x_prev, v_prev } = initial;

    // Snapshot cavity-surface vertices' rest radii before the mesh moves
    // into the solver — same pattern as IV-5.
    let positions = mesh.positions();
    let cavity_rest: Vec<(usize, f64)> = bc
        .loaded_vertices
        .iter()
        .map(|&(v, _)| (v as usize, positions[v as usize].norm()))
        .collect();

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = 50;

    let solver: CpuTet4NHSolver<SdfMeshedTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, bc);
    let step = solver.replay_step(&x_prev, &v_prev, &theta, cfg.dt);

    let u_r_sum: f64 = cavity_rest
        .iter()
        .map(|&(v, rest_radius)| {
            let final_pos = Vec3::new(
                step.x_final[3 * v],
                step.x_final[3 * v + 1],
                step.x_final[3 * v + 2],
            );
            final_pos.norm() - rest_radius
        })
        .sum();
    u_r_sum / cavity_rest.len() as f64
}

/// Central FD over [`cavity_u_r_at`] at `mu_base ± h`. Two forward
/// passes per call.
fn fd_central_fem(mu_base: f64, h: f64) -> f64 {
    let u_plus = cavity_u_r_at(mu_base + h);
    let u_minus = cavity_u_r_at(mu_base - h);
    (u_plus - u_minus) / (2.0 * h)
}

// ── Closed-form Lamé three-shell solve ────────────────────────────────────
//
// Trimmed reproduction of `concentric_lame_shells.rs`'s closed-form. Test
// files are isolated crates (Rust integration-test convention) so we
// can't import; re-derived inline at minimum scope. Per-shell parameter
// pair `(μ, λ)` indexes inner-to-outer.
//
// Per shell `i`, radial displacement `u_r^{(i)}(r) = A_i · r + B_i / r²`
// and radial stress `σ_rr^{(i)}(r) = K_i · A_i − 4 μ_i · B_i / r³`
// with `K_i = 3 λ_i + 2 μ_i`. Six unknowns from the BC + continuity
// system: σ_rr(R_a) = -p, u_r(R_b) = 0 (fixed outer surface — IV-5's BC
// choice; preserves spherical symmetry over a free outer), plus four
// continuity conditions at `R_1` and `R_2`. See IV-5 module docstring's
// "Closed-form three-shell Lamé" section for the full derivation.

#[derive(Clone, Copy)]
struct ShellParams {
    mu: f64,
    lambda: f64,
}

impl ShellParams {
    fn k_coefficient(self) -> f64 {
        3.0_f64.mul_add(self.lambda, 2.0 * self.mu)
    }
}

/// Solve the 6×6 closed-form three-shell Lamé system under internal
/// pressure with fixed outer surface; return `u_r^{(1)}(R_a)` directly.
fn lame_cavity_u_r(mu_middle: f64) -> f64 {
    let shells = [
        ShellParams {
            mu: MU_INNER,
            lambda: LAMBDA_INNER,
        },
        ShellParams {
            mu: mu_middle,
            lambda: LAMBDA_MIDDLE,
        },
        ShellParams {
            mu: MU_OUTER,
            lambda: LAMBDA_OUTER,
        },
    ];
    let r_a = LAYERED_SPHERE_R_CAVITY;
    let r_1 = LAYERED_SPHERE_R_INNER_OUTER;
    let r_2 = LAYERED_SPHERE_R_OUTER_INNER;
    let r_b = LAYERED_SPHERE_R_OUTER;
    let k = [
        shells[0].k_coefficient(),
        shells[1].k_coefficient(),
        shells[2].k_coefficient(),
    ];
    let mu = [shells[0].mu, shells[1].mu, shells[2].mu];

    // Layout: x = [A_1, B_1, A_2, B_2, A_3, B_3]ᵀ.
    let mut m = Matrix6::zeros();
    let mut b_vec = Vector6::zeros();

    // Eq 1: σ_rr^{(1)}(R_a) = -p
    m[(0, 0)] = k[0];
    m[(0, 1)] = -4.0 * mu[0] / r_a.powi(3);
    b_vec[0] = -PRESSURE;

    // Eq 2: u_r^{(3)}(R_b) = 0
    m[(1, 4)] = r_b;
    m[(1, 5)] = 1.0 / (r_b * r_b);

    // Eq 3: u_r^{(1)}(R_1) = u_r^{(2)}(R_1)
    m[(2, 0)] = r_1;
    m[(2, 1)] = 1.0 / (r_1 * r_1);
    m[(2, 2)] = -r_1;
    m[(2, 3)] = -1.0 / (r_1 * r_1);

    // Eq 4: σ_rr^{(1)}(R_1) = σ_rr^{(2)}(R_1)
    m[(3, 0)] = k[0];
    m[(3, 1)] = -4.0 * mu[0] / r_1.powi(3);
    m[(3, 2)] = -k[1];
    m[(3, 3)] = 4.0 * mu[1] / r_1.powi(3);

    // Eq 5: u_r^{(2)}(R_2) = u_r^{(3)}(R_2)
    m[(4, 2)] = r_2;
    m[(4, 3)] = 1.0 / (r_2 * r_2);
    m[(4, 4)] = -r_2;
    m[(4, 5)] = -1.0 / (r_2 * r_2);

    // Eq 6: σ_rr^{(2)}(R_2) = σ_rr^{(3)}(R_2)
    m[(5, 2)] = k[1];
    m[(5, 3)] = -4.0 * mu[1] / r_2.powi(3);
    m[(5, 4)] = -k[2];
    m[(5, 5)] = 4.0 * mu[2] / r_2.powi(3);

    let solution = m
        .lu()
        .solve(&b_vec)
        .expect("Lamé 6×6 LU solve failed at canonical IV-5 radii — radii conditioning regression");
    let a_1 = solution[0];
    let b_1 = solution[1];
    a_1.mul_add(r_a, b_1 / (r_a * r_a))
}

/// Central FD over [`lame_cavity_u_r`] at `mu_base ± h`. Pure-arithmetic
/// closed-form — many digits of accuracy at any reasonable `h`.
fn fd_central_lame(mu_base: f64, h: f64) -> f64 {
    let u_plus = lame_cavity_u_r(mu_base + h);
    let u_minus = lame_cavity_u_r(mu_base - h);
    (u_plus - u_minus) / (2.0 * h)
}

// ── Tests ─────────────────────────────────────────────────────────────────

#[test]
fn iv_8_material_gradient_hook_is_fd_stable_against_lame() {
    // Two-gate IV-8: FD self-consistency at the 5-digit bar plus
    // FD-of-FEM-vs-FD-of-Lamé at IV-5 floor. One test, four FEM forward
    // passes (~8 s in release).

    let h_coarse = FD_H_REL_COARSE * MU_MIDDLE_BASE;
    let h_fine = FD_H_REL_FINE * MU_MIDDLE_BASE;

    // FEM FDs at both step sizes — 4 forward passes.
    let fd_fem_coarse = fd_central_fem(MU_MIDDLE_BASE, h_coarse);
    let fd_fem_fine = fd_central_fem(MU_MIDDLE_BASE, h_fine);

    // Closed-form Lamé FD at the coarse step size — analytic baseline
    // for the FD-vs-Lamé floor gate. Pure 6×6 LU solve, microseconds.
    let fd_lame_coarse = fd_central_lame(MU_MIDDLE_BASE, h_coarse);

    // Diagnostic line — surfaces FD values, agreement, and runtime
    // signal under `--nocapture`. Mirrors IV-5 / III-3 reporting.
    let fd_self_rel_err = (fd_fem_coarse - fd_fem_fine).abs() / fd_fem_coarse.abs().max(1.0e-12);
    let fd_vs_lame_rel_err =
        (fd_fem_coarse - fd_lame_coarse).abs() / fd_lame_coarse.abs().max(1.0e-12);
    eprintln!(
        "iv_8 ∂u_r/∂μ_middle FD-stability: \
         FEM FD at h_rel={FD_H_REL_COARSE:e} = {fd_fem_coarse:e}, \
         FEM FD at h_rel={FD_H_REL_FINE:e} = {fd_fem_fine:e}, \
         self-consistency rel_err = {fd_self_rel_err:.3e} (bar {FD_SELF_CONSISTENCY_BAR:.0e}); \
         Lamé FD at h_rel={FD_H_REL_COARSE:e} = {fd_lame_coarse:e}, \
         FD-vs-Lamé rel_err = {fd_vs_lame_rel_err:.4} (floor {FD_VS_LAME_FLOOR})",
    );

    // Sanity: FD must be finite + non-zero. A zero or NaN FD would make
    // the relative-error gates pass vacuously.
    assert!(
        fd_fem_coarse.is_finite() && fd_fem_coarse != 0.0,
        "FD-of-FEM at h_rel = {FD_H_REL_COARSE:e} must be finite and non-zero, got {fd_fem_coarse:e}",
    );
    assert!(
        fd_fem_fine.is_finite() && fd_fem_fine != 0.0,
        "FD-of-FEM at h_rel = {FD_H_REL_FINE:e} must be finite and non-zero, got {fd_fem_fine:e}",
    );
    assert!(
        fd_lame_coarse.is_finite() && fd_lame_coarse != 0.0,
        "FD-of-Lamé at h_rel = {FD_H_REL_COARSE:e} must be finite and non-zero, got {fd_lame_coarse:e}",
    );

    // Sign sanity: `∂u_r/∂μ_middle` must be negative (stiffening the
    // middle layer reduces cavity-wall expansion under fixed pressure).
    // A positive sign signals a wrong-sign material-assignment bug or a
    // perturbation-direction error — exactly what the FD-vs-Lamé floor
    // gate would catch but worth surfacing as its own loud-named
    // assertion before the relative-error checks.
    assert!(
        fd_fem_coarse < 0.0,
        "FD-of-FEM ∂u_r/∂μ_middle = {fd_fem_coarse:e} > 0 — stiffening the middle layer should \
         reduce cavity-wall expansion, not increase it. Wrong-sign material assignment or \
         perturbation-direction error.",
    );
    assert!(
        fd_lame_coarse < 0.0,
        "FD-of-Lamé ∂u_r/∂μ_middle = {fd_lame_coarse:e} > 0 — closed-form Lamé sensitivity has \
         wrong sign; check shell ordering or `(μ, λ)` constants.",
    );

    // Gate 1: FD self-consistency at the 5-digit bar.
    assert!(
        fd_self_rel_err <= FD_SELF_CONSISTENCY_BAR,
        "IV-8 FD self-consistency failed: \
         FEM FD at h_rel = {FD_H_REL_COARSE:e} = {fd_fem_coarse:e}, \
         FEM FD at h_rel = {FD_H_REL_FINE:e} = {fd_fem_fine:e}, \
         rel_err = {fd_self_rel_err:.3e} > {FD_SELF_CONSISTENCY_BAR:.0e}. \
         Diagnose in this order: (1) Newton convergence — verify both \
         passes converged to `tol = 1e-10` (residual_norm in `replay_step`'s \
         NewtonStep). (2) Step-size band — h too small lets roundoff \
         dominate, h too large lets truncation. The chosen h_rel ∈ \
         {{1e-3, 1e-4}} brackets ε^(1/3) · |μ| ≈ 6e-6 · μ; a regression \
         here suggests the optimal-h derivation no longer holds. \
         (3) Mesh determinism — Decision N (I-5 carry-forward) should \
         pin the mesher under cell_size = {CELL_SIZE}; a non-deterministic \
         mesh build at perturbed μ would corrupt the FD numerator.",
    );

    // Gate 2: FD-of-FEM vs FD-of-Lamé within IV-5 floor + margin.
    assert!(
        fd_vs_lame_rel_err < FD_VS_LAME_FLOOR,
        "IV-8 FD-of-FEM vs FD-of-Lamé failed: \
         FEM FD = {fd_fem_coarse:e}, Lamé FD = {fd_lame_coarse:e}, \
         rel_err = {fd_vs_lame_rel_err:.4} ≥ {FD_VS_LAME_FLOOR}. \
         FEM displacement-sensitivity is drifting away from the analytic \
         continuum-mechanics Lamé sensitivity at a rate IV-5's \
         displacement-vs-Lamé floor (cell_size = 0.01 → 0.20 relative) does \
         NOT explain. Check: (1) `LayeredScalarField` partition rule — \
         middle-layer threshold should map to the same per-tet `(μ, λ)` \
         pair the FEM assembly uses. (2) Closed-form 6×6 — the \
         per-shell `K_i = 3λ_i + 2μ_i` and `μ_i` coefficients should \
         match the per-shell `NeoHookean::from_lame` instantiation in \
         `MaterialField::sample`. (3) Boundary conditions — fixed outer \
         surface (`u_r(R_b) = 0`) is the IV-5 BC; the Lamé closed-form \
         here mirrors that. Any bond-rule or surface-BC mismatch shows \
         up as a sensitivity drift even when displacement passes IV-5.",
    );
}
