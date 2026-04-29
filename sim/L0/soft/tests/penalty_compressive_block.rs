//! V-3a — penalty contact compressive block: force-pumping correctness
//! + bounded-by-pure-BC-limits + Cauchy convergence.
//!
//! Phase 5 scope memo §1 V-3a + §8 commit 8 + Decision D (V-3a as
//! simpler-geometry warmup before V-3 Hertz, isolating contact-force
//! pumping from contact-area-radius scaling). The first contact-active
//! scientific gate of the phase: `PenaltyRigidContact`'s force-pumping
//! validated at the integrated level by (i) sign convention exercised
//! end-to-end, (ii) FEM-integrated reaction force bounded by the two
//! pure-BC closed-form limits (uniaxial-stress lower / uniaxial-strain
//! upper) at every refinement level, (iii) Cauchy-style geometric
//! convergence across three refinement levels (`n_per_edge ∈ {2, 4, 8}`
//! → 48 / 384 / 3072 tets).
//!
//! ## R-5 lens (v) — first integrated sign-convention check
//!
//! V-2 commit 4's unit + FD tests pin sign convention at the per-pair
//! gradient/Hessian level. V-3a is the first end-to-end "elastic +
//! penalty equilibrium force balance" exercise: the elastic potential's
//! gradient is `+f_int` (scattered into `f_int` as `+∂Ψ/∂x` at commit
//! 5); the penalty potential's gradient at active config is
//! `-κ·(d̂-d)·n` where `n` is the outward primitive normal. For a
//! top-face vertex penetrating a plane with `n = -ẑ`, gradient.z is
//! `+κ·(d̂-d)` (positive), scattered as `+f_int.z`. The Newton step
//! `δx = -K⁻¹·r` drives `x_curr.z` DOWN, compressing the cube. An
//! inverted sign would either fail to converge (cube blown apart) or
//! converge to a config with the cube floating above the plane. V-3a's
//! `F_R_FEM > 0` per-level assert + `F_R_FEM ∈ [F_us, F_strain]`
//! two-bound assert catch sign flips at the integrated level.
//!
//! ## Two deviations from scope memo §1 V-3a + §9
//!
//! ### Deviation 1: gate replaced from "<5% closed-form match" to "two-bound + Cauchy convergence"
//!
//! Scope memo §1 V-3a names *"F = E · A · ε (uniaxial small-strain,
//! ε = δ/h, valid because lateral expansion is unconstrained →
//! uniaxial-stress regime)"* with a `< 5 %` finest-level relative
//! error gate. This presupposes the BC enforces pure uniaxial-stress
//! (z-only pin on the bottom face, x/y free). The commit-6 helper
//! [`SoftScene::compressive_block_on_plane`] **full-pins the entire
//! bottom face** ([`scene.rs:319-341`] + commit-6 lesson (d) on
//! [`BoundaryConditions`]'s lack of per-DOF pin granularity), giving
//! a **mixed BC**: bottom full-pinned (constrained-modulus regime
//! locally), sides free (uniaxial-stress regime), top z-contacted.
//! The deformation field is non-uniform; **no clean closed-form
//! exists** for this BC at general aspect ratio.
//!
//! V-3a therefore uses **two pure-BC bounds** to bracket the FEM
//! response, plus **Cauchy-style geometric convergence** to confirm
//! the FEM is converging to a stable answer:
//!
//! - **Lower bound** — uniaxial-stress small-strain `F_us = E · A · ε`
//!   (everywhere lateral free; achievable only with z-only-pin BC).
//!   The mixed BC must give `F ≥ F_us` because adding lateral
//!   constraints (the bottom full-pin) makes the system stiffer, not
//!   softer.
//! - **Upper bound** — uniaxial-strain small-strain `F_strain = M_c · A · ε`
//!   where `M_c = E · (1 - ν) / ((1 + ν)(1 - 2 ν))` (everywhere
//!   lateral pin; achievable only with full-pin throughout). The
//!   mixed BC must give `F ≤ F_strain` because removing lateral
//!   constraints (sides free, top free) makes the system softer.
//! - **Cauchy convergence** — `|F_R_n4 - F_R_n8| < |F_R_n2 - F_R_n4|`
//!   demonstrates the FEM sequence is geometrically converging
//!   (ratio < 1) to a stable asymptote. Combined with the bounds,
//!   confirms the asymptote sits inside the physically valid range.
//!
//! With `μ = 1e5`, `λ = 4e5` ⇒ `ν = 0.4`, `E = 2 μ (1 + ν) = 2.8e5
//! Pa`, `M_c = E · (0.6 / (1.4 · 0.2)) = 6.0e5 Pa` (note `M_c / E ≈
//! 2.14`); the bounds at `ε ≈ 0.0058` give `F_us ≈ 0.16 N` and
//! `F_strain ≈ 0.35 N`. The FEM at this scene's aspect ratio (cube)
//! sits close to `F_us` — the bottom-pin's lateral constraint is
//! geometrically confined to a thin boundary layer, with most of the
//! interior in uniaxial-stress regime.
//!
//! Reference: Sokolnikoff, *Mathematical Theory of Elasticity*, 2nd
//! ed., Ch 4 (Saint-Venant principle for boundary-layer effects);
//! Bonet & Wood, *Nonlinear Continuum Mechanics for Finite Element
//! Analysis*, 2nd ed., Ch 5 (small-strain regime). Phase 4 IV-3
//! ([`bonded_bilayer_beam.rs`]) precedent for three-refinement-level
//! single-test-fn structure with monotonic + iter-budget asserts +
//! diagnostic `eprintln!`.
//!
//! ### Deviation 2: `(d̂, δ)` override per scope memo Decision J's V-3a-may-tune authority
//!
//! At the scope-memo §9 V-3a parameters (`L = 1 cm`, `δ = 0.5 mm`,
//! `κ = 1e4 N/m`, `d̂ = 1 mm`, `ν = 0.4`), the cold-start penalty
//! residual is `~κ · d̂ ≈ 10 N` per top-face vertex. The raw Newton
//! step is `~residual / κ ≈ 1 mm` per vertex — `10 %` of the cube
//! edge, past the tet-inversion threshold. Armijo's residual-norm
//! sufficient-decrease condition does NOT check element invertibility,
//! and at `NeoHookean`'s compressive nonlinearity the line-search
//! accepts trial steps that push elements into the
//! NeoHookean-indefinite-Hessian regime; the next Newton iter's
//! tangent fails Cholesky. Three remediation paths considered:
//!
//! - **(a) Multi-step rollout with inertial damping.** Requires
//!   `dt ≈ 1e-5 s` to make the Tikhonov regulariser `M / dt²`
//!   competitive with `κ_pen`; `~10 000` steps to reach quasi-static.
//!   Infeasible test runtime.
//! - **(b) Decision J adjustment** — global default `κ` reduction.
//!   Out of V-3a authority (V-3 commit 9 has separate authority over
//!   defaults under Hertz geometry); would silently affect V-1 / V-3 /
//!   V-4 / V-5 / V-7.
//! - **(c) V-3a-local `(d̂, δ)` override via
//!   [`PenaltyRigidContact::with_params`].** Scope memo §6 R-5 lens
//!   (i): *"if defaults fall on the wrong side of the ceiling,
//!   surface as a Decision-J adjustment, not a fix-on-the-fly."* For
//!   V-3a, the ceiling sits comfortably above defaults at the V-3
//!   sphere geometry but below at the cube geometry — so V-3a-local
//!   override (NOT a global Decision-J adjustment) is the right
//!   reconciliation.
//!
//! V-3a takes (c). Override `d̂ = 1e-5 m` (100× smaller than default)
//! and `δ = 5e-5 m` (10× smaller than scope memo §9). At the override
//! parameters: cold-start residual `κ · (d̂ + δ) ≈ 0.6 N` per vertex,
//! raw Newton step `~6 × 10⁻⁵ m ≈ 0.6 %` of edge — safely below tet-
//! inversion threshold. Equilibrium strain `ε ≈ 0.6 %` — deep into
//! small-strain regime where the two pure-BC bounds (uniaxial-stress
//! `≈ 0.16 N` lower, uniaxial-strain `≈ 0.36 N` upper) cleanly bracket
//! the FEM response (`≈ 0.18 N` at finest level, sub-second debug-mode
//! runtime).
//!
//! Production scenes (V-1 commit 7 + V-3 commit 9 + V-4 / V-5 / V-7
//! commits 10 / 11) continue to use the default `(κ, d̂)` per
//! scope memo Decision J; V-3a's override is local to this test file
//! only, never propagated upstream.
//!
//! ## Reaction-force extraction
//!
//! [`PenaltyRigidContact`] is moved into the solver at construction;
//! the test cannot call `contact.gradient(...)` post-step. Instead the
//! test reconstructs the per-pair penalty force from the known plane
//! geometry (`normal = -ẑ`, `offset = DISPLACEMENT - EDGE_LEN` —
//! mirrors commit-6 helper's plane construction) + the test-local
//! `KAPPA` / `D_HAT_OVERRIDE` constants. For each top-face vertex at
//! `x_final`:
//!
//! ```text
//! sd(p_z) = -p_z - offset = EDGE_LEN - DISPLACEMENT - p_z
//! force_on_rigid.z = +κ · (D_HAT_OVERRIDE - sd)   if sd < D_HAT_OVERRIDE, else 0
//! F_R_FEM = Σ_v force_on_rigid.z
//! ```
//!
//! Mirrors `penalty.rs`'s `gradient` impl exactly: gradient is
//! `-κ·(d̂-d)·n = +κ·(d̂-d)·ẑ` (since `n = -ẑ`), and the force the
//! soft body exerts on the rigid (Newton's 3rd-law) is `+gradient`.
//!
//! ## Newton config — `STATIC_DT` regime per IV-3
//!
//! [`SolverConfig::skeleton`] defaults `dt = 1e-2` for transient
//! integration; under quasi-static-equilibrium tests the inertial
//! Tikhonov regulariser `M / dt²` dominates and skews the equilibrium.
//! Mirror IV-3's `STATIC_DT = 1.0` + `MAX_NEWTON_ITER = 50` to recover
//! the pure-static root-find regime. Under the V-3a `(d̂, δ)` override,
//! Newton typically takes `3-5` iters per level (cold-start residual
//! `~0.6 N` per top-face vertex; raw step well below tet-inversion
//! threshold); the `MAX_NEWTON_ITER = 50` cap exists to mirror IV-3's
//! per-IV-3 `STATIC_DT` precedent and to reserve headroom against load
//! / material perturbations rather than as a tight working budget.
//!
//! ## Why V-3a lands before V-3 (commit 9)
//!
//! V-3a has uniform (cube) geometry — no contact-area-radius scaling.
//! V-3 Hertz couples sphere-mesh resolution (BCC mesher's piecewise-
//! linear approximation of the curved sphere) with contact-force-
//! pumping correctness. V-3a failure at refinement isolates contact-
//! machinery integration bugs from Hertz-specific sphere-mesh
//! resolution sensitivity. V-3 cannot be diagnosed cleanly until V-3a
//! passes.

#![allow(
    // The helper signature returns a 4-tuple via direct destructure;
    // no `Result` to expect on. `expect_used` left enabled here for
    // future test additions that bring in `Result`-returning
    // helpers. Mirrors `bonded_bilayer_beam.rs` /
    // `concentric_lame_shells.rs` / `contact_passthrough.rs` precedent.
    clippy::expect_used
)]

use sim_ml_chassis::Tensor;
use sim_soft::readout::scene::pick_vertices_by_predicate;
use sim_soft::{
    CpuNewtonSolver, HandBuiltTetMesh, MaterialField, PenaltyRigidContact,
    PenaltyRigidContactSolver, RigidPlane, SceneInitial, SoftScene, Solver, SolverConfig, Tet4,
    Vec3, VertexId,
};

// ── Scene constants ──────────────────────────────────────────────────────

/// Cube edge length (1 cm). Scope memo §9 V-3a recommendation.
const EDGE_LEN: f64 = 0.01;

/// Rigid-plane axial displacement (0.05 mm). 10× smaller than scope
/// memo §9 V-3a's recommended 0.5 mm — see module docstring "Deviation
/// 2" section on the `(d̂, δ)` override under scope memo Decision J's
/// V-3a-may-tune authority.
const DISPLACEMENT: f64 = 5.0e-5;

/// Lamé pair `(μ, λ)` — Phase 4 IV-3 / IV-5 default Ecoflex-class
/// compressible `NeoHookean` (`λ = 4 μ` ⇒ `ν = 0.4`). The canonical
/// pair pins V-3a to the rest of the regression net.
const MU: f64 = 1.0e5;
const LAMBDA: f64 = 4.0e5;

/// Young's modulus from Lamé pair: `E = μ (3 λ + 2 μ) / (λ + μ)`.
/// Mirror of [`bonded_bilayer_beam`]'s same-named helper. At canonical
/// `(μ, λ) = (1e5, 4e5)`: `E = 2.8e5 Pa` (which also equals
/// `2 μ (1 + ν) = 2 · 1e5 · 1.4`).
const fn young_modulus(mu: f64, lambda: f64) -> f64 {
    // const fn doesn't allow `mul_add`; expand explicitly. The
    // formula is exact for compressible isotropic linear elastic.
    mu * (3.0 * lambda + 2.0 * mu) / (lambda + mu)
}

/// Constrained modulus (1D dilatational stiffness):
/// `M_c = E · (1 - ν) / ((1 + ν) · (1 - 2 ν)) = λ + 2 μ`. The second
/// equality is exact for isotropic linear elastic — so this is just
/// `LAMBDA + 2 MU = 6.0e5 Pa` at the canonical pair.
const fn constrained_modulus(mu: f64, lambda: f64) -> f64 {
    lambda + 2.0 * mu
}

/// V-3a-local penalty stiffness. Pinned at the
/// `sim_soft::contact::penalty::PENALTY_KAPPA_DEFAULT` value
/// (penalty.rs:57) per scope memo Decision J — V-3a's override is
/// scoped to `d̂` (and `δ`); `κ` stays at default. The `pub(crate)`
/// visibility on the upstream constant forces re-pinning here.
const KAPPA: f64 = 1.0e4;

/// V-3a-local penalty contact band. **Override of**
/// `PENALTY_DHAT_DEFAULT = 1e-3` (penalty.rs:65) per scope memo
/// Decision J's V-3a-may-tune authority — see module docstring
/// "Deviation 2" section. 100× smaller than default to bring
/// cold-start penalty residual `κ · (d̂ + δ) ≈ 0.6 N` per top-face
/// vertex below the tet-inversion threshold. Production scenes (V-1 /
/// V-3 / V-4 / V-5 / V-7) continue to use the default; this constant
/// only enters via [`PenaltyRigidContact::with_params`] in
/// [`run_at_refinement`] and must NOT be propagated upstream.
const D_HAT_OVERRIDE: f64 = 1.0e-5;

/// Static-equilibrium time-step — large `dt` damps the inertial
/// Tikhonov regulariser `M / dt²` to negligible relative magnitude,
/// yielding pure-static root-find. Mirrors IV-3's `STATIC_DT`.
const STATIC_DT: f64 = 1.0;

/// Newton iteration cap — bumped from skeleton's `10` to mirror IV-3's
/// `50` (static-equilibrium from rest needs more headroom than
/// transient-step's small `Δx`). Newton typically takes `3-5` iters
/// per level under the V-3a `(d̂, δ)` override; cap leaves wide margin
/// against load / material perturbations.
const MAX_NEWTON_ITER: usize = 50;

// ── Helpers ──────────────────────────────────────────────────────────────

fn material_field() -> MaterialField {
    MaterialField::uniform(MU, LAMBDA)
}

/// Per-refinement run output — `λ_z` (axial stretch ratio at top face),
/// FEM-integrated reaction force, Newton diagnostics.
struct StepReport {
    /// Mean axial stretch ratio: `(z_eq_avg over top face) / EDGE_LEN`.
    /// `λ_z < 1` for compression.
    lambda_z_avg: f64,
    /// FEM-integrated reaction force on the rigid plane (Newton's
    /// 3rd-law partner of the penalty force on the top face). Positive
    /// when the soft body pushes UP on the rigid plane.
    f_r_fem: f64,
    /// Active-pair count at converged `x_final` — sanity check that
    /// every top-face vertex sits inside the contact band.
    n_active_pairs: usize,
    /// Newton iteration count at convergence.
    iter_count: usize,
    /// Free-DOF residual norm at convergence.
    residual_norm: f64,
}

/// Uniaxial-stress small-strain reaction force at compressive strain
/// `epsilon` (positive in compression). `F_us = E · A · ε`. Lower
/// bound for the mixed-BC FEM response — see module docstring
/// "Deviation 1" section.
fn uniaxial_stress_reaction(epsilon: f64) -> f64 {
    young_modulus(MU, LAMBDA) * EDGE_LEN * EDGE_LEN * epsilon
}

/// Uniaxial-strain small-strain reaction force at compressive strain
/// `epsilon`. `F_strain = M_c · A · ε`. Upper bound for the mixed-BC
/// FEM response — see module docstring "Deviation 1" section.
fn uniaxial_strain_reaction(epsilon: f64) -> f64 {
    constrained_modulus(MU, LAMBDA) * EDGE_LEN * EDGE_LEN * epsilon
}

/// Single backward-Euler quasi-static step on the
/// [`SoftScene::compressive_block_on_plane`] scene at refinement
/// `n_per_edge`. Returns the [`StepReport`] with FEM-integrated
/// reaction force + equilibrium stretch ratio + Newton diagnostics.
fn run_at_refinement(n_per_edge: usize) -> StepReport {
    // n_per_edge ∈ {2, 4, 8}; cell_size divides EDGE_LEN to integer
    // by construction. All three levels pass the helper's
    // integer-divisibility assert.
    //
    // Cast safe: n_per_edge is a small-integer literal at every
    // call site; conversion to f64 is loss-free.
    #[allow(clippy::cast_precision_loss)]
    let cell_size = EDGE_LEN / (n_per_edge as f64);
    // Helper builds mesh + BC + initial via commit-6 scaffolding;
    // its returned `default_contact` (default `κ`, `d̂`) is discarded
    // and replaced with a `with_params` override per the module
    // docstring's "Deviation 2" section. The plane is reconstructed
    // identically to the helper's `RigidPlane::new(Vec3::new(0.0,
    // 0.0, -1.0), DISPLACEMENT - EDGE_LEN)` (see `scene.rs:450`).
    let (mesh, bc, initial, _default_contact) =
        SoftScene::compressive_block_on_plane(EDGE_LEN, cell_size, DISPLACEMENT, &material_field());
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), DISPLACEMENT - EDGE_LEN);
    let contact = PenaltyRigidContact::with_params(vec![plane], KAPPA, D_HAT_OVERRIDE);

    // Capture top-face vertex IDs (rest config `z = EDGE_LEN`,
    // half-cell tolerance) BEFORE moving the mesh into the solver. The
    // post-step λ_z + F_R extraction reads from these IDs against the
    // solver's returned `x_final`.
    let band_tol = 0.5 * cell_size;
    let top_face_vertices: Vec<VertexId> =
        pick_vertices_by_predicate(&mesh, |p| (p.z - EDGE_LEN).abs() < band_tol);
    assert!(
        !top_face_vertices.is_empty(),
        "top face must contain at least one vertex at refinement n = {n_per_edge}",
    );

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = MAX_NEWTON_ITER;

    let SceneInitial { x_prev, v_prev } = initial;

    let solver: PenaltyRigidContactSolver<HandBuiltTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc);

    // No external traction (helper's `loaded_vertices` is empty); θ
    // is length-0 to match BC per `assemble_external_force`'s
    // empty-loaded-vs-θ-length assert at `backward_euler.rs:789-795`.
    // The contact load enters via the penalty model; `step` is driven
    // by penalty alone.
    let empty_theta: [f64; 0] = [];
    let theta_tensor = Tensor::from_slice(&empty_theta, &[0]);
    let step = solver.replay_step(&x_prev, &v_prev, &theta_tensor, cfg.dt);

    // λ_z = (mean z_eq over top face) / L. Top face was at z = L at
    // rest; compression drops them to z_eq < L → λ_z < 1.
    let z_sum: f64 = top_face_vertices
        .iter()
        .map(|&v| step.x_final[3 * v as usize + 2])
        .sum();
    // Vertex count > 0 guaranteed by the predicate-empty assert above;
    // top-face counts at n=2/4/8 are 9/25/81, all f64-loss-free.
    #[allow(clippy::cast_precision_loss)]
    let lambda_z_avg = (z_sum / top_face_vertices.len() as f64) / EDGE_LEN;

    // FEM reaction force = sum over active top-face pairs of penalty
    // gradient.z. Manual reconstruction from the known plane geometry
    // (`normal = -ẑ`, `offset = DISPLACEMENT - EDGE_LEN`); see module
    // docstring's "Reaction-force extraction" section. The bottom-face
    // pinned vertices stay at `z = 0`, giving `sd = EDGE_LEN -
    // DISPLACEMENT ≈ 1e-2 m >> D_HAT_OVERRIDE = 1e-5 m`, never active
    // — so iterating only top-face vertices captures every active
    // pair. Walking all vertices would cost O(n) but produce the same
    // sum.
    let mut f_r_fem = 0.0;
    let mut n_active_pairs = 0;
    for &v in &top_face_vertices {
        let z_v = step.x_final[3 * v as usize + 2];
        let sd = EDGE_LEN - DISPLACEMENT - z_v;
        if sd < D_HAT_OVERRIDE {
            f_r_fem += KAPPA * (D_HAT_OVERRIDE - sd);
            n_active_pairs += 1;
        }
    }

    StepReport {
        lambda_z_avg,
        f_r_fem,
        n_active_pairs,
        iter_count: step.iter_count,
        residual_norm: step.final_residual_norm,
    }
}

// ── Tests ────────────────────────────────────────────────────────────────

#[test]
// Mirrors `concentric_lame_shells.rs:678` precedent — three-refinement-
// level analytic-comparison tests legitimately exceed clippy's 100-line
// soft cap once the diagnostic `eprintln!` + per-level assertion loops
// (Newton iters / sign sanity / two-bound) + Cauchy gate are inlined.
// Extracting helpers would add indirection without improving clarity.
#[allow(clippy::too_many_lines)]
fn v_3a_compressive_block_force_pumping_correctness() {
    let report_n2 = run_at_refinement(2);
    let report_n4 = run_at_refinement(4);
    let report_n8 = run_at_refinement(8);

    // Per-level compressive strain `ε = 1 - λ_z`. Used for the
    // two pure-BC bound calculations.
    let eps_n2 = 1.0 - report_n2.lambda_z_avg;
    let eps_n4 = 1.0 - report_n4.lambda_z_avg;
    let eps_n8 = 1.0 - report_n8.lambda_z_avg;
    let f_us_n2 = uniaxial_stress_reaction(eps_n2);
    let f_us_n4 = uniaxial_stress_reaction(eps_n4);
    let f_us_n8 = uniaxial_stress_reaction(eps_n8);
    let f_strain_n2 = uniaxial_strain_reaction(eps_n2);
    let f_strain_n4 = uniaxial_strain_reaction(eps_n4);
    let f_strain_n8 = uniaxial_strain_reaction(eps_n8);

    let cauchy_step_coarse = (report_n2.f_r_fem - report_n4.f_r_fem).abs();
    let cauchy_step_fine = (report_n4.f_r_fem - report_n8.f_r_fem).abs();
    let cauchy_ratio = cauchy_step_fine / cauchy_step_coarse;

    eprintln!(
        "v_3a convergence: \
         n=2 (lambda_z {lz2:.6}, F_R_FEM {fr2:.4} N, F_us {us2:.4} N, F_strain {st2:.4} N, \
         active {ap2}, iters {it2}, res {res2:e}); \
         n=4 (lambda_z {lz4:.6}, F_R_FEM {fr4:.4} N, F_us {us4:.4} N, F_strain {st4:.4} N, \
         active {ap4}, iters {it4}, res {res4:e}); \
         n=8 (lambda_z {lz8:.6}, F_R_FEM {fr8:.4} N, F_us {us8:.4} N, F_strain {st8:.4} N, \
         active {ap8}, iters {it8}, res {res8:e}); \
         Cauchy: |Δ_coarse| {cs:.4e} N, |Δ_fine| {cf:.4e} N, ratio {cr:.4}",
        lz2 = report_n2.lambda_z_avg,
        fr2 = report_n2.f_r_fem,
        us2 = f_us_n2,
        st2 = f_strain_n2,
        ap2 = report_n2.n_active_pairs,
        it2 = report_n2.iter_count,
        res2 = report_n2.residual_norm,
        lz4 = report_n4.lambda_z_avg,
        fr4 = report_n4.f_r_fem,
        us4 = f_us_n4,
        st4 = f_strain_n4,
        ap4 = report_n4.n_active_pairs,
        it4 = report_n4.iter_count,
        res4 = report_n4.residual_norm,
        lz8 = report_n8.lambda_z_avg,
        fr8 = report_n8.f_r_fem,
        us8 = f_us_n8,
        st8 = f_strain_n8,
        ap8 = report_n8.n_active_pairs,
        it8 = report_n8.iter_count,
        res8 = report_n8.residual_norm,
        cs = cauchy_step_coarse,
        cf = cauchy_step_fine,
        cr = cauchy_ratio,
    );

    // ── Per-level Newton + sign + active-pair sanity ────────────────────
    //
    // Newton-budget per level — mirrors IV-3 pattern. Under the V-3a
    // `(d̂, δ)` override, Newton typically completes in `3-5` iters
    // per level (cold-start residual `~0.6 N` per top-face vertex;
    // raw Newton step well below tet-inversion threshold). At `< 40`
    // we have `35+` iters of margin against future load / material
    // perturbations.
    for (n_per_edge, report) in [(2, &report_n2), (4, &report_n4), (8, &report_n8)] {
        assert!(
            report.iter_count < 40,
            "Newton at n={n_per_edge} ran {iters} iters, within 10 of the {cap}-iter cap — \
             investigate solver / penalty regime regression before bumping the cap",
            iters = report.iter_count,
            cap = MAX_NEWTON_ITER,
        );
    }

    // Sign sanity per level: `λ_z < 1` (cube compresses, not extends);
    // `λ_z > 0.5` rules out >50% compression as physically implausible
    // at the V-3a `(κ, d̂, δ)` regime; `F_R > 0` (soft body pushes UP
    // on rigid plane — Newton's 3rd-law partner of penalty's DOWN
    // force on top face). Catches sign-flip regressions at gross-
    // physics level before the bound asserts surface them numerically.
    for (n_per_edge, report) in [(2, &report_n2), (4, &report_n4), (8, &report_n8)] {
        assert!(
            report.lambda_z_avg < 1.0 && report.lambda_z_avg > 0.5,
            "λ_z at n={n_per_edge} = {lz} out of plausible compression range (0.5, 1.0) — \
             sign-flip or contact-machinery regression",
            lz = report.lambda_z_avg,
        );
        assert!(
            report.f_r_fem > 0.0,
            "F_R at n={n_per_edge} = {fr} should be positive (soft body pushes UP on rigid \
             plane); negative reaction is a sign-convention regression",
            fr = report.f_r_fem,
        );
        // Top-face vertex count: `(n_per_edge + 1)²` for the
        // `HandBuiltTetMesh::uniform_block` `(n+1)³` vertex grid. All
        // top-face vertices should sit inside the contact band at
        // equilibrium (cube compresses by `~6e-5 m` from `δ = 5e-5 m`
        // plane displacement plus `d̂_override = 1e-5 m` band
        // engagement, leaving every top vertex with a small positive
        // `sd` strictly less than `D_HAT_OVERRIDE`).
        let expected_top = (n_per_edge + 1) * (n_per_edge + 1);
        assert_eq!(
            report.n_active_pairs,
            expected_top,
            "active-pair count at n={n_per_edge} = {got} should match top-face vertex count \
             {expected_top} (every top-face vertex inside the d̂-band at equilibrium)",
            got = report.n_active_pairs,
        );
    }

    // ── Two-bound physical-constraint per level ─────────────────────────
    //
    // The mixed BC (bottom full-pin + top z-pressed + sides free) MUST
    // give a response between the two pure-BC limits at the same `ε`:
    //   F_us(ε) ≤ F_R_FEM(ε) ≤ F_strain(ε)
    // See module docstring "Deviation 1" section for the derivation
    // (adding lateral constraint can only stiffen; removing only
    // soften).
    for (n_per_edge, report, f_us, f_strain) in [
        (2, &report_n2, f_us_n2, f_strain_n2),
        (4, &report_n4, f_us_n4, f_strain_n4),
        (8, &report_n8, f_us_n8, f_strain_n8),
    ] {
        assert!(
            report.f_r_fem >= f_us,
            "F_R_FEM at n={n_per_edge} = {fr:.4} N below uniaxial-stress lower bound {us:.4} N \
             (ε = {eps:.4e}); mixed BC should be at least as stiff as pure uniaxial-stress",
            fr = report.f_r_fem,
            us = f_us,
            eps = 1.0 - report.lambda_z_avg,
        );
        assert!(
            report.f_r_fem <= f_strain,
            "F_R_FEM at n={n_per_edge} = {fr:.4} N above uniaxial-strain upper bound \
             {st:.4} N (ε = {eps:.4e}); mixed BC should be at most as stiff as pure \
             uniaxial-strain",
            fr = report.f_r_fem,
            st = f_strain,
            eps = 1.0 - report.lambda_z_avg,
        );
    }

    // ── Cauchy-style geometric convergence ──────────────────────────────
    //
    // `|F_R_n4 - F_R_n8| < |F_R_n2 - F_R_n4|` demonstrates the FEM
    // sequence is geometrically converging (ratio < 1) toward a
    // stable asymptote. Combined with the two-bound asserts above,
    // confirms the asymptote sits inside the physically valid range.
    // Stronger gate than monotonic-only — catches "F_R wobbles
    // bounded" regressions where the sequence stays bounded but
    // doesn't actually converge.
    assert!(
        cauchy_ratio < 1.0,
        "Cauchy ratio {cauchy_ratio:.4} (|Δ_fine| / |Δ_coarse|) ≥ 1.0 — FEM sequence is \
         not geometrically converging across n=2/4/8. \
         |F_R_n2 - F_R_n4| = {cauchy_step_coarse:.4e}, \
         |F_R_n4 - F_R_n8| = {cauchy_step_fine:.4e}",
    );
}
