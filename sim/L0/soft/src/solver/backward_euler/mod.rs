//! `CpuNewtonSolver` — backward-Euler Newton with Armijo line-search.
//!
//! Residual form per [`Part 6 Ch 02:13–15`][r]:
//! \\[ r(x; \theta) = (M / \Delta t^2)\,(x - x_\text{prev} - \Delta t\,v_\text{prev})
//!                  + f_\text{int}(x) - f_\text{ext}(\theta) \\]
//!
//! Tangent \\(A = \partial r / \partial x = M / \Delta t^2 + K(x)\\) with
//! \\(K = V \cdot B^{\mathsf{T}} \mathbb{C} B\\) assembled per
//! [`Part 3 Ch 00 00-tet4.md`][tet4]. Only the free-DOF block (scope §2
//! Dirichlet: `v_0`, `v_1`, `v_2` pinned) is factored — the free DOFs
//! start at index `FREE_OFFSET = 9` (private module constant).
//!
//! Solve path: faer `SymbolicLlt::try_new` + `SymbolicLu::try_new`
//! once per `step`-call (one symbolic factor per algorithm; both share
//! the same element-vertex sparsity pattern with `Side::Lower` and full
//! reflection respectively), then `Llt::try_new_with_symbolic` +
//! `solve_in_place_with_conj` per Newton iteration. A2 LU fallback
//! engages on `LltError::Numeric(NonPositivePivot)`: the helper
//! `factor_free_tangent` symmetrizes the lower-tri triplets to full
//! and factors via `Lu` against the cached `SymbolicLu`. Happy path
//! stays bit-identical to the pre-A2 Llt-only code (scope §11 S-3
//! Round-1-verified API shape preserved).
//!
//! After convergence, `step` re-factors `A` at `x_final` via
//! `factor_at_position` and pushes `NewtonStepVjp` onto the tape with
//! `theta_var` as parent. The VJP solves the IFT adjoint `A · λ = g_free`
//! and contracts against `∂r/∂θ` — see `NewtonStepVjp` for the math.
//!
//! [r]: ../../../../../../docs/studies/soft_body_architecture/src/60-differentiability/02-implicit-function.md
//! [tet4]: ../../../../../../docs/studies/soft_body_architecture/src/30-discretization/00-element-choice/00-tet4.md

use std::borrow::Cow;
use std::collections::{BTreeMap, BTreeSet};

use faer::linalg::solvers::SolveCore;
use faer::prelude::Reborrow;
use faer::sparse::linalg::LltError;
use faer::sparse::linalg::solvers::{Llt, Lu, SymbolicLlt, SymbolicLu};
use faer::sparse::{SparseColMat, Triplet};
use faer::{Conj, MatMut, Side};
use nalgebra::{DMatrix, DVector, Matrix3, SMatrix};
use sim_ml_chassis::{Tensor, Var};

use super::lm::{LmConfig, LmState};
use super::{CpuTape, NewtonStep, Solver, SolverFailure};
use crate::Vec3;
use crate::contact::{ActivePairsFor, ContactModel, RigidTwist};
use crate::differentiable::newton_vjp::{
    MaterialStepVjp, NewtonStepVjp, StateStepVjp, TrajectoryStepVjp,
};
use crate::element::Element;
use crate::material::{InversionHandling, Material};
use crate::mesh::{Mesh, TetId, VertexId};
use crate::readout::{BoundaryConditions, LoadAxis};

/// Armijo sufficient-decrease constant (scope §5 R-1).
const ARMIJO_C1: f64 = 1e-4;

/// Local stall-info carrier returned by [`CpuNewtonSolver::armijo_backtrack`]
/// when the line search exhausts its backtrack budget (F3.3 per
/// `docs/F3_LM_REGULARIZATION_SPEC.md` §2.5).
///
/// Distinct from the public [`SolverFailure::ArmijoStall`] variant —
/// shape mirrors the spec's local-struct fields exactly. The pre-F3
/// panic message included the per-call `final_alpha`; spec drops it
/// from this struct (and so the panic-message text in
/// `solve_impl`'s translation arm loses the `final α 4.77e-7`
/// substring vs pre-F3). No regression-test pins on that substring;
/// the substantive panic surface (panic-on-stall behavior + Newton
/// `iter` + `r_norm`) is preserved.
#[derive(Debug)]
struct ArmijoStallInfo {
    x_curr: Vec<f64>,
    iter: usize,
    r_norm: f64,
}

/// Local info carrier for the doubly-failed-factor case (Llt non-PD
/// AND Lu also failed) returned by the `try_*` factor methods (F3.3
/// per spec §2.5). The caller (e.g., `try_solve_impl`,
/// `try_factor_at_position`) wraps this into the public
/// [`SolverFailure::DoublyFailedFactor`] variant, filling in the
/// `x_partial` + `last_iter` fields from its own call-site context.
///
/// Distinct from the public variant for the same DRY reason as
/// `ArmijoStallInfo`: factor-site methods don't naturally have
/// `x_partial` (they take triplets, not positions), so plumbing the
/// position through every factor signature would be wrong-shape.
#[derive(Debug)]
struct DoublyFailedFactorInfo {
    /// Full panic-message-style context: factor site + Newton iter +
    /// Lu error details. Preserves the bit-equal panic message at
    /// `solve_impl`'s translation site.
    context: String,
}

/// Per-element reference-frame geometry, pre-computed at solver
/// construction (Phase 2 commit 4a).
///
/// `grad_x_n` is the material-frame shape-function gradient
/// (`SMatrix<f64, 4, 3>`, one row per node, constant across the
/// element for Tet4); `volume` is the rest-configuration tet volume.
/// One per tet in the mesh; cached so the per-iter assembly path
/// doesn't recompute on every Newton step.
#[derive(Clone, Debug)]
struct ElementGeometry {
    grad_x_n: SMatrix<f64, 4, 3>,
    volume: f64,
}

/// Solver configuration — integration parameters the skeleton scene
/// (spec §2) consumes. `skeleton()` returns the spec-§2 defaults.
#[derive(Clone, Copy, Debug)]
pub struct SolverConfig {
    /// Integration time-step (seconds).
    pub dt: f64,
    /// Newton residual tolerance on the free-DOF residual.
    pub tol: f64,
    /// Reference-configuration mass density (`kg/m^3`). The solver
    /// derives per-DOF lumped mass from this — see `reference_geometry`.
    pub density: f64,
    /// Maximum Newton iterations before declaring divergence.
    pub max_newton_iter: usize,
    /// Maximum Armijo backtracks before declaring line-search stall.
    pub max_line_search_backtracks: usize,
    /// Body-force gravitational acceleration along `+ẑ` (`m/s²`).
    /// Pass a negative value for "downward" gravity (e.g. `-9.81`).
    /// Wired alongside `tests/contact_drop_rest.rs` (drop-and-rest
    /// gravity hygiene); scalar (not `Vec3`) form preserves
    /// [`Self::skeleton`]'s `const fn` signature. Default `0.0` keeps
    /// pre-gravity regression nets bit-equal —
    /// `assemble_external_force` short-circuits the body-force scatter
    /// when this is exactly zero.
    pub gravity_z: f64,
    /// Levenberg-Marquardt regularization for non-PD tangent rescue
    /// per `docs/F3_LM_REGULARIZATION_SPEC.md`. `None` (the
    /// [`Self::skeleton`] default) preserves pre-F3 behavior bit-equal
    /// via `LmState::disabled` (pub(super) — see `super::lm`) short-circuit
    /// at `factor_free_tangent`'s retry loop: `Llt` first, then direct
    /// `Lu` fallback on non-PD, no `+λI`. `Some(LmConfig)` activates
    /// the in-iter Marquardt adapter; the `Lu` fallback then becomes
    /// the λ-saturation surface. Fork-B (cf-sim-research) consumers
    /// opt in via [`LmConfig::fork_b`] paired with
    /// [`Solver::try_step`] for graceful
    /// failure on Armijo stall.
    pub lm_regularization: Option<LmConfig>,
    /// Coulomb friction coefficient `μ_c` for the smoothed-Coulomb friction term
    /// (`contact::friction`). Default `0.0` = FRICTIONLESS, which short-circuits the
    /// friction scatter in the forward assembly → bit-equal to the pre-friction path
    /// (the [`Self::skeleton`] / `gravity_z = 0` pattern). Friction enters the FORWARD
    /// Newton solve (residual + its Hessian); the differentiable tangent
    /// (`factor_at_position`) stays friction-free until the differentiability leaf.
    ///
    /// PR1 is FORWARD-ONLY: gradients with `friction_mu > 0` are **not** supported and the
    /// differentiable paths (`step`, the VJP / equilibrium-sensitivity methods) panic rather
    /// than silently return a tangent that omits the friction Hessian. Use `replay_step` for
    /// forward-only friction; PR2 (the differentiability leaf) wires friction into the adjoint.
    pub friction_mu: f64,
    /// Friction velocity threshold `ε_v` (m/s): the transition-zone width in displacement
    /// space is `w = dt·ε_v` (below this sliding speed the smoothed force ramps from zero
    /// — the stick regime). Only consulted when `friction_mu > 0`. IPC default
    /// `≈ 1e-3·L_bbox` m/s.
    pub friction_eps_v: f64,
}

impl SolverConfig {
    /// Scope §2 defaults for the walking-skeleton scene: `dt = 1e-2`,
    /// `tol = 1e-10` (five digits below gradcheck's 1e-5 bar),
    /// `density = 1030` (silicone-class), up to 10 Newton iterations +
    /// 20 backtracks per iteration. `gravity_z = 0` — the
    /// drop-and-rest fixture opts in by mutating the field on a
    /// constructed config (mirrors the Hertzian and compressive-block
    /// fixtures' `STATIC_DT` bumping pattern).
    #[must_use]
    pub const fn skeleton() -> Self {
        Self {
            dt: 1e-2,
            tol: 1e-10,
            density: 1030.0,
            max_newton_iter: 10,
            max_line_search_backtracks: 20,
            gravity_z: 0.0,
            lm_regularization: None,
            friction_mu: 0.0,
            friction_eps_v: 0.0,
        }
    }
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self::skeleton()
    }
}

/// Factor of the free-DOF condensed tangent — either the SPD-happy-path
/// Cholesky factor or the A2 LU fallback that engages when Llt hits a
/// non-PD pivot. Both variants implement the same in-place solve via
/// `solve_in_place_with_conj`, so downstream consumers
/// ([`NewtonStepVjp`](crate::differentiable::newton_vjp::NewtonStepVjp))
/// stay solver-shape-agnostic.
///
/// Happy path: every factor returned is `Llt`. The fallback fires only
/// when the assembled tangent is indefinite at the current Newton iter
/// (or, rarely, at `x_final` for the IFT adjoint), which historically
/// surfaced at row 21 v1.5's capsule-cap apex contact concentration.
enum FactorInner {
    Llt(Llt<usize, f64>),
    /// Boxed because faer's `Lu` (carrying `NumericLu`'s row/col-perm
    /// vecs + factor data) is substantially larger than `Llt`. The Lu
    /// variant fires only on the cold A2 fallback path, so the
    /// per-fallback heap indirection beats enlarging every happy-path
    /// `Llt`-carrying value (per `clippy::large_enum_variant`).
    Lu(Box<Lu<usize, f64>>),
}

impl FactorInner {
    /// Base solve `A_sym · x = rhs` in place via the stored symmetric factor.
    fn solve_in_place_with_conj(&self, conj: Conj, rhs: MatMut<'_, f64>) {
        match self {
            Self::Llt(llt) => llt.solve_in_place_with_conj(conj, rhs),
            Self::Lu(lu) => lu.solve_in_place_with_conj(conj, rhs),
        }
    }
}

/// The asymmetric friction adjoint as a Woodbury low-rank correction around the
/// symmetric factor: the true tangent is `A = A_sym + Σₚ aₚ bₚᵀ`, where `A_sym`
/// (elastic + barrier + frozen-lag `∇²D`) is the factored symmetric part and each
/// active friction pair contributes a rank-1 `aₚ bₚᵀ` (the `∂λⁿ/∂x` normal-force
/// coupling that the frozen lag drops; friction is non-conservative, so this part is
/// non-symmetric). `z = A_sym⁻¹ U` and `M = I + Vᵀ Z` are precomputed once at factor
/// time, so each corrected solve is one symmetric back-solve plus a `k×k` dense solve
/// (`k` = active-pair count). Empty (`k = 0`) ⇒ the correction is a no-op.
struct WoodburyCorrection {
    /// `U` columns (the `aₚ = ∇D/λⁿ` friction-force directions), each length `n_free`.
    /// Kept for the transpose solve `A⁻ᵀ` (the reverse-mode tape); the forward solve
    /// uses only `zu_cols`.
    u_cols: Vec<Vec<f64>>,
    /// `V` columns (the `bₚ = ∂λⁿ/∂xₚ` rows), each length `n_free`.
    v_cols: Vec<Vec<f64>>,
    /// `A_sym⁻¹ U` columns (pre-solved), `n_free` each — the forward `A⁻¹` tail.
    zu_cols: Vec<Vec<f64>>,
    /// `A_sym⁻¹ V` columns (pre-solved), `n_free` each — the transpose `A⁻ᵀ` tail.
    zv_cols: Vec<Vec<f64>>,
    /// `M = I_k + Vᵀ Z` (`k×k`), the dense capacitance matrix. The transpose solve
    /// uses `Mᵀ` (its own LU), since `(I + VᵀA_sym⁻¹U)ᵀ = I + UᵀA_sym⁻¹V`.
    m: DMatrix<f64>,
}

impl WoodburyCorrection {
    /// Apply `rhs ← rhs − Z M⁻¹ (Vᵀ rhs)` in place — the Woodbury tail that turns a
    /// symmetric solve `A_sym⁻¹ rhs` into the full `A⁻¹ rhs`.
    //
    // expect_used: `M = I + VᵀZ` is the Woodbury capacitance matrix of the true tangent
    // `A = A_sym + UVᵀ`. `A` is non-singular at a converged stable equilibrium (the forward
    // Newton solve converged through it), so `M` is invertible — a singular `M` would mean a
    // structurally degenerate friction configuration, where the gradient is itself undefined.
    #[allow(clippy::expect_used)]
    fn apply_in_place(&self, rhs: &mut [f64]) {
        // Forward `A⁻¹`: rhs ← rhs − Z M⁻¹ (Vᵀ rhs), contracting V against rhs and Z back out.
        self.apply_tail(rhs, &self.v_cols, &self.zu_cols, false);
    }

    /// Apply the transpose Woodbury tail `rhs ← rhs − Zᵀ M⁻ᵀ (Uᵀ rhs)` — turns the
    /// symmetric solve `A_sym⁻¹ rhs` into the full `A⁻ᵀ rhs` (the reverse-mode adjoint).
    /// `Aᵀ = A_sym + V Uᵀ`, so the roles of `(U, Z)` and `(V, Zᵀ)` swap and `M → Mᵀ`.
    fn apply_transpose_in_place(&self, rhs: &mut [f64]) {
        self.apply_tail(rhs, &self.u_cols, &self.zv_cols, true);
    }

    /// Shared low-rank tail: `rhs ← rhs − out_cols · (M[ᵀ])⁻¹ · (in_colsᵀ rhs)`.
    //
    // expect_used: `M = I + VᵀZ` is the Woodbury capacitance matrix of the true tangent
    // `A = A_sym + UVᵀ` (the asymmetric friction λ-coupling AND, on a curved collider, the
    // `DN·C` tangent-rotation columns). `A` is non-singular at a converged stable equilibrium,
    // so `M` (and `Mᵀ`) is invertible — a singular `M` would mean a structurally degenerate
    // friction configuration, where the gradient is itself undefined. (The curved-T columns can
    // make `A` indefinite in the sharp/deep regime — the #415 `dE·H` caveat; `lm_regularization`
    // shrinks `A_sym⁻¹` and keeps `M` near `I` there. Fail-loud, never silently wrong.)
    #[allow(clippy::expect_used)]
    fn apply_tail(
        &self,
        rhs: &mut [f64],
        in_cols: &[Vec<f64>],
        out_cols: &[Vec<f64>],
        transpose_m: bool,
    ) {
        let k = out_cols.len();
        if k == 0 {
            return;
        }
        let mut t = DVector::<f64>::zeros(k);
        for (j, c) in in_cols.iter().enumerate() {
            t[j] = c.iter().zip(rhs.iter()).map(|(a, b)| a * b).sum();
        }
        let m = if transpose_m {
            self.m.transpose()
        } else {
            self.m.clone()
        };
        let s = m
            .lu()
            .solve(&t)
            .expect("Woodbury k×k capacitance solve (M = I + VᵀZ is invertible)");
        for (j, c) in out_cols.iter().enumerate() {
            let sj = s[j];
            for (r, &cr) in rhs.iter_mut().zip(c.iter()) {
                *r -= cr * sj;
            }
        }
    }
}

/// A factored free-DOF tangent: the symmetric factor plus an optional asymmetric
/// friction [`WoodburyCorrection`]. `None` correction ⇒ a plain symmetric solve,
/// bit-identical to the pre-friction path (the frictionless / `x_prev = None` case).
pub(crate) struct FactoredFreeTangent {
    inner: FactorInner,
    woodbury: Option<WoodburyCorrection>,
}

impl FactoredFreeTangent {
    /// Wrap a bare symmetric factor with no friction correction (the frictionless path).
    const fn symmetric(inner: FactorInner) -> Self {
        Self {
            inner,
            woodbury: None,
        }
    }

    /// Solve `A_sym · x = rhs` in place — the SYMMETRIC factor only, no friction
    /// correction. The forward Newton step and Woodbury `Z = A_sym⁻¹ U` build both use
    /// this (the forward Hessian is the frozen-lag symmetric tangent).
    pub(crate) fn solve_base_in_place(&self, rhs: &mut [f64]) {
        let n = rhs.len();
        let mat = MatMut::from_column_major_slice_mut(rhs, n, 1);
        self.inner.solve_in_place_with_conj(Conj::No, mat);
    }

    /// Solve the full free-DOF adjoint `A · x = rhs` in place: the symmetric back-solve
    /// followed by the friction Woodbury tail (a no-op when `woodbury` is `None`). This
    /// is the shared solve every adjoint consumer (sensitivities + tape VJPs) routes
    /// through, so friction-exactness is transparent.
    pub(crate) fn solve_free_in_place(&self, rhs: &mut [f64]) {
        self.solve_base_in_place(rhs);
        if let Some(wb) = &self.woodbury {
            wb.apply_in_place(rhs);
        }
    }

    /// Solve the TRANSPOSE free-DOF adjoint `Aᵀ · x = rhs` in place — the symmetric
    /// back-solve (`A_sym⁻ᵀ = A_sym⁻¹`) followed by the transpose Woodbury tail. This is
    /// the solve the reverse-mode tape (`TrajectoryStepVjp::vjp`) needs: a VJP contracts
    /// `(∂x*/∂·)ᵀ = −(∂r/∂·)ᵀ A⁻ᵀ`, so the cotangent passes through `A⁻ᵀ`, not `A⁻¹`. With
    /// no friction (`woodbury == None`) `A` is symmetric and this is bit-identical to
    /// [`Self::solve_free_in_place`].
    pub(crate) fn solve_free_transpose_in_place(&self, rhs: &mut [f64]) {
        self.solve_base_in_place(rhs);
        if let Some(wb) = &self.woodbury {
            wb.apply_transpose_in_place(rhs);
        }
    }

    /// `true` if the symmetric part factored as `Llt` (happy path) vs the `Lu` fallback.
    pub(crate) const fn is_llt(&self) -> bool {
        matches!(self.inner, FactorInner::Llt(_))
    }

    /// `true` if the symmetric part fell back to the `Lu` factor.
    pub(crate) const fn is_lu(&self) -> bool {
        matches!(self.inner, FactorInner::Lu(_))
    }
}

/// The friction reaction force on the rigid collider along a chosen direction, plus its
/// first-order sensitivities — the tangential-grip readout a staggered coupling routes
/// onto its rigid-state tape.
///
/// Produced by [`CpuNewtonSolver::friction_reaction_gradients`] (which documents the
/// math). All fields are along the `react_dir` the gradients were built with; `dforce_dx`
/// is length `n_dof` (zeros off the active set), the rest scalar.
#[derive(Clone, Debug)]
pub struct FrictionReactionGradients {
    /// `F = (Σ_v ∇D_v)·react_dir` — the reaction force along `react_dir`.
    pub force: f64,
    /// `∂F/∂x*` (length `n_dof`) — frozen-lag `∇²D` slip term plus the normal-force
    /// (λⁿ) coupling.
    pub dforce_dx: Vec<f64>,
    /// `∂F/∂x_prev` (length `n_dof`) — the friction reference `x_start = x_prev + Δ_surf`
    /// makes the reaction depend on the step-start config too: `∂F/∂x_prev = −`(the
    /// frozen-lag `∇²D` slip term of [`Self::dforce_dx`], λⁿ-coupling excluded, since λⁿ
    /// tracks `x*` not `x_prev`). The state-config companion of [`Self::dforce_ddrift`].
    pub dforce_dxprev: Vec<f64>,
    /// `∂F/∂Δ_surf` along the build's `drift_dir` (the moving-collider reference shift,
    /// λ-independent).
    pub dforce_ddrift: f64,
    /// `∂F/∂height` along the build's `pose_dir` plane translation (the λ-coupling).
    pub dforce_dheight: f64,
}

/// The per-vertex friction force on the rigid collider `∇D_v` (a 3-vector) and its
/// first-order sensitivities.
///
/// The VECTOR, per-contacted-vertex generalization of [`FrictionReactionGradients`] (which
/// projects onto a single `react_dir` and sums). A staggered coupling needs the per-vertex
/// VECTOR force to assemble the off-COM friction MOMENT `Σ_v (r_v − c) × ∇D_v` and its
/// Jacobian; the scalar aggregate cannot (the moment arm `r_v − c` weights each vertex
/// differently).
///
/// `force = ∇D_v` is the reaction on the RIGID body at vertex `v` (the soft body feels
/// `−∇D_v`). `dforce_dx`/`dforce_dxprev` are row-major `3 × n_dof` blocks (`∂force[r]/∂x[col]`
/// at flat index `r·n_dof + col`); the rest are `3`-vectors. Produced by
/// [`CpuNewtonSolver::friction_force_jacobians`].
#[derive(Clone, Debug)]
pub struct FrictionVertexForce {
    /// The contacted vertex.
    pub vid: VertexId,
    /// `∇D_v` — the friction force on the rigid body at `v` (on the soft body: `−∇D_v`).
    pub force: Vec3,
    /// `∂force/∂x*` — row-major `3 × n_dof` (frozen-lag `∇²D_v` slip at `v`'s own coords plus
    /// the normal-force λⁿ coupling `a_v ⊗ ∂λⁿ_v/∂x_c` spread over the contact-neighbor coords).
    pub dforce_dx: Vec<f64>,
    /// `∂force/∂x_prev` — row-major `3 × n_dof`; `−`(the frozen-lag `∇²D_v` slip at `v`'s coords),
    /// λⁿ-coupling excluded (λⁿ tracks `x*`, not `x_prev`), via `x_start = x_prev + Δ_surf`.
    pub dforce_dxprev: Vec<f64>,
    /// `∂force/∂Δ_surf` along the build's `drift_dir` (`−∇²D_v·drift_dir`, the moving-collider
    /// reference shift, λ-independent).
    pub dforce_ddrift: Vec3,
    /// `∂force/∂height` along the build's `pose_dir` plane translation (`a_v·(n̂·∂(plane)/∂pose)`,
    /// the λⁿ coupling).
    pub dforce_dheight: Vec3,
    /// `∂force/∂μ_c` — the friction force is `∇D_v = μ_c·λⁿ_v·f₁·Tû`, LINEAR in the Coulomb
    /// coefficient `μ_c`, so `∂force/∂μ_c = ∇D_v/μ_c` (the DIRECT channel, at fixed `x*`/λⁿ). The
    /// dominant lever for the friction-coefficient co-design gradient (the soft `x*` channel is
    /// tiny in deep slip).
    pub dforce_dmu_c: Vec3,
}

/// CPU backward-Euler Newton solver.
///
/// Six generic parameters: element `E<N, G>`, mesh `Msh`, contact `C`,
/// material `M`, and const-generic `(N, G)` for element shape.
///
/// `M` defaults to [`crate::material::NeoHookean`] for back-compat with Phase 4 scope
/// memo Decision G's monomorphization. Yeoh consumers (row 23+) write
/// `M = Yeoh` explicitly, typically via the [`crate::CpuTet4YeohSolver`]
/// alias, and use a `Mesh<Yeoh>` impl such as
/// `SdfMeshedTetMesh<Yeoh>` per arc memo D10. Per-tet `M`
/// instances live on the mesh and are read at the assembly hot
/// points via `self.mesh.materials()`.
pub struct CpuNewtonSolver<
    E,
    Msh,
    C,
    M = crate::material::NeoHookean,
    const N: usize = 4,
    const G: usize = 1,
> where
    E: Element<N, G>,
    Msh: Mesh<M>,
    M: Material,
    C: ContactModel + ActivePairsFor<M>,
{
    element: E,
    mesh: Msh,
    // Read once per Newton iter from `assemble_global_int_force` and
    // `assemble_free_hessian_triplets` (per-iter active-pair
    // recompute). `NullContact`'s zero-stubs preserve pre-penalty
    // numerics on non-contact scenes; `tests/contact_passthrough.rs`
    // is the regression-net spine.
    contact: C,
    config: SolverConfig,
    boundary_conditions: BoundaryConditions,

    // ── Assembly cache, populated by `new()`. ──
    // Read by `solve_impl` / `factor_at_position` / `armijo_backtrack`
    // and the assembly methods below. Replaces the pre-Phase-2
    // hardcoded `N_DOF` / `N_FREE` / `FREE_OFFSET` constants and the
    // per-iter `reference_geometry` recomputation.
    /// One entry per mesh tet — material-frame shape gradients and
    /// rest-configuration volume.
    element_geometries: Vec<ElementGeometry>,
    /// Lumped per-DOF mass (`length n_dof`). For each DOF `i` belonging
    /// to vertex `v = i / 3`, the entry is `Σ_e (ρ V_e / 4)` over every
    /// element `e` that contains `v`. Phase 2 reproduces the
    /// walking-skeleton's "per-vertex mass = ρ `V_total` / 4" rule when
    /// every vertex sits in exactly one tet.
    mass_per_dof: Vec<f64>,
    /// Full-DOF indices of the free DOFs, in ascending order. For the
    /// 1-tet skeleton: `[9, 10, 11]` (`v_3`'s xyz). For multi-tet:
    /// every non-pinned vertex's three DOFs.
    free_dof_indices: Vec<usize>,
    /// Inverse map of `free_dof_indices`: entry `i` is `Some(k)` when
    /// full-DOF `i` is the k-th free DOF, else `None` (pinned). Used
    /// for O(1) "is this DOF free?" lookups during sparse-pattern
    /// build and tangent assembly. Field name pairs with
    /// `free_dof_indices` (the forward direction: free idx → full
    /// DOF) — `full_to_free_idx` is the inverse direction.
    full_to_free_idx: Vec<Option<usize>>,
    /// Symbolic factor of the free-DOF Hessian sparsity pattern (Llt
    /// shape, `Side::Lower`), built once from element-vertex incidence
    /// per Decision J. Per-iter numeric refactor consumes a `clone()`
    /// of this (cheap — faer 0.24 wraps the symbolic in `Arc`
    /// internally).
    symbolic: SymbolicLlt<usize>,
    /// Symbolic factor of the same free-DOF Hessian pattern, in Lu
    /// shape (full matrix, no `Side`). Held alongside `symbolic` so
    /// the A2 LU fallback (Lu factorize when Llt hits a non-PD pivot)
    /// can run without rebuilding the symbolic factor at the failure
    /// site. Construction cost is one-shot at `new()` and small
    /// relative to the numeric factor; same `Arc`-internal sharing
    /// makes `clone()` cheap per fall-through.
    symbolic_lu: SymbolicLu<usize>,
    /// Total DOF count (`3 * n_vertices`), cached for slice indexing.
    n_dof: usize,
    /// Free DOF count (`free_dof_indices.len()`), cached.
    n_free: usize,

    /// The rigid contact surface's **within-step tangential drift** `Δ_surf` —
    /// the displacement the (kinematic) rigid collider sweeps over the step,
    /// against which the smoothed-Coulomb friction measures the soft vertices'
    /// RELATIVE tangential slip: `u_T = Tⁿᵀ((x_v − xᵗ_v) − Δ_surf)`. Default
    /// `(0,0,0)` recovers PR1's one-way (static-collider) friction
    /// byte-identically; a non-zero drift lets a *moving* collider DRAG the soft
    /// body (the two-way grip a sliding device exerts on a held limb). Set via
    /// [`Self::with_friction_surface_drift`]. Uniform translation only for now
    /// (a single `Δ_surf` for all pairs); per-contact-point rotation (`ω×r`) is a
    /// future refinement. Only consulted when `config.friction_mu > 0`.
    friction_surface_drift: Vec3,

    /// Phantom — `M` only appears in the `Msh: Mesh<M>` and
    /// `C: ContactModel + ActivePairsFor<M>` bounds, not in any
    /// concrete field. The marker tells rustc the type parameter is
    /// intentionally type-only.
    _material: std::marker::PhantomData<M>,
}

impl<E, Msh, C, M, const N: usize, const G: usize> CpuNewtonSolver<E, Msh, C, M, N, G>
where
    E: Element<N, G>,
    Msh: Mesh<M>,
    M: Material,
    C: ContactModel + ActivePairsFor<M>,
{
    /// Assemble a solver from its element, mesh, contact, integration
    /// configuration, and boundary conditions. Per-tet `M` instances
    /// are read from `mesh.materials()` at assembly time.
    ///
    /// `Box<dyn Solver<Tape = CpuTape>>` is the intended public handle;
    /// direct access to the concrete type is only needed for
    /// monomorphized benches (Phase E+). The assembly cache
    /// (`element_geometries`, `mass_per_dof`, `free_dof_indices`,
    /// `full_to_free_idx`, `symbolic`, `n_dof`, `n_free`) is populated
    /// here once and consumed by `solve_impl` / `factor_at_position` /
    /// `armijo_backtrack` per Newton iter. `new()` is non-`const fn`
    /// because the cache build runs `mesh.tet_vertices`, sparse-pattern
    /// construction, and faer's symbolic factorization — none of which
    /// are const-evaluable.
    //
    // expect_used + panic justifications:
    //   • Singular reference Jacobian = malformed rest mesh, programmer
    //     bug at construction time.
    //   • SymbolicLlt failure = solver-pattern build is wrong (impossible
    //     for any valid mesh + Dirichlet set), programmer bug.
    //   • SymbolicLu failure shares the same "pattern build wrong" rationale;
    //     LU symbolic factorization of a valid free-block pattern only
    //     errors out on OutOfMemory, which on a healthy host is a programmer
    //     bug on the mesh-size axis.
    //
    // Lint allows: same Mesh-trait-API tax + Tet4 4-node iteration +
    // per-element grad/grad_generic similar-name pair as the assembly
    // methods below.
    #[must_use]
    #[allow(
        clippy::expect_used,
        clippy::too_many_lines,
        clippy::cast_possible_truncation,
        clippy::needless_range_loop,
        clippy::similar_names
    )]
    pub fn new(
        element: E,
        mesh: Msh,
        contact: C,
        config: SolverConfig,
        boundary_conditions: BoundaryConditions,
    ) -> Self {
        assert!(
            N == 4,
            "Phase 2 solver is pinned to Tet4 (N=4) per scope §3 Decision A; got N={N}"
        );

        let n_tets = mesh.n_tets();
        let n_vertices = mesh.n_vertices();
        let n_dof = 3 * n_vertices;

        // 0. BoundaryConditions validation. Catch misconfigured scenes
        // at construction time rather than later as opaque
        // out-of-bounds panics inside the cache build or first
        // step()-call. Four checks: (a) pinned vertex IDs are in
        // range, (b) loaded vertex IDs are in range, (c) no vertex
        // appears in both pinned and loaded sets (load on a
        // Dirichlet-clamped DOF is unphysical and would silently
        // overwrite f_ext on a vertex that never moves), (d) loaded
        // vertices are referenced by some tet (load on an orphan is
        // unphysical — no element receives the traction; checked
        // below after the tet incidence walk).
        let pinned_set: BTreeSet<VertexId> = boundary_conditions
            .pinned_vertices
            .iter()
            .copied()
            .collect();
        let n_vertices_u32 =
            VertexId::try_from(n_vertices).expect("n_vertices must fit in VertexId (u32)");
        for &v in &boundary_conditions.pinned_vertices {
            assert!(
                v < n_vertices_u32,
                "BoundaryConditions.pinned_vertices contains vertex ID {v}, \
                 out of range for {n_vertices}-vertex mesh",
            );
        }
        for &(v, _) in &boundary_conditions.loaded_vertices {
            assert!(
                v < n_vertices_u32,
                "BoundaryConditions.loaded_vertices contains vertex ID {v}, \
                 out of range for {n_vertices}-vertex mesh",
            );
            assert!(
                !pinned_set.contains(&v),
                "BoundaryConditions.loaded_vertices contains vertex ID {v}, \
                 which is also in pinned_vertices — load on a Dirichlet-clamped \
                 vertex is unphysical (f_ext is overwritten but the vertex \
                 never moves)",
            );
        }

        // Roller / per-axis Dirichlet validation + dense mask. A roller
        // pins a vertex to rest on the `true` axes of its mask while
        // leaving the `false` axes free — the per-DOF generalization of
        // `pinned_vertices`. `roller_mask[v]` defaults to all-free; the
        // per-DOF free-DOF construction below consults it per axis.
        let mut roller_mask = vec![[false; 3]; n_vertices];
        let mut roller_seen: BTreeSet<VertexId> = BTreeSet::new();
        for &(v, mask) in &boundary_conditions.roller_vertices {
            assert!(
                v < n_vertices_u32,
                "BoundaryConditions.roller_vertices contains vertex ID {v}, \
                 out of range for {n_vertices}-vertex mesh",
            );
            assert!(
                mask.iter().any(|&b| b),
                "BoundaryConditions.roller_vertices entry for vertex ID {v} has \
                 an all-false mask (no axis constrained) — a free vertex needs no \
                 entry; use a mask with at least one constrained axis",
            );
            assert!(
                roller_seen.insert(v),
                "BoundaryConditions.roller_vertices lists vertex ID {v} twice \
                 (ambiguous per-axis mask)",
            );
            assert!(
                !pinned_set.contains(&v),
                "BoundaryConditions vertex ID {v} is in both pinned_vertices and \
                 roller_vertices — a full pin already constrains every axis, so a \
                 roller on the same vertex is ambiguous",
            );
            roller_mask[v as usize] = mask;
        }
        // A loaded vertex must be free on ALL three axes: the external-force
        // assembly and the autograd VJP both resolve its xyz free-DOF indices
        // (`full_to_free_idx[3v + {0,1,2}]` must be `Some`). Loaded ∩ pinned is
        // rejected above; reject loaded ∩ roller here for the same reason.
        for &(v, _) in &boundary_conditions.loaded_vertices {
            assert!(
                !roller_seen.contains(&v),
                "BoundaryConditions.loaded_vertices contains vertex ID {v}, \
                 which is also roller-constrained — a loaded vertex must be free \
                 on all three axes (assembly + VJP resolve all xyz free indices)",
            );
        }

        // Tet incidence walk. Used immediately below to (a) reject
        // loads on unreferenced vertices and (b) auto-pin
        // unreferenced vertices into `effective_pinned`. An orphan
        // free DOF would have zero mass (no element contributes to
        // `mass_per_dof` for that vertex) AND zero element
        // contribution to its Hessian row/column, leaving a singular
        // diagonal (`(k, k) == 0`) that faer's Cholesky would either
        // panic on (slice-out-of-bounds during pattern permute) or
        // silently produce garbage. Phase 2's hand-built meshes
        // never tripped this because every vertex was referenced;
        // Phase 3's `SdfMeshedTetMesh` retains the full BCC lattice
        // in `positions()` including unreferenced corners outside
        // the SDF zero set (per its module doc) and surfaces this
        // gap via III-3.
        let mut referenced: BTreeSet<VertexId> = BTreeSet::new();
        for tet_id in 0..n_tets as TetId {
            for v in mesh.tet_vertices(tet_id) {
                referenced.insert(v);
            }
        }
        for &(v, _) in &boundary_conditions.loaded_vertices {
            assert!(
                referenced.contains(&v),
                "BoundaryConditions.loaded_vertices contains vertex ID {v}, \
                 which is not referenced by any tet — load on an orphan \
                 vertex is unphysical (no element receives the traction)",
            );
        }
        // Effective pinned set: user-supplied pins UNION (vertices
        // not referenced by any tet). Orphans pinned silently —
        // their DOFs were unsolveable anyway, and exposing this as a
        // hard error would force every mesher-generated-mesh test
        // site to construct an orphan-aware BC. Per-iter assembly
        // reads `full_to_free_idx` (built from `effective_pinned`
        // below), so the auto-pin propagates without any other code
        // change. The `boundary_conditions` field stored on the
        // struct keeps the user's supplied form for transparency.
        let mut effective_pinned: BTreeSet<VertexId> = pinned_set;
        for v in 0..n_vertices_u32 {
            if !referenced.contains(&v) {
                effective_pinned.insert(v);
            }
        }

        // 1. Per-element reference geometry. Computed once; all per-iter
        // assembly reads from this cache rather than recomputing.
        let x_rest = mesh.positions();
        let mut element_geometries = Vec::with_capacity(n_tets);
        // Parametric gradients are constant for Tet4; pass centroid ξ.
        let grad_xi_n = element.shape_gradients(Vec3::new(0.25, 0.25, 0.25));
        for tet_id in 0..n_tets as TetId {
            let verts = mesh.tet_vertices(tet_id);
            let v0 = x_rest[verts[0] as usize];
            let v1 = x_rest[verts[1] as usize];
            let v2 = x_rest[verts[2] as usize];
            let v3 = x_rest[verts[3] as usize];
            let j_0 = Matrix3::from_columns(&[v1 - v0, v2 - v0, v3 - v0]);
            let j_0_inv = j_0
                .try_inverse()
                .expect("singular reference Jacobian — malformed rest mesh");
            let volume = j_0.determinant().abs() / 6.0;
            // Chain rule: grad_X N_a = grad_ξ N_a · (∂ξ/∂X) = grad_ξ N_a · J_0⁻¹.
            // Result is SMatrix<f64, N, 3>; copy first 4 rows out to the
            // concrete 4-node matrix per Tet4 (N == 4 asserted above).
            let grad_x_n_generic: SMatrix<f64, N, 3> = grad_xi_n * j_0_inv;
            let mut grad_x_n = SMatrix::<f64, 4, 3>::zeros();
            for a in 0..4 {
                for j in 0..3 {
                    grad_x_n[(a, j)] = grad_x_n_generic[(a, j)];
                }
            }
            element_geometries.push(ElementGeometry { grad_x_n, volume });
        }

        // 2. Lumped per-DOF mass. For each element `e`, contribute
        // `ρ V_e / 4` to every DOF of every vertex of `e`. Vertices
        // shared by multiple elements accumulate.
        let mut mass_per_dof = vec![0.0; n_dof];
        for tet_id in 0..n_tets as TetId {
            let verts = mesh.tet_vertices(tet_id);
            let mass_per_vertex_share =
                config.density * element_geometries[tet_id as usize].volume / 4.0;
            for a in 0..4 {
                let v = verts[a] as usize;
                mass_per_dof[3 * v] += mass_per_vertex_share;
                mass_per_dof[3 * v + 1] += mass_per_vertex_share;
                mass_per_dof[3 * v + 2] += mass_per_vertex_share;
            }
        }

        // 3. Free-DOF index mapping. Walks vertices in natural order
        // (deterministic per scope §15 D-3 + Decision M); uses
        // `effective_pinned` (user pins UNION auto-pinned orphans) and
        // `roller_mask` (per-axis Dirichlet) built at validation time.
        // No HashMap on numeric paths per Decision M.
        //
        // Per-DOF freedom: a full pin fixes all three axes; a roller
        // fixes only its `true` axes. A fully-free vertex pushes 3v,
        // 3v+1, 3v+2 in order, so a scene with no rollers yields a
        // BIT-IDENTICAL free-DOF map to the pre-roller per-vertex
        // construction (regression-guarded in tests).
        let mut free_dof_indices = Vec::with_capacity(n_dof);
        for v in 0..n_vertices as VertexId {
            let v_idx = v as usize;
            let full_pin = effective_pinned.contains(&v);
            for ax in 0..3 {
                if !full_pin && !roller_mask[v_idx][ax] {
                    free_dof_indices.push(3 * v_idx + ax);
                }
            }
        }
        let n_free = free_dof_indices.len();
        let mut full_to_free_idx: Vec<Option<usize>> = vec![None; n_dof];
        for (free_idx, &full_idx) in free_dof_indices.iter().enumerate() {
            full_to_free_idx[full_idx] = Some(free_idx);
        }

        // 4. Symbolic factor of the free-DOF Hessian sparsity pattern,
        // built from element-vertex incidence (Decision J). For each
        // element `e`, every (a, b) vertex pair contributes a 3×3
        // block at the (free_idx_a, free_idx_b) location, IF both `a`
        // and `b` are free. BTreeSet keyed by (col, row) gives sorted
        // column-major lower-triangle iteration without a HashMap.
        let mut triplet_set: BTreeSet<(usize, usize)> = BTreeSet::new();
        for tet_id in 0..n_tets as TetId {
            let verts = mesh.tet_vertices(tet_id);
            for a in 0..4 {
                for b in 0..4 {
                    let va = verts[a] as usize;
                    let vb = verts[b] as usize;
                    for i in 0..3 {
                        for j in 0..3 {
                            let row_full = 3 * va + i;
                            let col_full = 3 * vb + j;
                            if let (Some(row_free), Some(col_free)) =
                                (full_to_free_idx[row_full], full_to_free_idx[col_full])
                                && row_free >= col_free
                            {
                                triplet_set.insert((col_free, row_free));
                            }
                        }
                    }
                }
            }
        }
        let pattern_triplets: Vec<Triplet<usize, usize, f64>> = triplet_set
            .iter()
            .map(|&(c, r)| Triplet::new(r, c, 1.0))
            .collect();
        let pattern_mat: SparseColMat<usize, f64> =
            SparseColMat::try_new_from_triplets(n_free, n_free, &pattern_triplets)
                .expect("malformed free-block triplet pattern");
        let symbolic = SymbolicLlt::<usize>::try_new(pattern_mat.symbolic(), Side::Lower)
            .expect("symbolic factorization of free-block pattern failed");

        // A2 LU fallback: reflect the structurally-symmetric lower-tri
        // pattern into the full pattern needed by `SymbolicLu` (Lu has
        // no `Side` argument; it reads both halves). Diagonal entries
        // emit once; off-diagonals emit at both (r, c) and (c, r). The
        // numeric Lu factor at fall-through symmetrizes the assembled
        // tangent the same way.
        let mut pattern_triplets_full: Vec<Triplet<usize, usize, f64>> =
            Vec::with_capacity(triplet_set.len() * 2);
        for &(c, r) in &triplet_set {
            pattern_triplets_full.push(Triplet::new(r, c, 1.0));
            if c != r {
                pattern_triplets_full.push(Triplet::new(c, r, 1.0));
            }
        }
        let pattern_mat_full: SparseColMat<usize, f64> =
            SparseColMat::try_new_from_triplets(n_free, n_free, &pattern_triplets_full)
                .expect("malformed full free-block triplet pattern");
        let symbolic_lu = SymbolicLu::<usize>::try_new(pattern_mat_full.symbolic())
            .expect("symbolic LU factorization of free-block pattern failed");

        Self {
            element,
            mesh,
            contact,
            config,
            boundary_conditions,
            element_geometries,
            mass_per_dof,
            free_dof_indices,
            full_to_free_idx,
            symbolic,
            symbolic_lu,
            n_dof,
            n_free,
            friction_surface_drift: Vec3::zeros(),
            _material: std::marker::PhantomData,
        }
    }

    /// Set the rigid contact surface's within-step tangential drift `Δ_surf` (the
    /// `friction_surface_drift` field) and return the solver, builder-style. The
    /// drift is the kinematic collider's
    /// tangential displacement over the step (`≈ v_surf·dt`); friction measures
    /// the soft body's slip RELATIVE to it, so a non-zero drift lets a moving
    /// collider drag the soft body. `Δ_surf = (0,0,0)` (the default) is PR1's
    /// static-collider friction, byte-identical.
    #[must_use]
    pub const fn with_friction_surface_drift(mut self, drift: Vec3) -> Self {
        self.friction_surface_drift = drift;
        self
    }

    /// Check every per-tet [`Material::validity`] domain against the
    /// deformation gradient `F` evaluated at `x_curr` (Phase 4 commit
    /// 12, IV-7 per scope memo Decision Q).
    ///
    /// First-violator-wins: walks tets in ascending `tet_id` order,
    /// computes `F = Σ_a x_{a,i} · ∂N_a/∂X_j` per element, and panics
    /// at the first tet whose `F` falls outside the declared
    /// [`crate::ValidityDomain`]. The two slots checkable from `F`
    /// for every base [`Material`] impl Phase 4 ships are
    /// `inversion` (`det F ≤ 0` under
    /// [`InversionHandling::RequireOrientation`]) and
    /// `max_stretch_deviation` (max `|σ_i − 1|` over the three
    /// singular values `σ_i` of `F`). The other four
    /// [`crate::ValidityDomain`] slots are construction-time
    /// (`poisson_range`) or decorator-only (`temperature_range`,
    /// `strain_rate_range`, `max_rotation` infinite for the
    /// scalar-isotropic NH baseline) and not checked here.
    ///
    /// Diagnostic-only at the solver level — Decision K's "Newton
    /// hot path does not branch on diagnostic metadata" framing
    /// applies to the interface flag, not to validity; this check
    /// runs at two step boundaries per [`Solver::step`] call:
    /// (1) before the Newton loop starts (the original Decision Q
    /// "at step start" framing — catches invalid warm-starts), AND
    /// (2) at end of solve before returning a converged result
    /// (catches Newton converging to an invalid equilibrium —
    /// without this check, an invalid converged state silently
    /// flows to the next step's start check, making the failure
    /// step-delayed + the failed step's recorded output
    /// physically meaningless).  Both boundaries panic on first
    /// violation rather than degrading silently.  See
    /// `docs/CANDIDATE_E_B_FALSIFICATION_BOOKMARK.md` §10 for the
    /// motivating finding (cavity > 5 mm sliding-ramp step 1
    /// converged to `σ_max = 2.05` at tet 3206, was only caught at
    /// step 2's start check pre-this-fix).  The book Part 2 §00
    /// §02 prescription is a runtime warning; Decision Q upgrades
    /// to fail-closed semantics for Phase 4.
    ///
    /// # Errors
    ///
    /// Returns [`SolverFailure::ValidityViolation`] on the first violator, carrying the violated
    /// `tet_id` and the structured `message`
    /// `"validity violation at tet {id}: {slot} = {value:.3} ..."` (where `{slot}` is one of
    /// `max_stretch_deviation` / `max_principal_stretch` / `min_principal_stretch` / `inversion`).
    /// The `try_step`/`try_replay_step` callers surface this `Err` so a feasibility-aware caller can
    /// skip the design; the panic-path `step`/`replay_step` re-`panic!` with the same `message`, so
    /// the IV-7 `#[should_panic(expected = "max_stretch_deviation")]` slot-substring contract holds.
    //
    // similar_names: `tet_id`/`tet` mirrors the assembly methods.
    // cast_possible_truncation: same Mesh-trait API tax as the
    // assembly methods.
    #[allow(clippy::similar_names, clippy::cast_possible_truncation)]
    fn check_validity_at_step_start(&self, x_curr: &[f64]) -> Result<(), SolverFailure> {
        debug_assert!(x_curr.len() == self.n_dof);
        let materials = self.mesh.materials();
        for (tet_id, geom) in self.element_geometries.iter().enumerate() {
            let verts = self.mesh.tet_vertices(tet_id as TetId);
            let x_elem = extract_element_dof_values(x_curr, &verts);
            let f = deformation_gradient(&x_elem, &geom.grad_x_n);
            let validity = materials[tet_id].validity();

            // Inversion check first: a non-positive `det F` makes the
            // SVD-based stretch check below ambiguous (singular values
            // are non-negative), so the orientation slot is the
            // structurally prior gate. Programs that allow `det F <=
            // 0` declare a non-`RequireOrientation` inversion handler,
            // and Phase 4 has none — Phase H may add `Barrier` /
            // `OptIn` variants when an impl needs them.
            let det_f = f.determinant();
            if matches!(validity.inversion, InversionHandling::RequireOrientation) && det_f <= 0.0 {
                return Err(SolverFailure::ValidityViolation {
                    tet_id,
                    message: format!(
                        "validity violation at tet {tet_id}: inversion = det F = \
                         {det_f:.3} violates RequireOrientation handler (must be \
                         strictly positive). Phase 4 scope memo Decision Q \
                         fail-closed semantics."
                    ),
                });
            }

            // Principal-stretch bounds: SVD `F = U Σ V^T` gives
            // singular values `σ_i` which are the principal stretches.
            // `f.svd_unordered(false, false)` skips U/V (we only need
            // σ); cheap O(27) FLOPs per tet. Singular values are
            // non-negative; a reflection-only F has σ_i ≥ 0 with
            // `det F < 0`, already caught above.
            //
            // Two gate flavors (Yeoh arc memo D8): if either of the new
            // asymmetric bounds is `Some`, gate per-bound; else fall
            // back to the legacy NH symmetric `max_i |σ_i - 1|` bound.
            //
            // Edge case: `(Some, None)` or `(None, Some)` checks only
            // the populated bound — the other direction is unchecked.
            // No production constructor reaches that state today (every
            // `SiliconeMaterial::to_yeoh` sets both via
            // `with_principal_stretch_bounds`); future asymmetric-only
            // callers opt into the unchecked direction by construction.
            let svd = f.svd_unordered(false, false);
            let sigma = svd.singular_values;
            match (
                validity.max_principal_stretch,
                validity.min_principal_stretch,
            ) {
                (None, None) => {
                    let max_dev = sigma
                        .iter()
                        .map(|s| (s - 1.0).abs())
                        .fold(0.0_f64, f64::max);
                    let bound = validity.max_stretch_deviation;
                    if max_dev > bound {
                        return Err(SolverFailure::ValidityViolation {
                            tet_id,
                            message: format!(
                                "validity violation at tet {tet_id}: max_stretch_deviation \
                                 = {max_dev:.3} exceeds bound {bound:.3} (singular values \
                                 of F = [{s0:.3}, {s1:.3}, {s2:.3}]). Phase 4 scope memo \
                                 Decision Q fail-closed semantics.",
                                s0 = sigma[0],
                                s1 = sigma[1],
                                s2 = sigma[2],
                            ),
                        });
                    }
                }
                (max_p, min_p) => {
                    if let Some(max) = max_p {
                        let max_sigma = sigma.iter().fold(0.0_f64, |a, &b| a.max(b));
                        if max_sigma > max {
                            return Err(SolverFailure::ValidityViolation {
                                tet_id,
                                message: format!(
                                    "validity violation at tet {tet_id}: max_principal_stretch \
                                     = {max_sigma:.3} exceeds bound {max:.3} (singular values \
                                     of F = [{s0:.3}, {s1:.3}, {s2:.3}]). Phase 4 scope memo \
                                     Decision Q fail-closed semantics.",
                                    s0 = sigma[0],
                                    s1 = sigma[1],
                                    s2 = sigma[2],
                                ),
                            });
                        }
                    }
                    if let Some(min) = min_p {
                        let min_sigma = sigma.iter().fold(f64::INFINITY, |a, &b| a.min(b));
                        if min_sigma < min {
                            return Err(SolverFailure::ValidityViolation {
                                tet_id,
                                message: format!(
                                    "validity violation at tet {tet_id}: min_principal_stretch \
                                     = {min_sigma:.3} below bound {min:.3} (singular values \
                                     of F = [{s0:.3}, {s1:.3}, {s2:.3}]). Phase 4 scope memo \
                                     Decision Q fail-closed semantics.",
                                    s0 = sigma[0],
                                    s1 = sigma[1],
                                    s2 = sigma[2],
                                ),
                            });
                        }
                    }
                }
            }
        }
        Ok(())
    }

    /// Free-DOF residual norm (scope §5 R-1 convergence criterion).
    /// Reads only the entries at `self.free_dof_indices`; pinned-DOF
    /// residual entries are excluded from the convergence test.
    fn free_residual_norm(&self, r_full: &[f64]) -> f64 {
        debug_assert!(r_full.len() == self.n_dof);
        self.free_dof_indices
            .iter()
            .map(|&idx| r_full[idx] * r_full[idx])
            .sum::<f64>()
            .sqrt()
    }

    /// Factor the free-DOF condensed tangent assembled from `triplets`
    /// (lower-tri, column-major sorted), with F3 Levenberg-Marquardt
    /// `+λI` retry rescue.
    ///
    /// Cheap-path-first: try `Llt` on the cached symbolic factor; on
    /// `LltError::Numeric` (non-PD pivot — an indefinite tangent at the
    /// current Newton iter or position) the F3 retry loop bumps `λ`
    /// (seed = `seed_relative × max_diag` on first non-PD per call,
    /// then `λ ×= up_factor`) and retries the Llt on the regularized
    /// `A + λI`. On Llt success, `λ` decays via `on_llt_success` for
    /// the next call. When the retry budget exhausts OR `λ` saturates
    /// at `max_relative × max_diag`, the loop falls through to LU on
    /// the ORIGINAL (un-regularized) triplets — per spec §2.1, LU on
    /// the regularized system would conflate "LM saturation" with "LU
    /// on `λ_max`-regularized system" and make the saturation surface
    /// a moving target.
    ///
    /// When `lm_state` is [`LmState::disabled`] (the bit-equal path
    /// for `SolverConfig::lm_regularization == None`), `can_bump`
    /// returns `false` unconditionally, so the loop reduces to a
    /// single Llt attempt + direct LU fallback (today's pre-F3
    /// behavior, observably bit-equal including the existing
    /// `"sim-soft: faer LU fallback fired"` stderr line).
    ///
    /// `context` is a `&str` tag included in the fallback warn-log,
    /// the LM seed/bump/summary logs, and the doubly-failed panic
    /// message so debug can identify which factor site fired which
    /// path.
    ///
    /// **MAINTENANCE NOTE**: this method's retry-loop body is
    /// structurally mirrored by [`Self::try_factor_free_tangent`]
    /// (per F3.3 — the two diverge only at the LU-fallback dispatch:
    /// this one calls [`Self::lu_fallback`] which panics on
    /// doubly-failed factor; the `try_*` sibling calls
    /// [`Self::try_lu_fallback`] which returns `Err`). Spec authority
    /// (F3.3 `try_factor_free_tangent` doc) chose duplication over
    /// shared inner-helper to keep this method's hot path free of
    /// `Result` indirection. Any change to the retry-loop body
    /// (seed/bump rules, log forms, the Cow regularization branch)
    /// MUST be mirrored to `try_factor_free_tangent` to avoid silent
    /// divergence.
    //
    // expect_used + panic justifications (helper owns both):
    //   • `SparseColMat::try_new_from_triplets` can only fail on
    //     duplicate triplet entries, which the BTreeMap accumulator in
    //     `assemble_free_hessian_triplets` (and the new() pattern
    //     reflection) prevents by construction. `triplets_with_diagonal_
    //     offset` mutates existing diagonal entries in place rather
    //     than appending, preserving the no-duplicates invariant.
    //   • LltError::Generic (OutOfMemory) and LuError post-fallback are
    //     both non-recoverable: the symbolic patterns were validated at
    //     new(), so a fresh failure is either OOM or a deeper bug.
    //   • The fall-through `Lu::try_new_with_symbolic` failing is the
    //     A2 doubly-failed case (now reached via LM saturation OR the
    //     disabled short-circuit) — the Llt path tripped non-PD AND
    //     the LU path tripped a singular factor; not recoverable
    //     without a model-level change. Panic carries the context.
    #[allow(clippy::expect_used, clippy::panic)]
    fn factor_free_tangent(
        &self,
        triplets: &[Triplet<usize, usize, f64>],
        lm_state: &mut LmState,
        context: &str,
    ) -> FactoredFreeTangent {
        // Snapshot max_diag ONCE per call (not per retry) per spec §2.1.
        // The mass-diagonal scatter inside `assemble_free_hessian_
        // triplets` (search for `Mass diagonal:`) guarantees a positive
        // diagonal entry per free DOF — the orphan auto-pin in `new()`
        // excludes zero-mass DOFs from `free_dof_indices`; the
        // hand-built test fixtures in this module's `tests` mod also
        // include non-zero diagonal triplets. Both gates make
        // `max_diag > 0` a structural invariant.
        let max_diag = triplets_max_diag(triplets);
        debug_assert!(
            max_diag > 0.0,
            "factor_free_tangent: max diagonal entry must be positive (got {max_diag}); \
             empty triplets / zero-mass DOFs should have been excluded by new() validity gates"
        );

        // Snapshot λ at call start so the first-non-PD log can pick
        // the seed-vs-bump form per spec §2.5 logging policy.
        let lambda_at_call_start = lm_state.lambda();
        lm_state.begin_factor_call();

        loop {
            // λ == 0 → borrow the original triplets (bit-equal to pre-F3,
            // no allocation). λ > 0 → clone + mutate diagonal in place.
            let regularized: Cow<[Triplet<usize, usize, f64>]> = if lm_state.lambda() > 0.0 {
                Cow::Owned(triplets_with_diagonal_offset(triplets, lm_state.lambda()))
            } else {
                Cow::Borrowed(triplets)
            };
            let a_mat: SparseColMat<usize, f64> =
                SparseColMat::try_new_from_triplets(self.n_free, self.n_free, &regularized)
                    .expect("malformed condensed-tangent triplet list");
            match Llt::<usize, f64>::try_new_with_symbolic(
                self.symbolic.clone(),
                a_mat.rb(),
                Side::Lower,
            ) {
                Ok(llt) => {
                    // Post-retry success summary (only when we actually
                    // retried — disabled path has retry_count == 0
                    // and emits nothing, preserving pre-F3 stderr).
                    if lm_state.retry_count() > 0 {
                        eprintln!(
                            "sim-soft: LM converged in {} retries to λ = {:e} at {context}",
                            lm_state.retry_count(),
                            lm_state.lambda()
                        );
                    }
                    lm_state.on_llt_success();
                    return FactoredFreeTangent::symmetric(FactorInner::Llt(llt));
                }
                Err(LltError::Numeric(numeric_err)) if lm_state.can_bump(max_diag) => {
                    lm_state.on_non_pd(max_diag);
                    // First-non-PD per-call log (seed vs bump form
                    // determined by `lambda_at_call_start`). Subsequent
                    // retries within the same call are silent — the
                    // post-loop summary closes the trace.
                    if lm_state.retry_count() == 1 {
                        if lambda_at_call_start == 0.0 {
                            eprintln!(
                                "sim-soft: LM seeded λ = {:e} at {context} \
                                 (Llt non-PD pivot: {numeric_err:?})",
                                lm_state.lambda()
                            );
                        } else {
                            eprintln!(
                                "sim-soft: LM bumped λ = {:e} (was {:e}) at {context} \
                                 (Llt non-PD pivot: {numeric_err:?})",
                                lm_state.lambda(),
                                lambda_at_call_start
                            );
                        }
                    }
                    // Loop falls through to next iteration — explicit
                    // `continue` is clippy::needless_continue here.
                }
                Err(LltError::Numeric(numeric_err)) => {
                    // Saturated (retries exhausted OR λ at ceiling) OR
                    // LM disabled (can_bump false from the start). The
                    // LM-saturation summary fires only when LM is active;
                    // the underlying `faer LU fallback fired` log inside
                    // `lu_fallback` preserves the pre-F3 stderr surface.
                    if lm_state.is_active() {
                        eprintln!(
                            "sim-soft: LM saturated at λ = {:e} after {} retries at {context}; \
                             falling back to LU on un-regularized triplets",
                            lm_state.lambda(),
                            lm_state.retry_count()
                        );
                    }
                    return self.lu_fallback(triplets, numeric_err, context);
                }
                Err(LltError::Generic(faer_err)) => panic!(
                    "condensed tangent Llt factor failed at {context} with \
                     non-Numeric error: {faer_err:?} (symbolic pattern wrong or \
                     OutOfMemory — both should be impossible post-new())."
                ),
            }
        }
    }

    /// LU fallback path extracted from `factor_free_tangent` so the F3
    /// retry-loop saturation arm can dispatch to it on the ORIGINAL
    /// un-regularized triplets (per spec §2.1). Symmetrizes lower-tri
    /// → full pattern and factors via Lu against the cached
    /// `symbolic_lu`.
    ///
    /// `numeric_err` is forwarded from the Llt failure that triggered
    /// the fallback — included in the warn-log for debug parity with
    /// the pre-F3 single-attempt path. The warn-log surface is
    /// preserved exactly so pre-F3 stderr regressions stay bit-equal
    /// at LM-disabled call sites.
    //
    // expect_used + panic same justifications as `factor_free_tangent`'s
    // outer comment: pattern-build correctness validated at new().
    #[allow(clippy::expect_used, clippy::panic)]
    fn lu_fallback(
        &self,
        triplets: &[Triplet<usize, usize, f64>],
        numeric_err: faer::linalg::cholesky::llt::factor::LltError,
        context: &str,
    ) -> FactoredFreeTangent {
        eprintln!(
            "sim-soft: faer LU fallback fired at {context} \
             (Llt non-PD pivot: {numeric_err:?})"
        );
        let mut full_triplets: Vec<Triplet<usize, usize, f64>> =
            Vec::with_capacity(triplets.len() * 2);
        for t in triplets {
            full_triplets.push(Triplet::new(t.row, t.col, t.val));
            if t.row != t.col {
                full_triplets.push(Triplet::new(t.col, t.row, t.val));
            }
        }
        let a_mat_full: SparseColMat<usize, f64> =
            SparseColMat::try_new_from_triplets(self.n_free, self.n_free, &full_triplets)
                .expect("malformed full condensed-tangent triplet list");
        let lu = Lu::<usize, f64>::try_new_with_symbolic(self.symbolic_lu.clone(), a_mat_full.rb())
            .unwrap_or_else(|lu_err| {
                panic!(
                    "condensed tangent factor doubly failed at {context}: \
                         Llt tripped non-PD AND Lu fallback failed: {lu_err:?}. \
                         Tangent is degenerate beyond LU rescue — model-level \
                         change needed (looser validity domain, mesh \
                         refinement, or contact-geometry change)."
                )
            });
        FactoredFreeTangent::symmetric(FactorInner::Lu(Box::new(lu)))
    }

    /// Re-factor the free-DOF condensed tangent at a specific position
    /// (post-Newton-convergence for the IFT adjoint). Re-uses the
    /// cached symbolic factors — no rebuild. Returns either the
    /// happy-path Cholesky or the A2 LU fallback per
    /// [`Self::factor_free_tangent`].
    ///
    /// `lm_seed_lambda` carries the Newton-final λ from `solve_impl`
    /// per spec §2.1 — a fresh `LmState` is constructed from
    /// `self.config.lm_regularization` (disabled if `None`) seeded
    /// with this value, so late-iter LM activity that hadn't fully
    /// decayed warm-starts the adjoint factor. The adjoint runs at
    /// converged `x_final` which is typically SPD, so the expected λ
    /// is small-or-zero and LM typically does not fire here. On the
    /// disabled path, `lm_seed_lambda` is ignored (the sentinel
    /// state's λ stays permanently 0).
    fn factor_at_position(
        &self,
        x_curr: &[f64],
        x_prev: Option<&[f64]>,
        dt: f64,
        lm_seed_lambda: f64,
    ) -> FactoredFreeTangent {
        let mu = self.config.friction_mu;
        assert!(
            mu == 0.0 || x_prev.is_some(),
            "friction-exact gradient requested without x_prev: the friction adjoint needs the \
             step-start position xᵗ (= x_prev) to build the ∂λⁿ/∂x Woodbury correction. \
             Forward-mode sensitivities pass `Some(x_prev)`; reverse-mode tape VJPs stay \
             friction-free until the coupling leaf (PR3) threads x_prev through."
        );
        // A_sym includes the frozen-lag ∇²D when friction is active (the SAME symmetric
        // tangent the forward Newton step converged with); the asymmetric ∂λⁿ/∂x term is
        // added on top as a Woodbury correction below — now built at the drift-consistent
        // reference, so the adjoint A is exact under a moving collider (the drift guard PR3a
        // installed is retired; the coupling's drift-feedback edge is the remaining PR3b piece).
        let x_prev_for_hessian = if mu == 0.0 { None } else { x_prev };
        let triplets = self.assemble_free_hessian_triplets(x_curr, x_prev_for_hessian, dt);
        let mut lm_state = self
            .config
            .lm_regularization
            .map_or_else(LmState::disabled, |cfg| {
                LmState::from_config_with_seed(cfg, lm_seed_lambda)
            });
        let mut factor = self.factor_free_tangent(
            &triplets,
            &mut lm_state,
            "factor_at_position (IFT adjoint at x_final)",
        );
        if mu != 0.0 {
            if let Some(xp) = x_prev {
                factor.woodbury = self.assemble_friction_woodbury(x_curr, xp, dt, &factor);
            }
        }
        factor
    }

    /// The asymmetric friction adjoint as a [`WoodburyCorrection`] around the symmetric
    /// factor — the `∂λⁿ/∂x` normal-force coupling the frozen lag drops. Per active pair
    /// at contact vertex `v`: `aₚ = ∇D/λⁿ` (the friction force direction, at `v`'s DOFs)
    /// and `bₚ = ∂λⁿ/∂x = n̂ᵀ·(∇²E_contact)` (the row `Σ_cv blockᵀ·n̂` over the contact
    /// Hessian's `(v, cv)` blocks — for a plane only `(v, v)` is non-zero, but the sum
    /// stays correct for vertex–vertex contact). `Z = A_sym⁻¹ U` and `M = I + VᵀZ` are
    /// precomputed via the symmetric factor. Returns `None` when no pair is active.
    // similar_names: `zu_cols`/`zv_cols` (`A_sym⁻¹U` / `A_sym⁻¹V`) mirror `u_cols`/`v_cols`
    // by design — the Woodbury `U`/`V` factors and their symmetric back-solves.
    #[allow(clippy::similar_names)]
    fn assemble_friction_woodbury(
        &self,
        x_final: &[f64],
        x_prev: &[f64],
        dt: f64,
        factor: &FactoredFreeTangent,
    ) -> Option<WoodburyCorrection> {
        let mu = self.config.friction_mu;
        let w = dt * self.config.friction_eps_v;
        let positions = slice_to_vec3s(x_final);
        let pairs = self.contact.active_pairs(&self.mesh, &positions);
        let mut u_cols: Vec<Vec<f64>> = Vec::new();
        let mut v_cols: Vec<Vec<f64>> = Vec::new();
        for pair in &pairs {
            let grad = self.contact.gradient(pair, &positions);
            let hess = self.contact.hessian(pair, &positions);
            // Force-direction curvature `C = ∂n̂/∂x = sign(dE)·∇²sd` (0 for a plane) — feeds the
            // asymmetric curved-T tangent block below.
            let hess_n = self.contact.normal_curvature(pair, &positions);
            for (vid, force) in &grad.contributions {
                let lambda = force.norm();
                if lambda == 0.0 {
                    continue;
                }
                let v = *vid as usize;
                let x_v = positions[v];
                // Drift-consistent reference `xᵗ_eff = xᵗ + Δ_surf` — the SAME shift the forward
                // `friction_blocks` applies, so the Woodbury `aₚ = ∇D/λⁿ` is evaluated at the
                // config the forward solve converged with (the adjoint A stays exact under a
                // moving collider). `Δ_surf = 0` recovers the PR2 static-collider Woodbury.
                let x_start = crate::Vec3::new(x_prev[3 * v], x_prev[3 * v + 1], x_prev[3 * v + 2])
                    + self.friction_surface_drift;
                let (grad_d, _) =
                    crate::contact::friction::grad_hess(x_v, x_start, *force, lambda, mu, w);
                // aₚ = ∇D/λⁿ — the friction force direction. ∇D = μ·λⁿ·f₁·T·û is linear in
                // λⁿ, so ∂(∇D)/∂λⁿ = ∇D/λⁿ (the rank-1 column's left factor).
                let a_v = grad_d / lambda;
                let nhat = force / lambda;

                // Curved-normal tangent block `B = DN·C` (`∂(∇D_v)/∂x_v` via the rotating tangent
                // frame; `DN = ∂∇D/∂n̂`, `C = ∂n̂/∂x_v`) — ASYMMETRIC (the lagged friction force is
                // not a conservative gradient), so it cannot join the symmetric `A_sym` triplets;
                // it enters the adjoint tangent here as 3 rank-1 column pairs at v's DOFs,
                // `B = Σ_j B[:,j]·e_jᵀ`. `C = 0` for a plane ⇒ B = 0 ⇒ no columns ⇒ the plane's
                // Woodbury (k, M) is byte-identical.
                let dnh = crate::contact::friction::normal_rotation_term(
                    x_v, x_start, *force, lambda, mu, w,
                ) * hess_n;
                if dnh != Matrix3::zeros() {
                    for j in 0..3 {
                        let Some(fj) = self.full_to_free_idx[3 * v + j] else {
                            continue; // a pinned column DOF carries no free unknown
                        };
                        let mut uc = vec![0.0_f64; self.n_free];
                        for i in 0..3 {
                            if let Some(fi) = self.full_to_free_idx[3 * v + i] {
                                uc[fi] = dnh[(i, j)];
                            }
                        }
                        let mut vc = vec![0.0_f64; self.n_free];
                        vc[fj] = 1.0;
                        u_cols.push(uc);
                        v_cols.push(vc);
                    }
                }

                // U column: aₚ embedded at v's free DOFs.
                let mut u = vec![0.0_f64; self.n_free];
                for j in 0..3 {
                    if let Some(fi) = self.full_to_free_idx[3 * v + j] {
                        u[fi] = a_v[j];
                    }
                }
                // V column: ∂λⁿ_v/∂x = n̂ᵀ·Σ_cv block(v, cv), each placed at cv's free DOFs.
                let mut vv = vec![0.0_f64; self.n_free];
                for (rv, cv, block) in &hess.contributions {
                    if *rv as usize != v {
                        continue;
                    }
                    let row = block.transpose() * nhat; // (n̂ᵀ block)ᵀ at cv's DOFs
                    let c = *cv as usize;
                    for j in 0..3 {
                        if let Some(fi) = self.full_to_free_idx[3 * c + j] {
                            vv[fi] += row[j];
                        }
                    }
                }
                u_cols.push(u);
                v_cols.push(vv);
            }
        }

        let k = u_cols.len();
        if k == 0 {
            return None;
        }
        // Z = A_sym⁻¹ U and Zᵀ = A_sym⁻¹ V (one symmetric back-solve per column; Zᵀ
        // serves the transpose solve A⁻ᵀ that the reverse-mode tape needs).
        let base_solve = |c: &Vec<f64>| {
            let mut z = c.clone();
            factor.solve_base_in_place(&mut z);
            z
        };
        let zu_cols: Vec<Vec<f64>> = u_cols.iter().map(base_solve).collect();
        let zv_cols: Vec<Vec<f64>> = v_cols.iter().map(base_solve).collect();
        // M = I_k + Vᵀ Z.
        let mut m = DMatrix::<f64>::identity(k, k);
        for (i, v) in v_cols.iter().enumerate() {
            for (j, z) in zu_cols.iter().enumerate() {
                m[(i, j)] += v.iter().zip(z).map(|(a, b)| a * b).sum::<f64>();
            }
        }
        Some(WoodburyCorrection {
            u_cols,
            v_cols,
            zu_cols,
            zv_cols,
            m,
        })
    }

    /// Solve `A_free · δ = -r_free` via the cached symbolic factor.
    /// Returns the Newton step `δ_free` of length `self.n_free`.
    ///
    /// `triplets` must be the lower-triangle of `A_free` in `(col, row)`-
    /// sorted order; produced by `assemble_free_hessian_triplets`.
    /// `r_full` is the full-DOF residual; the free-DOF subset is
    /// gathered via `self.free_dof_indices`. The factor variant (Llt
    /// vs Lu) is encapsulated by [`FactoredFreeTangent`] and forwards
    /// the solve transparently. `lm_state` is threaded through to
    /// `factor_free_tangent`'s F3 retry loop (per spec §2.1).
    fn factor_and_solve_free(
        &self,
        triplets: &[Triplet<usize, usize, f64>],
        r_full: &[f64],
        newton_iter: usize,
        r_norm: f64,
        lm_state: &mut LmState,
    ) -> Vec<f64> {
        let context = format!(
            "factor_and_solve_free at Newton iter {newton_iter} \
             (free residual norm {r_norm:e})"
        );
        let factor = self.factor_free_tangent(triplets, lm_state, &context);
        // RHS = -r_free, gathered from r_full via the free-DOF index map.
        let mut rhs: Vec<f64> = self
            .free_dof_indices
            .iter()
            .map(|&idx| -r_full[idx])
            .collect();
        factor.solve_base_in_place(&mut rhs);
        rhs
    }

    /// Graceful-failure mirror of [`Self::lu_fallback`] (F3.3 per spec
    /// §2.5). On Lu's doubly-failed case, returns
    /// `Err(DoublyFailedFactorInfo)` carrying the full panic-message
    /// context string instead of panicking. The caller wraps into
    /// `SolverFailure::DoublyFailedFactor` with `x_partial` +
    /// `last_iter` from its own context.
    ///
    /// The `"sim-soft: faer LU fallback fired"` stderr line is
    /// preserved exactly — F3.3 changes the failure-reporting shape
    /// (panic vs Err) but does NOT change the diagnostic-log surface
    /// at successful Lu fallbacks.
    //
    // expect_used: pattern-build correctness validated at new() —
    // same justification as `lu_fallback`'s expect on
    // SparseColMat::try_new_from_triplets.
    #[allow(clippy::expect_used)]
    fn try_lu_fallback(
        &self,
        triplets: &[Triplet<usize, usize, f64>],
        numeric_err: faer::linalg::cholesky::llt::factor::LltError,
        context: &str,
    ) -> Result<FactoredFreeTangent, DoublyFailedFactorInfo> {
        eprintln!(
            "sim-soft: faer LU fallback fired at {context} \
             (Llt non-PD pivot: {numeric_err:?})"
        );
        let mut full_triplets: Vec<Triplet<usize, usize, f64>> =
            Vec::with_capacity(triplets.len() * 2);
        for t in triplets {
            full_triplets.push(Triplet::new(t.row, t.col, t.val));
            if t.row != t.col {
                full_triplets.push(Triplet::new(t.col, t.row, t.val));
            }
        }
        let a_mat_full: SparseColMat<usize, f64> =
            SparseColMat::try_new_from_triplets(self.n_free, self.n_free, &full_triplets)
                .expect("malformed full condensed-tangent triplet list");
        match Lu::<usize, f64>::try_new_with_symbolic(self.symbolic_lu.clone(), a_mat_full.rb()) {
            Ok(lu) => Ok(FactoredFreeTangent::symmetric(FactorInner::Lu(Box::new(
                lu,
            )))),
            Err(lu_err) => Err(DoublyFailedFactorInfo {
                context: format!(
                    "condensed tangent factor doubly failed at {context}: \
                     Llt tripped non-PD AND Lu fallback failed: {lu_err:?}. \
                     Tangent is degenerate beyond LU rescue — model-level \
                     change needed (looser validity domain, mesh \
                     refinement, or contact-geometry change)."
                ),
            }),
        }
    }

    /// Graceful-failure mirror of [`Self::factor_free_tangent`] (F3.3
    /// per spec §2.5). Same retry loop + saturation handoff, but the
    /// saturation arm dispatches to [`Self::try_lu_fallback`] and
    /// propagates `Err` on doubly-failed factor.
    ///
    /// The factor-loop body is structurally identical to the panicking
    /// `factor_free_tangent` — duplication is intentional for clarity
    /// and to keep the panicking variant's hot path free of `Result`
    /// indirection. The two methods diverge only at the LU fallback
    /// dispatch.
    ///
    /// **MAINTENANCE NOTE**: see [`Self::factor_free_tangent`]'s
    /// maintenance note. Any change to the retry-loop body
    /// (seed/bump rules, log forms, the Cow regularization branch)
    /// MUST be mirrored from the other direction too.
    //
    // expect_used + panic justifications: same as
    // `factor_free_tangent`'s outer comment.
    #[allow(clippy::expect_used, clippy::panic)]
    fn try_factor_free_tangent(
        &self,
        triplets: &[Triplet<usize, usize, f64>],
        lm_state: &mut LmState,
        context: &str,
    ) -> Result<FactoredFreeTangent, DoublyFailedFactorInfo> {
        let max_diag = triplets_max_diag(triplets);
        debug_assert!(
            max_diag > 0.0,
            "try_factor_free_tangent: max diagonal entry must be positive (got {max_diag})"
        );

        let lambda_at_call_start = lm_state.lambda();
        lm_state.begin_factor_call();

        loop {
            let regularized: Cow<[Triplet<usize, usize, f64>]> = if lm_state.lambda() > 0.0 {
                Cow::Owned(triplets_with_diagonal_offset(triplets, lm_state.lambda()))
            } else {
                Cow::Borrowed(triplets)
            };
            let a_mat: SparseColMat<usize, f64> =
                SparseColMat::try_new_from_triplets(self.n_free, self.n_free, &regularized)
                    .expect("malformed condensed-tangent triplet list");
            match Llt::<usize, f64>::try_new_with_symbolic(
                self.symbolic.clone(),
                a_mat.rb(),
                Side::Lower,
            ) {
                Ok(llt) => {
                    if lm_state.retry_count() > 0 {
                        eprintln!(
                            "sim-soft: LM converged in {} retries to λ = {:e} at {context}",
                            lm_state.retry_count(),
                            lm_state.lambda()
                        );
                    }
                    lm_state.on_llt_success();
                    return Ok(FactoredFreeTangent::symmetric(FactorInner::Llt(llt)));
                }
                Err(LltError::Numeric(numeric_err)) if lm_state.can_bump(max_diag) => {
                    lm_state.on_non_pd(max_diag);
                    if lm_state.retry_count() == 1 {
                        if lambda_at_call_start == 0.0 {
                            eprintln!(
                                "sim-soft: LM seeded λ = {:e} at {context} \
                                 (Llt non-PD pivot: {numeric_err:?})",
                                lm_state.lambda()
                            );
                        } else {
                            eprintln!(
                                "sim-soft: LM bumped λ = {:e} (was {:e}) at {context} \
                                 (Llt non-PD pivot: {numeric_err:?})",
                                lm_state.lambda(),
                                lambda_at_call_start
                            );
                        }
                    }
                }
                Err(LltError::Numeric(numeric_err)) => {
                    if lm_state.is_active() {
                        eprintln!(
                            "sim-soft: LM saturated at λ = {:e} after {} retries at {context}; \
                             falling back to LU on un-regularized triplets",
                            lm_state.lambda(),
                            lm_state.retry_count()
                        );
                    }
                    return self.try_lu_fallback(triplets, numeric_err, context);
                }
                Err(LltError::Generic(faer_err)) => panic!(
                    "condensed tangent Llt factor failed at {context} with \
                     non-Numeric error: {faer_err:?} (symbolic pattern wrong or \
                     OutOfMemory — both should be impossible post-new())."
                ),
            }
        }
    }

    /// Graceful-failure mirror of [`Self::factor_and_solve_free`]
    /// (F3.3 per spec §2.5). Routes through
    /// [`Self::try_factor_free_tangent`] and propagates
    /// `DoublyFailedFactorInfo` on factor failure; the solve path
    /// itself is infallible (factor success guarantees a valid solve).
    fn try_factor_and_solve_free(
        &self,
        triplets: &[Triplet<usize, usize, f64>],
        r_full: &[f64],
        newton_iter: usize,
        r_norm: f64,
        lm_state: &mut LmState,
    ) -> Result<Vec<f64>, DoublyFailedFactorInfo> {
        let context = format!(
            "factor_and_solve_free at Newton iter {newton_iter} \
             (free residual norm {r_norm:e})"
        );
        let factor = self.try_factor_free_tangent(triplets, lm_state, &context)?;
        let mut rhs: Vec<f64> = self
            .free_dof_indices
            .iter()
            .map(|&idx| -r_full[idx])
            .collect();
        factor.solve_base_in_place(&mut rhs);
        Ok(rhs)
    }

    /// Push `NewtonStepVjp` onto the tape with `theta_var` as parent.
    /// Shared by [`Solver::step`] and [`Solver::try_step`] — both must
    /// register the same VJP shape on the Ok-path so downstream
    /// tape consumers (`Tape::backward`) see no API difference based
    /// on which entry point built the step.
    ///
    /// The VJP owns the factor; `Tape::backward` feeds the
    /// scalar-or-vector cotangent of `x_final` into `vjp` and we solve
    /// the adjoint `A · λ = g_free` in place, contracting against
    /// `(∂r/∂θ)_free` per the per-stage closed forms in
    /// [`NewtonStepVjp::vjp`](crate::differentiable::NewtonStepVjp).
    /// Pre-resolves loaded vertices' xyz free-DOF indices via
    /// `full_to_free_idx` so `vjp` doesn't need solver-side metadata
    /// at backward-pass time.
    //
    // expect_used: the loaded_free_xyz construction `.expect`s on
    // `full_to_free_idx[loaded_dof]`, which BC validation guarantees
    // is `Some` (loaded ∩ pinned = ∅ asserted in `new()`).
    #[allow(clippy::expect_used)]
    fn push_newton_step_vjp(
        &self,
        tape: &mut CpuTape,
        theta_var: Var,
        step: &mut NewtonStep<CpuTape>,
        factor: FactoredFreeTangent,
    ) {
        let loaded_free_xyz: Vec<[usize; 3]> = self
            .boundary_conditions
            .loaded_vertices
            .iter()
            .map(|&(vid, _)| {
                let v = vid as usize;
                [
                    self.full_to_free_idx[3 * v]
                        .expect("loaded vertex must be free (BC validation)"),
                    self.full_to_free_idx[3 * v + 1]
                        .expect("loaded vertex must be free (BC validation)"),
                    self.full_to_free_idx[3 * v + 2]
                        .expect("loaded vertex must be free (BC validation)"),
                ]
            })
            .collect();
        let stage_1 = self
            .boundary_conditions
            .loaded_vertices
            .iter()
            .all(|(_, ax)| matches!(ax, LoadAxis::AxisZ));
        let vjp = NewtonStepVjp::new(
            factor,
            self.n_dof,
            self.free_dof_indices.clone(),
            loaded_free_xyz,
            stage_1,
        );
        let x_final_tensor = Tensor::from_slice(&step.x_final, &[self.n_dof]);
        let x_final_var = tape.push_custom(&[theta_var], x_final_tensor, Box::new(vjp));
        step.x_final_var = Some(x_final_var);
    }

    /// Graceful-failure mirror of [`Self::factor_at_position`] (F3.3
    /// per spec §2.5). Used by [`Self::try_step`] for the IFT-adjoint
    /// factor at converged `x_final`. On doubly-failed factor returns
    /// `Err(SolverFailure::DoublyFailedFactor)` with
    /// `x_partial = x_curr` (which IS `x_final` here — the natural
    /// partial position) and `last_iter = 0` (no Newton iter applies
    /// post-Newton; the `context` string carries the
    /// `"factor_at_position (IFT adjoint at x_final)"` site tag for
    /// disambiguation).
    fn try_factor_at_position(
        &self,
        x_curr: &[f64],
        x_prev: Option<&[f64]>,
        dt: f64,
        lm_seed_lambda: f64,
    ) -> Result<FactoredFreeTangent, SolverFailure> {
        let mu = self.config.friction_mu;
        assert!(
            mu == 0.0 || x_prev.is_some(),
            "friction-exact gradient requested without x_prev: the friction adjoint needs the \
             step-start position xᵗ (= x_prev) to build the ∂λⁿ/∂x Woodbury correction. \
             Forward-mode sensitivities pass `Some(x_prev)`; reverse-mode tape VJPs stay \
             friction-free until the coupling leaf (PR3) threads x_prev through."
        );
        // Drift-consistent Woodbury ⇒ the adjoint is exact under a moving collider; the PR3a
        // drift guard is retired (see `factor_at_position`).
        let x_prev_for_hessian = if mu == 0.0 { None } else { x_prev };
        let triplets = self.assemble_free_hessian_triplets(x_curr, x_prev_for_hessian, dt);
        let mut lm_state = self
            .config
            .lm_regularization
            .map_or_else(LmState::disabled, |cfg| {
                LmState::from_config_with_seed(cfg, lm_seed_lambda)
            });
        let mut factor = self
            .try_factor_free_tangent(
                &triplets,
                &mut lm_state,
                "factor_at_position (IFT adjoint at x_final)",
            )
            .map_err(|info| SolverFailure::DoublyFailedFactor {
                x_partial: x_curr.to_vec(),
                last_iter: 0,
                context: info.context,
            })?;
        if mu != 0.0 {
            if let Some(xp) = x_prev {
                factor.woodbury = self.assemble_friction_woodbury(x_curr, xp, dt, &factor);
            }
        }
        Ok(factor)
    }

    /// Inner solver: pure-function-of-θ Newton loop. Shared by `step`
    /// and `replay_step`.
    ///
    /// F3.3: thin panic-on-failure wrapper around [`Self::try_solve_impl`].
    /// All three F3 failure surfaces (`SolverFailure::ArmijoStall` /
    /// `NewtonIterCap` / `DoublyFailedFactor`) panic here with pre-F3
    /// messages preserved bit-equal — the source-level body differs
    /// from pre-F3 but observable behavior at this method's surface
    /// is unchanged. Graceful-failure consumers use `try_step` /
    /// `try_replay_step` which route through `try_solve_impl`
    /// directly.
    ///
    /// # Panics
    /// - Newton exceeds `config.max_newton_iter` iterations without
    ///   reaching `config.tol`.
    /// - Armijo backtracks exceed `config.max_line_search_backtracks`.
    /// - Tangent factorization fails doubly (Llt tripped non-PD AND
    ///   the A2 Lu fallback also failed — see
    ///   `Self::factor_free_tangent`). This is a model-level
    ///   degeneracy, not runtime-recoverable.
    /// - A tet leaves the material's validity domain (Decision Q —
    ///   over-stretch / inversion), at step start or at the converged
    ///   state. (`try_step`/`try_replay_step` return this as
    ///   `SolverFailure::ValidityViolation` instead.)
    //
    // panic: scope §3 R-1 (3-5-iter convergence prediction) cap +
    // Armijo cap are book-level findings, not runtime-recoverable
    // conditions at the `step()` API surface. F3.3 widens the
    // recoverable surface via `try_step` without changing `step`'s
    // panic contract.
    #[allow(clippy::panic)]
    fn solve_impl(
        &self,
        x_prev: &Tensor<f64>,
        v_prev: &Tensor<f64>,
        theta: &Tensor<f64>,
        dt: f64,
    ) -> (NewtonStep<CpuTape>, f64) {
        self.try_solve_impl(x_prev, v_prev, theta, dt)
            .unwrap_or_else(|fail| match fail {
                SolverFailure::ArmijoStall {
                    last_iter,
                    last_r_norm,
                    ..
                } => panic!("{}", armijo_stall_panic_message(last_iter, last_r_norm)),
                SolverFailure::NewtonIterCap { max_iter, .. } => panic!(
                    "Newton failed to converge within {max_iter} iterations at \
                     tol {tol:e}. Likely causes: θ drives system out of R-2's \
                     SPD region, or spec §3 R-1's assumption of 3-5 iter \
                     convergence from zero initial guess is wrong for this θ.",
                    tol = self.config.tol,
                ),
                SolverFailure::DoublyFailedFactor { context, .. } => panic!("{context}"),
                // Decision Q fail-closed: `step`/`replay_step` re-panic with the verbatim
                // validity message (the `try_` path returns it as `Err` instead).
                SolverFailure::ValidityViolation { message, .. } => panic!("{message}"),
            })
    }

    /// F3 recon candidate A — gated factor + solve + Armijo for one
    /// Newton iter (per `docs/F3_RECON_A_GATED_LM_SPEC.md` §2.2 + §2.3).
    /// Used exclusively by [`Self::try_solve_impl`].
    ///
    /// **First pass** (always): factor + solve with `LmState::disabled`
    /// → `δ_LU`; [`Self::armijo_backtrack`] on `δ_LU`. Bit-equal to
    /// pre-F3 LU + Armijo.
    ///
    /// **First pass succeeds** → return the Armijo-accepted iterate.
    ///
    /// **First pass fails AND outer `lm_state` is disabled** (§2.3
    /// short-circuit): return the first-pass failure directly. NO
    /// second factor + solve attempted — preserves the F3 spec's §F3.1
    /// bit-equal-when-dormant contract (no 2× factor wall-clock cost
    /// at LM-disabled stalls).
    ///
    /// **First pass fails AND outer `lm_state` is active** (§2.2
    /// ESCALATION): re-factor + re-solve using the outer persistent
    /// `lm_state` (which carries cross-Newton-iter λ per F3 spec §2.2).
    /// The first non-PD detection at this iter seeds-or-bumps λ via the
    /// inner `factor_free_tangent` retry loop. The LM-rescued `δ_LM`
    /// goes through `armijo_backtrack`; if Armijo accepts → return; if
    /// the LM step ALSO Armijo-stalls → return
    /// `Err(SolverFailure::ArmijoStall)` (no further escalation —
    /// already in the LM-rescue regime per §2.2 step 4b.ii).
    ///
    /// First-pass failure info (`DoublyFailedFactor` context,
    /// `ArmijoStall` `r_norm`) is DISCARDED on escalation per spec
    /// §2.5 ("the LU step already had its chance; escalation is the
    /// recovery attempt; if escalation also fails, THAT's the
    /// committed-iterate failure for the surfaced `SolverFailure`
    /// variant").
    //
    // too_many_arguments: 10 inputs mirror the residual + Newton-iter
    // formula's reads; bundling into a struct adds name-the-fields
    // ceremony for a sole caller (try_solve_impl) with no readability
    // gain (try_solve_impl already names its locals identically).
    #[allow(clippy::too_many_arguments)]
    fn try_gated_factor_solve_armijo(
        &self,
        triplets: &[Triplet<usize, usize, f64>],
        r_full: &[f64],
        x_curr: &[f64],
        x_prev_vec: &[f64],
        v_prev_vec: &[f64],
        f_ext: &[f64],
        dt: f64,
        r_norm: f64,
        newton_iter: usize,
        lm_state: &mut LmState,
    ) -> Result<Vec<f64>, SolverFailure> {
        // First pass: LM-disabled (bit-equal to pre-F3).
        let mut lm_disabled = LmState::disabled();
        let first_pass_outcome = self
            .try_factor_and_solve_free(triplets, r_full, newton_iter, r_norm, &mut lm_disabled)
            .map_err(|info| SolverFailure::DoublyFailedFactor {
                x_partial: x_curr.to_vec(),
                last_iter: newton_iter,
                context: info.context,
            })
            .and_then(|delta_lu| {
                self.armijo_backtrack(
                    x_curr,
                    x_prev_vec,
                    v_prev_vec,
                    f_ext,
                    dt,
                    &delta_lu,
                    r_norm,
                    newton_iter,
                )
                .map_err(|stall| SolverFailure::ArmijoStall {
                    x_partial: stall.x_curr,
                    last_iter: stall.iter,
                    last_r_norm: stall.r_norm,
                })
            });

        match first_pass_outcome {
            Ok(x_accepted) => Ok(x_accepted),
            Err(failure) if !lm_state.is_active() => {
                // §2.3 short-circuit: no LM rescue mechanism available.
                Err(failure)
            }
            Err(_first_pass_failure_discarded) => {
                // §2.2 ESCALATION via the OUTER persistent lm_state.
                let delta_lm = self
                    .try_factor_and_solve_free(triplets, r_full, newton_iter, r_norm, lm_state)
                    .map_err(|info| SolverFailure::DoublyFailedFactor {
                        x_partial: x_curr.to_vec(),
                        last_iter: newton_iter,
                        context: info.context,
                    })?;
                self.armijo_backtrack(
                    x_curr,
                    x_prev_vec,
                    v_prev_vec,
                    f_ext,
                    dt,
                    &delta_lm,
                    r_norm,
                    newton_iter,
                )
                .map_err(|stall| SolverFailure::ArmijoStall {
                    x_partial: stall.x_curr,
                    last_iter: stall.iter,
                    last_r_norm: stall.r_norm,
                })
            }
        }
    }

    /// Graceful-failure counterpart to [`Self::solve_impl`] (F3.3 per
    /// `docs/F3_LM_REGULARIZATION_SPEC.md` §2.5). Same Newton loop,
    /// but returns `Result<(NewtonStep, f64), SolverFailure>` instead
    /// of panicking. The `f64` is the Newton-final λ (threaded into
    /// the IFT-adjoint factor by [`Self::try_step`] per spec §2.1).
    ///
    /// `x_partial` on each `SolverFailure` variant is `x_curr` at the
    /// START of the failed Newton iter (per spec §2.5 `ArmijoStall`
    /// docstring — committed-Newton-iterate semantics, NOT `trial_x`
    /// from Armijo backtracking, NOT a partial-Armijo-accepted `x`).
    /// `NewtonIterCap`'s `x_partial` is the most-recent
    /// armijo-accepted iterate (one full iter past the last failed
    /// convergence check).
    ///
    /// **F3 recon candidate A — gated LM activation** (per
    /// `docs/F3_RECON_A_GATED_LM_SPEC.md`). At each Newton iter, the
    /// inner factor + solve + Armijo is attempted TWICE in the failure
    /// path: first with LM SUPPRESSED (bit-equal to pre-F3 LU + Armijo
    /// — the cavity = 3 mm baseline preservation lever); on first-pass
    /// failure AND `lm_state.is_active()`, escalate to the persistent
    /// outer `lm_state`'s LM-rescued retry. At LM-disabled
    /// (`SolverConfig::lm_regularization == None`), the §2.3
    /// short-circuit returns the first-pass failure directly to
    /// preserve the F3 spec's §F3.1 bit-equal-when-dormant contract
    /// (no 2× factor cost at LM-disabled stalls). The OUTER
    /// `lm_state` carries cross-iter λ per F3 spec §2.2 persistence
    /// rule — only consumed by the escalation branch.
    fn try_solve_impl(
        &self,
        x_prev: &Tensor<f64>,
        v_prev: &Tensor<f64>,
        theta: &Tensor<f64>,
        dt: f64,
    ) -> Result<(NewtonStep<CpuTape>, f64), SolverFailure> {
        assert!(
            x_prev.as_slice().len() == self.n_dof,
            "x_prev must have {} entries, got {}",
            self.n_dof,
            x_prev.as_slice().len(),
        );
        assert!(
            v_prev.as_slice().len() == self.n_dof,
            "v_prev must have {} entries, got {}",
            self.n_dof,
            v_prev.as_slice().len(),
        );
        assert!(dt > 0.0, "dt must be positive, got {dt}");

        let mut x_curr: Vec<f64> = x_prev.as_slice().to_vec();
        let x_prev_vec: Vec<f64> = x_prev.as_slice().to_vec();
        let v_prev_vec: Vec<f64> = v_prev.as_slice().to_vec();

        let mut f_ext = vec![0.0; self.n_dof];
        self.assemble_external_force(theta, &mut f_ext);

        // Decision Q validity check (step-start boundary): returns
        // `SolverFailure::ValidityViolation` on a tet leaving the
        // validity domain. `try_solve_impl` propagates it via `?` so
        // the `try_` path surfaces it as `Err` (a feasibility-aware
        // co-design caller skips the design); `solve_impl`/`step`
        // re-panic the verbatim Decision Q message (fail-closed
        // contract unchanged).
        self.check_validity_at_step_start(&x_curr)?;

        let mut f_int = vec![0.0; self.n_dof];
        let mut r_full = vec![0.0; self.n_dof];

        let mut lm_state = self
            .config
            .lm_regularization
            .map_or_else(LmState::disabled, LmState::from_config);

        let mut last_r_norm = 0.0_f64;

        for newton_iter in 0..self.config.max_newton_iter {
            self.assemble_global_int_force(&x_curr, &x_prev_vec, dt, &mut f_int);
            residual_into(
                &x_curr,
                &x_prev_vec,
                &v_prev_vec,
                &f_int,
                &f_ext,
                &self.mass_per_dof,
                dt,
                &mut r_full,
            );
            let r_norm = self.free_residual_norm(&r_full);
            last_r_norm = r_norm;

            if r_norm < self.config.tol {
                // Decision Q validity check at converged state — sister
                // of the step-start check at line 1525. Without this,
                // Newton can converge to a deformation field where some
                // tet's F violates max_stretch_deviation / inversion;
                // the failure surfaces only at the NEXT step's start
                // check, masking which step actually produced the
                // invalid state + leaving the failed step's recorded
                // output physically meaningless. See
                // `check_validity_at_step_start`'s docstring for the
                // motivating finding + the two-boundary check contract.
                self.check_validity_at_step_start(&x_curr)?;
                return Ok((
                    NewtonStep::new_converged(x_curr, newton_iter, r_norm),
                    lm_state.lambda(),
                ));
            }

            let triplets = self.assemble_free_hessian_triplets(&x_curr, Some(&x_prev_vec), dt);
            x_curr = self.try_gated_factor_solve_armijo(
                &triplets,
                &r_full,
                &x_curr,
                &x_prev_vec,
                &v_prev_vec,
                &f_ext,
                dt,
                r_norm,
                newton_iter,
                &mut lm_state,
            )?;
        }

        Err(SolverFailure::NewtonIterCap {
            x_partial: x_curr,
            max_iter: self.config.max_newton_iter,
            last_r_norm,
        })
    }

    /// Armijo backtracking per scope §5 R-1: shrink α geometrically
    /// until `‖r(x + α δ)‖ ≤ (1 - c₁ α) ‖r‖`. Updates only the free
    /// DOFs of `x` (looked up via `self.free_dof_indices`); pinned
    /// DOFs stay at their `x_curr` values.
    ///
    /// F3.3: returns `Result<Vec<f64>, ArmijoStallInfo>` so callers
    /// can choose their own failure response. `solve_impl` (the
    /// existing panic-on-stall consumer) translates the local
    /// `ArmijoStallInfo` to a `panic!` with the pre-F3 message
    /// preserved bit-equal; `try_solve_impl` (the new
    /// graceful-failure consumer) maps it into the public
    /// [`SolverFailure::ArmijoStall`] variant.
    //
    // too_many_arguments: 8 inputs mirror the residual formula's
    // reads; bundling into a struct adds name-the-fields ceremony for
    // two callers with no readability gain.
    #[allow(clippy::too_many_arguments)]
    fn armijo_backtrack(
        &self,
        x_curr: &[f64],
        x_prev_vec: &[f64],
        v_prev_vec: &[f64],
        f_ext: &[f64],
        dt: f64,
        delta_free: &[f64],
        r_norm: f64,
        newton_iter: usize,
    ) -> Result<Vec<f64>, ArmijoStallInfo> {
        debug_assert!(x_curr.len() == self.n_dof);
        debug_assert!(delta_free.len() == self.n_free);
        let mut alpha = 1.0;
        // trial_x is initialized from x_curr each iteration via the free-DOF
        // re-write below; pre-allocate the buffer once and reuse across
        // backtracks.
        let mut trial_x: Vec<f64> = x_curr.to_vec();
        let mut trial_f_int = vec![0.0; self.n_dof];
        let mut trial_r = vec![0.0; self.n_dof];

        for _ in 0..=self.config.max_line_search_backtracks {
            // Reset free DOFs to `x_curr + α · δ`; pinned DOFs are
            // unchanged from the initial copy.
            for (free_idx, &full_idx) in self.free_dof_indices.iter().enumerate() {
                trial_x[full_idx] = x_curr[full_idx] + alpha * delta_free[free_idx];
            }
            self.assemble_global_int_force(&trial_x, x_prev_vec, dt, &mut trial_f_int);
            residual_into(
                &trial_x,
                x_prev_vec,
                v_prev_vec,
                &trial_f_int,
                f_ext,
                &self.mass_per_dof,
                dt,
                &mut trial_r,
            );
            let trial_norm = self.free_residual_norm(&trial_r);
            if trial_norm <= alpha.mul_add(-ARMIJO_C1, 1.0) * r_norm {
                return Ok(trial_x);
            }
            alpha *= 0.5;
        }
        Err(ArmijoStallInfo {
            x_curr: x_curr.to_vec(),
            iter: newton_iter,
            r_norm,
        })
    }

    // ── Cache-using assembly methods (live since Phase 2 commit 4b). ──

    /// Read θ via the load map and emit the global external-force
    /// vector (Decision L). All-`AxisZ` BC (Stage 1) treats `theta` as
    /// a length-1 magnitude broadcast to every loaded vertex's `+ẑ`
    /// DOF; all-`FullVector` BC (Stage 2) consumes `theta` as
    /// `[3 * n_loaded]` per-vertex traction triples. Mixed-axis
    /// scenes are out of Phase 2 scope per `LoadAxis` doc.
    ///
    /// `config.gravity_z` adds a body-force `m_v · gravity_z` to every
    /// vertex's `+ẑ` DOF after the θ scatter. Default `gravity_z = 0`
    /// short-circuits the loop — bit-equal regression on the
    /// passthrough / compressive-block / Hertzian fixtures (which
    /// leave `gravity_z = 0`).
    //
    // Shape mismatches (BC contradicting θ length) panic — programmer
    // bug at scene wiring time, not runtime input.
    #[allow(clippy::panic)]
    fn assemble_external_force(&self, theta: &Tensor<f64>, f_ext: &mut [f64]) {
        debug_assert!(
            f_ext.len() == self.n_dof,
            "f_ext output buffer length {} ≠ n_dof {}",
            f_ext.len(),
            self.n_dof,
        );
        f_ext.fill(0.0);
        let theta_slice = theta.as_slice();
        let loaded = &self.boundary_conditions.loaded_vertices;

        if loaded.is_empty() {
            assert!(
                theta_slice.is_empty(),
                "θ has length {} but BC has no loaded vertices",
                theta_slice.len(),
            );
        } else {
            let all_axis_z = loaded.iter().all(|(_, ax)| matches!(ax, LoadAxis::AxisZ));
            let all_full_vec = loaded
                .iter()
                .all(|(_, ax)| matches!(ax, LoadAxis::FullVector));
            assert!(
                all_axis_z || all_full_vec,
                "Mixed-axis loaded_vertices are out of Phase 2 scope; got {loaded:?}"
            );

            if all_axis_z {
                // Stage-1 broadcast: one θ scalar drives `+ẑ` on every loaded vertex.
                assert!(
                    theta_slice.len() == 1,
                    "AxisZ-loaded θ must have length 1 (broadcast magnitude), got {}",
                    theta_slice.len(),
                );
                let mag = theta_slice[0];
                for &(vertex_id, _) in loaded {
                    f_ext[3 * vertex_id as usize + 2] = mag;
                }
            } else {
                // Stage-2 per-vertex: θ supplies xyz for each loaded vertex in order.
                let expected = 3 * loaded.len();
                assert!(
                    theta_slice.len() == expected,
                    "FullVector-loaded θ must have length {expected} (3 × {}), got {}",
                    loaded.len(),
                    theta_slice.len(),
                );
                for (i, &(vertex_id, _)) in loaded.iter().enumerate() {
                    let v = vertex_id as usize;
                    f_ext[3 * v] = theta_slice[3 * i];
                    f_ext[3 * v + 1] = theta_slice[3 * i + 1];
                    f_ext[3 * v + 2] = theta_slice[3 * i + 2];
                }
            }
        }

        // Gravity body force `f_z += m_v · g_z` per vertex. The
        // exact-zero short-circuit preserves bit-equality on the
        // regression net (the passthrough / compressive-block /
        // Hertzian fixtures leave `gravity_z = 0`). For loaded AxisZ
        // vertices the gravity term adds onto the θ-traction; orphan
        // vertices are auto-pinned at `new()`'s `effective_pinned`
        // step so their zero `mass_per_dof` never reaches a free DOF.
        if self.config.gravity_z != 0.0 {
            let n_vertices = self.n_dof / 3;
            for v in 0..n_vertices {
                f_ext[3 * v + 2] += self.mass_per_dof[3 * v + 2] * self.config.gravity_z;
            }
        }
    }

    /// Walk every element, scatter per-vertex internal-force
    /// contributions into the global `f_int` buffer.
    ///
    /// `f_int` is zeroed inside; caller need not pre-clear. Reads
    /// `x_curr` (length `n_dof`), cached `element_geometries`, the
    /// per-tet `M` from `self.mesh.materials()` (Phase 4 commit 5 —
    /// Newton hot path reads from the per-tet material cache per
    /// Part 7 §02 §00), and `self.contact` for active-pair gradient
    /// contributions (Phase 5 commit 5; scope memo Decision H —
    /// per-iter active-pair recompute).
    //
    // Lint allows: see assemble_free_hessian_triplets justification.
    #[allow(clippy::cast_possible_truncation, clippy::needless_range_loop)]
    fn assemble_global_int_force(
        &self,
        x_curr: &[f64],
        x_prev: &[f64],
        dt: f64,
        f_int: &mut [f64],
    ) {
        debug_assert!(x_curr.len() == self.n_dof);
        debug_assert!(f_int.len() == self.n_dof);
        f_int.fill(0.0);
        let materials = self.mesh.materials();
        for (tet_id, geom) in self.element_geometries.iter().enumerate() {
            let verts = self.mesh.tet_vertices(tet_id as TetId);
            let x_elem = extract_element_dof_values(x_curr, &verts);
            let f = deformation_gradient(&x_elem, &geom.grad_x_n);
            let piola = materials[tet_id].first_piola(&f);
            // Per-vertex internal-force contribution `V · P · grad_X N_a`.
            for a in 0..4 {
                let v = verts[a] as usize;
                for i in 0..3 {
                    let mut sum = 0.0;
                    for j in 0..3 {
                        sum += piola[(i, j)] * geom.grad_x_n[(a, j)];
                    }
                    f_int[3 * v + i] += geom.volume * sum;
                }
            }
        }

        // Contact gradient contributions. Sign convention per scope
        // memo §6 R-5 lens (v): the gradient `∂E_pen/∂x` is scattered
        // as `+force` into f_int, mirroring the elastic case where
        // `∂Ψ_elastic/∂x` is the +`f_int` contribution above.
        // `NullContact` returns an empty contributions Vec → empty
        // for-loops → bit-equal to the pre-Phase-5 elastic-only path.
        let positions = slice_to_vec3s(x_curr);
        let pairs = self.contact.active_pairs(&self.mesh, &positions);
        for pair in &pairs {
            let g = self.contact.gradient(pair, &positions);
            for &(vid, force) in &g.contributions {
                let v = vid as usize;
                f_int[3 * v] += force.x;
                f_int[3 * v + 1] += force.y;
                f_int[3 * v + 2] += force.z;
            }
        }

        // Smoothed-Coulomb friction gradient `∇D` (the dissipative tangential force),
        // scattered like the contact force. Empty when frictionless ⇒ bit-equal.
        for (v, grad, _) in self.friction_blocks(x_curr, x_prev, dt) {
            f_int[3 * v] += grad.x;
            f_int[3 * v + 1] += grad.y;
            f_int[3 * v + 2] += grad.z;
        }
    }

    /// Per-active-pair smoothed-Coulomb friction `(vertex, ∇D, ∇²D)` at the current
    /// configuration, with the lagged `(normal, λⁿ)` read from the contact's own gradient
    /// (the contact-energy gradient `force = ∇E_contact` has magnitude `λⁿ` and lies along
    /// `±n̂`; the tangent plane is sign-agnostic). `xᵗ = x_prev` (the step start), `w =
    /// dt·ε_v`. Empty when `friction_mu == 0` (the frictionless short-circuit) or no pairs.
    /// Shared by the forward residual and Hessian assemblies so the two stay in lockstep.
    fn friction_blocks(
        &self,
        x_curr: &[f64],
        x_prev: &[f64],
        dt: f64,
    ) -> Vec<(usize, crate::Vec3, nalgebra::Matrix3<f64>)> {
        let mu = self.config.friction_mu;
        if mu == 0.0 {
            return Vec::new();
        }
        let w = dt * self.config.friction_eps_v;
        let positions = slice_to_vec3s(x_curr);
        let pairs = self.contact.active_pairs(&self.mesh, &positions);
        let mut out = Vec::new();
        for pair in &pairs {
            for (vid, force) in self.contact.gradient(pair, &positions).contributions {
                let lambda = force.norm(); // |∇E_contact| = the normal-force magnitude λⁿ
                if lambda == 0.0 {
                    continue;
                }
                let v = vid as usize;
                let x_v = positions[v];
                // Effective step-start `xᵗ_eff = xᵗ + Δ_surf`: shifting the friction
                // reference by the rigid surface's within-step tangential drift makes
                // `u_T = Tⁿᵀ(x_v − xᵗ − Δ_surf)` the slip RELATIVE to the moving
                // collider (a stationary vertex under a sliding collider gets dragged).
                // Δ_surf = 0 (the default) recovers PR1's static-collider friction.
                let x_start = crate::Vec3::new(x_prev[3 * v], x_prev[3 * v + 1], x_prev[3 * v + 2])
                    + self.friction_surface_drift;
                let (grad, hess) =
                    crate::contact::friction::grad_hess(x_v, x_start, force, lambda, mu, w);
                out.push((v, grad, hess));
            }
        }
        out
    }

    /// `∂r/∂Δ_surf · dir` (full-DOF) — the residual's sensitivity to the moving-collider
    /// tangential drift along `dir`. The drift enters the residual ONLY through the friction
    /// term `∇D(x_v − xᵗ − Δ_surf)`, so `∂r_v/∂Δ_surf = −∇²D_v` (chain rule through the
    /// negated `Δ_surf`); the directional column is `−∇²D_v·dir`. Zeros off the active set.
    /// Reuses `friction_blocks` (which evaluates `∇²D` at the drift-consistent reference).
    fn assemble_drift_residual_grad(
        &self,
        x_final: &[f64],
        x_prev: &[f64],
        dt: f64,
        dir: Vec3,
    ) -> Vec<f64> {
        let mut out = vec![0.0_f64; self.n_dof];
        for (v, _grad, hess) in self.friction_blocks(x_final, x_prev, dt) {
            let col = -(hess * dir); // ∂r_v/∂Δ_surf · dir
            out[3 * v] = col.x;
            out[3 * v + 1] = col.y;
            out[3 * v + 2] = col.z;
        }
        out
    }

    /// Forward sensitivity `∂x*/∂Δ_surf` of the converged soft equilibrium w.r.t. the
    /// moving-collider tangential drift, along the direction `dir`. The drift shifts the
    /// friction reference (`u_T = Tⁿᵀ(x_v − xᵗ − Δ_surf)`), entering the residual only through
    /// the friction term, so `∂x*/∂Δ_surf = −A⁻¹·(∂r/∂Δ_surf)` reuses the SAME drift-consistent
    /// factored tangent `A` (Woodbury-corrected) the material/pose sensitivities use — the
    /// drift's own term is in the RHS. `x_prev` is the step-start `xᵗ` (required: the drift
    /// sensitivity is only meaningful with friction active). Length `n_dof`, zeros on
    /// pinned/roller DOFs and when frictionless / no active pair. The coupling contracts this
    /// with `∂Δ_surf/∂(rigid velocity)` to thread the two-way grip feedback.
    #[must_use]
    pub fn equilibrium_drift_sensitivity(
        &self,
        x_final: &[f64],
        x_prev: &[f64],
        dt: f64,
        dir: Vec3,
    ) -> Vec<f64> {
        debug_assert!(x_final.len() == self.n_dof);
        let dr = self.assemble_drift_residual_grad(x_final, x_prev, dt, dir);
        let rhs: Vec<f64> = self.free_dof_indices.iter().map(|&i| -dr[i]).collect();
        self.solve_free_and_scatter(x_final, Some(x_prev), dt, rhs)
    }

    /// `∂r/∂μ_c` (full-DOF) — the residual's sensitivity to the Coulomb friction COEFFICIENT.
    /// Friction enters the residual ONLY through `∇D = (μ_c·λⁿ)·(t·grad2)`, which is LINEAR in
    /// `μ_c` (the lagged normal force `λⁿ`, the tangent basis, and the slip kernel are all
    /// `μ_c`-independent), so `∂r_v/∂μ_c = ∇D_v / μ_c` — the friction force with the coefficient
    /// divided back out. Zeros off the active set; empty when frictionless. Reuses
    /// `friction_blocks` (the same `∇D` the forward residual scatters).
    fn assemble_friction_coeff_residual_grad(
        &self,
        x_final: &[f64],
        x_prev: &[f64],
        dt: f64,
    ) -> Vec<f64> {
        let mut out = vec![0.0_f64; self.n_dof];
        let mu = self.config.friction_mu;
        if mu == 0.0 {
            return out;
        }
        for (v, grad, _hess) in self.friction_blocks(x_final, x_prev, dt) {
            out[3 * v] = grad.x / mu;
            out[3 * v + 1] = grad.y / mu;
            out[3 * v + 2] = grad.z / mu;
        }
        out
    }

    /// Forward sensitivity `∂x*/∂μ_c` of the converged soft equilibrium w.r.t. the Coulomb
    /// friction COEFFICIENT. The coefficient enters the residual only through the friction term
    /// (`∂r/∂μ_c = ∇D/μ_c`, see `assemble_friction_coeff_residual_grad`), so
    /// `∂x*/∂μ_c = −A⁻¹·(∂r/∂μ_c)` reuses the SAME factored (Woodbury-corrected) tangent `A` the
    /// material/drift sensitivities use. Unlike `∂x*/∂μ` (material), `μ_c` is LINEAR in the
    /// residual ⇒ machine-exact even at a stiff block (no compliant-block conditioning needed).
    /// `x_prev` is the step-start `xᵗ` (required: the coefficient sensitivity is only meaningful
    /// with friction active). Length `n_dof`, zeros on pinned/roller DOFs and when frictionless.
    #[must_use]
    pub fn equilibrium_friction_coeff_sensitivity(
        &self,
        x_final: &[f64],
        x_prev: &[f64],
        dt: f64,
    ) -> Vec<f64> {
        debug_assert!(x_final.len() == self.n_dof);
        let dr = self.assemble_friction_coeff_residual_grad(x_final, x_prev, dt);
        let rhs: Vec<f64> = self.free_dof_indices.iter().map(|&i| -dr[i]).collect();
        self.solve_free_and_scatter(x_final, Some(x_prev), dt, rhs)
    }

    /// Per-active-pair smoothed-Coulomb friction FORCE on the soft body `(vertex, −∇D)` at
    /// configuration `x_curr` with step start `x_prev`, including this solver's
    /// [`friction_surface_drift`](Self::with_friction_surface_drift). `−∇D` is the force the
    /// contact exerts on the soft side (the `force_on_soft` sign convention, the tangential
    /// companion to the normal contact force); it is bit-equal to the per-pair friction the
    /// forward residual scatters (both go through `friction_blocks`). A staggered coupling
    /// routes `−Σ` of these (and their off-COM moment `−Σ(rᵢ−c)×`) onto the rigid body as the
    /// tangential grip reaction. Empty when `friction_mu == 0` or no pair is active. `dt` is
    /// the step used to form the stick-band width `w = dt·ε_v`.
    // `v as VertexId`: `v` was produced as `vid as usize` from a `VertexId` (u32) in
    // `friction_blocks`, so the round-trip is lossless.
    #[allow(clippy::cast_possible_truncation)]
    #[must_use]
    pub fn friction_forces_on_soft(
        &self,
        x_curr: &[f64],
        x_prev: &[f64],
        dt: f64,
    ) -> Vec<(VertexId, crate::Vec3)> {
        self.friction_blocks(x_curr, x_prev, dt)
            .into_iter()
            .map(|(v, grad, _)| (v as VertexId, -grad))
            .collect()
    }

    /// The friction reaction force on the RIGID collider along `react_dir`,
    /// `F = (Σ_v ∇D_v)·react_dir`, together with its first-order sensitivities — the
    /// readout a staggered coupling routes onto its tangential rigid-state tape (the
    /// tangential companion of the normal `ContactForceTrajVjp`). At configuration
    /// `x_curr`, step start `x_prev`, and this solver's
    /// [`friction_surface_drift`](Self::with_friction_surface_drift).
    ///
    /// `∇D_v = μ_c·λⁿ_v·f₁·Tû` is **linear in the lagged normal force** `λⁿ_v`, so the
    /// reaction's total dependence on the post-step config has TWO parts (the flat-plane
    /// tangent `Tⁿ` is config-independent, so `∂Tⁿ/∂x` drops):
    /// - the frozen-lag slip term `∂(∇D_v)/∂x_v = ∇²D_v` (and `∂(∇D_v)/∂Δ_surf = −∇²D_v`,
    ///   the moving-collider reference shift), plus
    /// - the **normal-force coupling** `∂(∇D_v)/∂λⁿ_v · ∂λⁿ_v/∂· = a_v ⊗ ∂λⁿ_v/∂·` with
    ///   `a_v = ∇D_v/λⁿ_v`, `∂λⁿ_v/∂x = n̂ᵀ·∇²E_contact` (the same `(a_v, ∂λⁿ/∂x)` rank-1
    ///   pair the [`equilibrium_drift_sensitivity`](Self::equilibrium_drift_sensitivity)
    ///   Woodbury adjoint uses), and `∂λⁿ_v/∂height = n̂ᵀ·∂(∇E_contact)/∂(plane pose)`.
    ///
    /// Returns the [`FrictionReactionGradients`]: `force` (`F`), `dforce_dx` (the full
    /// `∂F/∂x*`, length `n_dof`, zeros off the active set), `dforce_ddrift` (`∂F/∂Δ_surf`
    /// along `drift_dir`, λ-independent), `dforce_dheight` (`∂F/∂height` along the `pose_dir`
    /// plane translation, the λ-coupling), and `dforce_dxprev` (`∂F/∂x_prev = −`the frozen-lag
    /// slip term, since `x_start = x_prev + Δ_surf`). Zeros when `friction_mu == 0` or no pair
    /// is active.
    #[must_use]
    pub fn friction_reaction_gradients(
        &self,
        x_curr: &[f64],
        x_prev: &[f64],
        dt: f64,
        react_dir: Vec3,
        drift_dir: Vec3,
        pose_dir: Vec3,
    ) -> FrictionReactionGradients {
        let mu = self.config.friction_mu;
        let mut out = FrictionReactionGradients {
            force: 0.0,
            dforce_dx: vec![0.0_f64; self.n_dof],
            dforce_dxprev: vec![0.0_f64; self.n_dof],
            dforce_ddrift: 0.0,
            dforce_dheight: 0.0,
        };
        if mu == 0.0 {
            return out;
        }
        let w = dt * self.config.friction_eps_v;
        let positions = slice_to_vec3s(x_curr);
        let pairs = self.contact.active_pairs(&self.mesh, &positions);
        let twist = RigidTwist::translation(pose_dir);
        for pair in &pairs {
            let grad_c = self.contact.gradient(pair, &positions);
            let hess_c = self.contact.hessian(pair, &positions);
            let pose_c = self
                .contact
                .pose_residual_derivative(pair, &positions, twist);
            // Force-direction curvature `C = ∂n̂/∂x_v = sign(dE)·∇²sd` (0 for a plane,
            // [`ContactModel::normal_curvature`]). The friction force turns as the collider normal
            // turns — `∂(∇D)/∂x_v` gains `DN·C` (the contact point slides) and `∂(∇D)/∂height`
            // gains `DN·(−C·pose_dir)` (the primitive translates), `DN = ∂∇D/∂n̂`.
            let hess_n = self.contact.normal_curvature(pair, &positions);
            for (vid, force) in &grad_c.contributions {
                let lambda = force.norm(); // λⁿ
                if lambda == 0.0 {
                    continue;
                }
                let v = *vid as usize;
                let x_v = positions[v];
                let x_start = Vec3::new(x_prev[3 * v], x_prev[3 * v + 1], x_prev[3 * v + 2])
                    + self.friction_surface_drift;
                let (grad_d, hess_d) =
                    crate::contact::friction::grad_hess(x_v, x_start, *force, lambda, mu, w);
                out.force += grad_d.dot(&react_dir);
                // Frozen-lag slip term (∇²D symmetric): ∂F/∂x_v = react_dirᵀ·∇²D_v, and the
                // step-start config enters via `x_start = x_prev + Δ_surf` so
                // ∂F/∂x_prev = −(this frozen-lag row) (λⁿ tracks x*, not x_prev).
                let row = hess_d * react_dir;
                out.dforce_dx[3 * v] += row.x;
                out.dforce_dx[3 * v + 1] += row.y;
                out.dforce_dx[3 * v + 2] += row.z;
                out.dforce_dxprev[3 * v] -= row.x;
                out.dforce_dxprev[3 * v + 1] -= row.y;
                out.dforce_dxprev[3 * v + 2] -= row.z;
                // Curved-normal term `DN·C` — the tangent frame rotates as the contact point
                // slides over the primitive (`∂n̂/∂x_v = C`). `C = 0` for a plane ⇒ +0 (the row
                // and the dforce_dheight term below stay byte-identical to the plane path). `n̂`
                // does NOT depend on x_prev, so dforce_dxprev gains no curved term.
                let dn = crate::contact::friction::normal_rotation_term(
                    x_v, x_start, *force, lambda, mu, w,
                );
                let row_curv = (dn * hess_n).transpose() * react_dir;
                out.dforce_dx[3 * v] += row_curv.x;
                out.dforce_dx[3 * v + 1] += row_curv.y;
                out.dforce_dx[3 * v + 2] += row_curv.z;
                // ∂n̂/∂height = −C·pose_dir, so ∂F/∂height gains react_dirᵀ·DN·(−C·pose_dir).
                out.dforce_dheight += react_dir.dot(&(dn * (-(hess_n * pose_dir))));
                // Moving-collider reference: ∂F/∂Δ_surf = −react_dirᵀ·∇²D_v·drift_dir (n̂ ⊥ drift).
                out.dforce_ddrift -= react_dir.dot(&(hess_d * drift_dir));
                // Normal-force (λ) coupling — the same (a_v, ∂λⁿ/∂x) rank-1 pair as the
                // drift-consistent Woodbury adjoint.
                let a = grad_d / lambda; // ∂(∇D_v)/∂λⁿ_v
                let coeff = react_dir.dot(&a); // react_dirᵀ·a_v
                let nhat = force / lambda;
                for (rv, cv, block) in &hess_c.contributions {
                    if *rv as usize != v {
                        continue;
                    }
                    let dlam = block.transpose() * nhat; // ∂λⁿ_v/∂x_cv
                    let c = *cv as usize;
                    out.dforce_dx[3 * c] += coeff * dlam.x;
                    out.dforce_dx[3 * c + 1] += coeff * dlam.y;
                    out.dforce_dx[3 * c + 2] += coeff * dlam.z;
                }
                for &(pv, d) in &pose_c.contributions {
                    if pv as usize == v {
                        out.dforce_dheight += coeff * nhat.dot(&d); // ∂λⁿ_v/∂height
                    }
                }
            }
        }
        out
    }

    /// The PER-VERTEX friction force on the rigid collider `∇D_v` and its first-order
    /// sensitivities — the VECTOR generalization of [`Self::friction_reaction_gradients`]
    /// (which projects each `∇D_v` onto a single `react_dir` and sums to a scalar). A
    /// staggered coupling routes the per-vertex VECTOR force to assemble the off-COM friction
    /// MOMENT `Σ_v (r_v − c) × ∇D_v` and its Jacobian (the scalar aggregate cannot — each
    /// vertex's moment arm `r_v − c` differs). Same math as the aggregate, kept per-vertex:
    /// - frozen-lag slip `∂(∇D_v)/∂x_v = ∇²D_v` (3×3) at `v`'s own coords, and
    ///   `∂(∇D_v)/∂x_prev,v = −∇²D_v`, `∂(∇D_v)/∂Δ_surf = −∇²D_v·drift_dir`;
    /// - normal-force λⁿ coupling `∂(∇D_v)/∂x_c = a_v ⊗ ∂λⁿ_v/∂x_c` (`a_v = ∇D_v/λⁿ_v`,
    ///   `∂λⁿ_v/∂x_c = n̂ᵀ·∇²E_contact`) over the contact neighbors `c`, and
    ///   `∂(∇D_v)/∂height = a_v·(n̂ᵀ·∂(plane)/∂pose)`.
    ///
    /// Returns one [`FrictionVertexForce`] per active contacted vertex (empty when
    /// `friction_mu == 0` or no pair is active), with `dforce_dx`/`dforce_dxprev` as row-major
    /// `3 × n_dof` blocks. At configuration `x_curr`, step start `x_prev`, and this solver's
    /// [`friction_surface_drift`](Self::with_friction_surface_drift).
    ///
    /// **Curved-collider scope:** like the scalar [`Self::friction_reaction_gradients`], this
    /// per-vertex VECTOR version threads the curved-normal tangent-rotation term `DN·C` — so on a
    /// finite curved primitive `dforce_dx` gains `∂(∇D_v)/∂x_v += DN·C` and `dforce_dheight` gains
    /// `DN·(−C·pose_dir)` (`C = ∂n̂/∂x_v = sign(dE)·∇²sd`, [`ContactModel::normal_curvature`]).
    /// `C = 0` for a plane ⇒ byte-identical. FD-exact on a sphere
    /// (`per_vertex_force_jacobians_sphere_matches_fd`); its off-COM-moment / articulated-friction
    /// wrench consumer is curvature-correct and un-guarded for the articulated FRICTION gradients,
    /// and the actuator/policy siblings inherit it (curvature-correct on a centroid sphere; the
    /// `g_act` channel is contact-independent — they guard only the moving end-effector).
    #[must_use]
    pub fn friction_force_jacobians(
        &self,
        x_curr: &[f64],
        x_prev: &[f64],
        dt: f64,
        drift_dir: Vec3,
        pose_dir: Vec3,
    ) -> Vec<FrictionVertexForce> {
        let mu = self.config.friction_mu;
        let mut out = Vec::new();
        if mu == 0.0 {
            return out;
        }
        let nd = self.n_dof;
        let w = dt * self.config.friction_eps_v;
        let positions = slice_to_vec3s(x_curr);
        let pairs = self.contact.active_pairs(&self.mesh, &positions);
        let twist = RigidTwist::translation(pose_dir);
        for pair in &pairs {
            let grad_c = self.contact.gradient(pair, &positions);
            let hess_c = self.contact.hessian(pair, &positions);
            let pose_c = self
                .contact
                .pose_residual_derivative(pair, &positions, twist);
            // Force-direction curvature `C = ∂n̂/∂x_v = sign(dE)·∇²sd` (0 for a plane,
            // [`ContactModel::normal_curvature`]) — the curved-collider normal rotates as the
            // contact point slides, turning the friction force (the per-vertex VECTOR analog of
            // the curved-normal term `friction_reaction_gradients` carries).
            let hess_n = self.contact.normal_curvature(pair, &positions);
            for (vid, force) in &grad_c.contributions {
                let lambda = force.norm(); // λⁿ
                if lambda == 0.0 {
                    continue;
                }
                let v = *vid as usize;
                let x_v = positions[v];
                let x_start = Vec3::new(x_prev[3 * v], x_prev[3 * v + 1], x_prev[3 * v + 2])
                    + self.friction_surface_drift;
                let (grad_d, hess_d) =
                    crate::contact::friction::grad_hess(x_v, x_start, *force, lambda, mu, w);
                let mut dforce_dx = vec![0.0_f64; 3 * nd];
                let mut dforce_dxprev = vec![0.0_f64; 3 * nd];
                // Frozen-lag slip ∇²D_v at v's own coords (and −∇²D_v for x_prev).
                for r in 0..3 {
                    for col in 0..3 {
                        dforce_dx[r * nd + 3 * v + col] += hess_d[(r, col)];
                        dforce_dxprev[r * nd + 3 * v + col] -= hess_d[(r, col)];
                    }
                }
                // Curved-normal term `DN·C` (the VECTOR analog of `friction_reaction_gradients`'s
                // scalar `(DN·C)ᵀ·react_dir`, kept per-vertex for the off-COM moment routing):
                // ∂(∇D_v)/∂x_v gains `DN·C` as the tangent frame rotates with the sliding contact
                // point (`∂n̂/∂x_v = C`). `C = 0` for a plane ⇒ literal +0 (the rows and the
                // dforce_dheight curved term below stay byte-identical to the plane path). `n̂`
                // does NOT depend on x_prev, so dforce_dxprev gains no curved term.
                let dn = crate::contact::friction::normal_rotation_term(
                    x_v, x_start, *force, lambda, mu, w,
                );
                let m_curv = dn * hess_n; // DN·C (3×3)
                for r in 0..3 {
                    for col in 0..3 {
                        dforce_dx[r * nd + 3 * v + col] += m_curv[(r, col)];
                    }
                }
                // Moving-collider reference: ∂(∇D_v)/∂Δ_surf = −∇²D_v·drift_dir.
                let dforce_ddrift = -(hess_d * drift_dir);
                // Normal-force (λ) coupling: a_v ⊗ ∂λⁿ_v/∂x_c over contact neighbors.
                let a = grad_d / lambda; // ∂(∇D_v)/∂λⁿ_v
                let nhat = force / lambda;
                for (rv, cv, block) in &hess_c.contributions {
                    if *rv as usize != v {
                        continue;
                    }
                    let dlam = block.transpose() * nhat; // ∂λⁿ_v/∂x_cv
                    let c = *cv as usize;
                    for r in 0..3 {
                        dforce_dx[r * nd + 3 * c] += a[r] * dlam.x;
                        dforce_dx[r * nd + 3 * c + 1] += a[r] * dlam.y;
                        dforce_dx[r * nd + 3 * c + 2] += a[r] * dlam.z;
                    }
                }
                // Plane-pose (height) coupling: ∂(∇D_v)/∂height = a_v·(n̂·∂(plane)/∂pose).
                let mut dforce_dheight = Vec3::zeros();
                for &(pv, d) in &pose_c.contributions {
                    if pv as usize == v {
                        dforce_dheight += a * nhat.dot(&d);
                    }
                }
                // Curved-normal height term: raising the height translates the primitive +pose_dir,
                // so `∂n̂/∂height = −C·pose_dir` and ∂(∇D_v)/∂height gains `DN·(−C·pose_dir)`.
                // `C = 0` for a plane ⇒ +0 (byte-identical).
                dforce_dheight += dn * (-(hess_n * pose_dir));
                out.push(FrictionVertexForce {
                    vid: *vid,
                    force: grad_d,
                    dforce_dx,
                    dforce_dxprev,
                    dforce_ddrift,
                    dforce_dheight,
                    // ∇D_v is linear in μ_c ⇒ ∂force/∂μ_c = ∇D_v/μ_c (mu > 0 here: the mu == 0
                    // early-return above guarantees it).
                    dforce_dmu_c: grad_d / mu,
                });
            }
        }
        out
    }

    /// Assemble the lower-triangle triplets of the free-DOF Hessian
    /// `A_free = M_free / Δt² + K_free(x_curr) + K_contact(x_curr)`
    /// per Decision J + Phase 5 commit 5.
    ///
    /// For each element + each (a, b) vertex pair, contributes the
    /// 3×3 block from `B_a^T 𝕔 B_b` (using BF-5's flattening
    /// convention) into the global free-DOF sparse matrix at
    /// `(free_idx_a, free_idx_b)` whenever both DOFs are free (looked
    /// up via `full_to_free_idx`). Contact Hessian contributions from
    /// `self.contact.hessian` (rank-1 symmetric `κ·n⊗n` blocks for
    /// `PenaltyRigidContact`; symmetry assumption matches the existing
    /// lower-triangle filter) scatter through the same gate between
    /// the elastic and mass-diagonal passes. Diagonal mass added at
    /// the end. `BTreeMap` accumulates with deterministic (col, row)
    /// iteration order per Decision M.
    //
    // Lint allows: `as TetId` is the Mesh-trait-API tax (n_tets
    // returns usize, tet_vertices takes u32) — same as commit 2's
    // HandBuiltTetMesh. `for a/b in 0..4` iterates over Tet4's 4 nodes
    // by index (used for both `verts[a]` AND `geom.grad_x_n[(a, l)]`
    // — the lint flags the verts use only). `va`/`vb` similar-name
    // pair mirrors the (a, b) symmetry of the per-element block math.
    #[allow(
        clippy::cast_possible_truncation,
        clippy::needless_range_loop,
        clippy::similar_names
    )]
    fn assemble_free_hessian_triplets(
        &self,
        x_curr: &[f64],
        x_prev: Option<&[f64]>,
        dt: f64,
    ) -> Vec<Triplet<usize, usize, f64>> {
        debug_assert!(x_curr.len() == self.n_dof);
        // `dt * dt` then `mass / dt2` matches the pre-commit-4 code's
        // mass-diagonal expression order (`mass_per_dof / (dt * dt)`).
        // Distinct from `mass * (1.0 / (dt * dt))` at the last bit — the
        // FP-equal form preserves bit-equality with the pre-Phase-2
        // 1-tet path, simplifying any future bisect.
        let dt2 = dt * dt;
        // (col, row) → accumulated value. BTreeMap for sorted iteration
        // (Decision M D-3); no HashMap on numeric paths.
        let mut acc: BTreeMap<(usize, usize), f64> = BTreeMap::new();

        let materials = self.mesh.materials();
        for (tet_id, geom) in self.element_geometries.iter().enumerate() {
            let verts = self.mesh.tet_vertices(tet_id as TetId);
            let x_elem = extract_element_dof_values(x_curr, &verts);
            let f = deformation_gradient(&x_elem, &geom.grad_x_n);
            let tangent_9x9 = materials[tet_id].tangent(&f);

            for a in 0..4 {
                let va = verts[a] as usize;
                for b in 0..4 {
                    let vb = verts[b] as usize;
                    for i in 0..3 {
                        for j in 0..3 {
                            let row_full = 3 * va + i;
                            let col_full = 3 * vb + j;
                            if let (Some(row_free), Some(col_free)) = (
                                self.full_to_free_idx[row_full],
                                self.full_to_free_idx[col_full],
                            ) && row_free >= col_free
                            {
                                // (B_a^T 𝕔 B_b)[i,j] = Σ_{l,l'} (grad_X N_a)_l ·
                                //   𝕔[(i+3l), (j+3l')] · (grad_X N_b)_{l'}
                                let mut block = 0.0;
                                for l in 0..3 {
                                    for lp in 0..3 {
                                        block += geom.grad_x_n[(a, l)]
                                            * tangent_9x9[(i + 3 * l, j + 3 * lp)]
                                            * geom.grad_x_n[(b, lp)];
                                    }
                                }
                                *acc.entry((col_free, row_free)).or_insert(0.0) +=
                                    geom.volume * block;
                            }
                        }
                    }
                }
            }
        }

        // Contact Hessian contributions. Scatter through the same
        // free-DOF + lower-triangle filter as the elastic block above
        // (scope memo §6 R-5 lens (v) — sign-consistent with the
        // f_int gradient scatter in `assemble_global_int_force`).
        // `NullContact` returns an empty contributions Vec → empty
        // for-loops → acc unchanged → bit-equal Hessian.
        let positions = slice_to_vec3s(x_curr);
        let pairs = self.contact.active_pairs(&self.mesh, &positions);
        for pair in &pairs {
            let h = self.contact.hessian(pair, &positions);
            for &(row_vid, col_vid, block) in &h.contributions {
                for i in 0..3 {
                    for j in 0..3 {
                        let row_full = 3 * (row_vid as usize) + i;
                        let col_full = 3 * (col_vid as usize) + j;
                        if let (Some(row_free), Some(col_free)) = (
                            self.full_to_free_idx[row_full],
                            self.full_to_free_idx[col_full],
                        ) && row_free >= col_free
                        {
                            *acc.entry((col_free, row_free)).or_insert(0.0) += block[(i, j)];
                        }
                    }
                }
            }
        }

        // Smoothed-Coulomb friction Hessian `∇²D` (PSD per-pair 3×3 block at the contacted
        // vertex), scattered through the same free-DOF + lower-triangle filter. Threaded
        // ONLY when `x_prev` is supplied — the FORWARD Newton solve passes `Some(x_prev)`
        // (so the solve converges to the friction equilibrium); the differentiable tangent
        // (`factor_at_position`) passes `None` (friction enters the adjoint in the
        // differentiability leaf). Empty / `None` ⇒ acc unchanged ⇒ bit-equal.
        if let Some(x_prev) = x_prev {
            for (v, _, block) in self.friction_blocks(x_curr, x_prev, dt) {
                for i in 0..3 {
                    for j in 0..3 {
                        let (row_full, col_full) = (3 * v + i, 3 * v + j);
                        if let (Some(rf), Some(cf)) = (
                            self.full_to_free_idx[row_full],
                            self.full_to_free_idx[col_full],
                        ) && rf >= cf
                        {
                            *acc.entry((cf, rf)).or_insert(0.0) += block[(i, j)];
                        }
                    }
                }
            }
        }

        // Mass diagonal: M_free / Δt² · I on (k, k).
        for k in 0..self.n_free {
            let mass_dof = self.mass_per_dof[self.free_dof_indices[k]];
            *acc.entry((k, k)).or_insert(0.0) += mass_dof / dt2;
        }

        acc.into_iter()
            .map(|((c, r), v)| Triplet::new(r, c, v))
            .collect()
    }

    /// Forward sensitivity `∂x*/∂s` of the converged step's soft
    /// equilibrium to an infinitesimal *rigid motion* of the contact
    /// primitive(s) — the spatial [`RigidTwist`] `(ω, v)` — holding
    /// `(x_prev, v_prev, θ, dt)` fixed — the keystone S3 implicit factor
    /// (the soft re-equilibration the explicit coupled-step Jacobian was
    /// missing). A pure translation along `dir` is
    /// [`RigidTwist::translation(dir)`](RigidTwist::translation); a
    /// rotating contact normal (`ω ≠ 0`) is the rotating-normal leaf.
    ///
    /// At the converged step `r(x*; pose) = 0` the implicit function
    /// theorem gives `∂x*/∂s = −A⁻¹·(∂r/∂s)` with `A = ∂r/∂x|_{x*}` — the
    /// SAME tangent the forward Newton step converged with (re-factored at
    /// `x_final` via `factor_at_position`, the factor [`NewtonStepVjp`] also
    /// reuses for the load adjoint `∂x*/∂θ`). The plane pose enters the
    /// residual only through the contact term, so `(∂r/∂s)` is gathered
    /// from the active set via
    /// [`ContactModel::pose_residual_derivative`]; the free-DOF system is
    /// solved with the factored tangent and scattered back to a full-DOF
    /// vector (zeros on pinned / roller DOFs).
    ///
    /// `x_final` is the converged position from the matching
    /// [`Solver::step`] / [`Solver::replay_step`]; `dt` is that step's
    /// time-step (the tangent's `M/Δt²` inertia term must match). The
    /// result is length `n_dof` (`3·n_vertices`) in the solver's DOF
    /// layout. For a pose-independent contact ([`NullContact`]) the
    /// active set yields no pose contributions and the result is all
    /// zeros.
    ///
    /// With friction active (`friction_mu != 0` and `Some(x_prev)`), the pose RHS additionally
    /// carries the friction term `∂(∇D)/∂pose` (the λⁿ-coupling plus the curved-normal tangent
    /// rotation, `assemble_friction_pose_residual_grad`) for a pure translation along
    /// `twist.linear` — the same RHS the reverse grip path
    /// ([`Self::trajectory_step_vjp_grip`]) assembles. FD-validated under a curved (sphere)
    /// collider with friction in `tests/friction_sphere_tangent.rs`.
    ///
    /// Scope: contact-engaged, stable-active-set regime (the penalty
    /// active-set boundary is non-smooth — IPC the deferred cure); the
    /// normal-rotation term is exact for plane primitives (`δn̂ = ω×n̂`) and now also for curved
    /// primitives (`δn̂ = ω×n̂ − H·u`, the #415 curvature term in
    /// [`ContactModel::pose_residual_derivative`]). See
    /// `docs/keystone/rotating_normal_recon.md` and
    /// `docs/keystone/s3_soft_pose_sensitivity_recon.md`. FD-validated
    /// against a re-solve in `tests/soft_pose_sensitivity.rs`.
    ///
    /// [`NewtonStepVjp`]: crate::differentiable::newton_vjp::NewtonStepVjp
    /// [`ContactModel::pose_residual_derivative`]: crate::contact::ContactModel::pose_residual_derivative
    /// [`NullContact`]: crate::contact::NullContact
    #[must_use]
    pub fn equilibrium_pose_sensitivity(
        &self,
        x_final: &[f64],
        x_prev: Option<&[f64]>,
        dt: f64,
        twist: RigidTwist,
    ) -> Vec<f64> {
        debug_assert!(x_final.len() == self.n_dof);
        let mut dr_dpose = self.assemble_pose_residual_grad(x_final, twist);
        // With friction active (`Some(x_prev)`), the pose RHS also carries the friction term
        // `∂(∇D)/∂pose` (λⁿ-coupling + the curved-normal tangent rotation,
        // `assemble_friction_pose_residual_grad`) — the same RHS the reverse grip path
        // (`trajectory_step_vjp_grip`) assembles. The friction pose RHS supports only a pure
        // TRANSLATION (`twist.linear`); the rotational friction pose term is not yet wired, so
        // fail LOUDLY for an angular twist with friction rather than return a silently-incomplete
        // sensitivity (the normal RHS above DOES use the full twist — a partial result would be a
        // silent contract violation on this public API).
        if self.config.friction_mu != 0.0
            && let Some(xp) = x_prev
        {
            assert!(
                twist.angular == Vec3::zeros(),
                "equilibrium_pose_sensitivity: friction pose sensitivity supports only a pure \
                 translation (twist.angular must be 0); the rotational friction pose RHS is not \
                 yet wired. Pass a translation twist, or x_prev = None for the frictionless path."
            );
            let fric = self.assemble_friction_pose_residual_grad(x_final, xp, dt, twist.linear);
            for (d, f) in dr_dpose.iter_mut().zip(&fric) {
                *d += f;
            }
        }
        // A·w_free = −(∂r/∂s)_free reusing the tangent at x_final.
        let rhs: Vec<f64> = self
            .free_dof_indices
            .iter()
            .map(|&i| -dr_dpose[i])
            .collect();
        self.solve_free_and_scatter(x_final, x_prev, dt, rhs)
    }

    /// Reuse the tangent `A` factored at `x_final` to solve the free-DOF adjoint
    /// system `A·w_free = rhs_free` and scatter the result onto full DOFs (pinned
    /// / roller DOFs stay 0) — `−A⁻¹·(∂r/∂·)` for a pre-gathered, pre-signed
    /// `rhs_free = (−∂r/∂·)_free`. The shared back-half of the forward equilibrium
    /// sensitivities ([`Self::equilibrium_pose_sensitivity`] /
    /// [`Self::equilibrium_material_sensitivity`] /
    /// [`Self::equilibrium_state_sensitivity`]), which differ only in how they
    /// assemble `rhs_free`.
    /// `x_prev` is the step-start position `xᵗ`; `Some` enables the friction-exact
    /// adjoint (the Woodbury `∂λⁿ/∂x` correction), `None` is the frictionless / friction
    /// `μ = 0` path (bit-identical to pre-friction).
    fn solve_free_and_scatter(
        &self,
        x_final: &[f64],
        x_prev: Option<&[f64]>,
        dt: f64,
        mut rhs_free: Vec<f64>,
    ) -> Vec<f64> {
        debug_assert_eq!(rhs_free.len(), self.n_free);
        let factor = self.factor_at_position(x_final, x_prev, dt, 0.0);
        factor.solve_free_in_place(&mut rhs_free);
        let mut sensitivity = vec![0.0_f64; self.n_dof];
        for (k, &full_idx) in self.free_dof_indices.iter().enumerate() {
            sensitivity[full_idx] = rhs_free[k];
        }
        sensitivity
    }

    /// Forward sensitivity `∂x*/∂p_k` of the converged step's soft equilibrium
    /// to the `k`-th material parameter (`param_idx`; for [`crate::NeoHookean`]:
    /// `0 = μ`, `1 = λ`), holding `(x_prev, v_prev, θ, dt)` fixed — the keystone
    /// S5 material-parameter sensitivity.
    ///
    /// The material parameters enter the residual only through the elastic
    /// internal force, so `∂r/∂p_k = ∂f_int/∂p_k` assembles exactly like
    /// `f_int` (`assemble_global_int_force`) but with the per-element stress
    /// derivative `∂P/∂p_k` from [`Material::first_piola_param_grad`] in
    /// place of `P`. Then the IFT gives `∂x*/∂p_k = −A⁻¹·(∂r/∂p_k)`, solved with
    /// the SAME tangent `A` factored at `x_final` (`factor_at_position`) that the
    /// forward Newton step, the load adjoint, and the pose sensitivity reuse.
    /// The result is length `n_dof` (zeros on pinned / roller DOFs).
    ///
    /// The material path is **contact-independent** (penalty contact does not
    /// depend on the material parameters) — contact enters only through the
    /// tangent `A`, so this is valid with or without contact. A per-tet material
    /// that exposes no differentiable parameters (the default
    /// `first_piola_param_grad`) contributes zero; the result is all-zeros if no
    /// tet exposes parameters.
    ///
    /// # Panics
    /// Panics if `param_idx` is out of range for a tet whose material *does*
    /// expose parameters (a usage error — the index must match the material's
    /// parameter convention).
    //
    // Lint allows mirror `assemble_global_int_force` (the loop this method
    // re-runs with ∂P/∂p_k for P): `as TetId` is the Mesh-trait API tax,
    // `for a in 0..4` iterates Tet4's 4 nodes by index (used for both verts[a]
    // and grad_x_n[(a, j)]).
    #[must_use]
    pub fn equilibrium_material_sensitivity(
        &self,
        x_final: &[f64],
        x_prev: Option<&[f64]>,
        dt: f64,
        param_idx: usize,
    ) -> Vec<f64> {
        debug_assert!(x_final.len() == self.n_dof);
        let dr_dp = self.assemble_material_residual_grad(x_final, param_idx);
        // A·w_free = −(∂r/∂p_k)_free reusing the tangent at x_final.
        let rhs: Vec<f64> = self.free_dof_indices.iter().map(|&i| -dr_dp[i]).collect();
        self.solve_free_and_scatter(x_final, x_prev, dt, rhs)
    }

    /// `(∂r/∂height · pose_dir)_full` from the FRICTION term — the friction successor to the
    /// normal [`Self::assemble_pose_residual_grad`], needed when the contact plane moves (the
    /// grip's coupled height). The friction force `∇D = μ_c·λⁿ·f₁·Tû` scattered into the residual
    /// has TWO pose channels:
    /// - the **λⁿ-coupling** (`∇D` is linear in the lagged normal force `λⁿ`, which the pose
    ///   changes): `a_v·(n̂ᵀ·∂(∇E_contact_v)/∂pose)`, `a_v = ∇D_v/λⁿ_v`;
    /// - the **curved-normal tangent rotation** `DN·(−C·pose_dir)` (the tangent frame turns as the
    ///   primitive translates, `∂n̂/∂pose = −C·pose_dir`, `C` the force-direction curvature
    ///   [`ContactModel::normal_curvature`], `DN = ∂∇D/∂n̂`). Zero for a plane (`C = 0`, its
    ///   constant tangent IS pose-independent), the curved-contact term for a finite primitive.
    ///
    /// Zeros off the active set / when frictionless. Drift-consistent (`xᵗ + Δ_surf`). Pure
    /// translation along `pose_dir` (the rotational pose channel is not wired here).
    fn assemble_friction_pose_residual_grad(
        &self,
        x_final: &[f64],
        x_prev: &[f64],
        dt: f64,
        pose_dir: Vec3,
    ) -> Vec<f64> {
        let mu = self.config.friction_mu;
        let mut out = vec![0.0_f64; self.n_dof];
        if mu == 0.0 {
            return out;
        }
        let w = dt * self.config.friction_eps_v;
        let positions = slice_to_vec3s(x_final);
        let pairs = self.contact.active_pairs(&self.mesh, &positions);
        let twist = RigidTwist::translation(pose_dir);
        for pair in &pairs {
            let grad_c = self.contact.gradient(pair, &positions);
            let pose_c = self
                .contact
                .pose_residual_derivative(pair, &positions, twist);
            // Force-direction curvature `C = ∂n̂/∂x = sign(dE)·∇²sd` (0 for a plane) — the
            // curved-normal companion to the λ-coupling: the tangent frame rotates as the
            // primitive translates, `∂n̂/∂height = −C·pose_dir`, adding `DN·(−C·pose_dir)` to
            // `∂r_v/∂height`.
            let hess_n = self.contact.normal_curvature(pair, &positions);
            for (vid, force) in &grad_c.contributions {
                let lambda = force.norm();
                if lambda == 0.0 {
                    continue;
                }
                let v = *vid as usize;
                let x_v = positions[v];
                let x_start = Vec3::new(x_prev[3 * v], x_prev[3 * v + 1], x_prev[3 * v + 2])
                    + self.friction_surface_drift;
                let (grad_d, _) =
                    crate::contact::friction::grad_hess(x_v, x_start, *force, lambda, mu, w);
                let a = grad_d / lambda;
                let nhat = force / lambda;
                for &(pv, d) in &pose_c.contributions {
                    if pv as usize == v {
                        let dlam = nhat.dot(&d); // ∂λⁿ_v/∂height
                        out[3 * v] += a.x * dlam;
                        out[3 * v + 1] += a.y * dlam;
                        out[3 * v + 2] += a.z * dlam;
                    }
                }
                // Curved-normal tangent-rotation term `DN·(−C·pose_dir)` (`+0` for a plane).
                let dn = crate::contact::friction::normal_rotation_term(
                    x_v, x_start, *force, lambda, mu, w,
                );
                let curv = dn * (-(hess_n * pose_dir));
                out[3 * v] += curv.x;
                out[3 * v + 1] += curv.y;
                out[3 * v + 2] += curv.z;
            }
        }
        out
    }

    /// `(∂r/∂s)_full` — the contact-plane pose enters the residual only through
    /// the contact term, so this is the active-pair sum of
    /// [`ContactModel::pose_residual_derivative`](crate::contact::ContactModel::pose_residual_derivative)
    /// for the primitive's rigid-motion [`RigidTwist`] `(ω, v)`. Shared (kept in
    /// lockstep) by [`Self::equilibrium_pose_sensitivity`] (forward; negates +
    /// solves) and [`Self::trajectory_step_vjp`] (reverse; gathers the free DOFs).
    fn assemble_pose_residual_grad(&self, x_final: &[f64], twist: RigidTwist) -> Vec<f64> {
        let positions = slice_to_vec3s(x_final);
        let pairs = self.contact.active_pairs(&self.mesh, &positions);
        let mut dr_dpose = vec![0.0_f64; self.n_dof];
        for pair in &pairs {
            let g = self
                .contact
                .pose_residual_derivative(pair, &positions, twist);
            for &(vid, d) in &g.contributions {
                let v = vid as usize;
                dr_dpose[3 * v] += d.x;
                dr_dpose[3 * v + 1] += d.y;
                dr_dpose[3 * v + 2] += d.z;
            }
        }
        dr_dpose
    }

    /// `(∂r/∂p_k)_full = ∂f_int/∂p_k` — the `f_int` assembly
    /// (`assemble_global_int_force`) re-run with the per-element stress
    /// derivative `∂P/∂p_k` (entry `param_idx` of
    /// [`Material::first_piola_param_grad`]) in place of `P`. Shared by
    /// [`Self::equilibrium_material_sensitivity`] (forward) and
    /// [`Self::material_step_vjp`] (reverse).
    //
    // Lint allows mirror `assemble_global_int_force` (see that method).
    #[allow(clippy::cast_possible_truncation, clippy::needless_range_loop)]
    fn assemble_material_residual_grad(&self, x_final: &[f64], param_idx: usize) -> Vec<f64> {
        let materials = self.mesh.materials();
        let mut dr_dp = vec![0.0_f64; self.n_dof];
        for (tet_id, geom) in self.element_geometries.iter().enumerate() {
            let verts = self.mesh.tet_vertices(tet_id as TetId);
            let x_elem = extract_element_dof_values(x_final, &verts);
            let f = deformation_gradient(&x_elem, &geom.grad_x_n);
            let dp = materials[tet_id].first_piola_param_grad(&f);
            if dp.is_empty() {
                continue; // material exposes no differentiable params → zero
            }
            assert!(
                param_idx < dp.len(),
                "material param index {param_idx} out of range for tet {tet_id}'s material \
                 ({} differentiable parameter(s) per first_piola_param_grad)",
                dp.len(),
            );
            let dp_dpk = dp[param_idx];
            for a in 0..4 {
                let v = verts[a] as usize;
                for i in 0..3 {
                    let mut sum = 0.0;
                    for j in 0..3 {
                        sum += dp_dpk[(i, j)] * geom.grad_x_n[(a, j)];
                    }
                    dr_dp[3 * v + i] += geom.volume * sum;
                }
            }
        }
        dr_dp
    }

    /// `(∂r/∂p)_full` for a **linear combination** of the material parameters,
    /// `Σ_k weights[k]·(∂r/∂p_k)` — the residual sensitivity to a single design
    /// variable `p` that drives several material parameters at once via
    /// `p_k = p_k(p)` with `weights[k] = dp_k/dp`. Used for a tied reparametrization
    /// such as the coupling's stiffness scale `μ = p, λ = 4p` (`weights = [1, 4]`),
    /// so its total sensitivity rides ONE tape parent — `∂/∂p = Σ_k (dp_k/dp)·∂/∂p_k`.
    /// `weights.len()` is the number of driven material parameters; a zero weight
    /// skips that parameter. Delegates to
    /// [`Self::assemble_material_residual_grad`] per parameter (so it inherits the
    /// same per-element stress-derivative assembly), then accumulates.
    fn assemble_material_residual_grad_combined(
        &self,
        x_final: &[f64],
        weights: &[f64],
    ) -> Vec<f64> {
        let mut acc = vec![0.0_f64; self.n_dof];
        for (k, &w) in weights.iter().enumerate() {
            if w == 0.0 {
                continue;
            }
            let dr_k = self.assemble_material_residual_grad(x_final, k);
            for (a, &d) in acc.iter_mut().zip(&dr_k) {
                *a += w * d;
            }
        }
        acc
    }

    /// Build a [`MaterialStepVjp`] for one converged step — the reverse-mode
    /// (tape) sibling of [`Self::equilibrium_material_sensitivity`]: pushed onto
    /// a chassis tape with the material parameter as parent, it turns a
    /// downstream `∂L/∂x*` cotangent into `∂L/∂p_k` (keystone S5). The op stashes
    /// the tangent factored at `x_final` plus `(∂r/∂p_k)_free`, and its VJP
    /// solves `A·λ = g_free` then contracts `−λ^T·(∂r/∂p_k)_free` — the same
    /// adjoint as [`NewtonStepVjp`], with the material RHS factor.
    ///
    /// `x_final` is the converged position; `dt` is the step's time-step (the
    /// tangent's `M/Δt²` term must match); `param_idx` selects the material
    /// parameter (`NeoHookean`: `0 = μ`, `1 = λ`). The pushed node's value should
    /// be the `x_final` tensor (shape `[n_dof]`) and its single parent the
    /// material-parameter `Var` (shape `[1]`).
    #[must_use]
    pub fn material_step_vjp(&self, x_final: &[f64], dt: f64, param_idx: usize) -> MaterialStepVjp {
        debug_assert!(x_final.len() == self.n_dof);
        let dr_dp = self.assemble_material_residual_grad(x_final, param_idx);
        let dr_dp_free: Vec<f64> = self.free_dof_indices.iter().map(|&i| dr_dp[i]).collect();
        let factor = self.factor_at_position(x_final, None, dt, 0.0);
        MaterialStepVjp::new(
            factor,
            self.n_dof,
            self.free_dof_indices.clone(),
            dr_dp_free,
        )
    }

    /// Forward sensitivity of the converged step to the PREVIOUS state — the
    /// multi-step time-adjoint primitive (keystone time-adjoint leaf). Returns
    /// the directional derivative `∂x*` (length `n_dof`, zeros on pinned/roller
    /// DOFs) for a perturbation `(dx_prev, dv_prev)` of the previous position and
    /// velocity.
    ///
    /// The previous state enters the backward-Euler residual
    /// `r = (M/Δt²)·(x − x̂) + f_int − f_ext` ONLY through the predictor
    /// `x̂ = x_prev + Δt·v_prev`, so `∂r/∂x_prev = −(M/Δt²)`,
    /// `∂r/∂v_prev = −(M/Δt)` (diagonal, lumped mass). The IFT then gives
    /// `∂x* = −A⁻¹·(∂r/∂x_prev·dx_prev + ∂r/∂v_prev·dv_prev)
    ///      = A⁻¹·((M/Δt²)·dx_prev + (M/Δt)·dv_prev)`, solved with the SAME
    /// tangent `A` factored at `x_final` (`factor_at_position`) the forward
    /// Newton step, the load adjoint, the pose sensitivity, and the material
    /// sensitivity reuse. The RHS is gathered over the FREE DOFs only — the
    /// world-pinned base is a constant outside the differentiable thread (see
    /// [`StateStepVjp`]). The reverse-mode dual is [`Self::state_step_vjp`].
    // `dx_prev`/`dv_prev` are the perturbations of `x_prev`/`v_prev`; the
    // parallel naming is the clearest scheme (renaming would obscure the pair).
    #[allow(clippy::similar_names)]
    #[must_use]
    pub fn equilibrium_state_sensitivity(
        &self,
        x_final: &[f64],
        x_prev: Option<&[f64]>,
        dt: f64,
        dx_prev: &[f64],
        dv_prev: &[f64],
    ) -> Vec<f64> {
        debug_assert!(x_final.len() == self.n_dof);
        debug_assert!(dx_prev.len() == self.n_dof && dv_prev.len() == self.n_dof);
        let dt2 = dt * dt;
        // RHS_free = −(∂r/∂x_prev·dx_prev + ∂r/∂v_prev·dv_prev)_free
        //          = ((M/Δt²)·dx_prev + (M/Δt)·dv_prev)_free.
        // NOTE: under friction xᵗ = x_prev also enters ∂r/∂x_prev (a friction RHS term not
        // yet included here — PR2 gates material + pose; `x_prev` threads the adjoint A).
        let rhs: Vec<f64> = self
            .free_dof_indices
            .iter()
            .map(|&i| {
                self.mass_per_dof[i] / dt2 * dx_prev[i] + self.mass_per_dof[i] / dt * dv_prev[i]
            })
            .collect();
        self.solve_free_and_scatter(x_final, x_prev, dt, rhs)
    }

    /// Build a [`StateStepVjp`] for one converged step — the reverse-mode (tape)
    /// sibling of [`Self::equilibrium_state_sensitivity`]: pushed onto a chassis
    /// tape with `x_prev` and `v_prev` as the two parents, it turns a downstream
    /// `∂L/∂x*` cotangent into `(∂L/∂x_prev, ∂L/∂v_prev)`. This is the primitive
    /// that threads one soft step's adjoint to the previous step's, so one
    /// `tape.backward` can cross step boundaries over a rollout (the multi-step
    /// time-adjoint).
    ///
    /// The op stashes the tangent factored at `x_final` plus the per-DOF scales
    /// `M/Δt²` and `M/Δt`; its VJP solves `A·λ = g_free` (the same adjoint as
    /// [`NewtonStepVjp`]) then writes `∂L/∂x_prev = (M/Δt²)·λ_full`,
    /// `∂L/∂v_prev = (M/Δt)·λ_full`. The pushed node's value should be the
    /// `x_final` tensor (shape `[n_dof]`); its parents the `x_prev` then `v_prev`
    /// `Var`s (each shape `[n_dof]`).
    #[must_use]
    pub fn state_step_vjp(&self, x_final: &[f64], dt: f64) -> StateStepVjp {
        debug_assert!(x_final.len() == self.n_dof);
        let factor = self.factor_at_position(x_final, None, dt, 0.0);
        let dt2 = dt * dt;
        let m_over_dt2: Vec<f64> = self.mass_per_dof.iter().map(|&m| m / dt2).collect();
        let m_over_dt: Vec<f64> = self.mass_per_dof.iter().map(|&m| m / dt).collect();
        StateStepVjp::new(
            factor,
            self.n_dof,
            self.free_dof_indices.clone(),
            m_over_dt2,
            m_over_dt,
        )
    }

    /// Build a [`TrajectoryStepVjp`] — the unified multi-step / soft↔rigid VJP
    /// fusing the prev-state ([`Self::state_step_vjp`]), material
    /// ([`Self::material_step_vjp`]), and contact-pose
    /// ([`Self::equilibrium_pose_sensitivity`]) adjoints into ONE op with a
    /// single shared `A·λ = g_free` solve. Pushed onto a chassis tape with four
    /// parents in order `[x_prev, v_prev, param, pose]`, it lets one
    /// `tape.backward` cross both step boundaries and the soft↔rigid interface
    /// over a coupled rollout (keystone time-adjoint, PR2).
    ///
    /// `param_idx` selects the differentiated material parameter (`NeoHookean`:
    /// `0 = μ`, `1 = λ`); `dir` is the contact primitive's translation direction
    /// (the keystone plane rises along `+ẑ`). The pushed node's value is the
    /// `x_final` tensor (`[n_dof]`); the `param`/`pose` parents are scalars `[1]`,
    /// the state parents are `[n_dof]`. Engaged / stable-active-set / hard-penalty
    /// scope (the active set and factor are captured here).
    #[must_use]
    pub fn trajectory_step_vjp(
        &self,
        x_final: &[f64],
        dt: f64,
        param_idx: usize,
        dir: Vec3,
    ) -> TrajectoryStepVjp {
        debug_assert!(x_final.len() == self.n_dof);
        // Material RHS (∂r/∂param)_free (S5).
        let dr_dp = self.assemble_material_residual_grad(x_final, param_idx);
        let dr_dparam_free: Vec<f64> = self.free_dof_indices.iter().map(|&i| dr_dp[i]).collect();
        // Contact-pose RHS (∂r/∂pose)_free (S3) — shared with the forward
        // `equilibrium_pose_sensitivity` (kept in lockstep). The reverse pose
        // parent is a scalar translation along `dir`; the rotating-normal pose
        // (`ω ≠ 0`) is wired through the reverse path in the coupling leaf (PR2).
        let dr_dpose = self.assemble_pose_residual_grad(x_final, RigidTwist::translation(dir));
        let dr_dpose_free: Vec<f64> = self.free_dof_indices.iter().map(|&i| dr_dpose[i]).collect();
        // State scales (PR1) + the shared factor at x_final.
        let dt2 = dt * dt;
        let m_over_dt2: Vec<f64> = self.mass_per_dof.iter().map(|&m| m / dt2).collect();
        let m_over_dt: Vec<f64> = self.mass_per_dof.iter().map(|&m| m / dt).collect();
        let factor = self.factor_at_position(x_final, None, dt, 0.0);
        TrajectoryStepVjp::new(
            factor,
            self.n_dof,
            self.free_dof_indices.clone(),
            m_over_dt2,
            m_over_dt,
            dr_dparam_free,
            // One pose direction: the scalar translation along `dir`.
            vec![dr_dpose_free],
        )
    }

    /// Like [`Self::trajectory_step_vjp`] but on the **friction-coupled grip** tape:
    /// the soft step gains a fifth scalar parent for the moving-collider tangential
    /// drift `Δ_surf` (along `drift_dir`), and the shared adjoint factor is the
    /// friction-exact Woodbury-corrected tangent (`factor_at_position` with
    /// `Some(x_prev)`), evaluated at this solver's
    /// [`friction_surface_drift`](Self::with_friction_surface_drift) so the adjoint
    /// matches the drift-consistent forward solve (PR3b-1). Pushed with five parents
    /// `[x_prev, v_prev, param, pose, drift]`.
    ///
    /// The drift parent's `∂L/∂Δ_surf = −λ^T·(∂r/∂Δ_surf · drift_dir)_free` reuses the
    /// SAME single shared solve as the other parents — it is the reverse-mode companion
    /// of [`Self::equilibrium_drift_sensitivity`] (the two contract the same RHS with
    /// `A⁻ᵀ`/`A⁻¹`). `x_prev` is the step start `xᵗ`; `pose_dir` / `drift_dir` are the
    /// contact-plane translation and the surface-drift directions (the keystone grip
    /// scene uses `+ẑ` and `+x̂`). Engaged / stable-active-set / hard-penalty scope.
    #[must_use]
    pub fn trajectory_step_vjp_grip(
        &self,
        x_final: &[f64],
        x_prev: &[f64],
        dt: f64,
        param_idx: usize,
        pose_dir: Vec3,
        drift_dir: Vec3,
    ) -> TrajectoryStepVjp {
        // The scalar "param" parent carries the MATERIAL residual sensitivity `∂r/∂p_k`.
        let dr_dparam = self.assemble_material_residual_grad(x_final, param_idx);
        self.trajectory_step_vjp_grip_core(x_final, x_prev, dt, &dr_dparam, &[pose_dir], drift_dir)
    }

    /// Like [`Self::trajectory_step_vjp_grip`] but the pose parent is the 3-vector contact-sphere
    /// **centre** (a moving end-effector riding the arm tip) rather than a single scalar
    /// translation — the friction-grip analog of [`Self::trajectory_step_vjp_twist`] for pure
    /// translations. The pushed node's `pose` parent is `[basis.len()]` (the coupling passes the
    /// 3 translation axes `[x̂, ŷ, ẑ]`), and `∂L/∂pose[k] = −λᵀ·(∂r/∂(centre·e_k))_free` — each
    /// column the SAME normal + friction pose-residual the scalar grip builds, per axis. With a
    /// single-direction basis it reduces to [`Self::trajectory_step_vjp_grip`] exactly.
    #[must_use]
    pub fn trajectory_step_vjp_grip_centre(
        &self,
        x_final: &[f64],
        x_prev: &[f64],
        dt: f64,
        param_idx: usize,
        pose_basis: &[Vec3],
        drift_dir: Vec3,
    ) -> TrajectoryStepVjp {
        let dr_dparam = self.assemble_material_residual_grad(x_final, param_idx);
        self.trajectory_step_vjp_grip_core(x_final, x_prev, dt, &dr_dparam, pose_basis, drift_dir)
    }

    /// The friction-COEFFICIENT (`μ_c`) sibling of [`Self::trajectory_step_vjp_grip_centre`] — the
    /// 3-vector moving-EE centre pose channel with the `μ_c` param RHS (cf.
    /// [`Self::trajectory_step_vjp_grip_fric_coeff`]).
    #[must_use]
    pub fn trajectory_step_vjp_grip_fric_coeff_centre(
        &self,
        x_final: &[f64],
        x_prev: &[f64],
        dt: f64,
        pose_basis: &[Vec3],
        drift_dir: Vec3,
    ) -> TrajectoryStepVjp {
        let dr_dparam = self.assemble_friction_coeff_residual_grad(x_final, x_prev, dt);
        self.trajectory_step_vjp_grip_core(x_final, x_prev, dt, &dr_dparam, pose_basis, drift_dir)
    }

    /// Like [`Self::trajectory_step_vjp_grip`] but the scalar "param" parent is the Coulomb
    /// friction COEFFICIENT `μ_c` rather than a material parameter. The grip node is generic in
    /// its param slot — the reverse pass contracts `−λᵀ·(∂r/∂param)`, agnostic to which scalar
    /// the RHS came from — so this swaps in `∂r/∂μ_c` (see
    /// `assemble_friction_coeff_residual_grad`) and is otherwise byte-identical to the
    /// material grip node (same five parents, same Woodbury factor, same drift/pose/`x_prev`
    /// coupling). The coupling layer pairs this with a `∂fx/∂μ_c = fx/μ_c` term on the friction
    /// REACTION readout, since `μ_c` also scales the platen reaction directly (not only via `x*`).
    #[must_use]
    pub fn trajectory_step_vjp_grip_fric_coeff(
        &self,
        x_final: &[f64],
        x_prev: &[f64],
        dt: f64,
        pose_dir: Vec3,
        drift_dir: Vec3,
    ) -> TrajectoryStepVjp {
        let dr_dparam = self.assemble_friction_coeff_residual_grad(x_final, x_prev, dt);
        self.trajectory_step_vjp_grip_core(x_final, x_prev, dt, &dr_dparam, &[pose_dir], drift_dir)
    }

    /// Like [`Self::trajectory_step_vjp_grip`] but the scalar "param" parent is a single
    /// **design variable** driving a linear combination of the material parameters,
    /// `p_k = p_k(p)` with `param_weights[k] = dp_k/dp` (e.g. the coupling's tied stiffness
    /// scale `μ = p, λ = 4p`, `param_weights = [1, 4]`). It is to
    /// [`Self::trajectory_step_vjp_grip`] exactly what
    /// [`Self::trajectory_step_vjp_combined`] is to [`Self::trajectory_step_vjp`]: the grip
    /// core is generic in its param RHS (it contracts `−λᵀ·(∂r/∂param)`, agnostic to which
    /// scalar produced the RHS), so this swaps in the combined material RHS
    /// `Σ_k weights[k]·(∂r/∂p_k)` and is otherwise byte-identical to the single-material grip
    /// node (same pose/drift/`x_prev`-friction coupling and Woodbury factor). With a unit
    /// weight vector it reduces to `trajectory_step_vjp_grip` at that index exactly. Enables
    /// differentiating the buffer's stiffness AND a control policy on ONE friction-grip tape
    /// (the de-escalation co-design "one outer loop over both"). Same engaged /
    /// stable-active-set / hard-penalty scope.
    #[must_use]
    pub fn trajectory_step_vjp_grip_combined(
        &self,
        x_final: &[f64],
        x_prev: &[f64],
        dt: f64,
        param_weights: &[f64],
        pose_dir: Vec3,
        drift_dir: Vec3,
    ) -> TrajectoryStepVjp {
        let dr_dparam = self.assemble_material_residual_grad_combined(x_final, param_weights);
        self.trajectory_step_vjp_grip_core(x_final, x_prev, dt, &dr_dparam, &[pose_dir], drift_dir)
    }

    /// The moving-end-effector 3-vector centre sibling of [`Self::trajectory_step_vjp_grip_combined`]
    /// (the tied design-variable param RHS with the centre pose basis; cf.
    /// [`Self::trajectory_step_vjp_grip_centre`]).
    #[must_use]
    pub fn trajectory_step_vjp_grip_combined_centre(
        &self,
        x_final: &[f64],
        x_prev: &[f64],
        dt: f64,
        param_weights: &[f64],
        pose_basis: &[Vec3],
        drift_dir: Vec3,
    ) -> TrajectoryStepVjp {
        let dr_dparam = self.assemble_material_residual_grad_combined(x_final, param_weights);
        self.trajectory_step_vjp_grip_core(x_final, x_prev, dt, &dr_dparam, pose_basis, drift_dir)
    }

    /// Shared core of the friction-grip soft VJP node: everything except WHICH scalar the param
    /// parent represents. `dr_dparam` is the full-DOF residual sensitivity for that scalar
    /// (material `∂r/∂p_k` or friction-coefficient `∂r/∂μ_c`); the rest — pose, drift, `x_prev`
    /// friction coupling, state scales, and the Woodbury factor — is identical either way.
    fn trajectory_step_vjp_grip_core(
        &self,
        x_final: &[f64],
        x_prev: &[f64],
        dt: f64,
        dr_dparam: &[f64],
        pose_basis: &[Vec3],
        drift_dir: Vec3,
    ) -> TrajectoryStepVjp {
        debug_assert!(x_final.len() == self.n_dof);
        debug_assert!(x_prev.len() == self.n_dof);
        let dr_dparam_free: Vec<f64> = self
            .free_dof_indices
            .iter()
            .map(|&i| dr_dparam[i])
            .collect();
        // Pose RHS = normal contact pose grad + the FRICTION pose grad (the friction force is
        // linear in the pose-dependent normal force λⁿ — the PR2-deferred term that makes
        // `∂x*/∂pose` friction-exact, load-bearing for the coupled pose↔grip cancellation). One
        // column per pose-basis direction: a single scalar height/translation (`[pose_dir]`) or
        // the 3 translation axes of a moving end-effector centre (`[x̂, ŷ, ẑ]`).
        let dr_dpose_free: Vec<Vec<f64>> = pose_basis
            .iter()
            .map(|&pose_dir| {
                let mut dr_dpose =
                    self.assemble_pose_residual_grad(x_final, RigidTwist::translation(pose_dir));
                let dr_dpose_fric =
                    self.assemble_friction_pose_residual_grad(x_final, x_prev, dt, pose_dir);
                for (d, df) in dr_dpose.iter_mut().zip(&dr_dpose_fric) {
                    *d += df;
                }
                self.free_dof_indices.iter().map(|&i| dr_dpose[i]).collect()
            })
            .collect();
        // Drift RHS (∂r/∂Δ_surf · drift_dir)_free — the friction term's moving-collider
        // dependence (`∂r_v/∂Δ_surf = −∇²D_v`, see `assemble_drift_residual_grad`).
        let dr_ddrift = self.assemble_drift_residual_grad(x_final, x_prev, dt, drift_dir);
        let dr_ddrift_free: Vec<f64> = self
            .free_dof_indices
            .iter()
            .map(|&i| dr_ddrift[i])
            .collect();
        // Friction `x_prev` coupling: `x_start = x_prev + Δ_surf` makes the friction residual
        // depend on `x_prev` (`∂r_v/∂x_prev = −∇²D_v`), so the `x_prev` state cotangent gains
        // `+∇²D_v·λ_v` beyond `M/Δt²·λ`. Per-axis free indices (`None` on a pinned axis, where
        // `λ = 0`) keep a partially-pinned contact vertex's free terms — the same per-DOF
        // handling as the Woodbury / drift assemblies.
        let friction_xprev: Vec<(usize, [Option<usize>; 3], nalgebra::Matrix3<f64>)> = self
            .friction_blocks(x_final, x_prev, dt)
            .into_iter()
            .map(|(v, _grad, hess)| {
                let fi = [
                    self.full_to_free_idx[3 * v],
                    self.full_to_free_idx[3 * v + 1],
                    self.full_to_free_idx[3 * v + 2],
                ];
                (v, fi, hess)
            })
            .collect();
        // State scales + the friction-exact Woodbury factor at x_final (Some(x_prev)).
        let dt2 = dt * dt;
        let m_over_dt2: Vec<f64> = self.mass_per_dof.iter().map(|&m| m / dt2).collect();
        let m_over_dt: Vec<f64> = self.mass_per_dof.iter().map(|&m| m / dt).collect();
        let factor = self.factor_at_position(x_final, Some(x_prev), dt, 0.0);
        TrajectoryStepVjp::new_grip(
            factor,
            self.n_dof,
            self.free_dof_indices.clone(),
            m_over_dt2,
            m_over_dt,
            dr_dparam_free,
            dr_dpose_free,
            dr_ddrift_free,
            friction_xprev,
        )
    }

    /// Like [`Self::trajectory_step_vjp`] but the contact primitive's pose parent is
    /// the full 6-DOF spatial **twist** — one pose direction per [`RigidTwist`] in
    /// `twists` (the rotating-normal leaf, PR2). The pushed node's `pose` parent is
    /// `[twists.len()]`, and `∂L/∂pose[k] = −λ^T·(∂r/∂twist_k)_free` from the same
    /// single shared adjoint solve.
    ///
    /// The coupling passes the 6 canonical spatial-twist basis directions (three
    /// angular, three linear); the coupling-side seam (`PoseTwistSeamVjp`) then maps
    /// the twist cotangent through the rigid body's spatial Jacobian to the state. Each
    /// twist's `∂r/∂twist_k` gathers [`ContactModel::pose_residual_derivative`] over
    /// the active set — the rotating-normal `δn̂ = ω×n̂` term included (vs the
    /// translation-only [`Self::trajectory_step_vjp`]). Same engaged / stable-active-
    /// set / hard-penalty scope; the material/state parents and shared factor are
    /// identical. See `docs/keystone/rotating_normal_recon.md`.
    ///
    /// [`ContactModel::pose_residual_derivative`]: crate::contact::ContactModel::pose_residual_derivative
    #[must_use]
    pub fn trajectory_step_vjp_twist(
        &self,
        x_final: &[f64],
        dt: f64,
        param_idx: usize,
        twists: &[RigidTwist],
    ) -> TrajectoryStepVjp {
        debug_assert!(x_final.len() == self.n_dof);
        let dr_dp = self.assemble_material_residual_grad(x_final, param_idx);
        let dr_dparam_free: Vec<f64> = self.free_dof_indices.iter().map(|&i| dr_dp[i]).collect();
        // One (∂r/∂twist_k)_free per pose direction — shares the forward
        // `assemble_pose_residual_grad` (kept in lockstep with the forward
        // `equilibrium_pose_sensitivity`).
        let dr_dpose_free: Vec<Vec<f64>> = twists
            .iter()
            .map(|&tw| {
                let dr = self.assemble_pose_residual_grad(x_final, tw);
                self.free_dof_indices.iter().map(|&i| dr[i]).collect()
            })
            .collect();
        let dt2 = dt * dt;
        let m_over_dt2: Vec<f64> = self.mass_per_dof.iter().map(|&m| m / dt2).collect();
        let m_over_dt: Vec<f64> = self.mass_per_dof.iter().map(|&m| m / dt).collect();
        let factor = self.factor_at_position(x_final, None, dt, 0.0);
        TrajectoryStepVjp::new(
            factor,
            self.n_dof,
            self.free_dof_indices.clone(),
            m_over_dt2,
            m_over_dt,
            dr_dparam_free,
            dr_dpose_free,
        )
    }

    /// Like [`Self::trajectory_step_vjp`] but the material parent is a single
    /// **design variable** that drives a linear combination of the material
    /// parameters, `p_k = p_k(p)` with `param_weights[k] = dp_k/dp`. The op's
    /// `param` cotangent is then the *total* `∂L/∂p = Σ_k (dp_k/dp)·∂L/∂p_k` from
    /// ONE adjoint solve — letting a tied reparametrization (e.g. the coupling's
    /// stiffness scale `μ = p, λ = 4p`, `param_weights = [1, 4]`) ride one tape
    /// parent so a single `tape.backward` yields its total gradient (rather than
    /// summing two separate `param_idx` backward passes). All other parents
    /// (`x_prev`, `v_prev`, `pose`) and the shared factor are identical to
    /// [`Self::trajectory_step_vjp`]; with a unit weight vector (`1` at one index)
    /// it reduces to that method exactly. Same engaged / stable-active-set /
    /// hard-penalty scope.
    #[must_use]
    pub fn trajectory_step_vjp_combined(
        &self,
        x_final: &[f64],
        dt: f64,
        param_weights: &[f64],
        dir: Vec3,
    ) -> TrajectoryStepVjp {
        debug_assert!(x_final.len() == self.n_dof);
        // Material RHS for the tied design variable: Σ_k weights[k]·(∂r/∂p_k)_free.
        let dr_dp = self.assemble_material_residual_grad_combined(x_final, param_weights);
        let dr_dparam_free: Vec<f64> = self.free_dof_indices.iter().map(|&i| dr_dp[i]).collect();
        // Contact-pose RHS (∂r/∂pose)_free (S3) — identical to the single-param
        // path (scalar translation along `dir`; rotating normal is PR2).
        let dr_dpose = self.assemble_pose_residual_grad(x_final, RigidTwist::translation(dir));
        let dr_dpose_free: Vec<f64> = self.free_dof_indices.iter().map(|&i| dr_dpose[i]).collect();
        let dt2 = dt * dt;
        let m_over_dt2: Vec<f64> = self.mass_per_dof.iter().map(|&m| m / dt2).collect();
        let m_over_dt: Vec<f64> = self.mass_per_dof.iter().map(|&m| m / dt).collect();
        let factor = self.factor_at_position(x_final, None, dt, 0.0);
        TrajectoryStepVjp::new(
            factor,
            self.n_dof,
            self.free_dof_indices.clone(),
            m_over_dt2,
            m_over_dt,
            dr_dparam_free,
            // One pose direction: the scalar translation along `dir`.
            vec![dr_dpose_free],
        )
    }
}

impl<E, Msh, C, M, const N: usize, const G: usize> Solver for CpuNewtonSolver<E, Msh, C, M, N, G>
where
    E: Element<N, G>,
    Msh: Mesh<M>,
    M: Material,
    C: ContactModel + ActivePairsFor<M>,
{
    type Tape = CpuTape;

    fn step(
        &mut self,
        tape: &mut Self::Tape,
        x_prev: &Tensor<f64>,
        v_prev: &Tensor<f64>,
        theta_var: Var,
        dt: f64,
    ) -> NewtonStep<Self::Tape> {
        // Snapshot θ's value off the tape so the forward Newton loop can
        // proceed in pure-tensor space. The tape's role is reverse-mode
        // bookkeeping; the primal solve is tape-free.
        let theta_tensor = tape.value_tensor(theta_var).clone();
        let (mut step, lm_final_lambda) = self.solve_impl(x_prev, v_prev, &theta_tensor, dt);

        // IFT adjoint factor: re-assemble A at x_final (post-convergence)
        // and factor it via `factor_free_tangent` — Llt happy path or
        // A2 Lu fallback per `FactoredFreeTangent`. F3: pass the
        // Newton-final λ as the LM seed (per spec §2.1) so late-iter
        // LM activity warm-starts the adjoint factor; on the disabled
        // path the seed is ignored. Factor ownership pattern (I-3)
        // verified for Llt in tests/invariant_3_factor.rs; the same
        // Arc-internal ownership shape holds for Lu.
        let factor = self.factor_at_position(&step.x_final, None, dt, lm_final_lambda);
        self.push_newton_step_vjp(tape, theta_var, &mut step, factor);
        step
    }

    fn replay_step(
        &self,
        x_prev: &Tensor<f64>,
        v_prev: &Tensor<f64>,
        theta: &Tensor<f64>,
        dt: f64,
    ) -> NewtonStep<Self::Tape> {
        // Pure-function counterpart; no tape mutation. At skeleton scale
        // this is the same primal solve — Phase-E checkpoint replay will
        // diverge by reading a stored primal instead of re-solving.
        // `x_final_var` stays `None` — no tape means no Var. Drops the
        // F3 Newton-final λ: replay path doesn't run the IFT adjoint
        // factor (which is the only consumer of the seed).
        let (step, _lm_final_lambda) = self.solve_impl(x_prev, v_prev, theta, dt);
        step
    }

    // Graceful API: every fail-close (ArmijoStall included) surfaces as
    // `Err`, never a panic — `try_solve_impl` already returns all four
    // variants. The panic-on-fail-close mirror is `step` (a separate
    // `solve_impl` path), which is unaffected by this.
    fn try_step(
        &mut self,
        tape: &mut Self::Tape,
        x_prev: &Tensor<f64>,
        v_prev: &Tensor<f64>,
        theta_var: Var,
        dt: f64,
    ) -> Result<NewtonStep<Self::Tape>, SolverFailure> {
        let theta_tensor = tape.value_tensor(theta_var).clone();
        let (mut step, lm_final_lambda) = self.try_solve_impl(x_prev, v_prev, &theta_tensor, dt)?;
        let factor = self.try_factor_at_position(&step.x_final, None, dt, lm_final_lambda)?;
        self.push_newton_step_vjp(tape, theta_var, &mut step, factor);
        Ok(step)
    }

    // Replay mirror of `try_step` — same unconditional-`Err` contract.
    fn try_replay_step(
        &self,
        x_prev: &Tensor<f64>,
        v_prev: &Tensor<f64>,
        theta: &Tensor<f64>,
        dt: f64,
    ) -> Result<NewtonStep<Self::Tape>, SolverFailure> {
        let (step, _lm_final_lambda) = self.try_solve_impl(x_prev, v_prev, theta, dt)?;
        Ok(step)
    }

    fn current_dt(&self) -> f64 {
        self.config.dt
    }

    fn convergence_tol(&self) -> f64 {
        self.config.tol
    }
}

// ── free functions (assembly kernels) ──────────────────────────────────

/// Per-element deformation gradient `F_ij = Σ_a x_{a,i} · ∂N_a/∂X_j`
/// (direct form).
///
/// Takes 12 element-local position values (vertex-major, xyz-inner)
/// from `extract_element_dof_values`. At rest (`x_a = X_a`), returns
/// the identity.
fn deformation_gradient(x_elem: &[f64; 12], grad_x_n: &SMatrix<f64, 4, 3>) -> Matrix3<f64> {
    let mut f = Matrix3::zeros();
    for a in 0..4 {
        for i in 0..3 {
            let x_ai = x_elem[3 * a + i];
            for j in 0..3 {
                f[(i, j)] += x_ai * grad_x_n[(a, j)];
            }
        }
    }
    f
}

/// Extract one element's 12 vertex-DOF values from the global
/// position vector. Vertex-major + xyz-inner layout: `x_elem[3 * a +
/// k] = x_full[3 * verts[a] + k]`.
fn extract_element_dof_values(x_full: &[f64], verts: &[VertexId; 4]) -> [f64; 12] {
    let mut x_elem = [0.0; 12];
    for a in 0..4 {
        let v = verts[a] as usize;
        x_elem[3 * a] = x_full[3 * v];
        x_elem[3 * a + 1] = x_full[3 * v + 1];
        x_elem[3 * a + 2] = x_full[3 * v + 2];
    }
    x_elem
}

/// Full-DOF residual per scope §5 R-5: `r = (M/Δt²)·(x - x_prev -
/// Δt·v_prev) + f_int(x) - f_ext(θ)`, written into the caller-owned
/// `r` buffer.
///
/// `mass_per_dof` is the diagonal of the lumped mass matrix; for
/// vertices shared across multiple elements this exceeds `ρ V_e / 4`
/// of any single element. `mass / (dt * dt)` (rather than
/// `mass * (1.0 / (dt * dt))`) preserves the pre-Phase-2 1-tet
/// expression order at the last bit.
//
// 8 args mirrors the 8 input slices the residual formula reads from;
// bundling into a struct would add a name-the-fields ceremony for two
// callers with no readability gain.
#[allow(clippy::too_many_arguments)]
fn residual_into(
    x_curr: &[f64],
    x_prev: &[f64],
    v_prev: &[f64],
    f_int: &[f64],
    f_ext: &[f64],
    mass_per_dof: &[f64],
    dt: f64,
    r: &mut [f64],
) {
    debug_assert!(x_curr.len() == r.len());
    debug_assert!(x_prev.len() == r.len());
    debug_assert!(v_prev.len() == r.len());
    debug_assert!(f_int.len() == r.len());
    debug_assert!(f_ext.len() == r.len());
    debug_assert!(mass_per_dof.len() == r.len());
    let dt2 = dt * dt;
    for i in 0..r.len() {
        let x_hat = dt.mul_add(v_prev[i], x_prev[i]);
        let mass_over_dt2 = mass_per_dof[i] / dt2;
        r[i] = mass_over_dt2.mul_add(x_curr[i] - x_hat, f_int[i]) - f_ext[i];
    }
}

/// Standard Armijo-stall panic message text, shared by `solve_impl`,
/// `Solver::try_step`, and `Solver::try_replay_step` so the three
/// panic surfaces emit byte-identical stderr regardless of which
/// entry point tripped the stall. Pulled into a free fn to dodge
/// the multi-line-string-literal indentation trap where continuation
/// lines silently embed leading whitespace into the message text —
/// drifted across the three call sites pre-polish (21-space vs
/// 17-space continuation), making the panic UX site-dependent.
fn armijo_stall_panic_message(last_iter: usize, last_r_norm: f64) -> String {
    format!(
        "Armijo line-search stalled at Newton iter {last_iter} \
         (r_norm {last_r_norm:e}). Likely causes: non-SPD tangent \
         near solution (spec §3 R-2 violation), or near-singular \
         condensed system."
    )
}

/// Maximum diagonal entry magnitude (signed, not abs) across a
/// lower-tri triplet list. F3 uses this to scale the seed and ceiling
/// of the LM `+λI` retry loop relative to the assembled tangent's
/// dominant diagonal (per spec §2.1 / §2.2).
///
/// Returns `0.0` for a triplet list with no diagonal entries — the
/// caller (`factor_free_tangent`) `debug_assert!`s on `> 0`, so an
/// all-off-diagonal list trips fail-fast at the assertion. Production
/// assembly (`assemble_free_hessian_triplets`) always scatters a
/// positive mass-diagonal entry per free DOF, so the structural
/// invariant holds.
fn triplets_max_diag(triplets: &[Triplet<usize, usize, f64>]) -> f64 {
    triplets
        .iter()
        .filter(|t| t.row == t.col)
        .map(|t| t.val)
        .fold(0.0_f64, f64::max)
}

/// Clone a triplet list and add `lambda` to each diagonal entry in
/// place (the F3 `+λI` regularization step per spec §2.1). Off-diagonal
/// entries are copied unchanged. Preserves triplet ordering (the
/// cached symbolic factor depends on it).
///
/// Mutates EXISTING diagonal entries — does NOT append fresh
/// `(k, k, λ)` triplets — because `SparseColMat::try_new_from_triplets`
/// rejects duplicates. The `assemble_free_hessian_triplets` mass
/// scatter (search for `Mass diagonal:`) guarantees a diagonal entry
/// per free DOF, so every `(k, k)` is mutated exactly once.
/// `O(n_triplets)` clone + `O(n_triplets)` walk = `O(n_triplets)`;
/// negligible vs the Llt factor cost.
fn triplets_with_diagonal_offset(
    triplets: &[Triplet<usize, usize, f64>],
    lambda: f64,
) -> Vec<Triplet<usize, usize, f64>> {
    let mut out = triplets.to_vec();
    for t in &mut out {
        if t.row == t.col {
            t.val += lambda;
        }
    }
    out
}

/// Convert a flat `[f64]` DOF buffer (length `3·N`, vertex-major +
/// xyz-inner) to a `Vec<Vec3>` of length `N`.
///
/// Bridges the solver's flat representation to
/// [`crate::contact::ContactModel`]'s `&[Vec3]` argument shape (trait
/// surface unchanged from the pre-penalty era). Allocates per call
/// site — `assemble_global_int_force` and
/// `assemble_free_hessian_triplets` each build their own vec at the
/// contact-dispatch step. Negligible vs FEM assembly even at the
/// Hertzian fixture's finest-level scale.
fn slice_to_vec3s(x_flat: &[f64]) -> Vec<Vec3> {
    debug_assert!(x_flat.len().is_multiple_of(3));
    let n = x_flat.len() / 3;
    let mut out = Vec::with_capacity(n);
    for v in 0..n {
        out.push(Vec3::new(
            x_flat[3 * v],
            x_flat[3 * v + 1],
            x_flat[3 * v + 2],
        ));
    }
    out
}

#[cfg(test)]
mod tests;
