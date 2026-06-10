//! `CpuDifferentiable` — CPU-backend `Differentiable` impl — plus the
//! `NewtonStepVjp: VjpOp` type that plugs into chassis `Tape::push_custom`.
//!
//! **Math (scope §5, Part 6 Ch 02:13–15).** At equilibrium `r(x*; θ) = 0`,
//! the implicit function theorem gives
//!
//! ```text
//!     ∂x*/∂θ  =  -A⁻¹ · ∂r/∂θ          where  A = ∂r/∂x |_(x*, θ)
//! ```
//!
//! For a scalar loss `L(x*)` with upstream cotangent `g = ∂L/∂x*`, the
//! reverse-mode VJP is
//!
//! ```text
//!     ∂L/∂θ  =  g^T · ∂x*/∂θ  =  -g^T · A⁻¹ · ∂r/∂θ
//! ```
//!
//! Two-step factor-reuse form (avoids materializing `A⁻¹`):
//!
//! 1. Solve  `A · λ = g_free`  (A is symmetric for `NeoHookean` — the same
//!    factor the forward Newton step converged with).
//! 2. Contract  `∂L/∂θ = -λ^T · (∂r/∂θ)_free`.
//!
//! **Stage-1 `∂r/∂θ` (multi-loaded-vertex generalization).** θ is a
//! length-1 scalar broadcast as `+ẑ` traction magnitude to every
//! vertex in `BC.loaded_vertices`. For each loaded vertex `v` with
//! `LoadAxis::AxisZ`, `f_ext[3v + 2] = θ`, so
//!
//! ```text
//!     ∂f_ext/∂θ = Σ_v e_{3v+2}     →     (∂r/∂θ)_free has -1 at each
//!                                         loaded vertex's z free-DOF index
//! ```
//!
//! Substituting: `grad_θ = -λ^T · (-Σ_v e_{free(v.z)}) = +Σ_v λ[free(v.z)]`.
//! For the 1-tet skeleton (one loaded vertex `v_3`), this reduces to
//! `+λ[free(v_3.z)]` — bit-equal to the pre-multi-element form.
//!
//! **Stage-2 `∂r/∂θ`.** θ is `[3 · n_loaded]` per-vertex traction
//! triples. For loaded vertex `v` at θ-index `i`, `f_ext[3v + axis] =
//! θ[3i + axis]` for axis ∈ 0..3, so
//!
//! ```text
//!     (∂r/∂θ)_free = -I_{3 · n_loaded} on the loaded-vertices' free DOFs
//! ```
//!
//! Substituting: `grad_θ[3i + axis] = +λ[free(v_i.axis)]` per loaded
//! vertex per axis. For the 1-tet skeleton (one loaded vertex
//! `v_3`), this reduces to `+λ` over `(v_3.x, v_3.y, v_3.z)` —
//! bit-equal to the pre-multi-element form.
//!
//! **Stage discriminator.** Pre-computed `stage_1: bool` set at
//! construction from `BC.loaded_vertices`'s `LoadAxis` variants (all
//! `AxisZ` → Stage 1; all `FullVector` → Stage 2). Mixed-axis BCs are
//! rejected at solver construction (per `assemble_external_force`),
//! so `stage_1` is unambiguous.
//!
//! **Factor reuse.** The `FactoredFreeTangent` stashed here comes from
//! `CpuNewtonSolver::factor_at_position` at `x_final`, not from the last
//! Newton-iter factor (which was at the pre-convergence iterate). It
//! is the happy-path Cholesky in the common case, or the A2 LU
//! fallback when the tangent at `x_final` is indefinite. For
//! `NeoHookean` (and current Yeoh), `A = A^T` — the same factor serves
//! both forward-solve and adjoint-solve. A future asymmetric material
//! would need `solve_transpose_in_place` here. I-3 (factor ownership)
//! verified operationally in `tests/invariant_3_factor.rs`.

// `register_vjp`, `ift_adjoint`, `time_adjoint`, `fd_wrapper` are all
// `unimplemented!("skeleton phase 2 — ...")` by design (BF-4 / BF-6 +
// Phase E+/G deferrals); real bodies land when their phases do.
#![allow(clippy::unimplemented)]

use std::fmt;

use faer::{Conj, MatMut};
use sim_ml_chassis::{Tensor, autograd::VjpOp};

use super::{Differentiable, TapeNodeKey};
use crate::readout::GradientEstimate;
use crate::solver::{CpuTape, FactoredFreeTangent, NewtonStep};

/// CPU-backend `Differentiable` impl. Stateless at type level;
/// `NewtonStepVjp` holds primal data per tape node.
#[derive(Clone, Copy, Debug, Default)]
pub struct CpuDifferentiable;

impl Differentiable for CpuDifferentiable {
    type Tape = CpuTape;

    fn register_vjp(&mut self, _forward_key: TapeNodeKey, _vjp: Box<dyn VjpOp>) {
        unimplemented!(
            "skeleton phase 2 — BF-4 stub: chassis uses push_custom, not a key-indexed registry"
        )
    }

    fn ift_adjoint(
        &self,
        _tape: &Self::Tape,
        _step: &NewtonStep<Self::Tape>,
        _upstream: &Tensor<f64>,
    ) -> Tensor<f64> {
        unimplemented!("skeleton phase 2 — step 6 wraps tape.backward + theta-grad readback")
    }

    fn time_adjoint(
        &self,
        _tape: &Self::Tape,
        _rollout: &[NewtonStep<Self::Tape>],
        _upstream: &Tensor<f64>,
    ) -> Tensor<f64> {
        unimplemented!("skeleton phase 2 — time-adjoint is Phase E+")
    }

    fn fd_wrapper(
        &self,
        _forward: &dyn Fn(&Tensor<f64>) -> Tensor<f64>,
        _theta: &Tensor<f64>,
    ) -> (Tensor<f64>, GradientEstimate) {
        unimplemented!("skeleton phase 2 — fd_wrapper is Phase G stochastic-adjoint")
    }
}

// ── NewtonStepVjp ─────────────────────────────────────────────────────────

/// Custom VJP for one converged Newton step.
///
/// Stashes the faer factor of `A = ∂r/∂x` at `x_final` (either the
/// happy-path `Llt` or the A2 LU fallback — the variant is held
/// inside a `FactoredFreeTangent` enum, crate-private), plus the
/// metadata needed to gather `g_free` from the cotangent and dispatch
/// the per-stage closed-form contraction (Decision I generalization).
///
/// `vjp` solves the adjoint system `A · λ = g_free` and contracts
/// against `(∂r/∂θ)_free` per the module-level math:
/// - Stage 1 (all-AxisZ BC): `parent_cot[0] += Σ_v λ[free(v.z)]` over
///   loaded vertices `v`.
/// - Stage 2 (all-FullVector BC): per-loaded-vertex triplet
///   `parent_cot[3i + axis] += λ[free(v_i.axis)]`.
pub struct NewtonStepVjp {
    factor: FactoredFreeTangent,
    /// Total DOF count, asserted on the cotangent shape.
    n_dof: usize,
    /// Full-DOF indices of the free DOFs, in ascending free-index
    /// order. Used to gather `g_free` from the cotangent.
    free_dof_indices: Vec<usize>,
    /// For each loaded vertex (in `BC.loaded_vertices` order), the
    /// free-DOF indices of its xyz components. Pre-computed at
    /// construction so `vjp` doesn't need `full_to_free_idx` at
    /// runtime. All loaded vertices are fully free per BC validation
    /// (`loaded ∩ pinned = ∅`).
    loaded_free_xyz: Vec<[usize; 3]>,
    /// Stage discriminator: `true` when all loaded vertices have
    /// `LoadAxis::AxisZ` (Stage 1, broadcast scalar θ); `false` when
    /// all `LoadAxis::FullVector` (Stage 2, per-vertex triplet θ).
    /// Mixed-axis BCs are rejected at solver-construction time.
    stage_1: bool,
}

impl NewtonStepVjp {
    /// Construct a VJP for one converged Newton step.
    ///
    /// `factor` is the `FactoredFreeTangent` of `A = ∂r/∂x` at
    /// `x_final` — either the happy-path Cholesky or the A2 LU
    /// fallback. `free_dof_indices` is the full-DOF index list of free
    /// DOFs (in ascending free-index order). `loaded_free_xyz` is
    /// per-loaded-vertex xyz free indices, pre-computed via
    /// `full_to_free_idx` from the solver. `stage_1` is the BC's
    /// load-axis discriminator.
    ///
    /// Crate-private: the only caller is `CpuNewtonSolver::step`, and
    /// the `FactoredFreeTangent` parameter is itself `pub(crate)`.
    #[must_use]
    pub(crate) const fn new(
        factor: FactoredFreeTangent,
        n_dof: usize,
        free_dof_indices: Vec<usize>,
        loaded_free_xyz: Vec<[usize; 3]>,
        stage_1: bool,
    ) -> Self {
        Self {
            factor,
            n_dof,
            free_dof_indices,
            loaded_free_xyz,
            stage_1,
        }
    }
}

// `FactoredFreeTangent` wraps faer factors that don't derive Debug;
// hand-roll a minimal impl so the `VjpOp` `Debug` supertrait bound is
// satisfied without leaking factor internals.
impl fmt::Debug for NewtonStepVjp {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("NewtonStepVjp")
            .field("op_id", &"sim_soft::NewtonStepVjp")
            .field("factor", &"<FactoredFreeTangent>")
            .field("n_dof", &self.n_dof)
            .field("n_free", &self.free_dof_indices.len())
            .field("n_loaded", &self.loaded_free_xyz.len())
            .field("stage_1", &self.stage_1)
            .finish()
    }
}

impl VjpOp for NewtonStepVjp {
    fn op_id(&self) -> &'static str {
        "sim_soft::NewtonStepVjp"
    }

    // The hot-path VJP body. Assertions panic on shape mismatches because
    // those are programmer bugs (wrong output shape was pushed, wrong
    // parent shape registered) — not runtime-recoverable conditions.
    #[allow(clippy::panic)]
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        assert!(
            cotangent.shape() == [self.n_dof],
            "NewtonStepVjp: cotangent must have shape [{}], got {:?}",
            self.n_dof,
            cotangent.shape(),
        );
        assert!(
            parent_cotans.len() == 1,
            "NewtonStepVjp: expected 1 parent (theta_var), got {}",
            parent_cotans.len(),
        );

        let n_loaded = self.loaded_free_xyz.len();
        let expected_parent_dim = if self.stage_1 { 1 } else { 3 * n_loaded };
        assert!(
            parent_cotans[0].shape() == [expected_parent_dim],
            "NewtonStepVjp: θ cotangent must have shape [{expected_parent_dim}] \
             ({} loaded vertex/vertices, stage_1 = {}); got {:?}",
            n_loaded,
            self.stage_1,
            parent_cotans[0].shape(),
        );

        // Gather `g_free` from the cotangent at the free DOF indices.
        let cot = cotangent.as_slice();
        let mut rhs: Vec<f64> = self.free_dof_indices.iter().map(|&idx| cot[idx]).collect();
        let n_free = rhs.len();

        // Solve `A · λ = g_free` in place via the stashed factor (Llt
        // or — under A2 fallback — Lu). `A` is symmetric for
        // `NeoHookean`, so we use the same factor shape the forward
        // Newton step produced — no transpose needed. (Asymmetric
        // materials would need `solve_transpose_in_place`.)
        let rhs_mat: MatMut<'_, f64> = MatMut::from_column_major_slice_mut(&mut rhs, n_free, 1);
        self.factor.solve_in_place_with_conj(Conj::No, rhs_mat);
        // `rhs` now holds λ (in free-DOF indexing).

        let parent_slice = parent_cotans[0].as_mut_slice();
        if self.stage_1 {
            // Stage 1 (broadcast scalar θ): grad_θ = +Σ_v λ[free(v.z)]
            // over loaded vertices. Each loaded vertex's z-axis free
            // index lives at `loaded_free_xyz[v][2]`.
            for xyz in &self.loaded_free_xyz {
                parent_slice[0] += rhs[xyz[2]];
            }
        } else {
            // Stage 2 (per-vertex triplet θ): grad_θ[3i + axis] =
            // +λ[free(v_i.axis)] for each loaded vertex i and axis 0..3.
            for (i, xyz) in self.loaded_free_xyz.iter().enumerate() {
                parent_slice[3 * i] += rhs[xyz[0]];
                parent_slice[3 * i + 1] += rhs[xyz[1]];
                parent_slice[3 * i + 2] += rhs[xyz[2]];
            }
        }
    }
}

// ── MaterialStepVjp ───────────────────────────────────────────────────────

/// Custom VJP differentiating `x*` w.r.t. a scalar **material parameter** `p_k`
/// (keystone S5) — the material-parameter sibling of [`NewtonStepVjp`].
///
/// Where [`NewtonStepVjp`] differentiates the converged step w.r.t. the load θ,
/// this differentiates it w.r.t. a material parameter. The material parameters
/// enter the residual only through the elastic internal force, so
/// `∂r/∂p_k = ∂f_int/∂p_k` (assembled from the stress derivative `∂P/∂p_k`); the
/// IFT adjoint is then the SAME two-step factor-reuse as the load VJP — solve
/// `A · λ = g_free` with the tangent factored at `x_final`, then contract
/// `∂L/∂p_k = −λ^T · (∂r/∂p_k)_free`. The only difference from [`NewtonStepVjp`]
/// is the RHS factor: a *computed* `(∂r/∂p_k)_free` vector rather than the load
/// case's `−1` on the loaded DOFs.
///
/// Constructed by `CpuNewtonSolver::material_step_vjp`; the single output is the
/// scalar `∂L/∂p_k` accumulated into the material-parameter parent.
pub struct MaterialStepVjp {
    factor: FactoredFreeTangent,
    /// Total DOF count, asserted on the cotangent shape.
    n_dof: usize,
    /// Free-DOF full indices (ascending), to gather `g_free` from the cotangent.
    free_dof_indices: Vec<usize>,
    /// `(∂r/∂p_k)_free` — the material-residual sensitivity gathered onto the
    /// free DOFs, in `free_dof_indices` order. The contraction RHS.
    dr_dp_free: Vec<f64>,
}

impl MaterialStepVjp {
    /// Construct a material-parameter VJP for one converged Newton step.
    /// Crate-private: built only by [`CpuNewtonSolver::material_step_vjp`].
    #[must_use]
    pub(crate) fn new(
        factor: FactoredFreeTangent,
        n_dof: usize,
        free_dof_indices: Vec<usize>,
        dr_dp_free: Vec<f64>,
    ) -> Self {
        debug_assert_eq!(free_dof_indices.len(), dr_dp_free.len());
        Self {
            factor,
            n_dof,
            free_dof_indices,
            dr_dp_free,
        }
    }
}

// Hand-rolled Debug: `factor` (faer factors) doesn't derive Debug, and the
// `free_dof_indices` / `dr_dp_free` vectors are summarized by their shared
// length (`n_free`) rather than dumped. `finish_non_exhaustive` records that
// fields are intentionally elided.
impl fmt::Debug for MaterialStepVjp {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MaterialStepVjp")
            .field("op_id", &"sim_soft::MaterialStepVjp")
            .field("factor", &"<FactoredFreeTangent>")
            .field("n_dof", &self.n_dof)
            .field("n_free", &self.free_dof_indices.len())
            .finish_non_exhaustive()
    }
}

impl VjpOp for MaterialStepVjp {
    fn op_id(&self) -> &'static str {
        "sim_soft::MaterialStepVjp"
    }

    // Shape mismatches are programmer bugs (wrong output/parent shape pushed).
    #[allow(clippy::panic)]
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        assert!(
            cotangent.shape() == [self.n_dof],
            "MaterialStepVjp: cotangent must have shape [{}], got {:?}",
            self.n_dof,
            cotangent.shape(),
        );
        assert!(
            parent_cotans.len() == 1 && parent_cotans[0].shape() == [1],
            "MaterialStepVjp: expected 1 scalar parent (the material param [1]); \
             got {} parents, shape {:?}",
            parent_cotans.len(),
            parent_cotans.first().map(Tensor::shape),
        );

        // g_free, then solve A · λ = g_free with the factor at x_final.
        let cot = cotangent.as_slice();
        let mut rhs: Vec<f64> = self.free_dof_indices.iter().map(|&idx| cot[idx]).collect();
        let n_free = rhs.len();
        let rhs_mat: MatMut<'_, f64> = MatMut::from_column_major_slice_mut(&mut rhs, n_free, 1);
        self.factor.solve_in_place_with_conj(Conj::No, rhs_mat);
        // rhs now holds λ (free-DOF indexing). grad_p = −λ^T · (∂r/∂p)_free.
        let mut grad = 0.0_f64;
        for (lam, drdp) in rhs.iter().zip(&self.dr_dp_free) {
            grad -= lam * drdp;
        }
        parent_cotans[0].as_mut_slice()[0] += grad;
    }
}
