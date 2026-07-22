//! Free-tangent factorization (Llt / Lu fallback / Woodbury friction)
//! for [`CpuNewtonSolver`](super::CpuNewtonSolver).

use std::borrow::Cow;

use faer::linalg::solvers::SolveCore;
use faer::prelude::Reborrow;
use faer::sparse::linalg::LltError;
use faer::sparse::linalg::solvers::{Llt, Lu};
use faer::sparse::{SparseColMat, Triplet};
use faer::{Conj, MatMut, Side};
use nalgebra::{DMatrix, DVector, Matrix3};

use crate::contact::{ActivePairsFor, ContactModel};
use crate::element::Element;
use crate::material::Material;
use crate::mesh::Mesh;
use crate::solver::SolverFailure;
use crate::solver::lm::LmState;

use super::CpuNewtonSolver;
use super::helpers::{slice_to_vec3s, triplets_max_diag, triplets_with_diagonal_offset};

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
pub(super) struct DoublyFailedFactorInfo {
    /// Full panic-message-style context: factor site + Newton iter +
    /// Lu error details. Preserves the bit-equal panic message at
    /// `solve_impl`'s translation site.
    pub(super) context: String,
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
pub struct FactoredFreeTangent {
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

impl<E, Msh, C, M, const N: usize, const G: usize> CpuNewtonSolver<E, Msh, C, M, N, G>
where
    E: Element<N, G>,
    Msh: Mesh<M>,
    M: Material,
    C: ContactModel + ActivePairsFor<M>,
{
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
    pub(super) fn factor_free_tangent(
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
    pub(super) fn factor_at_position(
        &self,
        x_curr: &[f64],
        x_prev: Option<&[f64]>,
        dt: f64,
        lm_seed_lambda: f64,
    ) -> FactoredFreeTangent {
        assert!(
            !self.config.fbar,
            "F-bar differentiable gradients are not yet supported (forward-only, PR1): the \
             adjoint tangent IS F-bar-consistent, but the VJP RHS assembly (material-parameter \
             and load channels) still evaluates at the unmodified F, not F*, so a gradient here \
             would be silently wrong. Use `replay_step` for forward-only F-bar; the \
             differentiability leaf (PR2) carries F* into the adjoint RHS."
        );
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
    pub(super) fn try_factor_and_solve_free(
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

    /// Graceful-failure mirror of [`Self::factor_at_position`] (F3.3
    /// per spec §2.5). Used by [`Self::try_step`] for the IFT-adjoint
    /// factor at converged `x_final`. On doubly-failed factor returns
    /// `Err(SolverFailure::DoublyFailedFactor)` with
    /// `x_partial = x_curr` (which IS `x_final` here — the natural
    /// partial position) and `last_iter = 0` (no Newton iter applies
    /// post-Newton; the `context` string carries the
    /// `"factor_at_position (IFT adjoint at x_final)"` site tag for
    /// disambiguation).
    pub(super) fn try_factor_at_position(
        &self,
        x_curr: &[f64],
        x_prev: Option<&[f64]>,
        dt: f64,
        lm_seed_lambda: f64,
    ) -> Result<FactoredFreeTangent, SolverFailure> {
        assert!(
            !self.config.fbar,
            "F-bar differentiable gradients are not yet supported (forward-only, PR1): the \
             adjoint tangent IS F-bar-consistent, but the VJP RHS assembly (material-parameter \
             and load channels) still evaluates at the unmodified F, not F*, so a gradient here \
             would be silently wrong. Use `replay_step` for forward-only F-bar; the \
             differentiability leaf (PR2) carries F* into the adjoint RHS."
        );
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
    pub(super) fn solve_free_and_scatter(
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
}
