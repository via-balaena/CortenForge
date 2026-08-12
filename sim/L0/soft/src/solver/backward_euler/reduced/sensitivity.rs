//! Reduced-basis equilibrium sensitivities — rung **R1.2**.
//!
//! The implicit-function-theorem gradient of the [`ReducedNewtonSolver`] step, with the
//! basis `Φ` held **constant** (recon §6: the basis's own parameter dependence belongs
//! to the stated validity domain, not to the chain rule).

use nalgebra::DVector;

use crate::contact::{ActivePairsFor, ContactModel};
use crate::element::Element;
use crate::material::Material;
use crate::mesh::Mesh;
use crate::readout::LoadAxis;
use crate::solver::SolverFailure;

use super::newton::ReducedNewtonSolver;

/// The adjoint of one converged reduced step, bound to the configuration it was solved
/// at.
///
/// Produced by [`ReducedNewtonSolver::adjoint`] and consumed by
/// [`ReducedNewtonSolver::load_gradient`] / [`ReducedNewtonSolver::material_gradient`].
/// It is worth materialising rather than folding into each gradient because the
/// factorization behind it is the expensive part: one adjoint serves every parameter.
///
/// It carries the configuration privately, and there is no way to build one except
/// through `adjoint`. That is deliberate: an adjoint and the `∂r/∂p` it contracts
/// against must be evaluated at the **same** state, and a signature taking both an
/// adjoint and a loose `x` invites a caller to mix two linearization points — which
/// produces a plausible wrong gradient rather than an error.
#[derive(Clone, Debug)]
pub struct ReducedAdjoint {
    /// `μ = Φ λ_r` — the adjoint expressed over the solver's **free** DOFs.
    ///
    /// This is the object every parameter contraction actually uses, and the reason
    /// the reduced gradient is so cheap to reason about: **`μ` is a drop-in for the
    /// full-order adjoint `λ`.** Each reduced gradient is the full-order formula with
    /// `λ` replaced by `μ` — nothing else changes.
    ///
    /// `μ` is not an arbitrary approximation of `λ`, either. Galerkin on a symmetric
    /// positive-definite `A` makes `μ` the **best approximation to `λ` in the energy
    /// (`A`) norm** over `span(Φ)`. So the reduced gradient's error is the adjoint
    /// field's own representability in the basis — which is a different question from
    /// the *displacement* field's representability that R1.0 measured, and is not
    /// implied by it. A POD basis fitted to displacement snapshots was never asked to
    /// represent an adjoint.
    pub mu_free: Vec<f64>,
    /// The full-DOF configuration this adjoint was solved at — see the type's docs for
    /// why it is carried rather than re-passed.
    configuration: Vec<f64>,
}

impl<E, Msh, C, M, const N: usize, const G: usize> ReducedNewtonSolver<'_, E, Msh, C, M, N, G>
where
    E: Element<N, G>,
    Msh: Mesh<M>,
    M: Material,
    C: ContactModel + ActivePairsFor<M>,
{
    /// Solve the reduced adjoint system `A_r λ_r = Φᵀ g_free` at configuration `x`.
    ///
    /// `cotangent` is the objective's derivative `∂J/∂x` over **full** DOFs (length
    /// `n_dof`), matching the cotangent the crate's full-order VJPs take; its free-DOF
    /// entries are gathered and the pinned ones ignored. `x` is a converged
    /// configuration — [`ReducedNewtonSolver::expand_to_full`] of a converged `q` for
    /// the reduced model's own gradient, or the oracle's `x_final` to price the
    /// projection in isolation.
    ///
    /// ## Why the cotangent is projected with `Φᵀ` and not `ΦᵀM`
    ///
    /// `∂J/∂x` is a **covector**, exactly like a residual: `Φᵀg` pairs it with the
    /// Jacobian `ΦᵀAΦ` that the reduced Newton solve already factors. Using the
    /// mass-weighted displacement projection would solve `ΦᵀAΦ λ = ΦᵀMg`, whose answer
    /// is a plausible-looking wrong gradient — no line search stalls to announce it,
    /// because there is no line search here. This is the third object in the arc with
    /// this hazard (displacement, residual, adjoint) and the only one with no forward
    /// symptom.
    ///
    /// ## `x_prev`
    ///
    /// Threaded through to the tangent exactly as the forward solve threads it. Pass
    /// `Some(x_prev)` to differentiate the map [`ReducedNewtonSolver::step`] actually
    /// solves: with friction active the forward residual depends on `x` through `∇D`,
    /// so `∂r/∂x` includes `∇²D` and only the `Some` arm is the true Jacobian. (The
    /// crate's full-order differentiable path passes `None` by a separate convention;
    /// under [`NullContact`](crate::contact::NullContact) the two agree exactly.)
    ///
    /// ## Cost
    ///
    /// One `ΦᵀAΦ` build plus one dense `r × r` factorization — the same work as a
    /// single reduced Newton iteration. The tangent is **not** reused from the forward
    /// solve, which drops it; for a single gradient that trade is right, and a consumer
    /// taking many parameters against one state should hold the returned
    /// [`ReducedAdjoint`] rather than call this per parameter.
    ///
    /// # Errors
    /// [`SolverFailure::DoublyFailedFactor`] if the reduced tangent fails both Cholesky
    /// and LU — the same cold fallback the forward reduced step keeps.
    ///
    /// # Panics
    /// Panics if `cotangent` is not `n_dof` long.
    pub fn adjoint(
        &self,
        x: &[f64],
        x_prev: Option<&[f64]>,
        dt: f64,
        cotangent: &[f64],
    ) -> Result<ReducedAdjoint, SolverFailure> {
        assert!(
            cotangent.len() == self.full.n_dof,
            "cotangent has {} entries, expected n_dof = {}",
            cotangent.len(),
            self.full.n_dof,
        );
        let g_free = self.gather_free(cotangent);
        let g_r = self.basis.project_covector(&g_free);
        let a_r = self.project_tangent(x, x_prev, dt);
        let rhs = DVector::from_vec(g_r);

        let solved = a_r
            .clone()
            .cholesky()
            .map_or_else(|| a_r.clone().lu().solve(&rhs), |ch| Some(ch.solve(&rhs)));
        let Some(lambda) = solved else {
            return Err(SolverFailure::DoublyFailedFactor {
                x_partial: x.to_vec(),
                last_iter: 0,
                context: format!(
                    "reduced adjoint tangent (r = {}) failed both Cholesky and LU",
                    self.basis.n_modes()
                ),
            });
        };

        // μ = Φ λ_r. Written out rather than routed through `PodBasis::reconstruct`:
        // that method reconstructs a DISPLACEMENT from its coordinates, and while the
        // arithmetic here is identical, `λ_r` is a covector's coordinates. Borrowing
        // the displacement name would put the arc's one recurring bug class back into
        // the reader's blind spot.
        let mut mu_free = vec![0.0_f64; self.basis.n_free()];
        for (phi, lk) in self.basis.modes().iter().zip(lambda.iter()) {
            for (m, p) in mu_free.iter_mut().zip(phi) {
                *m += lk * p;
            }
        }
        Ok(ReducedAdjoint {
            mu_free,
            configuration: x.to_vec(),
        })
    }

    /// `dJ/dθ` — the reduced gradient w.r.t. the **load** parameter, from an adjoint
    /// built by [`Self::adjoint`].
    ///
    /// The load enters the residual only through `r = … − f_ext(θ)`, and `f_ext` copies
    /// `θ` onto the loaded DOFs, so `∂r/∂θ_j = −e_k` for the loaded free DOF `k`. The
    /// contraction `−μᵀ(∂r/∂θ_j)` therefore collapses to reading `μ` at that DOF —
    /// the full-order load adjoint's `λ[k]` with `μ` in place of `λ`.
    ///
    /// The returned length follows the load map's convention, matching what
    /// `assemble_external_force` consumes: `1` for an all-`AxisZ` (broadcast magnitude)
    /// scene, `3 · n_loaded` for an all-`FullVector` one.
    ///
    /// # Panics
    /// Panics on a mixed-axis load map (out of scope, as in the forward assembly), or
    /// if a loaded vertex is not free — both are scene-wiring errors that boundary-
    /// condition validation already rejects at construction.
    //
    // expect_used: the `full_to_free_idx` lookup mirrors `push_newton_step_vjp`'s, and
    // rests on the same invariant — `new()` asserts `loaded ∩ pinned = ∅`, so a loaded
    // vertex always has a free index. A `None` here is a broken construction-time
    // invariant, not a runtime condition a caller could handle.
    #[allow(clippy::expect_used)]
    #[must_use]
    pub fn load_gradient(&self, adjoint: &ReducedAdjoint) -> Vec<f64> {
        let loaded = &self.full.boundary_conditions.loaded_vertices;
        let all_axis_z = loaded.iter().all(|(_, ax)| matches!(ax, LoadAxis::AxisZ));
        let all_full_vec = loaded
            .iter()
            .all(|(_, ax)| matches!(ax, LoadAxis::FullVector));
        assert!(
            all_axis_z || all_full_vec,
            "Mixed-axis loaded_vertices are out of scope; got {loaded:?}"
        );

        let free_of = |full_dof: usize| -> f64 {
            let k = self.full.full_to_free_idx[full_dof]
                .expect("a loaded vertex must be free (boundary-condition validation)");
            adjoint.mu_free[k]
        };

        if all_axis_z {
            let mut acc = 0.0;
            for &(vid, _) in loaded {
                acc += free_of(3 * vid as usize + 2);
            }
            vec![acc]
        } else {
            let mut out = Vec::with_capacity(3 * loaded.len());
            for &(vid, _) in loaded {
                let v = 3 * vid as usize;
                out.extend([free_of(v), free_of(v + 1), free_of(v + 2)]);
            }
            out
        }
    }

    /// `dJ/dp_k` — the reduced gradient w.r.t. a scalar **material** parameter, from an
    /// adjoint built by [`Self::adjoint`].
    ///
    /// The configuration is taken from `adjoint`, not re-passed: `∂r/∂p_k` must be
    /// assembled at the state the adjoint was solved at, and letting a caller supply it
    /// separately would make a mismatch expressible.
    ///
    /// Material parameters enter the residual only through the elastic internal force,
    /// so `∂r/∂p_k = ∂f_int/∂p_k` — the *same* assembly the full-order
    /// `equilibrium_material_sensitivity` and `material_step_vjp` use, contracted as
    /// `−μᵀ(∂r/∂p_k)_free`. Reusing that assembly is deliberate: the reduced path must
    /// not acquire a second copy of the stress derivative to drift from.
    ///
    /// `param_idx` follows the material's own convention (`NeoHookean`: `0 = μ`,
    /// `1 = λ`); a material exposing no differentiable parameters contributes zero.
    ///
    /// # Panics
    /// Panics if `param_idx` is out of range for a material that does expose parameters.
    #[must_use]
    pub fn material_gradient(&self, adjoint: &ReducedAdjoint, param_idx: usize) -> f64 {
        let dr_dp = self
            .full
            .assemble_material_residual_grad(&adjoint.configuration, param_idx);
        let dr_dp_free = self.gather_free(&dr_dp);
        -adjoint
            .mu_free
            .iter()
            .zip(&dr_dp_free)
            .map(|(m, d)| m * d)
            .sum::<f64>()
    }
}
