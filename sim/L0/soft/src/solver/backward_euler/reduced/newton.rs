//! Galerkin-projected Newton on a POD subspace — rung **R1.1**.

use nalgebra::{DMatrix, DVector};
use sim_ml_chassis::Tensor;

use crate::contact::{ActivePairsFor, ContactModel};
use crate::element::Element;
use crate::material::Material;
use crate::mesh::Mesh;
use crate::solver::SolverFailure;

use super::super::CpuNewtonSolver;
use super::super::helpers::residual_into;
use super::pod::PodBasis;

/// Armijo sufficient-decrease constant — the full solver's value, so the two
/// globalisations differ only in the norm they descend.
const ARMIJO_C1: f64 = 1e-4;

/// One converged reduced step.
#[derive(Clone, Debug)]
pub struct ReducedStep {
    /// Converged reduced coordinates.
    pub q: Vec<f64>,
    /// Backward-Euler reduced velocity, `(q − q_prev) / dt`.
    pub qdot: Vec<f64>,
    /// Newton iterations taken.
    pub iter_count: usize,
    /// `‖Φᵀr_free‖` at convergence — the quantity the reduced solve drives to zero.
    pub projected_residual_norm: f64,
    /// `‖r_free‖` at convergence — the **full** residual of the reduced state.
    ///
    /// This does **not** go to zero, and that is not a defect. A Galerkin solve makes
    /// the residual *orthogonal to the basis*, not zero; whatever lives outside the
    /// span is invisible to it. The ratio
    /// `projected_residual_norm / full_residual_norm` is therefore a direct measure of
    /// how much of the equations the basis can see, and it is reported rather than
    /// asserted because no useful bound on it is known yet.
    pub full_residual_norm: f64,
}

/// A reduced-order Newton solver: a **wrapper** on a full-order solver plus a basis.
///
/// It borrows the full solver rather than replacing it, because R1.1 has no
/// hyper-reduction — every element is still swept to build `f_int` and `K`, and only
/// the *solve* is reduced. That is deliberate (it isolates "does a subspace represent
/// this deformation" from "can the element sweep be made cheap", which is R3), and it
/// has a useful consequence: the constitutive model is preserved **exactly, by
/// construction**, since the same assembly runs. There is no second copy of the physics
/// to drift.
///
/// ## What is solved
///
/// With `x = x_rest + Φq` on the free DOFs (constrained DOFs take their `x_prev`
/// values, exactly as in the full solve), the reduced problem is
///
/// ```text
///     r_r(q) = Φᵀ r_free(x_rest + Φq) = 0,    A_r = Φᵀ A_free Φ
/// ```
///
/// solved by Newton with a dense `r × r` factorization and Armijo backtracking on
/// `‖r_r‖`.
///
/// ## ⚠ Scope: the constrained DOFs must not move
///
/// Constrained (pinned / roller) DOFs are taken from the **rest** configuration on every
/// step, because that is what the reduced state can reconstruct — `q` spans the free
/// DOFs only. A scene that *drives* a constrained DOF (a prescribed-displacement
/// indenter, say) would therefore have its prescribed motion **silently ignored** here,
/// and would produce a plausible-looking wrong trajectory rather than an error.
///
/// R1.1's fixture loads through `theta` precisely so the constrained set is fixed (see
/// `tests/reduced_pod_basis.rs`, which explains why a moving Dirichlet patch also breaks
/// basis sharing across an ensemble). Supporting driven constraints needs an
/// inhomogeneous-Dirichlet treatment — a particular solution carrying the prescribed
/// motion, with the basis spanning only the homogeneous part — which is not in R1.1 and
/// is flagged here so it is not discovered by a wrong answer.
///
/// ## Why the state is `(q, q̇)` and never `x`
///
/// The reduced trajectory is carried in reduced coordinates. Handing this solver a
/// full-order `x_prev` each step would silently re-project it, discarding the
/// out-of-span part and hiding exactly the drift a reduced model has to be judged on.
/// Reduced state in, reduced state out.
pub struct ReducedNewtonSolver<'a, E, Msh, C, M, const N: usize, const G: usize>
where
    E: Element<N, G>,
    Msh: Mesh<M>,
    M: Material,
    C: ContactModel + ActivePairsFor<M>,
{
    full: &'a CpuNewtonSolver<E, Msh, C, M, N, G>,
    basis: &'a PodBasis,
    x_rest: Vec<f64>,
}

impl<'a, E, Msh, C, M, const N: usize, const G: usize> ReducedNewtonSolver<'a, E, Msh, C, M, N, G>
where
    E: Element<N, G>,
    Msh: Mesh<M>,
    M: Material,
    C: ContactModel + ActivePairsFor<M>,
{
    /// Wrap `full` with `basis`. `x_rest` is the reference configuration the basis's
    /// snapshots were taken relative to (full-DOF, length `n_dof`).
    ///
    /// # Panics
    /// Panics if `x_rest` is not `n_dof` long, or if the basis does not span the
    /// solver's free DOFs — either is a scene-wiring bug that would otherwise surface
    /// as silently wrong physics.
    #[must_use]
    pub fn new(
        full: &'a CpuNewtonSolver<E, Msh, C, M, N, G>,
        basis: &'a PodBasis,
        x_rest: &[f64],
    ) -> Self {
        assert!(
            x_rest.len() == full.n_dof,
            "x_rest has {} entries, expected n_dof = {}",
            x_rest.len(),
            full.n_dof,
        );
        assert!(
            basis.n_free() == full.free_dof_indices().len(),
            "basis spans {} free DOFs but the solver has {}",
            basis.n_free(),
            full.free_dof_indices().len(),
        );
        Self {
            full,
            basis,
            x_rest: x_rest.to_vec(),
        }
    }

    /// Reduced modes `r`.
    #[must_use]
    pub const fn n_modes(&self) -> usize {
        self.basis.n_modes()
    }

    /// Scatter `x = x_rest + Φq` onto full DOFs, taking constrained DOFs from `x_prev`.
    fn expand(&self, q: &[f64], x_prev: &[f64]) -> Vec<f64> {
        let mut x = x_prev.to_vec();
        let u = self.basis.reconstruct(q);
        for (k, &full_idx) in self.full.free_dof_indices().iter().enumerate() {
            x[full_idx] = self.x_rest[full_idx] + u[k];
        }
        x
    }

    /// Gather the free-DOF entries of a full-DOF vector.
    fn gather_free(&self, full: &[f64]) -> Vec<f64> {
        self.full
            .free_dof_indices()
            .iter()
            .map(|&i| full[i])
            .collect()
    }

    /// `A_r = Φᵀ A Φ` from the solver's **lower-triangle** tangent triplets.
    ///
    /// ⚠ The reflection is load-bearing. `assemble_free_hessian_triplets` emits only
    /// `row ≥ col`, so treating the triplet list as the whole matrix would silently
    /// build `Φᵀ tril(A) Φ` — a different, non-symmetric operator that still factors
    /// and still converges to something plausible. Every off-diagonal contributes
    /// twice, once transposed.
    // `y`/`r`/`n`/`v`/`t` mirror the matrix algebra (`Y = AΦ`, rank `r`, size `n`) and
    // longer names would obscure the correspondence with the formula above.
    #[allow(clippy::many_single_char_names)]
    fn project_tangent(&self, x: &[f64], x_prev: Option<&[f64]>, dt: f64) -> DMatrix<f64> {
        let triplets = self.full.assemble_free_hessian_triplets(x, x_prev, dt);
        let r = self.basis.n_modes();
        let n = self.basis.n_free();

        // Y = A Φ, dense n × r, column-major by mode. `Φ` is borrowed, not rebuilt:
        // reconstructing it from unit vectors costs `O(n·r²)` per call and this runs
        // once per Newton iteration.
        let mut y = vec![vec![0.0_f64; r]; n];
        let modes = self.basis.modes();
        for t in &triplets {
            let (row, col, v) = (t.row, t.col, t.val);
            for k in 0..r {
                y[row][k] += v * modes[k][col];
                if row != col {
                    y[col][k] += v * modes[k][row];
                }
            }
        }

        let mut a_r = DMatrix::<f64>::zeros(r, r);
        for i in 0..r {
            for j in i..r {
                let acc: f64 = (0..n).map(|p| modes[i][p] * y[p][j]).sum();
                a_r[(i, j)] = acc;
                a_r[(j, i)] = acc;
            }
        }
        a_r
    }

    /// Take one reduced backward-Euler step.
    ///
    /// # Errors
    /// [`SolverFailure::NewtonIterCap`] / [`SolverFailure::ArmijoStall`] as in the full
    /// solve, and [`SolverFailure::ValidityViolation`] if the reduced configuration
    /// leaves a material's validity domain — the reduced path is gated on exactly the
    /// same physical bounds as the oracle, because it produces a real configuration.
    /// `x_partial` on each variant is the **full-DOF** state at the failure, so a caller
    /// can inspect it with the ordinary readouts.
    ///
    /// # Panics
    /// Panics if `q_prev` / `qdot_prev` are not `n_modes` long.
    // One linear Newton narrative: residual, projection, tangent, factor, line search.
    // Splitting it would separate the residual definition from the norm it descends.
    #[allow(clippy::too_many_lines)]
    pub fn step(
        &self,
        q_prev: &[f64],
        qdot_prev: &[f64],
        theta: &Tensor<f64>,
        dt: f64,
    ) -> Result<ReducedStep, SolverFailure> {
        let r = self.basis.n_modes();
        assert!(
            q_prev.len() == r && qdot_prev.len() == r,
            "q must have n_modes entries"
        );
        assert!(dt > 0.0, "dt must be positive, got {dt}");

        let n_dof = self.full.n_dof;
        // The previous full state, reconstructed from reduced coordinates. Constrained
        // DOFs sit at their rest values plus whatever the caller's scene prescribes;
        // R1.1 scope is a fixed constrained set (see the R1.0 gate's fixture note).
        let x_prev = self.expand(q_prev, &self.x_rest);
        let v_prev_full = {
            let mut v = vec![0.0; n_dof];
            let du = self.basis.reconstruct(qdot_prev);
            for (k, &i) in self.full.free_dof_indices().iter().enumerate() {
                v[i] = du[k];
            }
            v
        };

        let mut f_ext = vec![0.0; n_dof];
        self.full.assemble_external_force(theta, &mut f_ext);
        let mut f_int = vec![0.0; n_dof];
        let mut r_full = vec![0.0; n_dof];

        let mut q = q_prev.to_vec();
        self.full.check_validity_at_step_start(&x_prev)?;

        let residual_at = |q: &[f64], f_int: &mut Vec<f64>, r_full: &mut Vec<f64>| -> Vec<f64> {
            let x = self.expand(q, &x_prev);
            self.full.assemble_global_int_force(&x, &x_prev, dt, f_int);
            residual_into(
                &x,
                &x_prev,
                &v_prev_full,
                f_int,
                &f_ext,
                &self.full.mass_per_dof,
                dt,
                r_full,
            );
            self.gather_free(r_full)
        };

        let mut last_proj = 0.0_f64;
        for iter in 0..self.full.config.max_newton_iter {
            let r_free = residual_at(&q, &mut f_int, &mut r_full);
            let r_r = self.basis.project_covector(&r_free);
            let proj_norm = r_r.iter().map(|a| a * a).sum::<f64>().sqrt();
            last_proj = proj_norm;

            if proj_norm < self.full.config.tol {
                let x = self.expand(&q, &x_prev);
                self.full.check_validity_at_step_start(&x)?;
                let full_norm = r_free.iter().map(|a| a * a).sum::<f64>().sqrt();
                let qdot: Vec<f64> = q.iter().zip(q_prev).map(|(a, b)| (a - b) / dt).collect();
                return Ok(ReducedStep {
                    q,
                    qdot,
                    iter_count: iter,
                    projected_residual_norm: proj_norm,
                    full_residual_norm: full_norm,
                });
            }

            let a_r = self.project_tangent(&self.expand(&q, &x_prev), Some(&x_prev), dt);
            let rhs = DVector::from_iterator(r_r.len(), r_r.iter().map(|v| -v));
            // Dense `r × r`: Cholesky when SPD, LU otherwise. The full tangent is SPD in
            // the regimes R1.1 targets and `ΦᵀAΦ` inherits that for a full-rank `Φ`, so
            // the LU arm is the same cold fallback `factor_free_tangent` keeps.
            let delta = a_r
                .clone()
                .cholesky()
                .map_or_else(|| a_r.clone().lu().solve(&rhs), |ch| Some(ch.solve(&rhs)));
            let Some(delta) = delta else {
                return Err(SolverFailure::DoublyFailedFactor {
                    x_partial: self.expand(&q, &x_prev),
                    last_iter: iter,
                    context: format!(
                        "reduced tangent (r = {r}) failed both Cholesky and LU at Newton iter {iter}"
                    ),
                });
            };

            // Armijo on ‖Φᵀr‖ — the quantity this solve descends.
            let mut alpha = 1.0;
            let mut accepted = None;
            for _ in 0..self.full.config.max_line_search_backtracks {
                let trial: Vec<f64> = q
                    .iter()
                    .zip(delta.iter())
                    .map(|(a, d)| a + alpha * d)
                    .collect();
                let tr = self
                    .basis
                    .project_covector(&residual_at(&trial, &mut f_int, &mut r_full));
                let tn = tr.iter().map(|a| a * a).sum::<f64>().sqrt();
                if tn <= (1.0 - ARMIJO_C1 * alpha) * proj_norm {
                    accepted = Some(trial);
                    break;
                }
                alpha *= 0.5;
            }
            let Some(next) = accepted else {
                return Err(SolverFailure::ArmijoStall {
                    x_partial: self.expand(&q, &x_prev),
                    last_iter: iter,
                    last_r_norm: proj_norm,
                });
            };
            q = next;
        }

        Err(SolverFailure::NewtonIterCap {
            x_partial: self.expand(&q, &x_prev),
            max_iter: self.full.config.max_newton_iter,
            last_r_norm: last_proj,
        })
    }

    /// The full-DOF configuration for reduced coordinates `q`, relative to rest.
    #[must_use]
    pub fn expand_to_full(&self, q: &[f64]) -> Vec<f64> {
        self.expand(q, &self.x_rest)
    }
}
