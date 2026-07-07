//! Equilibrium sensitivities + reverse-mode VJP family for
//! [`CpuNewtonSolver`](super::CpuNewtonSolver) — keystone differentiable path.

use sim_ml_chassis::{Tensor, Var};

use crate::Vec3;
use crate::contact::{ActivePairsFor, ContactModel, RigidTwist};
use crate::differentiable::newton_vjp::{
    MaterialStepVjp, NewtonStepVjp, StateStepVjp, TrajectoryStepVjp,
};
use crate::element::Element;
use crate::material::Material;
use crate::mesh::{Mesh, TetId, VertexId};
use crate::readout::LoadAxis;
use crate::solver::{CpuTape, NewtonStep};

use super::CpuNewtonSolver;
use super::helpers::{deformation_gradient, extract_element_dof_values, slice_to_vec3s};
use super::{FactoredFreeTangent, FrictionReactionGradients, FrictionVertexForce};

impl<E, Msh, C, M, const N: usize, const G: usize> CpuNewtonSolver<E, Msh, C, M, N, G>
where
    E: Element<N, G>,
    Msh: Mesh<M>,
    M: Material,
    C: ContactModel + ActivePairsFor<M>,
{
    /// Push `NewtonStepVjp` onto the tape with `theta_var` as parent.
    /// Shared by [`Solver::step`](crate::solver::Solver::step) and [`Solver::try_step`](crate::solver::Solver::try_step) — both must
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
    pub(super) fn push_newton_step_vjp(
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
    /// [`Solver::step`](crate::solver::Solver::step) / [`Solver::replay_step`](crate::solver::Solver::replay_step); `dt` is that step's
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
    ///
    /// # Panics
    ///
    /// Panics if friction is active (`config.friction_mu != 0` and `x_prev`
    /// is `Some`) while `twist.angular` is non-zero: the friction pose
    /// sensitivity supports only a pure translation (pass a translation
    /// twist, or `x_prev = None` for the frictionless path).
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

    /// Forward sensitivity `∂(reaction)/∂(pinned target)` of a converged **constrained
    /// (Dirichlet) equilibrium** — rung 6d, the bonded-disc reaction adjoint's forward
    /// JVP. Given a displacement direction `dx_pinned` of the *pinned* (Dirichlet) DOFs
    /// — zero on the free DOFs — returns the resulting change in the nodal reaction
    /// `R = −f_int` (length `n_dof`; the meaningful entries are the pinned DOFs — the
    /// bonded endplate the caller sums into a wrench).
    ///
    /// **Why the existing sensitivities don't cover this.** The keystone family
    /// ([`Self::equilibrium_pose_sensitivity`], [`Self::equilibrium_material_sensitivity`],
    /// [`Self::equilibrium_state_sensitivity`]) all solve `A_ff·λ = g_free` with the
    /// free-free factor and scatter to the FREE DOFs (zeros on pinned) — the *constrained*
    /// columns are never assembled. Here the input IS the pinned target and the output IS
    /// the pinned reaction, so we need those columns.
    ///
    /// **The math (Schur complement of the stiffness).** Partition DOFs into free `f` and
    /// pinned `p`; the pinned nodes are held at targets `x_p`. Free equilibrium
    /// `f_int,f(x_f, x_p) = 0` gives `∂x_f/∂x_p = −A_ff⁻¹·K_fp` (the coupling is purely
    /// elastic — the lumped mass is diagonal, so it does not couple free↔pinned). The
    /// reaction is `R_p = −f_int,p`, and its inertial term cancels (a pinned node's target
    /// equals its `x_prev`), so
    ///
    /// ```text
    ///     dR_p/dx_p = K_pf·A_ff⁻¹·K_fp − K_pp    (= −Schur complement of K_ff in K)
    /// ```
    ///
    /// Applied to `dx_pinned` in three steps, reusing the SAME factor the forward Newton
    /// step and the other sensitivities use:
    /// 1. `rhs_free = −(K·dx_pinned)_free`   (`= −K_fp·dx_pinned`, via the crate-private
    ///    `internal_force_tangent_matvec`);
    /// 2. `dx_free = A_ff⁻¹·rhs_free`, scattered to full DOFs (zeros on pinned);
    /// 3. `dR = −K·(dx_free + dx_pinned)`   (one more matvec — the reaction is `−f_int`).
    ///
    /// `x_final` is the converged constrained equilibrium; `dt` is that solve's time-step
    /// (the factor's `M/Δt²` must match). Frictionless / contact-free scope (the
    /// [`NullContact`](crate::contact::NullContact) Dirichlet bond) — see the
    /// `internal_force_tangent_matvec` scope note. The reverse (tape) dual is the rung-6d VJP.
    ///
    /// # Panics
    /// Panics if `dx_pinned.len() != n_dof`.
    #[must_use]
    pub fn equilibrium_dirichlet_reaction_sensitivity(
        &self,
        x_final: &[f64],
        dt: f64,
        dx_pinned: &[f64],
    ) -> Vec<f64> {
        assert!(
            dx_pinned.len() == self.n_dof,
            "dx_pinned must have length n_dof = {}, got {}",
            self.n_dof,
            dx_pinned.len()
        );
        // (1) rhs_free = −(K·dx_pinned)_free — only K_fp couples free rows to the pinned
        //     perturbation (dx_pinned is zero on free DOFs by contract).
        let k_dxp = self.internal_force_tangent_matvec(x_final, dx_pinned);
        let rhs: Vec<f64> = self.free_dof_indices.iter().map(|&i| -k_dxp[i]).collect();
        // (2) dx_free = A_ff⁻¹·rhs_free, scattered to full DOFs (zeros on pinned).
        let mut dx_total = self.solve_free_and_scatter(x_final, None, dt, rhs);
        // (3) total config perturbation = dx_free (free) + dx_pinned (pinned), then
        //     dR = −K·dx_total (the reaction is −f_int).
        for (d, &p) in dx_total.iter_mut().zip(dx_pinned) {
            *d += p;
        }
        let k_dx = self.internal_force_tangent_matvec(x_final, &dx_total);
        k_dx.iter().map(|&x| -x).collect()
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
