//! # cf-codesign — the co-design optimizer
//!
//! Mission connective-tissue #2: a gradient-based outer loop that optimizes a
//! **design** parameter so a physical outcome hits a target, driven by the
//! gradient that crosses the differentiable soft↔rigid coupling (the keystone,
//! `sim-coupling`). This is the first *consumer* of that gradient substrate —
//! it closes the body↔device co-design loop end to end.
//!
//! The optimizer is generic over a [`CoDesignProblem`] (a differentiable design
//! objective) and reuses the chassis Adam optimizer
//! ([`sim_ml_chassis::OptimizerConfig`]) rather than reinventing one. Two
//! concrete problems tune a soft body's Neo-Hookean material so a rigid-side
//! outcome hits a target, and each is backed by a worked **inverse-design** demo
//! that recovers a known material from its target behavior:
//! - [`SoftMaterialTarget`] (v1) — the rigid body's *next-step velocity* (the
//!   keystone single-step gradient `∂vz'/∂μ`).
//! - [`SoftMaterialTrajectoryTarget`] (v2) — the rigid body's *height after an
//!   N-step contact-engaged coupled rollout* (the keystone multi-step gradient
//!   `∂z_N/∂μ`, one `tape.backward` across both engines and every step boundary).
//!
//! Scope: a single design parameter, the keystone's contact-engaged regime.
//! Multi-parameter / manufacturing-constrained / policy co-optimization are
//! documented follow-ons (`docs/codesign/recon.md`).
//!
//! ```no_run
//! use cf_codesign::{optimize, OptConfig, SoftMaterialTarget};
//! # const MJCF: &str = "";
//! // Target behavior = the rigid outcome a reference material μ* produces.
//! let probe = SoftMaterialTarget::for_inverse_design(MJCF.to_string(), 0.099, 40_000.0);
//! let result = optimize(&probe, &[20_000.0], &OptConfig::default());
//! assert!(result.converged);
//! ```

use sim_coupling::StaggeredCoupling;
use sim_ml_chassis::OptimizerConfig;

/// A differentiable design objective the [`optimize`] loop drives: it maps a
/// parameter vector to a scalar loss and its gradient.
///
/// Object-safe so the optimizer can hold `&dyn CoDesignProblem`. Any
/// differentiable design objective — material (today), later geometry / lattice
/// / control policy — implements this and plugs into the same loop.
pub trait CoDesignProblem {
    /// Number of design parameters.
    fn n_params(&self) -> usize;

    /// `(loss, gradient)` at `params`. `gradient.len()` must equal
    /// [`n_params`](Self::n_params).
    fn evaluate(&self, params: &[f64]) -> (f64, Vec<f64>);

    /// Optional per-parameter lower bounds, clamped after each optimizer step
    /// (e.g. a positive-stiffness floor `μ > 0`). Default: unconstrained.
    fn lower_bounds(&self) -> Option<Vec<f64>> {
        None
    }
}

/// Optimization hyperparameters for [`optimize`].
#[derive(Clone, Copy, Debug)]
pub struct OptConfig {
    /// Adam learning rate (the per-iteration parameter step magnitude, since
    /// Adam normalizes the gradient by its running RMS).
    pub lr: f64,
    /// Maximum iterations.
    pub max_iters: usize,
    /// Stop when `‖gradient‖∞` falls below this. **`0.0` disables the
    /// gradient criterion** (the test is strict `<`, and `‖·‖∞ ≥ 0`), leaving
    /// `loss_tol` / `max_iters` as the active stops — the [`Default`].
    pub grad_tol: f64,
    /// Stop when the (absolute) loss falls below this.
    pub loss_tol: f64,
    /// Adam's numerical-stability constant `ε` (the `+ε` in
    /// `lr·m̂/(√v̂ + ε)`). The default `1e-8` is Adam's standard value and fine
    /// when the gradient is well above it. **Set it BELOW the gradient
    /// magnitude for weakly-sensitive objectives:** if `‖gradient‖ ≲ ε`, the `ε`
    /// term dominates the denominator and Adam loses its scale-invariance,
    /// degenerating into tiny SGD-like steps that crawl (e.g. a trajectory
    /// outcome `z_N` whose `∂z_N/∂μ ~ 1e-7` yields a `~1e-10` loss gradient —
    /// see [`SoftMaterialTrajectoryTarget`]). Below the gradient scale, Adam
    /// recovers full scale-invariance and converges normally.
    pub eps: f64,
}

impl Default for OptConfig {
    /// `lr = 2e3`, `max_iters = 300`, `loss_tol = 1e-10`, `grad_tol = 0.0`
    /// (gradient criterion disabled — stop on the absolute loss or the iteration
    /// cap), and `eps = 1e-8` (Adam's standard value). Tuned for the keystone
    /// `μ ~ 3e4` single-step stiffness scene; opt into a gradient-based stop by
    /// setting `grad_tol > 0`, and lower `eps` for weakly-sensitive objectives
    /// (see [`OptConfig::eps`]).
    fn default() -> Self {
        Self {
            lr: 2.0e3,
            max_iters: 300,
            grad_tol: 0.0,
            loss_tol: 1.0e-10,
            eps: 1.0e-8,
        }
    }
}

/// One iteration's record (pre-step), for inspecting the descent trajectory.
#[derive(Clone, Debug)]
pub struct IterRecord {
    /// Parameters at the start of the iteration.
    pub params: Vec<f64>,
    /// Loss at `params`.
    pub loss: f64,
    /// `‖gradient‖∞` at `params`.
    pub grad_inf: f64,
}

/// Result of an [`optimize`] run.
#[derive(Clone, Debug)]
pub struct OptResult {
    /// Final design parameters.
    pub params: Vec<f64>,
    /// Loss at the final parameters.
    pub loss: f64,
    /// Iterations performed.
    pub iters: usize,
    /// Whether a stopping criterion (`grad_tol` / `loss_tol`) was met before
    /// `max_iters`.
    pub converged: bool,
    /// Per-iteration `(params, loss, ‖grad‖∞)` history.
    pub history: Vec<IterRecord>,
}

/// Minimize `problem`'s loss from `x0` with Adam, to convergence.
///
/// Each iteration evaluates `(loss, gradient)` at the current parameters,
/// records them, checks the stopping criteria, then takes one Adam descent step
/// (`ascent = false`) and clamps to [`CoDesignProblem::lower_bounds`]. Reuses
/// the chassis [`sim_ml_chassis::OptimizerConfig::adam`] — Adam's per-parameter
/// adaptive scaling handles the design parameters' wide magnitude range (e.g.
/// `μ ~ 3e4` with a gradient `~1e-5`).
///
/// # Panics
/// Panics if `x0.len() != problem.n_params()`.
#[must_use]
pub fn optimize(problem: &dyn CoDesignProblem, x0: &[f64], cfg: &OptConfig) -> OptResult {
    assert_eq!(
        x0.len(),
        problem.n_params(),
        "x0 length {} != problem n_params {}",
        x0.len(),
        problem.n_params(),
    );
    // Adam's `step_in_place` updates the caller's `params` buffer directly
    // (it does not read the optimizer's internal param copy), so the buffer
    // below IS the optimization state; no `set_params` priming is needed.
    // Build the full Adam config (not the `adam(lr)` shortcut) so the caller's
    // `eps` is honored — Adam's defaults otherwise (β₁ = 0.9, β₂ = 0.999, no
    // gradient clipping), exactly the shortcut's values when `eps = 1e-8`.
    let mut opt = OptimizerConfig::Adam {
        lr: cfg.lr,
        beta1: 0.9,
        beta2: 0.999,
        eps: cfg.eps,
        max_grad_norm: f64::INFINITY,
    }
    .build(x0.len());
    let mut params = x0.to_vec();
    let bounds = problem.lower_bounds();
    let mut history = Vec::with_capacity(cfg.max_iters);

    for it in 0..cfg.max_iters {
        let (loss, grad) = problem.evaluate(&params);
        let grad_inf = grad.iter().fold(0.0_f64, |m, &g| m.max(g.abs()));
        history.push(IterRecord {
            params: params.clone(),
            loss,
            grad_inf,
        });
        if grad_inf < cfg.grad_tol || loss < cfg.loss_tol {
            return OptResult {
                params,
                loss,
                iters: it,
                converged: true,
                history,
            };
        }
        opt.step_in_place(&mut params, &grad, false);
        if let Some(lb) = &bounds {
            for (p, &b) in params.iter_mut().zip(lb) {
                if *p < b {
                    *p = b;
                }
            }
        }
    }
    // Exhausted max_iters without meeting a stopping criterion.
    let (loss, _) = problem.evaluate(&params);
    OptResult {
        params,
        loss,
        iters: cfg.max_iters,
        converged: false,
        history,
    }
}

/// A co-design problem over a soft block's Neo-Hookean stiffness: tune the
/// material parameter `μ` (with the keystone's `λ = 4μ` tie — i.e. scale the
/// stiffness at fixed Poisson ratio) so the platen's next-step vertical
/// velocity `vz'` matches `target_vz`. Loss `½(vz' − target_vz)²`.
///
/// Wraps the keystone coupling using only its public API: each
/// [`evaluate`](CoDesignProblem::evaluate) rebuilds a [`StaggeredCoupling`] at
/// the candidate `μ` and reads `(vz', ∂vz'/∂μ, ∂vz'/∂λ)` from one
/// `coupled_step_material_gradient` per parameter. Because the rebuild follows
/// the `λ = 4μ` line, the gradient w.r.t. the *stiffness scale* `μ` is the total
/// `∂vz'/∂μ + 4·∂vz'/∂λ` (the documented S5 linear combination), so it matches a
/// finite difference of `vz'(μ)` along that line.
pub struct SoftMaterialTarget {
    mjcf: String,
    /// Rigid body index in the model (the platen).
    body: usize,
    contact_clearance: f64,
    n_per_edge: usize,
    edge: f64,
    dt: f64,
    kappa: f64,
    d_hat: f64,
    rigid_damping: f64,
    /// Contact-plane height the coupled step is probed at.
    height: f64,
    /// Target platen next-step vertical velocity.
    target_vz: f64,
    /// Lower bound on `μ` (positive stiffness).
    mu_floor: f64,
}

impl SoftMaterialTarget {
    /// Construct with explicit coupling parameters.
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        mjcf: String,
        body: usize,
        contact_clearance: f64,
        n_per_edge: usize,
        edge: f64,
        dt: f64,
        kappa: f64,
        d_hat: f64,
        rigid_damping: f64,
        height: f64,
        target_vz: f64,
    ) -> Self {
        Self {
            mjcf,
            body,
            contact_clearance,
            n_per_edge,
            edge,
            dt,
            kappa,
            d_hat,
            rigid_damping,
            height,
            target_vz,
            mu_floor: 1.0,
        }
    }

    /// Convenience for the worked inverse-design demo with the keystone fixture
    /// (the `platen` body 1, clearance 5 mm, 4³ block edge 0.1 m, dt 1 ms,
    /// κ = 3e4, d̂ = 1e-2, damping 12). `target_vz` is the outcome to recover a
    /// material for (e.g. set it to [`Self::forward_vz`] at a reference `μ*`).
    #[must_use]
    pub fn for_inverse_design(mjcf: String, height: f64, target_vz: f64) -> Self {
        Self::new(
            mjcf, 1, 0.005, 4, 0.1, 1.0e-3, 3.0e4, 1.0e-2, 12.0, height, target_vz,
        )
    }

    /// Build the coupling at stiffness `mu` (`λ = 4μ` via `StaggeredCoupling`).
    //
    // expect: a malformed fixture MJCF / coupling is a caller error surfaced
    // loudly — the canonical fixture idiom, mirroring `sim-coupling`'s tests.
    #[allow(clippy::expect_used)]
    fn build(&self, mu: f64) -> StaggeredCoupling {
        let model = sim_mjcf::load_model(&self.mjcf).expect("SoftMaterialTarget: MJCF loads");
        let mut data = model.make_data();
        data.forward(&model)
            .expect("SoftMaterialTarget: initial forward");
        StaggeredCoupling::new(
            model,
            data,
            self.body,
            self.contact_clearance,
            self.n_per_edge,
            self.edge,
            mu,
            self.dt,
            self.kappa,
            self.d_hat,
            self.rigid_damping,
        )
    }

    /// The platen's next-step vertical velocity `vz'` at stiffness `mu` — the
    /// forward outcome (no gradient). Used to set up an inverse-design target.
    #[must_use]
    pub fn forward_vz(&self, mu: f64) -> f64 {
        self.build(mu)
            .coupled_step_material_gradient(self.height, 0)
            .0
    }

    /// The target this problem optimizes toward.
    #[must_use]
    pub const fn target_vz(&self) -> f64 {
        self.target_vz
    }
}

impl CoDesignProblem for SoftMaterialTarget {
    fn n_params(&self) -> usize {
        1
    }

    fn evaluate(&self, params: &[f64]) -> (f64, Vec<f64>) {
        let coupling = self.build(params[0]);
        // vz' and the two material sensitivities at this stiffness.
        let (vz, dvz_dmu) = coupling.coupled_step_material_gradient(self.height, 0);
        let (_, dvz_dlambda) = coupling.coupled_step_material_gradient(self.height, 1);
        // Stiffness scale μ with λ = 4μ ⇒ total dvz'/dμ = ∂/∂μ + 4·∂/∂λ.
        let dvz_dstiffness = 4.0_f64.mul_add(dvz_dlambda, dvz_dmu);
        let residual = vz - self.target_vz;
        (0.5 * residual * residual, vec![residual * dvz_dstiffness])
    }

    fn lower_bounds(&self) -> Option<Vec<f64>> {
        Some(vec![self.mu_floor])
    }
}

/// A co-design problem over a soft block's Neo-Hookean stiffness whose outcome is
/// the platen's height after a **multi-step** coupled rollout (not a single
/// step): tune `μ` (with the `λ = 4μ` tie) so the platen's final height `z_N`
/// after `n_steps` of the coupled contact dynamics matches `target_z`. Loss
/// `½(z_N − target_z)²`.
///
/// This is the trajectory analogue of [`SoftMaterialTarget`] and the first
/// consumer of the keystone's multi-step coupled gradient: each
/// [`evaluate`](CoDesignProblem::evaluate) reads `(z_N, ∂z_N/∂μ, ∂z_N/∂λ)` from
/// [`StaggeredCoupling::coupled_trajectory_material_gradient`] — ONE
/// `tape.backward` across both engines AND every step boundary. As with the
/// single-step target the `λ = 4μ` rebuild makes the stiffness-scale gradient
/// the total `∂z_N/∂μ + 4·∂z_N/∂λ`.
///
/// **Scope.** The fixture here ([`Self::for_inverse_design`]) starts the platen
/// already in contact, so the rollout is contact-engaged from step 0 and the
/// platen deepens/settles (it does not break and re-make contact). That is the
/// regime this target's gate validates. The consumed gradient is itself
/// machine-exact through genuine contact make/break — the keystone/IPC gates
/// (`sim-coupling`'s `coupled_trajectory_gradient` / `ipc_trajectory_gradient`,
/// which start the platen above contact) establish that independently.
///
/// **Conditioning.** `z_N` is a position, so `∂z_N/∂μ` is small (`~1e-7` for the
/// keystone scene) and the loss gradient is `~1e-10`. That is *below* Adam's
/// default `eps = 1e-8`, so the optimizer must run with a smaller
/// [`OptConfig::eps`] (below the gradient scale) or it crawls — see that field's
/// docs and [`Self::recommended_config`]. The objective is monotone in μ (a
/// stiffer block holds the platen higher), so `target_z = z_N(μ*)` has the
/// unique minimizer `μ*`.
///
/// Because `coupled_trajectory_material_gradient` takes `&mut self` (it advances
/// the rollout), each `evaluate` rebuilds the coupling **once per parameter
/// index** (twice total), unlike the single-step target's one shared build.
///
/// ```no_run
/// use cf_codesign::{optimize, SoftMaterialTrajectoryTarget};
/// # const MJCF: &str = "";
/// // z_N is weakly sensitive, so use the target's recommended config (it sets
/// // Adam's eps below the ~1e-10 loss gradient) rather than `OptConfig::default`.
/// let target = SoftMaterialTrajectoryTarget::for_inverse_design(MJCF.to_string(), 20, 0.124);
/// let cfg = target.recommended_config();
/// let result = optimize(&target, &[20_000.0], &cfg);
/// assert!(result.converged);
/// ```
pub struct SoftMaterialTrajectoryTarget {
    mjcf: String,
    /// Rigid body index in the model (the platen).
    body: usize,
    contact_clearance: f64,
    n_per_edge: usize,
    edge: f64,
    dt: f64,
    kappa: f64,
    d_hat: f64,
    rigid_damping: f64,
    /// Number of coupled steps in the rollout the outcome `z_N` is read after.
    n_steps: usize,
    /// Target platen height after the rollout.
    target_z: f64,
    /// Lower bound on `μ` (positive stiffness).
    mu_floor: f64,
}

impl SoftMaterialTrajectoryTarget {
    /// Construct with explicit coupling parameters.
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        mjcf: String,
        body: usize,
        contact_clearance: f64,
        n_per_edge: usize,
        edge: f64,
        dt: f64,
        kappa: f64,
        d_hat: f64,
        rigid_damping: f64,
        n_steps: usize,
        target_z: f64,
    ) -> Self {
        Self {
            mjcf,
            body,
            contact_clearance,
            n_per_edge,
            edge,
            dt,
            kappa,
            d_hat,
            rigid_damping,
            n_steps,
            target_z,
            mu_floor: 1.0,
        }
    }

    /// Convenience for the worked trajectory inverse-design demo with the
    /// keystone fixture (the `platen` body 1, clearance 5 mm, 4³ block edge
    /// 0.1 m, dt 1 ms, κ = 3e4, d̂ = 1e-2, damping 60 — the contact-engaged
    /// rollout of the keystone trajectory gate). The platen MJCF should start
    /// already in contact (e.g. `pos.z = 0.108`) so the rollout is engaged from
    /// step 0. `target_z` is the height to recover a material for (e.g. set it to
    /// [`Self::forward_z`] at a reference `μ*`).
    #[must_use]
    pub fn for_inverse_design(mjcf: String, n_steps: usize, target_z: f64) -> Self {
        Self::new(
            mjcf, 1, 0.005, 4, 0.1, 1.0e-3, 3.0e4, 1.0e-2, 60.0, n_steps, target_z,
        )
    }

    /// An [`OptConfig`] tuned for this (weakly-sensitive) trajectory objective —
    /// the batteries-included counterpart to [`Self::for_inverse_design`]. It
    /// lowers Adam's `eps` to `1e-12` (below the `~1e-10` loss gradient, so the
    /// optimizer keeps its scale-invariance instead of crawling — see
    /// [`OptConfig::eps`]) and tightens `loss_tol`/`max_iters` for the deep
    /// convergence the small `z_N` slope needs. Prefer this over
    /// [`OptConfig::default`], whose `eps = 1e-8` is too large for `z_N`.
    #[must_use]
    pub fn recommended_config(&self) -> OptConfig {
        OptConfig {
            eps: 1.0e-12,
            loss_tol: 1.0e-18,
            max_iters: 400,
            ..OptConfig::default()
        }
    }

    /// Build the coupling at stiffness `mu` (`λ = 4μ` via `StaggeredCoupling`).
    //
    // expect: a malformed fixture MJCF / coupling is a caller error surfaced
    // loudly — the canonical fixture idiom, mirroring `sim-coupling`'s tests.
    #[allow(clippy::expect_used)]
    fn build(&self, mu: f64) -> StaggeredCoupling {
        let model =
            sim_mjcf::load_model(&self.mjcf).expect("SoftMaterialTrajectoryTarget: MJCF loads");
        let mut data = model.make_data();
        data.forward(&model)
            .expect("SoftMaterialTrajectoryTarget: initial forward");
        StaggeredCoupling::new(
            model,
            data,
            self.body,
            self.contact_clearance,
            self.n_per_edge,
            self.edge,
            mu,
            self.dt,
            self.kappa,
            self.d_hat,
            self.rigid_damping,
        )
    }

    /// The platen's height `z_N` after the `n_steps` coupled rollout at stiffness
    /// `mu` — the forward outcome (no gradient). Used to set up an inverse-design
    /// target.
    #[must_use]
    pub fn forward_z(&self, mu: f64) -> f64 {
        self.build(mu)
            .coupled_trajectory_material_gradient(self.n_steps, 0)
            .0
    }

    /// The target this problem optimizes toward.
    #[must_use]
    pub const fn target_z(&self) -> f64 {
        self.target_z
    }
}

impl CoDesignProblem for SoftMaterialTrajectoryTarget {
    fn n_params(&self) -> usize {
        1
    }

    fn evaluate(&self, params: &[f64]) -> (f64, Vec<f64>) {
        // `coupled_trajectory_material_gradient` takes `&mut self` (advances the
        // rollout), so each parameter index needs its own fresh build.
        let (z_n, dz_dmu) = self
            .build(params[0])
            .coupled_trajectory_material_gradient(self.n_steps, 0);
        let (_, dz_dlambda) = self
            .build(params[0])
            .coupled_trajectory_material_gradient(self.n_steps, 1);
        // Stiffness scale μ with λ = 4μ ⇒ total dz_N/dμ = ∂/∂μ + 4·∂/∂λ.
        let dz_dstiffness = 4.0_f64.mul_add(dz_dlambda, dz_dmu);
        let residual = z_n - self.target_z;
        (0.5 * residual * residual, vec![residual * dz_dstiffness])
    }

    fn lower_bounds(&self) -> Option<Vec<f64>> {
        Some(vec![self.mu_floor])
    }
}
