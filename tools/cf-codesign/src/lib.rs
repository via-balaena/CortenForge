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
//! A weakly-sensitive objective (a tiny gradient, like the position-valued
//! trajectory outcome `z_N`) is conditioned for the optimizer by the general
//! [`Normalized`] wrapper — a dimensionless residual plus log-space (relative)
//! parameter steps — so the **standard** Adam `eps` keeps its scale-invariance
//! instead of crawling (rather than chasing a fragile per-scene tiny `eps`).
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
    /// `lr·m̂/(√v̂ + ε)`). The default `1e-8` is Adam's standard value and is
    /// scale-invariant when the gradient is well above it; if `‖gradient‖ ≲ ε`
    /// the `ε` term dominates the denominator and Adam degenerates into tiny
    /// SGD-like steps that crawl. **For a weakly-sensitive objective (a tiny
    /// gradient, e.g. a position-valued trajectory outcome `z_N` whose
    /// `∂z_N/∂μ ~ 1e-7` gives a `~2e-10` loss gradient), prefer conditioning the
    /// objective with [`Normalized`] so the gradient rises above this standard
    /// `ε`** — a robust, scale-aware fix — rather than chasing a fragile per-scene
    /// tiny `ε` (which only works while the un-normalized gradient stays above it).
    /// `ε` remains exposed as the legitimate general Adam knob it is.
    pub eps: f64,
}

impl Default for OptConfig {
    /// `lr = 2e3`, `max_iters = 300`, `loss_tol = 1e-10`, `grad_tol = 0.0`
    /// (gradient criterion disabled — stop on the absolute loss or the iteration
    /// cap), and `eps = 1e-8` (Adam's standard value). Tuned for the keystone
    /// `μ ~ 3e4` single-step stiffness scene; opt into a gradient-based stop by
    /// setting `grad_tol > 0`, and condition a weakly-sensitive objective with
    /// [`Normalized`] rather than shrinking `eps` (see [`OptConfig::eps`]).
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

/// A [`CoDesignProblem`] decorator that **conditions** an objective for gradient
/// descent: it rescales the loss/gradient and (optionally) reparametrizes the
/// design variables into a better-scaled space, so a stock Adam (`OptConfig`'s
/// standard `eps = 1e-8`) keeps its scale-invariance instead of crawling.
///
/// # Why this exists
///
/// Adam's update is `lr · m̂ / (√v̂ + eps)` — scale-invariant (the gradient
/// *magnitude* cancels) ONLY while `‖grad‖ ≫ eps`; below `eps` the `+ eps`
/// dominates the denominator and Adam degenerates into eps-dominated SGD that
/// crawls. A weakly-sensitive design objective — e.g. a position-valued outcome
/// against a large-magnitude parameter, where `∂(outcome)/∂μ` is tiny — produces
/// a gradient *below* the standard `eps`, so the optimizer stalls. Rather than
/// chase a per-scene tiny `eps` (fragile — the right value tracks the
/// un-normalized gradient and shrinks with the scene), normalize the objective so
/// its gradient is `O(1)`-ish and the STANDARD `eps` works across magnitudes and
/// scenes (`lr` and `loss_tol` remain per-objective tunables).
///
/// # The two levers (both validated by the v3 spike, `docs/codesign/recon.md`)
///
/// - **`loss_scale` — a dimensionless residual.** The loss and its gradient are
///   multiplied by `loss_scale`. For a residual objective `½(y − y*)²`, passing
///   `loss_scale = 1/L²` (a characteristic length `L` carrying `y`'s unit) yields
///   `½((y − y*)/L)²` — a dimensionless loss, so `loss_tol` reads as a
///   relative-tolerance². (See [`Self::with_residual_scale`].) On its own a
///   constant `loss_scale` only shifts the gradient up by a fixed factor, which can
///   leave it near `eps` near the optimum (the gradient still → 0 there); it is the
///   weaker lever.
/// - **`log_space` — relative parameter steps.** When set, the optimizer works in
///   `p = ln(param)` and this wrapper maps to/from the physical value. The chain
///   rule multiplies the gradient by `dμ/dp = μ`, which lifts a tiny `∂/∂μ` above
///   `eps` *throughout* descent (the stronger lever for positive scale parameters
///   like stiffness); an Adam step becomes a *fractional* change in the parameter,
///   and `param > 0` is enforced structurally (a finite lower bound still maps
///   through `ln` and runs as a no-op safety net). Use a smaller, relative `lr`
///   (≈ 0.05) in this space — see [`Self::recommended_config`].
///
/// Both compose: the gradient the optimizer sees is `loss_scale · (dμ/dp) ·
/// ∂loss/∂μ`. The transforms apply element-wise, so this generalizes to any
/// vector of positive design variables (geometry, lattice density, …), which is
/// why it lives as a general combinator rather than being baked into one target.
/// Put **both** levers on a SINGLE wrapper — nesting a `Normalized` inside another
/// would double-apply the `ln`/`exp` reparametrization.
///
/// # Example
///
/// ```no_run
/// use cf_codesign::{Normalized, SoftMaterialTrajectoryTarget};
/// # const MJCF: &str = "";
/// let target = SoftMaterialTrajectoryTarget::for_inverse_design(MJCF.to_string(), 20, 0.124);
/// // Dimensionless residual (L = the 0.1 m block edge) + log-μ relative steps.
/// let problem = Normalized::with_residual_scale(&target, 0.1, true);
/// let cfg = problem.recommended_config();          // standard eps = 1e-8
/// // `optimize` brackets the physical↔normalized mapping: x0 and the result's
/// // params are in PHYSICAL units (no manual to_normalized / to_physical).
/// let result = problem.optimize(&[20_000.0], &cfg);
/// let mu = result.params[0];
/// ```
pub struct Normalized<'a> {
    inner: &'a dyn CoDesignProblem,
    /// Multiplier applied to the loss and its gradient.
    loss_scale: f64,
    /// Whether the optimizer's parameter space is `ln(param)` (relative steps).
    log_space: bool,
}

impl<'a> Normalized<'a> {
    /// Wrap `inner` with an explicit `loss_scale` and `log_space` flag.
    ///
    /// # Panics
    /// Panics unless `loss_scale` is finite and strictly positive — a zero,
    /// negative, or non-finite scale would silently flip the descent direction,
    /// fake a converged (zero) loss, or poison the gradient with `NaN`/`inf`.
    #[must_use]
    pub fn new(inner: &'a dyn CoDesignProblem, loss_scale: f64, log_space: bool) -> Self {
        assert!(
            loss_scale.is_finite() && loss_scale > 0.0,
            "loss_scale must be finite and > 0, got {loss_scale}",
        );
        Self {
            inner,
            loss_scale,
            log_space,
        }
    }

    /// Wrap a **residual** objective `½(y − y*)²` so its residual is measured in
    /// units of the characteristic length `residual_scale` (`L`): sets
    /// `loss_scale = 1/L²`, giving the dimensionless loss `½((y − y*)/L)²`. The
    /// `log_space` flag is forwarded (see the type docs).
    ///
    /// # Panics
    /// Panics unless `residual_scale` is finite, strictly positive, and small
    /// enough that `1/L²` does not overflow to `inf`.
    #[must_use]
    pub fn with_residual_scale(
        inner: &'a dyn CoDesignProblem,
        residual_scale: f64,
        log_space: bool,
    ) -> Self {
        assert!(
            residual_scale.is_finite() && residual_scale > 0.0,
            "residual_scale must be finite and > 0, got {residual_scale}",
        );
        // `new` rejects the residual `inf`/0 cases (L→0 ⇒ 1/L²→inf; L→inf ⇒ 0).
        Self::new(inner, residual_scale.powi(-2), log_space)
    }

    /// Map physical design parameters to the space the optimizer works in
    /// (`ln` when [`log_space`](Self) is set, else identity). Use it on the
    /// initial guess `x0` before [`optimize`].
    ///
    /// # Panics
    /// In `log_space`, panics if any parameter is not strictly positive
    /// (`ln` of a non-positive value is undefined).
    #[must_use]
    pub fn to_normalized(&self, physical: &[f64]) -> Vec<f64> {
        if self.log_space {
            physical
                .iter()
                .map(|&x| {
                    assert!(x > 0.0, "log_space requires positive params, got {x}");
                    x.ln()
                })
                .collect()
        } else {
            physical.to_vec()
        }
    }

    /// Map optimizer-space parameters (e.g. an [`OptResult`]'s `params`) back to
    /// physical values (`exp` when [`log_space`](Self) is set, else identity).
    #[must_use]
    pub fn to_physical(&self, normalized: &[f64]) -> Vec<f64> {
        if self.log_space {
            normalized.iter().map(|&p| p.exp()).collect()
        } else {
            normalized.to_vec()
        }
    }

    /// A starting-point [`OptConfig`] for the normalized space, on the **standard
    /// `eps = 1e-8`** (the point of normalizing — no special eps). In
    /// [`log_space`](Self) the parameter step is fractional, so it uses a small
    /// relative `lr = 0.05` (the default `2e3` is a physical-μ step and would
    /// explode in log-space); otherwise it keeps the default `lr`. `loss_tol` is
    /// tightened to `1e-15` on the (typically dimensionless) loss so the weakly
    /// sensitive recovery descends deep enough; treat both as tunable starting
    /// values, not universal constants.
    #[must_use]
    pub fn recommended_config(&self) -> OptConfig {
        OptConfig {
            lr: if self.log_space {
                0.05
            } else {
                OptConfig::default().lr
            },
            loss_tol: 1.0e-15,
            max_iters: 400,
            ..OptConfig::default()
        }
    }

    /// Run [`optimize`] with the physical↔normalized mapping bracketed: `x0` is
    /// given in **physical** units and the returned [`OptResult`]'s `params` (and
    /// every `history` record's `params`) are mapped back to physical units. This
    /// is the footgun-free entry point — a caller never touches the optimizer's
    /// (possibly log) space, so a `log_space` wrapper can't be driven with an
    /// un-transformed `x0` by mistake. (`loss`/`grad_inf` stay in the normalized
    /// space the optimizer worked in.)
    #[must_use]
    pub fn optimize(&self, x0_physical: &[f64], cfg: &OptConfig) -> OptResult {
        let mut result = optimize(self, &self.to_normalized(x0_physical), cfg);
        result.params = self.to_physical(&result.params);
        for rec in &mut result.history {
            rec.params = self.to_physical(&rec.params);
        }
        result
    }
}

impl CoDesignProblem for Normalized<'_> {
    fn n_params(&self) -> usize {
        self.inner.n_params()
    }

    fn evaluate(&self, params: &[f64]) -> (f64, Vec<f64>) {
        let physical = self.to_physical(params);
        let (loss, grad) = self.inner.evaluate(&physical);
        debug_assert_eq!(
            grad.len(),
            physical.len(),
            "inner CoDesignProblem returned a gradient of the wrong length",
        );
        // dL_norm/dp = loss_scale · (dx/dp) · ∂L/∂x, with dx/dp = x in log-space
        // (chain rule, since x = exp(p)) or 1 otherwise.
        let scaled = grad
            .iter()
            .zip(&physical)
            .map(|(&g, &x)| {
                let dx_dp = if self.log_space { x } else { 1.0 };
                self.loss_scale * dx_dp * g
            })
            .collect();
        (loss * self.loss_scale, scaled)
    }

    fn lower_bounds(&self) -> Option<Vec<f64>> {
        // In log-space `param > 0` is automatic (exp is always positive), so a
        // finite positive floor maps through `ln` (a no-op safety net), and a
        // non-positive floor — meaningless in log-space — maps to −∞ (no clamp)
        // rather than panicking, keeping this a total combinator. The loss_scale
        // does not affect bounds.
        self.inner.lower_bounds().map(|lb| {
            if self.log_space {
                lb.iter()
                    .map(|&b| if b > 0.0 { b.ln() } else { f64::NEG_INFINITY })
                    .collect()
            } else {
                lb
            }
        })
    }
}

/// Build a [`StaggeredCoupling`] (a soft block under a rigid platen) from the
/// shared coupling parameters — the construction both [`SoftMaterialTarget`] and
/// [`SoftMaterialTrajectoryTarget`] use. They differ only in the outcome they read
/// (single-step `vz'` vs trajectory `z_N`), not in how the scene is built.
//
// expect: a malformed fixture MJCF / coupling is a caller error surfaced loudly —
// the canonical fixture idiom, mirroring `sim-coupling`'s tests.
#[allow(clippy::expect_used, clippy::too_many_arguments)]
fn build_coupling(
    mjcf: &str,
    body: usize,
    contact_clearance: f64,
    n_per_edge: usize,
    edge: f64,
    mu: f64,
    dt: f64,
    kappa: f64,
    d_hat: f64,
    rigid_damping: f64,
) -> StaggeredCoupling {
    let model = sim_mjcf::load_model(mjcf).expect("co-design coupling fixture: MJCF loads");
    let mut data = model.make_data();
    data.forward(&model)
        .expect("co-design coupling fixture: initial forward");
    StaggeredCoupling::new(
        model,
        data,
        body,
        contact_clearance,
        n_per_edge,
        edge,
        mu,
        dt,
        kappa,
        d_hat,
        rigid_damping,
    )
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
    fn build(&self, mu: f64) -> StaggeredCoupling {
        build_coupling(
            &self.mjcf,
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
/// keystone scene) and the raw loss gradient is `~2e-10` — *below* Adam's
/// standard `eps = 1e-8`, so optimizing this target directly would crawl
/// (eps-dominated, see [`OptConfig::eps`]). The fix is to condition the objective
/// with [`Normalized`] (a dimensionless residual + log-μ relative steps), which
/// lifts the gradient above the standard `eps` — so you keep `eps = 1e-8` rather
/// than chasing a fragile per-scene tiny value. See [`Normalized`] and
/// `docs/codesign/recon.md` §v3. The objective is monotone in μ (a stiffer block
/// holds the platen higher), so `target_z = z_N(μ*)` has the unique minimizer `μ*`.
///
/// Because `coupled_trajectory_material_gradient` takes `&mut self` (it advances
/// the rollout), each `evaluate` rebuilds the coupling **once per parameter
/// index** (twice total), unlike the single-step target's one shared build.
///
/// ```no_run
/// use cf_codesign::{Normalized, optimize, SoftMaterialTrajectoryTarget};
/// # const MJCF: &str = "";
/// let target = SoftMaterialTrajectoryTarget::for_inverse_design(MJCF.to_string(), 20, 0.124);
/// // z_N is weakly sensitive — condition it so the STANDARD eps converges.
/// let problem = Normalized::with_residual_scale(&target, 0.1, true);
/// let cfg = problem.recommended_config();
/// let x0 = problem.to_normalized(&[20_000.0]);
/// let result = optimize(&problem, &x0, &cfg);
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

    /// Build the coupling at stiffness `mu` (`λ = 4μ` via `StaggeredCoupling`).
    fn build(&self, mu: f64) -> StaggeredCoupling {
        build_coupling(
            &self.mjcf,
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
