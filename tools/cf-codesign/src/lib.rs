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
//! ([`sim_ml_chassis::OptimizerConfig`]) rather than reinventing one. The mission
//! aims at "one outer loop differentiating w.r.t. **both** design and policy
//! parameters"; the concrete problems here exercise each axis separately (a single
//! design parameter, a control schedule, or a closed-loop policy) AND **jointly**
//! ([`JointTarget`] — both axes on one tape, one backward), each backed by a worked
//! **inverse-design** demo that recovers a target behavior:
//!
//! *Design axis* — tune a soft body's Neo-Hookean material so a rigid-side
//! outcome hits a target:
//! - [`SoftMaterialTarget`] (v1) — the rigid body's *next-step velocity* (the
//!   keystone single-step gradient `∂vz'/∂μ`).
//! - [`SoftMaterialTrajectoryTarget`] (v2) — the rigid body's *height after an
//!   N-step contact-engaged coupled rollout* (the keystone multi-step gradient
//!   `∂z_N/∂μ`, one `tape.backward` across both engines and every step boundary).
//!
//! *Policy axis* — tune the control inputs applied each step:
//! - [`ControlScheduleTarget`] — an **open-loop** per-step platen control-force
//!   schedule `u_0 … u_{N−1}` so the platen's height after the coupled rollout
//!   hits a target (the keystone multi-step *control* gradient `∂z_N/∂u_k`, the
//!   whole vector from one `tape.backward`). The control force adds to the
//!   platen's `xfrc_applied`, so it rides the same coupled tape as the material
//!   gradient.
//! - [`FeedbackPolicyTarget`] — a **closed-loop** state-feedback policy
//!   `u_k = π_θ(state_k)` ([`DiffPolicy`]/[`LinearFeedback`]) whose parameters θ
//!   are tuned so the platen's height after the rollout hits a target (the
//!   keystone closed-loop *policy* gradient `∂z_N/∂θ`, backprop-through-time
//!   across the state→control recurrence from one `tape.backward`).
//!
//! *Joint axis* — the mission's "one outer loop differentiating w.r.t. **both**
//! design and policy parameters":
//! - [`JointTarget`] — optimize the soft material `μ` **and** a feedback policy `θ`
//!   *together* so the platen's height hits a target, reading BOTH gradient blocks
//!   `(∂z_N/∂μ_total, ∂z_N/∂θ)` from ONE
//!   [`StaggeredCoupling::coupled_trajectory_joint_gradient`] backward. It owns the
//!   mixed conditioning (positive `μ` log-reparametrized internally, signed `θ`
//!   linear) so a single `loss_scale`-conditioned Adam loop drives both.
//!
//! *System-ID axis* — recover *rigid dynamics* parameters from an observed
//! trajectory (what sim-to-real calibration needs), rather than tuning a design:
//! - [`DampingSystemId`] — recover a pure-rigid model's **joint-damping**
//!   coefficients so its terminal state after an N-step rollout matches an
//!   observed one, reading `∂x_N/∂D` from the analytic rigid-parameter channel
//!   [`sim_core::mjd_damping_trajectory_jacobian`] (forward-sensitivity
//!   `s_{t+1} = A_t·s_t + P_t`). The first gradient path to a *rigid* parameter
//!   (mass/damping/inertia/friction), where the other axes all tune a design.
//!   Recovers several dampings at once; the Gauss-Newton `JᵀJ` conditioning is
//!   reported by [`DampingSystemId::observation_jacobian`] with [`identifiability`]
//!   so a parameter trade-off (weak joint-identifiability) shows up as data,
//!   not a silent stall.
//!
//! A weakly-sensitive objective (a tiny gradient, like the position-valued
//! trajectory outcome `z_N`) is conditioned for the optimizer by the general
//! [`Normalized`] wrapper — a dimensionless residual plus log-space (relative)
//! parameter steps — so the **standard** Adam `eps` keeps its scale-invariance
//! instead of crawling (rather than chasing a fragile per-scene tiny `eps`).
//!
//! Scope: a single design parameter (material), an open-loop control schedule, a
//! closed-loop state-feedback policy, or joint design+policy, all in the keystone's
//! contact-engaged regime. Richer policies (MLP), multi-parameter design, and
//! manufacturing-constrained co-optimization are documented follow-ons
//! (`docs/codesign/recon.md`).
//!
//! ```no_run
//! use cf_codesign::{optimize, OptConfig, SoftMaterialTarget};
//! # const MJCF: &str = "";
//! // Target behavior = the rigid outcome a reference material μ* produces.
//! let probe = SoftMaterialTarget::for_inverse_design(MJCF.to_string(), 0.099, 40_000.0);
//! let result = optimize(&probe, &[20_000.0], &OptConfig::default());
//! assert!(result.converged);
//! ```

use sim_coupling::{DiffPolicy, LinearFeedback, StaggeredCoupling};
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
/// For a [`Normalized`] problem with a reparametrized (e.g. log) space, prefer
/// [`Normalized::optimize`], which brackets the physical↔normalized mapping so
/// `x0` and the result stay in physical units.
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
    /// Panics unless `residual_scale` is finite and strictly positive; and, if it
    /// is so small that `1/L²` overflows to `inf`, the forwarded [`new`](Self::new)
    /// finite-`loss_scale` check fires.
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
    /// Ordinary callers should prefer [`Self::optimize`], which applies this
    /// mapping for them; this is exposed for manual control (e.g. finite-difference
    /// checks in the optimizer's own space).
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
    /// tightened to `1e-15` (on the typically dimensionless loss) and `max_iters`
    /// raised to `400` so the weakly sensitive recovery descends deep enough; treat
    /// all three as tunable starting values, not universal constants.
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
    ///
    /// # Panics
    /// Panics if `x0_physical.len() != n_params()` (via [`optimize`]), or — in
    /// [`log_space`](Self) — if any `x0_physical` is not strictly positive (via
    /// [`to_normalized`](Self::to_normalized)).
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
        let scaled: Vec<f64> = grad
            .iter()
            .zip(&physical)
            .map(|(&g, &x)| {
                let dx_dp = if self.log_space { x } else { 1.0 };
                self.loss_scale * dx_dp * g
            })
            .collect();
        let scaled_loss = loss * self.loss_scale;
        // A `log_space` parameter `p` is unbounded above, so `μ = exp(p)` can
        // overflow to `+inf` and poison the run — silently, since the optimizer
        // would only record `converged = false` with garbage params. The
        // load-bearing quantity is the GRADIENT (`opt.step_in_place` feeds it into
        // `params`), and the `exp(p)` overflow lands in `dx/dp` here, so guard the
        // scaled gradient AND the loss (the convergence test reads the loss; and
        // `grad_inf`'s `f64::max` would mask a NaN gradient component, leaving the
        // step to corrupt `params` undetected). Unreachable at the shipped configs
        // (`lr · max_iters` keeps `p ≈ O(10)`, far from the `p ≈ 709` overflow), so
        // it is pure insurance — but a `debug_assert!` would compile out of
        // release-heavy CI and real long runs (the only place a runaway could
        // occur), so it must be a plain `assert!`. The cost is one `is_finite` per
        // value per iteration, negligible against the coupled rollout it guards —
        // the safety net any reusable combinator should carry.
        assert!(
            scaled_loss.is_finite() && scaled.iter().all(|g| g.is_finite()),
            "Normalized produced a non-finite loss/gradient at params {physical:?} \
             (loss {scaled_loss}; a log_space parameter may have overflowed to ±inf)",
        );
        (scaled_loss, scaled)
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
/// use cf_codesign::{Normalized, SoftMaterialTrajectoryTarget};
/// # const MJCF: &str = "";
/// let target = SoftMaterialTrajectoryTarget::for_inverse_design(MJCF.to_string(), 20, 0.124);
/// // z_N is weakly sensitive — condition it so the STANDARD eps converges.
/// let problem = Normalized::with_residual_scale(&target, 0.1, true);
/// let cfg = problem.recommended_config();
/// // `optimize` brackets the physical↔normalized mapping (x0 + result in μ units).
/// let result = problem.optimize(&[20_000.0], &cfg);
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

/// A co-design problem over an **open-loop control schedule** — the policy half
/// of the co-design loop. The design parameters are a per-step vertical control
/// force on the platen, `u_0 … u_{N−1}`; tune them so the platen's height `z_N`
/// after the `n_steps` coupled rollout matches `target_z`. Loss
/// `½(z_N − target_z)²`. The soft material is held fixed (a *joint* design+policy
/// problem — optimizing μ and the schedule together — is a documented follow-on).
///
/// This is the first consumer of the keystone's multi-step **control** gradient:
/// each [`evaluate`](CoDesignProblem::evaluate) reads `(z_N, [∂z_N/∂u_k])` from
/// [`StaggeredCoupling::coupled_trajectory_control_gradient`] in ONE
/// `tape.backward` across both engines and every step boundary, so the whole
/// gradient vector comes from a single coupled rollout (unlike the material
/// target, which rebuilds once per scalar parameter). The control gradient flows
/// the same way the material one does — the control force adds to the platen's
/// `xfrc_applied`, so it rides the same rigid carry as the contact reaction.
///
/// **Under-determination (honest scope).** There are `n_steps` controls for one
/// scalar target, so the inverse problem is under-determined: gradient descent
/// finds *a* schedule achieving `z_N = target_z`, not a unique one (and the final
/// control has no effect on `z_N` at all — its velocity bump never integrates
/// into a height; see [`StaggeredCoupling::coupled_trajectory_control_gradient`]).
/// The objective is therefore behavior-recovery (hit the target platen height),
/// the natural open-loop trajectory-optimization framing — not parameter recovery.
/// Every `∂z_N/∂u_k ≥ 0` (a larger upward force raises the final height), so the
/// loss is well-behaved for descent.
///
/// **Conditioning.** Like the trajectory material target, `z_N` is a position so
/// `∂z_N/∂u_k` is small (`~1e-5` for the keystone scene) and the raw loss gradient
/// falls below Adam's standard `eps`. Condition with [`Normalized`] — but a
/// control force is **signed** (can be zero or negative), so the log-space lever
/// does NOT apply; use the `loss_scale` (dimensionless-residual) lever only
/// (`log_space = false`) with a control-appropriate `lr`. See
/// [`Self::recommended_normalized`].
///
/// Because [`StaggeredCoupling::coupled_trajectory_control_gradient`] takes
/// `&mut self` (it advances the rollout), each `evaluate` rebuilds the coupling
/// once (a single build, since one backward yields the whole gradient).
pub struct ControlScheduleTarget {
    mjcf: String,
    /// Rigid body index in the model (the platen).
    body: usize,
    contact_clearance: f64,
    n_per_edge: usize,
    edge: f64,
    /// Fixed soft-block stiffness `μ` (the material is not a design variable here).
    mu: f64,
    dt: f64,
    kappa: f64,
    d_hat: f64,
    rigid_damping: f64,
    /// Number of coupled steps = number of control inputs.
    n_steps: usize,
    /// Target platen height after the rollout.
    target_z: f64,
}

impl ControlScheduleTarget {
    /// Construct with explicit coupling parameters and a fixed soft stiffness `mu`.
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        mjcf: String,
        body: usize,
        contact_clearance: f64,
        n_per_edge: usize,
        edge: f64,
        mu: f64,
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
            mu,
            dt,
            kappa,
            d_hat,
            rigid_damping,
            n_steps,
            target_z,
        }
    }

    /// Convenience for the worked open-loop control demo with the keystone fixture
    /// (the `platen` body 1, clearance 5 mm, 4³ block edge 0.1 m, μ = 3e4,
    /// dt 1 ms, κ = 3e4, d̂ = 1e-2, damping 60 — the contact-engaged rollout of the
    /// keystone trajectory gate). The platen MJCF should start already in contact
    /// (e.g. `pos.z = 0.108`). `target_z` is the platen height to drive to (e.g.
    /// set it to [`Self::forward_z`] of a reference schedule).
    #[must_use]
    pub fn for_inverse_design(mjcf: String, n_steps: usize, target_z: f64) -> Self {
        Self::new(
            mjcf, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 60.0, n_steps, target_z,
        )
    }

    /// Build the coupling at the fixed stiffness `mu` (`λ = 4μ`).
    fn build(&self) -> StaggeredCoupling {
        build_coupling(
            &self.mjcf,
            self.body,
            self.contact_clearance,
            self.n_per_edge,
            self.edge,
            self.mu,
            self.dt,
            self.kappa,
            self.d_hat,
            self.rigid_damping,
        )
    }

    /// The platen's height `z_N` after the `n_steps` coupled rollout under the
    /// control schedule `controls` — the forward outcome (no gradient). Used to
    /// set up an inverse-design target.
    ///
    /// # Panics
    /// Panics if `controls.len() != n_steps` (the rollout length is the schedule
    /// length, so a mismatch would silently run a different-length rollout than
    /// [`n_params`](CoDesignProblem::n_params) advertises).
    #[must_use]
    pub fn forward_z(&self, controls: &[f64]) -> f64 {
        assert_eq!(
            controls.len(),
            self.n_steps,
            "control schedule length {} != n_steps {}",
            controls.len(),
            self.n_steps,
        );
        self.build().coupled_trajectory_control_z(controls)
    }

    /// The target this problem optimizes toward.
    #[must_use]
    pub const fn target_z(&self) -> f64 {
        self.target_z
    }

    /// A [`Normalized`] wrapper conditioning this target for the standard Adam
    /// `eps`: a dimensionless residual (`loss_scale = 1/L²`, `L` the
    /// characteristic platen-height scale) with `log_space = false` (control
    /// forces are signed — log-space requires positive parameters). Drive it with
    /// [`Normalized::optimize`] and an [`OptConfig`] on the **standard** `eps` but
    /// a control-appropriate `lr` (an `O(1)`-newton step, e.g. `lr = 0.02` — the
    /// default physical-`μ` `lr = 2e3` is far too large for a control force, and
    /// is why this returns only the wrapper, not a full config). See
    /// `examples/control_inverse_design.rs` and `docs/codesign/recon.md`.
    #[must_use]
    pub fn recommended_normalized(&self, residual_scale: f64) -> Normalized<'_> {
        Normalized::with_residual_scale(self, residual_scale, false)
    }
}

impl CoDesignProblem for ControlScheduleTarget {
    fn n_params(&self) -> usize {
        self.n_steps
    }

    /// `(loss, gradient)` at the control schedule `params`.
    ///
    /// # Panics
    /// Panics if `params.len() != n_steps` (the rollout length is the schedule
    /// length; [`optimize`] already guards this, but a direct caller must match).
    fn evaluate(&self, params: &[f64]) -> (f64, Vec<f64>) {
        assert_eq!(
            params.len(),
            self.n_steps,
            "control schedule length {} != n_steps {}",
            params.len(),
            self.n_steps,
        );
        // One coupled rollout yields z_N and the whole control gradient vector.
        let (z_n, dz_du) = self.build().coupled_trajectory_control_gradient(params);
        let residual = z_n - self.target_z;
        let grad = dz_du.iter().map(|&g| residual * g).collect();
        (0.5 * residual * residual, grad)
    }

    // Control forces are signed (push up or down), so there is no lower bound —
    // and `lower_bounds = None` keeps the `Normalized` log-space lever inapplicable
    // here by construction (it is the signed-parameter case).
}

/// A co-design problem over a **closed-loop feedback policy**: tune the policy
/// parameters `θ` of `u_k = π_θ(state_k)` ([`DiffPolicy`]) so the platen's height
/// after an `n_steps` coupled rollout hits `target_z`. Loss `½(z_N − target_z)²`.
///
/// The closed-loop analogue of [`ControlScheduleTarget`]. There the parameters are
/// a per-step *schedule* (each `u_k` free); here they are the *policy weights*,
/// shared across every step, and the control at each step is a function of the
/// platen state — a genuine feedback law. Each
/// [`evaluate`](CoDesignProblem::evaluate) reads `(z_N, ∂z_N/∂θ)` from one
/// [`StaggeredCoupling::coupled_trajectory_policy_gradient`] — a single
/// `tape.backward` that backpropagates through time across the state→control
/// recurrence, so the feedback weights' gradient (which flows *only* through the
/// recurrence) is captured exactly.
///
/// **Under-determination (honest scope).** A `P`-parameter policy against one
/// scalar target `z_N` is under-determined for `P > 1`: gradient descent finds *a*
/// policy achieving `z_N = target_z`, not a unique one. The objective is
/// behavior-recovery (hit the target platen height), the natural trajectory-
/// optimization framing — not parameter recovery.
///
/// **Conditioning.** `z_N` is a position so `∂z_N/∂θ` is small (`~1e-5..1e-4`) and
/// the raw loss gradient falls below Adam's standard `eps`. Condition with
/// [`Normalized`]. The first policy ([`LinearFeedback`]) has **signed** weights, so
/// the log-space lever does NOT apply — use the `loss_scale` (dimensionless-
/// residual) lever only (`log_space = false`) with a state-feedback-appropriate
/// `lr`. See [`Self::recommended_normalized`]. (A positive-gain PD parametrization,
/// if added, may re-open the log-space lever — re-measure then.)
///
/// Generic over the policy `P` so a richer policy (e.g. an MLP `DiffPolicy`) plugs
/// into the same loop; the concrete `FeedbackPolicyTarget<LinearFeedback>` coerces
/// to `&dyn CoDesignProblem` for [`optimize`]/[`Normalized`].
pub struct FeedbackPolicyTarget<P: DiffPolicy> {
    mjcf: String,
    /// Rigid body index in the model (the platen).
    body: usize,
    contact_clearance: f64,
    n_per_edge: usize,
    edge: f64,
    /// Fixed soft-block stiffness `μ` (the material is not a design variable here).
    mu: f64,
    dt: f64,
    kappa: f64,
    d_hat: f64,
    rigid_damping: f64,
    /// Number of coupled steps in the rollout.
    n_steps: usize,
    /// Target platen height after the rollout.
    target_z: f64,
    /// The feedback policy whose parameters are optimized.
    policy: P,
}

impl<P: DiffPolicy> FeedbackPolicyTarget<P> {
    /// Construct with explicit coupling parameters, a fixed soft stiffness `mu`,
    /// and the feedback `policy`.
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        mjcf: String,
        body: usize,
        contact_clearance: f64,
        n_per_edge: usize,
        edge: f64,
        mu: f64,
        dt: f64,
        kappa: f64,
        d_hat: f64,
        rigid_damping: f64,
        n_steps: usize,
        target_z: f64,
        policy: P,
    ) -> Self {
        Self {
            mjcf,
            body,
            contact_clearance,
            n_per_edge,
            edge,
            mu,
            dt,
            kappa,
            d_hat,
            rigid_damping,
            n_steps,
            target_z,
            policy,
        }
    }

    /// Build the coupling at the fixed stiffness `mu` (`λ = 4μ`).
    fn build(&self) -> StaggeredCoupling {
        build_coupling(
            &self.mjcf,
            self.body,
            self.contact_clearance,
            self.n_per_edge,
            self.edge,
            self.mu,
            self.dt,
            self.kappa,
            self.d_hat,
            self.rigid_damping,
        )
    }

    /// The platen's height `z_N` after the `n_steps` closed-loop coupled rollout
    /// under the policy params — the forward outcome (no gradient). Used to set up
    /// an inverse-design target.
    #[must_use]
    pub fn forward_z(&self, params: &[f64]) -> f64 {
        self.build()
            .coupled_trajectory_policy_z(&self.policy, params, self.n_steps)
    }

    /// The target this problem optimizes toward.
    #[must_use]
    pub const fn target_z(&self) -> f64 {
        self.target_z
    }

    /// A [`Normalized`] wrapper conditioning this target for the standard Adam
    /// `eps`: a dimensionless residual (`loss_scale = 1/L²`, `L` the characteristic
    /// platen-height scale) with `log_space = false` (the first policy's weights are
    /// signed — log-space requires positive parameters). Drive it with
    /// [`Normalized::optimize`] and an [`OptConfig`] on the **standard** `eps` but a
    /// state-feedback-appropriate `lr`. See `examples/policy_inverse_design.rs`.
    #[must_use]
    pub fn recommended_normalized(&self, residual_scale: f64) -> Normalized<'_> {
        Normalized::with_residual_scale(self, residual_scale, false)
    }
}

impl FeedbackPolicyTarget<LinearFeedback> {
    /// Convenience for the worked closed-loop demo with the keystone fixture (the
    /// `platen` body 1, clearance 5 mm, 4³ block edge 0.1 m, μ = 3e4, dt 1 ms,
    /// κ = 3e4, d̂ = 1e-2, damping 60 — the contact-engaged rollout) and a
    /// [`LinearFeedback`] policy `u = w_z·z + w_vz·vz + b`. The platen MJCF should
    /// start already in contact (e.g. `pos.z = 0.108`). `target_z` is the platen
    /// height to drive to (e.g. set it to [`Self::forward_z`] of a reference policy).
    #[must_use]
    pub fn linear_for_inverse_design(mjcf: String, n_steps: usize, target_z: f64) -> Self {
        Self::new(
            mjcf,
            1,
            0.005,
            4,
            0.1,
            3.0e4,
            1.0e-3,
            3.0e4,
            1.0e-2,
            60.0,
            n_steps,
            target_z,
            LinearFeedback,
        )
    }
}

impl<P: DiffPolicy> CoDesignProblem for FeedbackPolicyTarget<P> {
    fn n_params(&self) -> usize {
        self.policy.n_params()
    }

    /// `(loss, gradient)` at the policy parameters `params`.
    ///
    /// # Panics
    /// Panics if `params.len() != policy.n_params()` (the gradient call asserts it).
    fn evaluate(&self, params: &[f64]) -> (f64, Vec<f64>) {
        // One closed-loop coupled rollout yields z_N and the whole ∂z_N/∂θ vector.
        let (z_n, dz_dtheta) =
            self.build()
                .coupled_trajectory_policy_gradient(&self.policy, params, self.n_steps);
        let residual = z_n - self.target_z;
        let grad = dz_dtheta.iter().map(|&g| residual * g).collect();
        (0.5 * residual * residual, grad)
    }

    // The first policy's weights are signed (no lower bound), so `lower_bounds =
    // None` keeps the `Normalized` log-space lever inapplicable by construction
    // (the signed-parameter case, like `ControlScheduleTarget`).
}

/// **The joint design+policy co-design problem** — the mission's "one outer loop
/// differentiating w.r.t. **both** design and policy parameters". It optimizes the
/// soft material design variable `μ` (stiffness scale, `λ = 4μ`) AND a closed-loop
/// feedback policy `u_k = π_θ(state_k)` *together* so the platen's height after the
/// coupled rollout hits `target_z`. Each [`evaluate`](CoDesignProblem::evaluate)
/// reads BOTH gradient blocks `(∂z_N/∂μ_total, ∂z_N/∂θ)` from ONE
/// [`StaggeredCoupling::coupled_trajectory_joint_gradient`] backward pass.
///
/// **Mixed conditioning (the joint-specific choice).** `μ` is a *positive* scale
/// parameter (the log-space lever applies — relative steps, as the v3 material
/// target found) while the policy weights `θ` are *signed* (log-space does not
/// apply — `loss_scale` only, as the control/policy targets found). A single
/// [`Normalized`] `log_space` flag cannot serve both. This target therefore owns
/// the **`μ` log-reparametrization internally**: its parameter vector is
/// `p = [ln μ, θ_0, θ_1, …]` (so the `μ` slot takes *relative* Adam steps and `μ`
/// stays positive by construction), and `evaluate` applies the `dμ/d(ln μ) = μ`
/// chain rule to the `μ` gradient component while passing the `θ` components
/// through linearly. The whole `p`-vector is then uniformly linear, so a single
/// [`Normalized`] with the `loss_scale` (dimensionless-residual) lever and one `lr`
/// conditions both blocks (the relative-`μ` and linear-`θ` steps are both `O(1)`).
/// Use [`Self::recommended_normalized`] + [`Self::x0`].
///
/// **Under-determination.** `1 + P` parameters against one scalar target is
/// under-determined — this is behavior-recovery (hit `target_z`), the natural
/// co-design framing, not unique `(μ*, θ*)` recovery.
///
/// Generic over the policy `P`; `JointTarget<LinearFeedback>` coerces to
/// `&dyn CoDesignProblem`.
pub struct JointTarget<P: DiffPolicy> {
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
    policy: P,
}

impl<P: DiffPolicy> JointTarget<P> {
    /// Construct with explicit coupling parameters and the feedback `policy`.
    /// (`μ` is a design variable here, not a constructor argument — it enters
    /// through the optimized parameter vector as `ln μ`.)
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
        policy: P,
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
            policy,
        }
    }

    /// Build the coupling at the physical stiffness `mu` (`λ = 4μ`).
    fn build_at(&self, mu: f64) -> StaggeredCoupling {
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

    /// The platen's height `z_N` after the closed-loop rollout at physical
    /// stiffness `mu` under policy params `theta` — the forward outcome (no
    /// gradient). Takes PHYSICAL `μ` (not `ln μ`); used to set up the target.
    #[must_use]
    pub fn forward_z(&self, mu: f64, theta: &[f64]) -> f64 {
        self.build_at(mu)
            .coupled_trajectory_policy_z(&self.policy, theta, self.n_steps)
    }

    /// Assemble an optimizer start vector `p = [ln μ0, θ0…]` from a PHYSICAL
    /// `(mu0, theta0)`. The optimizer works in this `p`-space (see the type docs).
    ///
    /// # Panics
    /// Panics if `mu0 <= 0` (its logarithm is the optimized parameter) or
    /// `theta0.len() != policy.n_params()`.
    #[must_use]
    pub fn x0(&self, mu0: f64, theta0: &[f64]) -> Vec<f64> {
        assert!(
            mu0 > 0.0,
            "mu0 must be positive (the optimized param is ln μ), got {mu0}"
        );
        assert_eq!(
            theta0.len(),
            self.policy.n_params(),
            "theta0 length {} != policy n_params {}",
            theta0.len(),
            self.policy.n_params(),
        );
        let mut p = Vec::with_capacity(1 + theta0.len());
        p.push(mu0.ln());
        p.extend_from_slice(theta0);
        p
    }

    /// Read the PHYSICAL design `(μ, θ)` back out of an optimizer parameter vector
    /// `p = [ln μ, θ…]` (`μ = exp(p[0])`).
    ///
    /// # Panics
    /// Panics if `p` is empty.
    #[must_use]
    pub fn to_physical(&self, p: &[f64]) -> (f64, Vec<f64>) {
        assert!(!p.is_empty(), "joint parameter vector must be non-empty");
        (p[0].exp(), p[1..].to_vec())
    }

    /// The target this problem optimizes toward.
    #[must_use]
    pub const fn target_z(&self) -> f64 {
        self.target_z
    }

    /// A [`Normalized`] wrapper conditioning this target for the standard Adam
    /// `eps`: a dimensionless residual (`loss_scale = 1/L²`) with `log_space =
    /// false` — the `μ` log-reparametrization is already done *inside* this target
    /// (the parameter vector is `[ln μ, θ…]`), so `Normalized` only needs to supply
    /// the `loss_scale` lever uniformly over the now-linear `p`-vector. Drive it
    /// with [`Normalized::optimize`] (or [`optimize`]) and an [`OptConfig`] on the
    /// **standard** `eps` and a single `lr ~ 0.03` (both the relative-`μ` and the
    /// linear-`θ` steps are `O(1)`). See `examples/joint_inverse_design.rs`.
    ///
    /// Note on conditioning: the log-μ reparametrization is what lets one `lr` serve
    /// the `3e4`-scale `μ` and the `O(1)` `θ` (relative vs linear steps), and it
    /// already lifts the `p`-space gradient near/above the standard `eps`, so the
    /// `loss_scale` lever here is additional headroom rather than strictly required
    /// (unlike the single-axis policy target, where the signed-parameter case makes
    /// `loss_scale` itself the conditioning lever). The under-determination (θ can
    /// compensate for a stuck μ) prevents a clean negative control that isolates the
    /// log-μ lever, so log-μ + `loss_scale` ship as a unit; that both axes actually
    /// move is shown directly by the recovery gate.
    #[must_use]
    pub fn recommended_normalized(&self, residual_scale: f64) -> Normalized<'_> {
        Normalized::with_residual_scale(self, residual_scale, false)
    }
}

impl JointTarget<LinearFeedback> {
    /// Convenience for the worked joint demo with the keystone fixture (the
    /// `platen` body 1, clearance 5 mm, 4³ block edge 0.1 m, dt 1 ms, κ = 3e4,
    /// d̂ = 1e-2, damping 60) and a [`LinearFeedback`] policy. The platen MJCF
    /// should start already in contact (e.g. `pos.z = 0.108`).
    #[must_use]
    pub fn linear_for_inverse_design(mjcf: String, n_steps: usize, target_z: f64) -> Self {
        Self::new(
            mjcf,
            1,
            0.005,
            4,
            0.1,
            1.0e-3,
            3.0e4,
            1.0e-2,
            60.0,
            n_steps,
            target_z,
            LinearFeedback,
        )
    }
}

impl<P: DiffPolicy> CoDesignProblem for JointTarget<P> {
    fn n_params(&self) -> usize {
        1 + self.policy.n_params()
    }

    /// `(loss, gradient)` at `p = [ln μ, θ…]`. The `μ` component of the gradient
    /// carries the `dμ/d(ln μ) = μ` chain-rule factor; the `θ` components are
    /// linear. One joint backward yields both blocks.
    ///
    /// # Panics
    /// Panics if `p.len() != 1 + policy.n_params()`.
    fn evaluate(&self, p: &[f64]) -> (f64, Vec<f64>) {
        assert_eq!(
            p.len(),
            1 + self.policy.n_params(),
            "joint param length {} != 1 + policy n_params {}",
            p.len(),
            1 + self.policy.n_params(),
        );
        let mu = p[0].exp();
        let theta = &p[1..];
        // ONE backward over the coupled rollout → both gradient blocks.
        let (z_n, dz_dmu, dz_dtheta) =
            self.build_at(mu)
                .coupled_trajectory_joint_gradient(&self.policy, theta, self.n_steps);
        let residual = z_n - self.target_z;
        let mut grad = Vec::with_capacity(p.len());
        // d loss / d(ln μ) = residual · ∂z/∂μ_total · μ (the log-μ chain rule).
        grad.push(residual * dz_dmu * mu);
        // d loss / dθ_i = residual · ∂z/∂θ_i (linear).
        grad.extend(dz_dtheta.iter().map(|&g| residual * g));
        (0.5 * residual * residual, grad)
    }

    // μ is reparametrized as ln μ (positive by construction, no bound needed) and
    // the θ weights are signed (no bound) — so `lower_bounds = None`.
}

/// **System-ID axis** — recover *rigid* dynamics parameters (joint damping) from
/// an observed trajectory, the gradient channel sim-to-real calibration needs.
///
/// Where the design/policy targets above tune a *design* parameter via the
/// coupling gradient, this problem tunes the **joint-damping coefficients** of a
/// pure-rigid [`sim_core`] model so that its terminal state after an `n_steps`
/// rollout matches an observed one. It consumes the analytic rigid-parameter
/// channel [`sim_core::mjd_damping_trajectory_jacobian`] (forward-sensitivity
/// `s_{t+1} = A_t·s_t + P_t`), turning damping system-ID from FD-only into a
/// one-pass analytic gradient.
///
/// Parameters are the damping coefficients on the joints in `joints`; the loss is
/// `½‖x_N(D) − x_N*‖²` over the terminal state in tangent space. Damping is
/// bounded below at zero ([`lower_bounds`](CoDesignProblem::lower_bounds)).
///
/// Preconditions follow [`sim_core::mjd_damping_trajectory_jacobian`]: Euler
/// integrator, hinge/slide joints, no tendons.
pub struct DampingSystemId {
    /// Rigid model; the `joints`' damping is overwritten each `evaluate`, other
    /// parameters (mass, geometry, untargeted damping) are held fixed.
    model: sim_core::Model,
    /// Initial state the rollout starts from.
    data0: sim_core::Data,
    /// Observed terminal state `x_N*` in tangent space (relative to `data0.qpos`).
    target_terminal: sim_core::DVector<f64>,
    /// Number of rollout steps the observation spans.
    n_steps: usize,
    /// Indices of the joints whose damping is identified (the parameter order).
    joints: Vec<usize>,
    /// Transition-derivative config for the per-step `A_t`.
    cfg: sim_core::DerivativeConfig,
}

impl DampingSystemId {
    /// Build a sim-to-sim **inverse-design** problem: generate the observed
    /// terminal state by rolling out at the *true* damping values
    /// `true_damping[i]` (on `joints[i]`), then forget them — `evaluate` recovers
    /// them from the trajectory. `model`'s untargeted joints keep their damping.
    ///
    /// # Panics
    /// Panics if `joints.len() != true_damping.len()`, or if the reference rollout
    /// fails (a malformed model/state).
    #[must_use]
    #[allow(clippy::expect_used)] // a failed reference rollout is fixture misuse.
    pub fn for_inverse_design(
        mut model: sim_core::Model,
        data0: sim_core::Data,
        n_steps: usize,
        joints: Vec<usize>,
        true_damping: &[f64],
    ) -> Self {
        assert_eq!(
            joints.len(),
            true_damping.len(),
            "joints and true_damping length mismatch",
        );
        for (&jid, &d) in joints.iter().zip(true_damping) {
            model.jnt_damping[jid] = d;
        }
        model.compute_implicit_params();
        let cfg = sim_core::DerivativeConfig::default();
        let target_terminal =
            sim_core::mjd_damping_trajectory_jacobian(&model, &data0, n_steps, &cfg)
                .expect("system-ID reference rollout")
                .terminal_state;
        Self {
            model,
            data0,
            target_terminal,
            n_steps,
            joints,
            cfg,
        }
    }

    /// Roll out at `params` and return the analytic terminal-state Jacobian.
    #[allow(clippy::expect_used)] // a rollout failure is unrecoverable model misuse.
    fn rollout(&self, params: &[f64]) -> sim_core::TrajectoryDampingJacobian {
        let mut model = self.model.clone();
        for (&jid, &d) in self.joints.iter().zip(params) {
            model.jnt_damping[jid] = d;
        }
        model.compute_implicit_params();
        sim_core::mjd_damping_trajectory_jacobian(&model, &self.data0, self.n_steps, &self.cfg)
            .expect("system-ID trajectory jacobian")
    }

    /// The observation Jacobian `J = ∂x_N/∂θ` at `params` — sensitivity of the
    /// terminal observation to each identified damping parameter (columns in
    /// `joints` order, rows = terminal-state dim `2·nv + na`). Its conditioning
    /// (via [`identifiability`]) governs whether the parameters are *jointly*
    /// recoverable or trade off.
    #[must_use]
    pub fn observation_jacobian(&self, params: &[f64]) -> sim_core::DMatrix<f64> {
        let traj = self.rollout(params);
        let nx = traj.terminal_state.len();
        let mut j = sim_core::DMatrix::zeros(nx, self.joints.len());
        for (col, &jid) in self.joints.iter().enumerate() {
            let dof = self.model.jnt_dof_adr[jid];
            j.set_column(col, &traj.dterminal_dD.column(dof));
        }
        j
    }
}

impl CoDesignProblem for DampingSystemId {
    fn n_params(&self) -> usize {
        self.joints.len()
    }

    fn evaluate(&self, params: &[f64]) -> (f64, Vec<f64>) {
        let traj = self.rollout(params);
        // Residual over the terminal state; gradient flows through ∂x_N/∂D.
        let residual = &traj.terminal_state - &self.target_terminal;
        let loss = 0.5 * residual.dot(&residual);
        let grad = self
            .joints
            .iter()
            .map(|&jid| {
                let dof = self.model.jnt_dof_adr[jid];
                residual.dot(&traj.dterminal_dD.column(dof))
            })
            .collect();
        (loss, grad)
    }

    fn lower_bounds(&self) -> Option<Vec<f64>> {
        // Joint damping is physically non-negative.
        Some(vec![0.0; self.joints.len()])
    }
}

/// Identifiability summary of a least-squares system-ID problem, from the
/// singular values of an observation Jacobian `J = ∂(observation)/∂(params)`.
///
/// The Gauss-Newton / Fisher information is `JᵀJ`, whose eigenvalues are the
/// squared singular values of `J`. A large [`condition_number`](Self::condition_number)
/// (or a [`rank`](Self::rank) below the parameter count) means some parameter
/// combination is weakly observed — the parameters trade off and are not jointly
/// identifiable from this observation.
#[derive(Clone, Debug)]
pub struct Identifiability {
    /// Singular values of `J`, descending. `σ_i²` are the Fisher eigenvalues.
    pub singular_values: Vec<f64>,
    /// `σ_max / σ_min` (so `cond(JᵀJ) = condition_number²`). `+∞` if any singular
    /// value is zero — a rank-deficient, structurally unidentifiable direction.
    pub condition_number: f64,
    /// Numerical rank: singular values above `rel_tol · σ_max`. Equals the
    /// parameter count iff every parameter direction is independently observable.
    pub rank: usize,
}

/// Compute the [`Identifiability`] of an observation Jacobian `J` (rows =
/// observations, columns = parameters). `rel_tol` sets the rank threshold
/// relative to the largest singular value (e.g. `1e-9`).
///
/// Generic over any least-squares system-ID Jacobian — e.g. the one from
/// [`DampingSystemId::observation_jacobian`].
#[must_use]
pub fn identifiability(jacobian: &sim_core::DMatrix<f64>, rel_tol: f64) -> Identifiability {
    // nalgebra returns singular values in descending order.
    let singular_values: Vec<f64> = jacobian.clone().singular_values().iter().copied().collect();
    let s_max = singular_values.first().copied().unwrap_or(0.0);
    let s_min = singular_values.last().copied().unwrap_or(0.0);
    let condition_number = if s_min > 0.0 {
        s_max / s_min
    } else {
        f64::INFINITY
    };
    let threshold = rel_tol * s_max;
    let rank = singular_values.iter().filter(|&&s| s > threshold).count();
    Identifiability {
        singular_values,
        condition_number,
        rank,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A stub problem returning a non-finite loss — to verify [`Normalized`]'s
    /// finite-loss guard (an overflowed `log_space` `μ = exp(p)` could feed a NaN
    /// back and otherwise silently poison a run).
    struct NonFiniteLoss;
    impl CoDesignProblem for NonFiniteLoss {
        fn n_params(&self) -> usize {
            1
        }
        fn evaluate(&self, _params: &[f64]) -> (f64, Vec<f64>) {
            (f64::INFINITY, vec![0.0])
        }
    }

    /// A stub returning a FINITE loss but a non-finite GRADIENT — the load-bearing
    /// case (the gradient is what `step_in_place` feeds into the params, and a NaN
    /// component is masked by `grad_inf`'s `f64::max`).
    struct NonFiniteGrad;
    impl CoDesignProblem for NonFiniteGrad {
        fn n_params(&self) -> usize {
            1
        }
        fn evaluate(&self, _params: &[f64]) -> (f64, Vec<f64>) {
            (1.0, vec![f64::NAN])
        }
    }

    /// [`Normalized::evaluate`] panics (loudly) when the inner loss is non-finite,
    /// rather than scaling `inf`/`NaN` onward into a silent `converged = false`
    /// run. The guard is a plain `assert!` (always on, including release-heavy CI).
    #[test]
    #[should_panic(expected = "non-finite loss")]
    fn normalized_rejects_non_finite_inner_loss() {
        let wrapped = Normalized::new(&NonFiniteLoss, 1.0, false);
        let _ = wrapped.evaluate(&[1.0]);
    }

    /// The guard also covers a non-finite GRADIENT with a finite loss — the case
    /// `grad_inf`'s `f64::max` would otherwise mask, letting a NaN step corrupt the
    /// params undetected.
    #[test]
    #[should_panic(expected = "non-finite loss/gradient")]
    fn normalized_rejects_non_finite_inner_gradient() {
        let wrapped = Normalized::new(&NonFiniteGrad, 1.0, false);
        let _ = wrapped.evaluate(&[1.0]);
    }

    /// A finite inner loss passes the guard untouched (the common path is
    /// unaffected — the loss is just scaled by `loss_scale`).
    #[test]
    fn normalized_passes_finite_inner_loss() {
        struct FiniteLoss;
        impl CoDesignProblem for FiniteLoss {
            fn n_params(&self) -> usize {
                1
            }
            fn evaluate(&self, _params: &[f64]) -> (f64, Vec<f64>) {
                (2.0, vec![3.0])
            }
        }
        let wrapped = Normalized::new(&FiniteLoss, 0.5, false);
        let (loss, grad) = wrapped.evaluate(&[1.0]);
        assert!((loss - 1.0).abs() < 1e-15, "loss scaled by loss_scale");
        assert!((grad[0] - 1.5).abs() < 1e-15, "grad scaled by loss_scale");
    }

    // Platen already in contact (the keystone trajectory fixture).
    const POLICY_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.108">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

    /// Lib-level smoke test of [`FeedbackPolicyTarget`] (the scientific FD
    /// validation is in `tests/policy_inverse_design.rs`): the linear feedback
    /// policy advertises 3 params, the loss/gradient are finite with the correct
    /// `residual · ∂z_N/∂θ` sign wiring, and the feedback-weight gradients are
    /// nonzero (the closed-loop recurrence is live, not just the bias).
    #[test]
    fn feedback_policy_target_smoke() {
        let n_steps = 6;
        let setup =
            FeedbackPolicyTarget::linear_for_inverse_design(POLICY_MJCF.to_string(), n_steps, 0.0);
        assert_eq!(setup.n_params(), 3);

        let theta = [-15.0_f64, -3.0, 1.2];
        // Make the target the policy's OWN outcome ⇒ zero residual ⇒ zero gradient.
        let z_star = setup.forward_z(&theta);
        let at_target = FeedbackPolicyTarget::linear_for_inverse_design(
            POLICY_MJCF.to_string(),
            n_steps,
            z_star,
        );
        let (loss0, grad0) = at_target.evaluate(&theta);
        assert!(
            loss0 < 1e-20 && grad0.iter().all(|g| g.abs() < 1e-12),
            "zero residual ⇒ zero grad"
        );

        // Off-target: finite loss, the feedback weights carry nonzero gradient.
        let off = FeedbackPolicyTarget::linear_for_inverse_design(
            POLICY_MJCF.to_string(),
            n_steps,
            z_star + 1e-3,
        );
        let (loss, grad) = off.evaluate(&theta);
        assert_eq!(grad.len(), 3);
        assert!(loss > 0.0 && loss.is_finite() && grad.iter().all(|g| g.is_finite()));
        assert!(
            grad[0].abs() > 1e-12 && grad[1].abs() > 1e-12,
            "feedback-weight gradients should be nonzero (recurrence live), got {grad:?}"
        );
    }

    /// Lib-level smoke test of [`JointTarget`] (the scientific FD validation is in
    /// `tests/joint_inverse_design.rs`): `n_params = 1 + policy` over `[ln μ, θ]`,
    /// the `x0`/`to_physical` round-trip, a zero-residual point gives zero gradient,
    /// and an off-target point gives a finite gradient whose design (`ln μ`) and
    /// policy components are both live.
    #[test]
    fn joint_target_smoke() {
        let n_steps = 6;
        let setup = JointTarget::linear_for_inverse_design(POLICY_MJCF.to_string(), n_steps, 0.0);
        assert_eq!(setup.n_params(), 4);

        // x0 / to_physical round-trip.
        let p0 = setup.x0(3.0e4, &[-15.0, -3.0, 1.2]);
        let (mu_rt, th_rt) = setup.to_physical(&p0);
        assert!((mu_rt - 3.0e4).abs() < 1e-6 && (th_rt[0] + 15.0).abs() < 1e-12);

        // Zero residual ⇒ zero gradient (target = the point's own outcome).
        let z_star = setup.forward_z(3.0e4, &[-15.0, -3.0, 1.2]);
        let at = JointTarget::linear_for_inverse_design(POLICY_MJCF.to_string(), n_steps, z_star);
        let (loss0, grad0) = at.evaluate(&p0);
        assert!(
            loss0 < 1e-20 && grad0.iter().all(|g| g.abs() < 1e-12),
            "zero residual ⇒ zero grad"
        );

        // Off-target: finite gradient, both the design (ln μ) and policy blocks live.
        let off =
            JointTarget::linear_for_inverse_design(POLICY_MJCF.to_string(), n_steps, z_star + 1e-3);
        let (loss, grad) = off.evaluate(&p0);
        assert_eq!(grad.len(), 4);
        assert!(loss > 0.0 && grad.iter().all(|g| g.is_finite()));
        assert!(
            grad[0].abs() > 1e-12,
            "design (ln μ) block should be live, got {}",
            grad[0]
        );
        assert!(
            grad[1..].iter().any(|g| g.abs() > 1e-12),
            "policy block should be live, got {:?}",
            &grad[1..]
        );
    }

    /// System-ID: recover a rigid joint-damping coefficient (`d* = 0.8`) from a
    /// short rollout, starting from `d₀ = 0.2`, via the analytic
    /// `∂(trajectory)/∂(damping)` channel + the Adam chassis (the spike did this
    /// with FD-Newton; this pins the analytic, scalable version).
    #[test]
    fn recovers_joint_damping_from_trajectory() {
        // 2-link hinge pendulum; identify joint 0's damping. Joint 1 carries a
        // fixed, untargeted damping (so the recovery is not trivially decoupled).
        let mut model = sim_core::Model::n_link_pendulum(2, 1.0, 0.1);
        model.jnt_damping = vec![0.0, 0.3];
        model.compute_implicit_params();
        let mut data0 = model.make_data();
        data0.qpos[0] = 1.0;
        data0.qpos[1] = 0.5; // released from an angle; gravity drives the decaying swing.

        let problem = DampingSystemId::for_inverse_design(model, data0, 40, vec![0], &[0.8]);

        // At the true value the residual — hence the gradient — vanishes.
        let (loss_star, grad_star) = problem.evaluate(&[0.8]);
        assert!(
            loss_star < 1e-20 && grad_star[0].abs() < 1e-10,
            "zero residual at d* ⇒ zero grad: loss={loss_star:.3e} grad={:.3e}",
            grad_star[0]
        );

        // Off-target: the analytic gradient the optimizer consumes matches a
        // central FD of the problem's own loss — validates sign AND scale
        // independent of whether Adam happens to converge.
        let d = 0.5;
        let (_, grad) = problem.evaluate(&[d]);
        let h = 1e-6;
        let fd = (problem.evaluate(&[d + h]).0 - problem.evaluate(&[d - h]).0) / (2.0 * h);
        let g_rel = (grad[0] - fd).abs() / fd.abs().max(1e-12);
        assert!(
            g_rel < 1e-4 && grad[0] < 0.0,
            "gradient {:.6e} vs FD {fd:.6e} (rel {g_rel:.3e}); expected match and grad<0 below d*",
            grad[0]
        );

        // Damping is a positive scale parameter ⇒ log-space relative steps.
        let normalized = Normalized::with_residual_scale(&problem, 1.0, true);
        let cfg = normalized.recommended_config();
        let result = normalized.optimize(&[0.2], &cfg);

        let recovered = result.params[0];
        let rel = (recovered - 0.8).abs() / 0.8;
        // Observed ~1e-8 (analytic gradient + Adam); 1e-5 is a robust guard.
        assert!(
            rel < 1e-5,
            "recovered damping {recovered:.6} (rel err {rel:.3e}) after {} iters",
            result.iters
        );
    }

    /// Build the canonical 2-link, 2-damping system-ID fixture used by the
    /// multi-parameter tests (joints 0 and 1, true damping `[0.5, 0.8]`).
    fn two_param_fixture() -> DampingSystemId {
        let mut model = sim_core::Model::n_link_pendulum(2, 1.0, 0.1);
        model.jnt_damping = vec![0.0, 0.0];
        model.compute_implicit_params();
        let mut data0 = model.make_data();
        data0.qpos[0] = 1.0;
        data0.qpos[1] = 0.5;
        DampingSystemId::for_inverse_design(model, data0, 40, vec![0, 1], &[0.5, 0.8])
    }

    /// Multi-parameter system-ID: recover BOTH joint dampings at once, with each
    /// gradient component FD-validated (sign + scale) independent of convergence.
    #[test]
    fn recovers_two_joint_dampings() {
        let problem = two_param_fixture();

        // Off-target: each analytic gradient component matches a central FD of the loss.
        let p = [0.3, 0.3];
        let (_, grad) = problem.evaluate(&p);
        let h = 1e-6;
        for i in 0..2 {
            let (mut pp, mut pm) = (p, p);
            pp[i] += h;
            pm[i] -= h;
            let fd = (problem.evaluate(&pp).0 - problem.evaluate(&pm).0) / (2.0 * h);
            let rel = (grad[i] - fd).abs() / fd.abs().max(1e-12);
            assert!(
                rel < 1e-4,
                "grad[{i}] {} vs FD {fd} (rel {rel:.3e})",
                grad[i]
            );
        }

        // Recover both from an off-target start via log-space Adam.
        let normalized = Normalized::with_residual_scale(&problem, 1.0, true);
        let cfg = normalized.recommended_config();
        let result = normalized.optimize(&[0.2, 0.2], &cfg);
        let r0 = (result.params[0] - 0.5).abs() / 0.5;
        let r1 = (result.params[1] - 0.8).abs() / 0.8;
        assert!(
            r0 < 1e-3 && r1 < 1e-3,
            "recovered [{:.6}, {:.6}] rel [{r0:.2e}, {r1:.2e}] in {} iters",
            result.params[0],
            result.params[1],
            result.iters
        );
    }

    /// Identifiability readout: the terminal-state observation Jacobian is
    /// full-rank (both dampings observable) and its conditioning is surfaced as
    /// data — the Gauss-Newton `JᵀJ` verdict on whether the params trade off.
    #[test]
    fn damping_identifiability_readout() {
        let problem = two_param_fixture();
        let j = problem.observation_jacobian(&[0.5, 0.8]);
        let id = identifiability(&j, 1e-9);
        println!(
            "IDENTIFIABILITY sv={:?} cond={:.4e} rank={}",
            id.singular_values, id.condition_number, id.rank
        );
        assert_eq!(
            id.rank, 2,
            "both dampings should be observable from the terminal state"
        );
        // Observed cond ≈ 12 (cond(JᵀJ) ≈ 150) — well-conditioned, no trade-off;
        // terminal-state matching suffices here (trajectory-matching would only be
        // needed if this were ill-conditioned). cond ≥ 1 always (σ_max ≥ σ_min), so
        // the lower bound catches an inverted σ_max/σ_min; 100 is a robust upper guard.
        assert!(
            (1.0..100.0).contains(&id.condition_number),
            "terminal-state observation should be well-conditioned, cond={:.3e}",
            id.condition_number
        );
    }

    /// The identifiability readout must also *detect* badness: a rank-deficient
    /// observation Jacobian (parallel columns ⇒ a parameter direction unobserved)
    /// reports reduced rank and a blown-up condition number.
    #[test]
    fn identifiability_detects_rank_deficiency() {
        // Two identical columns ⇒ rank 1, second singular value ≈ 0.
        let degenerate = sim_core::DMatrix::from_fn(4, 2, |i, _| (i as f64) + 1.0);
        let id = identifiability(&degenerate, 1e-9);
        assert_eq!(
            id.rank, 1,
            "parallel columns ⇒ numerical rank 1, got {}",
            id.rank
        );
        assert!(
            id.condition_number > 1e6,
            "a near-singular direction should blow up cond, got {:.3e}",
            id.condition_number
        );
    }
}
