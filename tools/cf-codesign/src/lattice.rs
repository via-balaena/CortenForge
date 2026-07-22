//! **Lattice-as-design-var co-design** — the *structural* axis of the optimizer.
//!
//! Where [`RouteTarget`](crate::RouteTarget) tunes the *shape* of a route and
//! [`SoftMaterialTarget`](crate::SoftMaterialTarget) tunes a *material*,
//! [`LatticeTarget`] tunes the **cross-section areas of the struts of a truss**:
//! given a fixed ground structure (nodes, connectivity, supports, load), the
//! per-strut areas `A₀ … A_{n−1}` are the design variables, optimized so the
//! structure is stiff (low compliance) without spending more material than it
//! must. This is mission connective-tissue #5 — the rung with *hundreds* of
//! design variables where "the strut is removed" (`Aₑ → 0`) is a legitimate,
//! common answer, so no per-strut outcome can be hand-checked.
//!
//! # The objective — mass-penalized compliance
//!
//! ```text
//! J(A) = C(A)  +  w · Σₑ Aₑ·Lₑ
//! ```
//!
//! `C(A) = fᵀu` is the truss compliance (the inverse-stiffness the load sees; small
//! `C` = stiff), and `Σ Aₑ·Lₑ` is the structural volume (a mass proxy at unit
//! density). The exchange rate `w` is a **calibrated knob**, not a free one: it is
//! the compliance you are willing to give up to save one unit of volume — raise it
//! and the optimizer trims more aggressively, driving marginal struts to zero;
//! lower it and it keeps material for stiffness. Both terms are non-negative, so the
//! two pull against each other and the minimum sits at the lightest structure that
//! still carries the load stiffly enough.
//!
//! Because the objective enters both a reward-free penalty *and* the compliance it
//! trades against, it is **bounded below by zero** (`C ≥ 0`, volume `≥ 0`, `w > 0`).
//! Unlike the signed-reward [`ConduitTarget`](crate::ConduitTarget) — which can run
//! `J → −∞` and so reports no bound — [`LatticeTarget`] declares
//! [`loss_lower_bound`](CoDesignProblem::loss_lower_bound) `= Some(0.0)`, a slack but
//! *sound* bound, so the loss-tolerance stop stays meaningful here.
//!
//! # Log-space areas and the analytic gradient
//!
//! Areas are positive, span orders of magnitude, and want *relative* optimizer
//! steps (a 10 % change means the same thing at a thick strut and a thin one), so
//! the parameter vector is `p = [ln A₀ … ln A_{n−1}]` — the internal
//! log-reparametrization of [`ConduitTarget`](crate::ConduitTarget) /
//! [`JointTarget`](crate::JointTarget),
//! which also keeps every `Aₑ = exp(pₑ) > 0` structurally.
//!
//! Unlike the geometry axis, the gradient here is **analytic**, not finite
//! differences: [`sim_truss`] returns the exact self-adjoint compliance sensitivity
//! `∂C/∂Aₑ` from the *same single solve* as `C` itself, so
//!
//! ```text
//! ∂J/∂pₑ = Aₑ · (∂C/∂Aₑ + w·Lₑ) .
//! ```
//!
//! One truss solve per evaluation yields the whole gradient vector — where central
//! FD would cost `n + 1` solves — which is why [`sim_truss`] ships sensitivities.
//!
//! # Collapse, and why the stop is not yet trustworthy at scale
//!
//! A strut carrying no load has optimal area zero, reached at `pₑ → −∞`; its
//! log-space gradient `∂J/∂pₑ = Aₑ·w·Lₑ` **vanishes as `Aₑ → 0`** even though the
//! physical gradient `w·Lₑ` is a nonzero constant. So a shrinking strut is
//! indistinguishable, by gradient norm, from a converged one —
//! [`ConduitTarget`](crate::ConduitTarget)'s
//! radius-collapse ambiguity, now replicated across *every* redundant strut at once
//! and impossible to hand-check. This target therefore leaves the metric-aware
//! stopping criterion to a later rung; the gates here *measure and pin* the
//! conditioning (see `tests/lattice_inverse_design.rs`) rather than trusting the
//! stop flag, exactly as the geometry gates assert physical outcomes.
//!
//! A design so under-strutted that its stiffness matrix is a **mechanism** is
//! reported as [`InfeasibleDesign`] by [`try_evaluate`](CoDesignProblem::try_evaluate),
//! the feasibility boundary the optimizer must respect. A caller that expects to
//! reach it should drive the run with [`OptConfig::reject_infeasible`], which routes
//! through `try_evaluate` and backtracks; the plain path (what
//! [`recommended_config`](LatticeTarget::recommended_config) selects) instead
//! *panics* on an infeasible point. That is safe for the scenes shipped here —
//! the compliance term blows up as a load-bearing strut thins, so descent steers
//! well clear of a mechanism, and the shipped horizons keep even a collapsing
//! strut's area far above floating-point underflow — but a much longer run or a
//! genuinely singular ground structure should opt into the feasibility-aware mode.

use crate::{CoDesignProblem, InfeasibleDesign, OptConfig};
use sim_truss::{Truss, TrussError};

/// Co-design the per-strut cross-section areas of a fixed truss ground structure
/// to minimize mass-penalized compliance `J(A) = C(A) + w·Σ Aₑ·Lₑ`.
///
/// Generic over the truss dimension `D` (2-D or 3-D — the same solver and the same
/// co-design loop). The design vector is `p = [ln A₀ … ln A_{n−1}]`; build a
/// uniform start with [`x0`](Self::x0) and read areas back with
/// [`to_physical`](Self::to_physical). See the module docs for the objective, the
/// analytic gradient, and the calibration of `w`.
#[derive(Debug, Clone)]
pub struct LatticeTarget<const D: usize> {
    truss: Truss<D>,
    /// The mass/compliance exchange rate `w` (compliance per unit volume).
    penalty_weight: f64,
    /// Cached `Lₑ` (fixed with the ground structure), reused every evaluation.
    lengths: Vec<f64>,
}

impl<const D: usize> LatticeTarget<D> {
    /// Build a lattice problem over a fixed `truss` with mass/compliance exchange
    /// rate `penalty_weight` (`w`).
    ///
    /// # Panics
    /// Panics if `penalty_weight` is not finite and strictly positive (a
    /// non-positive weight removes the material cost, so the areas would grow
    /// without bound), or if the truss has no struts.
    #[must_use]
    pub fn new(truss: Truss<D>, penalty_weight: f64) -> Self {
        assert!(
            penalty_weight.is_finite() && penalty_weight > 0.0,
            "penalty_weight must be positive and finite, got {penalty_weight}"
        );
        assert!(
            truss.n_struts() >= 1,
            "the truss must have at least one strut"
        );
        let lengths = truss.lengths();
        Self {
            truss,
            penalty_weight,
            lengths,
        }
    }

    /// The mass/compliance exchange rate `w`.
    #[must_use]
    pub const fn penalty_weight(&self) -> f64 {
        self.penalty_weight
    }

    /// The fixed ground structure being optimized.
    #[must_use]
    pub const fn truss(&self) -> &Truss<D> {
        &self.truss
    }

    /// The uniform design vector `[ln a₀ … ln a₀]` that starts every strut at area
    /// `a0`.
    ///
    /// # Panics
    /// Panics if `a0` is not strictly positive and finite (log-space has no
    /// representation for a non-positive area).
    #[must_use]
    pub fn x0(&self, a0: f64) -> Vec<f64> {
        assert!(
            a0.is_finite() && a0 > 0.0,
            "initial area must be positive and finite, got {a0}"
        );
        vec![a0.ln(); self.truss.n_struts()]
    }

    /// The physical areas `Aₑ = exp(pₑ)` encoded in `p` — strictly positive for any
    /// finite parameter vector, which is the point of the log-space parametrization.
    ///
    /// # Panics
    /// Panics if `p.len() != n_struts`.
    #[must_use]
    pub fn to_physical(&self, p: &[f64]) -> Vec<f64> {
        self.check_len(p);
        p.iter().map(|&pe| pe.exp()).collect()
    }

    /// The objective `J = C(A) + w·Σ Aₑ·Lₑ` at `p`, or [`InfeasibleDesign`] if the
    /// areas make the truss a mechanism.
    ///
    /// # Errors
    /// Returns [`InfeasibleDesign`] if the truss forward model fails at these areas
    /// (a singular stiffness / mechanism, or an area that has overflowed out of
    /// log-space).
    ///
    /// # Panics
    /// Panics if `p.len() != n_struts`.
    pub fn objective(&self, p: &[f64]) -> Result<f64, InfeasibleDesign> {
        self.check_len(p);
        let areas = self.to_physical(p);
        let compliance = self.truss.compliance(&areas).map_err(infeasible)?;
        Ok(compliance + self.penalty_weight * self.volume(&areas))
    }

    /// Learning rate and iteration budget suited to this convex, log-space
    /// objective. The gradient is analytic and relative (log-space), so a standard
    /// Adam `eps` keeps its scale-invariance — no [`Normalized`](crate::Normalized)
    /// wrapper is needed.
    ///
    /// # The gradient norm is area-weighted, so read a stop with care
    /// `grad_tol` is not scale-free in log-space: `dJ/d(ln Aₑ) = Aₑ · dJ/dAₑ`, so a
    /// strut collapsing toward zero has a vanishing gradient component *whatever its
    /// optimality*. A `GradTol` stop can therefore fire while redundant struts are
    /// still shrinking — check the recovered design, not the flag. This is the
    /// conditioning a later rung's metric-aware criterion will address.
    #[must_use]
    pub fn recommended_config(&self) -> OptConfig {
        OptConfig {
            lr: 0.1,
            max_iters: 4000,
            grad_tol: 1.0e-9,
            ..OptConfig::default()
        }
    }

    /// The structural volume `Σ Aₑ·Lₑ` (a mass proxy at unit density) at the given
    /// physical areas.
    fn volume(&self, areas: &[f64]) -> f64 {
        areas
            .iter()
            .zip(&self.lengths)
            .map(|(&ae, &le)| ae * le)
            .sum()
    }

    fn check_len(&self, p: &[f64]) {
        assert_eq!(
            p.len(),
            self.truss.n_struts(),
            "lattice params length {} != n_struts {}",
            p.len(),
            self.truss.n_struts(),
        );
    }
}

/// Map any truss forward-model failure at a design point to an [`InfeasibleDesign`]:
/// a singular stiffness (mechanism) or an area driven out of range is an infeasible
/// design, not a bug.
fn infeasible(e: TrussError) -> InfeasibleDesign {
    InfeasibleDesign::new(format!("truss forward model failed: {e}"))
}

impl<const D: usize> CoDesignProblem for LatticeTarget<D> {
    fn n_params(&self) -> usize {
        self.truss.n_struts()
    }

    /// `(loss, gradient)` at `p = [ln A₀ … ln A_{n−1}]`. Panics on a mechanism —
    /// use [`try_evaluate`](Self::try_evaluate) to handle that as a value (the
    /// feasibility-aware optimizer does).
    ///
    /// # Panics
    /// Panics if `p.len() != n_struts`, or if the truss is a mechanism at these
    /// areas.
    fn evaluate(&self, p: &[f64]) -> (f64, Vec<f64>) {
        // panic: re-panics the fallible `try_evaluate`'s `InfeasibleDesign` for
        // callers that want it loud; the feasibility-aware optimizer calls
        // `try_evaluate` and handles the mechanism as a value instead.
        self.try_evaluate(p).unwrap_or_else(|e| panic!("{e}"))
    }

    /// `(loss, gradient)` at `p = [ln A₀ … ln A_{n−1}]`, or [`InfeasibleDesign`] if
    /// the truss is a mechanism at these areas.
    ///
    /// One truss solve yields both the compliance `C` and the exact sensitivity
    /// `∂C/∂Aₑ`; the loss is `C + w·Σ Aₑ·Lₑ` and each gradient component carries the
    /// `dAₑ/d(ln Aₑ) = Aₑ` log-chain factor: `∂J/∂pₑ = Aₑ·(∂C/∂Aₑ + w·Lₑ)`.
    ///
    /// # Errors
    /// Returns [`InfeasibleDesign`] if the truss is a mechanism at these areas (or an
    /// area has overflowed out of log-space).
    ///
    /// # Panics
    /// Panics if `p.len() != n_struts`.
    fn try_evaluate(&self, p: &[f64]) -> Result<(f64, Vec<f64>), InfeasibleDesign> {
        self.check_len(p);
        let areas = self.to_physical(p);
        let (compliance, dc_da) = self
            .truss
            .compliance_sensitivity(&areas)
            .map_err(infeasible)?;
        let loss = compliance + self.penalty_weight * self.volume(&areas);
        let grad = (0..areas.len())
            .map(|e| areas[e] * (dc_da[e] + self.penalty_weight * self.lengths[e]))
            .collect();
        Ok((loss, grad))
    }

    // No `lower_bounds`: areas are reparametrized as `ln Aₑ` (positive by
    // construction), so nothing needs clamping.

    /// `Some(0.0)` — a sound lower bound: `J = C + w·Σ Aₑ·Lₑ` is a sum of
    /// non-negative terms (`C ≥ 0`, volume `≥ 0`, `w > 0`), so it can never go
    /// below zero. The bound is slack (the true optimum is strictly positive), which
    /// only costs completeness, never soundness — the loss-tolerance stop stays
    /// meaningful here, unlike for the signed-reward
    /// [`ConduitTarget`](crate::ConduitTarget).
    fn loss_lower_bound(&self) -> Option<f64> {
        Some(0.0)
    }
}
