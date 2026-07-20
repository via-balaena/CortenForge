//! **Route-vs-body co-design** — the *geometry* axis of the co-design optimizer.
//!
//! Where [`SoftMaterialTarget`](crate::SoftMaterialTarget) tunes a soft *material*
//! and [`ControlScheduleTarget`](crate::ControlScheduleTarget) tunes a *control
//! schedule*, [`RouteTarget`] tunes the *shape of a route*: the interior control
//! points of a differentiable centerline ([`cf_routing::Path`]) are the design
//! variables, optimized so a cable path clears a body while staying short.
//!
//! Two targets live here, in rungs. [`RouteTarget`] optimizes the route alone at a
//! *fixed* tube radius. [`ConduitTarget`] promotes the radius to a second design
//! variable — "the fattest conduit that fits" — so route and radius trade off against
//! each other; see its docs for the objective and for how to calibrate the radius
//! reward. Both read the same body field and share the sampling helpers below.
//!
//! The objective is `path_length + w · Σ max(0, req − φ_body(sample))²`: the length
//! term pulls the route straight; the clearance penalty (over the body's signed
//! distance field `φ_body`) pushes the centerline `req = tube_radius + margin` clear
//! of the body so the whole conduit — not just the centerline — misses it. The two
//! terms trade off at the shortest route that still clears.
//!
//! **Gradient.** The route field is differentiable with respect to a control point
//! (proven in the routing kernel: a control-point move perturbs `sample(t)` by the
//! Catmull–Rom basis weight, a smooth plateau). With only a handful of control-point
//! degrees of freedom, that gradient is taken by **finite differences of the
//! objective** — the same choice [`FrictionSpec`](crate::FrictionSpec) makes for its
//! non-analytic channel, and well-conditioned here because the objective reads the
//! body's own smooth field `φ_body` directly — no CSG-`max` crease like the R0 bored
//! field. (The clearance penalty's `max(0, ·)` is a ReLU with a *continuous* gradient,
//! so central FD stays well-conditioned everywhere except exactly at the clearance
//! threshold `φ_body = req`, a measure-zero set the sampled route generally misses.)
//! Analytic centerline Jacobians (`length_grad`) stay unbuilt until a many-DOF
//! consumer needs them.
//!
//! **Symmetry note.** A straight route through a body-symmetric scene is an *even*
//! objective in the detour direction (`+`/`−` detours are identical), so its
//! gradient vanishes there — a stationary ridge, not a minimum. Initialize the
//! optimizer *off* that ridge (a small detour); [`RouteTarget::straight_line`] gives
//! the on-ridge line so a caller can perturb it deliberately.

use cf_design::Solid;
use cf_routing::Path;
use nalgebra::Point3;

use crate::CoDesignProblem;

/// The full control-point list `[start, interior…, end]` for the flat `[x,y,z, …]`
/// interior coordinates in `interior`.
///
/// Shared by [`RouteTarget`] and [`ConduitTarget`], whose design vectors differ only
/// in what follows the coordinate block.
fn assemble_control_points(
    start: Point3<f64>,
    end: Point3<f64>,
    interior: &[f64],
) -> Vec<Point3<f64>> {
    let n = interior.len() / 3;
    let mut cps = Vec::with_capacity(n + 2);
    cps.push(start);
    for i in 0..n {
        cps.push(Point3::new(
            interior[3 * i],
            interior[3 * i + 1],
            interior[3 * i + 2],
        ));
    }
    cps.push(end);
    cps
}

/// `n_samples` evenly-spaced points along the centerline through `cps`, or `None` if
/// the control points are degenerate (a [`Path`] needs ≥2 finite points) so callers
/// can report the design as strictly worse than any valid one.
fn sample_route(cps: Vec<Point3<f64>>, n_samples: usize) -> Option<Vec<Point3<f64>>> {
    let path = Path::new(cps)?;
    Some(
        (0..n_samples)
            .map(|i| path.sample(i as f64 / (n_samples as f64 - 1.0)))
            .collect(),
    )
}

/// Total polyline length through `pts`.
fn polyline_length(pts: &[Point3<f64>]) -> f64 {
    pts.windows(2).map(|w| (w[1] - w[0]).norm()).sum()
}

/// `Σ max(0, req − φ_body(p))²` over the sampled route — the squared clearance
/// shortfall against a required centerline clearance `req`.
fn clearance_penalty(body: &Solid, pts: &[Point3<f64>], req: f64) -> f64 {
    pts.iter()
        .map(|p| {
            let viol = (req - body.evaluate(p)).max(0.0);
            viol * viol
        })
        .sum()
}

/// A co-design problem over a route's shape: optimize the interior control points of
/// a cable centerline so it clears `body` while staying short. The fixed endpoints
/// bracket the interior points, which are the design variables (`3 ·
/// n_interior` parameters, `[x,y,z]` per point, in order).
#[derive(Debug, Clone)]
pub struct RouteTarget {
    body: Solid,
    start: Point3<f64>,
    end: Point3<f64>,
    n_interior: usize,
    /// Required centerline clearance from the body surface (`tube_radius + margin`),
    /// so the whole conduit — not just the centerline — clears.
    req_clearance: f64,
    penalty_weight: f64,
    n_samples: usize,
    fd_eps: f64,
}

impl RouteTarget {
    /// Build a route problem: optimize `n_interior` interior control points between
    /// fixed `start` and `end`, keeping the centerline `tube_radius + margin` clear
    /// of `body`, with clearance weighted by `penalty_weight` and the route sampled
    /// at `n_samples` points for the length/clearance quadrature.
    ///
    /// # Panics
    /// Panics if `n_interior == 0`, `n_samples < 2`, or any of `tube_radius`,
    /// `margin`, `penalty_weight` is not finite-and-non-negative (`tube_radius` and
    /// `penalty_weight` must be strictly positive) — a malformed scene, not a design.
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        body: Solid,
        start: Point3<f64>,
        end: Point3<f64>,
        n_interior: usize,
        tube_radius: f64,
        margin: f64,
        penalty_weight: f64,
        n_samples: usize,
    ) -> Self {
        assert!(n_interior >= 1, "need at least one interior control point");
        assert!(
            n_samples >= 2,
            "need at least two samples for length/clearance"
        );
        assert!(
            tube_radius.is_finite() && tube_radius > 0.0,
            "tube_radius must be positive and finite, got {tube_radius}"
        );
        assert!(
            margin.is_finite() && margin >= 0.0,
            "margin must be non-negative and finite, got {margin}"
        );
        assert!(
            penalty_weight.is_finite() && penalty_weight > 0.0,
            "penalty_weight must be positive and finite, got {penalty_weight}"
        );
        Self {
            body,
            start,
            end,
            n_interior,
            req_clearance: tube_radius + margin,
            penalty_weight,
            n_samples,
            fd_eps: 1e-6,
        }
    }

    /// The `3 · n_interior` design parameters `[x,y,z, …]` of the straight line from
    /// `start` to `end` (interior points evenly spaced). This is the *on-ridge*
    /// symmetric route — perturb it before optimizing (see the module symmetry note).
    #[must_use]
    pub fn straight_line(&self) -> Vec<f64> {
        let mut p = Vec::with_capacity(3 * self.n_interior);
        for k in 1..=self.n_interior {
            let s = k as f64 / (self.n_interior as f64 + 1.0);
            let c = self.start + (self.end - self.start) * s;
            p.extend_from_slice(&[c.x, c.y, c.z]);
        }
        p
    }

    /// The full control-point list `[start, interior…, end]` for design `params`.
    ///
    /// # Panics
    /// Panics if `params.len() != 3 · n_interior` (a clear error rather than an
    /// opaque out-of-bounds index or a silently-truncated route).
    fn control_points(&self, params: &[f64]) -> Vec<Point3<f64>> {
        assert_eq!(
            params.len(),
            3 * self.n_interior,
            "route params length {} != 3·n_interior {}",
            params.len(),
            3 * self.n_interior,
        );
        assemble_control_points(self.start, self.end, params)
    }

    /// The objective at `params`: `path_length + penalty_weight · Σ clearance
    /// violation²`. Returns `+∞` if the control points are degenerate (a `Path`
    /// needs ≥2 finite points) so the optimizer treats it as strictly worse.
    ///
    /// # Panics
    /// Panics if `params.len() != 3 · n_interior`.
    #[must_use]
    pub fn objective(&self, params: &[f64]) -> f64 {
        let Some(pts) = sample_route(self.control_points(params), self.n_samples) else {
            return f64::INFINITY;
        };
        polyline_length(&pts)
            + self.penalty_weight * clearance_penalty(&self.body, &pts, self.req_clearance)
    }

    /// The smallest centerline clearance `min_t φ_body(sample(t))` along the route at
    /// `params` (negative inside the body). A converged route should have this at or
    /// just above [`req_clearance`](Self::req_clearance).
    ///
    /// # Panics
    /// Panics if `params.len() != 3 · n_interior`.
    #[must_use]
    pub fn min_clearance(&self, params: &[f64]) -> f64 {
        let Some(pts) = sample_route(self.control_points(params), self.n_samples) else {
            return f64::NEG_INFINITY;
        };
        pts.iter()
            .map(|p| self.body.evaluate(p))
            .fold(f64::INFINITY, f64::min)
    }

    /// The required centerline clearance (`tube_radius + margin`).
    #[must_use]
    pub const fn req_clearance(&self) -> f64 {
        self.req_clearance
    }
}

impl CoDesignProblem for RouteTarget {
    fn n_params(&self) -> usize {
        3 * self.n_interior
    }

    /// `(loss, gradient)` at the route `params`, the gradient by central finite
    /// differences of [`objective`](Self::objective) (see the module note on why FD).
    ///
    /// # Panics
    /// Panics if `params.len() != 3 · n_interior` (the optimizer guards this, but a
    /// direct caller must match).
    fn evaluate(&self, params: &[f64]) -> (f64, Vec<f64>) {
        assert_eq!(
            params.len(),
            self.n_params(),
            "route params length {} != 3·n_interior {}",
            params.len(),
            self.n_params(),
        );
        let loss = self.objective(params);
        let mut grad = vec![0.0; params.len()];
        let mut p = params.to_vec();
        for i in 0..params.len() {
            let x = p[i];
            p[i] = x + self.fd_eps;
            let jp = self.objective(&p);
            p[i] = x - self.fd_eps;
            let jm = self.objective(&p);
            p[i] = x;
            grad[i] = (jp - jm) / (2.0 * self.fd_eps);
        }
        (loss, grad)
    }
    // No `lower_bounds`: control-point coordinates are signed positions in space.

    /// `Some(‖end − start‖)` — the straight-line chord between the fixed endpoints.
    ///
    /// The objective is `length + w·penalty`; the penalty is a sum of squares (`≥ 0`)
    /// and the sampled polyline runs from `start` to `end`, so by the triangle
    /// inequality its length is at least the chord. (Verified rather than assumed: the
    /// route interpolates its endpoints, `sample(0) = start` and `sample(1) = end`.)
    ///
    /// The bound is **slack whenever the body blocks the straight route** — the true
    /// optimum then includes an unavoidable detour whose size is the thing being
    /// solved for. That costs only completeness, never soundness: `loss − chord <
    /// loss_tol` still implies the design is within `loss_tol` of optimal, so the stop
    /// cannot fire falsely. It is tight exactly when an unobstructed straight route is
    /// the answer, which is precisely when firing is correct.
    fn loss_lower_bound(&self) -> Option<f64> {
        Some((self.end - self.start).norm())
    }
}

/// A co-design problem over a conduit's *route and radius together*: find the
/// **fattest conduit that fits**, bending the centerline and growing the tube until
/// the two stop paying for each other.
///
/// Where [`RouteTarget`] optimizes a route at a *fixed* tube radius, `ConduitTarget`
/// promotes the radius to a second design variable — the geometry axis's mixed-
/// conditioning rung. The objective is
///
/// ```text
/// J = −w_r·r  +  length  +  w_c · Σ max(0, (r + margin) − φ_body(sample))²
/// ```
///
/// The radius enters **both** the reward and the clearance requirement, which is what
/// makes the two axes genuinely trade off: a fatter tube is worth more, but it demands
/// more centerline clearance, which forces a longer detour. Without the `−w_r·r`
/// reward a free radius would collapse to zero — a thinner tube is strictly cheaper on
/// every other term — so the reward is load-bearing, not decoration.
///
/// # Parameters and conditioning
/// The design vector is `[x,y,z, …, ln r]`: the `3 · n_interior` interior control-point
/// coordinates (signed, linear) followed by the **log** of the radius. This is the
/// mixed-conditioning regime — signed positions alongside a strictly-positive scale —
/// and it is resolved the way [`JointTarget`](crate::JointTarget) resolves it: the
/// target owns the reparametrization *internally* rather than wrapping itself in
/// [`Normalized`](crate::Normalized), because a single `log_space` flag cannot serve
/// both blocks. Log-space makes the radius step *relative* (so one learning rate
/// serves a millimetre radius and a metre-scale route) and keeps `r > 0` structurally,
/// with no bound to clamp. The gradient is central finite differences taken directly
/// in this `p`-space, including the `ln r` component — the same deliberate FD choice
/// [`RouteTarget`] makes, with no hand-applied chain rule to get wrong.
///
/// # Choosing `radius_reward` (`w_r`)
/// `w_r` is an **exchange rate**: how many metres of extra path length one metre of
/// extra radius is worth. Its calibrated meaning comes from the radius stationarity
/// condition — with a single sample binding against the body,
///
/// ```text
/// dJ/dr = −w_r + 2·w_c·(r + margin − φ_min) = 0
///   ⇒   r* = (φ_min − margin) + w_r / (2·w_c)
/// ```
///
/// so **`w_r / (2·w_c)` is the soft-constraint slack**: how far past the true geometric
/// fit the optimum is allowed to push. Tune `w_r` *relative to* `penalty_weight` to buy
/// a tolerance in metres.
///
/// When `m` samples bind at once — they share the reward between them — the slack
/// shrinks by `m`, and the binding samples enter through their *mean* clearance `φ̄`:
///
/// ```text
/// r* = (φ̄_binding − margin) + w_r / (2·m·w_c)
/// ```
///
/// This is genuinely predictive where both quantities are known: the gate's flat-walled
/// corridor scene puts every sample at the same clearance, so `m = n_samples` and
/// `φ̄ = φ_min` exactly, and the optimizer lands within ~1% of this `r*` across a 6×
/// range of corridor widths. On a curved body **both** inputs degrade — `m` is whatever
/// fraction of the route is actually pressed against the surface, and `φ̄` exceeds
/// `φ_min` because the binding samples sit at a spread of clearances — so there the
/// formula sets the scale of `w_r` rather than predicting `r*` outright.
///
/// # Symmetry
/// The inherited stationary-ridge footgun applies to the route block exactly as it does
/// for [`RouteTarget`] — initialize off the ridge.
///
/// # The objective is signed, so the loss-tolerance stop does not apply
/// Unlike every other target here, `J` is not a squared residual: the `−w_r·r` reward
/// takes it negative, and a "small loss" then says nothing about optimality. This
/// target therefore declares no
/// [`loss_lower_bound`](CoDesignProblem::loss_lower_bound), which switches that
/// criterion off structurally — no caller can mis-drive it. See that impl for why no
/// bound can be stated.
#[derive(Debug, Clone)]
pub struct ConduitTarget {
    body: Solid,
    start: Point3<f64>,
    end: Point3<f64>,
    n_interior: usize,
    /// Clearance demanded *beyond* the tube radius; the required centerline clearance
    /// is `r + margin`, with `r` a design variable rather than a constant.
    margin: f64,
    penalty_weight: f64,
    /// The `w_r` exchange rate — metres of path length worth one metre of radius.
    radius_reward: f64,
    n_samples: usize,
    fd_eps: f64,
}

impl ConduitTarget {
    /// Build a conduit problem: co-optimize `n_interior` interior control points and
    /// the tube radius between fixed `start` and `end`, keeping the centerline
    /// `r + margin` clear of `body`.
    ///
    /// See the type docs for how to choose `radius_reward` against `penalty_weight`.
    ///
    /// # Panics
    /// Panics if `n_interior == 0`, `n_samples < 2`, or any of `margin`,
    /// `penalty_weight`, `radius_reward` is not finite-and-non-negative
    /// (`penalty_weight` and `radius_reward` must be strictly positive) — a malformed
    /// scene, not a design. A non-positive `radius_reward` is rejected specifically
    /// because it is the degenerate case the objective is built to avoid: without a
    /// reward the radius collapses to zero.
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        body: Solid,
        start: Point3<f64>,
        end: Point3<f64>,
        n_interior: usize,
        margin: f64,
        penalty_weight: f64,
        radius_reward: f64,
        n_samples: usize,
    ) -> Self {
        assert!(n_interior >= 1, "need at least one interior control point");
        assert!(
            n_samples >= 2,
            "need at least two samples for length/clearance"
        );
        assert!(
            margin.is_finite() && margin >= 0.0,
            "margin must be non-negative and finite, got {margin}"
        );
        assert!(
            penalty_weight.is_finite() && penalty_weight > 0.0,
            "penalty_weight must be positive and finite, got {penalty_weight}"
        );
        assert!(
            radius_reward.is_finite() && radius_reward > 0.0,
            "radius_reward must be positive and finite, got {radius_reward} \
             (a non-positive reward collapses the radius to zero)"
        );
        Self {
            body,
            start,
            end,
            n_interior,
            margin,
            penalty_weight,
            radius_reward,
            n_samples,
            fd_eps: 1e-6,
        }
    }

    /// Learning rate and iteration budget suited to this objective's O(1) geometric
    /// gradients.
    ///
    /// The loss-tolerance stop no longer needs disabling by hand here: this target
    /// reports no [`loss_lower_bound`](Self::loss_lower_bound), so the optimizer
    /// switches that criterion off structurally, whatever `loss_tol` a caller passes.
    /// (This method used to carry a `loss_tol = −∞` sentinel for exactly that purpose.)
    ///
    /// # The gradient norm is radius-weighted, so read it with care
    /// `grad_tol` is not scale-free in log-space: `dJ/d(ln r) = r · dJ/dr`, so the
    /// radius component vanishes as `r → 0` *by construction*, whatever the design's
    /// optimality. In an infeasible scene, where the radius is driven down without
    /// limit, a small enough `r` therefore trips any fixed `grad_tol` and reports a
    /// [`GradTol`](crate::StopReason::GradTol) stop on a conduit that is merely
    /// shrinking to nothing. A longer run or a smaller `penalty_weight` reaches it — so
    /// check that a stopped radius is a *fit*, not a collapse.
    ///
    /// (The radius component alone cannot trip this in practice at these constants,
    /// because the route block dominates the norm by orders of magnitude — see
    /// [`OptConfig::grad_tol`](crate::OptConfig::grad_tol). That is a property of the
    /// mixed-scale vector, not a safety margin to rely on.)
    #[must_use]
    pub fn recommended_config(&self) -> crate::OptConfig {
        crate::OptConfig {
            lr: 0.05,
            max_iters: 800,
            grad_tol: 1.0e-4,
            ..crate::OptConfig::default()
        }
    }

    /// The design vector for the straight line from `start` to `end` at initial radius
    /// `r0`: `[x,y,z, …, ln r0]`. This is the *on-ridge* symmetric route — perturb the
    /// coordinate block before optimizing (see the type's symmetry note).
    ///
    /// # Panics
    /// Panics if `r0` is not strictly positive and finite (the log-space
    /// parametrization has no representation for a non-positive radius).
    #[must_use]
    pub fn x0(&self, r0: f64) -> Vec<f64> {
        assert!(
            r0.is_finite() && r0 > 0.0,
            "initial radius must be positive and finite, got {r0}"
        );
        let mut p = Vec::with_capacity(self.n_params());
        for k in 1..=self.n_interior {
            let s = k as f64 / (self.n_interior as f64 + 1.0);
            let c = self.start + (self.end - self.start) * s;
            p.extend_from_slice(&[c.x, c.y, c.z]);
        }
        p.push(r0.ln());
        p
    }

    /// The physical design behind `p`: the interior control-point coordinates and the
    /// tube radius `r = exp(p_last)`.
    ///
    /// # Panics
    /// Panics if `p.len() != 3 · n_interior + 1`.
    #[must_use]
    pub fn to_physical(&self, p: &[f64]) -> (Vec<f64>, f64) {
        self.check_len(p);
        (p[..3 * self.n_interior].to_vec(), self.radius(p))
    }

    /// The tube radius `r = exp(p_last)` encoded in `p` — strictly positive for any
    /// finite parameter vector, which is the point of the log-space parametrization.
    ///
    /// # Panics
    /// Panics if `p.len() != 3 · n_interior + 1`.
    #[must_use]
    pub fn radius(&self, p: &[f64]) -> f64 {
        self.check_len(p);
        p[3 * self.n_interior].exp()
    }

    /// The required centerline clearance `r + margin` at `p`.
    ///
    /// # Panics
    /// Panics if `p.len() != 3 · n_interior + 1`.
    #[must_use]
    pub fn req_clearance(&self, p: &[f64]) -> f64 {
        self.radius(p) + self.margin
    }

    /// The clearance demanded beyond the tube radius (the constructor's `margin`). A
    /// corridor narrower than this cannot admit *any* conduit, however thin.
    #[must_use]
    pub const fn margin(&self) -> f64 {
        self.margin
    }

    fn check_len(&self, p: &[f64]) {
        assert_eq!(
            p.len(),
            self.n_params(),
            "conduit params length {} != 3·n_interior + 1 = {}",
            p.len(),
            self.n_params(),
        );
    }

    /// The objective at `p`: `−w_r·r + length + w_c · Σ clearance violation²`, with
    /// `r = exp(p_last)`. Returns `+∞` for a degenerate route or a radius that has
    /// overflowed out of log-space, ranking such a design below any valid one.
    ///
    /// **This ranking does not survive differentiation.** A `+∞` on both sides of a
    /// finite-difference stencil gives `∞ − ∞ = NaN`, and on one side `±∞`; either
    /// poisons the Adam step. Worse, [`optimize`](crate::optimize) derives its stopping
    /// norm with `f64::max`, which *returns the non-NaN operand*, so an all-NaN
    /// gradient reads as `‖grad‖∞ = 0` and the run reports a `GradTol` stop on a design it
    /// never actually evaluated — returning the overflowed-but-finite parameters
    /// untouched, since it stops before stepping (NaN parameters arise only on a later
    /// iterate, after a partially-NaN gradient has been stepped in). This is inherited
    /// from [`RouteTarget`] and not specific to the
    /// radius, and it is far out of reach in practice — log-space overflow needs
    /// `p_last > 709`, some 14000 Adam steps at the recommended `lr` from any sane
    /// start — but treat `+∞` as a diagnostic for a malformed scene, not as a
    /// constraint the optimizer can be driven against.
    ///
    /// # Panics
    /// Panics if `p.len() != 3 · n_interior + 1`.
    #[must_use]
    pub fn objective(&self, p: &[f64]) -> f64 {
        self.check_len(p);
        let r = self.radius(p);
        if !r.is_finite() {
            return f64::INFINITY;
        }
        let cps = assemble_control_points(self.start, self.end, &p[..3 * self.n_interior]);
        let Some(pts) = sample_route(cps, self.n_samples) else {
            return f64::INFINITY;
        };
        -self.radius_reward * r
            + polyline_length(&pts)
            + self.penalty_weight * clearance_penalty(&self.body, &pts, r + self.margin)
    }

    /// The smallest centerline clearance `min_t φ_body(sample(t))` along the route at
    /// `p` (negative inside the body). A converged conduit should have this at or just
    /// below [`req_clearance`](Self::req_clearance) — the soft equilibrium sits a
    /// slack `w_r / (2·m·w_c)` *inside* the requirement, `m` being the number of
    /// samples binding against the body (see the type docs; the shortfall is divided
    /// among them, so quoting the single-sample `w_r / (2·w_c)` here would overstate
    /// it by a factor of `m`).
    ///
    /// # Panics
    /// Panics if `p.len() != 3 · n_interior + 1`.
    #[must_use]
    pub fn min_clearance(&self, p: &[f64]) -> f64 {
        self.check_len(p);
        let cps = assemble_control_points(self.start, self.end, &p[..3 * self.n_interior]);
        let Some(pts) = sample_route(cps, self.n_samples) else {
            return f64::NEG_INFINITY;
        };
        pts.iter()
            .map(|q| self.body.evaluate(q))
            .fold(f64::INFINITY, f64::min)
    }
}

impl CoDesignProblem for ConduitTarget {
    fn n_params(&self) -> usize {
        3 * self.n_interior + 1
    }

    /// `(loss, gradient)` at `p = [x,y,z, …, ln r]`, the gradient by central finite
    /// differences of [`objective`](Self::objective) taken directly in `p`-space — so
    /// the last component is `dJ/d(ln r)`, already carrying the `dr/d(ln r) = r`
    /// factor without a hand-applied chain rule.
    ///
    /// # Panics
    /// Panics if `p.len() != 3 · n_interior + 1`.
    fn evaluate(&self, p: &[f64]) -> (f64, Vec<f64>) {
        self.check_len(p);
        let loss = self.objective(p);
        let mut grad = vec![0.0; p.len()];
        let mut q = p.to_vec();
        for i in 0..p.len() {
            let x = q[i];
            q[i] = x + self.fd_eps;
            let jp = self.objective(&q);
            q[i] = x - self.fd_eps;
            let jm = self.objective(&q);
            q[i] = x;
            grad[i] = (jp - jm) / (2.0 * self.fd_eps);
        }
        (loss, grad)
    }
    // No `lower_bounds`: the coordinates are signed positions and the radius is
    // reparametrized as `ln r` (positive by construction), so nothing needs clamping.

    /// `None` — **no lower bound can be stated for this objective**, which disables
    /// the loss-tolerance stop. This is the method that makes the signed-reward
    /// objective safe to optimize, and it replaces the `loss_tol = −∞` sentinel this
    /// target used to need.
    ///
    /// Whether `J` is bounded below at all depends on `w_r` and the scene, so it
    /// cannot be answered at construction. Fatten in place and the clearance penalty
    /// `~w_c·m·r²` overwhelms the linear reward, so `J → +∞`. But *detour and fatten
    /// together* — holding the route roughly `r` clear of the body so the penalty
    /// stays zero — costs length only linearly, giving `J ≈ (c − w_r)·r`, which runs
    /// to `−∞` for any `w_r` above the length cost per unit of clearance. So the
    /// objective is bounded below for a modest reward and unbounded for a generous
    /// one, with the crossover set by geometry.
    ///
    /// Either way a numeric `loss_tol` is meaningless here: `J` goes negative as soon
    /// as the reward outweighs length plus penalty, so the stop fires at that sign
    /// crossing rather than at an optimum — silently returning a plausible radius that
    /// is merely wherever `J` first went negative. That produced a **bit-identical
    /// `r*` for two corridors with genuinely different optima** before it was caught.
    fn loss_lower_bound(&self) -> Option<f64> {
        None
    }
}
