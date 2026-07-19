//! **Route-vs-body co-design** — the *geometry* axis of the co-design optimizer.
//!
//! Where [`SoftMaterialTarget`](crate::SoftMaterialTarget) tunes a soft *material*
//! and [`ControlScheduleTarget`](crate::ControlScheduleTarget) tunes a *control
//! schedule*, [`RouteTarget`] tunes the *shape of a route*: the interior control
//! points of a differentiable centerline ([`cf_routing::Path`]) are the design
//! variables, optimized so a cable path clears a body while staying short.
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
//! non-analytic channel, and well-conditioned here because the objective is smooth
//! (it reads `φ_body` directly, with no CSG-`max` crease). Analytic centerline
//! Jacobians (`length_grad`) stay unbuilt until a many-DOF consumer needs them.
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
        let mut cps = Vec::with_capacity(self.n_interior + 2);
        cps.push(self.start);
        for i in 0..self.n_interior {
            cps.push(Point3::new(
                params[3 * i],
                params[3 * i + 1],
                params[3 * i + 2],
            ));
        }
        cps.push(self.end);
        cps
    }

    /// The objective at `params`: `path_length + penalty_weight · Σ clearance
    /// violation²`. Returns `+∞` if the control points are degenerate (a `Path`
    /// needs ≥2 finite points) so the optimizer treats it as strictly worse.
    ///
    /// # Panics
    /// Panics if `params.len() != 3 · n_interior`.
    #[must_use]
    pub fn objective(&self, params: &[f64]) -> f64 {
        let Some(path) = Path::new(self.control_points(params)) else {
            return f64::INFINITY;
        };
        let pts: Vec<Point3<f64>> = (0..self.n_samples)
            .map(|i| path.sample(i as f64 / (self.n_samples as f64 - 1.0)))
            .collect();
        let length: f64 = pts.windows(2).map(|w| (w[1] - w[0]).norm()).sum();
        let penalty: f64 = pts
            .iter()
            .map(|p| {
                let viol = (self.req_clearance - self.body.evaluate(p)).max(0.0);
                viol * viol
            })
            .sum();
        length + self.penalty_weight * penalty
    }

    /// The smallest centerline clearance `min_t φ_body(sample(t))` along the route at
    /// `params` (negative inside the body). A converged route should have this at or
    /// just above [`req_clearance`](Self::req_clearance).
    ///
    /// # Panics
    /// Panics if `params.len() != 3 · n_interior`.
    #[must_use]
    pub fn min_clearance(&self, params: &[f64]) -> f64 {
        let Some(path) = Path::new(self.control_points(params)) else {
            return f64::NEG_INFINITY;
        };
        (0..self.n_samples)
            .map(|i| {
                self.body
                    .evaluate(&path.sample(i as f64 / (self.n_samples as f64 - 1.0)))
            })
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
}
