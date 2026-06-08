//! [`Spline`] — a 1-D function of a joint coordinate, the building block of the
//! IR's coupled transform axes and moving path points.
//!
//! An OpenSim `SimmSpline` (or a constant as a single knot), with its
//! `MultiplierFunction` scale folded in. OpenSim's `SimmSpline` is a **natural
//! cubic** spline; we reproduce that (tridiagonal solve, second-derivative = 0 at
//! the ends) because a moment arm is `dL/dθ`, so the spline's *derivative* enters
//! the result directly — linear interpolation would inject a discontinuous, biased
//! slope (~mm/rad on the dominant knee couplings). Within-knot interpolation only;
//! the gait2392 flexion sweep stays inside every spline's knot range, so end
//! extrapolation is not exercised (we clamp flat there).
//!
//! NOTE: this is a faithful re-derivation of OpenSim's spline math, NOT
//! bit-identical to OpenSim's own evaluator — the real-OpenSim cross-check
//! (`cf-osim`'s `opensim_cross_check`) is the independent anchor.
//!
//! (Moved verbatim from `cf-osim::osim` when the source-agnostic IR became the
//! home of the model types; the dependency now points cf-osim → cf-msk-lib.)

/// A natural-cubic spline through `(x, y)` with a folded `MultiplierFunction`
/// `scale`. A constant is a single knot.
#[derive(Debug, Clone)]
pub struct Spline {
    pub x: Vec<f64>,
    pub y: Vec<f64>,
    pub scale: f64,
    /// Precomputed second derivatives at the knots (natural cubic).
    m: Vec<f64>,
}

impl Spline {
    /// Build a natural-cubic spline through `(x, y)` with multiplier `scale`.
    pub fn new(x: Vec<f64>, y: Vec<f64>, scale: f64) -> Self {
        assert_eq!(x.len(), y.len(), "spline x/y length mismatch");
        let m = natural_cubic_second_derivs(&x, &y);
        Spline { x, y, scale, m }
    }

    pub fn constant(v: f64) -> Self {
        Spline {
            x: vec![0.0],
            y: vec![v],
            scale: 1.0,
            m: vec![0.0],
        }
    }

    /// Evaluate at `t` (natural-cubic within range, flat clamp outside), scaled.
    pub fn eval(&self, t: f64) -> f64 {
        let (xs, ys, m) = (&self.x, &self.y, &self.m);
        let v = if xs.len() == 1 || t <= xs[0] {
            ys[0]
        } else if t >= xs[xs.len() - 1] {
            ys[ys.len() - 1]
        } else {
            let i = xs.partition_point(|&xi| xi <= t) - 1;
            let (klo, khi) = (i, i + 1);
            let h = xs[khi] - xs[klo];
            let a = (xs[khi] - t) / h;
            let b = (t - xs[klo]) / h;
            // Numerical-Recipes "splint": cubic through the bracketing knots
            // using the precomputed second derivatives.
            a * ys[klo]
                + b * ys[khi]
                + ((a * a * a - a) * m[klo] + (b * b * b - b) * m[khi]) * (h * h) / 6.0
        };
        v * self.scale
    }
}

/// Second derivatives at the knots for a natural cubic spline (y''=0 at the
/// ends), via the standard tridiagonal solve. Degenerate inputs (<3 knots)
/// return zeros, so `eval` falls back to linear/constant behavior there.
fn natural_cubic_second_derivs(x: &[f64], y: &[f64]) -> Vec<f64> {
    let n = x.len();
    if n < 3 {
        return vec![0.0; n];
    }
    let mut y2 = vec![0.0; n];
    let mut u = vec![0.0; n];
    for i in 1..n - 1 {
        let sig = (x[i] - x[i - 1]) / (x[i + 1] - x[i - 1]);
        let p = sig * y2[i - 1] + 2.0;
        y2[i] = (sig - 1.0) / p;
        let d = (y[i + 1] - y[i]) / (x[i + 1] - x[i]) - (y[i] - y[i - 1]) / (x[i] - x[i - 1]);
        u[i] = (6.0 * d / (x[i + 1] - x[i - 1]) - sig * u[i - 1]) / p;
    }
    for k in (0..n - 1).rev() {
        y2[k] = y2[k] * y2[k + 1] + u[k];
    }
    y2
}

#[cfg(test)]
mod tests {
    use super::Spline;

    #[test]
    fn interpolates_knots_exactly() {
        let s = Spline::new(vec![0.0, 1.0, 2.0, 3.0], vec![0.0, 1.0, 4.0, 9.0], 2.0);
        for (x, y) in [(0.0, 0.0), (1.0, 1.0), (2.0, 4.0), (3.0, 9.0)] {
            assert!(
                (s.eval(x) - y * 2.0).abs() < 1e-12,
                "knot ({x},{y}) not hit"
            );
        }
    }

    #[test]
    fn reduces_to_linear_for_collinear_data() {
        // Natural cubic through collinear points is the line itself (y''=0).
        let s = Spline::new(vec![-2.0, 0.0, 1.0, 4.0], vec![-4.0, 0.0, 2.0, 8.0], 1.0);
        for t in [-1.3, 0.25, 2.7, 3.9] {
            assert!((s.eval(t) - 2.0 * t).abs() < 1e-9, "not linear at {t}");
        }
    }

    #[test]
    fn second_derivative_is_continuous() {
        // C2: a finite-difference second derivative is smooth across an interior
        // knot (no jump like linear interpolation would show).
        let s = Spline::new(vec![0.0, 1.0, 2.0, 3.0], vec![0.0, 1.0, 0.0, 1.0], 1.0);
        let d2 = |t: f64| {
            let h = 1e-3;
            (s.eval(t + h) - 2.0 * s.eval(t) + s.eval(t - h)) / (h * h)
        };
        assert!(
            (d2(1.0 - 1e-2) - d2(1.0 + 1e-2)).abs() < 1e-1,
            "y'' jumps across knot"
        );
    }

    #[test]
    fn clamps_flat_outside_range() {
        let s = Spline::new(vec![0.0, 1.0, 2.0], vec![3.0, 5.0, 7.0], 1.0);
        assert!((s.eval(-10.0) - 3.0).abs() < 1e-12);
        assert!((s.eval(10.0) - 7.0).abs() < 1e-12);
    }

    #[test]
    fn two_knot_and_constant_degenerate() {
        // <3 knots → falls back to linear; equal endpoints → constant.
        let s = Spline::new(vec![-2.0, 0.17], vec![0.0014, 0.0014], 0.98);
        assert!((s.eval(-1.0) - 0.0014 * 0.98).abs() < 1e-12);
        let c = Spline::constant(0.42);
        assert!((c.eval(-5.0) - 0.42).abs() < 1e-12);
    }
}
