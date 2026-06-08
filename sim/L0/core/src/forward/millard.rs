//! Faithful port of OpenSim's `Millard2012EquilibriumMuscle` force-length /
//! force-velocity curves (the default-parameter curves used by gait2392).
//!
//! This is the dynamics-fidelity counterpart to [`super::hill`]: the native Hill
//! model (Gaussian FL / hyperbolic FV / exp FP) agrees with Millard only near the
//! plateau and diverges 15–60% on the ascending/descending limbs (measured by the
//! G2 spike). For a twin that borrows OpenSim's literature validation, we
//! reproduce Millard's actual curves.
//!
//! Each curve is a C2 quintic-Bézier spline — a port of OpenSim's
//! `SmoothSegmentedFunction` + `SegmentedQuinticBezierToolkit` (`OpenSim/Common`)
//! built by `SmoothSegmentedFunctionFactory` from the documented default
//! parameters. The curves are analytic (exact derivatives, no embedded data
//! tables), which matters for the differentiable co-design loop downstream.
//!
//! Validated to ~1e-6 against densely sampled real OpenSim 4.6 curve values
//! (`tools/cf-osim` cross-checks, vendored `millard_curves_opensim.json`).
//!
//! References:
//! - Millard, Uchida, Seth, Delp (2013) "Flexing computational muscle", J Biomech Eng.
//! - OpenSim `SmoothSegmentedFunctionFactory`, `SegmentedQuinticBezierToolkit`.

/// OpenSim's curviness rescaling: a user curviness in `[0,1]` maps to the
/// internal corner-fraction `c = 0.1 + 0.8·curviness`.
fn scale_curviness(curviness: f64) -> f64 {
    0.1 + 0.8 * curviness
}

/// Evaluate a quintic Bézier (Bernstein form) over the 6 control values `p` at
/// parameter `u ∈ [0,1]`. `order = 0` → value, `order = 1` → d/du. This is the
/// clean Bernstein form, mathematically identical to OpenSim's expanded
/// `calcQuinticBezierCurveDerivU`.
fn bezier_u(u: f64, p: &[f64; 6], order: u32) -> f64 {
    let v = 1.0 - u;
    match order {
        0 => {
            p[0] * v * v * v * v * v
                + 5.0 * p[1] * u * v * v * v * v
                + 10.0 * p[2] * u * u * v * v * v
                + 10.0 * p[3] * u * u * u * v * v
                + 5.0 * p[4] * u * u * u * u * v
                + p[5] * u * u * u * u * u
        }
        // d/du of a degree-5 Bézier = 5 · (degree-4 Bézier of the differences).
        _ => {
            5.0 * ((p[1] - p[0]) * v * v * v * v
                + 4.0 * (p[2] - p[1]) * u * v * v * v
                + 6.0 * (p[3] - p[2]) * u * u * v * v
                + 4.0 * (p[4] - p[3]) * u * u * u * v
                + (p[5] - p[4]) * u * u * u * u)
        }
    }
}

/// Invert `x = bezier_x(u)` for `u ∈ [0,1]` on one segment, via Newton from a
/// linear initial guess (the segment's x is monotone, so Newton converges in a
/// handful of iterations). Port of `SegmentedQuinticBezierToolkit::calcU`.
fn calc_u(ax: f64, px: &[f64; 6]) -> f64 {
    const UTOL: f64 = 1e-12;
    const MAXITER: u32 = 20;
    let span = px[5] - px[0];
    let mut u = if span.abs() > 1e-15 {
        ((ax - px[0]) / span).clamp(0.0, 1.0)
    } else {
        0.5
    };
    for _ in 0..MAXITER {
        let f = bezier_u(u, px, 0) - ax;
        if f.abs() <= UTOL {
            break;
        }
        let df = bezier_u(u, px, 1);
        if df.abs() < 1e-15 {
            break;
        }
        u = (u - f / df).clamp(0.0, 1.0);
    }
    u
}

/// Build the 6 control points of one corner-shaped quintic Bézier segment that
/// joins `(x0,y0)` with tangent `dydx0` to `(x1,y1)` with tangent `dydx1`,
/// rounded by corner-fraction `c ∈ [0,1]`. Port of
/// `SegmentedQuinticBezierToolkit::calcQuinticBezierCornerControlPoints`.
#[allow(clippy::similar_names, clippy::many_single_char_names)]
fn corner(
    x0: f64,
    y0: f64,
    dydx0: f64,
    x1: f64,
    y1: f64,
    dydx1: f64,
    c: f64,
) -> ([f64; 6], [f64; 6]) {
    let root_eps = f64::EPSILON.sqrt();
    // Intersection of the two tangent lines.
    let xc = if (dydx0 - dydx1).abs() > root_eps {
        (y1 - y0 - x1 * dydx1 + x0 * dydx0) / (dydx0 - dydx1)
    } else {
        f64::midpoint(x1, x0)
    };
    let yc = (xc - x1) * dydx1 + y1;
    debug_assert!(
        {
            let a = (xc - x0).powi(2) + (yc - y0).powi(2);
            let b = (xc - x1).powi(2) + (yc - y1).powi(2);
            let cc = (x1 - x0).powi(2) + (y1 - y0).powi(2);
            cc > a && cc > b
        },
        "corner control points require a C-shaped (not S-shaped) corner"
    );
    let x0_mid = x0 + c * (xc - x0);
    let y0_mid = y0 + c * (yc - y0);
    let x1_mid = x1 + c * (xc - x1);
    let y1_mid = y1 + c * (yc - y1);
    (
        [x0, x0_mid, x0_mid, x1_mid, x1_mid, x1],
        [y0, y0_mid, y0_mid, y1_mid, y1_mid, y1],
    )
}

/// A C2 quintic-Bézier spline curve (port of OpenSim `SmoothSegmentedFunction`).
///
/// Evaluated by locating the segment containing `x`, Newton-solving the Bézier
/// parameter `u`, then evaluating the y-Bézier. Outside the fitted domain
/// `[x_lo, x_hi]` the curve extends linearly with the stored end slopes (exactly
/// as OpenSim does).
#[derive(Clone, Debug)]
pub struct SmoothSegmentedFunction {
    seg_x: Vec<[f64; 6]>,
    seg_y: Vec<[f64; 6]>,
    x_lo: f64,
    x_hi: f64,
    y_lo: f64,
    y_hi: f64,
    dydx_lo: f64,
    dydx_hi: f64,
}

impl SmoothSegmentedFunction {
    /// Evaluate the curve at `x`.
    #[must_use]
    pub fn value(&self, x: f64) -> f64 {
        if x < self.x_lo {
            return self.y_lo + self.dydx_lo * (x - self.x_lo);
        }
        if x > self.x_hi {
            return self.y_hi + self.dydx_hi * (x - self.x_hi);
        }
        // Locate the segment: x ∈ [seg_x[i][0], seg_x[i][5]). The last point
        // (x == x_hi) falls through to the final segment with u = 1.
        let mut idx = self.seg_x.len() - 1;
        for (i, sx) in self.seg_x.iter().enumerate() {
            if x >= sx[0] && x < sx[5] {
                idx = i;
                break;
            }
        }
        let u = calc_u(x, &self.seg_x[idx]);
        bezier_u(u, &self.seg_y[idx], 0)
    }
}

/// The three default Millard2012 fiber curves, built once and reused.
#[derive(Clone, Debug)]
pub struct MillardCurves {
    active_fl: SmoothSegmentedFunction,
    passive_fl: SmoothSegmentedFunction,
    force_velocity: SmoothSegmentedFunction,
}

impl Default for MillardCurves {
    fn default() -> Self {
        Self {
            active_fl: active_force_length_curve(),
            passive_fl: passive_force_length_curve(),
            force_velocity: force_velocity_curve(),
        }
    }
}

impl MillardCurves {
    /// Active force-length multiplier at normalized fiber length `norm_len`.
    #[must_use]
    pub fn active_fl(&self, norm_len: f64) -> f64 {
        self.active_fl.value(norm_len)
    }
    /// Passive (parallel-elastic) force multiplier at normalized fiber length.
    #[must_use]
    pub fn passive_fl(&self, norm_len: f64) -> f64 {
        self.passive_fl.value(norm_len)
    }
    /// Force-velocity multiplier at normalized fiber velocity `norm_vel`
    /// (`-1` = max shortening, `0` = isometric → 1.0, `+1` = max lengthening).
    #[must_use]
    pub fn force_velocity(&self, norm_vel: f64) -> f64 {
        self.force_velocity.value(norm_vel)
    }
}

/// Default Millard active-force-length curve (`createFiberActiveForceLengthCurve`,
/// curviness 1.0): min 0.4441, transition 0.73, peak 1.0, max 1.8123, shallow
/// ascending slope 0.8616.
#[allow(clippy::similar_names, clippy::many_single_char_names)]
fn active_force_length_curve() -> SmoothSegmentedFunction {
    let (x0, x1, x2, x3) = (0.4441, 0.73, 1.0, 1.8123);
    let ylow = 0.0;
    let dydx = 0.8616;
    let c = scale_curviness(1.0);

    let xdelta = 0.05 * x2;
    let xs = x2 - xdelta;

    let y0 = 0.0;
    let dydx0 = 0.0;
    let y1 = 1.0 - dydx * (xs - x1);
    let dydx01 = 1.25 * (y1 - y0) / (x1 - x0);
    let x01 = x0 + 0.5 * (x1 - x0);
    let y01 = y0 + 0.5 * (y1 - y0);
    let x1s = x1 + 0.5 * (xs - x1);
    let y1s = y1 + 0.5 * (1.0 - y1);
    let dydx1s = dydx;
    let y2 = 1.0;
    let dydx2 = 0.0;
    let y3 = 0.0;
    let dydx3 = 0.0;
    let x23 = (x2 + xdelta) + 0.5 * (x3 - (x2 + xdelta));
    let y23 = y2 + 0.5 * (y3 - y2);
    let dydx23 = (y3 - y2) / ((x3 - xdelta) - (x2 + xdelta));

    let segs = [
        corner(x0, ylow, dydx0, x01, y01, dydx01, c),
        corner(x01, y01, dydx01, x1s, y1s, dydx1s, c),
        corner(x1s, y1s, dydx1s, x2, y2, dydx2, c),
        corner(x2, y2, dydx2, x23, y23, dydx23, c),
        corner(x23, y23, dydx23, x3, ylow, dydx3, c),
    ];
    SmoothSegmentedFunction {
        seg_x: segs.iter().map(|s| s.0).collect(),
        seg_y: segs.iter().map(|s| s.1).collect(),
        x_lo: x0,
        x_hi: x3,
        y_lo: ylow,
        y_hi: ylow,
        dydx_lo: 0.0,
        dydx_hi: 0.0,
    }
}

/// Default Millard passive fiber-force-length curve
/// (`createFiberForceLengthCurve`): strain at zero force 0.0, strain at one
/// norm force 0.7, low-force stiffness 0.2, one-norm-force stiffness 2/0.7,
/// curviness 0.75.
#[allow(clippy::similar_names, clippy::many_single_char_names)]
fn passive_force_length_curve() -> SmoothSegmentedFunction {
    let e_zero: f64 = 0.0;
    let e_iso: f64 = 0.7;
    let k_low = 0.2;
    let k_iso = 2.0 / (e_iso - e_zero);
    let c = scale_curviness(0.75);

    let x_zero = 1.0 + e_zero;
    let y_zero = 0.0;
    let x_iso = 1.0 + e_iso;
    let y_iso = 1.0;
    let delta_x = (0.1 * (1.0 / k_iso)).min(0.1 * (x_iso - x_zero));
    let x_low = x_zero + delta_x;
    let x_foot = x_zero + 0.5 * (x_low - x_zero);
    let y_low = 0.0 + k_low * (x_low - x_foot);

    let segs = [
        corner(x_zero, y_zero, 0.0, x_low, y_low, k_low, c),
        corner(x_low, y_low, k_low, x_iso, y_iso, k_iso, c),
    ];
    SmoothSegmentedFunction {
        seg_x: segs.iter().map(|s| s.0).collect(),
        seg_y: segs.iter().map(|s| s.1).collect(),
        x_lo: x_zero,
        x_hi: x_iso,
        y_lo: y_zero,
        y_hi: y_iso,
        dydx_lo: 0.0,
        dydx_hi: k_iso,
    }
}

/// Default Millard fiber-force-velocity curve (`createFiberForceVelocityCurve`):
/// max eccentric multiplier 1.4, concentric slopes 0/0.25, isometric slope 5.0,
/// eccentric slopes 0/0.15, concentric curviness 0.6, eccentric 0.9.
#[allow(clippy::similar_names, clippy::many_single_char_names)]
fn force_velocity_curve() -> SmoothSegmentedFunction {
    let fmax_e = 1.4;
    let dydx_c = 0.0;
    let dydx_near_c = 0.25;
    let dydx_iso = 5.0;
    let dydx_e = 0.0;
    let dydx_near_e = 0.15;
    let cc = scale_curviness(0.6);
    let ce = scale_curviness(0.9);

    let (xc, yc) = (-1.0, 0.0);
    let x_near_c = -0.9;
    let y_near_c = yc + 0.5 * dydx_near_c * (x_near_c - xc) + 0.5 * dydx_c * (x_near_c - xc);
    let (x_iso, y_iso) = (0.0, 1.0);
    let (xe, ye) = (1.0, fmax_e);
    let x_near_e = 0.9;
    let y_near_e = ye + 0.5 * dydx_near_e * (x_near_e - xe) + 0.5 * dydx_e * (x_near_e - xe);

    let segs = [
        corner(xc, yc, dydx_c, x_near_c, y_near_c, dydx_near_c, cc),
        corner(x_near_c, y_near_c, dydx_near_c, x_iso, y_iso, dydx_iso, cc),
        corner(x_iso, y_iso, dydx_iso, x_near_e, y_near_e, dydx_near_e, ce),
        corner(x_near_e, y_near_e, dydx_near_e, xe, ye, dydx_e, ce),
    ];
    SmoothSegmentedFunction {
        seg_x: segs.iter().map(|s| s.0).collect(),
        seg_y: segs.iter().map(|s| s.1).collect(),
        x_lo: xc,
        x_hi: xe,
        y_lo: yc,
        y_hi: ye,
        dydx_lo: dydx_c,
        dydx_hi: dydx_e,
    }
}

/// Isometric Millard muscle force along the tendon/path, in newtons (tension
/// positive).
///
/// Rigid tendon (matching gait2392's `ignore_tendon_compliance=True` and our
/// engine) + variable-width (constant-height) pennation. At isometric the
/// force-velocity factor is 1.
///
/// - `mtu`: musculotendon (path) length [m]
/// - `act`: activation in `[0,1]`
/// - `f0`: max isometric force [N]; `l0`: optimal fiber length [m];
///   `lts`: tendon slack length [m]; `penn0`: pennation at optimal fiber length [rad]
// `imprecise_flops`: keep the naive `sqrt(a²+b²)` (not `hypot`) to match OpenSim's
// MuscleFixedWidthPennationModel bit-for-bit.
#[allow(clippy::imprecise_flops)]
#[must_use]
pub fn millard_isometric_path_force(
    curves: &MillardCurves,
    mtu: f64,
    act: f64,
    f0: f64,
    l0: f64,
    lts: f64,
    penn0: f64,
) -> f64 {
    // Rigid tendon: the fiber's projection along the tendon is the path length
    // minus the (constant) tendon slack length. Constant-height pennation gives
    // the true fiber length and the pennation angle.
    let along_tendon = mtu - lts;
    let height = l0 * penn0.sin();
    let fiber_len = (along_tendon * along_tendon + height * height).sqrt();
    let cos_penn = if fiber_len > 1e-12 {
        along_tendon / fiber_len
    } else {
        1.0
    };
    let norm_len = fiber_len / l0.max(1e-12);
    let active = act * curves.active_fl(norm_len);
    let passive = curves.passive_fl(norm_len);
    f0 * (active + passive) * cos_penn
}

#[cfg(test)]
mod spotcheck {
    use super::*;

    /// Self-contained anchor check vs real OpenSim 4.6 `calcValue` samples (the
    /// dense ~1e-6 curve validation + the muscle-force cross-check live in
    /// `tools/cf-osim`, which carries the vendored OpenSim references).
    #[test]
    fn curves_match_opensim_anchors() {
        let c = MillardCurves::default();
        for (x, want) in [
            (0.5, 0.09853),
            (0.8, 0.87068),
            (1.0, 1.0),
            (1.2, 0.82368),
            (1.5, 0.40334),
        ] {
            assert!(
                (c.active_fl(x) - want).abs() < 1e-4,
                "AFL({x})={} vs {want}",
                c.active_fl(x)
            );
        }
        for (x, want) in [
            (1.0, 0.0),
            (1.2, 0.04969),
            (1.4, 0.26289),
            (1.6, 0.71707),
            (1.7, 1.0),
        ] {
            assert!(
                (c.passive_fl(x) - want).abs() < 1e-4,
                "PFL({x})={} vs {want}",
                c.passive_fl(x)
            );
        }
        for (x, want) in [
            (-1.0, 0.0),
            (-0.5, 0.15809),
            (0.0, 1.0),
            (0.5, 1.33113),
            (1.0, 1.4),
        ] {
            assert!(
                (c.force_velocity(x) - want).abs() < 1e-4,
                "FV({x})={} vs {want}",
                c.force_velocity(x)
            );
        }
    }
}
