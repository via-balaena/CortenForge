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
//! Validated to ~1e-12 against densely sampled real OpenSim 4.6 curve values
//! (gate 1e-5; `tools/cf-osim` cross-checks, vendored `millard_curves_opensim.json`).
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
    // Match OpenSim's tolerance/iteration budget exactly (SmoothSegmentedFunction.cpp:
    // UTOL = eps*1e2, MAXITER = 20).
    const UTOL: f64 = f64::EPSILON * 1e2;
    const MAXITER: u32 = 20;
    let span = px[5] - px[0];
    let mut u = if span.abs() > 1e-15 {
        ((ax - px[0]) / span).clamp(0.0, 1.0)
    } else {
        0.5
    };
    let mut f = bezier_u(u, px, 0) - ax;
    for _ in 0..MAXITER {
        if f.abs() <= UTOL {
            break;
        }
        let df = bezier_u(u, px, 1);
        if df.abs() < 1e-15 {
            break;
        }
        u = (u - f / df).clamp(0.0, 1.0);
        f = bezier_u(u, px, 0) - ax;
    }
    // The Bézier x-segments are monotone, so Newton from the linear guess always
    // converges; surface any pathology in dev/test (OpenSim hard-errors here).
    debug_assert!(
        f.abs() < 1e-9,
        "calc_u did not converge: residual {f:e} for x={ax}"
    );
    u
}

/// Build the 6 control points of one corner-shaped quintic Bézier segment that
/// joins `(x0,y0)` with tangent `dydx0` to `(x1,y1)` with tangent `dydx1`,
/// rounded by corner-fraction `c ∈ [0,1]`. Port of
/// `SegmentedQuinticBezierToolkit::calcQuinticBezierCornerControlPoints`.
// Short knot/slope/control-point names (x0, y0, dydx0, …) mirror the ported OpenSim source.
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
///
/// `ylow` (shoulder/minimum value) is 0.0, the value `Millard2012EquilibriumMuscle`
/// and gait2392 use; note the standalone `ActiveForceLengthCurve` *class* default is
/// 0.1, so this is bound to the validation target. The other knots/slopes match the
/// class defaults.
// Short knot/slope/control-point names (x0, y0, dydx0, …) mirror the ported OpenSim source.
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
// Short knot/slope/control-point names (x0, y0, dydx0, …) mirror the ported OpenSim source.
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
// Short knot/slope/control-point names (x0, y0, dydx0, …) mirror the ported OpenSim source.
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

/// Millard2012's default fiber-damping coefficient (`fiber_damping = 0.1`), the
/// value gait2392's muscles use. Adds a force `β·v̄` proportional to normalized
/// fiber velocity, so even a passive fiber resists rapid length change.
const FIBER_DAMPING: f64 = 0.1;

/// Minimum normalized fiber length OpenSim clamps to (= the active-force-length
/// curve's domain minimum, 0.4441). When the kinematic fiber would fall below it,
/// OpenSim holds the fiber at this length AND freezes its velocity to zero — so the
/// active (FV) and damping terms vanish there. Force is 0 either way (AFL and PFL
/// are both 0 at/below it), but freezing the velocity is what kills the damping term.
const MIN_NORM_FIBER_LENGTH: f64 = 0.4441;

/// A muscle's intrinsic force-generating parameters (a grouped, transposition-safe
/// alternative to bare same-typed `f64` arguments).
#[derive(Clone, Copy, Debug)]
pub struct MillardMuscleParams {
    /// Max isometric force, newtons.
    pub f0: f64,
    /// Optimal fiber length, meters.
    pub l0: f64,
    /// Tendon slack length, meters.
    pub lts: f64,
    /// Pennation angle at optimal fiber length, radians.
    pub penn0: f64,
    /// Max contraction velocity, in optimal fiber lengths per second (gait2392: 10).
    pub vmax: f64,
}

/// Millard muscle force along the tendon/path, in newtons (tension positive), at a
/// known musculotendon length and lengthening rate.
///
/// Rigid tendon (matching gait2392's `ignore_tendon_compliance=True` and our
/// engine) + variable-width (constant-height) pennation + fiber damping. The
/// normalized fiber force is `a·AFL(l̄)·FV(v̄) + PFL(l̄) + β·v̄`, scaled by `F0·cos(penn)`.
///
/// - `p`: the muscle's [`MillardMuscleParams`]
/// - `mtu`: musculotendon (path) length, meters
/// - `mtu_vel`: musculotendon lengthening rate, meters per second (shortening < 0)
/// - `act`: activation in `[0,1]`
// `imprecise_flops`: keep the naive `sqrt(a²+b²)` (not `hypot`) to match OpenSim's
// MuscleFixedWidthPennationModel bit-for-bit.
#[allow(clippy::imprecise_flops)]
#[must_use]
pub fn millard_path_force(
    curves: &MillardCurves,
    p: MillardMuscleParams,
    mtu: f64,
    mtu_vel: f64,
    act: f64,
) -> f64 {
    // Caller invariants (OpenSim hard-errors on these; we surface them in dev/test
    // and otherwise let a NaN propagate loudly rather than silently zero the force).
    debug_assert!(
        mtu.is_finite() && mtu_vel.is_finite() && act.is_finite(),
        "millard_path_force: non-finite input (mtu={mtu}, mtu_vel={mtu_vel}, act={act})"
    );
    debug_assert!(
        p.l0 > 0.0 && p.vmax > 0.0,
        "millard_path_force: l0 and vmax must be positive (l0={}, vmax={})",
        p.l0,
        p.vmax
    );
    // Rigid tendon: the fiber's projection along the tendon is the path length
    // minus the (constant) tendon slack length. Constant-height pennation gives
    // the true fiber length and the pennation angle.
    let along_tendon = mtu - p.lts;
    let height = p.l0 * p.penn0.sin();
    let fiber_len = (along_tendon * along_tendon + height * height).sqrt();
    let cos_penn = if fiber_len > 1e-12 {
        along_tendon / fiber_len
    } else {
        1.0
    };
    // Fiber velocity (rigid tendon, constant height): d(fiber)/dt = cos(penn)·d(mtu)/dt.
    // Normalize by the maximum shortening rate (vmax · L0), per OpenSim. Below the
    // floor, OpenSim clamps the fiber at the minimum length and freezes its velocity
    // (so the damping term vanishes, not just the active force).
    let raw_norm_len = fiber_len / p.l0.max(1e-12);
    let (norm_len, norm_vel) = if raw_norm_len < MIN_NORM_FIBER_LENGTH {
        (MIN_NORM_FIBER_LENGTH, 0.0)
    } else {
        (
            raw_norm_len,
            cos_penn * mtu_vel / (p.l0 * p.vmax).max(1e-12),
        )
    };

    let active = act * curves.active_fl(norm_len) * curves.force_velocity(norm_vel);
    let passive = curves.passive_fl(norm_len);
    let damping = FIBER_DAMPING * norm_vel;
    // A muscle pulls, never pushes: OpenSim floors the tendon force at 0 (which the
    // damping term can otherwise drive negative at high shortening). At isometric the
    // force is always ≥ 0, so this leaves the PR1 isometric result unchanged. Written
    // as a branch (not `.max(0.0)`, which would swallow a NaN into 0) so a bad input
    // propagates loudly instead of masquerading as a silent zero force.
    let force = p.f0 * (active + passive + damping) * cos_penn;
    if force < 0.0 { 0.0 } else { force }
}

/// Isometric Millard muscle force along the tendon/path (i.e. [`millard_path_force`]
/// with zero lengthening rate → force-velocity factor 1, zero damping).
#[must_use]
pub fn millard_isometric_path_force(
    curves: &MillardCurves,
    p: MillardMuscleParams,
    mtu: f64,
    act: f64,
) -> f64 {
    millard_path_force(curves, p, mtu, 0.0, act)
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

    /// Exact curve anchors (the dense 1e-4 grid skips x = 1.0 / 0.0). These land
    /// on segment knots and must be exact: AFL peaks at 1, FV is 1 at isometric,
    /// passive force is 0 at the resting fiber length.
    #[test]
    fn curve_anchors_are_exact() {
        let c = MillardCurves::default();
        assert!(
            (c.active_fl(1.0) - 1.0).abs() < 1e-12,
            "AFL(1)={}",
            c.active_fl(1.0)
        );
        assert!(
            (c.force_velocity(0.0) - 1.0).abs() < 1e-12,
            "FV(0)={}",
            c.force_velocity(0.0)
        );
        assert!(
            c.passive_fl(1.0).abs() < 1e-12,
            "PFL(1)={}",
            c.passive_fl(1.0)
        );
    }

    /// Force assembly at optimal fiber length, zero pennation (cos = 1, AFL = 1,
    /// passive = 0): the path force is exactly `act · F0`, and activation scales it.
    #[test]
    fn isometric_force_at_optimal_length() {
        let c = MillardCurves::default();
        // mtu = lts + l0 ⇒ along-tendon = l0, penn0 = 0 ⇒ fiber = l0 ⇒ norm_len = 1.
        let p = MillardMuscleParams {
            f0: 1000.0,
            l0: 0.1,
            lts: 0.2,
            penn0: 0.0,
            vmax: 10.0,
        };
        let mtu = p.lts + p.l0;
        assert!((millard_isometric_path_force(&c, p, mtu, 1.0) - 1000.0).abs() < 1e-9);
        assert!((millard_isometric_path_force(&c, p, mtu, 0.5) - 500.0).abs() < 1e-9);
        // Passive-only (activation 0) at resting length is 0 N.
        assert!(millard_isometric_path_force(&c, p, mtu, 0.0).abs() < 1e-9);
    }

    /// Force-velocity behavior at optimal length (cos = 1, AFL = 1, passive = 0):
    /// the isometric wrapper equals zero-velocity; shortening (mtu_vel < 0) reduces
    /// active force (FV < 1) and adds negative damping; lengthening raises it.
    #[test]
    fn force_velocity_reduces_shortening_force() {
        let c = MillardCurves::default();
        let p = MillardMuscleParams {
            f0: 1000.0,
            l0: 0.1,
            lts: 0.2,
            penn0: 0.0,
            vmax: 10.0,
        };
        let mtu = p.lts + p.l0;
        let iso = millard_path_force(&c, p, mtu, 0.0, 1.0);
        assert!((iso - millard_isometric_path_force(&c, p, mtu, 1.0)).abs() < 1e-12);
        // vmax·l0 = 1.0 m/s; shortening at 0.3 m/s → norm_vel = -0.3 → FV < 1.
        let shortening = millard_path_force(&c, p, mtu, -0.3, 1.0);
        let lengthening = millard_path_force(&c, p, mtu, 0.3, 1.0);
        assert!(shortening < iso, "shortening {shortening} !< iso {iso}");
        assert!(lengthening > iso, "lengthening {lengthening} !> iso {iso}");
        // At max shortening FV(-1)=0 and damping β·(-1)=-0.1 would give a negative
        // fiber force; a muscle can't push, so the tendon force is floored at 0.
        let max_short = millard_path_force(&c, p, mtu, -1.0, 1.0);
        assert!(
            max_short.abs() < 1e-12,
            "max_short={max_short} should clamp to 0"
        );
    }

    /// Min-fiber clamp freezes fiber VELOCITY, not just length: below the floor the
    /// damping term must vanish. A LENGTHENING velocity (where damping would be
    /// strictly positive if not frozen) must still give exactly 0 N.
    #[test]
    fn min_fiber_clamp_freezes_velocity() {
        let c = MillardCurves::default();
        let p = MillardMuscleParams {
            f0: 1000.0,
            l0: 0.1,
            lts: 0.2,
            penn0: 0.0,
            vmax: 10.0,
        };
        // mtu = lts + 0.4·l0 ⇒ raw_norm_len = 0.4 < 0.4441 (clamped); penn0 = 0.
        let mtu = p.lts + 0.4 * p.l0;
        // Lengthening: an unfrozen β·v̄ would be > 0, yet the frozen clamp gives 0.
        assert!(millard_path_force(&c, p, mtu, 0.5, 1.0).abs() < 1e-12);
        assert!(millard_path_force(&c, p, mtu, 0.5, 0.0).abs() < 1e-12);
    }
}
