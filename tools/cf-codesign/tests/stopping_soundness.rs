//! Gate for the **sound stopping rule**: `loss_tol` is measured against a problem's
//! declared [`CoDesignProblem::loss_lower_bound`], and a problem that can state no
//! bound has that criterion disabled structurally rather than by a caller remembering
//! to pass a sentinel.
//!
//! The bug this prevents is silent: an objective with a reward term goes negative, any
//! finite `loss_tol` fires at that sign crossing, and the run returns a plausible
//! design that has nothing to do with an optimum.

use cf_codesign::{
    CoDesignProblem, ConduitTarget, OptConfig, OptResult, RouteTarget, StopReason, optimize,
};
use cf_design::Solid;
use nalgebra::{Point3, Vector3};

const MARGIN: f64 = 0.1;
const W_C: f64 = 10.0;
const W_R: f64 = 8.0;
const N_SAMPLES: usize = 40;

fn corridor_scene(gap: f64) -> ConduitTarget {
    let half = Vector3::new(5.0, 2.0, 5.0);
    let offset = gap / 2.0 + half.y;
    let body = Solid::cuboid(half)
        .translate(Vector3::new(0.0, offset, 0.0))
        .union(Solid::cuboid(half).translate(Vector3::new(0.0, -offset, 0.0)));
    ConduitTarget::new(
        body,
        Point3::new(-3.0, 0.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
        2,
        MARGIN,
        W_C,
        W_R,
        N_SAMPLES,
    )
}

fn off_ridge(t: &ConduitTarget, r0: f64) -> Vec<f64> {
    let mut x = t.x0(r0);
    x[1] += 0.05;
    x[4] += 0.05;
    x
}

/// (1) THE REGRESSION THIS EXISTS FOR. `ConduitTarget` declares no lower bound, so a
/// finite `loss_tol` must be IGNORED rather than halting the run where the signed
/// objective first crosses zero.
///
/// The config here deliberately carries the default `loss_tol = 1e-10` — before this
/// rule, `recommended_config` had to pass `loss_tol = −∞` by hand to stay correct, and
/// forgetting that sentinel silently produced a wrong answer. The optimum must still
/// land on the closed form `r* = (gap/2 − margin) + w_r/(2·m·w_c)`.
#[test]
fn conduit_ignores_loss_tol_and_still_finds_the_optimum() {
    let t = corridor_scene(2.0);
    assert!(
        t.loss_lower_bound().is_none(),
        "ConduitTarget must declare no lower bound"
    );
    let cfg = t.recommended_config();
    assert!(
        cfg.loss_tol.is_finite(),
        "this test is only meaningful with a FINITE loss_tol that would otherwise fire"
    );

    let res = optimize(&t, &off_ridge(&t, 0.3), &cfg);
    assert_ne!(
        res.stop_reason,
        StopReason::LossTol,
        "the loss criterion fired on a problem with no lower bound"
    );
    // The loss really does go negative here — i.e. the old absolute test WOULD have
    // tripped, so this is a live regression guard and not a vacuous one.
    assert!(
        res.loss < 0.0,
        "expected a negative objective at the optimum, got {}",
        res.loss
    );
    let predicted = (2.0 / 2.0 - MARGIN) + W_R / (2.0 * N_SAMPLES as f64 * W_C);
    let r = t.radius(&res.params);
    assert!(
        (r - predicted).abs() < 0.005,
        "radius {r} vs predicted {predicted} — the run stopped somewhere meaningless"
    );
}

/// (2) `RouteTarget`'s bound is the straight-line chord, and it is SOUND: the objective
/// can never dip below it. Checked against the real objective across a spread of
/// routes, including the degenerate straight one where the bound is tight.
#[test]
fn route_chord_bound_is_never_violated() {
    let t = RouteTarget::new(
        Solid::capsule(1.0, 3.0),
        Point3::new(-3.0, 0.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
        2,
        0.3,
        0.1,
        10.0,
        40,
    );
    let bound = t.loss_lower_bound().expect("route declares a bound");
    assert!(
        (bound - 6.0).abs() < 1e-12,
        "chord between (-3,0,0) and (3,0,0) should be 6, got {bound}"
    );

    let straight = t.straight_line();
    assert!(
        t.objective(&straight) >= bound - 1e-12,
        "straight route dips below its own chord bound"
    );
    // Each perturbation is applied to a FRESH copy, so these are five independent
    // routes rather than one drifting one.
    for (i, delta) in [(1, 0.5), (4, -2.0), (2, 3.0), (0, -7.0), (5, 12.0)] {
        let mut x = straight.clone();
        x[i] += delta;
        let j = t.objective(&x);
        assert!(
            j >= bound - 1e-12,
            "objective {j} fell below the chord bound {bound} after perturbing param {i}"
        );
    }
}

/// (3) `StopReason` names which criterion ended the run. The boolean it replaced could
/// not tell "reached an optimum" from "ran out of budget", so each exit is pinned here.
#[test]
fn stop_reason_distinguishes_the_exits() {
    let t = corridor_scene(2.0);
    let x0 = off_ridge(&t, 0.3);

    // Budget exhausted: no criterion can fire in one iteration.
    let capped = optimize(
        &t,
        &x0,
        &OptConfig {
            max_iters: 1,
            ..t.recommended_config()
        },
    );
    assert_eq!(capped.stop_reason, StopReason::MaxIters);

    // Gradient criterion: a tolerance nothing can miss.
    let loose = optimize(
        &t,
        &x0,
        &OptConfig {
            grad_tol: 1e9,
            ..t.recommended_config()
        },
    );
    assert_eq!(loose.stop_reason, StopReason::GradTol);

    // Loss criterion — on a target that DOES declare a bound, so it is reachable.
    // `RouteTarget`'s bound is the chord, and a tolerance wider than any detour the
    // scene can require forces the certificate immediately.
    let route = RouteTarget::new(
        Solid::capsule(1.0, 3.0),
        Point3::new(-3.0, 0.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
        2,
        0.3,
        0.1,
        10.0,
        40,
    );
    let by_loss = optimize(
        &route,
        &route.straight_line(),
        &OptConfig {
            loss_tol: 1e6,
            grad_tol: 0.0,
            max_iters: 50,
            ..OptConfig::default()
        },
    );
    assert_eq!(by_loss.stop_reason, StopReason::LossTol);

    // Every exit reports the gradient it stopped at, so a caller can judge the stop
    // rather than trusting the flag.
    assert!(
        capped.final_grad_inf.is_finite() && loose.final_grad_inf.is_finite(),
        "final_grad_inf must be reported on every exit"
    );
    assert_eq!(
        capped.final_grad_inf,
        capped.history.last().expect("one iteration ran").grad_inf,
        "final_grad_inf must match the stopping iterate"
    );
}

/// (4) CHARACTERIZATION of the mixed-scale gradient norm — pinned, not asserted as
/// desirable.
///
/// `ConduitTarget`'s design vector mixes a LINEAR route block with a LOG-space radius,
/// and nothing reconciles their scales, so `‖grad‖∞` is dominated by the route by
/// orders of magnitude. That means a `GradTol` stop on this problem is effectively a
/// test on the route alone — documented on [`OptConfig::grad_tol`].
///
/// This is committed evidence rather than a recollection: if the stopping norm is ever
/// made metric-aware, this test fails loudly and forces the conversation, instead of
/// the meaning of `converged` changing silently underneath the crate.
#[test]
fn conduit_gradient_norm_is_route_dominated() {
    let t = corridor_scene(2.0);
    let res: OptResult = optimize(&t, &off_ridge(&t, 0.3), &t.recommended_config());
    let (_loss, grad) = t.evaluate(&res.params);

    let route_block = grad[..grad.len() - 1]
        .iter()
        .fold(0.0_f64, |m, &g| m.max(g.abs()));
    let radius_block = grad[grad.len() - 1].abs();
    // Measured ratio is ~1.2e6; the bar sits an order below it so the assertion is
    // tight enough to actually protect the "orders of magnitude" claim in the
    // `grad_tol` doc, rather than passing on any mild imbalance.
    assert!(
        route_block > 1.0e5 * radius_block,
        "expected a route-dominated norm (the documented shape); route {route_block:e} \
         vs radius {radius_block:e} — if this narrowed deliberately, update the \
         `grad_tol` scope note"
    );
}
