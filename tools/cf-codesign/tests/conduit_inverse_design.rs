//! Gate for the route+radius co-design target (`ConduitTarget`) — the *fattest
//! conduit that fits*, the mixed-conditioning rung of the geometry axis.
//!
//! Two scenes. The **capsule** scene mirrors `route_inverse_design.rs` (capsule radius
//! 1 on the Z axis, cable from (-3,0,0) to (3,0,0), two interior control points) and
//! exercises the coupling: a fatter tube demands more clearance, forcing a wider
//! detour. The **corridor** scene is two flat slabs straddling the route, where the
//! available clearance is exactly `gap/2` everywhere along a centered route — flat
//! walls make the optimal radius analytically predictable, which is what turns the
//! headline gate from a direction check into a quantitative one.

use cf_codesign::{CoDesignProblem, ConduitTarget, OptConfig, optimize};
use cf_design::Solid;
use nalgebra::{Point3, Vector3};

const MARGIN: f64 = 0.1;
const W_C: f64 = 10.0; // clearance penalty weight
const W_R: f64 = 8.0; // radius reward (exchange rate)
const N_SAMPLES: usize = 40;

/// The capsule scene of the sibling route gate, with the radius now a design variable.
fn capsule_scene() -> ConduitTarget {
    ConduitTarget::new(
        Solid::capsule(1.0, 3.0),
        Point3::new(-3.0, 0.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
        2,
        MARGIN,
        W_C,
        W_R,
        N_SAMPLES,
    )
}

/// Two flat slabs leaving a corridor of width `gap` centered on the route, so a
/// centered centerline sees a *constant* clearance `gap/2` at every sample.
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

/// The straight route at radius `r0`, nudged in +y so the optimizer starts off the
/// symmetric stationary ridge (see the sibling route gate).
fn off_ridge(target: &ConduitTarget, r0: f64, nudge: f64) -> Vec<f64> {
    let mut x = target.x0(r0);
    x[1] += nudge; // interior CP 0, y
    x[4] += nudge; // interior CP 1, y
    x
}

/// The optimal radius predicted by the radius stationarity condition when `m` samples
/// bind simultaneously: `r* = (φ_min − margin) + w_r / (2·m·w_c)`. In the flat-walled
/// corridor a centered route puts *every* sample at the same clearance, so `m` is the
/// full sample count — which is what makes this scene analytically checkable.
fn predicted_radius(gap: f64) -> f64 {
    (gap / 2.0 - MARGIN) + W_R / (2.0 * N_SAMPLES as f64 * W_C)
}

/// (1) The `evaluate` gradient (central FD at the target's internal eps) matches an
/// INDEPENDENT central FD taken in the test at a different eps — over the coordinate
/// block AND the trailing `ln r` component, confirming the log-space parameter is
/// differentiated correctly in `p`-space rather than by a mis-applied chain rule.
#[test]
fn conduit_gradient_matches_independent_fd() {
    let t = capsule_scene();
    let x = off_ridge(&t, 0.3, 0.15);
    let (_loss, grad) = t.evaluate(&x);
    assert_eq!(grad.len(), 7, "6 coordinates + ln r");

    let eps = 1e-4; // deliberately different from the target's internal 1e-6
    for i in 0..grad.len() {
        let mut xp = x.clone();
        xp[i] += eps;
        let jp = t.objective(&xp);
        xp[i] -= 2.0 * eps;
        let jm = t.objective(&xp);
        let fd = (jp - jm) / (2.0 * eps);
        let tol = 1e-3 * grad[i].abs().max(1.0);
        assert!(
            (grad[i] - fd).abs() < tol,
            "param {i}: evaluate grad {} vs independent FD {fd} (tol {tol})",
            grad[i]
        );
    }
}

/// (2) The straight route is a stationary RIDGE in the detour direction, exactly as for
/// the fixed-radius sibling — but the RADIUS axis is NOT on a ridge there. Adding the
/// second design variable must not accidentally make the whole problem stationary at
/// the symmetric start: the route cannot pick a side, while the radius still knows
/// which way to move.
#[test]
fn conduit_symmetric_ridge_is_stationary_in_route_only() {
    let t = capsule_scene();
    let (_loss, grad) = t.evaluate(&t.x0(0.3));
    assert!(
        grad[1].abs() < 1e-6,
        "ridge y-grad (CP0) not stationary: {}",
        grad[1]
    );
    assert!(
        grad[4].abs() < 1e-6,
        "ridge y-grad (CP1) not stationary: {}",
        grad[4]
    );
    // The route starts buried in the body, so a fatter tube only deepens the breach:
    // the radius gradient is strictly positive (descent shrinks r) and far from zero.
    assert!(
        grad[6] > 1e-3,
        "radius axis should be live on the route ridge, got d/d(ln r) = {}",
        grad[6]
    );
}

/// (3) RECOVERY, both axes at once: from a thin tube on a breaching route, the real
/// Adam loop bends the route around the body AND grows the radius, ending with the
/// conduit clear. This is the coupling in action — the radius could only grow because
/// the route made room for it.
#[test]
fn conduit_codesign_bends_and_sizes() {
    let t = capsule_scene();
    let r0 = 0.3;
    let x0 = off_ridge(&t, r0, 0.15);
    assert!(
        t.min_clearance(&x0) < 0.0,
        "start route should begin inside the body — not a real test"
    );

    let res = optimize(&t, &x0, &t.recommended_config());
    assert!(res.converged(), "capsule recovery did not converge");

    let r = t.radius(&res.params);
    assert!(r > 2.0 * r0, "radius did not grow: {r0} -> {r}");
    // The route detoured far enough that the fattened conduit essentially clears the
    // body. The equilibrium settles a slack INSIDE the requirement (the reward buys a
    // sliver of tolerance), so a shortfall is expected and the bound must allow it.
    // On this curved body the slack is NOT predictable — `m`, the number of binding
    // samples, is unknown — so 0.1 is an empirical bound with ~2× headroom over the
    // measured 0.049 shortfall, not a figure derived from `w_r/(2·m·w_c)`. The
    // quantitative check of the closed form lives in the flat-corridor gate below,
    // where `m` is known.
    let clr = t.min_clearance(&res.params);
    let req = t.req_clearance(&res.params);
    assert!(
        clr > req - 0.1,
        "conduit did not clear the body: clearance {clr}, required {req}"
    );
    // Both axes moved off their start (a radius-only or route-only solution would fail
    // one of these).
    assert!(
        res.params[1].abs() > 0.5,
        "route did not detour: CP0 y = {}",
        res.params[1]
    );
}

/// (4) NEGATIVE CONTROL for the symmetry finding, carried over to the two-axis problem:
/// initialized ON the ridge, the route still cannot break symmetry, so it stays through
/// the body no matter what the radius does.
#[test]
fn conduit_on_ridge_stalls() {
    let t = capsule_scene();
    let r0 = 0.3;
    let res = optimize(&t, &t.x0(r0), &t.recommended_config());
    assert!(
        t.min_clearance(&res.params) < 0.0,
        "on-ridge run unexpectedly cleared the body (clearance {})",
        t.min_clearance(&res.params)
    );
    // The stall must be SYMMETRY, not a dead optimizer: the route is frozen on the
    // ridge while the radius axis — which is not symmetry-locked — still moves. Without
    // this the assertion above would pass even if `optimize` were a no-op, since
    // `min_clearance` reads only the route.
    assert!(
        res.params[1].abs() < 1e-6 && res.params[4].abs() < 1e-6,
        "route left the ridge: y = {}, {}",
        res.params[1],
        res.params[4]
    );
    let r = t.radius(&res.params);
    assert!(
        (r - r0).abs() > 1e-3,
        "radius did not move, so the run proves nothing about symmetry: {r}"
    );
}

/// (5) **THE GATE THAT MATTERS**: narrow the corridor and the optimal radius must FALL,
/// then no conduit fits at all.
///
/// In the flat-walled corridor the available clearance is `gap/2` at every sample, so
/// the optimum is not merely monotone in the gap — it is *predicted* by the radius
/// stationarity condition `r* = (gap/2 − margin) + w_r/(2·m·w_c)` with `m = n_samples`
/// binding. Asserting the closed form (rather than only the ordering) makes this a
/// quantitative check of the objective's trade-off, not just of its sign.
///
/// The feasible runs stop at `max_iters` rather than `grad_tol`: the length↔clearance
/// equilibrium is a shallow Adam limit cycle (the same one documented for the
/// fixed-radius sibling). It is benign — `r*` is stable to ~1% between 800 and 8000
/// iterations — so the gate asserts the optimum, not the convergence flag.
#[test]
fn conduit_radius_falls_as_corridor_narrows() {
    let gaps = [3.0, 2.0, 1.0, 0.5];
    let mut radii = Vec::new();
    for gap in gaps {
        let t = corridor_scene(gap);
        let res = optimize(&t, &off_ridge(&t, 0.3, 0.05), &t.recommended_config());
        let r = t.radius(&res.params);
        let pred = predicted_radius(gap);
        // The tolerance is deliberately BELOW the slack term `w_r/(2·m·w_c)` = 0.01
        // that distinguishes this closed form from the naive `gap/2 − margin`. A
        // looser bound would pass for a prediction that drops the slack entirely,
        // making the test unable to check the thing it exists to check. Measured
        // error is ~0.0013, so this still leaves ~4× headroom.
        assert!(
            (r - pred).abs() < 0.005,
            "gap {gap}: optimal radius {r} vs predicted {pred}"
        );
        radii.push(r);
    }
    // Strictly monotone: a narrower corridor admits a thinner conduit.
    for w in radii.windows(2) {
        assert!(
            w[1] < w[0],
            "radius did not fall as the corridor narrowed: {radii:?}"
        );
    }

    // Narrower than the margin alone: NO conduit fits, however thin, because the
    // required clearance `r + margin` exceeds the available `gap/2` even as r -> 0.
    let gap = 0.1;
    let t = corridor_scene(gap);
    assert!(
        gap / 2.0 < MARGIN,
        "this sub-case is only meaningful when the corridor is narrower than the margin"
    );
    let res = optimize(&t, &off_ridge(&t, 0.3, 0.05), &t.recommended_config());
    let clr = t.min_clearance(&res.params);
    assert!(
        clr < t.req_clearance(&res.params),
        "an infeasible corridor reported a fitting conduit: clearance {clr}, required {}",
        t.req_clearance(&res.params)
    );
    // The radius collapses toward zero — the only thing left to give.
    assert!(
        t.radius(&res.params) < 0.05,
        "radius did not collapse in an infeasible corridor: {}",
        t.radius(&res.params)
    );
}

/// (6) The LOG-SPACE GUARANTEE, tested where it is actually under pressure: in an
/// infeasible corridor the objective pushes the radius down without limit, yet
/// `r = exp(p)` stays strictly positive and finite at EVERY iterate — it decays toward
/// zero geometrically and never reaches or crosses it. A linear radius parameter would
/// have gone negative here and produced a nonsensical negative required clearance; this
/// is the structural property that makes the bound-free `lower_bounds = None` sound.
#[test]
fn conduit_radius_stays_positive_under_collapse() {
    let t = corridor_scene(0.1); // narrower than the margin: r is driven down forever
    let cfg = OptConfig {
        max_iters: 3000,
        ..t.recommended_config()
    };
    let res = optimize(&t, &off_ridge(&t, 0.3, 0.05), &cfg);

    for (k, rec) in res.history.iter().enumerate() {
        let r = t.radius(&rec.params);
        assert!(
            r > 0.0 && r.is_finite(),
            "iterate {k}: radius left the positive reals: {r}"
        );
        assert!(
            t.req_clearance(&rec.params) > 0.0,
            "iterate {k}: required clearance went non-positive"
        );
    }
    // It really was driven hard against zero (otherwise the test proves nothing).
    let r_final = t.radius(&res.params);
    assert!(
        r_final < 1e-3,
        "radius was not actually driven toward zero: {r_final}"
    );

    // The DECAY ITSELF, pinned: run the same infeasible scene to three horizons and
    // require the radius to keep falling by roughly an order of magnitude each time
    // while never reaching zero. This is what distinguishes a genuine geometric
    // collapse in log-space from a radius that has merely parked at a small value —
    // and it is the evidence behind the claim that the log-space parametrization holds
    // `r > 0` under unbounded downward pressure.
    // Seeded from the STARTING radius rather than infinity, so the first horizon's
    // comparison is load-bearing too: 800 iterations must already have collapsed the
    // radius well below where it began, not merely returned something finite.
    let mut previous = 0.3;
    for max_iters in [800, 3000, 8000] {
        let cfg = OptConfig {
            max_iters,
            ..t.recommended_config()
        };
        let r = t.radius(&optimize(&t, &off_ridge(&t, 0.3, 0.05), &cfg).params);
        assert!(r > 0.0, "radius reached zero at {max_iters} iters");
        assert!(
            r < previous / 3.0,
            "radius stopped collapsing at {max_iters} iters: {r} vs {previous} before"
        );
        previous = r;
    }
}
