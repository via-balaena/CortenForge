//! Gate for the route-vs-body co-design target (`RouteTarget`) — the geometry axis
//! of the co-design optimizer.
//!
//! Scene: a capsule body (radius 1, axis Z, at origin) and a cable from (-3,0,0) to
//! (3,0,0) whose two interior control points are the design variables. The straight
//! route plunges through the body; the optimizer must bend it clear while keeping it
//! short.

use std::sync::Arc;

use cf_codesign::{CoDesignProblem, OptConfig, RouteTarget, optimize};
use cf_design::Solid;
use nalgebra::Point3;

fn scene() -> RouteTarget {
    RouteTarget::new(
        Arc::new(Solid::capsule(1.0, 3.0)), // body: capsule along Z, radius 1
        Point3::new(-3.0, 0.0, 0.0),        // start
        Point3::new(3.0, 0.0, 0.0),         // end
        2,                                  // interior control points (design vars)
        0.3,                                // tube radius
        0.1,                                // clearance margin  → req = 0.4
        10.0,                               // clearance penalty weight
        40,                                 // samples
    )
}

/// An off-ridge initial route: the straight line nudged in +y so the optimizer is
/// off the symmetric stationary ridge (see the symmetry test below).
fn off_ridge(target: &RouteTarget) -> Vec<f64> {
    let mut x = target.straight_line();
    x[1] += 0.15; // interior CP 0, y
    x[4] += 0.15; // interior CP 1, y
    x
}

/// (1) The `evaluate` gradient (central FD at the target's internal eps) matches an
/// INDEPENDENT central FD of `objective` taken in the test at a different eps —
/// confirming the FD plumbing is correct and the objective is well-conditioned here
/// (the two eps agree ⇒ a plateau; no sample sits on the penalty's `max(0, ·)`
/// threshold at this point, where the ReLU kink would otherwise show up).
#[test]
fn route_gradient_matches_independent_fd() {
    let t = scene();
    let x = off_ridge(&t);
    let (_loss, grad) = t.evaluate(&x);

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

/// (2) The straight route through a body-symmetric scene is a STATIONARY RIDGE: the
/// detour-direction (y) gradient vanishes there. This is why the optimizer must be
/// initialized off the ridge — a load-bearing property, tested so it stays true.
#[test]
fn route_symmetric_ridge_is_stationary() {
    let t = scene();
    let (_loss, grad) = t.evaluate(&t.straight_line());
    // The y-components of both interior control points (indices 1, 4) are the
    // detour direction; by the even symmetry of the objective they are ~0.
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
}

/// (3) RECOVERY: from an off-ridge start, the real Adam `optimize` loop bends the
/// route around the body — the objective falls substantially and the converged route
/// stands well clear of the body. The clearance settles just *inside* the required
/// margin (the soft length↔clearance equilibrium trims the last sliver), so the gate
/// asserts it reaches near the target, not the full margin.
#[test]
fn route_codesign_bends_around_body() {
    let t = scene();
    let x0 = off_ridge(&t);
    let j0 = t.objective(&x0);
    // The start route is inside the body (a real bend is required, not a no-op).
    assert!(
        t.min_clearance(&x0) < 0.0,
        "start route should begin inside the body — not a real test"
    );

    let cfg = OptConfig {
        lr: 0.05,
        max_iters: 600,
        grad_tol: 1e-3,
        ..OptConfig::default()
    };
    let res = optimize(&t, &x0, &cfg);

    assert!(
        res.loss < 0.5 * j0,
        "objective did not fall: {j0} -> {}",
        res.loss
    );
    // Cleared to near the target margin. The bound is loose (req − 0.1) so the
    // documented soft-equilibrium Adam limit cycle cannot flake it, while still
    // proving a real detour (start clearance was negative).
    let clr = t.min_clearance(&res.params);
    assert!(
        clr > t.req_clearance() - 0.1,
        "converged route did not clear the body near the target: clearance {clr}, req {}",
        t.req_clearance()
    );
}

/// (4) NEGATIVE CONTROL for the symmetry finding: initialized ON the ridge (the
/// straight line), the optimizer STALLS — it cannot break symmetry — so the route
/// stays through the body. This is why (3) starts off the ridge.
#[test]
fn route_on_ridge_stalls() {
    let t = scene();
    let x0 = t.straight_line();
    let cfg = OptConfig {
        lr: 0.05,
        max_iters: 200,
        grad_tol: 1e-3,
        ..OptConfig::default()
    };
    let res = optimize(&t, &x0, &cfg);
    // Still breaching the body (never detoured), because the ridge gradient can't
    // pick a side.
    assert!(
        t.min_clearance(&res.params) < 0.0,
        "on-ridge run unexpectedly cleared the body (clearance {})",
        t.min_clearance(&res.params)
    );
}
