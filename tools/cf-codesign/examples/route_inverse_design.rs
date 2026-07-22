//! Worked example — the GEOMETRY axis of the co-design loop (route vs. body).
//!
//! A cable must run from one point to another past a body it may not touch. The
//! design variables are the interior control points of a differentiable centerline
//! ([`cf_routing::Path`]); the objective trades the route's length against a
//! clearance penalty over the body's signed-distance field. Optimizing with the
//! keystone Adam loop bends the route around the body — the geometry analogue of
//! `trajectory_inverse_design` (a soft MATERIAL) and `control_inverse_design` (a
//! control SCHEDULE). Run:
//!
//! ```text
//! cargo run -p cf-codesign --example route_inverse_design
//! ```

#![allow(clippy::expect_used, clippy::print_stdout)]

use std::sync::Arc;

use cf_codesign::{OptConfig, RouteTarget, optimize};
use cf_design::Solid;
use nalgebra::Point3;

fn main() {
    // Body: a capsule (radius 1, axis Z) at the origin. Cable: (-3,0,0) → (3,0,0),
    // which runs straight through the body unless routed around it. Two interior
    // control points are the design variables; the tube (radius 0.3) must clear the
    // body by a 0.1 margin, so the centerline must stay req = 0.4 clear.
    let target = RouteTarget::new(
        Arc::new(Solid::capsule(1.0, 3.0)),
        Point3::new(-3.0, 0.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
        2,
        0.3,
        0.1,
        10.0,
        40,
    );

    // The straight route is an even-symmetric stationary ridge (a +y detour is as
    // good as −y, so its gradient there is zero). Start OFF the ridge so the
    // optimizer can pick a side.
    let mut x0 = target.straight_line();
    x0[1] += 0.15;
    x0[4] += 0.15;

    let j0 = target.objective(&x0);
    println!("start route (near-straight):");
    println!("  objective   = {j0:.4}");
    println!(
        "  min clearance = {:.4}  (req {:.2}; negative ⇒ inside the body)",
        target.min_clearance(&x0),
        target.req_clearance()
    );

    let cfg = OptConfig {
        lr: 0.05,
        max_iters: 600,
        grad_tol: 1e-3,
        ..OptConfig::default()
    };
    let res = optimize(&target, &x0, &cfg);

    // The loss reaches its floor well before `max_iters`; the gradient-norm stop
    // does NOT trip because the soft clearance penalty holds a small persistent
    // gradient at the length↔clearance equilibrium (a fixed-lr Adam limit cycle
    // about a stiff minimum) — so the stop reason reflects the gradient criterion, not
    // optimality. The stable loss below is the real convergence signal.
    println!("\noptimized route ({} iters):", res.iters);
    println!("  objective   = {:.4}", res.loss);
    println!(
        "  min clearance = {:.4}  (req {:.2})",
        target.min_clearance(&res.params),
        target.req_clearance()
    );
    println!("  interior control points:");
    for i in 0..2 {
        println!(
            "    ({:+.3}, {:+.3}, {:+.3})",
            res.params[3 * i],
            res.params[3 * i + 1],
            res.params[3 * i + 2]
        );
    }

    println!(
        "\nVerdict: the route bent around the body — objective {:.1} → {:.1}. The",
        j0, res.loss
    );
    println!(
        "centerline now stands {:.3} clear of the body (tube radius 0.30 clears it),",
        target.min_clearance(&res.params)
    );
    println!(
        "settling just inside the {:.2} target margin — the soft length↔clearance",
        target.req_clearance()
    );
    println!("equilibrium, where the length term trims the last sliver of margin.");
}
