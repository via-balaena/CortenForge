//! Worked example — route AND radius together: the *fattest conduit that fits*.
//!
//! `route_inverse_design` bends a cable of FIXED thickness around a body. Here the
//! thickness is a second design variable, so the optimizer must answer a genuinely
//! two-sided question: a fatter conduit is worth more, but it demands more clearance,
//! which forces a longer detour. The design vector is `[x,y,z, …, ln r]` — signed
//! positions alongside a strictly-positive scale, the mixed-conditioning regime, with
//! the radius reparametrized in log-space internally (the `JointTarget` pattern). Run:
//!
//! ```text
//! cargo run -p cf-codesign --example conduit_inverse_design
//! ```

#![allow(clippy::expect_used, clippy::print_stdout)]

use std::sync::Arc;

use cf_codesign::{ConduitTarget, optimize};
use cf_design::Solid;
use nalgebra::{Point3, Vector3};

const MARGIN: f64 = 0.1;
const W_C: f64 = 10.0;
const W_R: f64 = 8.0;
const N_SAMPLES: usize = 40;

/// Two flat slabs leaving a corridor of width `gap` centered on the route.
fn corridor(gap: f64) -> ConduitTarget {
    let half = Vector3::new(5.0, 2.0, 5.0);
    let offset = gap / 2.0 + half.y;
    let body = Solid::cuboid(half)
        .translate(Vector3::new(0.0, offset, 0.0))
        .union(Solid::cuboid(half).translate(Vector3::new(0.0, -offset, 0.0)));
    ConduitTarget::new(
        Arc::new(body),
        Point3::new(-3.0, 0.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
        2,
        MARGIN,
        W_C,
        W_R,
        N_SAMPLES,
    )
}

fn solve(target: &ConduitTarget, r0: f64, nudge: f64) -> (f64, f64, Vec<f64>) {
    let mut x0 = target.x0(r0);
    x0[1] += nudge; // off the symmetric stationary ridge
    x0[4] += nudge;
    let res = optimize(target, &x0, &target.recommended_config());
    (
        target.radius(&res.params),
        target.min_clearance(&res.params),
        res.params,
    )
}

fn main() {
    // Part 1 — a capsule to route around, with the tube free to fatten.
    let capsule = ConduitTarget::new(
        Arc::new(Solid::capsule(1.0, 3.0)),
        Point3::new(-3.0, 0.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
        2,
        MARGIN,
        W_C,
        W_R,
        N_SAMPLES,
    );
    let r0 = 0.3;
    let (r, clr, params) = solve(&capsule, r0, 0.15);
    println!("routing a conduit around a capsule (body radius 1.0):");
    println!("  start:  radius {r0:.3}, straight route through the body");
    println!("  solved: radius {r:.3}, min clearance {clr:.3}");
    println!(
        "          interior control points ({:+.3}, {:+.3}, {:+.3}) / ({:+.3}, {:+.3}, {:+.3})",
        params[0], params[1], params[2], params[3], params[4], params[5]
    );
    println!("  The route bent AND the tube grew — each made room for the other.\n");

    // Part 2 — squeeze the corridor and watch the optimal radius follow it down.
    // With flat walls every sample sits at the same clearance gap/2, so the optimum
    // is predicted in closed form by the radius stationarity condition:
    //     r* = (gap/2 − margin) + w_r / (2·m·w_c),   m = n_samples binding.
    println!("squeezing a flat corridor — the optimal radius tracks the space available:");
    println!("    gap    r* (found)   r* (predicted)   min clearance");
    for gap in [3.0, 2.0, 1.0, 0.5, 0.1] {
        let target = corridor(gap);
        let (r, clr, _) = solve(&target, 0.3, 0.05);
        let predicted = (gap / 2.0 - MARGIN) + W_R / (2.0 * N_SAMPLES as f64 * W_C);
        // A negative prediction is the closed form reporting infeasibility: the
        // corridor cannot even accommodate the margin, let alone a tube.
        let flag = if predicted < 0.0 {
            "  ← infeasible"
        } else {
            ""
        };
        println!("  {gap:5.2}   {r:9.4}   {predicted:14.4}   {clr:13.4}{flag}");
    }

    println!("\nVerdict: narrowing the corridor drives the optimal radius down in lockstep");
    println!("with the closed form. The last row is INFEASIBLE — the corridor is narrower");
    println!("than the clearance margin alone, so no conduit fits however thin, and the");
    println!("radius collapses toward zero without ever reaching it (the log-space");
    println!("parametrization keeps r > 0 structurally, with no bound to clamp).");
}
