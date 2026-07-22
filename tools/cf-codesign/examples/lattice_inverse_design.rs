//! Worked example — sizing the struts of a truss: the *lightest structure that is
//! stiff enough*.
//!
//! The per-strut cross-section areas are the design variables; the objective is
//! mass-penalized compliance `J = C(A) + w·Σ Aₑ·Lₑ`. For a statically determinate
//! truss the optimum is the closed-form fully-stressed design `Aₑ* = |Nₑ|/√(Eₑ·w)`,
//! and a strut that carries no load has optimal area zero — it is designed away.
//! The design vector is `[ln A₀ … ln A_{n−1}]` (log-space, so every area stays
//! positive). Run:
//!
//! ```text
//! cargo run -p cf-codesign --example lattice_inverse_design
//! ```

#![allow(clippy::expect_used, clippy::print_stdout)]

use cf_codesign::{LatticeTarget, optimize};
use nalgebra::vector;
use sim_truss::{Load, Strut, Support, Truss2};

const W: f64 = 1.0; // mass/compliance exchange rate

fn main() {
    let load = vector![0.0, -std::f64::consts::SQRT_2]; // gives member force |N| = 1

    // Part 1 — a determinate two-bar truss with a closed-form optimum. Symmetric
    // geometry (free node at the origin, supports up-left and up-right) so both
    // struts carry |N| = 1, but different moduli (E = 1 and 4) so their optimal
    // areas differ: A* = |N|/√(E·w) = 1.0 and 0.5.
    let two_bar = LatticeTarget::new(
        Truss2::new(
            vec![vector![0.0, 0.0], vector![-1.0, 1.0], vector![1.0, 1.0]],
            vec![
                Strut {
                    i: 0,
                    j: 1,
                    youngs_modulus: 1.0,
                },
                Strut {
                    i: 0,
                    j: 2,
                    youngs_modulus: 4.0,
                },
            ],
            vec![
                Support {
                    node: 1,
                    fixed: [true, true],
                },
                Support {
                    node: 2,
                    fixed: [true, true],
                },
            ],
            vec![Load {
                node: 0,
                force: load,
            }],
        )
        .expect("valid two-bar truss"),
        W,
    );
    let res = optimize(&two_bar, &two_bar.x0(1.0), &two_bar.recommended_config());
    let areas = two_bar.to_physical(&res.params);
    println!("determinate two-bar truss — recovering the fully-stressed design A* = |N|/√(Ew):");
    println!(
        "  strut A (E = 1):  area {:.4}   (closed form 1.0000)",
        areas[0]
    );
    println!(
        "  strut B (E = 4):  area {:.4}   (closed form 0.5000)",
        areas[1]
    );
    println!("  the stiffer strut needs less material for the same force.\n");

    // Part 2 — add a horizontal strut to the same free node. With symmetric moduli
    // the node moves only vertically, so the horizontal strut is a zero-force
    // member: pure cost, no stiffness. The optimizer designs it away.
    let with_redundant = LatticeTarget::new(
        Truss2::new(
            vec![
                vector![0.0, 0.0],
                vector![-1.0, 1.0],
                vector![1.0, 1.0],
                vector![2.0, 0.0],
            ],
            vec![
                Strut {
                    i: 0,
                    j: 1,
                    youngs_modulus: 1.0,
                },
                Strut {
                    i: 0,
                    j: 2,
                    youngs_modulus: 1.0,
                },
                Strut {
                    i: 0,
                    j: 3,
                    youngs_modulus: 1.0,
                }, // horizontal — zero force
            ],
            vec![
                Support {
                    node: 1,
                    fixed: [true, true],
                },
                Support {
                    node: 2,
                    fixed: [true, true],
                },
                Support {
                    node: 3,
                    fixed: [true, true],
                },
            ],
            vec![Load {
                node: 0,
                force: load,
            }],
        )
        .expect("valid three-bar truss"),
        W,
    );
    let res = optimize(
        &with_redundant,
        &with_redundant.x0(1.0),
        &with_redundant.recommended_config(),
    );
    let areas = with_redundant.to_physical(&res.params);
    println!("same truss plus a zero-force horizontal strut — watch it collapse:");
    println!(
        "  load-bearing struts:  area {:.4}, {:.4}",
        areas[0], areas[1]
    );
    println!("  zero-force strut:     area {:.6}", areas[2]);

    println!("\nVerdict: the two load-bearing struts hold their determinate optimum while the");
    println!("zero-force strut collapses toward zero — it is removed from the design. It stays");
    println!("strictly positive (log-space keeps A > 0), approaching zero without reaching it,");
    println!("and its gradient vanishes with its area — which is why certifying convergence for");
    println!("a lattice of hundreds of such struts needs more than a gradient-norm stop.");
}
