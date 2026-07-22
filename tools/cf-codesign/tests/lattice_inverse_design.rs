//! Gates for the lattice design axis ([`LatticeTarget`]) — the structural rung of
//! the co-design optimizer.
//!
//! The quantitative anchor is a **statically determinate** truss, whose member
//! forces are independent of the areas. For such a truss the mass-penalized
//! compliance `J(A) = C(A) + w·Σ Aₑ·Lₑ` is convex in `A > 0` with a closed-form
//! optimum — the fully-stressed design
//!
//! ```text
//! Aₑ* = |Nₑ| / √(Eₑ · w),
//! ```
//!
//! `Nₑ` the (area-independent) member force. The gates check recovery to that
//! optimum, the analytic gradient against finite differences, and that a genuine
//! zero-force strut collapses toward zero.
//!
//! They also settle the stopping question, with a scoped answer. The log-norm cannot
//! tell a converged strut from a still-collapsing one
//! (`collapse_is_not_certifiable_by_gradient_norm`), which raised whether a
//! metric-aware stop was needed. For **determinate** structures the plain log-norm is
//! already a valid KKT certificate (`converged_lattice_satisfies_kkt`,
//! `interior_optimum_fires_gradtol`) and is scale-stable
//! (`log_norm_stationarity_is_scale_invariant_under_material_scaling`). For
//! **indeterminate** structures it is *not universal*
//! (`log_norm_hides_a_kkt_violation_on_indeterminate_structures`) — but the violation
//! needs an adversarial thin start, and uniform-start descent never reaches it
//! (`indeterminate_uniform_start_strands_no_grower`), so no metric was warranted.

use cf_codesign::{CoDesignProblem, LatticeTarget, OptConfig, StopReason, optimize};
use nalgebra::{SVector, vector};
use sim_truss::{Load, Strut, Support, Truss, Truss2};

/// A statically determinate symmetric two-bar truss: free node `P = (0,0)` hangs
/// from fixed supports `A = (−1,1)` and `B = (1,1)` under a downward load
/// `(0, −√2)`. Equilibrium at `P` gives `N_A = N_B = F_y/√2 = 1` (area- and
/// modulus-independent). With `E_A = 1`, `E_B = 4`, `w = 1` the closed-form optima
/// are `A_A* = |N|/√(E_A·w) = 1` and `A_B* = |N|/√(E_B·w) = 0.5` — distinct, so a
/// per-strut recovery is genuinely tested.
fn determinate_two_bar() -> LatticeTarget<2> {
    let truss = Truss2::new(
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
            force: vector![0.0, -std::f64::consts::SQRT_2],
        }],
    )
    .unwrap();
    LatticeTarget::new(truss, 1.0)
}

/// The determinate two-bar plus a **horizontal** strut `P → D = (2,0)`. With `E_A =
/// E_B` the geometry is symmetric and the load vertical, so `P` moves only
/// vertically — the horizontal strut has zero elongation, hence zero force, for any
/// area. Its optimal area is therefore zero: it is the redundant strut that must
/// collapse, while the two inclined struts keep their determinate optima (`A* = 1`
/// at `E = 1`).
fn two_bar_with_zero_force_strut() -> LatticeTarget<2> {
    let truss = Truss2::new(
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
            force: vector![0.0, -std::f64::consts::SQRT_2],
        }],
    )
    .unwrap();
    LatticeTarget::new(truss, 1.0)
}

#[test]
fn recovers_closed_form_optimum() {
    let target = determinate_two_bar();
    let result = optimize(&target, &target.x0(1.0), &target.recommended_config());
    let areas = target.to_physical(&result.params);

    // A_A* = 1, A_B* = 0.5 (distinct, from |N|/√(Ew) with N = 1, E = {1, 4}, w = 1).
    assert!(
        (areas[0] - 1.0).abs() < 5e-3,
        "strut A area {} should recover to 1.0",
        areas[0]
    );
    assert!(
        (areas[1] - 0.5).abs() < 5e-3,
        "strut B area {} should recover to 0.5",
        areas[1]
    );

    // The run improved on the uniform start and reached near-stationarity.
    let (start_loss, _) = target.evaluate(&target.x0(1.0));
    assert!(
        result.loss < start_loss,
        "optimization must reduce the objective"
    );
    assert!(
        result.final_grad_inf < 1e-6,
        "expected a near-stationary stop, ‖grad‖∞ = {}",
        result.final_grad_inf
    );
}

#[test]
fn gradient_matches_central_fd() {
    let target = determinate_two_bar();
    // Off-optimum, distinct areas so the two struts have independent gradients.
    let p = [1.3_f64.ln(), 0.7_f64.ln()];
    let (_, grad) = target.evaluate(&p);

    let eps = 1e-7;
    for k in 0..target.n_params() {
        let mut plus = p;
        let mut minus = p;
        plus[k] += eps;
        minus[k] -= eps;
        let fd =
            (target.objective(&plus).unwrap() - target.objective(&minus).unwrap()) / (2.0 * eps);
        assert!(
            (grad[k] - fd).abs() <= 1e-6 * fd.abs().max(1.0),
            "strut {k}: analytic {} vs central FD {}",
            grad[k],
            fd
        );
    }
}

#[test]
fn zero_force_strut_collapses_toward_zero() {
    let target = two_bar_with_zero_force_strut();
    let result = optimize(&target, &target.x0(1.0), &target.recommended_config());
    let areas = target.to_physical(&result.params);

    // The two load-bearing struts hold their finite optimum (A* = 1 at E = 1)…
    for (k, a) in areas[..2].iter().enumerate() {
        assert!(
            (0.8..1.25).contains(a),
            "load-bearing strut {k} area {a} should stay near 1.0"
        );
    }
    // …while the zero-force strut collapses far below them, yet stays strictly
    // positive (log-space can only approach zero, never reach it).
    assert!(
        areas[2] < 1e-3,
        "zero-force strut area {} should collapse toward zero",
        areas[2]
    );
    assert!(
        areas[2] > 0.0,
        "log-space keeps every area strictly positive"
    );
}

#[test]
fn collapse_is_not_certifiable_by_gradient_norm() {
    // The rung-C seed: pin that the log-space gradient CANNOT tell a converged
    // strut from a still-collapsing one. Run a FIXED-length descent (grad_tol
    // disabled) so the redundant strut is sampled while it is still shrinking.
    let target = two_bar_with_zero_force_strut();
    let cfg = OptConfig {
        lr: 0.1,
        max_iters: 6000,
        grad_tol: 0.0, // disabled → run the full budget, do not stop on the norm
        ..OptConfig::default()
    };
    // Start with the two load-bearing struts already at their optimum (A* = 1 at
    // E = 1), so the *only* thing still moving is the redundant strut — and the
    // whole gradient norm is therefore its collapsing component.
    let result = optimize(&target, &target.x0(1.0), &cfg);
    assert_eq!(result.stop_reason, StopReason::MaxIters);

    // The zero-force strut's area at 1/4, 1/2, and the end of the run.
    let a_d = |it: usize| result.history[it].params[2].exp();
    let (quarter, half, last) = (a_d(1500), a_d(3000), a_d(5999));

    // It keeps decreasing by >2× each horizon — collapse is approached, never
    // reached (strictly positive throughout): the strut is demonstrably NOT
    // converged at the end of the run.
    assert!(
        last > 0.0 && last < half / 2.0 && half < quarter / 2.0,
        "redundant area should more than halve each horizon: {quarter} → {half} → {last}"
    );

    // Yet the gradient norm — the exact quantity a `grad_tol` stop tests — has
    // already fallen below a plausible tolerance, because ∂J/∂p_D = A_D·w·L_D
    // vanishes as the area shrinks. So a `grad_tol` at or above this value reports a
    // `GradTol` stop while the strut is still shrinking — the log-norm cannot
    // distinguish "converged" from "still collapsing."
    //
    // The metric-aware stopping criterion this seeded (rung C) was *investigated and
    // found unnecessary*: here the distinction is immaterial (this zero-force strut is
    // collapsing outward, `∂J/∂A > 0`, and a strut at 1e-6 vs 1e-20 is the same
    // structure — the KKT-valid case proved by `converged_lattice_satisfies_kkt`). The
    // log-norm is a sound certificate for determinate structures and reliable on
    // uniform-start indeterminate runs; where it is *not* universal is measured
    // separately (`log_norm_hides_a_kkt_violation_on_indeterminate_structures`,
    // `indeterminate_uniform_start_strands_no_grower`).
    assert!(
        result.final_grad_inf < 1e-4,
        "‖grad‖∞ = {} — the norm has collapsed with the redundant area",
        result.final_grad_inf
    );
}

#[test]
fn interior_optimum_fires_gradtol() {
    // The clean half of rung C's conclusion: for a design whose optimum is
    // interior (every strut load-bearing), the log-norm stop FIRES a `GradTol` stop
    // at a genuine first-order-optimal point. The determinate two-bar has no
    // collapsing strut, so its log-norm falls all the way to `grad_tol`.
    let target = determinate_two_bar();
    let result = optimize(&target, &target.x0(1.0), &target.recommended_config());
    assert_eq!(result.stop_reason, StopReason::GradTol);
    // Both struts are stationary in PHYSICAL space (∂J/∂A ≈ 0), i.e. KKT-interior.
    let areas = target.to_physical(&result.params);
    let (_, grad_p) = target.evaluate(&result.params);
    for (&a, &gp) in areas.iter().zip(&grad_p) {
        assert!((gp / a).abs() < 1e-6, "interior strut not stationary");
    }
}

#[test]
fn converged_lattice_satisfies_kkt() {
    // At this converged design the log-norm stop lands on a first-order-optimal
    // (KKT) point: every strut is either interior-stationary (∂J/∂A ≈ 0) or
    // collapsed to the A ≥ 0 boundary with an OUTWARD gradient (A small, ∂J/∂A > 0).
    // Here the redundant strut is *exactly* zero-force by symmetry, so its force is
    // determinate — the case where a small log-norm genuinely certifies the KKT
    // structure. (The general indeterminate case does NOT: see
    // `log_norm_hides_a_kkt_violation_on_indeterminate_structures`.) The run stops on
    // MaxIters — the collapsing strut's log-norm decays too slowly to trip a tight
    // `grad_tol` — but the DESIGN is optimal, which is what the crate's gates assert.
    let target = two_bar_with_zero_force_strut();
    let result = optimize(&target, &target.x0(1.0), &target.recommended_config());
    let areas = target.to_physical(&result.params);
    let (_, grad_p) = target.evaluate(&result.params);
    // ∂J/∂Aₑ = (∂J/∂pₑ) / Aₑ  (undo the log-chain factor).
    for (e, (&a, &gp)) in areas.iter().zip(&grad_p).enumerate() {
        let g_phys = gp / a;
        let interior_stationary = g_phys.abs() < 1e-3;
        let collapsed_to_boundary = a < 1e-3 && g_phys > 0.0;
        assert!(
            interior_stationary || collapsed_to_boundary,
            "strut {e}: A={a:.3e}, ∂J/∂A={g_phys:.3e} violates KKT (neither stationary nor collapsed)"
        );
    }
    // Concretely: the two load-bearing struts are interior-stationary at A ≈ 1, the
    // zero-force strut has collapsed toward the boundary.
    assert!((areas[0] - 1.0).abs() < 1e-2 && (areas[1] - 1.0).abs() < 1e-2);
    assert!(areas[2] < 1e-3);
}

#[test]
fn log_norm_stationarity_is_scale_invariant_under_material_scaling() {
    // A reason the plain log-norm is a stable stopping quantity here: at a fixed
    // *relative* distance from the optimum, the log-space stationarity `Aₑ·∂J/∂Aₑ`
    // is invariant under the material scaling `E→kE, w→kw` (which preserves `w/E`).
    // Derivation: at A = 2·A* = 2|N|/√(Ew), the log-gradient works out to
    // (3/2)|N|L·√(w/E), and √(w/E) is unchanged by that scaling. So one `grad_tol`
    // transfers across such material scales without any per-problem tuning.
    let two_bar = |ke: f64, kw: f64| -> LatticeTarget<2> {
        let truss = Truss2::new(
            vec![vector![0.0, 0.0], vector![-1.0, 1.0], vector![1.0, 1.0]],
            vec![
                Strut {
                    i: 0,
                    j: 1,
                    youngs_modulus: ke,
                },
                Strut {
                    i: 0,
                    j: 2,
                    youngs_modulus: 4.0 * ke,
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
                force: vector![0.0, -std::f64::consts::SQRT_2],
            }],
        )
        .unwrap();
        LatticeTarget::new(truss, kw)
    };
    // ‖∂J/∂p‖∞ evaluated at areas = 2× the closed-form optimum A* = |N|/√(Ew), N = 1.
    let log_norm_at_2x_optimum = |ke: f64, kw: f64| -> f64 {
        let t = two_bar(ke, kw);
        let a_star = |e: f64| 1.0 / (e * kw).sqrt();
        let p = vec![(2.0 * a_star(ke)).ln(), (2.0 * a_star(4.0 * ke)).ln()];
        let (_, g) = t.evaluate(&p);
        g.iter().fold(0.0_f64, |m, &gi| m.max(gi.abs()))
    };
    let base = log_norm_at_2x_optimum(1.0, 1.0);
    for k in [10.0, 100.0, 0.01] {
        let scaled = log_norm_at_2x_optimum(k, k); // E→kE, w→kw (w/E preserved)
        assert!(
            (scaled - base).abs() < 1e-9 * base,
            "log-norm not invariant under E→kE,w→kw: base {base}, k={k} → {scaled}"
        );
    }
}

#[test]
fn loss_lower_bound_is_sound_zero() {
    // Mass-penalized compliance is a sum of non-negative terms, so 0 is a sound
    // (slack) lower bound — unlike the signed-reward conduit target.
    let target = determinate_two_bar();
    assert_eq!(target.loss_lower_bound(), Some(0.0));
    // The objective is indeed non-negative at an arbitrary design.
    assert!(target.objective(&[0.3_f64.ln(), 2.0_f64.ln()]).unwrap() >= 0.0);
}

// ── The indeterminate case: the log-norm's limit, and why it doesn't bite ──────
//
// For a statically DETERMINATE truss a thin load-bearing strut has a *large*
// log-gradient (force fixed ⇒ `∂C/∂A ~ −1/A²`), so a small log-norm cannot hide a
// live strut — the log-norm is a sound certificate. For an INDETERMINATE ground
// structure a redundant member's force vanishes with its own area, so `∂C/∂A` stays
// finite, and a member hand-set to tiny area can *want to grow* (`∂J/∂A < 0`) while
// its log-gradient `A·∂J/∂A` is negligible. The two gates below pin both halves:
// the log-norm is NOT a universal certificate, but uniform-start descent never
// reaches the point where that matters.

/// A statically indeterminate truss: free node `P=(0,0)` held by two 45° struts to
/// `(∓1,1)` plus a redundant vertical strut to `(0,1)`, under a downward load. The
/// vertical member is the most efficient for the load, so from any real start it
/// earns area rather than collapsing.
fn vertical_redundant() -> LatticeTarget<2> {
    let truss = Truss2::new(
        vec![
            vector![0.0, 0.0],
            vector![-1.0, 1.0],
            vector![1.0, 1.0],
            vector![0.0, 1.0],
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
            }, // vertical redundant
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
            force: vector![0.0, -1.0],
        }],
    )
    .unwrap();
    LatticeTarget::new(truss, 1.0)
}

/// A dense cantilever ground structure: an `nx × ny` grid of nodes, left column
/// pinned, a downward tip load, every node pair within `radius` joined by a strut —
/// a richly indeterminate "many-strut" problem.
fn cantilever_grid(nx: usize, ny: usize, radius: f64, w: f64) -> LatticeTarget<2> {
    let mut nodes: Vec<SVector<f64, 2>> = Vec::new();
    for ix in 0..nx {
        for iy in 0..ny {
            nodes.push(vector![ix as f64, iy as f64]);
        }
    }
    let idx = |ix: usize, iy: usize| ix * ny + iy;
    let mut struts = Vec::new();
    for a in 0..nodes.len() {
        for b in (a + 1)..nodes.len() {
            if (nodes[a] - nodes[b]).norm() <= radius {
                struts.push(Strut {
                    i: a,
                    j: b,
                    youngs_modulus: 1.0,
                });
            }
        }
    }
    let supports = (0..ny)
        .map(|iy| Support {
            node: idx(0, iy),
            fixed: [true, true],
        })
        .collect();
    let loads = vec![Load {
        node: idx(nx - 1, 0),
        force: vector![0.0, -1.0],
    }];
    LatticeTarget::new(Truss::<2>::new(nodes, struts, supports, loads).unwrap(), w)
}

#[test]
fn log_norm_hides_a_kkt_violation_on_indeterminate_structures() {
    // The committed limit: on an indeterminate structure the log-norm can miss a
    // KKT violation. Hand-place the two main struts near their 2-bar optimum
    // (A ≈ 0.707 = |N|/√(Ew), N = 1/√2) and the redundant vertical member at a tiny
    // area. The vertical member would slash compliance, so it wants to grow —
    // ∂J/∂A ≪ 0 — yet its log-gradient A·∂J/∂A is negligible and would pass a loose
    // `grad_tol`.
    let target = vertical_redundant();
    let p = vec![0.707_f64.ln(), 0.707_f64.ln(), 1e-8_f64.ln()];
    let areas = target.to_physical(&p);
    let (_, grad_p) = target.evaluate(&p);

    let g_phys_vertical = grad_p[2] / areas[2];
    // The member's own log-gradient component is negligible…
    assert!(
        grad_p[2].abs() < 1e-6,
        "vertical member's log-gradient {} should be hidden by its tiny area",
        grad_p[2]
    );
    // …while it strongly wants to grow (large negative physical gradient)…
    assert!(
        g_phys_vertical < -1.0,
        "vertical member should want to grow, ∂J/∂A = {g_phys_vertical}"
    );
    // …so its KKT residual πₑ = Aₑ − max(0, Aₑ − ∂J/∂Aₑ) is large: the log-norm is
    // NOT a universal certificate here.
    let pi = areas[2] - (areas[2] - g_phys_vertical).max(0.0);
    assert!(
        pi.abs() > 1.0,
        "KKT residual {pi} should expose the violation the log-norm hides"
    );
}

#[test]
fn indeterminate_uniform_start_strands_no_grower() {
    // Why the limit above does not bite in practice: from a UNIFORM start, descent
    // never leaves a member both small-area AND wanting to grow — a useful member
    // earns area, it is not stranded thin. Measured across the 3-bar indeterminate
    // truss (several starts) and a 53-strut grid. So the log-norm stop is reliable
    // on realistic runs even though it is not a universal certificate.
    let check = |target: &LatticeTarget<2>, a0: f64| {
        let cfg = OptConfig {
            lr: 0.1,
            max_iters: 8000,
            grad_tol: 1e-9,
            ..OptConfig::default()
        };
        let x0 = target.x0(a0);
        let result = optimize(target, &x0, &cfg);
        let areas = target.to_physical(&result.params);
        let (_, grad_p) = target.evaluate(&result.params);
        for (e, (&a, &gp)) in areas.iter().zip(&grad_p).enumerate() {
            let g_phys = gp / a;
            // No member is both thin and wanting to grow (the pathology). A member
            // may have a small negative gradient near convergence, but only at a
            // non-thin area — never the `small AND ∂J/∂A ≪ 0` combination.
            assert!(
                !(a < 1e-2 && g_phys < -1e-3),
                "member {e}: A={a:.3e}, ∂J/∂A={g_phys:.3e} is stranded thin wanting to grow"
            );
        }
    };
    for a0 in [0.1, 0.5, 1.0, 2.0] {
        check(&vertical_redundant(), a0);
    }
    // The useful vertical member is kept, not collapsed, from a uniform start.
    let vr = vertical_redundant();
    let res = optimize(
        &vr,
        &vr.x0(1.0),
        &OptConfig {
            lr: 0.1,
            max_iters: 8000,
            grad_tol: 1e-9,
            ..OptConfig::default()
        },
    );
    assert!(
        vr.to_physical(&res.params)[2] > 0.1,
        "the efficient vertical member should earn area from a uniform start"
    );
    // And at the many-strut scale.
    check(&cantilever_grid(4, 3, 2.3, 0.1), 0.5);
}
