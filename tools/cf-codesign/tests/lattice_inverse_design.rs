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
//! optimum, the analytic gradient against finite differences, that a genuine
//! zero-force strut collapses toward zero, and — the seed for the deferred
//! metric-aware stop — that the collapse is *not* certifiable by gradient norm.

use cf_codesign::{CoDesignProblem, LatticeTarget, OptConfig, StopReason, optimize};
use nalgebra::vector;
use sim_truss::{Load, Strut, Support, Truss2};

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
    // vanishes as the area shrinks. So a `grad_tol` anywhere at or above this value
    // would report a `GradTol` "converged" stop on a lattice whose strut is still
    // marching to zero. Pinning this is the whole motivation for the deferred
    // metric-aware stopping criterion (rung C); it is a property of the log-space
    // parametrization, not a bug to fix here.
    assert!(
        result.final_grad_inf < 1e-4,
        "‖grad‖∞ = {} — a grad_tol ≥ this would falsely certify the ongoing collapse",
        result.final_grad_inf
    );
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
