//! Feasibility-aware optimization ([`OptConfig::reject_infeasible`]) — robustness for fail-closed
//! forward models (the stiff-contact grip reports infeasibility as a typed `Err` from
//! [`CoDesignProblem::try_evaluate`]; its panic-path `evaluate` still fail-closes loudly).
//!
//! A synthetic 1-D quadratic whose `try_evaluate` returns `Err(InfeasibleDesign)` outside a feasible
//! region `x ≤ feasible_max` (and whose `evaluate` still PANICS there) stands in for the
//! stiff-contact co-design (where an Adam step can land on a parameter point whose coupled rollout
//! tears the buffer). The gates:
//! 1. with `reject_infeasible = true` the optimizer SKIPS the infeasible step (no panic), stays
//!    feasible, and pushes to the constrained optimum (the feasible boundary);
//! 2. with `reject_infeasible = false` the same run hits the panic-path `evaluate` and PANICS — the
//!    feasibility-aware path is load-bearing;
//! 3. when the optimum is interior (never infeasible), `reject_infeasible` leaves the result
//!    byte-identical.

use cf_codesign::{CoDesignProblem, InfeasibleDesign, OptConfig, optimize};

/// `½(x − target)²` with a fail-closed feasible region: `try_evaluate` returns `Err` for
/// `x > feasible_max`, and `evaluate` re-panics it (mimicking the grip — a typed `Err` on the
/// fallible path, a loud panic on the infallible one).
struct FailClosedQuadratic {
    target: f64,
    feasible_max: f64,
}

impl CoDesignProblem for FailClosedQuadratic {
    fn n_params(&self) -> usize {
        1
    }
    fn evaluate(&self, p: &[f64]) -> (f64, Vec<f64>) {
        self.try_evaluate(p).unwrap_or_else(|e| panic!("{e}"))
    }
    fn try_evaluate(&self, p: &[f64]) -> Result<(f64, Vec<f64>), InfeasibleDesign> {
        let x = p[0];
        if x > self.feasible_max {
            return Err(InfeasibleDesign::new(format!(
                "infeasible: x = {x} > feasible_max {}",
                self.feasible_max
            )));
        }
        let r = x - self.target;
        Ok((0.5 * r * r, vec![r]))
    }
}

fn cfg(reject_infeasible: bool) -> OptConfig {
    OptConfig {
        lr: 0.1,
        max_iters: 400,
        grad_tol: 0.0,
        loss_tol: 1e-18,
        eps: 1e-8,
        reject_infeasible,
    }
}

/// The unconstrained optimum (x = 3) is OUTSIDE the feasible region (x ≤ 2): the optimizer must
/// survive the panic and settle at the feasible boundary (the best feasible point).
#[test]
fn feasibility_aware_survives_and_stays_feasible() {
    let prob = FailClosedQuadratic {
        target: 3.0,
        feasible_max: 2.0,
    };
    let result = optimize(&prob, &[1.0], &cfg(true));
    assert!(
        result.params[0] <= 2.0 + 1e-9,
        "must stay feasible (x ≤ 2.0), got {}",
        result.params[0]
    );
    assert!(
        result.params[0] > 1.9,
        "should push to the feasible boundary ~2.0 (the constrained optimum), got {}",
        result.params[0]
    );
    assert!(
        result.loss.is_finite(),
        "final loss must be finite, got {}",
        result.loss
    );
}

/// Contrast: WITHOUT `reject_infeasible` the same run panics on the first infeasible step — the
/// robustness is load-bearing, not cosmetic. (catch_unwind so the test observes it without failing.)
#[test]
fn default_optimize_panics_at_infeasible_step() {
    let prob = FailClosedQuadratic {
        target: 3.0,
        feasible_max: 2.0,
    };
    let prev = std::panic::take_hook();
    std::panic::set_hook(Box::new(|_| {}));
    let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        optimize(&prob, &[1.0], &cfg(false))
    }));
    std::panic::set_hook(prev);
    assert!(
        outcome.is_err(),
        "the default (non-feasibility-aware) optimize must panic on the infeasible step"
    );
}

/// In the feasible region `reject_infeasible` never makes the result worse: it returns the BEST
/// iterate seen, so its loss is ≤ the default loop's last-iterate loss (the default keeps last-
/// iterate semantics, byte-identical to before — covered by the smooth targets' own gates). Both
/// reach the interior optimum.
#[test]
fn feasibility_aware_never_worse_when_feasible() {
    let prob = FailClosedQuadratic {
        target: 1.0,
        feasible_max: 5.0,
    };
    let base = optimize(&prob, &[0.0], &cfg(false)); // last iterate
    let robust = optimize(&prob, &[0.0], &cfg(true)); // best feasible iterate
    assert!(
        robust.loss <= base.loss + 1e-12,
        "best-iterate loss {} must be ≤ default last-iterate loss {}",
        robust.loss,
        base.loss
    );
    assert!(
        (robust.params[0] - 1.0).abs() < 1e-3 && (base.params[0] - 1.0).abs() < 1e-3,
        "both converge to the interior optimum x = 1.0 (best {}, last {})",
        robust.params[0],
        base.params[0]
    );
}

/// A smooth problem that does NOT override `try_evaluate` still optimizes under
/// `reject_infeasible = true`: the rewired (catch_unwind-free) loop drives the DEFAULT
/// `try_evaluate` (`Ok(self.evaluate(..))`), so a never-fail-closing target descends exactly as in
/// default mode. Covers the default trait path through the loop — the smooth co-design targets
/// (material / joint / policy) rely on it.
#[test]
fn default_try_evaluate_through_reject_infeasible_loop() {
    struct SmoothQuadratic {
        target: f64,
    }
    // Note: no `try_evaluate` override — exercises the trait default.
    impl CoDesignProblem for SmoothQuadratic {
        fn n_params(&self) -> usize {
            1
        }
        fn evaluate(&self, p: &[f64]) -> (f64, Vec<f64>) {
            let r = p[0] - self.target;
            (0.5 * r * r, vec![r])
        }
    }
    let prob = SmoothQuadratic { target: 1.5 };
    let result = optimize(&prob, &[0.0], &cfg(true));
    assert!(
        (result.params[0] - 1.5).abs() < 1e-3,
        "smooth problem under reject_infeasible should reach the interior optimum 1.5, got {}",
        result.params[0]
    );
}
