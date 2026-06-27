//! Feasibility-aware optimization ([`OptConfig::reject_infeasible`]) — robustness for fail-closed
//! forward models (the soft solver PANICS, not `Err`s, on a torn/diverged step).
//!
//! A synthetic 1-D quadratic whose `evaluate` PANICS outside a feasible region `x ≤ feasible_max`
//! stands in for the stiff-contact co-design (where an Adam step can land on a parameter point whose
//! coupled rollout tears the buffer and panics). The gates:
//! 1. with `reject_infeasible = true` the optimizer SURVIVES the panic, stays feasible, and pushes
//!    to the constrained optimum (the feasible boundary);
//! 2. with `reject_infeasible = false` the same run PANICS — the robustness is load-bearing;
//! 3. when the optimum is interior (no panic), `reject_infeasible` leaves the result byte-identical.

use cf_codesign::{CoDesignProblem, OptConfig, optimize};

/// `½(x − target)²` with a fail-closed feasible region: `evaluate` panics for `x > feasible_max`
/// (mimicking the soft solver's panic on a torn step).
struct FailClosedQuadratic {
    target: f64,
    feasible_max: f64,
}

impl CoDesignProblem for FailClosedQuadratic {
    fn n_params(&self) -> usize {
        1
    }
    fn evaluate(&self, p: &[f64]) -> (f64, Vec<f64>) {
        let x = p[0];
        assert!(
            x <= self.feasible_max,
            "infeasible: x = {x} > feasible_max {}",
            self.feasible_max
        );
        let r = x - self.target;
        (0.5 * r * r, vec![r])
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
