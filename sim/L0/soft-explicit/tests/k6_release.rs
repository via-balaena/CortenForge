//! K6's CI check (plan §16b, §16i): the Cattaneo–Mindlin block at `a/h` 12,
//! in f64, judged at 0.1 rather than K6's 0.03. It must fail under the two
//! anchor mutations of §16b (an anchor that never releases, and one not
//! dragged while slipping); at this mesh it cannot see a friction limit 10 %
//! off. Release-only: named in this crate's `coverage_skip_binaries`, and run
//! by tests-release, which names the crate.

#![allow(clippy::unwrap_used)]

use sim_soft_explicit::cpu;
use sim_soft_explicit::fixtures::partial_slip::{Leg, PartialSlipRun};

#[test]
#[cfg_attr(
    debug_assertions,
    ignore = "release-only: a 60k-step run of K6's block"
)]
fn the_coarse_k6_holds_the_stick_zone_to_the_closed_forms() {
    let run = PartialSlipRun::plan(12.0);
    let result = run
        .run(|model, obstacle| cpu::f64::CpuExecutor::new(model, obstacle).unwrap())
        .unwrap();
    let errors = result.errors(0.2, 0.8);
    let judged = |leg: Leg| {
        result
            .samples
            .iter()
            .filter(|s| s.leg == leg)
            .filter(|s| result.fraction(s).is_some_and(|f| (0.2..=0.8).contains(&f)))
            .count()
    };
    eprintln!(
        "MARGIN coarse K6: loading {:?}, unloading {:?} (bar 0.1) over {} and {} samples; \
         energy balance {:?}",
        errors.loading,
        errors.unloading,
        judged(Leg::Push),
        judged(Leg::Return),
        result.energy_balance,
    );
    assert!(judged(Leg::Push) >= 50 && judged(Leg::Return) >= 50);
    let worst = errors.worst().unwrap();
    assert!(worst <= 0.1, "the stick zone is off by {worst} of a");
    assert!(result.energy_balance.unwrap() <= 0.01);
    assert!(!result.inverted);
}
