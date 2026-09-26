//! K6's CI check (plan §16b, §16i): the Cattaneo–Mindlin block at `a/h` 12,
//! in f64, judged at 0.1 rather than K6's 0.03. It must fail under the two
//! anchor mutations of §16b (an anchor that never releases, and one not
//! dragged while slipping); at this mesh it cannot see a friction limit 10 %
//! off. And the measurement that makes K6 elastic: the silicone's viscosity
//! would shrink its step (§16q). Release-only: named in this crate's
//! `coverage_skip_binaries`, and run by tests-release, which names the crate.

#![allow(clippy::unwrap_used)]

use sim_soft_explicit::cpu;
use sim_soft_explicit::fixtures::partial_slip::{Leg, PartialSlipRun};
use sim_soft_explicit::fixtures::tube::ECOFLEX_00_30_VISCOUS_TIME;
use sim_soft_explicit::stepping::{Stepper, StepperConfig};

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
    // Readings in the band's lower half: the leg's end and its hold alone
    // put about half the readings near 0.8.
    let judged = |leg: Leg| {
        result
            .readings
            .iter()
            .filter(|r| r.leg == leg)
            .filter(|r| result.fraction(r).is_some_and(|f| (0.2..=0.5).contains(&f)))
            .count()
    };
    eprintln!(
        "MARGIN coarse K6: loading {:?}, unloading {:?} (bar 0.1), with {} and {} readings \
         at fractions 0.2–0.5; energy balance {:?}",
        errors.loading,
        errors.unloading,
        judged(Leg::Push),
        judged(Leg::Return),
        result.energy_balance,
    );
    assert_eq!(result.unfinished, None, "a leg did not reach its end");
    assert!(judged(Leg::Push) >= 20 && judged(Leg::Return) >= 20);
    let worst = errors.worst().unwrap();
    assert!(worst <= 0.1, "the stick zone is off by {worst} of a");
    assert!(result.energy_balance.unwrap() <= 0.01);
    assert!(!result.inverted);
}

/// Why K6 runs elastic (plan §16q): Kelvin–Voigt's damping grows as `1/h²` on
/// a mesh, and with Ecoflex 00-30's `η/μ` the stable step at the start falls
/// by more than an order of magnitude on K6's blocks.
#[test]
#[cfg_attr(debug_assertions, ignore = "release-only: K6's finest block")]
fn the_silicones_viscosity_would_shrink_k6s_step_by_more_than_ten() {
    let start = |divisions: f64, viscous_time: f64| {
        let mut run = PartialSlipRun::plan(divisions);
        run.viscous_time = viscous_time;
        let model = run.block.model(run.material()).unwrap();
        let obstacle = run.obstacle().unwrap();
        let executor = cpu::f64::CpuExecutor::new(&model, &obstacle).unwrap();
        Stepper::new(executor, StepperConfig::new(0.0), 0.0).dt()
    };
    for divisions in [12.0, 50.0] {
        let (elastic, viscous) = (
            start(divisions, 0.0),
            start(divisions, ECOFLEX_00_30_VISCOUS_TIME),
        );
        eprintln!(
            "MARGIN K6's step at a/h {divisions}: {elastic:.3e} s elastic, {viscous:.3e} s with \
             Ecoflex 00-30's viscosity ({:.1}x smaller)",
            elastic / viscous
        );
        assert!(viscous < 0.1 * elastic);
    }
}
