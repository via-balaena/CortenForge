//! Release-only runs on the 10k tube: K2's CI check (plan §15g, §16i) and the
//! accuracy of the stable step the loop takes, at the power iteration's count,
//! at rest and loaded (plan §16e, §16p). Named in tests-release's explicit list and in this crate's
//! `coverage_skip_binaries`.

#![allow(clippy::unwrap_used)]

use sim_soft_explicit::cpu;
use sim_soft_explicit::executor::{Executor, Obstacle, Snapshot};
use sim_soft_explicit::fixtures::golden::THICK_TUBE;
use sim_soft_explicit::fixtures::tube::{
    ECOFLEX_00_30_VISCOUS_TIME, Insertion, Mandrel, Mesh, Tube, TubeRun, Walls,
};
use sim_soft_explicit::stepping::{Stepper, StepperConfig};
use sim_soft_explicit::{ExplicitModel, f64::Material};

const MU: f64 = 23.0e3;
const DENSITY: f64 = 1070.0;

/// K2's worst corner on the 10k tube: `λ_a` 1.1, ν 0.495 (+5.29 %; plan
/// §16i moves this check to the worst 10k corner 2b finds), frictionless,
/// loaded over 10 axial-shear periods (the loading-time ladder's start, plan
/// §15c).
fn k2_run() -> TubeRun {
    TubeRun {
        mesh: Mesh::TenK,
        case: THICK_TUBE[1],
        mu: MU,
        viscous_time: ECOFLEX_00_30_VISCOUS_TIME,
        density: DENSITY,
        insertion: Insertion::plan(10.0 * TubeRun::shear_period(MU, DENSITY)),
        window: 0.1,
        friction: 0.0,
        grid_cell: 0.0005,
    }
}

/// The tube's model and its mandrel as an obstacle, for fresh executors.
fn tube_and_mandrel(material: Material) -> (ExplicitModel, Obstacle) {
    let tube = Tube::plan(Mesh::TenK);
    let model = tube.model(material, Walls::Free).unwrap();
    let obstacle = k2_run()
        .insertion
        .obstacle(Mandrel { radius: 0.011 }, &tube, 0.0005, 0.0)
        .unwrap();
    (model, obstacle)
}

/// The loop's stable step at a state, against the critical step it
/// estimates, converged: returns the f32 and f64 loops' steps over 0.9 of it,
/// less 1. The bar is 2 %: as §16e's 5 % on `ω²` was, about a fifth of the
/// 11 % margin 0.9 leaves on the step (plan §16p).
///
/// The loop estimates at `β = 2/Δt`: the elastic top mode first, then again at
/// its step (`Stepper::new`). The reference is the fixed point of the same
/// estimate at 6000 iterations in f64, `β` updated to `2/Δt_c` until the step
/// moves by at most 1e-4, with 4000 iterations agreeing to 1e-4.
fn step_errors(state: &Snapshot, time: f64, label: &str) -> (f64, f64) {
    let (model, obstacle) = tube_and_mandrel(k2_run().material());
    let place = |e: &mut dyn Executor| {
        e.set_state(time, &state.displacements, &state.velocities, None);
        e.epsilon().sqrt() * e.shortest_edge()
    };
    let mut reference = cpu::f64::CpuExecutor::new(&model, &obstacle).unwrap();
    let p = place(&mut reference);
    // Central differences' limit with the lagging damping force, written here
    // rather than taken from `StepperConfig::stable_step`, which it checks.
    let limit = |e: &mut cpu::f64::CpuExecutor, iterations: usize, weight: f64| {
        let top = e.estimate_top_mode(iterations, p, weight);
        let xi = top.damping_ratio();
        2.0 / top.omega_squared.sqrt() * ((1.0 + xi * xi).sqrt() - xi)
    };
    let mut step = limit(&mut reference, 6000, 0.0);
    for _ in 0..8 {
        let next = limit(&mut reference, 6000, 2.0 / step);
        let moved = next / step - 1.0;
        step = next;
        if moved.abs() <= 1e-4 {
            break;
        }
    }
    let drift = limit(&mut reference, 4000, 2.0 / step) / step - 1.0;
    assert!(
        drift.abs() <= 1e-4,
        "{label}: the reference has not converged: {drift:e}"
    );
    let mut wide = cpu::f64::CpuExecutor::new(&model, &obstacle).unwrap();
    place(&mut wide);
    let wide_error = Stepper::new(wide, StepperConfig::new(0.0), time).dt() / (0.9 * step) - 1.0;
    let mut narrow = cpu::f32::CpuExecutor::new(&model, &obstacle).unwrap();
    place(&mut narrow);
    let narrow_error =
        Stepper::new(narrow, StepperConfig::new(0.0), time).dt() / (0.9 * step) - 1.0;
    eprintln!(
        "MARGIN the loop's step on the 10k tube, {label}, over 0.9 of the converged critical step: f64 {wide_error:+e}, f32 {narrow_error:+e} (bar 0.02; reference drift {drift:e})"
    );
    (wide_error, narrow_error)
}

#[test]
#[cfg_attr(debug_assertions, ignore = "release-only: a 13k-step tube run")]
fn k2_on_the_10k_tube_is_within_seven_percent() {
    let run = k2_run();
    let r = run
        .run(|model, obstacle| cpu::f32::CpuExecutor::new(model, obstacle).unwrap())
        .unwrap();
    let corrected = r.errors.gap_corrected.unwrap();
    eprintln!(
        "MARGIN K2 10k: raw {:+.4} gap-corrected {corrected:+.4} (bar 0.07); gap {:e} m; deepest penetration {:e} m; {} steps",
        r.errors.raw, r.reading.gap, r.max_penetration, r.steps
    );
    eprintln!(
        "MARGIN validity: KE/IE {:?} (bar 0.05), lambda_z {:+e} (bar 0.005), energy balance {:?} (bar 0.01)",
        r.kinetic_over_internal, r.axial_stretch_error, r.energy_balance
    );
    assert!(!r.inverted, "K4: an element reached J <= 0");
    assert!(r.kinetic_over_internal.unwrap() <= 0.05, "invalid: KE/IE");
    assert!(r.axial_stretch_error.abs() <= 0.005, "invalid: lambda_z");
    assert!(r.energy_balance.unwrap() <= 0.01, "invalid: energy balance");
    assert!(r.errors.raw.abs() <= 0.07, "K2 raw: {:+}", r.errors.raw);
    assert!(corrected.abs() <= 0.07, "K2 gap-corrected: {corrected:+}");
    // G2: no node deeper than 1 % of the inset (the interference) at any step.
    let inset = (run.case.mandrel_ratio - 1.0) * Tube::plan(Mesh::TenK).inner_radius;
    assert!(
        r.max_penetration <= 0.01 * inset,
        "G2: {:e} m",
        r.max_penetration
    );

    // Check the step a fresh start makes at the loaded state, where a
    // warm-started estimate once read 3.9 % low. In the run, the estimate also
    // depends on the step in use (β = 2/Δt); the review measured the in-run
    // step at +0.12 % here (plan §16p).
    let end = run.insertion.end();
    let (wide, narrow) = step_errors(&r.snapshot, end, "loaded, at K2's end");
    assert!(wide.abs() <= 0.02 && narrow.abs() <= 0.02);
}

#[test]
#[cfg_attr(debug_assertions, ignore = "release-only: a converged power iteration")]
fn the_loops_step_is_accurate_on_the_10k_tube_at_rest() {
    let nodes = Tube::plan(Mesh::TenK).node_count();
    let rest = Snapshot {
        displacements: vec![[0.0; 3]; nodes],
        velocities: vec![[0.0; 3]; nodes],
        ..Snapshot::default()
    };
    let (wide, narrow) = step_errors(&rest, 0.0, "at rest");
    assert!(wide.abs() <= 0.02 && narrow.abs() <= 0.02);
}
