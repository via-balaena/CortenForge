//! Release-only runs on the 10k tube: K2's CI check (plan §15g, §16i) and the
//! power iteration's accuracy at the count the loop uses, at rest and loaded
//! (plan §16e). Named in tests-release's explicit list and in this crate's
//! `coverage_skip_binaries`.

#![allow(clippy::unwrap_used)]

use sim_soft_explicit::cpu;
use sim_soft_explicit::executor::{Executor, Obstacle, Snapshot};
use sim_soft_explicit::fixtures::golden::THICK_TUBE;
use sim_soft_explicit::fixtures::tube::{Insertion, Mandrel, Mesh, Tube, TubeRun, Walls};
use sim_soft_explicit::stepping::StepperConfig;
use sim_soft_explicit::{ExplicitModel, f64::Material};

const MU: f64 = 23.0e3;
const DENSITY: f64 = 1070.0;

/// K2's first corner: `λ_a` 1.1, ν 0.49, frictionless, loaded over 10
/// axial-shear periods (the loading-time ladder's start, plan §15c).
fn k2_run() -> TubeRun {
    TubeRun {
        mesh: Mesh::TenK,
        case: THICK_TUBE[0],
        mu: MU,
        c2: 0.0,
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

/// The loop's estimate of `ω_el²` at a state, and a converged one: f64 from
/// the same fixed start, run until another 2000 iterations move it by at most
/// 1e-4. Returns the f32 and f64 estimates' errors.
fn power_iteration_errors(state: &Snapshot, time: f64, label: &str) -> (f64, f64) {
    let (model, obstacle) = tube_and_mandrel(k2_run().material());
    let place = |e: &mut dyn Executor| {
        e.set_state(time, &state.displacements, &state.velocities, None);
        e.epsilon().sqrt() * e.shortest_edge()
    };
    let mut reference = cpu::f64::CpuExecutor::new(&model, &obstacle).unwrap();
    let p = place(&mut reference);
    let converged = reference.elastic_rayleigh_quotient(4000, p);
    let further = reference.elastic_rayleigh_quotient(6000, p);
    let drift = further / converged - 1.0;
    assert!(
        drift.abs() <= 1e-4,
        "{label}: the reference has not converged: {drift:e}"
    );
    let iterations = StepperConfig::new(0.0).power_iterations;
    let mut wide = cpu::f64::CpuExecutor::new(&model, &obstacle).unwrap();
    let p = place(&mut wide);
    let wide_error = wide.elastic_rayleigh_quotient(iterations, p) / further - 1.0;
    let mut narrow = cpu::f32::CpuExecutor::new(&model, &obstacle).unwrap();
    let p = place(&mut narrow);
    let narrow_error = narrow.elastic_rayleigh_quotient(iterations, p) / further - 1.0;
    eprintln!(
        "MARGIN power iteration on the 10k tube, {label} ({iterations} iterations): f64 {wide_error:+e}, f32 {narrow_error:+e} (bar 0.05; reference drift {drift:e})"
    );
    (wide_error, narrow_error)
}

#[test]
#[cfg_attr(debug_assertions, ignore = "release-only: a 14k-step tube run")]
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

    // Each of the loop's estimates depends only on the state (tests/executor.rs);
    // check the one it would make here, loaded, where a warm-started estimate
    // once read 3.9 % low.
    let end = run.insertion.end();
    let (wide, narrow) = power_iteration_errors(&r.snapshot, end, "loaded, at K2's end");
    assert!(wide.abs() <= 0.05 && narrow.abs() <= 0.05);
}

#[test]
#[cfg_attr(debug_assertions, ignore = "release-only: a converged power iteration")]
fn the_power_iteration_is_accurate_on_the_10k_tube_at_rest() {
    let nodes = Tube::plan(Mesh::TenK).node_count();
    let rest = Snapshot {
        displacements: vec![[0.0; 3]; nodes],
        velocities: vec![[0.0; 3]; nodes],
        ..Snapshot::default()
    };
    let (wide, narrow) = power_iteration_errors(&rest, 0.0, "at rest");
    assert!(wide.abs() <= 0.05 && narrow.abs() <= 0.05);
}
