//! Release-only runs on the 10k tube: K2's CI check (plan §15g, §16i) and the
//! power iteration's accuracy bar (plan §16e). Named in tests-release's
//! explicit list and in this crate's `coverage_skip_binaries`.

#![allow(clippy::unwrap_used)]

use sim_soft_explicit::cpu;
use sim_soft_explicit::executor::{Executor, Obstacle};
use sim_soft_explicit::fixtures::golden::THICK_TUBE;
use sim_soft_explicit::fixtures::tube::{Insertion, Mandrel, Mesh, Tube, TubeRun, Walls};
use sim_soft_explicit::stepping::StepperConfig;

const MU: f64 = 23.0e3;
const DENSITY: f64 = 1070.0;

#[test]
#[cfg_attr(debug_assertions, ignore = "release-only: a 14k-step tube run")]
fn k2_on_the_10k_tube_is_within_seven_percent() {
    // λ_a 1.1, ν 0.49, frictionless, loaded over 10 axial-shear periods
    // (the loading-time ladder's start, plan §15c).
    let run = TubeRun {
        mesh: Mesh::TenK,
        case: THICK_TUBE[0],
        mu: MU,
        density: DENSITY,
        insertion: Insertion::plan(10.0 * TubeRun::shear_period(MU, DENSITY)),
        window: 0.1,
        friction: 0.0,
    };
    let r = run
        .run(|model, obstacle| cpu::f32::CpuExecutor::new(model, obstacle).unwrap())
        .unwrap();
    eprintln!(
        "MARGIN K2 10k: raw {:+.4} gap-corrected {:+.4} (bar 0.07); gap {:e} m; {} steps",
        r.errors.raw, r.errors.gap_corrected, r.reading.gap, r.steps
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
    assert!(
        r.errors.gap_corrected.abs() <= 0.07,
        "K2 gap-corrected: {:+}",
        r.errors.gap_corrected
    );
}

fn tube_executor<E: Executor>(
    make: impl FnOnce(&sim_soft_explicit::ExplicitModel, &Obstacle) -> E,
) -> E {
    let tube = Tube::plan(Mesh::TenK);
    let run = TubeRun {
        mesh: Mesh::TenK,
        case: THICK_TUBE[0],
        mu: MU,
        density: DENSITY,
        insertion: Insertion::plan(1.0),
        window: 0.1,
        friction: 0.0,
    };
    let model = tube.model(run.material(), Walls::Free).unwrap();
    let obstacle = run
        .insertion
        .obstacle(Mandrel { radius: 0.011 }, &tube, 0.0005, 0.0, 0.5);
    make(&model, &obstacle)
}

#[test]
#[cfg_attr(debug_assertions, ignore = "release-only: a converged power iteration")]
fn the_power_iteration_is_accurate_on_the_10k_tube() {
    // The reference: f64, iterated until the quotient stops moving.
    let mut reference = tube_executor(|m, o| cpu::f64::CpuExecutor::new(m, o).unwrap());
    let perturbation = reference.epsilon().sqrt() * reference.shortest_edge();
    let mut converged = reference.elastic_rayleigh_quotient(4000, perturbation);
    let further = reference.elastic_rayleigh_quotient(2000, perturbation);
    let drift = further / converged - 1.0;
    eprintln!("MARGIN reference drift over 2000 more iterations: {drift:e}");
    assert!(
        drift.abs() <= 1e-4,
        "the reference has not converged: {drift:e}"
    );
    converged = converged.max(further);
    let iterations = StepperConfig::new(0.5, 0.0, 0.0).first_iterations;
    for (label, estimate) in [
        ("f64", {
            let mut e = tube_executor(|m, o| cpu::f64::CpuExecutor::new(m, o).unwrap());
            let p = e.epsilon().sqrt() * e.shortest_edge();
            e.elastic_rayleigh_quotient(iterations, p)
        }),
        ("f32", {
            let mut e = tube_executor(|m, o| cpu::f32::CpuExecutor::new(m, o).unwrap());
            let p = e.epsilon().sqrt() * e.shortest_edge();
            e.elastic_rayleigh_quotient(iterations, p)
        }),
    ] {
        let error = estimate / converged - 1.0;
        eprintln!(
            "MARGIN power iteration on the 10k tube ({label}, {iterations} iterations): {error:+e} (bar 0.05)"
        );
        assert!(error.abs() <= 0.05, "{label}: {error:+e}");
    }
}
