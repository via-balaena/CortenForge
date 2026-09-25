//! A short insertion on the 10k tube, run in every CI test job: finite
//! energies and no inverted element, with contact and friction. It makes no
//! accuracy claim (plan §15g, §16i); `tube_release` does.

#![allow(clippy::unwrap_used)]

use sim_soft_explicit::cpu;
use sim_soft_explicit::fixtures::golden::THICK_TUBE;
use sim_soft_explicit::fixtures::tube::{Insertion, Mesh, TubeRun};

#[test]
fn a_short_insertion_on_the_10k_tube_stays_finite_and_uninverted() {
    // The nose first touches the entry 6.42 mm in; this pushes it 2 mm past.
    let run = TubeRun {
        mesh: Mesh::TenK,
        case: THICK_TUBE[0],
        mu: 23.0e3,
        density: 1070.0,
        insertion: Insertion {
            start_gap: 0.0,
            depth: 0.0084,
            loading_time: 0.02,
            hold: 0.004,
        },
        window: 0.002,
        friction: 0.3,
    };
    let result = run
        .run(|model, obstacle| cpu::f32::CpuExecutor::new(model, obstacle).unwrap())
        .unwrap();
    let last = result.samples.last().unwrap().monitors;
    eprintln!(
        "MARGIN sanity: {} steps, internal energy {:e} J, max penetration {:e} m",
        result.steps, last.internal_energy, result.max_penetration
    );
    assert!(!result.inverted, "an element reached J <= 0");
    assert!(result.samples.iter().all(|s| {
        let m = s.monitors;
        m.kinetic_energy.is_finite() && m.internal_energy.is_finite() && m.contact_work.is_finite()
    }));
    assert!(
        last.internal_energy > 0.0 && result.max_penetration > 0.0,
        "the nose must touch"
    );
    // The band (20–60 mm) is past where this nose reaches, so it reads zero.
    assert!(result.reading.pressure.is_finite());
}
