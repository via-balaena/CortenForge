//! Per-phase conformance (recon §17b): on every fixture, the GPU at f32
//! against the CPU executor at f32, the phases run in order up to one read,
//! each output held to its bar. The CPU at f64 runs beside them: phase 6's bar
//! is twice the CPU at f32's distance from it, and every output's is printed.

#![cfg(test)]

use sim_soft_explicit::cpu;
use sim_soft_explicit::executor::{Executor, Monitors, PhaseOutputs, Snapshot, rigid_motion};
use sim_soft_explicit::f64 as shared;

use super::super::GpuExecutor;
use super::fixtures::{self, Fixture, Margins};
use super::{context, relative_difference};

/// What one step of a fixture gives an executor.
struct Step {
    /// The state set, as the executor holds it.
    before: Snapshot,
    /// Phases 1–6's outputs after phases 1–8.
    outputs: PhaseOutputs,
    /// The state after phases 1–8.
    after: Snapshot,
    /// The monitors read after phases 1–8.
    monitors: Monitors,
    /// The state after phases 1–7 alone, from the same start.
    integrated: Snapshot,
}

fn set(e: &mut dyn Executor, f: &Fixture) {
    e.set_state(
        f.time,
        &f.displacements,
        &f.velocities,
        f.anchors.as_deref(),
    );
}

fn phases(e: &mut dyn Executor, f: &Fixture, last: usize) {
    let steps: [&dyn Fn(&mut dyn Executor); 8] = [
        &|e| e.element_dilations(),
        &|e| e.gather_volume_changes(),
        &|e| e.nodal_pressures(),
        &|e| e.element_forces(),
        &|e| e.gather_forces(),
        &|e| e.contact(f.time, f.dt, f.damping),
        &|e| e.integrate(f.dt, f.damping),
        &|e| e.boundary_conditions(f.dt, f.damping),
    ];
    for phase in &steps[..last] {
        phase(e);
    }
}

/// One step of `f` on `e`, and phases 1–7 again from the same start.
fn step(e: &mut dyn Executor, f: &Fixture) -> Step {
    set(e, f);
    let before = e.snapshot();
    phases(e, f, 8);
    let outputs = e.phase_outputs();
    let after = e.snapshot();
    let monitors = e.monitors();
    set(e, f);
    phases(e, f, 7);
    Step {
        before,
        outputs,
        after,
        monitors,
        integrated: e.snapshot(),
    }
}

fn vectors(v: &[[f64; 3]]) -> impl Iterator<Item = &f64> {
    v.iter().flatten()
}

/// Each array output, by name, from a step.
fn arrays(s: &Step) -> Vec<(&'static str, Vec<f64>)> {
    let o = &s.outputs;
    let flat = |v: &[[f64; 3]]| vectors(v).copied().collect::<Vec<f64>>();
    vec![
        ("1 dilations", o.dilations.clone()),
        ("2 volume changes", o.volume_changes.clone()),
        ("3 pressures", o.pressures.clone()),
        (
            "4 element forces",
            o.element_forces.iter().flatten().copied().collect(),
        ),
        (
            "4 element viscous forces",
            o.element_viscous_forces.iter().flatten().copied().collect(),
        ),
        ("5 elastic forces", flat(&o.elastic_forces)),
        ("5 viscous forces", flat(&o.viscous_forces)),
        ("6 contact forces", flat(&o.contact_forces)),
        ("6 normal forces", o.normal_forces.clone()),
        ("6 anchors", flat(&s.after.anchors)),
        ("7 displacements", flat(&s.integrated.displacements)),
        ("7 velocities", flat(&s.integrated.velocities)),
        ("8 displacements", flat(&s.after.displacements)),
        ("8 velocities", flat(&s.after.velocities)),
    ]
}

/// Each sum the monitors read after one step, and the summed magnitudes of
/// its terms, from the CPU's step.
fn sums(f: &Fixture, s: &Step, m: &Monitors) -> Vec<(&'static str, f64, f64)> {
    let model = &f.model;
    let o = &s.outputs;
    let centroid = model
        .rest_positions()
        .iter()
        .fold([0.0; 3], |sum, &p| shared::vec3_add(sum, p))
        .map(|x| x / model.node_count() as f64);
    let (from, to) = (
        f.obstacle.pose_at(f.time),
        f.obstacle.pose_at(f.time + f.dt),
    );
    let origin = [from.tx, from.ty, from.tz];
    let (moved, turn) = rigid_motion(from, to);
    let force: f64 = o
        .contact_forces
        .iter()
        .map(|&x| shared::vec3_length(x))
        .sum();
    let arm: f64 = (0..model.node_count())
        .map(|a| {
            let x = shared::vec3_add(model.rest_positions()[a], s.before.displacements[a]);
            shared::vec3_length(shared::vec3_sub(x, centroid))
                * shared::vec3_length(o.contact_forces[a])
        })
        .sum();
    let moment = arm + shared::vec3_length(shared::vec3_sub(centroid, origin)) * force;
    let (mut work, mut loss) = (0.0, 0.0);
    for a in 0..model.node_count() {
        let (before, after) = (s.before.velocities[a], s.after.velocities[a]);
        work += shared::step_work(o.contact_forces[a], before, after, f.dt).abs();
        loss += shared::damping_loss(model.node_masses()[a], f.damping, before, after, f.dt).abs()
            + shared::step_work(o.viscous_forces[a], before, after, f.dt).abs();
    }
    let obstacle_work = shared::vec3_length(moved) * force + shared::vec3_length(turn) * moment;
    let mut sums = vec![
        ("kinetic energy", m.kinetic_energy, m.kinetic_energy),
        ("internal energy", m.internal_energy, m.internal_energy),
        (
            "contact kinetic energy",
            m.contact_kinetic_energy,
            m.contact_kinetic_energy,
        ),
        (
            "normal force",
            m.normal_force,
            o.normal_forces.iter().map(|n| n.abs()).sum(),
        ),
        ("contact work", m.contact_work, work),
        ("damping loss", m.damping_loss, loss),
        ("obstacle work", m.obstacle_work, obstacle_work),
    ];
    for d in 0..3 {
        sums.push(("contact force", m.contact_force[d], force));
        sums.push(("contact moment", m.contact_moment[d], moment));
    }
    sums
}

/// Run `f` on the CPU at f32 and f64 and on the GPU; assert its margins and
/// every output against its bar; return the lines of its record.
fn conform(f: &Fixture) -> Vec<String> {
    let ctx = context().expect("an adapter, checked by the caller");
    let mut cpu32 = cpu::f32::CpuExecutor::new(&f.model, &f.obstacle).unwrap();
    let mut cpu64 = cpu::f64::CpuExecutor::new(&f.model, &f.obstacle).unwrap();
    let mut gpu = GpuExecutor::new(&ctx, &f.model, &f.obstacle).unwrap();
    let (c32, c64, g) = (step(&mut cpu32, f), step(&mut cpu64, f), step(&mut gpu, f));
    let margins: Margins = fixtures::margins(f, &c32.before, &c32.outputs);
    let mut record = vec![format!(
        "{}: {} nodes, {} in contact, {} slipping; margins {margins:?}",
        f.name,
        f.model.node_count(),
        margins.in_contact,
        margins.slipping
    )];
    margins.assert_clear(f.name);
    let mut failures = Vec::new();
    for ((name, a32), ((_, a64), (_, ag))) in arrays(&c32)
        .into_iter()
        .zip(arrays(&c64).into_iter().zip(arrays(&g)))
    {
        let gpu_from_cpu = relative_difference(&a32, &ag);
        let cpu_from_f64 = relative_difference(&a32, &a64);
        let bar = if name.starts_with("6 contact") || name.starts_with("6 normal") {
            1e-5_f64.max(2.0 * cpu_from_f64)
        } else {
            1e-5
        };
        record.push(format!(
            "  {name:<26} GPU {gpu_from_cpu:.1e}  CPU f32 from f64 {cpu_from_f64:.1e}  bar {bar:.1e}"
        ));
        if gpu_from_cpu > bar {
            failures.push(format!("{name}: {gpu_from_cpu:e} over {bar:e}"));
        }
    }
    for ((name, value, magnitude), (_, gpu_value, _)) in sums(f, &c32, &c32.monitors)
        .into_iter()
        .zip(sums(f, &c32, &g.monitors))
    {
        let difference = (gpu_value - value).abs();
        let relative = if magnitude > 0.0 {
            difference / magnitude
        } else {
            difference
        };
        record.push(format!(
            "  {name:<26} GPU {relative:.1e} of its terms' magnitudes"
        ));
        if difference > 1e-5 * magnitude {
            failures.push(format!("{name}: {difference:e} of {magnitude:e}"));
        }
    }
    let (m32, mg) = (&c32.monitors, &g.monitors);
    for (name, a, b) in [
        ("max penetration", m32.max_penetration, mg.max_penetration),
        (
            "deepest prediction",
            m32.deepest_prediction,
            mg.deepest_prediction,
        ),
    ] {
        record.push(format!("  {name:<26} CPU {a:.3e} GPU {b:.3e}"));
        if (a - b).abs() > 1e-5 * a.abs() {
            failures.push(format!("{name}: {a:e} vs {b:e}"));
        }
    }
    for (name, a, b) in [
        (
            "inverted elements",
            m32.inverted_element_steps,
            mg.inverted_element_steps,
        ),
        (
            "coarse corrections",
            m32.coarse_corrections,
            mg.coarse_corrections,
        ),
        ("steps", m32.steps, mg.steps),
    ] {
        record.push(format!("  {name:<26} CPU {a} GPU {b}"));
        if a != b {
            failures.push(format!("{name}: {a} vs {b}"));
        }
    }
    assert!(
        failures.is_empty(),
        "{}: {failures:#?}\n{}",
        f.name,
        record.join("\n")
    );
    record
}

/// ★ Every fixture's every phase output and monitor, GPU against the CPU at
/// f32, within its bar (recon §17b); each fixture clear of its margins.
#[test]
fn every_phase_matches_the_cpu_at_f32_on_every_fixture() {
    if context().is_none() {
        return;
    }
    for fixture in fixtures::all() {
        for line in conform(&fixture) {
            println!("{line}");
        }
    }
}
