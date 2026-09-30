//! The gates on what is particular to the GPU executor (recon §17b).

#![cfg(test)]

use bytemuck::Zeroable;
use sim_soft_explicit::cpu;
use sim_soft_explicit::executor::{Executor, Monitors, PhaseOutputs, Snapshot};
use sim_soft_explicit::stepping::{RunError, Stepper, StepperConfig};
use wgpu::util::DeviceExt;

use super::super::kernels::binding::{LOG_ROWS, PARTIALS, REDUCTION, TERMS};
use super::super::kernels::{Dispatch, Kernel, Kernels, Shared};
use super::super::{
    Constants, GpuExecutor, MAX, Maker, RING_SLOTS, Reduction, SUM, StepValues, TREE, bytes,
    record_pass,
};
use super::fixtures::{self, Fixture, SILICONE, block_model, nowhere, turning_floor};
use super::{context, monitor_bits, outputs_bits, snapshot_bits};
use crate::context::GpuContext;
use crate::submit::Recorder;

/// One step of `f`'s fixed `dt` from time `time`.
fn step(e: &mut dyn Executor, f: &Fixture, time: f64) {
    e.element_dilations();
    e.gather_volume_changes();
    e.nodal_pressures();
    e.element_forces();
    e.gather_forces();
    e.contact(time, f.dt, f.damping);
    e.integrate(f.dt, f.damping);
    e.boundary_conditions(f.dt, f.damping);
}

/// A GPU executor on `f`, in its state.
fn gpu(ctx: &GpuContext, f: &Fixture) -> GpuExecutor {
    let mut gpu = GpuExecutor::new(ctx, &f.model, &f.obstacle).unwrap();
    gpu.set_state(
        f.time,
        &f.displacements,
        &f.velocities,
        f.anchors.as_deref(),
    );
    gpu
}

/// ★ A new pose track set between two steps leaves the obstacle's work as it
/// was with a read before it as without one: each step's motion is kept when
/// its contact phase runs, not formed when its row is read. The same steps on
/// the old track do other work, so the new track is taken.
#[test]
fn a_new_pose_track_between_steps_leaves_the_work_as_a_read_before_it_would() {
    let Some(ctx) = context() else { return };
    let f = fixtures::moving_and_turning();
    let turned = turning_floor(3.0);
    let work = |read_before: bool, new_track: bool| {
        let mut gpu = gpu(&ctx, &f);
        let mut time = f.time;
        for _ in 0..5 {
            step(&mut gpu, &f, time);
            time += f.dt;
        }
        if read_before {
            gpu.monitors();
        }
        if new_track {
            gpu.set_poses(turned.start, turned.interval, &turned.poses)
                .unwrap();
        }
        for _ in 0..5 {
            step(&mut gpu, &f, time);
            time += f.dt;
        }
        gpu.monitors().obstacle_work
    };
    let (with_read, without) = (work(true, true), work(false, true));
    assert!(with_read != 0.0, "the obstacle does work");
    assert!(
        with_read != work(false, false),
        "the new track changed nothing: {with_read:e}"
    );
    assert_eq!(
        with_read.to_bits(),
        without.to_bits(),
        "{with_read:e} vs {without:e}"
    );
}

/// ★ With elements inverted well past `J = 0`, the phase outputs and the
/// count read after the monitors and after an estimate are those read
/// before: the internal energy's read and the estimate run on arrays of
/// their own, and count nothing.
#[test]
fn a_read_and_an_estimate_leave_the_steps_outputs_and_count_alone() {
    let Some(ctx) = context() else { return };
    let mut f = fixtures::two_materials();
    // A corner node pushed through the block, turning its elements inside out.
    f.displacements[0] = [0.015, 0.015, 0.015];
    let mut gpu = gpu(&ctx, &f);
    step(&mut gpu, &f, f.time);
    let before = gpu.phase_outputs();
    let inverted = before.dilations.iter().filter(|&&d| d <= -1.0).count() as u64;
    let deepest = before.dilations.iter().fold(0.0_f64, |m, &d| m.min(d));
    assert!(
        inverted > 0 && deepest < -1.5,
        "{inverted} inverted, J − 1 down to {deepest}"
    );
    let counted = gpu.monitors().inverted_element_steps;
    let after_read = gpu.phase_outputs();
    gpu.estimate_top_mode(100, 1e-6, 0.0);
    let after_estimate = gpu.phase_outputs();
    assert_eq!(
        counted, inverted,
        "one step counts its inverted elements once"
    );
    assert_eq!(gpu.monitors().inverted_element_steps, inverted);
    assert!(
        after_read == before,
        "the monitors' read changed the phase outputs"
    );
    assert!(
        after_estimate == before,
        "the estimate changed the phase outputs"
    );
}

/// A run of `steps` steps of `f` with a window open, as one pass a step or
/// pass by pass; what it reads at the end.
fn run(
    ctx: &GpuContext,
    f: &Fixture,
    pass_per_phase: bool,
    steps: u32,
) -> (Snapshot, PhaseOutputs, Monitors) {
    let mut gpu = gpu(ctx, f);
    gpu.pass_per_phase = pass_per_phase;
    gpu.clear_accumulators();
    let mut time = f.time;
    for _ in 0..steps {
        step(&mut gpu, f, time);
        gpu.accumulate();
        time += f.dt;
    }
    (gpu.snapshot(), gpu.phase_outputs(), gpu.monitors())
}

/// ★ Steps recorded one pass a step are byte for byte the same steps recorded
/// pass by pass.
#[test]
fn one_pass_a_step_is_the_same_as_a_pass_a_phase() {
    let Some(ctx) = context() else { return };
    let f = fixtures::tube();
    let (one, each) = (run(&ctx, &f, false, 10), run(&ctx, &f, true, 10));
    assert!(
        snapshot_bits(&one.0) == snapshot_bits(&each.0),
        "the snapshots differ"
    );
    assert!(
        outputs_bits(&one.1) == outputs_bits(&each.1),
        "the phase outputs differ"
    );
    assert!(
        monitor_bits(&one.2) == monitor_bits(&each.2),
        "the monitors differ: {:?} vs {:?}",
        one.2,
        each.2
    );
}

/// The estimate's arguments as the stepping loop passes them.
fn perturbation(e: &dyn Executor) -> f64 {
    e.epsilon().sqrt() * e.shortest_edge()
}

/// ★ The estimate's `ω²` and damping quotient within 1e-3 of the CPU at
/// f32's, with one read an estimate; the CPU at f32's own distance from f64
/// printed beside.
#[test]
fn the_estimate_follows_the_cpu_with_one_read() {
    let Some(ctx) = context() else { return };
    for f in [
        fixtures::tube(),
        fixtures::viscous(),
        fixtures::stabilized(),
        fixtures::constrained(),
    ] {
        let mut cpu32 = cpu::f32::CpuExecutor::new(&f.model, &f.obstacle).unwrap();
        let mut cpu64 = cpu::f64::CpuExecutor::new(&f.model, &f.obstacle).unwrap();
        let mut gpu = gpu(&ctx, &f);
        for e in [&mut cpu32 as &mut dyn Executor, &mut cpu64] {
            e.set_state(
                f.time,
                &f.displacements,
                &f.velocities,
                f.anchors.as_deref(),
            );
        }
        // At the weight the loop's first estimate gives, as it takes it.
        let first = cpu32.estimate_top_mode(100, perturbation(&cpu32), 0.0);
        let weight = 2.0 / StepperConfig::new(0.0).stable_step(first.omega_squared, 0.0);
        let estimate = |e: &mut dyn Executor| {
            let perturbation = perturbation(e);
            e.estimate_top_mode(100, perturbation, weight)
        };
        let (c32, c64) = (estimate(&mut cpu32), estimate(&mut cpu64));
        let reads = gpu.recorder.reads();
        let g = estimate(&mut gpu);
        assert_eq!(
            gpu.recorder.reads() - reads,
            1,
            "{}: one read an estimate",
            f.name
        );
        let relative = |a: f64, b: f64| {
            if a == b {
                0.0
            } else {
                (a - b).abs() / a.abs().max(b.abs())
            }
        };
        println!(
            "{}: ω² GPU {:.1e} from the CPU at f32, which is {:.1e} from f64; damping quotient \
             GPU {:.1e}, CPU at f32 {:.1e}",
            f.name,
            relative(g.omega_squared, c32.omega_squared),
            relative(c32.omega_squared, c64.omega_squared),
            relative(g.damping_quotient, c32.damping_quotient),
            relative(c32.damping_quotient, c64.damping_quotient),
        );
        assert!(
            relative(g.omega_squared, c32.omega_squared) <= 1e-3,
            "{}: {g:?} vs {c32:?}",
            f.name
        );
        assert!(
            relative(g.damping_quotient, c32.damping_quotient) <= 1e-3,
            "{}: {g:?} vs {c32:?}",
            f.name
        );
    }
}

/// ★ A start vector of zeros breaks the power iteration at its first
/// iteration, and `ω²` is 0 as the CPU leaves it, not 0/0.
#[test]
fn a_break_at_the_first_iteration_leaves_omega_squared_at_zero() {
    let Some(ctx) = context() else { return };
    let model = block_model((2, 2, 1), |_| SILICONE, |_| true);
    let mut cpu = cpu::f32::CpuExecutor::new(&model, &nowhere()).unwrap();
    let mut gpu = GpuExecutor::new(&ctx, &model, &nowhere()).unwrap();
    let (c, g) = (
        cpu.estimate_top_mode(100, 1e-6, 0.0),
        gpu.estimate_top_mode(100, 1e-6, 0.0),
    );
    assert_eq!(c.omega_squared, 0.0);
    assert_eq!(
        g.omega_squared.to_bits(),
        c.omega_squared.to_bits(),
        "{g:?}"
    );
}

/// ★ On Metal, three runs of 1 000 tube steps through the stepping loop
/// repeat bit for bit: no float is added by an atomic, and every sum is a
/// fixed tree. A run that stops is compared as far as it went, by bits, so a
/// `NaN` compares by its payload.
#[test]
fn three_runs_on_metal_repeat_bit_for_bit() {
    let Some(ctx) = context() else { return };
    if ctx.adapter_info.backend != wgpu::Backend::Metal {
        println!("  the repeat gate runs on Metal (recon §17b)");
        return;
    }
    let f = fixtures::tube();
    let runs: Vec<(Result<(), RunError>, Vec<u64>)> = (0..3)
        .map(|_| {
            let mut stepper = Stepper::new(gpu(&ctx, &f), StepperConfig::new(f.damping), f.time);
            let stopped = (0..1000).try_for_each(|_| stepper.step());
            let snapshot = stepper.executor_mut().snapshot();
            let bits = snapshot_bits(&snapshot)
                .into_iter()
                .chain(
                    stepper
                        .samples()
                        .iter()
                        .flat_map(|s| monitor_bits(&s.monitors)),
                )
                .collect();
            (stopped, bits)
        })
        .collect();
    for run in &runs[1..] {
        assert_eq!(run.0, runs[0].0, "the runs stopped differently");
        assert!(run.1 == runs[0].1, "the runs differ");
    }
    assert!(runs[0].0.is_ok(), "the runs stopped: {:?}", runs[0].0);
}

/// ★ With no window open, a run read every step and the same run read every
/// 100, its log growing between reads, give the same cumulative monitors and
/// final state, bit for bit.
#[test]
fn reading_every_step_or_every_hundred_gives_the_same_run() {
    let Some(ctx) = context() else { return };
    let f = fixtures::tube();
    let run = |monitor_every: u64| {
        let config = StepperConfig {
            monitor_every,
            ..StepperConfig::new(f.damping)
        };
        let mut stepper = Stepper::new(gpu(&ctx, &f), config, f.time);
        for _ in 0..300 {
            stepper.step().unwrap();
        }
        let grown = stepper.executor().contact_rows.capacity;
        let last = stepper.samples().last().unwrap().monitors;
        (stepper.executor_mut().snapshot(), last, grown)
    };
    let (every, hundred) = (run(1), run(100));
    assert!(hundred.2 > super::super::LOG_ROWS_AT_START, "the log grew");
    assert!(
        snapshot_bits(&every.0) == snapshot_bits(&hundred.0),
        "the state differs"
    );
    let cumulative = |m: &Monitors| {
        [
            m.contact_work,
            m.obstacle_work,
            m.damping_loss,
            m.max_penetration,
            m.deepest_prediction,
        ]
        .map(f64::to_bits)
    };
    assert_eq!(cumulative(&every.1), cumulative(&hundred.1));
    assert_eq!(every.1.coarse_corrections, hundred.1.coarse_corrections);
    assert_eq!(
        every.1.inverted_element_steps,
        hundred.1.inverted_element_steps
    );
}

/// ★ A still state's window sums over 1 000 steps, read every 100, are each
/// within 1e-5 of 1 000 times the state; the state's f32 sum over all 1 000
/// steps is not, so a sum kept on the device across reads would miss.
#[test]
fn a_still_states_window_sums_hold_over_a_thousand_steps() {
    let Some(ctx) = context() else { return };
    let model = block_model((2, 2, 1), |_| SILICONE, |_| false);
    // A rigid translation: no strain, no force, no motion. Each component
    // one whose f32 sum over 1 000 steps drifts 1.5e-5 from 1 000 times it,
    // about the most a constant addend drifts.
    let shift = [3.847_241_314e-3, -1.884_043_12e-3, 4.350_066_139e-3];
    let shifted = vec![shift; model.node_count()];
    let mut gpu = GpuExecutor::new(&ctx, &model, &nowhere()).unwrap();
    gpu.set_state(0.0, &shifted, &vec![[0.0; 3]; model.node_count()], None);
    let mut stepper = Stepper::new(gpu, StepperConfig::new(0.0), 0.0);
    stepper.open_window();
    for _ in 0..1000 {
        stepper.step().unwrap();
    }
    stepper.close_window();
    let snapshot = stepper.executor_mut().snapshot();
    assert_eq!(snapshot.accumulated_steps, 1000);
    let state = shift.map(|x| f64::from(x as f32));
    let f32_sum = shift.map(|x| {
        let mut sum = 0.0_f32;
        for _ in 0..1000 {
            sum += x as f32;
        }
        f64::from(sum)
    });
    let missed =
        (0..3).any(|d| (f32_sum[d] - 1000.0 * state[d]).abs() > 1e-5 * (1000.0 * state[d]).abs());
    assert!(
        missed,
        "the f32 sum over 1 000 steps holds 1e-5 too: the gate cannot fail"
    );
    for sums in &snapshot.displacement_sums {
        for d in 0..3 {
            let expected = 1000.0 * state[d];
            assert!(
                (sums[d] - expected).abs() <= 1e-5 * expected.abs(),
                "{} vs {expected}",
                sums[d]
            );
        }
    }
}

/// ★ A `NaN` in one node's velocity, read before any step, makes the monitors
/// not finite, and the loop stops, on the GPU as on the CPU.
#[test]
fn a_nan_stops_the_loop() {
    let Some(ctx) = context() else { return };
    let f = fixtures::two_materials();
    let mut velocities = f.velocities.clone();
    velocities[7][1] = f64::NAN;
    stops(
        cpu::f32::CpuExecutor::new(&f.model, &f.obstacle).unwrap(),
        &f,
        &velocities,
    );
    stops(
        GpuExecutor::new(&ctx, &f.model, &f.obstacle).unwrap(),
        &f,
        &velocities,
    );
}

/// `e` in `f`'s state with `velocities`: its monitors not finite before any
/// step, and the loop stopped.
fn stops(mut e: impl Executor, f: &Fixture, velocities: &[[f64; 3]]) {
    e.set_state(f.time, &f.displacements, velocities, None);
    assert!(!e.monitors().finite(), "the read before any step");
    let mut stepper = Stepper::new(e, StepperConfig::new(f.damping), f.time);
    let stopped = stepper.run_until(f.time + 1.0);
    assert!(
        matches!(stopped, Err(RunError::NonFinite { .. })),
        "{stopped:?}"
    );
}

/// ★ A count set just below 2³² and counted past it reads the exact total:
/// the low word carries into the high one, and the other count is untouched.
#[test]
fn a_count_carries_past_two_to_the_thirty_second() {
    let Some(ctx) = context() else { return };
    let model = block_model((3, 2, 2), |_| SILICONE, |_| false);
    let mut gpu = GpuExecutor::new(&ctx, &model, &nowhere()).unwrap();
    // Each node sent through the origin to its mirror image: every element
    // turned inside out.
    let mirrored: Vec<[f64; 3]> = model
        .rest_positions()
        .iter()
        .map(|p| p.map(|x| -2.0 * x))
        .collect();
    gpu.set_state(0.0, &mirrored, &vec![[0.0; 3]; model.node_count()], None);
    let (inverted, coarse) = ((1_u64 << 32) - 10, (1_u64 << 32) - 1);
    gpu.set_counts(inverted, coarse);
    gpu.element_dilations();
    let monitors = gpu.monitors();
    let elements = model.element_count() as u64;
    assert_eq!(monitors.inverted_element_steps, inverted + elements);
    assert_eq!(monitors.coarse_corrections, coarse);
}

/// ★ A reduction over more blocks than `reduce_row` takes in one row of
/// partials (256), and over more workgroups than a dispatch's dimension
/// holds (65 535), reaches its row with every partial: the sum and the
/// largest of terms placed in the first block and the last.
#[test]
fn a_reduction_past_256_blocks_and_65_535_workgroups_takes_every_partial() {
    let Some(ctx) = context() else { return };
    let kernels = Kernels::new(&ctx);
    let mut recorder = Recorder::<StepValues>::with_step_values(&ctx, RING_SLOTS);
    for blocks in [300, 65_537] {
        let mut values = vec![0.0_f32; (blocks * TREE) as usize];
        values[0] = 1.0;
        values[(blocks * TREE) as usize - 1] = 2.0;
        assert_eq!(
            sum_and_largest(&ctx, &kernels, &mut recorder, &values),
            [3.0, 2.0],
            "{blocks} blocks: the sum, the largest"
        );
    }
}

/// The sum and the largest of `values`, each reduced into a row of its own,
/// in one pass.
fn sum_and_largest(
    ctx: &GpuContext,
    kernels: &Kernels,
    recorder: &mut Recorder<StepValues>,
    values: &[f32],
) -> Vec<f32> {
    let maker = Maker {
        device: &ctx.device,
        limit: u64::MAX,
    };
    let items = values.len() as u32;
    let constants = ctx
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("constants"),
            contents: bytemuck::bytes_of(&Constants::zeroed()),
            usage: wgpu::BufferUsages::UNIFORM,
        });
    let terms = maker.filled("terms", values).unwrap();
    let partials = maker
        .zeroed("partials", bytes::<f32>(items.div_ceil(TREE)))
        .unwrap();
    let rows = maker.zeroed("rows", bytes::<f32>(2)).unwrap();
    let dispatches: Vec<Dispatch> = {
        let shared = Shared {
            constants: &constants,
            step_values: recorder.step_values_binding().unwrap(),
        };
        [(SUM, 0), (MAX, 1)]
            .into_iter()
            .flat_map(|(operation, row)| {
                let reduction = maker.reduction(
                    "reduction",
                    Reduction {
                        items,
                        components: 1,
                        operation,
                        row,
                    },
                );
                [
                    kernels.bind(
                        &ctx.device,
                        &shared,
                        Kernel::ReducePartials,
                        items,
                        &[
                            (REDUCTION, &reduction),
                            (TERMS, &terms),
                            (PARTIALS, &partials),
                        ],
                    ),
                    kernels.bind(
                        &ctx.device,
                        &shared,
                        Kernel::ReduceRow,
                        1,
                        &[
                            (REDUCTION, &reduction),
                            (PARTIALS, &partials),
                            (LOG_ROWS, &rows),
                        ],
                    ),
                ]
            })
            .collect()
    };
    record_pass(recorder, kernels, &StepValues::zeroed(), &dispatches);
    recorder.read(&rows, 2)
}
