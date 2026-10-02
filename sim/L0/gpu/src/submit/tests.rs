//! The recorder's gates (recon §17a): each step reads its own values, a write
//! waits for the steps before it, a submit at the cap completes, timed passes
//! come back one a pass (§17d), and what the recorder refuses, it refuses with
//! a message.
//!
//! The two collapse tests assert wgpu's own behaviour, the reason the ring and
//! the write rule exist: if either stops collapsing, the reason is gone.

#![allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]

use std::panic::{AssertUnwindSafe, catch_unwind};
use std::time::Instant;

use bytemuck::{Pod, Zeroable};

use super::{PASS_CAP, Recorder, Recording, STEP_PASS_CAP, read_buffer};
use crate::context::GpuContext;
use crate::test_support::{gpu_context_or_skip, within_a_minute};

const SUITE: &str = "recorder tests";

/// A cell no step has written.
const UNWRITTEN: u32 = u32::MAX;

/// What each step carries: the cell it writes, and the value it writes there.
#[repr(C)]
#[derive(Debug, Copy, Clone, Pod, Zeroable)]
struct Step {
    cell: u32,
    value: u32,
    pad: [u32; 2],
}

const fn step(k: u32) -> Step {
    Step {
        cell: k,
        value: 7 * k + 3,
        pad: [0; 2],
    }
}

/// Writes the step's value into the step's cell, or, with `from_source`, the
/// source buffer's first value.
const KERNEL: &str = "
struct Step { cell: u32, value: u32, pad0: u32, pad1: u32 }
@group(0) @binding(0) var<uniform> step: Step;
@group(0) @binding(1) var<storage, read_write> cells: array<u32>;
@group(0) @binding(2) var<storage, read> source: array<u32>;
@compute @workgroup_size(1) fn own_value() { cells[step.cell] = step.value; }
@compute @workgroup_size(1) fn from_source() { cells[step.cell] = source[0]; }
@compute @workgroup_size(1) fn count() { cells[0] = cells[0] + 1u; }
@compute @workgroup_size(64) fn spin(@builtin(local_invocation_index) i: u32) {
    var x = i;
    for (var k = 0u; k < step.value; k = k + 1u) { x = x * 1664525u + 1013904223u; }
    if (i == 0u) { cells[step.cell] = x; }
}
";

struct Kernel {
    own_value: wgpu::ComputePipeline,
    from_source: wgpu::ComputePipeline,
    count: wgpu::ComputePipeline,
    spin: wgpu::ComputePipeline,
    layout: wgpu::BindGroupLayout,
    cells: wgpu::Buffer,
    source: wgpu::Buffer,
}

impl Kernel {
    /// The kernel over `n` cells. `dynamic` binds the step values with a
    /// dynamic offset, as the recorder's ring does; without it the step values
    /// are one plain uniform, as a naive loop would use.
    fn new(ctx: &GpuContext, n: u32, dynamic: bool) -> Self {
        let storage = |binding, read_only| wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::COMPUTE,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Storage { read_only },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        };
        let mut values = Recorder::<Step>::step_values_layout_entry(0);
        if !dynamic {
            values.ty = wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: None,
            };
        }
        let layout = ctx
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("recorder_test"),
                entries: &[values, storage(1, false), storage(2, true)],
            });
        let pipeline_layout = ctx
            .device
            .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("recorder_test"),
                bind_group_layouts: &[&layout],
                push_constant_ranges: &[],
            });
        let module = ctx
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("recorder_test"),
                source: wgpu::ShaderSource::Wgsl(KERNEL.into()),
            });
        let entry = |name| {
            ctx.device
                .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                    label: Some(name),
                    layout: Some(&pipeline_layout),
                    module: &module,
                    entry_point: Some(name),
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                    cache: None,
                })
        };
        let storage_buffer = |label, bytes| {
            ctx.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: bytes,
                usage: wgpu::BufferUsages::STORAGE
                    | wgpu::BufferUsages::COPY_SRC
                    | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            })
        };
        Self {
            own_value: entry("own_value"),
            from_source: entry("from_source"),
            count: entry("count"),
            spin: entry("spin"),
            layout,
            cells: storage_buffer("cells", u64::from(n) * 4),
            source: storage_buffer("source", 16),
        }
    }

    fn bind_group(&self, ctx: &GpuContext, values: wgpu::BindingResource<'_>) -> wgpu::BindGroup {
        ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("recorder_test"),
            layout: &self.layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: values,
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: self.cells.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: self.source.as_entire_binding(),
                },
            ],
        })
    }
}

/// Record one step: begin it with `values`, dispatch `pipeline` once at the
/// step's offset, end it.
fn record_step(
    rec: &mut Recorder<Step>,
    group: &wgpu::BindGroup,
    pipeline: &wgpu::ComputePipeline,
    values: &Step,
) {
    let offset = rec.begin_step(values);
    {
        let mut pass = rec.pass("step");
        pass.set_pipeline(pipeline);
        pass.set_bind_group(0, group, &[offset]);
        pass.dispatch_workgroups(1, 1, 1);
    }
    rec.end_step();
}

/// ★★ Each step's values reach that step's passes, across submits the ring
/// forces (8 slots, 40 steps) and one a read forces mid-run. The cells start
/// unwritten, so a step that read another's values, or all steps reading the
/// last one's, leaves its own cell unwritten.
#[test]
fn each_step_reads_its_own_values() {
    const STEPS: u32 = 40;
    const READ_AFTER: u32 = 21;
    let Some(ctx) = gpu_context_or_skip(SUITE) else {
        return;
    };
    let kernel = Kernel::new(&ctx, STEPS, true);
    let mut rec = Recorder::<Step>::with_step_values(&ctx, 8);
    let group = kernel.bind_group(&ctx, rec.step_values_binding().unwrap());
    rec.write(
        &kernel.cells,
        0,
        bytemuck::cast_slice(&[UNWRITTEN; STEPS as usize]),
    );

    let mut read_mid_run = Vec::new();
    for k in 0..STEPS {
        record_step(&mut rec, &group, &kernel.own_value, &step(k));
        if k + 1 == READ_AFTER {
            read_mid_run = rec.read::<u32>(&kernel.cells, STEPS as usize);
        }
    }
    let cells = rec.read::<u32>(&kernel.cells, STEPS as usize);

    for k in 0..STEPS {
        assert_eq!(cells[k as usize], step(k).value, "step {k} after the run");
        let mid = if k < READ_AFTER {
            step(k).value
        } else {
            UNWRITTEN
        };
        assert_eq!(read_mid_run[k as usize], mid, "step {k} at the read");
    }
}

/// ★ What the ring prevents, asserted: each step's values written to one
/// uniform with `write_buffer` between encodes all land before the submit, so
/// every step reads the last step's values and only its cell is written.
#[test]
fn values_written_between_encodes_collapse_to_the_last() {
    const STEPS: u32 = 8;
    let Some(ctx) = gpu_context_or_skip(SUITE) else {
        return;
    };
    let kernel = Kernel::new(&ctx, STEPS, false);
    let uniform = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("one_slot"),
        size: std::mem::size_of::<Step>() as u64,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let group = kernel.bind_group(&ctx, uniform.as_entire_binding());
    ctx.queue.write_buffer(
        &kernel.cells,
        0,
        bytemuck::cast_slice(&[UNWRITTEN; STEPS as usize]),
    );
    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
    for k in 0..STEPS {
        ctx.queue
            .write_buffer(&uniform, 0, bytemuck::bytes_of(&step(k)));
        let mut pass = encoder.pass("naive_step");
        pass.set_pipeline(&kernel.own_value);
        pass.set_bind_group(0, &group, &[]);
        pass.dispatch_workgroups(1, 1, 1);
    }
    ctx.queue.submit([encoder.finish()]);
    let cells: Vec<u32> = read_buffer(&ctx, &kernel.cells, STEPS as usize);

    let last = STEPS - 1;
    for k in 0..last {
        assert_eq!(cells[k as usize], UNWRITTEN, "step {k} read its own values");
    }
    assert_eq!(cells[last as usize], step(last).value);
}

/// Record four steps that copy the source (1) into their cells, write 2 to
/// the source by `write`, then record four more.
fn steps_around_a_write(
    ctx: &GpuContext,
    write: impl FnOnce(&mut Recorder<Step>, &wgpu::Buffer),
) -> Vec<u32> {
    const STEPS: u32 = 8;
    const BEFORE: u32 = 4;
    let kernel = Kernel::new(ctx, STEPS, true);
    let mut rec = Recorder::<Step>::with_step_values(ctx, STEPS);
    let group = kernel.bind_group(ctx, rec.step_values_binding().unwrap());
    rec.write(&kernel.source, 0, bytemuck::bytes_of(&1_u32));
    let mut write = Some(write);
    for k in 0..STEPS {
        if k == BEFORE {
            write.take().unwrap()(&mut rec, &kernel.source);
        }
        record_step(&mut rec, &group, &kernel.from_source, &step(k));
    }
    rec.read::<u32>(&kernel.cells, STEPS as usize)
}

/// ★★ A write through the recorder is seen only by the steps after it: the
/// recorder submits the steps before it first.
#[test]
fn a_write_waits_for_the_steps_before_it() {
    let Some(ctx) = gpu_context_or_skip(SUITE) else {
        return;
    };
    let cells = steps_around_a_write(&ctx, |rec, source| {
        rec.write(source, 0, bytemuck::bytes_of(&2_u32));
    });
    assert_eq!(cells, [1, 1, 1, 1, 2, 2, 2, 2]);
}

/// ★ What the write rule prevents, asserted: a bare `write_buffer` while
/// steps are recorded but not submitted overtakes them all.
#[test]
fn a_bare_write_overtakes_the_steps_before_it() {
    let Some(ctx) = gpu_context_or_skip(SUITE) else {
        return;
    };
    let queue = ctx.queue.clone();
    let cells = steps_around_a_write(&ctx, |_, source| {
        queue.write_buffer(source, 0, bytemuck::bytes_of(&2_u32));
    });
    assert_eq!(cells, [2; 8]);
}

/// A plain uniform holding `step(0)`, for work recorded without step values.
fn one_step_uniform(ctx: &GpuContext, rec: &mut Recorder) -> wgpu::Buffer {
    let uniform = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("one_step"),
        size: std::mem::size_of::<Step>() as u64,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    rec.write(&uniform, 0, bytemuck::bytes_of(&step(0)));
    uniform
}

/// ★★ A submit of [`PASS_CAP`] passes, each after a clear, completes on this
/// backend. A clear after a pass opens one more Metal command buffer, so this
/// is the cap's worst case, about 3 072 of the Metal queue's 4 096.
#[test]
fn a_submit_at_the_cap_with_a_clear_between_passes_completes() {
    let Some(ctx) = gpu_context_or_skip(SUITE) else {
        return;
    };
    let cells = within_a_minute("a submit of PASS_CAP passes", move || {
        let kernel = Kernel::new(&ctx, 1, false);
        let mut rec = Recorder::new(&ctx);
        let uniform = one_step_uniform(&ctx, &mut rec);
        let group = kernel.bind_group(&ctx, uniform.as_entire_binding());
        for _ in 0..PASS_CAP {
            rec.commands().clear_buffer(&kernel.source, 0, None);
            let mut pass = rec.pass("at_the_cap");
            pass.set_pipeline(&kernel.own_value);
            pass.set_bind_group(0, &group, &[]);
            pass.dispatch_workgroups(1, 1, 1);
        }
        rec.read::<u32>(&kernel.cells, 1)
    });
    assert_eq!(cells, [step(0).value]);
}

/// ★★ A step that carries no values, and opens more passes than one command
/// buffer holds on Metal, completes: the recorder submits itself at
/// [`PASS_CAP`], inside the step. Every pass counts once.
#[test]
fn a_step_without_values_past_the_cap_submits_itself() {
    const PASSES: u32 = 2 * PASS_CAP + 100;
    let Some(ctx) = gpu_context_or_skip(SUITE) else {
        return;
    };
    let counted = within_a_minute("a step past the cap", move || {
        let kernel = Kernel::new(&ctx, 1, false);
        let mut rec = Recorder::new(&ctx);
        let uniform = one_step_uniform(&ctx, &mut rec);
        let group = kernel.bind_group(&ctx, uniform.as_entire_binding());
        rec.write(&kernel.cells, 0, bytemuck::bytes_of(&0_u32));
        rec.begin_step(&());
        for _ in 0..PASSES {
            let mut pass = rec.pass("count");
            pass.set_pipeline(&kernel.count);
            pass.set_bind_group(0, &group, &[]);
            pass.dispatch_workgroups(1, 1, 1);
        }
        rec.end_step();
        rec.read::<u32>(&kernel.cells, 1)
    });
    assert_eq!(counted, [PASSES]);
}

/// Record one step whose one pass, labelled `label`, spins `turns` times.
fn spin_step(
    rec: &mut Recorder<Step>,
    (kernel, group): (&Kernel, &wgpu::BindGroup),
    label: &str,
    turns: u32,
) {
    let offset = rec.begin_step(&Step {
        cell: 0,
        value: turns,
        pad: [0; 2],
    });
    {
        let mut pass = rec.pass(label);
        pass.set_pipeline(&kernel.spin);
        pass.set_bind_group(0, group, &[offset]);
        pass.dispatch_workgroups(1, 1, 1);
    }
    rec.end_step();
}

/// ★★ Timed passes (recon §17d) come back one a pass, under their labels: a
/// pass that spins 2²⁰ times takes over ten times one that spins once, and
/// the passes together took no longer on the GPU than the host took from the
/// first to the wait for them. A 4-slot ring submits every 4 steps and each
/// round reads twice; a read after the last round finds nothing new. The
/// host's submits are counted: two at the ring and one at each read, then
/// one and one. It cannot see a query set written again before its resolve
/// completed, nor when the resolves happen.
#[test]
fn timed_passes_come_back_one_a_pass_under_their_labels() {
    if gpu_context_or_skip(SUITE).is_none() {
        return;
    }
    let Ok(ctx) = GpuContext::with_timestamps() else {
        eprintln!("{SUITE}: SKIP the timed passes: this adapter cannot write timestamps");
        return;
    };
    let kernel = Kernel::new(&ctx, 1, true);
    let mut rec = Recorder::<Step>::with_step_values(&ctx, 4);
    let group = kernel.bind_group(&ctx, rec.step_values_binding().unwrap());
    rec.time_passes();
    assert_eq!(rec.submit_times(), super::SubmitTimes::default());
    for (pairs, submits) in [(8, 6), (4, 10)] {
        let started = Instant::now();
        for _ in 0..2 {
            for _ in 0..pairs / 2 {
                spin_step(&mut rec, (&kernel, &group), "once", 1);
                spin_step(&mut rec, (&kernel, &group), "spun", 1 << 20);
            }
            rec.read::<u32>(&kernel.cells, 1);
        }
        let waited = started.elapsed().as_secs_f64();
        let times = rec.pass_times();
        assert_eq!(times.keys().collect::<Vec<_>>(), ["once", "spun"]);
        let median = |label: &str| {
            let mut each = times[label].clone();
            assert_eq!(each.len(), pairs, "{label}");
            assert!(
                each.iter().all(|&t| t > 0.0 && t.is_finite()),
                "{label}: {each:?}"
            );
            each.sort_by(f64::total_cmp);
            each[pairs / 2]
        };
        assert!(median("spun") > 10.0 * median("once"), "{times:?}");
        let host = rec.submit_times();
        assert_eq!(host.submits, submits);
        assert!(host.finishing > 0.0 && host.submitting > 0.0, "{host:?}");
        let total: f64 = times.values().flatten().sum();
        assert!(
            total <= waited,
            "the passes took {total} s on the GPU, the host {waited} s"
        );
    }
    assert!(
        rec.pass_times().is_empty(),
        "a read after the last finds nothing new"
    );
}

/// ★ A recorder on a context without timestamps refuses to time passes.
#[test]
fn timing_passes_without_timestamps_stops_with_a_message() {
    let Some(ctx) = gpu_context_or_skip(SUITE) else {
        return;
    };
    let mut rec = Recorder::new(&ctx);
    let message = panic_message(|| rec.time_passes());
    assert!(message.contains("with_timestamps"), "{message}");
}

/// The message a closure panicked with.
fn panic_message(run: impl FnOnce()) -> String {
    let payload = catch_unwind(AssertUnwindSafe(run)).expect_err("it should have stopped");
    payload
        .downcast_ref::<String>()
        .cloned()
        .or_else(|| payload.downcast_ref::<&str>().map(|s| (*s).to_owned()))
        .unwrap_or_default()
}

/// ★ A step carrying values stops past [`STEP_PASS_CAP`] passes: its values
/// cannot split across submits.
#[test]
fn a_step_carrying_values_past_its_pass_cap_stops_with_a_message() {
    let Some(ctx) = gpu_context_or_skip(SUITE) else {
        return;
    };
    let mut rec = Recorder::<Step>::with_step_values(&ctx, 1);
    rec.begin_step(&step(0));
    let message = panic_message(|| {
        for _ in 0..=STEP_PASS_CAP {
            drop(rec.pass("past_the_step_cap"));
        }
    });
    assert!(message.contains("opened more than"), "{message}");
}

/// ★ Passes opened outside steps do not shrink the next step's budget: the
/// step begins with a submit once half the cap is pending.
#[test]
fn a_step_after_passes_outside_steps_keeps_its_budget() {
    let Some(ctx) = gpu_context_or_skip(SUITE) else {
        return;
    };
    let mut rec = Recorder::<Step>::with_step_values(&ctx, 1);
    for _ in 0..600 {
        drop(rec.pass("outside_steps"));
    }
    rec.begin_step(&step(0));
    for _ in 0..STEP_PASS_CAP {
        drop(rec.pass("in_the_step"));
    }
    rec.end_step();
    assert!(rec.submit().is_none(), "the full step submitted at its end");
}

/// A read whose size is not a multiple of 4 bytes stops with a message:
/// wgpu copies buffers in 4-byte units.
#[test]
fn a_read_not_in_four_byte_units_stops_with_a_message() {
    let Some(ctx) = gpu_context_or_skip(SUITE) else {
        return;
    };
    let kernel = Kernel::new(&ctx, 1, true);
    let message = panic_message(|| drop(read_buffer::<u16>(&ctx, &kernel.cells, 1)));
    assert!(message.contains("multiples of 4 bytes"), "{message}");
}

/// Reads, writes and submits inside a step stop with a message: each would
/// split the step's values across two submits.
#[test]
fn reads_writes_and_submits_inside_a_step_stop() {
    let Some(ctx) = gpu_context_or_skip(SUITE) else {
        return;
    };
    let kernel = Kernel::new(&ctx, 1, true);
    let in_a_step = || {
        let mut rec = Recorder::<Step>::with_step_values(&ctx, 1);
        rec.begin_step(&step(0));
        rec
    };
    let mut rec = in_a_step();
    let read = panic_message(|| drop(rec.read::<u32>(&kernel.cells, 1)));
    let mut rec = in_a_step();
    let write = panic_message(|| rec.write(&kernel.cells, 0, &[0; 4]));
    let mut rec = in_a_step();
    let submit = panic_message(|| {
        rec.submit();
    });
    assert!(read.contains("a read inside a step"), "{read}");
    assert!(write.contains("a write inside a step"), "{write}");
    assert!(submit.contains("a submit inside a step"), "{submit}");
}
