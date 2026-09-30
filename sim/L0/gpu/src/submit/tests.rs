//! The recorder's gates (recon §17a): each step reads its own values, a write
//! waits for the steps before it, a submit at the cap completes, and what the
//! recorder refuses, it refuses with a message.
//!
//! The two collapse tests assert wgpu's own behaviour, the reason the ring and
//! the write rule exist: if either stops collapsing, the reason is gone.

#![allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]

use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::mpsc;
use std::time::Duration;

use bytemuck::{Pod, Zeroable};

use super::{PASS_CAP, Recorder, Recording, read_buffer};
use crate::context::GpuContext;
use crate::test_support::gpu_context_or_skip;

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
";

struct Kernel {
    own_value: wgpu::ComputePipeline,
    from_source: wgpu::ComputePipeline,
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

/// ★★ A submit of [`PASS_CAP`] passes completes on this backend. Run on a
/// thread, so a submit that blocks, as 2 048 passes do on Metal, fails the
/// test after a time limit instead of hanging the suite.
#[test]
fn a_submit_at_the_cap_completes() {
    let Some(ctx) = gpu_context_or_skip(SUITE) else {
        return;
    };
    let (done, finished) = mpsc::channel();
    std::thread::spawn(move || {
        let kernel = Kernel::new(&ctx, 1, true);
        let mut rec = Recorder::<Step>::with_step_values(&ctx, 1);
        let group = kernel.bind_group(&ctx, rec.step_values_binding().unwrap());
        rec.write(&kernel.cells, 0, bytemuck::bytes_of(&0_u32));
        let offset = rec.begin_step(&step(0));
        for _ in 0..PASS_CAP {
            let mut pass = rec.pass("at_the_cap");
            pass.set_pipeline(&kernel.own_value);
            pass.set_bind_group(0, &group, &[offset]);
            pass.dispatch_workgroups(1, 1, 1);
        }
        rec.end_step();
        done.send(rec.read::<u32>(&kernel.cells, 1)).unwrap();
    });
    let cells = finished
        .recv_timeout(Duration::from_mins(1))
        .unwrap_or_else(|_| panic!("a submit of {PASS_CAP} passes did not finish in a minute"));
    assert_eq!(cells, [step(0).value]);
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

/// ★ One pass past the cap stops with a message, before a submit that would
/// block could be built.
#[test]
fn a_pass_past_the_cap_stops_with_a_message() {
    let Some(ctx) = gpu_context_or_skip(SUITE) else {
        return;
    };
    let mut rec = Recorder::new(&ctx);
    let message = panic_message(|| {
        for _ in 0..=PASS_CAP {
            drop(rec.pass("past_the_cap"));
        }
    });
    assert!(
        message.contains("compute passes without a submit"),
        "{message}"
    );
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
