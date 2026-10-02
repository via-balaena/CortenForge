//! §17e: which kernels hold a GPU step, and how near what this GPU streams they run ([`step7_gpu_kernels`]).

use std::collections::BTreeMap;
use std::time::Instant;

use sim_gpu::soft::GpuExecutor;
use sim_gpu::{GpuContext, Recorder, Recording};

use super::{
    POISSON, Press, SIZES, Spec, Stage, Timed, Wall, corners, env_number, run, spec, surface_nodes,
};

/// [`step7_gpu_kernels`]'s runs, in order: one pass a step (false) and a pass a dispatch (true), the second pair
/// reversed so a drift over the runs falls on both alike.
const KERNEL_RUNS: [bool; 4] = [false, true, true, false];

/// Each of the streaming copy's two buffers, bytes. Whether that is past the GPU's caches is not measured.
const STREAM_BYTES: u64 = 64 << 20;

/// Copies run before the timed ones, untimed.
const STREAM_WARMUP: usize = 10;

/// Timed copies a measurement.
const STREAM_COPIES: usize = 50;

/// The M4 Pro's memory bandwidth as Apple quotes it, GB/s: a specification, not measured.
const SPEC_GB_S: f64 = 273.0;

/// The step passes' speed-up D4 needs at ×8 with the estimates gone: D4's 2.30 times (§17c) on the step passes alone,
/// by §17d's ×8 shares (the step passes 0.828 of a run, 0.035 besides them and the estimates).
const D4_STEP_SPEEDUP: f64 = 2.07;

/// The items a reduction sums into one partial (`soft.wgsl`'s `TREE`).
const TREE: f64 = 256.0;

/// The kernels in a step that run on scratch arrays in the estimate too, as `soft.wgsl` names them.
const PHASE_KERNELS: [&str; 6] = [
    "element_dilations",
    "gather_volume_changes",
    "nodal_pressures",
    "elastic_element_forces",
    "viscous_element_forces",
    "gather_forces",
];

/// The copy kernel the stream is measured with.
const STREAM_WGSL: &str = "
@group(0) @binding(0) var<storage, read> source: array<vec4<u32>>;
@group(0) @binding(1) var<storage, read_write> destination: array<vec4<u32>>;

@compute @workgroup_size(256)
fn copy(@builtin(global_invocation_id) id: vec3<u32>) {
    destination[id.x] = source[id.x];
}
";

/// What [`stream`] measured.
struct Stream {
    /// Bytes read and written a second, the median copy's and the fastest's: the fastest over the specification bounds
    /// the timestamps' period from below, if the specification is this GPU's peak and the copies' bytes all crossed
    /// its memory.
    median: f64,
    most: f64,
    /// The copies' summed GPU seconds, over the host's from the first timed copy recorded to the read after the last.
    over_host: f64,
}

/// What this GPU streams: [`STREAM_COPIES`] copies of [`STREAM_BYTES`] by a compute kernel, each a pass timed by
/// its timestamps, after [`STREAM_WARMUP`] untimed. The destination, cleared after the warm-up, read back must equal
/// the source, and no copy may start before an earlier one ended.
fn stream(ctx: &GpuContext) -> Stream {
    let device = &ctx.device;
    let words = u32::try_from(STREAM_BYTES / 4).unwrap();
    let pattern: Vec<u32> = (1..=words).collect();
    let bytes: Vec<u8> = pattern.iter().flat_map(|w| w.to_le_bytes()).collect();
    let buffer = |label, usage| {
        device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size: STREAM_BYTES,
            usage: wgpu::BufferUsages::STORAGE | usage,
            mapped_at_creation: false,
        })
    };
    let source = buffer("stream source", wgpu::BufferUsages::COPY_DST);
    let destination = buffer(
        "stream destination",
        wgpu::BufferUsages::COPY_SRC | wgpu::BufferUsages::COPY_DST,
    );
    ctx.queue.write_buffer(&source, 0, &bytes);
    let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("stream"),
        source: wgpu::ShaderSource::Wgsl(STREAM_WGSL.into()),
    });
    let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("stream"),
        layout: None,
        module: &module,
        entry_point: Some("copy"),
        compilation_options: wgpu::PipelineCompilationOptions::default(),
        cache: None,
    });
    let group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("stream"),
        layout: &pipeline.get_bind_group_layout(0),
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: source.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: destination.as_entire_binding(),
            },
        ],
    });
    let mut recorder = Recorder::new(ctx);
    let copy = |recorder: &mut Recorder<()>| {
        let mut pass = recorder.pass("copy");
        pass.set_pipeline(&pipeline);
        pass.set_bind_group(0, &group, &[]);
        pass.dispatch_workgroups(words / 4 / 256, 1, 1);
    };
    for _ in 0..STREAM_WARMUP {
        copy(&mut recorder);
    }
    recorder.commands().clear_buffer(&destination, 0, None);
    recorder.time_passes();
    let started = Instant::now();
    for _ in 0..STREAM_COPIES {
        copy(&mut recorder);
    }
    let copied: Vec<u32> = recorder.read(&destination, words as usize);
    let host = started.elapsed().as_secs_f64();
    assert!(copied == pattern, "the copy did not copy the source");
    let mut times = recorder.pass_times().remove("copy").unwrap();
    assert_eq!(times.len(), STREAM_COPIES, "a copy's time is missing");
    assert!(times.iter().all(|&t| t > 0.0 && t.is_finite()), "{times:?}");
    assert_eq!(recorder.pass_overlaps(), 0, "copies ran at once");
    times.sort_by(f64::total_cmp);
    let rate = |seconds: f64| 2.0 * STREAM_BYTES as f64 / seconds;
    Stream {
        median: rate(times[STREAM_COPIES / 2]),
        most: rate(times[0]),
        over_host: times.iter().sum::<f64>() / host,
    }
}

/// The counts the bytes are taken over.
#[derive(Clone, Copy)]
struct Counts {
    elements: f64,
    nodes: f64,
    surface: f64,
}

/// The bytes a step's kernel, `phase/entry` as its pass's label ends, must read and write at least (`sim-gpu`'s
/// `soft.wgsl`): each field it uses counted once, a gathered array once whole. A scattered read fetches more than the
/// bytes it uses, by an amount not measured here. Uniforms, workgroup memory, the counters and the contact kernel's
/// grid samples are not counted; every node is taken to be in an element, and a held node's constraints, which it
/// skips, are counted.
fn least_bytes(kernel: &str, counts: Counts) -> Option<f64> {
    let Counts {
        elements: e,
        nodes: n,
        surface: s,
    } = counts;
    let partials = |items: f64, components: f64| 4.0 * components * (items / TREE).ceil();
    Some(match kernel {
        // An element's nodes and edge inverse; the displacements; its dilation written.
        "dilations/element_dilations" => 52.0 * e + 12.0 * n + 4.0 * e,
        // The offsets and entries; each element's rest volume, and its dilation; the volume changes written.
        "volume_changes/gather_volume_changes" => 4.0 * (n + 1.0) + 16.0 * e + 8.0 * e + 4.0 * n,
        // A node's rest volume and λ; its volume change; its pressure written.
        "pressures/nodal_pressures" => 8.0 * n + 8.0 * n,
        // An element's nodes, edge inverse, rest volume, stabilization, μ and C₂; the pressures and displacements, its
        // dilation; its forces written.
        "element_forces/elastic_element_forces" => 68.0 * e + 16.0 * n + 4.0 * e + 48.0 * e,
        // An element's nodes, edge inverse, rest volume and viscosity; the displacements and velocities; its forces
        // written.
        "element_forces/viscous_element_forces" => 60.0 * e + 24.0 * n + 48.0 * e,
        // The offsets and entries; the element forces; the node forces written.
        "gather_forces/gather_forces" => 4.0 * (n + 1.0) + 64.0 * e + 12.0 * n,
        // A surface node's index, and its node's rest, arm, masses and constraints; its displacement, velocity, elastic
        // and viscous forces; its anchor and maxima read and written; its contact and terms written.
        "contact/contact" => s * (4.0 + 56.0 + 48.0 + 24.0 + 16.0 + 28.0 + 28.0),
        "contact/reduce_partials" => 28.0 * s + partials(s, 7.0),
        "contact/reduce_row" => partials(s, 7.0) + 28.0,
        // A node's inverse mass and surface index, and a surface node's contact force; the velocities read and
        // written, the previous ones written, the elastic and viscous forces, the displacements read and written.
        "integrate/integrate" => 8.0 * n + 12.0 * s + 84.0 * n,
        // A node's inverse mass, constraints, mass and surface index, and a surface node's contact force; the
        // velocities and displacements read and written, the previous velocities, the viscous forces, the terms
        // written.
        "boundary_conditions/boundary_conditions" => 36.0 * n + 12.0 * s + 80.0 * n,
        "boundary_conditions/reduce_partials" => 8.0 * n + partials(n, 2.0),
        "boundary_conditions/reduce_row" => partials(n, 2.0) + 8.0,
        // A node's surface index, and a surface node's normal and friction forces; the displacements; the window's
        // sums read and written.
        "accumulate/accumulate" => 4.0 * n + 16.0 * s + 12.0 * n + 24.0 * n + 32.0 * s,
        _ => return None,
    })
}

/// The items a step's kernel runs over: elements, nodes, or surface nodes; one for a reduction's row.
fn items(kernel: &str, counts: Counts) -> f64 {
    match kernel {
        k if k.ends_with("reduce_row") => 1.0,
        k if k.starts_with("contact/") => counts.surface,
        "dilations/element_dilations"
        | "element_forces/elastic_element_forces"
        | "element_forces/viscous_element_forces" => counts.elements,
        _ => counts.nodes,
    }
}

/// The GPU seconds summed of each label in `times` that starts with `prefix`, keyed by the rest of the label.
fn under(times: &BTreeMap<String, Vec<f64>>, prefix: &str) -> BTreeMap<String, (f64, usize)> {
    times
        .iter()
        .filter_map(|(label, each)| {
            let rest = label.strip_prefix(prefix)?;
            Some((rest.to_owned(), (each.iter().sum(), each.len())))
        })
        .collect()
}

/// §17e: where the GPU's time inside a step goes, kernel by kernel, on [`super::step7_gpu_split`]'s press and corner,
/// and how near what this GPU streams each kernel runs. [`KERNEL_RUNS`] alternates runs recorded one pass a step
/// with runs recorded a pass a dispatch (`GpuExecutor::pass_per_dispatch`), every pass timed on the GPU; before each
/// run, a streaming copy measures what this GPU streams ([`stream`]). Every run prints the GPU's shares of its setup
/// and stepping (steps, reads, estimates) and its checks. A run a pass a dispatch also prints each step kernel's share
/// of the step passes' time, its passes a step, and its [`least_bytes`] over its time as a fraction of the stream's
/// median and of the specification; then the phases' shares, the whole step's fraction, and the estimate's kernels.
/// The last lines compare the runs: each split run's step time over the one-pass runs', the one-pass runs' drift, and
/// the repeat gap between the two split runs.
#[test]
#[ignore = "needs the repo-excluded product scan; run with --release --ignored --nocapture"]
fn step7_gpu_kernels() {
    let factor = env_number("STEP7_LOADING", 1.0);
    let size = SIZES[env_number("STEP7_SIZE", 0.0) as usize];
    let mut stage = Stage::new(&format!(
        "§17e, a GPU step's kernels at ×{size} h_K2's elements, loading ×{factor}"
    ));
    let wall = Wall::build(&stage.scene, stage.h_k2 / size.cbrt(), None, [0.0; 3]);
    let model = wall.model(POISSON, 1.0, 1.0);
    let start = stage.wall_line(&format!("×{size}"), &wall, &model);
    let loading = factor * Stage::budget_loading(&wall, start);
    let friction = corners()[1];
    let spec = Spec {
        instruments: false,
        reestimate_every: 500,
        gpu: true,
        ..spec(friction, loading)
    };
    stage.set(&spec);
    let damping = stage.damping(&wall);
    let press = Press {
        scene: &stage.scene,
        model: &model,
        surface: surface_nodes(&model),
        start,
        loading: spec.loading,
    };
    let counts = Counts {
        elements: model.elements().len() as f64,
        nodes: model.rest_positions().len() as f64,
        surface: press.surface.len() as f64,
    };
    println!(
        "  μ_f {friction}; the loop's re-estimate every 500 steps; the probe's instruments off [PUBLIC]; elements {}, \
         nodes {}, surface nodes {} [LOCAL]",
        counts.elements, counts.nodes, counts.surface
    );
    // Each run's step passes' GPU seconds a step, and a split run's shares of them by kernel.
    let mut one_pass = Vec::new();
    let mut split_runs: Vec<(f64, BTreeMap<String, f64>)> = Vec::new();
    for (i, split) in KERNEL_RUNS.into_iter().enumerate() {
        let ctx = GpuContext::with_timestamps().expect("a GPU adapter that writes timestamps");
        let streamed = stream(&ctx);
        println!(
            "  the stream before run {}: {:.0} GB/s median, {:.0} fastest, over {STREAM_COPIES} copies of {} MiB; the \
             copies' GPU time over the host's {:.3}; the fastest over the specification {:.3}, a lower bound on the \
             timestamps' period in wgpu's 1 ns [PUBLIC]",
            i + 1,
            streamed.median / 1e9,
            streamed.most / 1e9,
            STREAM_BYTES >> 20,
            streamed.over_host,
            streamed.most / 1e9 / SPEC_GB_S
        );
        let (run, mut stepper) = run(
            &press,
            &stage.obstacle,
            |m, o| {
                let mut executor = GpuExecutor::new(&ctx, m, o).unwrap();
                executor.time_passes();
                if split {
                    executor.pass_per_dispatch();
                }
                Timed::new(executor)
            },
            damping,
            spec,
        );
        let host = stepper.executor().times;
        let device = stepper.executor_mut().inner.pass_times();
        let overlaps = stepper.executor().inner.pass_overlaps();
        let steps = run.steps as f64;
        let stepping = run.clock.setup + run.clock.stepping;
        let total = |prefix: &str| under(&device, prefix).values().map(|(t, _)| t).sum::<f64>();
        let (step, read, estimate) = (total("step"), total("read"), total("estimate"));
        let on_device = step + read + estimate;
        println!(
            "  run {} ({}): {:.1} s over {} steps, {} reads, {} estimates [LOCAL]; stood {}; GPU of the setup and \
             stepping {:.3}: steps {:.3}, reads {:.3}, estimates {:.3}; over the host's waits {:.3}; any pass not \
             positive {}; any pass started before an earlier one ended {} [PUBLIC]; the step passes' µs a step {:.1}, \
             such passes {overlaps} [LOCAL]",
            i + 1,
            if split {
                "a pass a dispatch"
            } else {
                "one pass a step"
            },
            stepping,
            run.steps,
            host.reads_made,
            host.estimates_made,
            run.stopped.is_none(),
            on_device / stepping,
            step / stepping,
            read / stepping,
            estimate / stepping,
            on_device / (host.reads + host.estimates),
            device
                .values()
                .flatten()
                .any(|&t| t <= 0.0 || !t.is_finite()),
            overlaps > 0,
            1e6 * step / steps
        );
        if !split {
            let passes = device.get("step").map_or(0, Vec::len);
            println!(
                "    step passes over steps {:.3} [PUBLIC]",
                passes as f64 / steps
            );
            one_pass.push(step / steps);
            continue;
        }
        let kernels = under(&device, "step/");
        let mut ranked: Vec<_> = kernels.iter().collect();
        ranked.sort_by(|a, b| b.1.0.total_cmp(&a.1.0));
        let (mut counted_bytes, mut counted_time) = (0.0, 0.0);
        for (kernel, &(seconds, passes)) in ranked {
            let spec_over_stream = streamed.median / 1e9 / SPEC_GB_S;
            let bytes_line = least_bytes(kernel, counts).map_or_else(
                || "no bytes counted".to_owned(),
                |bytes| {
                    counted_bytes += bytes * passes as f64;
                    counted_time += seconds;
                    let fraction = bytes * passes as f64 / seconds / streamed.median;
                    format!(
                        "of the stream {fraction:.2}, of the specification {:.2}",
                        fraction * spec_over_stream
                    )
                },
            );
            println!(
                "    {kernel}: {:.3} of the step passes' time, {:.2} passes a step, {bytes_line} [PUBLIC]; µs a step \
                 {:.2}, ns an item {:.4} [LOCAL]",
                seconds / step,
                passes as f64 / steps,
                1e6 * seconds / steps,
                1e9 * seconds / passes as f64 / items(kernel, counts)
            );
        }
        let mut phases: BTreeMap<&str, f64> = BTreeMap::new();
        for (kernel, &(seconds, _)) in &kernels {
            let phase = kernel.split('/').next().unwrap_or(kernel);
            *phases.entry(phase).or_default() += seconds / step;
        }
        println!("    by phase: {phases:.3?} [PUBLIC]");
        let rate = counted_bytes / counted_time / streamed.median;
        println!(
            "    the step's counted kernels ({:.3} of its time): of the stream {rate:.2}; times {D4_STEP_SPEEDUP} = \
             {:.2} [PUBLIC]",
            counted_time / step,
            rate * D4_STEP_SPEEDUP
        );
        let estimated = under(&device, "estimate/");
        let mut parts = [0.0; 3];
        for (kernel, &(seconds, _)) in &estimated {
            let part = if PHASE_KERNELS.contains(&kernel.as_str()) {
                0
            } else if kernel.starts_with("estimate_") {
                1
            } else {
                2
            };
            parts[part] += seconds / estimate;
        }
        let mut by_kernel: Vec<_> = estimated
            .iter()
            .map(|(kernel, &(seconds, passes))| (kernel, seconds / estimate, passes))
            .collect();
        by_kernel.sort_by(|a, b| b.1.total_cmp(&a.1));
        println!(
            "    the estimates: phase kernels on scratch arrays {:.3}, the estimate's own {:.3}, reductions {:.3} of \
             their time [PUBLIC]",
            parts[0], parts[1], parts[2]
        );
        for (kernel, share, passes) in by_kernel {
            println!(
                "      {kernel}: {share:.3}, {:.1} passes an estimate [PUBLIC]",
                passes as f64 / host.estimates_made as f64
            );
        }
        split_runs.push((
            step / steps,
            kernels
                .into_iter()
                .map(|(kernel, (seconds, _))| (kernel, seconds / step))
                .collect(),
        ));
    }
    let one_pass_mean = one_pass.iter().sum::<f64>() / one_pass.len() as f64;
    for (i, (per_step, _)) in split_runs.iter().enumerate() {
        println!(
            "  split run {}: its step passes' time a step over the one-pass runs' mean {:.3} [PUBLIC]",
            i + 1,
            per_step / one_pass_mean
        );
    }
    println!(
        "  the one-pass runs: the second's step passes' time a step over the first's {:.3} [PUBLIC]",
        one_pass[1] / one_pass[0]
    );
    let (first, second) = (&split_runs[0].1, &split_runs[1].1);
    let gap = first
        .iter()
        .map(|(kernel, share)| (share - second.get(kernel).copied().unwrap_or(0.0)).abs())
        .fold(0.0, f64::max);
    println!(
        "  the repeat gap: the largest share difference between the split runs {gap:.4} [PUBLIC]"
    );
}
