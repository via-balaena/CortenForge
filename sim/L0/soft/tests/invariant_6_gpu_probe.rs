//! I-6b GPU probe: minimal wgpu build-graph tractability + round-trip.
//!
//! Feature-gated behind `gpu-probe`. Acquires a wgpu adapter, uploads a
//! `[f32; 16]` storage buffer, dispatches a trivial compute shader that
//! doubles each element, reads it back, and asserts `[1..=16] → [2..=32]`.
//!
//! Scope §1 I-6 clarification: this probe validates wgpu build-graph
//! tractability and round-trip correctness only. It does NOT exercise
//! `sim_ml_chassis::gpu` (B.5 ungated), GPU sparse linalg, GPU autograd,
//! or any chassis↔GPU state sharing — all Phase E. If no adapter is
//! available (headless CI), the test `eprintln!`s a skip message and
//! returns; libtest treats the clean return as pass.
//!
//! wgpu 27 API shapes verified against the existing `sim-gpu` consumer
//! (`sim/L0/gpu/src/{context, buffers, pipeline/integrate}.rs`) — in
//! particular: `request_adapter` returns `Result<_, RequestAdapterError>`,
//! `DeviceDescriptor` carries `trace` + `experimental_features`, and
//! `PollType::Wait` is a struct variant with `submission_index` + `timeout`.

#![cfg(feature = "gpu-probe")]
#![allow(
    // Probe is test-only: `expect` surfaces wgpu API misuse loudly
    // rather than being swept into silent `Result` propagation.
    clippy::expect_used,
    // `(i + 1) as f32` for `i ∈ 0..16` is lossless — 16 fits in f32's
    // 24-bit mantissa with ~20 bits to spare.
    clippy::cast_precision_loss,
    // Single end-to-end setup → dispatch → readback sequence; splitting
    // into helpers would spread the probe's one flow across multiple
    // fns for no readability win. Matches `sim-gpu`'s pipeline pattern
    // (`sim/L0/gpu/src/pipeline/integrate.rs:9`).
    clippy::too_many_lines
)]

use std::time::Duration;

use wgpu::util::DeviceExt;

/// Trivial compute shader: doubles each element of a 16-element f32 buffer.
/// `@workgroup_size(16)` × `dispatch_workgroups(1, 1, 1)` = 16 invocations.
const PROBE_WGSL: &str = r"
@group(0) @binding(0)
var<storage, read_write> buf: array<f32, 16>;

@compute @workgroup_size(16)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i < 16u) {
        buf[i] = buf[i] * 2.0;
    }
}
";

#[test]
fn gpu_probe_round_trip() {
    // ── Instance + adapter ──────────────────────────────────────────────
    let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor {
        backends: wgpu::Backends::METAL | wgpu::Backends::VULKAN,
        ..Default::default()
    });

    let adapter = match pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
        power_preference: wgpu::PowerPreference::LowPower,
        compatible_surface: None,
        force_fallback_adapter: false,
    })) {
        Ok(a) => a,
        Err(e) => {
            eprintln!("gpu_probe: no wgpu adapter on this host — skipping ({e})");
            return;
        }
    };
    eprintln!("gpu_probe: adapter = {}", adapter.get_info().name);

    // ── Device + queue ──────────────────────────────────────────────────
    let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
        label: Some("sim-soft-gpu-probe"),
        required_features: wgpu::Features::empty(),
        required_limits: wgpu::Limits::default(),
        memory_hints: wgpu::MemoryHints::Performance,
        trace: wgpu::Trace::Off,
        experimental_features: wgpu::ExperimentalFeatures::default(),
    }))
    .expect("request_device failed after adapter acquired");

    // ── Input / expected ────────────────────────────────────────────────
    let input: [f32; 16] = std::array::from_fn(|i| (i + 1) as f32);
    let expected: [f32; 16] = std::array::from_fn(|i| 2.0 * (i + 1) as f32);
    let size = std::mem::size_of_val(&input) as u64;

    // ── Buffers ─────────────────────────────────────────────────────────
    let storage = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("gpu_probe_storage"),
        contents: bytemuck::cast_slice(&input),
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
    });
    let staging = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("gpu_probe_staging"),
        size,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });

    // ── Shader + pipeline ───────────────────────────────────────────────
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("gpu_probe_shader"),
        source: wgpu::ShaderSource::Wgsl(PROBE_WGSL.into()),
    });
    let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("gpu_probe_bgl"),
        entries: &[wgpu::BindGroupLayoutEntry {
            binding: 0,
            visibility: wgpu::ShaderStages::COMPUTE,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Storage { read_only: false },
                has_dynamic_offset: false,
                min_binding_size: wgpu::BufferSize::new(size),
            },
            count: None,
        }],
    });
    let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("gpu_probe_pl"),
        bind_group_layouts: &[&bgl],
        push_constant_ranges: &[],
    });
    let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("gpu_probe_pipeline"),
        layout: Some(&pipeline_layout),
        module: &shader,
        entry_point: Some("main"),
        compilation_options: wgpu::PipelineCompilationOptions::default(),
        cache: None,
    });
    let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("gpu_probe_bg"),
        layout: &bgl,
        entries: &[wgpu::BindGroupEntry {
            binding: 0,
            resource: storage.as_entire_binding(),
        }],
    });

    // ── Encode + dispatch + copy-to-staging ─────────────────────────────
    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("gpu_probe_encoder"),
    });
    {
        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("gpu_probe_pass"),
            timestamp_writes: None,
        });
        pass.set_pipeline(&pipeline);
        pass.set_bind_group(0, &bind_group, &[]);
        pass.dispatch_workgroups(1, 1, 1);
    }
    encoder.copy_buffer_to_buffer(&storage, 0, &staging, 0, size);
    queue.submit([encoder.finish()]);

    // ── Map staging + poll + read ───────────────────────────────────────
    let slice = staging.slice(..);
    slice.map_async(wgpu::MapMode::Read, |_| {});
    device
        .poll(wgpu::PollType::Wait {
            submission_index: None,
            timeout: Some(Duration::from_secs(5)),
        })
        .expect("device.poll(Wait) failed on gpu_probe staging readback");

    let data = slice.get_mapped_range();
    let got: Vec<f32> = bytemuck::cast_slice(&data).to_vec();
    drop(data);

    // ── Verify ──────────────────────────────────────────────────────────
    assert_eq!(got.len(), expected.len(), "readback length mismatch");
    for (i, (&g, &e)) in got.iter().zip(expected.iter()).enumerate() {
        assert!(
            (g - e).abs() < 1e-6,
            "gpu_probe: mismatch at index {i}: got {g}, want {e}",
        );
    }
    eprintln!("gpu_probe: round-trip ok — 16-element [*=2.0] pass");
}
