//! GPU-accelerated SDF-SDF collision detection.
//!
//! Dispatches the `trace_surface.wgsl` compute shader to find contacts
//! between two SDF grids, replacing the CPU triple-nested loop in
//! `trace_surface_into_other()`.

#![allow(
    clippy::cast_possible_truncation, // f64→f32 intentional
    clippy::cast_precision_loss
)]

use bytemuck::{Pod, Zeroable};
use nalgebra::{Isometry3, Point3, Translation3, Vector3};
use sim_core::sdf::SdfContact;
use sim_types::Pose;

use crate::buffers::GpuSdfGrid;
use crate::context::GpuContext;

// ── GPU-side types (must match WGSL layout exactly) ────────────────────

/// Trace dispatch parameters matching the WGSL `TraceParams` struct.
///
/// 208 bytes: 3 × mat4x4 (192) + 4 × f32/u32 (16).
#[repr(C)]
#[derive(Debug, Copy, Clone, Pod, Zeroable)]
pub struct TraceParams {
    /// Source body pose: local → world (column-major mat4x4).
    pub src_pose: [f32; 16],
    /// Destination body pose: local → world (for normal rotation).
    pub dst_pose: [f32; 16],
    /// Inverse destination pose: world → local.
    pub dst_pose_inv: [f32; 16],
    /// Surface band filter: `src_cell_size * 2.0`.
    pub surface_threshold: f32,
    /// Contact detection range.
    pub contact_margin: f32,
    /// Normal convention: 0 = A→B (negate), 1 = B→A (keep).
    pub flip_normal: u32,
    /// Padding for 16-byte alignment.
    #[allow(clippy::pub_underscore_fields)]
    pub _pad: f32,
}

/// Contact output matching the WGSL `GpuContact` struct.
///
/// 32 bytes.
#[repr(C)]
#[derive(Debug, Copy, Clone, Pod, Zeroable)]
pub struct GpuContact {
    /// World-space contact position.
    pub point: [f32; 3],
    /// Penetration depth (>= 0).
    pub penetration: f32,
    /// World-space surface normal.
    pub normal: [f32; 3],
    /// Padding.
    #[allow(clippy::pub_underscore_fields)]
    pub _pad: f32,
}

/// Maximum contacts per dispatch (sized for worst case: `grid_size`^3).
const MAX_CONTACTS: u32 = 32768;

// ── Pose conversion ────────────────────────────────────────────────────

/// Convert a `Pose` to a column-major 4×4 f32 matrix for WGSL.
///
/// Uses `nalgebra::Isometry3::to_homogeneous()` which produces column-major
/// layout matching WGSL `mat4x4<f32>`.
#[must_use]
pub fn pose_to_mat4(pose: &Pose) -> [f32; 16] {
    let iso = Isometry3::from_parts(Translation3::from(pose.position.coords), pose.rotation);
    let mat = iso.to_homogeneous();
    let mut result = [0.0f32; 16];
    for (i, &v) in mat.as_slice().iter().enumerate() {
        result[i] = v as f32;
    }
    result
}

/// Convert a `Pose` to its inverse as a column-major 4×4 f32 matrix.
#[must_use]
pub fn pose_to_mat4_inv(pose: &Pose) -> [f32; 16] {
    let iso = Isometry3::from_parts(Translation3::from(pose.position.coords), pose.rotation);
    let inv = iso.inverse();
    let mat = inv.to_homogeneous();
    let mut result = [0.0f32; 16];
    for (i, &v) in mat.as_slice().iter().enumerate() {
        result[i] = v as f32;
    }
    result
}

// ── GPU tracer ─────────────────────────────────────────────────────────

/// GPU compute pipeline for SDF-SDF surface tracing.
///
/// Owns persistent GPU buffers for output contacts, atomic counter,
/// staging readback, and per-dispatch params. Buffers are allocated
/// once and reused each step to avoid per-step allocation overhead.
pub struct GpuTracer {
    pipeline: wgpu::ComputePipeline,
    bind_group_layout: wgpu::BindGroupLayout,
    // Persistent buffers (allocated once, reused each step)
    contact_buffer: wgpu::Buffer,
    count_buffer: wgpu::Buffer,
    contact_staging: wgpu::Buffer,
    count_staging: wgpu::Buffer,
    params_ab_buf: wgpu::Buffer,
    params_ba_buf: wgpu::Buffer,
}

impl GpuTracer {
    /// Create the compute pipeline and persistent buffers.
    #[must_use]
    pub fn new(ctx: &GpuContext) -> Self {
        let shader_source = include_str!("shaders/trace_surface.wgsl");
        let shader_module = ctx
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("trace_surface"),
                source: wgpu::ShaderSource::Wgsl(shader_source.into()),
            });

        let bind_group_layout =
            ctx.device
                .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                    label: Some("trace_surface_layout"),
                    entries: &[
                        bgl_entry(0, wgpu::BufferBindingType::Storage { read_only: true }),
                        bgl_entry(1, wgpu::BufferBindingType::Storage { read_only: true }),
                        bgl_entry(2, wgpu::BufferBindingType::Uniform),
                        bgl_entry(3, wgpu::BufferBindingType::Uniform),
                        bgl_entry(4, wgpu::BufferBindingType::Uniform),
                        bgl_entry(5, wgpu::BufferBindingType::Storage { read_only: false }),
                        bgl_entry(6, wgpu::BufferBindingType::Storage { read_only: false }),
                    ],
                });

        let pipeline_layout = ctx
            .device
            .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("trace_surface_pipeline_layout"),
                bind_group_layouts: &[&bind_group_layout],
                push_constant_ranges: &[],
            });

        let pipeline = ctx
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("trace_surface_pipeline"),
                layout: Some(&pipeline_layout),
                module: &shader_module,
                entry_point: Some("trace_surface"),
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                cache: None,
            });

        let contact_buf_size = (MAX_CONTACTS as usize * std::mem::size_of::<GpuContact>()) as u64;
        let params_size = std::mem::size_of::<TraceParams>() as u64;

        let contact_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("contacts"),
            size: contact_buf_size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });

        let count_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("contact_count"),
            size: 4,
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_SRC
                | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let contact_staging = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("contact_staging"),
            size: contact_buf_size,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        let count_staging = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("count_staging"),
            size: 4,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        let params_ab_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("params_ab"),
            size: params_size,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let params_ba_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("params_ba"),
            size: params_size,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        Self {
            pipeline,
            bind_group_layout,
            contact_buffer,
            count_buffer,
            contact_staging,
            count_staging,
            params_ab_buf,
            params_ba_buf,
        }
    }

    /// Trace contacts between two SDF grids (symmetric: A→B and B→A).
    ///
    /// Returns deduplicated contacts in world space. Uses persistent
    /// buffers — no per-step GPU allocation.
    #[must_use]
    #[allow(clippy::too_many_lines)]
    pub fn trace_contacts(
        &self,
        ctx: &GpuContext,
        src_grid: &GpuSdfGrid,
        src_pose: &Pose,
        dst_grid: &GpuSdfGrid,
        dst_pose: &Pose,
        margin: f64,
    ) -> Vec<SdfContact> {
        let t_total = std::time::Instant::now();

        let contact_buf_size =
            (MAX_CONTACTS as usize * std::mem::size_of::<GpuContact>()) as wgpu::BufferAddress;

        // Reset atomic counter to 0
        ctx.queue
            .write_buffer(&self.count_buffer, 0, &0u32.to_le_bytes());

        // Write params for A→B dispatch
        let src_mat = pose_to_mat4(src_pose);
        let dst_mat = pose_to_mat4(dst_pose);
        let src_mat_inv = pose_to_mat4_inv(src_pose);
        let dst_mat_inv = pose_to_mat4_inv(dst_pose);
        let margin_f32 = margin as f32;

        let params_ab = TraceParams {
            src_pose: src_mat,
            dst_pose: dst_mat,
            dst_pose_inv: dst_mat_inv,
            surface_threshold: src_grid.meta.cell_size * 2.0,
            contact_margin: margin_f32,
            flip_normal: 0,
            _pad: 0.0,
        };
        ctx.queue
            .write_buffer(&self.params_ab_buf, 0, bytemuck::bytes_of(&params_ab));

        // Write params for B→A dispatch
        let params_ba = TraceParams {
            src_pose: dst_mat,
            dst_pose: src_mat,
            dst_pose_inv: src_mat_inv,
            surface_threshold: dst_grid.meta.cell_size * 2.0,
            contact_margin: margin_f32,
            flip_normal: 1,
            _pad: 0.0,
        };
        ctx.queue
            .write_buffer(&self.params_ba_buf, 0, bytemuck::bytes_of(&params_ba));

        // Create bind groups (lightweight — just references to existing buffers)
        let bind_group_ab = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("trace_ab"),
            layout: &self.bind_group_layout,
            entries: &[
                bg_entry(0, &src_grid.values_buffer),
                bg_entry(1, &dst_grid.values_buffer),
                bg_entry(2, &src_grid.meta_buffer),
                bg_entry(3, &dst_grid.meta_buffer),
                bg_entry(4, &self.params_ab_buf),
                bg_entry(5, &self.contact_buffer),
                bg_entry(6, &self.count_buffer),
            ],
        });

        let bind_group_ba = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("trace_ba"),
            layout: &self.bind_group_layout,
            entries: &[
                bg_entry(0, &dst_grid.values_buffer),
                bg_entry(1, &src_grid.values_buffer),
                bg_entry(2, &dst_grid.meta_buffer),
                bg_entry(3, &src_grid.meta_buffer),
                bg_entry(4, &self.params_ba_buf),
                bg_entry(5, &self.contact_buffer),
                bg_entry(6, &self.count_buffer),
            ],
        });

        let t_setup = t_total.elapsed();

        // ── Encode both dispatches + readback copy ─────────────────────
        let mut encoder = ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("trace_surface"),
            });

        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("trace_ab"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &bind_group_ab, &[]);
            pass.dispatch_workgroups(
                src_grid.meta.width.div_ceil(8),
                src_grid.meta.height.div_ceil(8),
                src_grid.meta.depth.div_ceil(4),
            );
        }

        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("trace_ba"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &bind_group_ba, &[]);
            pass.dispatch_workgroups(
                dst_grid.meta.width.div_ceil(8),
                dst_grid.meta.height.div_ceil(8),
                dst_grid.meta.depth.div_ceil(4),
            );
        }

        encoder.copy_buffer_to_buffer(&self.count_buffer, 0, &self.count_staging, 0, 4);
        encoder.copy_buffer_to_buffer(
            &self.contact_buffer,
            0,
            &self.contact_staging,
            0,
            contact_buf_size,
        );

        ctx.queue.submit([encoder.finish()]);

        // ── Single readback (one poll for everything) ──────────────────
        let count_slice = self.count_staging.slice(..);
        let contact_slice = self.contact_staging.slice(..);
        count_slice.map_async(wgpu::MapMode::Read, |_| {});
        contact_slice.map_async(wgpu::MapMode::Read, |_| {});
        poll_wait(ctx);

        let count = {
            let data = count_slice.get_mapped_range();
            u32::from_le_bytes([data[0], data[1], data[2], data[3]])
        };

        let count = count.min(MAX_CONTACTS) as usize;
        let gpu_contacts: Vec<GpuContact> = if count > 0 {
            let data = contact_slice.get_mapped_range();
            let all: &[GpuContact] = bytemuck::cast_slice(&data);
            all[..count].to_vec()
        } else {
            vec![]
        };

        // Unmap staging buffers for next use
        self.count_staging.unmap();
        self.contact_staging.unmap();

        let t_readback = t_total.elapsed().saturating_sub(t_setup);

        // Promote f32 → f64 and build SdfContact
        let cell_size = f64::from(src_grid.meta.cell_size).min(f64::from(dst_grid.meta.cell_size));
        let mut contacts: Vec<SdfContact> = gpu_contacts
            .iter()
            .map(|c| SdfContact {
                point: Point3::new(
                    f64::from(c.point[0]),
                    f64::from(c.point[1]),
                    f64::from(c.point[2]),
                ),
                normal: Vector3::new(
                    f64::from(c.normal[0]),
                    f64::from(c.normal[1]),
                    f64::from(c.normal[2]),
                ),
                penetration: f64::from(c.penetration),
            })
            .collect();

        dedup_contacts(&mut contacts, cell_size);
        let t_dedup = t_total.elapsed().saturating_sub(t_setup + t_readback);
        log_timing(t_setup, t_readback, t_dedup, contacts.len());
        contacts
    }
}

/// Log GPU dispatch timing (sampled: every 200th call to avoid spam).
fn log_timing(
    setup: std::time::Duration,
    readback: std::time::Duration,
    dedup: std::time::Duration,
    n_contacts: usize,
) {
    use std::sync::atomic::{AtomicU64, Ordering};
    static CALL_COUNT: AtomicU64 = AtomicU64::new(0);
    let n = CALL_COUNT.fetch_add(1, Ordering::Relaxed);
    if n % 200 == 0 {
        let total = setup + readback + dedup;
        eprintln!(
            "  GPU trace #{n}: total={total:.1?}  setup={setup:.1?}  readback={readback:.1?}  dedup={dedup:.1?}  contacts={n_contacts}"
        );
    }
}

// ── Deduplication ──────────────────────────────────────────────────────

/// Deduplicate contacts by spatial proximity.
///
/// Sorts by penetration depth (deepest first), then greedily keeps
/// contacts that are at least `cell_size * 0.5` apart.
fn dedup_contacts(contacts: &mut Vec<SdfContact>, cell_size: f64) {
    let min_dist_sq = (cell_size * 0.5) * (cell_size * 0.5);

    contacts.sort_by(|a, b| {
        b.penetration
            .partial_cmp(&a.penetration)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut kept = Vec::with_capacity(contacts.len());
    for c in contacts.drain(..) {
        let dominated = kept
            .iter()
            .any(|k: &SdfContact| (k.point - c.point).norm_squared() < min_dist_sq);
        if !dominated {
            kept.push(c);
        }
    }
    *contacts = kept;
}

// ── Helpers ────────────────────────────────────────────────────────────

fn poll_wait(ctx: &GpuContext) {
    match ctx.device.poll(wgpu::PollType::Wait {
        submission_index: None,
        timeout: Some(std::time::Duration::from_secs(5)),
    }) {
        Ok(_) => {}
        Err(e) => log::warn!("GPU poll timeout: {e:?}"),
    }
}

const fn bgl_entry(binding: u32, ty: wgpu::BufferBindingType) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty,
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    }
}

fn bg_entry(binding: u32, buffer: &wgpu::Buffer) -> wgpu::BindGroupEntry<'_> {
    wgpu::BindGroupEntry {
        binding,
        resource: buffer.as_entire_binding(),
    }
}

// ── GpuSdfCollider (trait implementation) ──────────────────────────────

use std::sync::Arc;

use sim_core::sdf::{GpuSdfCollision, PhysicsShape};
use sim_core::types::Model;

use crate::context::GpuError;

/// GPU-accelerated SDF collision backend.
///
/// Owns a GPU context, compute pipeline, and pre-uploaded SDF grids.
/// Implements [`GpuSdfCollision`] so it can be stored on `Model::gpu_collider`.
pub struct GpuSdfCollider {
    ctx: GpuContext,
    tracer: GpuTracer,
    grids: Vec<GpuSdfGrid>,
}

impl std::fmt::Debug for GpuSdfCollider {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GpuSdfCollider")
            .field("adapter", &self.ctx.adapter_info.name)
            .field("num_grids", &self.grids.len())
            .finish_non_exhaustive()
    }
}

impl GpuSdfCollider {
    /// Create a GPU collider from a model's shape data.
    ///
    /// Uploads all SDF grids to GPU memory and creates the compute pipeline.
    ///
    /// # Errors
    ///
    /// Returns [`GpuError`] if no GPU is available or device creation fails.
    pub fn new(shape_data: &[Arc<dyn PhysicsShape>]) -> Result<Self, GpuError> {
        let ctx = GpuContext::new()?;
        let tracer = GpuTracer::new(&ctx);
        let grids = shape_data
            .iter()
            .map(|shape| GpuSdfGrid::upload(&ctx, shape.sdf_grid()))
            .collect();
        Ok(Self { ctx, tracer, grids })
    }
}

impl GpuSdfCollision for GpuSdfCollider {
    fn sdf_sdf_contacts(
        &self,
        grid_a: usize,
        pose_a: &Pose,
        grid_b: usize,
        pose_b: &Pose,
        margin: f64,
    ) -> Vec<SdfContact> {
        self.tracer.trace_contacts(
            &self.ctx,
            &self.grids[grid_a],
            pose_a,
            &self.grids[grid_b],
            pose_b,
            margin,
        )
    }

    fn sdf_plane_contacts(
        &self,
        _grid: usize,
        _pose: &Pose,
        _plane_normal: &Vector3<f64>,
        _plane_offset: f64,
    ) -> Vec<SdfContact> {
        // Phase 5: SDF-plane GPU shader (not yet implemented)
        vec![]
    }
}

/// Enable GPU-accelerated SDF collision on a model.
///
/// Uploads all SDF grids from `model.shape_data` to GPU memory and
/// sets `model.gpu_collider`. If no GPU is available, returns an error
/// and the model continues using CPU collision.
///
/// # Errors
///
/// Returns [`GpuError`] if no GPU is available or device creation fails.
pub fn enable_gpu_collision(model: &mut Model) -> Result<(), GpuError> {
    if model.shape_data.is_empty() {
        return Ok(()); // No SDF shapes to accelerate
    }
    let collider = GpuSdfCollider::new(&model.shape_data)?;
    log::info!(
        "GPU collision enabled: {} on {} grids",
        collider.ctx.adapter_info.name,
        collider.grids.len()
    );
    model.gpu_collider = Some(Arc::new(collider));
    Ok(())
}

// ── Tests ──────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::unwrap_used
)]
mod tests {
    use super::*;
    use crate::{GpuContext, GpuSdfGrid};
    use cf_geometry::SdfGrid;
    use nalgebra::Point3;

    #[test]
    fn trace_contacts_match_cpu() {
        let ctx = GpuContext::new().expect("need GPU");

        // Two overlapping spheres (radius 5, centers 8 apart → 2mm overlap)
        let sdf_a = SdfGrid::sphere(Point3::origin(), 5.0, 16, 1.0);
        let sdf_b = SdfGrid::sphere(Point3::origin(), 5.0, 16, 1.0);
        let pose_a = Pose::identity();
        let pose_b = Pose::from_position(Point3::new(8.0, 0.0, 0.0));
        let margin = 1.0;

        // GPU path
        let gpu_grid_a = GpuSdfGrid::upload(&ctx, &sdf_a);
        let gpu_grid_b = GpuSdfGrid::upload(&ctx, &sdf_b);
        let tracer = GpuTracer::new(&ctx);
        let gpu_contacts =
            tracer.trace_contacts(&ctx, &gpu_grid_a, &pose_a, &gpu_grid_b, &pose_b, margin);

        // CPU path
        let cpu_contacts = sim_core::sdf::sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b, margin);

        eprintln!("  CPU contacts: {}", cpu_contacts.len());
        eprintln!("  GPU contacts: {}", gpu_contacts.len());
        for (i, c) in gpu_contacts.iter().enumerate() {
            eprintln!(
                "    GPU[{i}] pos=({:.2},{:.2},{:.2}) pen={:.4} n=({:.2},{:.2},{:.2})",
                c.point.x, c.point.y, c.point.z, c.penetration, c.normal.x, c.normal.y, c.normal.z
            );
        }
        for (i, c) in cpu_contacts.iter().enumerate() {
            eprintln!(
                "    CPU[{i}] pos=({:.2},{:.2},{:.2}) pen={:.4} n=({:.2},{:.2},{:.2})",
                c.point.x, c.point.y, c.point.z, c.penetration, c.normal.x, c.normal.y, c.normal.z
            );
        }

        // Both should find contacts
        assert!(!cpu_contacts.is_empty(), "CPU should find contacts");
        assert!(!gpu_contacts.is_empty(), "GPU should find contacts");

        // Most CPU contacts should have a GPU contact within 1.5mm (1.5× cell size).
        // f32 precision in surface reconstruction can shift contacts by ~1 cell.
        // Require ≥90% match rate (boundary contacts may differ).
        let mut matched = 0usize;
        for cpu_c in &cpu_contacts {
            let nearest = gpu_contacts
                .iter()
                .map(|g| (g.point - cpu_c.point).norm())
                .min_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap_or(f64::MAX);
            if nearest < 1.5 {
                matched += 1;
            }
        }
        let match_rate = matched as f64 / cpu_contacts.len() as f64;
        eprintln!(
            "  Match rate: {matched}/{} = {:.1}%",
            cpu_contacts.len(),
            match_rate * 100.0
        );
        assert!(
            match_rate > 0.9,
            "GPU should match ≥90% of CPU contacts (got {:.1}%)",
            match_rate * 100.0,
        );

        // Net force direction should agree (both should push B away from A → +X)
        let cpu_net: Vector3<f64> = cpu_contacts.iter().map(|c| c.normal * c.penetration).sum();
        let gpu_net: Vector3<f64> = gpu_contacts.iter().map(|c| c.normal * c.penetration).sum();
        assert!(
            cpu_net.x > 0.0,
            "CPU net force should be +X, got {:.4}",
            cpu_net.x
        );
        assert!(
            gpu_net.x > 0.0,
            "GPU net force should be +X, got {:.4}",
            gpu_net.x
        );
    }

    /// Cylinder SDF: `max(r - radius, |z| - half_h)`
    fn cylinder_sdf(p: Point3<f64>, radius: f64, half_h: f64) -> f64 {
        let r = p.x.hypot(p.y);
        (r - radius).max(p.z.abs() - half_h)
    }

    #[test]
    fn hinge_contacts_match_cpu() {
        let ctx = GpuContext::new().expect("need GPU");

        // Concave socket: outer cylinder - bore - cap openings
        // Same dimensions as Phase 3d test in model_builder.rs
        let socket_grid = SdfGrid::from_fn(16, 16, 24, 1.0, Point3::new(-8.0, -8.0, -12.0), |p| {
            let outer = cylinder_sdf(p, 5.5, 10.0);
            let bore = cylinder_sdf(p, 3.5, 8.0);
            let caps = cylinder_sdf(p, 2.5, 11.0);
            outer.max((-bore).max(-caps)) // CSG: outer - bore - caps
        });

        // Convex pin: shaft + flanges
        let pin_grid = SdfGrid::from_fn(12, 12, 20, 1.0, Point3::new(-6.0, -6.0, -10.0), |p| {
            let shaft = cylinder_sdf(p, 3.0, 6.0);
            let top = cylinder_sdf(Point3::new(p.x, p.y, p.z - 6.0), 3.2, 1.0);
            let bot = cylinder_sdf(Point3::new(p.x, p.y, p.z + 6.0), 3.2, 1.0);
            shaft.min(top).min(bot) // CSG union
        });

        // Pin offset 0.9mm below socket center (bottom flange near cap)
        let pose_socket = Pose::from_position(Point3::new(0.0, 0.0, 15.0));
        let pose_pin = Pose::from_position(Point3::new(0.0, 0.0, 14.1)); // 15.0 - 0.9
        let margin = 0.5; // half cell size

        // GPU path
        let gpu_socket = GpuSdfGrid::upload(&ctx, &socket_grid);
        let gpu_pin = GpuSdfGrid::upload(&ctx, &pin_grid);
        let tracer = GpuTracer::new(&ctx);
        let gpu_contacts =
            tracer.trace_contacts(&ctx, &gpu_socket, &pose_socket, &gpu_pin, &pose_pin, margin);

        // CPU path
        let cpu_contacts = sim_core::sdf::sdf_sdf_contact(
            &socket_grid,
            &pose_socket,
            &pin_grid,
            &pose_pin,
            margin,
        );

        eprintln!("  Hinge — CPU contacts: {}", cpu_contacts.len());
        eprintln!("  Hinge — GPU contacts: {}", gpu_contacts.len());

        assert!(!cpu_contacts.is_empty(), "CPU should find hinge contacts");
        assert!(!gpu_contacts.is_empty(), "GPU should find hinge contacts");

        // Match rate ≥ 90%
        let mut matched = 0usize;
        for cpu_c in &cpu_contacts {
            let nearest = gpu_contacts
                .iter()
                .map(|g| (g.point - cpu_c.point).norm())
                .min_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap_or(f64::MAX);
            if nearest < 1.5 {
                matched += 1;
            }
        }
        let match_rate = matched as f64 / cpu_contacts.len() as f64;
        eprintln!(
            "  Hinge match rate: {matched}/{} = {:.1}%",
            cpu_contacts.len(),
            match_rate * 100.0
        );
        assert!(
            match_rate > 0.9,
            "GPU should match ≥90% of CPU hinge contacts (got {:.1}%)",
            match_rate * 100.0,
        );

        // Net force should have a -Z component (pin pushed up by cap)
        let gpu_net: Vector3<f64> = gpu_contacts.iter().map(|c| c.normal * c.penetration).sum();
        let cpu_net: Vector3<f64> = cpu_contacts.iter().map(|c| c.normal * c.penetration).sum();
        eprintln!(
            "  Hinge CPU net: ({:.3},{:.3},{:.3})",
            cpu_net.x, cpu_net.y, cpu_net.z
        );
        eprintln!(
            "  Hinge GPU net: ({:.3},{:.3},{:.3})",
            gpu_net.x, gpu_net.y, gpu_net.z
        );
    }
}
