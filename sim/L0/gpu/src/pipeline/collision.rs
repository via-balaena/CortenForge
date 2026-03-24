//! GPU collision pipeline — AABB + pre-computed narrowphase dispatch.
//!
//! Session 4: Per-geom AABB computation, SDF-SDF narrowphase (adapted from
//! `trace_surface.wgsl`), and SDF-plane narrowphase. Produces contacts in
//! a GPU buffer for Session 5's constraint assembly.
//!
//! Instead of a broadphase shader, uses a pre-computed pair plan determined
//! at model upload time. Each narrowphase dispatch includes an AABB overlap
//! guard — if the pair's AABBs don't overlap, all threads return immediately.

#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::similar_names,     // ct_i/ca_i intentional (contype/conaffinity)
    clippy::doc_markdown,
    clippy::too_many_lines,
)]

use sim_core::types::Model;

use super::model_buffers::GpuModelBuffers;
use super::state_buffers::GpuStateBuffers;
use super::types::{
    GPU_GEOM_PLANE, GPU_GEOM_SDF, NarrowphaseParams, SDF_META_NONE, SdfMetaGpu, geom_type_to_gpu,
};
use crate::context::GpuContext;

// ── Pair descriptor ────────────────────────────────────────────────────

/// Pre-computed narrowphase dispatch info for one collision pair.
struct NarrowphasePair {
    geom_a: u32,
    geom_b: u32,
    pair_type: PairType,
    /// Combined friction: element-wise max of both geoms.
    friction: [f32; 3],
    /// Contact margin.
    margin: f32,
}

enum PairType {
    /// Two SDF geoms — dispatch `sdf_sdf_narrow.wgsl` twice (A→B, B→A).
    SdfSdf { sdf_meta_a: u32, sdf_meta_b: u32 },
    /// SDF geom vs plane geom — dispatch `sdf_plane_narrow.wgsl` once.
    SdfPlane {
        sdf_geom: u32,
        plane_geom: u32,
        sdf_meta_idx: u32,
    },
}

// ── AABB params (uniform) ──────────────────────────────────────────────

/// Minimal uniform for the AABB shader.
#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct AabbParams {
    ngeom: u32,
    _pad: [u32; 3],
}

// ── Pipeline ───────────────────────────────────────────────────────────

/// GPU collision pipeline with pre-computed pair plan.
pub struct GpuCollisionPipeline {
    // Compute pipelines
    aabb_pipeline: wgpu::ComputePipeline,
    sdf_sdf_pipeline: wgpu::ComputePipeline,
    sdf_plane_pipeline: wgpu::ComputePipeline,

    // Bind group layouts
    aabb_bg_layout_0: wgpu::BindGroupLayout,
    aabb_bg_layout_1: wgpu::BindGroupLayout,
    aabb_bg_layout_2: wgpu::BindGroupLayout,
    narrow_bg_layout_0: wgpu::BindGroupLayout,
    narrow_bg_layout_1: wgpu::BindGroupLayout,
    narrow_bg_layout_2: wgpu::BindGroupLayout,
    narrow_bg_layout_3: wgpu::BindGroupLayout,

    // Pre-computed pair plan
    pairs: Vec<NarrowphasePair>,

    // Per-pair params uniform buffer
    narrow_params_buf: wgpu::Buffer,

    // AABB params uniform buffer
    aabb_params_buf: wgpu::Buffer,

    // Cached grid metadata (for building narrowphase params)
    sdf_metas_cpu: Vec<SdfMetaGpu>,
}

impl GpuCollisionPipeline {
    /// Create the collision pipeline and pre-compute the pair plan.
    #[must_use]
    pub fn new(ctx: &GpuContext, model: &Model, model_bufs: &GpuModelBuffers) -> Self {
        // ── Build pair plan ───────────────────────────────────────────
        let pairs = build_pair_plan(model);

        // ── Cache SDF metadata for narrowphase param building ─────────
        let sdf_metas_cpu: Vec<SdfMetaGpu> = model
            .shape_data
            .iter()
            .map(|shape| {
                let grid = shape.sdf_grid();
                SdfMetaGpu {
                    width: grid.width() as u32,
                    height: grid.height() as u32,
                    depth: grid.depth() as u32,
                    cell_size: grid.cell_size() as f32,
                    origin: [
                        grid.origin().x as f32,
                        grid.origin().y as f32,
                        grid.origin().z as f32,
                    ],
                    values_offset: 0, // not needed on CPU side
                }
            })
            .collect();

        // ── AABB shader ───────────────────────────────────────────────
        let aabb_source = include_str!("../shaders/aabb.wgsl");
        let aabb_module = ctx
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("aabb"),
                source: wgpu::ShaderSource::Wgsl(aabb_source.into()),
            });

        let aabb_bg_layout_0 = ctx
            .device
            .create_bind_group_layout(&bgl_desc("aabb_g0", &[bgl_uniform(0)]));
        let aabb_bg_layout_1 = ctx
            .device
            .create_bind_group_layout(&bgl_desc("aabb_g1", &[bgl_storage_ro(0)]));
        let aabb_bg_layout_2 = ctx.device.create_bind_group_layout(&bgl_desc(
            "aabb_g2",
            &[bgl_storage_ro(0), bgl_storage_ro(1), bgl_storage_rw(2)],
        ));

        let aabb_pl = ctx
            .device
            .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("aabb_layout"),
                bind_group_layouts: &[&aabb_bg_layout_0, &aabb_bg_layout_1, &aabb_bg_layout_2],
                push_constant_ranges: &[],
            });

        let aabb_pipeline = ctx
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("aabb"),
                layout: Some(&aabb_pl),
                module: &aabb_module,
                entry_point: Some("compute_aabb"),
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                cache: None,
            });

        // ── SDF-SDF narrowphase shader ────────────────────────────────
        let sdf_sdf_source = include_str!("../shaders/sdf_sdf_narrow.wgsl");
        let sdf_sdf_module = ctx
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("sdf_sdf_narrow"),
                source: wgpu::ShaderSource::Wgsl(sdf_sdf_source.into()),
            });

        // ── SDF-plane narrowphase shader ──────────────────────────────
        let sdf_plane_source = include_str!("../shaders/sdf_plane_narrow.wgsl");
        let sdf_plane_module = ctx
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("sdf_plane_narrow"),
                source: wgpu::ShaderSource::Wgsl(sdf_plane_source.into()),
            });

        // ── Narrowphase bind group layouts (shared by both shaders) ───
        let narrow_bg_layout_0 = ctx
            .device
            .create_bind_group_layout(&bgl_desc("narrow_g0", &[bgl_uniform(0)]));
        let narrow_bg_layout_1 = ctx.device.create_bind_group_layout(&bgl_desc(
            "narrow_g1",
            &[bgl_storage_ro(0), bgl_storage_ro(1)],
        ));
        let narrow_bg_layout_2 = ctx.device.create_bind_group_layout(&bgl_desc(
            "narrow_g2",
            &[bgl_storage_ro(0), bgl_storage_ro(1), bgl_storage_ro(2)],
        ));
        let narrow_bg_layout_3 = ctx.device.create_bind_group_layout(&bgl_desc(
            "narrow_g3",
            &[bgl_storage_rw(0), bgl_storage_rw(1)],
        ));

        let narrow_pl = ctx
            .device
            .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("narrow_layout"),
                bind_group_layouts: &[
                    &narrow_bg_layout_0,
                    &narrow_bg_layout_1,
                    &narrow_bg_layout_2,
                    &narrow_bg_layout_3,
                ],
                push_constant_ranges: &[],
            });

        let sdf_sdf_pipeline =
            ctx.device
                .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                    label: Some("sdf_sdf_narrow"),
                    layout: Some(&narrow_pl),
                    module: &sdf_sdf_module,
                    entry_point: Some("sdf_sdf_narrow"),
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                    cache: None,
                });

        let sdf_plane_pipeline =
            ctx.device
                .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                    label: Some("sdf_plane_narrow"),
                    layout: Some(&narrow_pl),
                    module: &sdf_plane_module,
                    entry_point: Some("sdf_plane_narrow"),
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                    cache: None,
                });

        // ── Uniform buffers ───────────────────────────────────────────
        let aabb_params_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("aabb_params"),
            size: 16, // AabbParams = 16 bytes
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let narrow_params_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("narrow_params"),
            size: 48, // NarrowphaseParams = 48 bytes
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        // Write AABB params once (ngeom doesn't change)
        let aabb_params = AabbParams {
            ngeom: model_bufs.ngeom,
            _pad: [0; 3],
        };
        ctx.queue
            .write_buffer(&aabb_params_buf, 0, bytemuck::bytes_of(&aabb_params));

        Self {
            aabb_pipeline,
            sdf_sdf_pipeline,
            sdf_plane_pipeline,
            aabb_bg_layout_0,
            aabb_bg_layout_1,
            aabb_bg_layout_2,
            narrow_bg_layout_0,
            narrow_bg_layout_1,
            narrow_bg_layout_2,
            narrow_bg_layout_3,
            pairs,
            narrow_params_buf,
            aabb_params_buf,
            sdf_metas_cpu,
        }
    }

    /// Encode collision dispatches into the command encoder.
    ///
    /// Must be called AFTER FK (which writes geom_xpos, geom_xmat).
    /// Writes: `geom_aabb`, `contact_buffer`, `contact_count`.
    pub fn encode(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        ctx: &GpuContext,
        model_bufs: &GpuModelBuffers,
        state_bufs: &GpuStateBuffers,
    ) {
        // 1. Reset contact counter to 0
        encoder.clear_buffer(&state_bufs.contact_count, 0, Some(4));

        // 2. AABB computation (one dispatch, all geoms)
        let aabb_bg0 = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("aabb_bg0"),
            layout: &self.aabb_bg_layout_0,
            entries: &[bg_entry(0, &self.aabb_params_buf)],
        });
        let aabb_bg1 = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("aabb_bg1"),
            layout: &self.aabb_bg_layout_1,
            entries: &[bg_entry(0, &model_bufs.geoms)],
        });
        let aabb_bg2 = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("aabb_bg2"),
            layout: &self.aabb_bg_layout_2,
            entries: &[
                bg_entry(0, &state_bufs.geom_xpos),
                bg_entry(1, &state_bufs.geom_xmat),
                bg_entry(2, &state_bufs.geom_aabb),
            ],
        });

        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("aabb"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.aabb_pipeline);
            pass.set_bind_group(0, &aabb_bg0, &[]);
            pass.set_bind_group(1, &aabb_bg1, &[]);
            pass.set_bind_group(2, &aabb_bg2, &[]);
            pass.dispatch_workgroups(model_bufs.ngeom.div_ceil(64), 1, 1);
        }

        // 3. Narrowphase dispatches (one per pre-computed pair)
        // Build shared bind groups (same for all narrowphase dispatches)
        let narrow_bg1 = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("narrow_bg1"),
            layout: &self.narrow_bg_layout_1,
            entries: &[
                bg_entry(0, &model_bufs.sdf_metas),
                bg_entry(1, &model_bufs.sdf_values),
            ],
        });
        let narrow_bg2 = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("narrow_bg2"),
            layout: &self.narrow_bg_layout_2,
            entries: &[
                bg_entry(0, &state_bufs.geom_xpos),
                bg_entry(1, &state_bufs.geom_xmat),
                bg_entry(2, &state_bufs.geom_aabb),
            ],
        });
        let narrow_bg3 = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("narrow_bg3"),
            layout: &self.narrow_bg_layout_3,
            entries: &[
                bg_entry(0, &state_bufs.contact_buffer),
                bg_entry(1, &state_bufs.contact_count),
            ],
        });

        for pair in &self.pairs {
            match &pair.pair_type {
                PairType::SdfSdf {
                    sdf_meta_a,
                    sdf_meta_b,
                } => {
                    let meta_a = &self.sdf_metas_cpu[*sdf_meta_a as usize];
                    let meta_b = &self.sdf_metas_cpu[*sdf_meta_b as usize];

                    // A→B dispatch (flip_normal = 0)
                    self.dispatch_sdf_sdf(
                        encoder,
                        ctx,
                        pair.geom_a,
                        pair.geom_b,
                        *sdf_meta_a,
                        *sdf_meta_b,
                        meta_a,
                        pair.friction,
                        pair.margin,
                        0, // flip_normal
                        &narrow_bg1,
                        &narrow_bg2,
                        &narrow_bg3,
                    );

                    // B→A dispatch (flip_normal = 1)
                    self.dispatch_sdf_sdf(
                        encoder,
                        ctx,
                        pair.geom_a, // geom1 stays the same (normal convention)
                        pair.geom_b,
                        *sdf_meta_b, // swap src/dst
                        *sdf_meta_a,
                        meta_b,
                        pair.friction,
                        pair.margin,
                        1, // flip_normal
                        &narrow_bg1,
                        &narrow_bg2,
                        &narrow_bg3,
                    );
                }
                PairType::SdfPlane {
                    sdf_geom,
                    plane_geom,
                    sdf_meta_idx,
                } => {
                    let meta = &self.sdf_metas_cpu[*sdf_meta_idx as usize];
                    self.dispatch_sdf_plane(
                        encoder,
                        ctx,
                        *sdf_geom,
                        *plane_geom,
                        *sdf_meta_idx,
                        meta,
                        pair.friction,
                        pair.margin,
                        &narrow_bg1,
                        &narrow_bg2,
                        &narrow_bg3,
                    );
                }
            }
        }
    }

    /// Number of pre-computed collision pairs.
    #[must_use]
    pub fn num_pairs(&self) -> usize {
        self.pairs.len()
    }

    #[allow(clippy::too_many_arguments)]
    fn dispatch_sdf_sdf(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        ctx: &GpuContext,
        geom1: u32,
        geom2: u32,
        src_meta_idx: u32,
        dst_meta_idx: u32,
        src_meta: &SdfMetaGpu,
        friction: [f32; 3],
        margin: f32,
        flip_normal: u32,
        narrow_bg1: &wgpu::BindGroup,
        narrow_bg2: &wgpu::BindGroup,
        narrow_bg3: &wgpu::BindGroup,
    ) {
        let params = NarrowphaseParams {
            geom1,
            geom2,
            src_sdf_meta_idx: src_meta_idx,
            dst_sdf_meta_idx: dst_meta_idx,
            surface_threshold: src_meta.cell_size * 2.0,
            contact_margin: margin,
            flip_normal,
            _pad: 0,
            friction: [friction[0], friction[1], friction[2], 0.0],
        };
        ctx.queue
            .write_buffer(&self.narrow_params_buf, 0, bytemuck::bytes_of(&params));

        let bg0 = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("sdf_sdf_bg0"),
            layout: &self.narrow_bg_layout_0,
            entries: &[bg_entry(0, &self.narrow_params_buf)],
        });

        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("sdf_sdf_narrow"),
            timestamp_writes: None,
        });
        pass.set_pipeline(&self.sdf_sdf_pipeline);
        pass.set_bind_group(0, &bg0, &[]);
        pass.set_bind_group(1, narrow_bg1, &[]);
        pass.set_bind_group(2, narrow_bg2, &[]);
        pass.set_bind_group(3, narrow_bg3, &[]);
        pass.dispatch_workgroups(
            src_meta.width.div_ceil(8),
            src_meta.height.div_ceil(8),
            src_meta.depth.div_ceil(4),
        );
    }

    #[allow(clippy::too_many_arguments)]
    fn dispatch_sdf_plane(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        ctx: &GpuContext,
        sdf_geom: u32,
        plane_geom: u32,
        sdf_meta_idx: u32,
        sdf_meta: &SdfMetaGpu,
        friction: [f32; 3],
        margin: f32,
        narrow_bg1: &wgpu::BindGroup,
        narrow_bg2: &wgpu::BindGroup,
        narrow_bg3: &wgpu::BindGroup,
    ) {
        let params = NarrowphaseParams {
            geom1: sdf_geom,
            geom2: plane_geom,
            src_sdf_meta_idx: sdf_meta_idx,
            dst_sdf_meta_idx: SDF_META_NONE,
            surface_threshold: sdf_meta.cell_size * 2.0,
            contact_margin: margin,
            flip_normal: 0, // unused for plane
            _pad: 0,
            friction: [friction[0], friction[1], friction[2], 0.0],
        };
        ctx.queue
            .write_buffer(&self.narrow_params_buf, 0, bytemuck::bytes_of(&params));

        let bg0 = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("sdf_plane_bg0"),
            layout: &self.narrow_bg_layout_0,
            entries: &[bg_entry(0, &self.narrow_params_buf)],
        });

        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("sdf_plane_narrow"),
            timestamp_writes: None,
        });
        pass.set_pipeline(&self.sdf_plane_pipeline);
        pass.set_bind_group(0, &bg0, &[]);
        pass.set_bind_group(1, narrow_bg1, &[]);
        pass.set_bind_group(2, narrow_bg2, &[]);
        pass.set_bind_group(3, narrow_bg3, &[]);
        pass.dispatch_workgroups(
            sdf_meta.width.div_ceil(8),
            sdf_meta.height.div_ceil(8),
            sdf_meta.depth.div_ceil(4),
        );
    }
}

// ── Pair plan builder ──────────────────────────────────────────────────

fn build_pair_plan(model: &Model) -> Vec<NarrowphasePair> {
    let ngeom = model.ngeom;
    let mut pairs = Vec::new();

    for i in 0..ngeom {
        for j in (i + 1)..ngeom {
            let ct_i = model.geom_contype[i];
            let ca_i = model.geom_conaffinity[i];
            let ct_j = model.geom_contype[j];
            let ca_j = model.geom_conaffinity[j];

            // Affinity check (same as CPU check_collision_affinity)
            if (ct_i & ca_j) == 0 && (ct_j & ca_i) == 0 {
                continue;
            }

            // Same-body exclusion
            if model.geom_body[i] == model.geom_body[j] {
                continue;
            }

            let type_i = geom_type_to_gpu(model.geom_type[i]);
            let type_j = geom_type_to_gpu(model.geom_type[j]);

            // Combine friction: element-wise max
            let fi = &model.geom_friction[i];
            let fj = &model.geom_friction[j];
            let friction = [
                fi.x.max(fj.x) as f32,
                fi.y.max(fj.y) as f32,
                fi.z.max(fj.z) as f32,
            ];

            // Combine margin: max
            let margin = model.geom_margin[i].max(model.geom_margin[j]) as f32;

            let sdf_meta_i = model.geom_shape.get(i).and_then(|o| *o).map(|s| s as u32);
            let sdf_meta_j = model.geom_shape.get(j).and_then(|o| *o).map(|s| s as u32);

            let pair_type = match (type_i, type_j) {
                (GPU_GEOM_SDF, GPU_GEOM_SDF) => {
                    if let (Some(a), Some(b)) = (sdf_meta_i, sdf_meta_j) {
                        Some(PairType::SdfSdf {
                            sdf_meta_a: a,
                            sdf_meta_b: b,
                        })
                    } else {
                        None
                    }
                }
                (GPU_GEOM_SDF, GPU_GEOM_PLANE) => sdf_meta_i.map(|idx| PairType::SdfPlane {
                    sdf_geom: i as u32,
                    plane_geom: j as u32,
                    sdf_meta_idx: idx,
                }),
                (GPU_GEOM_PLANE, GPU_GEOM_SDF) => sdf_meta_j.map(|idx| PairType::SdfPlane {
                    sdf_geom: j as u32,
                    plane_geom: i as u32,
                    sdf_meta_idx: idx,
                }),
                _ => None, // Non-SDF pairs: CPU-only
            };

            if let Some(pt) = pair_type {
                pairs.push(NarrowphasePair {
                    geom_a: i as u32,
                    geom_b: j as u32,
                    pair_type: pt,
                    friction,
                    margin,
                });
            }
        }
    }

    pairs
}

// ── wgpu helpers ───────────────────────────────────────────────────────

const fn bgl_desc<'a>(
    label: &'a str,
    entries: &'a [wgpu::BindGroupLayoutEntry],
) -> wgpu::BindGroupLayoutDescriptor<'a> {
    wgpu::BindGroupLayoutDescriptor {
        label: Some(label),
        entries,
    }
}

const fn bgl_uniform(binding: u32) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Uniform,
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    }
}

const fn bgl_storage_ro(binding: u32) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Storage { read_only: true },
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    }
}

const fn bgl_storage_rw(binding: u32) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Storage { read_only: false },
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
