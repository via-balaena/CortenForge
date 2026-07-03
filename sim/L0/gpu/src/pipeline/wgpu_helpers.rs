//! Shared wgpu boilerplate for the compute pipelines under [`super`].
//!
//! Every physics stage (`fk`, `crba`, `rne`, `smooth`, `integrate`,
//! `velocity_fk`, `eulerdamp`, `collision`, `constraint`) builds the same
//! handful of wgpu descriptors: a compute pipeline from a shared layout +
//! shader module, storage/uniform bind-group-layout entries, and whole-buffer
//! bind-group entries. These constructors previously lived as verbatim
//! copies (under three different naming conventions) in each stage module;
//! this module holds the single copy of each.
//!
//! All bindings target [`wgpu::ShaderStages::COMPUTE`] (this crate has no
//! render pipelines).

use crate::context::GpuContext;

/// Build a compute pipeline for `entry_point` from a shared pipeline layout
/// and shader module. The pipeline is labelled with its entry-point name.
pub(super) fn create_pipeline(
    ctx: &GpuContext,
    layout: &wgpu::PipelineLayout,
    module: &wgpu::ShaderModule,
    entry_point: &str,
) -> wgpu::ComputePipeline {
    ctx.device
        .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some(entry_point),
            layout: Some(layout),
            module,
            entry_point: Some(entry_point),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            cache: None,
        })
}

/// A storage-buffer bind-group-layout entry at `binding`. `read_only` selects
/// `var<storage, read>` (`true`) vs `var<storage, read_write>` (`false`);
/// no dynamic offset.
pub(super) const fn storage_entry(binding: u32, read_only: bool) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Storage { read_only },
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    }
}

/// A uniform-buffer bind-group-layout entry at `binding`, no dynamic offset.
///
/// For a uniform with a dynamic offset (per-level params), build the entry
/// inline — the sized `min_binding_size` differs per stage.
pub(super) const fn uniform_entry(binding: u32) -> wgpu::BindGroupLayoutEntry {
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

/// A bind-group entry binding the whole of `buffer` at `binding`.
pub(super) fn buf_entry(binding: u32, buffer: &wgpu::Buffer) -> wgpu::BindGroupEntry<'_> {
    wgpu::BindGroupEntry {
        binding,
        resource: buffer.as_entire_binding(),
    }
}
