//! `cf-studio-gui` binary — the Slint event-loop + `rfd` file-dialog +
//! wgpu 3D-view glue. The Project → display mapping and the per-step
//! actions (and their tests) live in the lib; the viewer's mesh + camera
//! math lives in `cf_studio_gui::viewer`. This file picks files, wires
//! state into the UI, and drives the wgpu render (which needs Slint's
//! device/queue, so it can't be headless-tested).

// Slint's generated code uses unwrap/expect/panic internally; confine the
// crate's restriction-lint denies so they don't fire on generated code,
// while our own code below stays held to them.
mod ui {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
    slint::include_modules!();
}

use std::cell::{Cell, RefCell};
use std::path::{Path, PathBuf};
use std::rc::Rc;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cf_studio_core::{
    DesignDraft, LayerDraft, MoldOutputs, PlugDraft, PourRecord, Project, RidgeOptions, RidgeRing,
    Step,
};
use cf_studio_engine::{
    CastMode, EditSession, PartId, PartSelection, PlugPreview, PrintExportReport, ReconstructShape,
    export_print_package, generate_molds_for_design, proxy_preview_mesh, run_simplify,
    silicone_catalog,
};

/// Cendrillon casts **bonded** (one plug, cast-in-place) — the product default
/// per `docs/CF_CAST_BONDED_INPLACE_TEXTURE_RECON.md`. The SDK keeps the
/// detachable model for other consumers; this is where the app pins its choice.
const CENDRILLON_CAST_MODE: CastMode = CastMode::Bonded;
use cf_studio_gui::viewer::{MeshData, OrbitCamera, Uniforms, Vertex, mesh_data_from_indexed};
use cf_studio_gui::{
    RidgeControls, StepOutcome, apply_design, apply_design_draft, apply_plug, apply_prep,
    apply_scan, cell_size_m_for_quality, enumerate_parts, format_molds_summary, format_pour_active,
    format_pour_plan, gate_ridge_options, nav_state, part_selection_from_checks, pour_countdown,
    print_step_summary, step_rows,
};
use cortenforge::mesh_types::IndexedMesh;
use slint::wgpu_28::wgpu;
use slint::{ComponentHandle, Model, ModelRc, SharedString, VecModel};
use ui::{AppWindow, LayerRow, PartRow, RingRow, StepRow};

/// Offscreen render-target edge (px). The view is square; Slint scales it
/// into the viewport with `image-fit: contain`.
const RENDER_SIZE: u32 = 560;
const COLOR_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8UnormSrgb;
const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

/// Lambert shading, two-sided so inconsistent scan winding never goes
/// fully black.
const SHADER: &str = r"
struct Uniforms { mvp: mat4x4<f32> };
@group(0) @binding(0) var<uniform> u: Uniforms;

struct VOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) normal: vec3<f32>,
};

@vertex
fn vs(@location(0) pos: vec3<f32>, @location(1) nrm: vec3<f32>) -> VOut {
    var o: VOut;
    o.clip = u.mvp * vec4<f32>(pos, 1.0);
    o.normal = nrm;
    return o;
}

@fragment
fn fs(in: VOut) -> @location(0) vec4<f32> {
    let n = normalize(in.normal);
    let light = normalize(vec3<f32>(0.4, 0.8, 0.6));
    let d = abs(dot(n, light));
    let base = vec3<f32>(0.55, 0.60, 0.70);
    return vec4<f32>(base * (0.30 + 0.70 * d), 1.0);
}
";

/// Cyan overlay line (the centerline), drawn over the mesh with the same
/// MVP. `depth_compare: Always` so the interior line shows through the
/// surface; positions only (no normals).
const LINE_SHADER: &str = r"
struct Uniforms { mvp: mat4x4<f32> };
@group(0) @binding(0) var<uniform> u: Uniforms;

@vertex
fn vs(@location(0) pos: vec3<f32>) -> @builtin(position) vec4<f32> {
    return u.mvp * vec4<f32>(pos, 1.0);
}

@fragment
fn fs() -> @location(0) vec4<f32> {
    return vec4<f32>(0.05, 0.85, 0.95, 1.0);
}
";

/// GPU resources + camera for the currently-shown mesh. Built from
/// Slint's device/queue once a scan is loaded; re-rendered on orbit.
struct Scene {
    device: wgpu::Device,
    queue: wgpu::Queue,
    pipeline: wgpu::RenderPipeline,
    /// Line-strip pipeline for the centerline overlay (shares the uniform).
    line_pipeline: wgpu::RenderPipeline,
    bind_group: wgpu::BindGroup,
    uniform: wgpu::Buffer,
    vbuf: wgpu::Buffer,
    ibuf: wgpu::Buffer,
    index_count: u32,
    /// Centerline overlay: vertex buffer + point count (0 = nothing to draw).
    line_vbuf: Option<wgpu::Buffer>,
    line_count: u32,
    camera: OrbitCamera,
    radius: f32,
}

impl Scene {
    fn build(device: &wgpu::Device, queue: &wgpu::Queue, mesh: &MeshData) -> Self {
        let vbuf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("verts"),
            size: std::mem::size_of_val(mesh.vertices.as_slice()) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        queue.write_buffer(&vbuf, 0, bytemuck::cast_slice(&mesh.vertices));

        let ibuf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("indices"),
            size: std::mem::size_of_val(mesh.indices.as_slice()) as u64,
            usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        queue.write_buffer(&ibuf, 0, bytemuck::cast_slice(&mesh.indices));

        let uniform = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("uniform"),
            size: std::mem::size_of::<Uniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("bgl"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("bg"),
            layout: &bgl,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform.as_entire_binding(),
            }],
        });

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("mesh-shader"),
            source: wgpu::ShaderSource::Wgsl(SHADER.into()),
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("layout"),
            bind_group_layouts: &[&bgl],
            immediate_size: 0,
        });
        let attrs = [
            wgpu::VertexAttribute {
                offset: 0,
                shader_location: 0,
                format: wgpu::VertexFormat::Float32x3,
            },
            wgpu::VertexAttribute {
                offset: 12,
                shader_location: 1,
                format: wgpu::VertexFormat::Float32x3,
            },
        ];
        let vbl = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &attrs,
        };
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("mesh-pipeline"),
            layout: Some(&layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs"),
                buffers: &[vbl],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: COLOR_FORMAT,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            primitive: wgpu::PrimitiveState {
                cull_mode: None, // scans have inconsistent winding; show both sides
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview_mask: None,
            cache: None,
        });

        // Line-strip pipeline for the centerline overlay — reuses the same
        // bind group layout (MVP); depth_compare Always so it draws on top.
        let line_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("line-shader"),
            source: wgpu::ShaderSource::Wgsl(LINE_SHADER.into()),
        });
        let line_attrs = [wgpu::VertexAttribute {
            offset: 0,
            shader_location: 0,
            format: wgpu::VertexFormat::Float32x3,
        }];
        let line_vbl = wgpu::VertexBufferLayout {
            array_stride: (std::mem::size_of::<f32>() * 3) as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &line_attrs,
        };
        let line_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("line-pipeline"),
            layout: Some(&layout),
            vertex: wgpu::VertexState {
                module: &line_shader,
                entry_point: Some("vs"),
                buffers: &[line_vbl],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &line_shader,
                entry_point: Some("fs"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: COLOR_FORMAT,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineStrip,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: false,
                depth_compare: wgpu::CompareFunction::Always,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview_mask: None,
            cache: None,
        });

        Self {
            device: device.clone(),
            queue: queue.clone(),
            pipeline,
            line_pipeline,
            bind_group,
            uniform,
            vbuf,
            ibuf,
            index_count: mesh.index_count(),
            line_vbuf: None,
            line_count: 0,
            camera: OrbitCamera::framing(mesh.center, mesh.radius),
            radius: mesh.radius,
        }
    }

    /// Replace the centerline overlay (display-frame points). Empty / <2
    /// points clears it.
    fn set_overlay(&mut self, points: &[[f32; 3]]) {
        if points.len() < 2 {
            self.line_vbuf = None;
            self.line_count = 0;
            return;
        }
        let vbuf = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("centerline"),
            size: std::mem::size_of_val(points) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        self.queue
            .write_buffer(&vbuf, 0, bytemuck::cast_slice(points));
        self.line_vbuf = Some(vbuf);
        self.line_count = points.len() as u32;
    }

    /// Swap in a new mesh (after a step-2 edit), recreating the vertex /
    /// index buffers but PRESERVING the orbit angle. The target + framing
    /// radius update, so edits that move the mesh (auto-center / orient)
    /// re-center the view while in-place edits (weld / simplify) keep the
    /// user's exact viewpoint.
    fn set_mesh(&mut self, mesh: &MeshData) {
        let vbuf = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("verts"),
            size: std::mem::size_of_val(mesh.vertices.as_slice()) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        self.queue
            .write_buffer(&vbuf, 0, bytemuck::cast_slice(&mesh.vertices));
        let ibuf = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("indices"),
            size: std::mem::size_of_val(mesh.indices.as_slice()) as u64,
            usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        self.queue
            .write_buffer(&ibuf, 0, bytemuck::cast_slice(&mesh.indices));
        self.vbuf = vbuf;
        self.ibuf = ibuf;
        self.index_count = mesh.index_count();
        self.radius = mesh.radius;
        self.camera.target = mesh.center;
    }

    /// Render the current camera into a fresh texture → Slint `Image`.
    fn render(&self) -> Result<slint::Image, String> {
        self.queue.write_buffer(
            &self.uniform,
            0,
            bytemuck::cast_slice(&[Uniforms {
                mvp: self.camera.mvp(self.radius),
            }]),
        );

        let extent = wgpu::Extent3d {
            width: RENDER_SIZE,
            height: RENDER_SIZE,
            depth_or_array_layers: 1,
        };
        let color = self.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("color"),
            size: extent,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: COLOR_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        });
        let color_view = color.create_view(&wgpu::TextureViewDescriptor::default());
        let depth = self.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("depth"),
            size: extent,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: DEPTH_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        let depth_view = depth.create_view(&wgpu::TextureViewDescriptor::default());

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("enc") });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &color_view,
                    resolve_target: None,
                    depth_slice: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.91,
                            g: 0.92,
                            b: 0.94,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &self.bind_group, &[]);
            pass.set_vertex_buffer(0, self.vbuf.slice(..));
            pass.set_index_buffer(self.ibuf.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..self.index_count, 0, 0..1);

            // Centerline overlay (on top, via the line pipeline's depth=Always).
            if let Some(line_vbuf) = &self.line_vbuf {
                if self.line_count >= 2 {
                    pass.set_pipeline(&self.line_pipeline);
                    pass.set_bind_group(0, &self.bind_group, &[]);
                    pass.set_vertex_buffer(0, line_vbuf.slice(..));
                    pass.draw(0..self.line_count, 0..1);
                }
            }
        }
        self.queue.submit(std::iter::once(encoder.finish()));

        slint::Image::try_from(color).map_err(|e| format!("Image::try_from failed: {e:?}"))
    }
}

/// Slint's wgpu device + queue, captured once at rendering setup so the
/// scan view can be built/re-rendered on demand later.
type GpuHandles = Rc<RefCell<Option<(wgpu::Device, wgpu::Queue)>>>;

/// The live step-2 scan-editing session (created when a scan is picked).
type EditCell = Rc<RefCell<Option<EditSession>>>;

/// Hand-off slot for an async Simplify outcome: `Ok((decimated mesh, target
/// face count, elapsed seconds))`, or `Err(())` if the worker panicked. The
/// background thread fills it; the UI-thread timer drains it (and always
/// clears `busy`, even on the panic path). `Arc<Mutex>` because it crosses
/// the thread boundary.
type SimplifyInbox = Arc<Mutex<Option<Result<(IndexedMesh, usize, f64), ()>>>>;

/// Hand-off slot for an async mold-generation outcome: the [`MoldOutputs`]
/// on success, or a user-facing error string (the engine error is
/// `to_string`'d on the worker so only `Send` data crosses the boundary).
/// The polling timer drains it on the UI thread.
type MoldsInbox = Arc<Mutex<Option<Result<MoldOutputs, String>>>>;

/// Hand-off slot for an async print-export outcome (the file copy can be
/// hundreds of MB at 0.5 mm, which would freeze the UI thread). Holds the
/// [`PrintExportReport`] on success or a user-facing error string.
type PrintInbox = Arc<Mutex<Option<Result<PrintExportReport, String>>>>;

/// Scans carry no unit metadata; the workshop scanner exports millimeters,
/// and the cast pipeline works in meters. (A units selector can override
/// this later; mm is the base_mold default + the cf-scan-prep default.)
const SCAN_SCALE_TO_M: f64 = 0.001;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Bring up Slint on the wgpu-28 backend so the rendering notifier hands
    // us a device/queue we can render the scan view with.
    slint::BackendSelector::new()
        .require_wgpu_28(slint::wgpu_28::WGPUConfiguration::default())
        .select()?;

    // In-memory project for the session (autosave/resume is a later
    // increment). Shared via RefCell so each step callback can mutate it.
    let project = Rc::new(RefCell::new(Project::new("Untitled")));
    // The screen being shown, 0-based into Step::ALL.
    let viewed = Rc::new(Cell::new(0usize));
    // GPU handles (filled at RenderingSetup) + the live scan scene.
    let gpu: GpuHandles = Rc::new(RefCell::new(None));
    let scene: Rc<RefCell<Option<Scene>>> = Rc::new(RefCell::new(None));
    // "Shape your piece" live preview: a second, independent scene (its own
    // mesh + orbit camera) for the textured plug render.
    let texture_scene: Rc<RefCell<Option<Scene>>> = Rc::new(RefCell::new(None));
    // The cached real-scan plug preview: the cleaned-scan flood-fill SDF +
    // centerline, built once on page entry (the flood-fill is the slow part)
    // so each inset / ridge edit only re-offsets + re-textures + re-meshes.
    // `None` until built / if the scan can't be loaded (→ the proxy fallback).
    let shape_preview: Rc<RefCell<Option<PlugPreview>>> = Rc::new(RefCell::new(None));
    // The chosen scan file + the step-2 edit session over its mesh. The
    // session is created at pick time and rendered on both step 1 (the
    // scan) and step 2 (the same mesh, being cleaned).
    let scan_path: Rc<RefCell<Option<PathBuf>>> = Rc::new(RefCell::new(None));
    let edit: EditCell = Rc::new(RefCell::new(None));
    // Inbox for an async Simplify result (mesh, target, elapsed) computed on
    // a background thread — the polling timer below applies it on the UI
    // thread, keeping the window responsive during the ~10-40s decimation.
    let simplify_inbox: SimplifyInbox = Arc::new(Mutex::new(None));
    // Inbox + start-time for an async mold-generation run (step 4). The run
    // is minutes-long; the polling timer below shows an elapsed-time line
    // while it works and applies the MoldOutputs when it lands.
    let molds_inbox: MoldsInbox = Arc::new(Mutex::new(None));
    let molds_start: Rc<Cell<Option<Instant>>> = Rc::new(Cell::new(None));
    // Step-5 print-export runs on a background thread too (the copy can be
    // hundreds of MB); the polling timer applies the result.
    let print_inbox: PrintInbox = Arc::new(Mutex::new(None));
    // Step-6 pour assistant: which layer is being poured (0-based, session
    // state — not persisted in the Project, which only records the final
    // completion) + the running pot-life countdown's deadline (None = no
    // timer running).
    let pour_current: Rc<Cell<usize>> = Rc::new(Cell::new(0));
    let pour_deadline: Rc<Cell<Option<Instant>>> = Rc::new(Cell::new(None));

    let ui = AppWindow::new()?;
    refresh(&ui, &project.borrow(), viewed.get());
    let weak = ui.as_weak();

    // ── poll the async-simplify inbox on the UI thread ──────────
    let simplify_timer = slint::Timer::default();
    {
        let (edit, scene, weak, inbox) = (
            edit.clone(),
            scene.clone(),
            weak.clone(),
            simplify_inbox.clone(),
        );
        simplify_timer.start(
            slint::TimerMode::Repeated,
            Duration::from_millis(80),
            move || {
                let Some(outcome) = inbox.lock().ok().and_then(|mut g| g.take()) else {
                    return;
                };
                if let Some(ui) = weak.upgrade() {
                    ui.set_busy(false);
                }
                match outcome {
                    Ok((mesh, target, secs)) => {
                        {
                            let mut e = edit.borrow_mut();
                            if let Some(s) = e.as_mut() {
                                s.apply_simplified(mesh, target);
                            }
                        }
                        apply_edit(
                            &weak,
                            &scene,
                            &edit,
                            &format!("✓ Simplified to {target} faces ({secs:.1}s)."),
                            false,
                        );
                    }
                    Err(()) => {
                        if let Some(ui) = weak.upgrade() {
                            ui.set_step_message(
                                "Simplify failed unexpectedly — try a higher target face count."
                                    .into(),
                            );
                            ui.set_step_message_is_error(true);
                        }
                    }
                }
            },
        );
    }

    // ── poll the async mold-generation inbox on the UI thread ──────
    // Every tick: drain a finished run (apply it + complete step 4), else
    // refresh the elapsed-time line so the user sees it's still working.
    let molds_timer = slint::Timer::default();
    {
        let (project, viewed, weak) = (project.clone(), viewed.clone(), weak.clone());
        let (inbox, start) = (molds_inbox.clone(), molds_start.clone());
        let (pour_current, pour_deadline) = (pour_current.clone(), pour_deadline.clone());
        molds_timer.start(
            slint::TimerMode::Repeated,
            Duration::from_millis(200),
            move || {
                // 1. A finished run lands here.
                if let Some(result) = inbox.lock().ok().and_then(|mut g| g.take()) {
                    start.set(None);
                    let Some(ui) = weak.upgrade() else { return };
                    ui.set_busy(false);
                    match result {
                        Ok(outputs) => {
                            let summary = format_molds_summary(&outputs);
                            // Bind to a local so the borrow_mut RefMut drops
                            // here — holding it as the match scrutinee would
                            // keep it alive across the arm, colliding with the
                            // project.borrow() in refresh (RefCell panic).
                            let recorded = project.borrow_mut().set_molds(outputs);
                            match recorded {
                                Ok(()) => {
                                    // New molds → a fresh pour (set_molds cleared
                                    // any prior print/pour); reset the step-6
                                    // progress so it starts at layer 1, not stale.
                                    pour_current.set(0);
                                    pour_deadline.set(None);
                                    ui.set_pour_timer_running(false);
                                    ui.set_molds_summary(summary.into());
                                    ui.set_step_message("✓ Molds ready — click Next →.".into());
                                    ui.set_step_message_is_error(false);
                                    refresh(&ui, &project.borrow(), viewed.get());
                                }
                                Err(e) => {
                                    ui.set_step_message(
                                        format!("Molds made, but couldn't record them: {e}").into(),
                                    );
                                    ui.set_step_message_is_error(true);
                                }
                            }
                        }
                        Err(msg) => {
                            ui.set_step_message(format!("Mold generation failed: {msg}").into());
                            ui.set_step_message_is_error(true);
                        }
                    }
                    return;
                }
                // 2. A run is in flight — show elapsed time (whole seconds, so
                // the property only changes ~1×/s even though we poll faster).
                if let Some(t0) = start.get() {
                    if let Some(ui) = weak.upgrade() {
                        let secs = t0.elapsed().as_secs();
                        ui.set_step_message(
                            format!(
                                "Making molds… {}:{:02} elapsed  \
                                 (this can take a while — the window stays responsive)",
                                secs / 60,
                                secs % 60,
                            )
                            .into(),
                        );
                        ui.set_step_message_is_error(false);
                    }
                }
            },
        );
    }

    // ── tick the step-6 pot-life countdown on the UI thread ───────
    // When a pour timer is running, refresh the remaining-time line (+ its
    // urgency colour) twice a second; idle otherwise.
    let pour_timer = slint::Timer::default();
    {
        let (weak, deadline) = (weak.clone(), pour_deadline.clone());
        pour_timer.start(
            slint::TimerMode::Repeated,
            Duration::from_millis(500),
            move || {
                let Some(dl) = deadline.get() else {
                    return; // no countdown running
                };
                let Some(ui) = weak.upgrade() else { return };
                let remaining = dl
                    .checked_duration_since(Instant::now())
                    .map_or(0, |d| i64::try_from(d.as_secs()).unwrap_or(i64::MAX));
                let cd = pour_countdown(remaining);
                ui.set_pour_timer_text(cd.text.into());
                ui.set_pour_timer_urgency(cd.urgency);
            },
        );
    }

    // ── poll the async print-export inbox on the UI thread ─────────
    let print_timer = slint::Timer::default();
    {
        let (project, viewed, weak, inbox) = (
            project.clone(),
            viewed.clone(),
            weak.clone(),
            print_inbox.clone(),
        );
        print_timer.start(
            slint::TimerMode::Repeated,
            Duration::from_millis(150),
            move || {
                let Some(result) = inbox.lock().ok().and_then(|mut g| g.take()) else {
                    return;
                };
                let Some(ui) = weak.upgrade() else { return };
                ui.set_busy(false);
                let (text, is_err) = match result {
                    Ok(report) => {
                        let dest = report.export.export_dir.clone();
                        let stl_count = report.stl_count;
                        let guide = if report.procedure_copied { " + the guide" } else { "" };
                        // Bind so the borrow_mut RefMut drops before refresh's borrow.
                        let recorded = project.borrow_mut().set_print(report.export);
                        match recorded {
                            Ok(()) => {
                                reveal_in_file_manager(&dest);
                                (
                                    format!(
                                        "✓ Saved {stl_count} file(s){guide} to {} — opening it now. \
                                         Print each piece, then click Next →.",
                                        dest.display(),
                                    ),
                                    false,
                                )
                            }
                            Err(e) => (format!("Copied the files, but couldn't record: {e}"), true),
                        }
                    }
                    Err(msg) => (format!("Couldn't save the files: {msg}"), true),
                };
                ui.set_step_message(text.into());
                ui.set_step_message_is_error(is_err);
                refresh(&ui, &project.borrow(), viewed.get());
            },
        );
    }

    // ── capture the wgpu device/queue once Slint sets rendering up ──
    {
        let gpu = gpu.clone();
        ui.window()
            .set_rendering_notifier(move |state, graphics_api| {
                if let (
                    slint::RenderingState::RenderingSetup,
                    slint::GraphicsAPI::WGPU28 { device, queue, .. },
                ) = (state, graphics_api)
                {
                    *gpu.borrow_mut() = Some((device.clone(), queue.clone()));
                }
            })?;
    }

    // ── step 3: silicone layer-stack editor ─────────────────────
    let catalog: Rc<Vec<(&'static str, &'static str)>> = Rc::new(silicone_catalog());
    {
        let names: Vec<SharedString> = catalog
            .iter()
            .map(|(_, display)| SharedString::from(*display))
            .collect();
        ui.set_material_names(ModelRc::new(VecModel::from(names)));
    }
    let idx_of = |key: &str| {
        catalog
            .iter()
            .position(|(k, _)| *k == key)
            .map_or(0, |i| i32::try_from(i).unwrap_or(0))
    };
    // Default stack ≈ the base_mold recipe — a sensible starting point the
    // user edits freely. Thickness is whole-mm here; load a .design.toml for
    // sub-mm precision.
    let layers_model = Rc::new(VecModel::from(vec![
        LayerRow {
            material_index: idx_of("ECOFLEX_00_30"),
            thickness_mm: 18,
            slacker_pct: 25,
        },
        LayerRow {
            material_index: idx_of("DRAGON_SKIN_10A"),
            thickness_mm: 8,
            slacker_pct: 0,
        },
        LayerRow {
            material_index: idx_of("DRAGON_SKIN_20A"),
            thickness_mm: 5,
            slacker_pct: 0,
        },
    ]));
    ui.set_layers(ModelRc::from(layers_model.clone()));
    {
        let layers_model = layers_model.clone();
        let default_idx = idx_of("DRAGON_SKIN_10A");
        ui.on_add_layer(move || {
            layers_model.push(LayerRow {
                material_index: default_idx,
                thickness_mm: 5,
                slacker_pct: 0,
            });
        });
    }
    {
        let layers_model = layers_model.clone();
        ui.on_remove_layer(move |idx| {
            let i = idx.max(0) as usize;
            // Keep at least one layer (the cast needs a stack).
            if i < layers_model.row_count() && layers_model.row_count() > 1 {
                layers_model.remove(i);
            }
        });
    }
    {
        let layers_model = layers_model.clone();
        ui.on_set_layer_material(move |idx, mat| {
            let i = idx.max(0) as usize;
            if let Some(mut row) = layers_model.row_data(i) {
                row.material_index = mat;
                layers_model.set_row_data(i, row);
            }
        });
    }
    {
        let layers_model = layers_model.clone();
        ui.on_set_layer_thickness(move |idx, mm| {
            let i = idx.max(0) as usize;
            if let Some(mut row) = layers_model.row_data(i) {
                row.thickness_mm = mm;
                layers_model.set_row_data(i, row);
            }
        });
    }
    {
        let layers_model = layers_model.clone();
        ui.on_set_layer_slacker(move |idx, pct| {
            let i = idx.max(0) as usize;
            if let Some(mut row) = layers_model.row_data(i) {
                row.slacker_pct = pct;
                layers_model.set_row_data(i, row);
            }
        });
    }
    // The step-4 part picker's models, declared here so the design handlers
    // below can rebuild them when the committed layer count changes. The
    // rows model (labels + checked) is parallel to the Rust-owned `Vec<PartId>`.
    let part_ids: Rc<RefCell<Vec<PartId>>> = Rc::new(RefCell::new(Vec::new()));
    let parts_model: Rc<VecModel<PartRow>> = Rc::new(VecModel::default());
    ui.set_parts(ModelRc::from(parts_model.clone()));
    rebuild_parts_model(&parts_model, &part_ids, layers_model.row_count());
    {
        let (project, viewed, weak) = (project.clone(), viewed.clone(), weak.clone());
        let (layers_model, catalog) = (layers_model.clone(), catalog.clone());
        let (parts_model, part_ids) = (parts_model.clone(), part_ids.clone());
        ui.on_use_design(move || {
            let layers: Vec<LayerDraft> = (0..layers_model.row_count())
                .filter_map(|i| layers_model.row_data(i))
                .map(|row| {
                    let key = usize::try_from(row.material_index)
                        .ok()
                        .and_then(|i| catalog.get(i))
                        .map_or("ECOFLEX_00_30", |(k, _)| *k)
                        .to_string();
                    LayerDraft {
                        thickness_m: f64::from(row.thickness_mm) / 1000.0,
                        material_key: key,
                        slacker_fraction: f64::from(row.slacker_pct) / 100.0,
                    }
                })
                .collect();
            // The cavity inset is owned by the "Shape your piece" step; the
            // layer stack builds outward off that shaped plug.
            let cavity_inset_m = project.borrow().plug().map_or(0.0, |pl| pl.cavity_inset_m);
            let draft = DesignDraft {
                cavity_inset_m,
                layers,
            };
            let outcome = apply_design_draft(&mut project.borrow_mut(), draft);
            // The committed design fixes the layer count → rebuild the part
            // picker (all parts checked) to match.
            if let Some(design) = project.borrow().design() {
                rebuild_parts_model(&parts_model, &part_ids, design.layers.len());
            }
            update(&weak, &project, viewed.get(), &outcome);
        });
    }

    // ── step 3 "Shape your piece": the surface-ridge grip-ring editor ──
    // The rings VecModel mirrors the layer stack. Seeded from the validated
    // RidgeOptions::default() rings, converted to the UI's integer units
    // (position/width as %, depth in tenths-of-mm). commit-plug reads this
    // model + the ridge property knobs to build the RidgeOptions it commits.
    let rings_model = Rc::new(VecModel::from(
        RidgeOptions::default()
            .rings
            .iter()
            .map(ring_row_from_ridge)
            .collect::<Vec<_>>(),
    ));
    ui.set_ridge_rings(ModelRc::from(rings_model.clone()));
    {
        let rings_model = rings_model.clone();
        ui.on_add_ring(move || {
            // A new ring lands mid-channel, a conservative 2 mm deep.
            rings_model.push(RingRow {
                position_pct: 50,
                depth_tenths_mm: 20,
                width_pct: 4,
            });
        });
    }
    {
        let rings_model = rings_model.clone();
        ui.on_remove_ring(move |idx| {
            let i = idx.max(0) as usize;
            if i < rings_model.row_count() {
                rings_model.remove(i);
            }
        });
    }
    {
        let rings_model = rings_model.clone();
        ui.on_set_ring_position(move |idx, pct| {
            let i = idx.max(0) as usize;
            if let Some(mut row) = rings_model.row_data(i) {
                row.position_pct = pct;
                rings_model.set_row_data(i, row);
            }
        });
    }
    {
        let rings_model = rings_model.clone();
        ui.on_set_ring_depth(move |idx, tenths| {
            let i = idx.max(0) as usize;
            if let Some(mut row) = rings_model.row_data(i) {
                row.depth_tenths_mm = tenths;
                rings_model.set_row_data(i, row);
            }
        });
    }
    {
        let rings_model = rings_model.clone();
        ui.on_set_ring_width(move |idx, pct| {
            let i = idx.max(0) as usize;
            if let Some(mut row) = rings_model.row_data(i) {
                row.width_pct = pct;
                rings_model.set_row_data(i, row);
            }
        });
    }

    // ── step 3 "Shape your piece": commit the plug (Continue) + advance ──
    // Build the PlugDraft from the cavity inset (the spin passes its mm) + the
    // ridge controls, then advance to the layer stack.
    {
        let (project, viewed, weak) = (project.clone(), viewed.clone(), weak.clone());
        let rings_model_c = rings_model.clone();
        ui.on_commit_plug(move |cavity_mm| {
            let Some(ui) = weak.upgrade() else { return };
            let plug = PlugDraft {
                cavity_inset_m: f64::from(cavity_mm) / 1000.0,
                ridges: ridge_options_from_ui(&ui, &rings_model_c),
            };
            let outcome = apply_plug(&mut project.borrow_mut(), plug);
            if outcome.is_ok() {
                let _ = project.borrow_mut().advance();
            }
            viewed.set(project.borrow().current_step().index());
            update(&weak, &project, viewed.get(), &outcome);
        });
    }

    // ── step 5 (Make molds): the "parts to generate" picker callbacks ──
    // (the models are declared above the design handlers so they can rebuild
    // them; make-molds reads the checked mask + the ids to build the selection.)
    {
        let parts_model = parts_model.clone();
        ui.on_set_part_checked(move |idx, checked| {
            let i = idx.max(0) as usize;
            if let Some(mut row) = parts_model.row_data(i) {
                row.checked = checked;
                parts_model.set_row_data(i, row);
            }
        });
    }
    {
        let parts_model = parts_model.clone();
        ui.on_select_all_parts(move || set_all_parts(&parts_model, true));
    }
    {
        let parts_model = parts_model.clone();
        ui.on_select_no_parts(move || set_all_parts(&parts_model, false));
    }

    // ── step 3 "Shape your piece": the live plug preview ──
    // On page entry, (re)build the cached real-scan SDF from the cleaned scan;
    // then render. The build is the slow part — done once here, not per edit.
    {
        let (weak, gpu, texture_scene, shape_preview, project) = (
            weak.clone(),
            gpu.clone(),
            texture_scene.clone(),
            shape_preview.clone(),
            project.clone(),
        );
        let rings_model = rings_model.clone();
        ui.on_shape_enter(move || {
            let Some(ui) = weak.upgrade() else { return };
            // Build the cache from the project's cleaned scan + prep. On any
            // failure leave it None → the proxy fallback still previews.
            let built = project
                .borrow()
                .prep()
                .map(|prep| PlugPreview::load(&prep.cleaned_stl, &prep.prep_toml));
            *shape_preview.borrow_mut() = match built {
                Some(Ok(p)) => Some(p),
                _ => None,
            };
            refresh_shape_preview(&ui, &gpu, &texture_scene, &shape_preview, &rings_model);
        });
    }
    // On any inset / ridge edit, re-mesh from the cached SDF (cheap).
    {
        let (weak, gpu, texture_scene, shape_preview) = (
            weak.clone(),
            gpu.clone(),
            texture_scene.clone(),
            shape_preview.clone(),
        );
        let rings_model = rings_model.clone();
        ui.on_shape_changed(move || {
            if let Some(ui) = weak.upgrade() {
                refresh_shape_preview(&ui, &gpu, &texture_scene, &shape_preview, &rings_model);
            }
        });
    }
    {
        let (weak, texture_scene) = (weak.clone(), texture_scene.clone());
        ui.on_texture_orbit(move |dx, dy| {
            let mut guard = texture_scene.borrow_mut();
            let Some(s) = guard.as_mut() else {
                return;
            };
            s.camera.orbit(dx, dy);
            if let (Some(ui), Ok(img)) = (weak.upgrade(), s.render()) {
                ui.set_texture_preview(img);
            }
        });
    }

    // ── navigation (gated by the wizard state) ──────────────────
    {
        let (project, viewed, weak) = (project.clone(), viewed.clone(), weak.clone());
        let pour_current = pour_current.clone();
        ui.on_back(move || {
            viewed.set(viewed.get().saturating_sub(1));
            if let Some(ui) = weak.upgrade() {
                ui.set_step_message(SharedString::default());
                let p = project.borrow();
                refresh(&ui, &p, viewed.get());
                sync_pour_ui(&ui, &p, pour_current.get());
            }
        });
    }
    {
        let (project, viewed, weak) = (project.clone(), viewed.clone(), weak.clone());
        let pour_current = pour_current.clone();
        ui.on_next(move || {
            let v = viewed.get();
            let step = Step::ALL.get(v).copied().unwrap_or(Step::FIRST);
            // Respect the gate even if the disabled button somehow fires.
            if nav_state(&project.borrow(), step).can_next {
                viewed.set(v + 1);
            }
            if let Some(ui) = weak.upgrade() {
                ui.set_step_message(SharedString::default());
                let p = project.borrow();
                refresh(&ui, &p, viewed.get());
                // Entering step 6 populates the pour panel from the plan.
                sync_pour_ui(&ui, &p, pour_current.get());
            }
        });
    }
    ui.on_help(|| {
        // Placeholder — a help panel / per-step guidance arrives later.
    });

    // ── first-launch waiver/age gate ──────────────────────────────
    // Accept clears the overlay for this session only (never persisted, so the
    // gate reappears every launch). Quit closes the app straight from the gate.
    {
        let weak = weak.clone();
        ui.on_accept_waiver(move || {
            if let Some(ui) = weak.upgrade() {
                ui.set_waiver_accepted(true);
            }
        });
    }
    ui.on_quit_app(|| {
        let _ = slint::quit_event_loop();
    });

    // ── step 1: pick a scan → record it + open the live edit session ──
    {
        let (project, viewed, weak) = (project.clone(), viewed.clone(), weak.clone());
        let (gpu, scene) = (gpu.clone(), scene.clone());
        let (scan_path, edit) = (scan_path.clone(), edit.clone());
        ui.on_pick_scan(move || {
            let Some(path) = rfd::FileDialog::new()
                .set_title("Choose your 3D scan")
                .add_filter("3D scan", &["stl", "obj", "ply", "3mf"])
                .pick_file()
            else {
                return;
            };
            let outcome = apply_scan(&mut project.borrow_mut(), &path);

            // On success, open the edit session over the scan and render
            // its working mesh. The same session + scene back step 1 (the
            // scan) and step 2 (cleaning it). GPU handles are present once
            // the window has rendered a frame.
            if outcome.is_ok() {
                *scan_path.borrow_mut() = Some(path.clone());
                match EditSession::load(&path, SCAN_SCALE_TO_M) {
                    Ok(session) => {
                        if let Some((device, queue)) = gpu.borrow().clone() {
                            let md = mesh_data_from_indexed(&session.display_mesh());
                            let built = Scene::build(&device, &queue, &md);
                            if let (Some(ui), Ok(img)) = (weak.upgrade(), built.render()) {
                                ui.set_scan_view(img);
                                ui.set_edit_stats(stats_line(&session).into());
                            }
                            *scene.borrow_mut() = Some(built);
                        }
                        *edit.borrow_mut() = Some(session);
                    }
                    Err(e) => eprintln!("scan loaded but edit session failed: {e}"),
                }
            }
            update(&weak, &project, viewed.get(), &outcome);
        });
    }

    // ── drag in the 3D view → orbit the camera + re-render ──
    {
        let (scene, weak) = (scene.clone(), weak.clone());
        ui.on_orbit(move |dx, dy| {
            let mut guard = scene.borrow_mut();
            let Some(s) = guard.as_mut() else {
                return;
            };
            s.camera.orbit(dx, dy);
            if let (Some(ui), Ok(img)) = (weak.upgrade(), s.render()) {
                ui.set_scan_view(img);
            }
        });
    }

    // ── step 2: live edit ops (drive the EditSession, re-render) ──
    {
        let (scene, edit, weak) = (scene.clone(), edit.clone(), weak.clone());
        ui.on_weld(move || {
            let msg = {
                let mut e = edit.borrow_mut();
                let Some(s) = e.as_mut() else { return };
                let (before, after) = s.weld();
                format!("✓ Welded vertices: {before} → {after}")
            };
            apply_edit(&weak, &scene, &edit, &msg, false);
        });
    }
    {
        let (edit, weak, inbox) = (edit.clone(), weak.clone(), simplify_inbox.clone());
        ui.on_simplify(move |target| {
            let target = usize::try_from(target).unwrap_or(200_000);
            // Snapshot the working mesh, then decimate on a background thread
            // (it's the only ~10-40s op) so the window stays responsive; the
            // polling timer applies the result.
            let working = {
                let e = edit.borrow();
                let Some(s) = e.as_ref() else { return };
                s.working_clone()
            };
            if let Some(ui) = weak.upgrade() {
                ui.set_busy(true);
                ui.set_step_message(
                    format!("Simplifying to {target} faces… (this can take ~10–40 s)").into(),
                );
                ui.set_step_message_is_error(false);
            }
            let inbox = inbox.clone();
            std::thread::spawn(move || {
                // catch_unwind so a (near-impossible) decimation panic still
                // reports back — otherwise busy would stick + lock the UI.
                let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    run_simplify(&working, target)
                }));
                let result = outcome
                    .map(|(mesh, secs)| (mesh, target, secs))
                    .map_err(|_| ());
                if let Ok(mut g) = inbox.lock() {
                    *g = Some(result);
                }
            });
        });
    }
    {
        let (scene, edit, weak) = (scene.clone(), edit.clone(), weak.clone());
        ui.on_find_floor(move || {
            let (msg, is_err) = {
                let mut e = edit.borrow_mut();
                let Some(s) = e.as_mut() else { return };
                let scan = s.detect_caps();
                if scan.looks_unwelded {
                    (
                        "Looks like a raw scan — click Weld first, then Find floor.".to_string(),
                        true,
                    )
                } else if scan.loop_count == 0 {
                    ("No open edges found to stand it on.".to_string(), true)
                } else {
                    match s.level_to_floor() {
                        Some(tilt) => (
                            format!(
                                "✓ Found floor — {} open loop(s), {}-segment centerline. \
                                 Stood upright (corrected {tilt:.0}° tilt).",
                                scan.loop_count, scan.centerline_segments,
                            ),
                            false,
                        ),
                        None => (
                            format!(
                                "Found {} loop(s) but couldn't trace a centerline to level by.",
                                scan.loop_count,
                            ),
                            true,
                        ),
                    }
                }
            };
            apply_edit(&weak, &scene, &edit, &msg, is_err);
        });
    }
    {
        let (scene, edit, weak) = (scene.clone(), edit.clone(), weak.clone());
        ui.on_apply_trim(move |tip, floor| {
            let msg = {
                let mut e = edit.borrow_mut();
                let Some(s) = e.as_mut() else { return };
                s.apply_trim(f64::from(tip), f64::from(floor));
                // Re-level onto the new cut floor (the predicted/reconstructed
                // floor path now that there's a trim).
                s.level_to_floor();
                format!("✓ Trimmed — tip {tip} mm, floor {floor} mm — and re-leveled.")
            };
            apply_edit(&weak, &scene, &edit, &msg, false);
        });
    }
    {
        let (scene, edit, weak) = (scene.clone(), edit.clone(), weak.clone());
        ui.on_apply_reconstruct(move |shape_idx, reference_mm| {
            let shape = match shape_idx {
                1 => ReconstructShape::Taper,
                2 => ReconstructShape::Extrapolate,
                _ => ReconstructShape::Constant,
            };
            let (msg, is_err) = {
                let mut e = edit.borrow_mut();
                let Some(s) = e.as_mut() else { return };
                if s.apply_reconstruct(f64::from(reference_mm), shape) {
                    s.level_to_floor();
                    (
                        format!("✓ Reconstructed the floor ({reference_mm} mm reference zone)."),
                        false,
                    )
                } else {
                    (
                        "Apply a floor trim first, then reconstruct.".to_string(),
                        true,
                    )
                }
            };
            apply_edit(&weak, &scene, &edit, &msg, is_err);
        });
    }
    {
        let (scene, edit, weak) = (scene.clone(), edit.clone(), weak.clone());
        ui.on_reset_edit(move || {
            {
                let mut e = edit.borrow_mut();
                let Some(s) = e.as_mut() else { return };
                s.reset();
            }
            apply_edit(&weak, &scene, &edit, "↺ Reset to the original scan", false);
        });
    }
    {
        let (project, viewed, weak) = (project.clone(), viewed.clone(), weak.clone());
        let (edit, scan_path) = (edit.clone(), scan_path.clone());
        ui.on_save(move |smoothing| {
            // The cast needs a centerline; require Find floor first (a clean
            // prompt before the engine would refuse + before the save freeze).
            if !edit
                .borrow()
                .as_ref()
                .is_some_and(EditSession::has_centerline)
            {
                if let Some(ui) = weak.upgrade() {
                    ui.set_step_message(
                        "Click Find floor first — it traces the centerline the cast needs.".into(),
                    );
                    ui.set_step_message_is_error(true);
                }
                return;
            }
            let Some(path) = scan_path.borrow().clone() else {
                return;
            };
            let default_dir = path
                .parent()
                .map(|p| p.to_path_buf())
                .unwrap_or_else(|| PathBuf::from("."));
            let stem = path
                .file_stem()
                .and_then(|s| s.to_str())
                .unwrap_or("scan")
                .to_string();

            // Don't silently clobber existing outputs: if they're already
            // there, let the user overwrite, pick a different folder, or
            // cancel.
            let output_dir = {
                let exists = default_dir.join(format!("{stem}.cleaned.stl")).exists()
                    || default_dir.join(format!("{stem}.prep.toml")).exists();
                if exists {
                    let choice = rfd::MessageDialog::new()
                        .set_level(rfd::MessageLevel::Warning)
                        .set_title("Output already exists")
                        .set_description(format!(
                            "{stem}.cleaned.stl / .prep.toml already exist in {}.\n\n\
                             Overwrite them, or choose a different folder?",
                            default_dir.display(),
                        ))
                        .set_buttons(rfd::MessageButtons::YesNoCancel)
                        .show();
                    match choice {
                        rfd::MessageDialogResult::Yes => default_dir,
                        rfd::MessageDialogResult::No => {
                            match rfd::FileDialog::new()
                                .set_title("Choose a folder to save the cleaned scan")
                                .pick_folder()
                            {
                                Some(dir) => dir,
                                None => {
                                    if let Some(ui) = weak.upgrade() {
                                        ui.set_step_message("Save cancelled.".into());
                                        ui.set_step_message_is_error(false);
                                    }
                                    return;
                                }
                            }
                        }
                        _ => {
                            if let Some(ui) = weak.upgrade() {
                                ui.set_step_message("Save cancelled.".into());
                                ui.set_step_message_is_error(false);
                            }
                            return;
                        }
                    }
                } else {
                    default_dir
                }
            };
            let smoothing = usize::try_from(smoothing).unwrap_or(0);
            let result = {
                let guard = edit.borrow();
                let Some(s) = guard.as_ref() else {
                    return;
                };
                s.save(&output_dir, &stem, "mm", smoothing)
            };
            let (text, is_err) = match result {
                Ok(report) => {
                    // Accept the written output as the cleaned-scan input —
                    // this completes step 2 and unblocks Next.
                    match apply_prep(
                        &mut project.borrow_mut(),
                        &report.cleaned_stl,
                        &report.prep_toml,
                    ) {
                        Ok(_) => (
                            format!(
                                "✓ Saved {stem}.cleaned.stl ({} faces) + {stem}.prep.toml — \
                                 step complete, click Next →.",
                                report.face_count,
                            ),
                            false,
                        ),
                        Err(e) => (
                            format!("Saved the files, but they didn't validate: {e}"),
                            true,
                        ),
                    }
                }
                Err(e) => (format!("Save failed: {e}"), true),
            };
            if let Some(ui) = weak.upgrade() {
                ui.set_step_message(text.into());
                ui.set_step_message_is_error(is_err);
                refresh(&ui, &project.borrow(), viewed.get());
            }
        });
    }
    {
        let (project, viewed, weak) = (project.clone(), viewed.clone(), weak.clone());
        let (parts_model, part_ids) = (parts_model.clone(), part_ids.clone());
        ui.on_pick_design(move || {
            let Some(path) = rfd::FileDialog::new()
                .set_title("Choose the design file")
                .add_filter("design", &["toml"])
                .pick_file()
            else {
                return;
            };
            let outcome = apply_design(&mut project.borrow_mut(), &path);
            // A loaded design fixes the layer count → rebuild the part picker.
            if let Some(design) = project.borrow().design() {
                rebuild_parts_model(&parts_model, &part_ids, design.layers.len());
            }
            update(&weak, &project, viewed.get(), &outcome);
        });
    }

    // ── step 5: make the molds (run the cast async) ─────────────
    {
        let (project, weak) = (project.clone(), weak.clone());
        let (inbox, start) = (molds_inbox.clone(), molds_start.clone());
        let (parts_model, part_ids) = (parts_model.clone(), part_ids.clone());
        ui.on_make_molds(move |quality_idx| {
            // Re-entrancy guard: a run already in flight (the UI disables the
            // button via `busy`, but a queued click could still arrive). Don't
            // spawn a second 15-min cast into the same output dir.
            if start.get().is_some() {
                return;
            }
            // Need at least one part selected (an empty selection would mesh
            // nothing).
            if !any_part_checked(&parts_model) {
                if let Some(ui) = weak.upgrade() {
                    ui.set_step_message("Pick at least one part to generate.".into());
                    ui.set_step_message_is_error(true);
                }
                return;
            }
            // Pull the inputs from the project: the cleaned scan + prep
            // (step 2) and the in-app design (step 3). The wizard gate
            // ensures both exist, but guard anyway.
            let inputs = {
                let p = project.borrow();
                match (p.prep(), p.design()) {
                    (Some(prep), Some(design)) => Some((
                        prep.cleaned_stl.clone(),
                        prep.prep_toml.clone(),
                        design.clone(),
                    )),
                    _ => None,
                }
            };
            let Some((cleaned_stl, prep_toml, draft)) = inputs else {
                if let Some(ui) = weak.upgrade() {
                    ui.set_step_message(
                        "Finish steps 2 and 3 first (clean the scan, choose a design).".into(),
                    );
                    ui.set_step_message_is_error(true);
                }
                return;
            };
            let cell_size_m = cell_size_m_for_quality(quality_idx);
            // The surface ridges were committed on "Shape your piece" as part
            // of the plug. The one field rides every offset (plug + shells).
            let ridges = project
                .borrow()
                .plug()
                .map(|pl| pl.ridges.clone())
                .unwrap_or_default();
            // Snapshot the part picker into a PartSelection (all checked →
            // the full-cast path; a subset skips the rest).
            let selection = part_selection_from_ui(&parts_model, &part_ids);

            if let Some(ui) = weak.upgrade() {
                ui.set_busy(true);
                ui.set_molds_summary(SharedString::default());
                ui.set_step_message("Making molds… starting…".into());
                ui.set_step_message_is_error(false);
            }
            start.set(Some(Instant::now()));
            let inbox = inbox.clone();
            std::thread::spawn(move || {
                // catch_unwind so a panic in the cast still reports back —
                // otherwise `busy` would stick and lock the UI.
                let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    generate_molds_for_design(
                        &cleaned_stl,
                        &prep_toml,
                        &draft,
                        cell_size_m,
                        &ridges,
                        &selection,
                        CENDRILLON_CAST_MODE,
                        None,
                    )
                }));
                let result = match outcome {
                    Ok(Ok(o)) => Ok(o),
                    Ok(Err(e)) => Err(e.to_string()),
                    Err(_) => Err("internal error (panic) during mold generation".to_string()),
                };
                if let Ok(mut g) = inbox.lock() {
                    *g = Some(result);
                }
            });
        });
    }

    // ── step 5: save the printable files into a chosen folder ───
    {
        let (project, weak, inbox) = (project.clone(), weak.clone(), print_inbox.clone());
        ui.on_export_print(move || {
            // The molds must be made (the wizard gate ensures it on step 5).
            let Some(molds) = project.borrow().molds().cloned() else {
                if let Some(ui) = weak.upgrade() {
                    ui.set_step_message("Make the molds first (step 4).".into());
                    ui.set_step_message_is_error(true);
                }
                return;
            };
            let Some(dest) = rfd::FileDialog::new()
                .set_title("Choose a folder to save the printable files")
                .pick_folder()
            else {
                return; // cancelled
            };
            // The copy can be hundreds of MB at 0.5 mm — run it off the UI
            // thread so the window doesn't freeze; the print timer applies the
            // result. busy gates Save/Back/Next for the duration.
            if let Some(ui) = weak.upgrade() {
                ui.set_busy(true);
                ui.set_step_message("Saving the printable files…".into());
                ui.set_step_message_is_error(false);
            }
            let inbox = inbox.clone();
            std::thread::spawn(move || {
                let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    export_print_package(&molds, &dest)
                }));
                let result = match outcome {
                    Ok(Ok(r)) => Ok(r),
                    Ok(Err(e)) => Err(e.to_string()),
                    Err(_) => Err("internal error (panic) during export".to_string()),
                };
                if let Ok(mut g) = inbox.lock() {
                    *g = Some(result);
                }
            });
        });
    }
    {
        let (project, weak) = (project.clone(), weak.clone());
        ui.on_open_export_folder(move || {
            if let Some(dir) = project.borrow().print().map(|p| p.export_dir.clone()) {
                reveal_in_file_manager(&dir);
            } else if let Some(ui) = weak.upgrade() {
                ui.set_step_message("Nothing exported yet — save the files first.".into());
                ui.set_step_message_is_error(true);
            }
        });
    }

    // ── step 6: start the current layer's pot-life countdown ────
    {
        let (project, weak) = (project.clone(), weak.clone());
        let (pour_current, pour_deadline) = (pour_current.clone(), pour_deadline.clone());
        ui.on_start_pour_timer(move || {
            let mins = project.borrow().molds().and_then(|m| {
                m.pour_plan
                    .steps
                    .get(pour_current.get())
                    .map(|s| s.pot_life_minutes)
            });
            let Some(mins) = mins else { return };
            pour_deadline.set(Some(
                Instant::now() + Duration::from_secs(u64::from(mins) * 60),
            ));
            if let Some(ui) = weak.upgrade() {
                ui.set_pour_timer_running(true);
                // Paint the full time immediately (the tick is up to 500ms away).
                let cd = pour_countdown(i64::from(mins) * 60);
                ui.set_pour_timer_text(cd.text.into());
                ui.set_pour_timer_urgency(cd.urgency);
            }
        });
    }
    // ── step 6: mark the current layer poured (advance / finish) ──
    {
        let (project, viewed, weak) = (project.clone(), viewed.clone(), weak.clone());
        let (pour_current, pour_deadline) = (pour_current.clone(), pour_deadline.clone());
        ui.on_mark_poured(move || {
            let total = project
                .borrow()
                .molds()
                .map_or(0, |m| m.pour_plan.steps.len());
            if total == 0 {
                return;
            }
            let next = pour_current.get() + 1;
            pour_current.set(next);
            pour_deadline.set(None); // stop any running countdown

            // The last layer's pour finishes the project.
            let completed = if next >= total {
                let recorded = project.borrow_mut().set_pour(PourRecord {
                    layers_poured: total,
                });
                Some(match recorded {
                    Ok(()) => (
                        "🎉 All layers poured — your device is complete!".to_string(),
                        false,
                    ),
                    Err(e) => (format!("Couldn't record completion: {e}"), true),
                })
            } else {
                None
            };

            if let Some(ui) = weak.upgrade() {
                ui.set_pour_timer_running(false);
                ui.set_pour_timer_text(SharedString::default());
                if let Some((msg, is_err)) = completed {
                    ui.set_step_message(msg.into());
                    ui.set_step_message_is_error(is_err);
                }
                let p = project.borrow();
                sync_pour_ui(&ui, &p, pour_current.get());
                refresh(&ui, &p, viewed.get());
            }
        });
    }

    ui.run()?;
    Ok(())
}

/// Push the step-6 pour-assistant state (plan overview, active-layer line,
/// finished flag) for `current` (0-based) into the UI. The live countdown
/// text is driven separately by the pour timer.
/// Rebuild the step-4 part picker for a `layer_count`-layer design: refill
/// the Rust-owned `ids` with the enumerated [`PartId`]s and the UI `rows`
/// with their labels (all checked = full cast). Called whenever the
/// committed design changes.
fn rebuild_parts_model(rows: &VecModel<PartRow>, ids: &RefCell<Vec<PartId>>, layer_count: usize) {
    let parts = enumerate_parts(layer_count, CENDRILLON_CAST_MODE);
    let new_rows: Vec<PartRow> = parts
        .iter()
        .map(|(_, label)| PartRow {
            label: SharedString::from(label.as_str()),
            checked: true,
        })
        .collect();
    *ids.borrow_mut() = parts.into_iter().map(|(id, _)| id).collect();
    rows.set_vec(new_rows);
}

/// Set every part row's checkbox to `checked` (the All / None buttons).
fn set_all_parts(rows: &VecModel<PartRow>, checked: bool) {
    for i in 0..rows.row_count() {
        if let Some(mut row) = rows.row_data(i) {
            row.checked = checked;
            rows.set_row_data(i, row);
        }
    }
}

/// Read the part picker into a [`PartSelection`]: the checked rows mapped
/// back through the parallel `ids`. All checked → [`PartSelection::all`].
fn part_selection_from_ui(rows: &VecModel<PartRow>, ids: &RefCell<Vec<PartId>>) -> PartSelection {
    let ids = ids.borrow();
    let parts: Vec<(PartId, String)> = ids.iter().map(|id| (*id, String::new())).collect();
    let checked: Vec<bool> = (0..rows.row_count())
        .map(|i| rows.row_data(i).is_some_and(|r| r.checked))
        .collect();
    part_selection_from_checks(&parts, &checked, CENDRILLON_CAST_MODE)
}

/// `true` when at least one part is checked (make-molds needs ≥1 piece).
fn any_part_checked(rows: &VecModel<PartRow>) -> bool {
    (0..rows.row_count()).any(|i| rows.row_data(i).is_some_and(|r| r.checked))
}

/// A grip-ring's owned [`RidgeRing`] → the UI's integer-unit [`RingRow`]
/// (axial fraction → %, half-width fraction → %, depth m → tenths-of-mm).
fn ring_row_from_ridge(ring: &RidgeRing) -> RingRow {
    RingRow {
        position_pct: (ring.position_frac * 100.0).round() as i32,
        depth_tenths_mm: (ring.depth_m * 10_000.0).round() as i32,
        width_pct: (ring.half_width_frac * 100.0).round() as i32,
    }
}

/// Read the "Shape your piece" ridge controls off the UI + the rings VecModel
/// into an owned [`RidgeOptions`] (the inverse of [`ring_row_from_ridge`] +
/// the property getters). Integer UI units convert back to meters/fractions
/// (percents ÷100, tenths-of-mm ÷10 000 → meters), then the pure, tested
/// [`gate_ridge_options`] applies the per-feature toggles (off → that feature
/// contributes nothing). The master `ridges-enabled` gates the whole field on
/// top (off → the engine maps to the no-op canal-off cast).
fn ridge_options_from_ui(ui: &AppWindow, rings: &VecModel<RingRow>) -> RidgeOptions {
    let tenths_mm_to_m = |t: i32| f64::from(t) / 10_000.0;
    let rings = (0..rings.row_count())
        .filter_map(|i| rings.row_data(i))
        .map(|row| RidgeRing {
            position_frac: f64::from(row.position_pct) / 100.0,
            depth_m: tenths_mm_to_m(row.depth_tenths_mm),
            half_width_frac: f64::from(row.width_pct) / 100.0,
        })
        .collect();
    gate_ridge_options(RidgeControls {
        enabled: ui.get_ridges_enabled(),
        rings_enabled: ui.get_rings_enabled(),
        rings,
        texture_enabled: ui.get_texture_enabled(),
        texture_depth_m: tenths_mm_to_m(ui.get_ridge_texture_depth_tenths_mm()),
        texture_spacing_m: tenths_mm_to_m(ui.get_ridge_texture_spacing_tenths_mm()),
        side_pinch_enabled: ui.get_side_pinch_enabled(),
        side_pinch_depth_m: tenths_mm_to_m(ui.get_ridge_side_pinch_tenths_mm()),
        tip_relief_enabled: ui.get_tip_relief_enabled(),
        tip_relief_depth_m: tenths_mm_to_m(ui.get_ridge_tip_relief_tenths_mm()),
        orientation_enabled: ui.get_orientation_enabled(),
        orientation_deg: f64::from(ui.get_ridge_orientation_deg()),
    })
}

/// Render `mesh` into the shared `texture-preview` image (preserving orbit on
/// a re-render). A no-op until the GPU handles are ready.
fn render_texture_preview(
    ui: &AppWindow,
    gpu: &GpuHandles,
    texture_scene: &RefCell<Option<Scene>>,
    mesh: &cortenforge::mesh_types::IndexedMesh,
) {
    let Some((device, queue)) = gpu.borrow().clone() else {
        return;
    };
    let md = mesh_data_from_indexed(mesh);
    let img = {
        let mut guard = texture_scene.borrow_mut();
        match guard.as_mut() {
            Some(s) => {
                s.set_mesh(&md);
                s.render()
            }
            None => {
                let s = Scene::build(&device, &queue, &md);
                let r = s.render();
                *guard = Some(s);
                r
            }
        }
    };
    if let Ok(img) = img {
        ui.set_texture_preview(img);
    }
}

/// Regenerate the "Shape your piece" plug preview from the current cavity
/// inset + ridge controls. Uses the cached real-scan SDF when available (the
/// faithful plug); falls back to the flat-floor proxy when the scan couldn't
/// be loaded.
fn refresh_shape_preview(
    ui: &AppWindow,
    gpu: &GpuHandles,
    texture_scene: &RefCell<Option<Scene>>,
    shape_preview: &RefCell<Option<PlugPreview>>,
    rings: &VecModel<RingRow>,
) {
    let ridges = ridge_options_from_ui(ui, rings);
    let cavity_inset_m = f64::from(ui.get_cavity_inset_mm()) / 1000.0;
    let mesh = match shape_preview.borrow().as_ref() {
        Some(preview) => preview.mesh(&ridges, cavity_inset_m),
        None => proxy_preview_mesh(&ridges),
    };
    render_texture_preview(ui, gpu, texture_scene, &mesh);
}

fn sync_pour_ui(ui: &AppWindow, project: &Project, current: usize) {
    let (plan_text, active_text, complete) = match project.molds() {
        Some(m) => (
            format_pour_plan(&m.pour_plan),
            format_pour_active(&m.pour_plan, current),
            project.pour().is_some(),
        ),
        None => (String::new(), String::new(), false),
    };
    ui.set_pour_plan_text(plan_text.into());
    ui.set_pour_active_text(active_text.into());
    ui.set_pour_complete(complete);
}

/// Open `dir` in the OS file manager (best-effort; a failure to spawn is
/// silently ignored — it's a convenience, not part of the workflow).
fn reveal_in_file_manager(dir: &Path) {
    #[cfg(target_os = "macos")]
    let program = "open";
    #[cfg(target_os = "windows")]
    let program = "explorer";
    #[cfg(all(unix, not(target_os = "macos")))]
    let program = "xdg-open";
    let _ = std::process::Command::new(program).arg(dir).spawn();
}

/// After a step action: surface its message + re-render from the project.
fn update(
    weak: &slint::Weak<AppWindow>,
    project: &Rc<RefCell<Project>>,
    viewed_idx: usize,
    outcome: &StepOutcome,
) {
    if let Some(ui) = weak.upgrade() {
        set_message(&ui, outcome);
        refresh(&ui, &project.borrow(), viewed_idx);
    }
}

/// Push the project state + the previewed step into the UI properties.
fn refresh(ui: &AppWindow, project: &Project, viewed_idx: usize) {
    let viewed_step = Step::ALL.get(viewed_idx).copied().unwrap_or(Step::FIRST);
    let rows: Vec<StepRow> = step_rows(project, viewed_step)
        .into_iter()
        .map(|r| StepRow {
            number: r.number,
            title: r.title.into(),
            mark: if r.done { "✓" } else { "○" }.into(),
            // "you are here" marks the screen being shown (the wizard
            // cursor); ✓/○ shows real completion.
            here: if r.viewing {
                SharedString::from("   ← you are here")
            } else {
                SharedString::default()
            },
            viewing: r.viewing,
        })
        .collect();

    let nav = nav_state(project, viewed_step);
    ui.set_project_name(project.name.clone().into());
    ui.set_steps(ModelRc::new(VecModel::from(rows)));
    ui.set_viewed_number(i32::try_from(viewed_step.number()).unwrap_or(0));
    ui.set_total_steps(i32::try_from(Step::TOTAL).unwrap_or(7));
    ui.set_viewed_title(viewed_step.title().into());
    ui.set_can_back(nav.can_back);
    ui.set_can_next(nav.can_next);
    ui.set_has_scan(project.is_complete(Step::AddScan));
    // Step-5 print state (persists across nav, like the molds summary).
    ui.set_print_summary(print_step_summary(project).into());
    ui.set_has_print_export(project.print().is_some());
}

/// Set the step-message text + its error styling from a step outcome.
fn set_message(ui: &AppWindow, outcome: &StepOutcome) {
    let (text, is_error) = match outcome {
        Ok(msg) => (msg.clone(), false),
        Err(msg) => (msg.clone(), true),
    };
    ui.set_step_message(text.into());
    ui.set_step_message_is_error(is_error);
}

/// The step-2 working-mesh stats line.
fn stats_line(session: &EditSession) -> String {
    format!(
        "{} faces · {} vertices",
        session.face_count(),
        session.vertex_count()
    )
}

/// After a step-2 edit: re-render the viewport from the session's DISPLAY
/// mesh (the working mesh with any unbaked reorient applied, so a leveled
/// scan stays upright — and edits after leveling keep that orientation),
/// preserving the orbit angle, then surface the new stats + a result
/// message (`is_error` colors it red for "can't do that" cases).
fn apply_edit(
    weak: &slint::Weak<AppWindow>,
    scene: &Rc<RefCell<Option<Scene>>>,
    edit: &EditCell,
    message: &str,
    is_error: bool,
) {
    let eguard = edit.borrow();
    let Some(session) = eguard.as_ref() else {
        return;
    };
    let display = session.display_mesh();
    // Over-trim guard: an empty mesh would mean 0-size wgpu buffers. Don't
    // render it — keep the last good view + tell the user to ease off.
    if display.faces.is_empty() {
        if let Some(ui) = weak.upgrade() {
            ui.set_step_message("That trim removes the whole mesh — reduce it.".into());
            ui.set_step_message_is_error(true);
        }
        return;
    }
    let md = mesh_data_from_indexed(&display);
    let overlay: Vec<[f32; 3]> = session
        .display_centerline()
        .iter()
        .map(|p| [p.x as f32, p.y as f32, p.z as f32])
        .collect();
    let img = {
        let mut sguard = scene.borrow_mut();
        let Some(s) = sguard.as_mut() else {
            return;
        };
        s.set_mesh(&md);
        s.set_overlay(&overlay);
        s.render()
    };
    if let Some(ui) = weak.upgrade() {
        if let Ok(img) = img {
            ui.set_scan_view(img);
        }
        ui.set_edit_stats(stats_line(session).into());
        ui.set_step_message(message.into());
        ui.set_step_message_is_error(is_error);
        // Trim controls appear once a centerline exists; bound the sliders
        // to its arc length.
        ui.set_has_centerline(session.has_centerline());
        let max_mm = session.centerline_arc_length_mm().round() as i32;
        ui.set_trim_max_mm(max_mm.clamp(10, 100_000));
        // Reconstruct is available once a floor trim is applied (and the
        // centerline it was trimmed along still exists).
        ui.set_has_floor_trim(session.reconstruct_available() && session.has_centerline());
    }
}
