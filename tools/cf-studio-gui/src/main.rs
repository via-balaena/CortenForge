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
use std::path::PathBuf;
use std::rc::Rc;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cf_studio_core::{DesignDraft, LayerDraft, MoldOutputs, Project, Step};
use cf_studio_engine::{
    EditSession, ReconstructShape, generate_molds_for_design, run_simplify, silicone_catalog,
};
use cf_studio_gui::viewer::{MeshData, OrbitCamera, Uniforms, Vertex, mesh_data_from_indexed};
use cf_studio_gui::{
    StepOutcome, apply_design, apply_design_draft, apply_prep, apply_scan, cell_size_m_for_quality,
    format_molds_summary, nav_state, step_rows,
};
use mesh_types::IndexedMesh;
use slint::wgpu_28::wgpu;
use slint::{ComponentHandle, Model, ModelRc, SharedString, VecModel};
use ui::{AppWindow, LayerRow, StepRow};

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
    {
        let (project, viewed, weak) = (project.clone(), viewed.clone(), weak.clone());
        let (layers_model, catalog) = (layers_model.clone(), catalog.clone());
        ui.on_use_design(move |cavity_mm| {
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
            let draft = DesignDraft {
                cavity_inset_m: f64::from(cavity_mm) / 1000.0,
                layers,
            };
            let outcome = apply_design_draft(&mut project.borrow_mut(), draft);
            update(&weak, &project, viewed.get(), &outcome);
        });
    }

    // ── navigation (gated by the wizard state) ──────────────────
    {
        let (project, viewed, weak) = (project.clone(), viewed.clone(), weak.clone());
        ui.on_back(move || {
            viewed.set(viewed.get().saturating_sub(1));
            if let Some(ui) = weak.upgrade() {
                ui.set_step_message(SharedString::default());
                refresh(&ui, &project.borrow(), viewed.get());
            }
        });
    }
    {
        let (project, viewed, weak) = (project.clone(), viewed.clone(), weak.clone());
        ui.on_next(move || {
            let v = viewed.get();
            let step = Step::ALL.get(v).copied().unwrap_or(Step::FIRST);
            // Respect the gate even if the disabled button somehow fires.
            if nav_state(&project.borrow(), step).can_next {
                viewed.set(v + 1);
            }
            if let Some(ui) = weak.upgrade() {
                ui.set_step_message(SharedString::default());
                refresh(&ui, &project.borrow(), viewed.get());
            }
        });
    }
    ui.on_help(|| {
        // Placeholder — a help panel / per-step guidance arrives later.
    });

    // ── step 1: pick a scan → record it + open the live edit session ──
    {
        let (project, viewed, weak) = (project.clone(), viewed.clone(), weak.clone());
        let (gpu, scene) = (gpu.clone(), scene.clone());
        let (scan_path, edit) = (scan_path.clone(), edit.clone());
        ui.on_pick_scan(move || {
            let Some(path) = rfd::FileDialog::new()
                .set_title("Choose your 3D scan")
                .add_filter("3D scan", &["stl", "obj", "ply"])
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
        ui.on_pick_design(move || {
            let Some(path) = rfd::FileDialog::new()
                .set_title("Choose the design file")
                .add_filter("design", &["toml"])
                .pick_file()
            else {
                return;
            };
            let outcome = apply_design(&mut project.borrow_mut(), &path);
            update(&weak, &project, viewed.get(), &outcome);
        });
    }

    // ── step 4: make the molds (run the cast async) ─────────────
    {
        let (project, weak) = (project.clone(), weak.clone());
        let (inbox, start) = (molds_inbox.clone(), molds_start.clone());
        ui.on_make_molds(move |quality_idx| {
            // Re-entrancy guard: a run already in flight (the UI disables the
            // button via `busy`, but a queued click could still arrive). Don't
            // spawn a second 15-min cast into the same output dir.
            if start.get().is_some() {
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
                    generate_molds_for_design(&cleaned_stl, &prep_toml, &draft, cell_size_m, None)
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

    ui.run()?;
    Ok(())
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
    ui.set_total_steps(i32::try_from(Step::TOTAL).unwrap_or(6));
    ui.set_viewed_title(viewed_step.title().into());
    ui.set_can_back(nav.can_back);
    ui.set_can_next(nav.can_next);
    ui.set_has_scan(project.is_complete(Step::AddScan));
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
