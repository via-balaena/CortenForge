//! 3D-preview spike — STAGE 2: load + orbit a real scan mesh.
//!
//! Stage 1 proved the wgpu→Slint pipe. Stage 2 turns it into an actual
//! mesh viewer — the heart of the live scan-editing step: load an STL,
//! frame it with an auto-fit orbit camera, depth-test it, Lambert-shade
//! it, and re-render live as you drag to orbit.
//!
//! Run:  `cargo run -p cf-studio-gui --bin spike-wgpu -- /path/to/scan.stl`
//! (no path → a built-in cube, so it runs standalone). Drag to orbit.
//!
//! Throwaway-grade exploration: blanket-allow the panicking shortcuts.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::cell::RefCell;
use std::rc::Rc;

use nalgebra::{Matrix4, Perspective3, Point3, Vector3};
use slint::ComponentHandle;
use slint::wgpu_28::wgpu;

slint::slint! {
    export component SpikeWindow inherits Window {
        title: "scan viewer spike (stage 2) — drag to orbit";
        preferred-width: 600px;
        preferred-height: 660px;

        in property <image> scene;
        callback orbit(length, length);

        VerticalLayout {
            padding: 16px;
            spacing: 10px;
            Text { text: "Stage 2 — orbit a scan mesh. Drag the view."; font-size: 16px; }
            Rectangle {
                width: 560px;
                height: 560px;
                Image { source: scene; width: 100%; height: 100%; }
                TouchArea {
                    property <length> px;
                    property <length> py;
                    pointer-event(e) => {
                        if (e.kind == PointerEventKind.down) {
                            self.px = self.mouse-x;
                            self.py = self.mouse-y;
                        }
                    }
                    moved => {
                        root.orbit(self.mouse-x - self.px, self.mouse-y - self.py);
                        self.px = self.mouse-x;
                        self.py = self.mouse-y;
                    }
                }
            }
        }
    }
}

const TEX_SIZE: u32 = 560;
const COLOR_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8UnormSrgb;
const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

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
    let d = abs(dot(n, light));          // two-sided so nothing goes black
    let base = vec3<f32>(0.55, 0.60, 0.70);
    return vec4<f32>(base * (0.30 + 0.70 * d), 1.0);
}
";

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Vertex {
    pos: [f32; 3],
    normal: [f32; 3],
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Uniforms {
    mvp: [f32; 16],
}

/// Orbit camera framing a target.
struct OrbitCamera {
    azimuth: f32,
    elevation: f32,
    distance: f32,
    target: Point3<f32>,
}

impl OrbitCamera {
    fn mvp(&self, radius: f32) -> [f32; 16] {
        let (ce, se) = (self.elevation.cos(), self.elevation.sin());
        let (ca, sa) = (self.azimuth.cos(), self.azimuth.sin());
        let dir = Vector3::new(ce * sa, se, ce * ca);
        let eye = self.target + dir * self.distance;
        let view = Matrix4::look_at_rh(&eye, &self.target, &Vector3::y());

        let near = (self.distance - radius).max(self.distance * 0.05);
        let far = self.distance + radius * 2.0;
        let proj = Perspective3::new(1.0, 50f32.to_radians(), near, far).to_homogeneous();

        // nalgebra's perspective is OpenGL z∈[-1,1]; wgpu wants z∈[0,1].
        let correction = Matrix4::new(
            1.0, 0.0, 0.0, 0.0, //
            0.0, 1.0, 0.0, 0.0, //
            0.0, 0.0, 0.5, 0.5, //
            0.0, 0.0, 0.0, 1.0,
        );

        let mvp = correction * proj * view;
        let mut out = [0.0_f32; 16];
        out.copy_from_slice(mvp.as_slice()); // column-major, matches WGSL mat4x4
        out
    }
}

/// CPU-side mesh ready to upload.
struct MeshData {
    vertices: Vec<Vertex>,
    indices: Vec<u32>,
    center: Point3<f32>,
    radius: f32,
}

/// Load an STL (or fall back to a cube) and compute smooth normals + bbox.
fn load_mesh(path: Option<&str>) -> MeshData {
    let (positions, faces): (Vec<Point3<f32>>, Vec<[u32; 3]>) = match path {
        Some(p) => {
            let mesh = cortenforge::mesh::io::load_mesh(p)
                .unwrap_or_else(|e| panic!("failed to load mesh at {p}: {e}"));
            let positions = mesh
                .vertices
                .iter()
                .map(|v| Point3::new(v.x as f32, v.y as f32, v.z as f32))
                .collect();
            (positions, mesh.faces)
        }
        None => cube(),
    };

    // Smooth (area-weighted) vertex normals.
    let mut normals = vec![Vector3::<f32>::zeros(); positions.len()];
    for f in &faces {
        let (a, b, c) = (
            positions[f[0] as usize],
            positions[f[1] as usize],
            positions[f[2] as usize],
        );
        let face_n = (b - a).cross(&(c - a));
        for &i in f {
            normals[i as usize] += face_n;
        }
    }

    let vertices: Vec<Vertex> = positions
        .iter()
        .zip(normals.iter())
        .map(|(p, n)| {
            let nn = if n.norm() > 1e-12 {
                n.normalize()
            } else {
                Vector3::y()
            };
            Vertex {
                pos: [p.x, p.y, p.z],
                normal: [nn.x, nn.y, nn.z],
            }
        })
        .collect();
    let indices: Vec<u32> = faces.iter().flat_map(|f| f.iter().copied()).collect();

    let mut min = Point3::new(f32::MAX, f32::MAX, f32::MAX);
    let mut max = Point3::new(f32::MIN, f32::MIN, f32::MIN);
    for p in &positions {
        min = Point3::new(min.x.min(p.x), min.y.min(p.y), min.z.min(p.z));
        max = Point3::new(max.x.max(p.x), max.y.max(p.y), max.z.max(p.z));
    }
    let center = nalgebra::center(&min, &max);
    let radius = ((max - min).norm() / 2.0).max(1e-3);

    MeshData {
        vertices,
        indices,
        center,
        radius,
    }
}

/// A unit cube (fallback when no STL is given).
fn cube() -> (Vec<Point3<f32>>, Vec<[u32; 3]>) {
    let p = [
        Point3::new(-0.5, -0.5, -0.5),
        Point3::new(0.5, -0.5, -0.5),
        Point3::new(0.5, 0.5, -0.5),
        Point3::new(-0.5, 0.5, -0.5),
        Point3::new(-0.5, -0.5, 0.5),
        Point3::new(0.5, -0.5, 0.5),
        Point3::new(0.5, 0.5, 0.5),
        Point3::new(-0.5, 0.5, 0.5),
    ];
    let f = vec![
        [0, 2, 1],
        [0, 3, 2],
        [4, 5, 6],
        [4, 6, 7],
        [0, 1, 5],
        [0, 5, 4],
        [2, 3, 7],
        [2, 7, 6],
        [1, 2, 6],
        [1, 6, 5],
        [0, 4, 7],
        [0, 7, 3],
    ];
    (p.to_vec(), f)
}

/// GPU + camera state, rebuilt once Slint hands us its device/queue.
struct Scene {
    device: wgpu::Device,
    queue: wgpu::Queue,
    pipeline: wgpu::RenderPipeline,
    bind_group: wgpu::BindGroup,
    uniform: wgpu::Buffer,
    vbuf: wgpu::Buffer,
    ibuf: wgpu::Buffer,
    index_count: u32,
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

        // Frame the mesh: pull the camera back enough to fit the radius.
        let distance = mesh.radius / (50f32.to_radians() / 2.0).sin() * 1.4;
        Self {
            device: device.clone(),
            queue: queue.clone(),
            pipeline,
            bind_group,
            uniform,
            vbuf,
            ibuf,
            index_count: mesh.indices.len() as u32,
            camera: OrbitCamera {
                azimuth: 0.6,
                elevation: 0.5,
                distance,
                target: mesh.center,
            },
            radius: mesh.radius,
        }
    }

    fn render(&self) -> Result<slint::Image, String> {
        self.queue.write_buffer(
            &self.uniform,
            0,
            bytemuck::cast_slice(&[Uniforms {
                mvp: self.camera.mvp(self.radius),
            }]),
        );

        let extent = wgpu::Extent3d {
            width: TEX_SIZE,
            height: TEX_SIZE,
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
                            r: 0.93,
                            g: 0.94,
                            b: 0.96,
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
        }
        self.queue.submit(std::iter::once(encoder.finish()));

        slint::Image::try_from(color).map_err(|e| format!("Image::try_from failed: {e:?}"))
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mesh = Rc::new(load_mesh(std::env::args().nth(1).as_deref()));

    slint::BackendSelector::new()
        .require_wgpu_28(slint::wgpu_28::WGPUConfiguration::default())
        .select()?;

    let ui = SpikeWindow::new()?;
    let weak = ui.as_weak();
    let scene: Rc<RefCell<Option<Scene>>> = Rc::new(RefCell::new(None));

    {
        let (scene, weak, mesh) = (scene.clone(), weak.clone(), mesh.clone());
        ui.window()
            .set_rendering_notifier(move |state, graphics_api| {
                if let (
                    slint::RenderingState::RenderingSetup,
                    slint::GraphicsAPI::WGPU28 { device, queue, .. },
                ) = (state, graphics_api)
                {
                    let built = Scene::build(device, queue, &mesh);
                    let image = built.render();
                    *scene.borrow_mut() = Some(built);
                    if let (Some(ui), Ok(image)) = (weak.upgrade(), image) {
                        ui.set_scene(image);
                    }
                }
            })
            .map_err(|e| format!("set_rendering_notifier: {e}"))?;
    }

    {
        let (scene, weak) = (scene.clone(), weak.clone());
        ui.on_orbit(move |dx, dy| {
            let mut guard = scene.borrow_mut();
            let Some(s) = guard.as_mut() else { return };
            s.camera.azimuth -= dx * 0.01;
            s.camera.elevation = (s.camera.elevation + dy * 0.01).clamp(-1.5, 1.5);
            if let (Some(ui), Ok(image)) = (weak.upgrade(), s.render()) {
                ui.set_scene(image);
            }
        });
    }

    ui.run()?;
    Ok(())
}
