//! 3D-preview spike — STAGE 1: prove the pipe.
//!
//! Goal: confirm that a wgpu render shows up inside a Slint `Image`, using
//! Slint's own device/queue (`require_wgpu_28` + the rendering notifier +
//! `Image::try_from(texture)`). This draws a hardcoded blue triangle on a
//! light background into an offscreen texture and displays it. If you run
//! it and see a blue triangle in the window, the wgpu→Slint pipe works and
//! stage 2 (load + orbit the real scan mesh) is buildable on it.
//!
//! Run:  `cargo run -p cf-studio-gui --bin spike-wgpu`
//!
//! This is a throwaway exploration: blanket-allow the panicking shortcuts
//! (Slint's generated code + spike convenience) rather than thread the
//! crate's strict library lints through it.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use slint::ComponentHandle;
use slint::wgpu_28::wgpu;

slint::slint! {
    export component SpikeWindow inherits Window {
        title: "wgpu → Slint spike (stage 1)";
        preferred-width: 560px;
        preferred-height: 600px;

        in property <image> scene;

        VerticalLayout {
            padding: 16px;
            spacing: 12px;
            Text {
                text: "Stage 1 — wgpu renders into a Slint Image.";
                font-size: 16px;
            }
            Text {
                text: "If you see a blue triangle below, the pipe works.";
                font-size: 14px;
                color: #888888;
            }
            Image {
                source: scene;
                width: 512px;
                height: 512px;
            }
        }
    }
}

const SHADER: &str = r"
@vertex
fn vs(@builtin(vertex_index) i: u32) -> @builtin(position) vec4<f32> {
    var p = array<vec2<f32>, 3>(
        vec2<f32>(0.0, 0.6),
        vec2<f32>(-0.6, -0.6),
        vec2<f32>(0.6, -0.6),
    );
    return vec4<f32>(p[i], 0.0, 1.0);
}

@fragment
fn fs() -> @location(0) vec4<f32> {
    return vec4<f32>(0.2, 0.5, 0.9, 1.0);
}
";

/// Render a single triangle into a fresh texture and hand it to Slint.
fn render_triangle(device: &wgpu::Device, queue: &wgpu::Queue) -> Result<slint::Image, String> {
    let format = wgpu::TextureFormat::Rgba8UnormSrgb;
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("spike-target"),
        size: wgpu::Extent3d {
            width: 512,
            height: 512,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
        view_formats: &[],
    });
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());

    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("spike-shader"),
        source: wgpu::ShaderSource::Wgsl(SHADER.into()),
    });

    let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("spike-pipeline"),
        layout: None,
        vertex: wgpu::VertexState {
            module: &shader,
            entry_point: Some("vs"),
            buffers: &[],
            compilation_options: wgpu::PipelineCompilationOptions::default(),
        },
        fragment: Some(wgpu::FragmentState {
            module: &shader,
            entry_point: Some("fs"),
            targets: &[Some(wgpu::ColorTargetState {
                format,
                blend: None,
                write_mask: wgpu::ColorWrites::ALL,
            })],
            compilation_options: wgpu::PipelineCompilationOptions::default(),
        }),
        primitive: wgpu::PrimitiveState::default(),
        depth_stencil: None,
        multisample: wgpu::MultisampleState::default(),
        multiview_mask: None,
        cache: None,
    });

    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("spike"),
    });
    {
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("spike-pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &view,
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
            depth_stencil_attachment: None,
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        pass.set_pipeline(&pipeline);
        pass.draw(0..3, 0..1);
    }
    queue.submit(std::iter::once(encoder.finish()));

    slint::Image::try_from(texture).map_err(|e| format!("Image::try_from failed: {e:?}"))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    slint::BackendSelector::new()
        .require_wgpu_28(slint::wgpu_28::WGPUConfiguration::default())
        .select()?;

    let ui = SpikeWindow::new()?;
    let weak = ui.as_weak();

    ui.window()
        .set_rendering_notifier(move |state, graphics_api| {
            if let (
                slint::RenderingState::RenderingSetup,
                slint::GraphicsAPI::WGPU28 { device, queue, .. },
            ) = (state, graphics_api)
            {
                if let Some(ui) = weak.upgrade() {
                    match render_triangle(device, queue) {
                        Ok(image) => ui.set_scene(image),
                        Err(e) => eprintln!("spike render failed: {e}"),
                    }
                }
            }
        })
        .map_err(|e| format!("set_rendering_notifier: {e}"))?;

    ui.run()?;
    Ok(())
}
