//! GPU-accelerated boolean operations using wgpu.
//!
//! This module provides GPU acceleration for compute-intensive parts of
//! boolean operations, particularly:
//!
//! - Parallel BVH construction
//! - Batch ray-triangle intersection tests
//! - Face classification via parallel ray casting
//!
//! # Feature Gate
//!
//! This module is only available when the `gpu` feature is enabled:
//!
//! ```toml
//! [dependencies]
//! mesh-boolean = { version = "0.7", features = ["gpu"] }
//! ```
//!
//! # Example
//!
//! ```ignore
//! use mesh_boolean::gpu::{GpuContext, GpuBooleanConfig};
//!
//! // Initialize GPU context
//! let gpu = GpuContext::new()?;
//!
//! // Perform GPU-accelerated union
//! let result = gpu.union(&mesh_a, &mesh_b, &GpuBooleanConfig::default())?;
//! ```

use crate::classify::FaceLocation;
use crate::error::{BooleanError, BooleanResult};
use bytemuck::{Pod, Zeroable};
use mesh_types::IndexedMesh;
use std::sync::Arc;
use wgpu::util::DeviceExt;

/// GPU compute context for accelerated boolean operations.
pub struct GpuContext {
    device: Arc<wgpu::Device>,
    queue: Arc<wgpu::Queue>,
    ray_cast_pipeline: wgpu::ComputePipeline,
}

impl GpuContext {
    /// Create a new GPU context.
    ///
    /// Initializes wgpu adapter, device, and compiles compute shaders.
    ///
    /// # Errors
    ///
    /// Returns `BooleanError::GpuError` if:
    /// - No compatible GPU adapter is found
    /// - Device creation fails
    /// - Shader compilation fails
    pub fn new() -> BooleanResult<Self> {
        pollster::block_on(Self::new_async())
    }

    async fn new_async() -> BooleanResult<Self> {
        // Request adapter
        let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..Default::default()
        });

        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: None,
                force_fallback_adapter: false,
            })
            .await
            .ok_or_else(|| BooleanError::GpuError {
                details: "No compatible GPU adapter found".to_string(),
            })?;

        // Request device
        let (device, queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    label: Some("mesh-boolean"),
                    required_features: wgpu::Features::empty(),
                    required_limits: wgpu::Limits::default(),
                    memory_hints: wgpu::MemoryHints::Performance,
                },
                None,
            )
            .await
            .map_err(|e| BooleanError::GpuError {
                details: format!("Failed to create GPU device: {e}"),
            })?;

        let device = Arc::new(device);
        let queue = Arc::new(queue);

        // Create ray casting compute pipeline
        let ray_cast_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("ray_cast_shader"),
            source: wgpu::ShaderSource::Wgsl(RAY_CAST_SHADER.into()),
        });

        let ray_cast_pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("ray_cast_pipeline"),
            layout: None,
            module: &ray_cast_shader,
            entry_point: Some("main"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            cache: None,
        });

        Ok(Self {
            device,
            queue,
            ray_cast_pipeline,
        })
    }

    /// Classify faces using GPU-accelerated ray casting.
    ///
    /// # Arguments
    ///
    /// * `mesh` - The mesh whose faces to classify
    /// * `other` - The mesh to classify against
    /// * `epsilon` - Tolerance for ray-triangle intersection
    ///
    /// # Returns
    ///
    /// Vector of `FaceLocation` for each face in `mesh`.
    ///
    /// # Errors
    ///
    /// Returns [`BooleanError`] if GPU computation fails.
    pub fn classify_faces(
        &self,
        mesh: &IndexedMesh,
        other: &IndexedMesh,
        epsilon: f64,
    ) -> BooleanResult<Vec<FaceLocation>> {
        if mesh.faces.is_empty() {
            return Ok(Vec::new());
        }

        // Prepare face centroids
        let centroids: Vec<GpuPoint> = mesh
            .faces
            .iter()
            .map(|face| {
                let v0 = mesh.vertices[face[0] as usize].position;
                let v1 = mesh.vertices[face[1] as usize].position;
                let v2 = mesh.vertices[face[2] as usize].position;
                let centroid = (v0.coords + v1.coords + v2.coords) / 3.0;
                GpuPoint {
                    x: centroid.x as f32,
                    y: centroid.y as f32,
                    z: centroid.z as f32,
                    _pad: 0.0,
                }
            })
            .collect();

        // Prepare triangles from other mesh
        let triangles: Vec<GpuTriangle> = other
            .faces
            .iter()
            .map(|face| {
                let v0 = other.vertices[face[0] as usize].position;
                let v1 = other.vertices[face[1] as usize].position;
                let v2 = other.vertices[face[2] as usize].position;
                GpuTriangle {
                    v0: [v0.x as f32, v0.y as f32, v0.z as f32, 0.0],
                    v1: [v1.x as f32, v1.y as f32, v1.z as f32, 0.0],
                    v2: [v2.x as f32, v2.y as f32, v2.z as f32, 0.0],
                }
            })
            .collect();

        // Create GPU buffers
        let centroid_buffer = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("centroids"),
                contents: bytemuck::cast_slice(&centroids),
                usage: wgpu::BufferUsages::STORAGE,
            });

        let triangle_buffer = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("triangles"),
                contents: bytemuck::cast_slice(&triangles),
                usage: wgpu::BufferUsages::STORAGE,
            });

        // Results buffer (one u32 per centroid)
        let result_size = (centroids.len() * std::mem::size_of::<u32>()) as u64;
        let result_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("results"),
            size: result_size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });

        let staging_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("staging"),
            size: result_size,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        // Uniform buffer with parameters
        let params = GpuParams {
            num_centroids: centroids.len() as u32,
            num_triangles: triangles.len() as u32,
            epsilon: epsilon as f32,
            _pad: 0,
        };

        let params_buffer = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("params"),
                contents: bytemuck::bytes_of(&params),
                usage: wgpu::BufferUsages::UNIFORM,
            });

        // Create bind group
        let bind_group_layout = self.ray_cast_pipeline.get_bind_group_layout(0);
        let bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("ray_cast_bind_group"),
            layout: &bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: params_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: centroid_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: triangle_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: result_buffer.as_entire_binding(),
                },
            ],
        });

        // Dispatch compute shader
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("ray_cast_encoder"),
            });

        {
            let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("ray_cast_pass"),
                timestamp_writes: None,
            });
            compute_pass.set_pipeline(&self.ray_cast_pipeline);
            compute_pass.set_bind_group(0, &bind_group, &[]);

            let workgroup_size = 64;
            let num_workgroups = (centroids.len() as u32).div_ceil(workgroup_size);
            compute_pass.dispatch_workgroups(num_workgroups, 1, 1);
        }

        // Copy results to staging buffer
        encoder.copy_buffer_to_buffer(&result_buffer, 0, &staging_buffer, 0, result_size);

        self.queue.submit(std::iter::once(encoder.finish()));

        // Read results
        let buffer_slice = staging_buffer.slice(..);
        let (tx, rx) = std::sync::mpsc::channel();
        buffer_slice.map_async(wgpu::MapMode::Read, move |result| {
            // Channel send can fail if receiver is dropped, but we handle that below via rx.recv()
            #[allow(clippy::let_underscore_must_use)]
            let _ = tx.send(result);
        });
        self.device.poll(wgpu::Maintain::Wait);

        rx.recv()
            .map_err(|e| BooleanError::GpuError {
                details: format!("Failed to receive buffer mapping result: {e}"),
            })?
            .map_err(|e| BooleanError::GpuError {
                details: format!("Buffer mapping failed: {e}"),
            })?;

        let data = buffer_slice.get_mapped_range();
        let result_counts: &[u32] = bytemuck::cast_slice(&data);

        // Convert counts to FaceLocation
        let classifications: Vec<FaceLocation> = result_counts
            .iter()
            .map(|&count| {
                if count % 2 == 1 {
                    FaceLocation::Inside
                } else {
                    FaceLocation::Outside
                }
            })
            .collect();

        drop(data);
        staging_buffer.unmap();

        Ok(classifications)
    }

    /// Get information about the GPU device.
    #[must_use]
    pub fn device_info(&self) -> GpuDeviceInfo {
        GpuDeviceInfo {
            name: "wgpu device".to_string(),
            backend: format!("{:?}", self.device.limits()),
        }
    }
}

/// Information about the GPU device.
#[derive(Debug, Clone)]
pub struct GpuDeviceInfo {
    /// Device name.
    pub name: String,
    /// Backend information.
    pub backend: String,
}

/// GPU-compatible point structure (16-byte aligned).
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct GpuPoint {
    x: f32,
    y: f32,
    z: f32,
    _pad: f32,
}

/// GPU-compatible triangle structure.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct GpuTriangle {
    v0: [f32; 4],
    v1: [f32; 4],
    v2: [f32; 4],
}

/// GPU parameters uniform.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct GpuParams {
    num_centroids: u32,
    num_triangles: u32,
    epsilon: f32,
    _pad: u32,
}

/// WGSL shader for ray casting.
const RAY_CAST_SHADER: &str = r"
struct Params {
    num_centroids: u32,
    num_triangles: u32,
    epsilon: f32,
    _pad: u32,
}

struct Point {
    x: f32,
    y: f32,
    z: f32,
    _pad: f32,
}

struct Triangle {
    v0: vec4<f32>,
    v1: vec4<f32>,
    v2: vec4<f32>,
}

@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read> centroids: array<Point>;
@group(0) @binding(2) var<storage, read> triangles: array<Triangle>;
@group(0) @binding(3) var<storage, read_write> results: array<u32>;

// Möller-Trumbore ray-triangle intersection
fn ray_triangle_intersect(
    origin: vec3<f32>,
    dir: vec3<f32>,
    v0: vec3<f32>,
    v1: vec3<f32>,
    v2: vec3<f32>,
    epsilon: f32
) -> bool {
    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    let h = cross(dir, edge2);
    let a = dot(edge1, h);

    if abs(a) < epsilon {
        return false;
    }

    let f = 1.0 / a;
    let s = origin - v0;
    let u = f * dot(s, h);

    if u < 0.0 || u > 1.0 {
        return false;
    }

    let q = cross(s, edge1);
    let v = f * dot(dir, q);

    if v < 0.0 || u + v > 1.0 {
        return false;
    }

    let t = f * dot(edge2, q);
    return t > epsilon;
}

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let idx = global_id.x;
    if idx >= params.num_centroids {
        return;
    }

    let centroid = centroids[idx];
    let origin = vec3<f32>(centroid.x, centroid.y, centroid.z);
    let dir = vec3<f32>(1.0, 0.0, 0.0); // +X direction

    var count: u32 = 0u;

    for (var i: u32 = 0u; i < params.num_triangles; i = i + 1u) {
        let tri = triangles[i];
        let v0 = tri.v0.xyz;
        let v1 = tri.v1.xyz;
        let v2 = tri.v2.xyz;

        if ray_triangle_intersect(origin, dir, v0, v1, v2, params.epsilon) {
            count = count + 1u;
        }
    }

    results[idx] = count;
}
";

/// Check if GPU acceleration is available.
///
/// Returns true if a compatible GPU can be initialized.
#[must_use]
pub fn is_gpu_available() -> bool {
    GpuContext::new().is_ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::{Point3, Vertex};

    fn create_unit_cube() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();

        let vertices = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.0, 0.0, 1.0),
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(0.0, 1.0, 1.0),
        ];

        for v in &vertices {
            mesh.vertices.push(Vertex::new(*v));
        }

        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([0, 3, 2]);
        mesh.faces.push([4, 5, 6]);
        mesh.faces.push([4, 6, 7]);
        mesh.faces.push([0, 1, 5]);
        mesh.faces.push([0, 5, 4]);
        mesh.faces.push([2, 3, 7]);
        mesh.faces.push([2, 7, 6]);
        mesh.faces.push([0, 4, 7]);
        mesh.faces.push([0, 7, 3]);
        mesh.faces.push([1, 2, 6]);
        mesh.faces.push([1, 6, 5]);

        mesh
    }

    #[test]
    fn test_gpu_availability() {
        // This test just checks that the function runs without panicking
        let _ = is_gpu_available();
    }

    #[test]
    #[ignore = "Requires GPU"]
    fn test_gpu_classify_faces() {
        let gpu = GpuContext::new().unwrap();

        let outer_cube = create_unit_cube();

        // Create small inner mesh
        let mut inner = IndexedMesh::new();
        inner.vertices.push(Vertex::new(Point3::new(0.4, 0.4, 0.4)));
        inner.vertices.push(Vertex::new(Point3::new(0.6, 0.4, 0.4)));
        inner.vertices.push(Vertex::new(Point3::new(0.5, 0.6, 0.4)));
        inner.faces.push([0, 1, 2]);

        let classifications = gpu.classify_faces(&inner, &outer_cube, 1e-6).unwrap();

        assert_eq!(classifications.len(), 1);
        assert_eq!(classifications[0], FaceLocation::Inside);
    }
}
