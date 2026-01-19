//! GPU-accelerated mesh processing using WGPU compute shaders.
//!
//! This crate provides GPU-accelerated SDF (Signed Distance Field) computation
//! for triangle meshes. It uses WGPU for cross-platform GPU support.
//!
//! # Performance Summary
//!
//! Based on benchmarks:
//!
//! | Operation | GPU vs CPU | Recommendation |
//! |-----------|------------|----------------|
//! | SDF Computation | **3-68x faster** for meshes <5k tri | Use GPU for shell generation |
//!
//! ## When to Use GPU
//!
//! **SDF Computation** is the primary use case for GPU acceleration:
//! - Small meshes (<100 tri) with 128³ grid: **48-68x speedup**
//! - Medium meshes (320 tri) with 128³ grid: **14x speedup**
//! - Large meshes (>5k tri): GPU overhead dominates, CPU is faster
//!
//! **Best use case**: Shell generation for 3D printing, where typical scan
//! meshes (1k-10k triangles) are processed with high-resolution grids (128³+).
//!
//! # Layer 0 Crate
//!
//! This is a Layer 0 crate with **zero Bevy dependencies**. It can be used in:
//! - CLI tools
//! - Web applications (WASM) - with appropriate wgpu backend
//! - Servers with GPU acceleration
//! - Python bindings via `PyO3`
//!
//! # GPU Availability
//!
//! GPU support is automatically detected at runtime. Use [`GpuContext::is_available()`]
//! to check if a GPU is available, or use the `try_*` variants of computation
//! functions which return `None` instead of erroring when GPU is unavailable.
//!
//! # Example
//!
//! ```no_run
//! use mesh_gpu::{GpuContext, compute_sdf_gpu, try_compute_sdf_gpu, GpuSdfParams};
//! use mesh_types::IndexedMesh;
//!
//! // Check GPU availability
//! if GpuContext::is_available() {
//!     println!("GPU available: {}", GpuContext::get().map_or("none", |c| &c.adapter_info.name));
//! }
//!
//! // Create a simple mesh
//! let mesh = IndexedMesh::new();
//! let params = GpuSdfParams::new([64, 64, 64], [0.0, 0.0, 0.0], 0.1);
//!
//! // Compute SDF (will error if no GPU)
//! match compute_sdf_gpu(&mesh, &params) {
//!     Ok(result) => println!("Computed {} voxels in {:.2}ms",
//!         result.values.len(), result.compute_time_ms),
//!     Err(e) => eprintln!("GPU computation failed: {}", e),
//! }
//! ```
//!
//! # Automatic Fallback
//!
//! For production use, implement automatic CPU fallback:
//!
//! ```no_run
//! use mesh_gpu::{try_compute_sdf_gpu, GpuSdfParams};
//! use mesh_types::IndexedMesh;
//!
//! fn compute_sdf_with_fallback(mesh: &IndexedMesh, params: &GpuSdfParams) -> Vec<f32> {
//!     // Try GPU first
//!     if let Some(result) = try_compute_sdf_gpu(mesh, params) {
//!         return result.values;
//!     }
//!
//!     // Fall back to CPU implementation
//!     // In a real app, use mesh_sdf crate here
//!     vec![0.0; params.total_voxels()]
//! }
//! ```
//!
//! # Running Benchmarks
//!
//! ```bash
//! # Run all GPU benchmarks
//! cargo bench -p mesh-gpu
//!
//! # Run specific benchmark group
//! cargo bench -p mesh-gpu -- "SDF Computation"
//!
//! # View HTML reports
//! open target/criterion/report/index.html
//! ```
//!
//! # Quality Standards
//!
//! This crate maintains A-grade standards per [STANDARDS.md](../../STANDARDS.md):
//! - ≥90% test coverage
//! - Zero clippy/doc warnings
//! - Zero `unwrap`/`expect` in library code

#![warn(missing_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

pub mod buffers;
pub mod context;
pub mod error;
pub mod sdf;

// Re-export commonly used types
pub use context::{GpuAdapterInfo, GpuContext, GpuDevicePreference};
pub use error::{GpuError, GpuResult};
pub use sdf::{GpuSdfParams, GpuSdfResult, SdfPipeline, compute_sdf_gpu, try_compute_sdf_gpu};

// Re-export buffer types for advanced usage
pub use buffers::{GpuGridParams, GpuTriangle, GpuVertex, MeshBuffers, SdfGridBuffers, TileConfig};
