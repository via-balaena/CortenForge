//! GPU-accelerated SDF collision trait.
//!
//! Defines the [`GpuSdfCollision`] trait that sim-gpu implements.
//! sim-core dispatches to this trait when available, falling back
//! to CPU grid tracing when absent.

use nalgebra::Vector3;
use sim_types::Pose;

use super::SdfContact;

/// GPU-accelerated SDF collision backend.
///
/// Implementations upload SDF grids to GPU memory once, then dispatch
/// compute shaders each step with updated body poses. Grid indices
/// correspond to `Model::shape_data` entries.
///
/// Stored on `Model::gpu_collider` as `Option<Arc<dyn GpuSdfCollision>>`.
pub trait GpuSdfCollision: Send + Sync + std::fmt::Debug {
    /// Grid-based SDF-SDF contact detection (non-convex fallback).
    ///
    /// Traces the surface of both grids into each other (A→B and B→A).
    /// Returns deduplicated contacts in world space.
    fn sdf_sdf_contacts(
        &self,
        grid_a: usize,
        pose_a: &Pose,
        grid_b: usize,
        pose_b: &Pose,
        margin: f64,
    ) -> Vec<SdfContact>;

    /// SDF grid vs infinite plane contact detection.
    fn sdf_plane_contacts(
        &self,
        grid: usize,
        pose: &Pose,
        plane_normal: &Vector3<f64>,
        plane_offset: f64,
    ) -> Vec<SdfContact>;
}
