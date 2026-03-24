//! GPU physics pipeline infrastructure.
//!
//! Session 1: FK + tree scan primitive. Body poses, cinert, cdof, geom
//! poses, and subtree COM computed on GPU via level-order tree scan.
//!
//! Session 2: CRBA (mass matrix M + dense Cholesky) + velocity FK
//! (body spatial velocities cvel from qvel).

pub mod crba;
pub mod fk;
pub mod model_buffers;
pub mod state_buffers;
#[cfg(test)]
mod tests;
pub mod types;
pub mod velocity_fk;

pub use crba::GpuCrbaPipeline;
pub use fk::GpuFkPipeline;
pub use model_buffers::GpuModelBuffers;
pub use state_buffers::GpuStateBuffers;
pub use velocity_fk::GpuVelocityFkPipeline;
