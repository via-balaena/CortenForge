//! GPU physics pipeline infrastructure.
//!
//! Session 1: FK + tree scan primitive. Body poses, cinert, cdof, geom
//! poses, and subtree COM computed on GPU via level-order tree scan.
//!
//! Session 2: CRBA (mass matrix M + dense Cholesky) + velocity FK
//! (body spatial velocities cvel from qvel).
//!
//! Session 3: RNE (bias forces) + smooth dynamics (`qacc_smooth`) +
//! semi-implicit Euler integration. First full GPU physics loop.
//!
//! Session 4: `collision` — per-geom AABB + SDF-SDF / SDF-plane narrowphase,
//! producing contacts in a GPU buffer.
//!
//! Session 5: `constraint` — assemble pyramidal rows, run the primal Newton
//! solver, map forces back to joint space.
//!
//! Session 6: `orchestrator` ties every stage into a single GPU step
//! (`GpuPhysicsPipeline`), with optional implicit joint damping (`eulerdamp`)
//! folded into the integration substep.

pub mod collision;
#[cfg(test)]
mod conformance_tests;
pub mod constraint;
#[cfg(test)]
mod contact_conformance_tests;
pub mod crba;
pub mod eulerdamp;
pub mod fk;
pub mod integrate;
pub mod model_buffers;
pub mod orchestrator;
pub mod rne;
pub mod smooth;
pub mod state_buffers;
#[cfg(test)]
mod tests;
pub mod types;
pub mod velocity_fk;
mod wgpu_helpers;

pub use collision::GpuCollisionPipeline;
pub use constraint::GpuConstraintPipeline;
pub use crba::GpuCrbaPipeline;
pub use fk::GpuFkPipeline;
pub use integrate::GpuIntegratePipeline;
pub use model_buffers::GpuModelBuffers;
pub use orchestrator::{GpuPhysicsPipeline, GpuPipelineError};
pub use rne::GpuRnePipeline;
pub use smooth::GpuSmoothPipeline;
pub use state_buffers::GpuStateBuffers;
pub use velocity_fk::GpuVelocityFkPipeline;
