//! SIMD-optimized math primitives for physics simulation.
//!
//! This crate provides explicit SIMD vectorization for hot paths in the physics
//! simulation stack. It offers batch operations that process multiple vectors
//! simultaneously using portable SIMD intrinsics.
//!
//! # Architecture
//!
//! The crate is organized around batched vector types:
//! - [`Vec3x4`] - Process 4 `Vector3<f64>` values simultaneously
//! - [`Vec3x8`] - Process 8 `Vector3<f64>` values simultaneously (AVX-512)
//!
//! # Hot Paths Optimized
//!
//! These operations are the primary targets for SIMD optimization:
//!
//! 1. **Contact force computation** - Multiple contacts processed in parallel
//! 2. **GJK support vertex search** - Batch dot products for convex mesh support
//! 3. **AABB overlap tests** - Broad phase collision detection
//! 4. **Constraint solver operations** - Matrix-vector products, dot products
//!
//! # Example
//!
//! ```
//! use sim_simd::{Vec3x4, batch_dot_product_4};
//! use nalgebra::Vector3;
//!
//! // Process 4 dot products simultaneously
//! let vectors = [
//!     Vector3::new(1.0, 0.0, 0.0),
//!     Vector3::new(0.0, 1.0, 0.0),
//!     Vector3::new(0.0, 0.0, 1.0),
//!     Vector3::new(1.0, 1.0, 1.0),
//! ];
//! let direction = Vector3::new(1.0, 2.0, 3.0);
//!
//! let dots = batch_dot_product_4(&vectors, &direction);
//! assert_eq!(dots[0], 1.0);  // (1,0,0) · (1,2,3) = 1
//! assert_eq!(dots[1], 2.0);  // (0,1,0) · (1,2,3) = 2
//! assert_eq!(dots[2], 3.0);  // (0,0,1) · (1,2,3) = 3
//! assert_eq!(dots[3], 6.0);  // (1,1,1) · (1,2,3) = 6
//! ```
//!
//! # Performance Notes
//!
//! - On modern x86-64, expect 2-4x speedup for batch operations
//! - On Apple Silicon (NEON), expect 2-3x speedup
//! - Fallback scalar implementation available when SIMD is unavailable

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, missing_docs)]
// SIMD code intentionally uses indexed loops for auto-vectorization patterns
#![allow(clippy::needless_range_loop)]
// Suboptimal mul_add is intentional for auto-vectorization patterns
#![allow(clippy::suboptimal_flops)]
// Some functions use non-const methods internally
#![allow(clippy::missing_const_for_fn)]

mod batch_ops;
mod vec3x4;
mod vec3x8;

pub use batch_ops::*;
pub use vec3x4::*;
pub use vec3x8::*;

#[cfg(test)]
mod tests;
