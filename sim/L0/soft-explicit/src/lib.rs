//! # sim-soft-explicit
//!
//! The explicit soft-body solver's core, minus the GPU: the physics shared by
//! the CPU and GPU executors, written once, and the lowered model they both
//! run on. The plan is `docs/SOFT_CONTACT_ARCHITECTURE_RECON.md` (§14 the
//! crate layout, §15 the first experiment).
//!
//! ## The shared math, written once
//!
//! The per-element and per-node physics lives in `src/shared/`, written in the
//! loop-free Rust subset that `sim-wgsl-gen` translates, against a scalar
//! alias `R`. Each file is compiled twice here:
//!
//! - [`f32`](mod@f32), what the GPU runs and the CPU executor's working precision;
//! - [`f64`](mod@f64), the reference that says whether `f32` is enough (plan §15a K3);
//!
//! and translated once into [`SHARED_WGSL`] for the GPU executor. A freshness
//! test regenerates that WGSL and fails, naming the command, if the committed
//! file is stale.
//!
//! What the shared math holds (plan §14b):
//!
//! - **the constitutive law**: `sim-soft`'s compressible Yeoh, neo-Hookean at
//!   `C₂ = 0`, written in the displacement gradient and split into the terms
//!   evaluated per element and the λ term averaged over nodes (selective
//!   averaged nodal pressure, §15c);
//! - **the element**: the four-node tetrahedron's displacement gradient,
//!   dilation, forces, energy and stable-step estimate;
//! - **pressure averaging** and its rule where materials meet;
//! - **time integration**: the central-difference update with mass damping;
//! - **the obstacle's pose** and its interpolation between time samples;
//! - **the obstacle's distance field**, with the product's clamped lookup;
//! - **the contact law**: the nodal-mass penalty with elastic-slip Coulomb
//!   friction.
//!
//! Displacements, not positions, are the state (plan §6): a node's position
//! is its rest position plus its displacement. Orchestration (gathers,
//! storage indexing, dispatch) belongs to each executor. Vectors are `[R; 3]`
//! and 3×3 matrices `[R; 9]`, row-major.
//!
//! ## The lowered model
//!
//! [`ExplicitModel`] is the data every executor uploads: flat node and
//! element arrays, a material per element (bonded layers need it), and the
//! rest-state quantities computed once, in `f64`.

#![deny(clippy::unwrap_used, clippy::expect_used)]

pub mod cpu;
pub mod executor;
mod model;
pub mod stepping;

pub use model::{ExplicitModel, Incidence, ModelError};

/// The shared math at `f32`: what the GPU runs, and the CPU executor's
/// working precision.
pub mod f32 {
    /// This module's scalar.
    pub type R = f32;

    include!("shared/linalg.rs");
    include!("shared/material.rs");
    include!("shared/element.rs");
    include!("shared/anp.rs");
    include!("shared/integrate.rs");
    include!("shared/pose.rs");
    include!("shared/sdf.rs");
    include!("shared/contact.rs");
}

/// The shared math at `f64`: the reference the `f32` results are checked
/// against.
pub mod f64 {
    /// This module's scalar.
    pub type R = f64;

    include!("shared/linalg.rs");
    include!("shared/material.rs");
    include!("shared/element.rs");
    include!("shared/anp.rs");
    include!("shared/integrate.rs");
    include!("shared/pose.rs");
    include!("shared/sdf.rs");
    include!("shared/contact.rs");
}

/// The shared math as WGSL, generated from `src/shared/` by `sim-wgsl-gen`
/// with `R` as `f32`. The GPU executor prepends it to its entry points.
pub const SHARED_WGSL: &str = include_str!("shared.wgsl");
