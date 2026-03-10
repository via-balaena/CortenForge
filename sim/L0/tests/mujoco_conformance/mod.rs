//! MuJoCo conformance tests — Phase 12 four-layer conformance test suite.
//!
//! Validates CortenForge against MuJoCo 3.4.0 at every pipeline stage.
//!
//! Layers:
//! - A: Self-consistency (no MuJoCo dependency)
//! - B: Per-stage reference comparison (MuJoCo 3.4.0)
//! - C: Trajectory comparison
//! - D: Property/invariant tests

mod common;
mod layer_a;
mod layer_b;
mod layer_d;
