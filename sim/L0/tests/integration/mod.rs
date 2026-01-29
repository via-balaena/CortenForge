//! Integration tests for the sim-* crate ecosystem.
//!
//! These tests verify end-to-end functionality of the simulation pipeline:
//! - MJCF loading → Model/Data simulation (MuJoCo-aligned API)
//! - URDF loading → Model/Data simulation (via MJCF conversion)
//! - Musculoskeletal dynamics (muscles, tendons)
//! - Sensor readings from simulation state
//! - Phase 7 validation tests for MuJoCo consolidation
//! - Phase 6 collision detection validation (COLLISION_UPGRADE_SPEC.md)

// ============================================================================
// Phase 6: Collision Detection Tests (COLLISION_UPGRADE_SPEC.md)
// ============================================================================

/// Shared test utilities: tolerances, rotations, assertion helpers.
#[macro_use]
pub mod collision_test_utils;

/// Plane collision tests: all primitives vs infinite plane.
pub mod collision_plane;

/// Primitive-primitive collision tests: analytical pairs.
pub mod collision_primitives;

/// Edge case tests: numerical pathology, degenerate geometry.
pub mod collision_edge_cases;

/// Performance tests: regression gates, scaling validation.
pub mod collision_performance;

// ============================================================================
// Phase 1.1: Equality Constraint Tests (MUJOCO_PARITY_SPEC.md)
// ============================================================================

/// Equality constraint tests: connect, weld, joint coupling.
pub mod equality_constraints;

// ============================================================================
// Phase 1.2: Implicit Integration Tests (MUJOCO_PARITY_SPEC.md)
// ============================================================================

/// Implicit spring-damper integration tests: stability, accuracy, convergence.
pub mod implicit_integration;

// ============================================================================
// Existing Test Modules
// ============================================================================

pub mod model_data_pipeline;
pub mod musculoskeletal;
pub mod passive_forces;
pub mod sensors;
pub mod validation;
