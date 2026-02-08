//! Integration tests for the sim-* crate ecosystem.
//!
//! These tests verify end-to-end functionality of the simulation pipeline:
//! - MJCF loading → Model/Data simulation (MuJoCo-aligned API)
//! - URDF loading → Model/Data simulation (via MJCF conversion)
//! - Musculoskeletal dynamics (muscles, tendons)
//! - Sensor readings from simulation state
//! - Phase 7 validation tests for MuJoCo consolidation
//! - Phase 6 collision detection validation

// ============================================================================
// Phase 6: Collision Detection Tests
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
// Equality Constraint Tests (connect, weld, joint coupling)
// ============================================================================

/// Equality constraint tests: connect, weld, joint coupling.
pub mod equality_constraints;

// ============================================================================
// Implicit Integration Tests (spring-damper stability)
// ============================================================================

/// Implicit spring-damper integration tests: stability, accuracy, convergence.
pub mod implicit_integration;

// ============================================================================
// RK4 Integration Tests (True 4-stage Runge-Kutta)
// ============================================================================

/// RK4 integration tests: order-of-convergence, energy conservation, quaternion
/// correctness, contact handling, sensor non-corruption, warmstart preservation.
pub mod rk4_integration;

// ============================================================================
// Default Class Resolution Tests
// ============================================================================

/// Default class resolution: wiring DefaultResolver into model_builder.
pub mod default_classes;

// ============================================================================
// Spatial Tendon Tests (Acceptance tests for §4 Spatial Tendons + Wrapping)
// ============================================================================

/// Spatial tendon tests: pulley divisor, wrapping, Jacobian correctness.
pub mod spatial_tendons;

// ============================================================================
// Site-Transmission Actuator Tests (§5 Site-Transmission Actuators)
// ============================================================================

/// Site-transmission tests: Mode A/B, common-ancestor, sensors, validation.
pub mod site_transmission;

// ============================================================================
// Existing Test Modules
// ============================================================================

pub mod batch_sim;
pub mod cg_solver;
pub mod mjcf_sensors;
pub mod model_data_pipeline;
pub mod musculoskeletal;
pub mod passive_forces;
pub mod sensors;
pub mod validation;

// ============================================================================
// Deformable Body Pipeline Tests (§11, feature-gated)
// ============================================================================

/// Deformable-rigid contact pipeline tests.
#[cfg(feature = "deformable")]
pub mod deformable_contact;
