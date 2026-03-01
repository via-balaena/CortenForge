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

/// Default class resolution: wiring DefaultResolver into builder/.
pub mod default_classes;

// ============================================================================
// Spatial Tendon Tests (Acceptance tests for §4 Spatial Tendons + Wrapping)
// ============================================================================

/// Spatial tendon tests: pulley divisor, wrapping, Jacobian correctness.
pub mod spatial_tendons;

// ============================================================================
// Tendon Springlength Tests (§22 Tendon springlength + Deadband Physics)
// ============================================================================

/// Tendon springlength: parsing, defaults, sentinel, deadband physics.
pub mod tendon_springlength;

// ============================================================================
// Exact Mesh Inertia Tests (§23 exactmeshinertia + Full Tensor Pipeline)
// ============================================================================

/// Exact mesh inertia: parsing, algorithm, pipeline refactor, multi-geom.
pub mod exactmeshinertia;

// ============================================================================
// Site-Transmission Actuator Tests (§5 Site-Transmission Actuators)
// ============================================================================

/// Site-transmission tests: Mode A/B, common-ancestor, sensors, validation.
pub mod site_transmission;

// ============================================================================
// Activation Clamping Tests (§34 actlimited/actrange/actearly)
// ============================================================================

/// Activation clamping tests: actlimited, actrange, actearly, muscle defaults.
pub mod activation_clamping;

// ============================================================================
// Gravity Compensation Tests (§35 gravcomp)
// ============================================================================

/// Gravcomp tests: full/partial/over/negative compensation, kinematic chain,
/// sleep filtering, CoM offset, Jacobian projection, qfrc_gravcomp isolation.
pub mod gravcomp;

// ============================================================================
// Body-Transmission Actuator Tests (§36 adhesion)
// ============================================================================

/// Adhesion tests: moment computation, contact normal Jacobian, force sign,
/// gear bypass, multiple contacts, kinematic chain, two-body contacts.
pub mod adhesion;

// ============================================================================
// Tendon Equality Constraint Tests (§37)
// ============================================================================

/// Tendon equality tests: two-tendon coupling, single-tendon lock, polynomial
/// coupling, solver compatibility, solref/solimp sensitivity, inactive bypass.
pub mod tendon_equality;

// ============================================================================
// Existing Test Modules
// ============================================================================

pub mod batch_sim;
pub mod cg_solver;
pub mod mjcf_sensors;
pub mod model_data_pipeline;
pub mod passive_forces;
pub mod sensors;
pub mod validation;

// ============================================================================
// Analytical Derivatives Tests (§12, Part 1: Steps 0–7)
// ============================================================================

/// Derivative infrastructure tests: FD, analytical qDeriv, acceptance criteria 1–23.
pub mod derivatives;

// ============================================================================
// Keyframe & Mocap Body Tests (§14)
// ============================================================================

/// Keyframe parsing, reset, mocap body semantics, validation error tests.
pub mod keyframes;

// ============================================================================
// Newton Solver Tests (§15 Phase A + B)
// ============================================================================

/// Newton solver integration tests: convergence, warmstart, fallbacks.
pub mod newton_solver;

// ============================================================================
// Sleeping / Body Deactivation Tests (§16)
// ============================================================================

/// Sleep deactivation tests: tree enumeration, policies, wake detection,
/// pipeline skip, sensor freezing, batch independence.
pub mod sleeping;

// ============================================================================
// Flex Solver Unification Tests (§6b)
// ============================================================================

/// Flex unification acceptance tests: AC1–AC15 from future_work_6b_precursor_to_7.
pub mod flex_unified;

// ============================================================================
// Unified PGS/CG Solver Tests (§29)
// ============================================================================

/// Unified PGS/CG solver tests: ne/nf bookkeeping, all constraint types, cost guard, primal CG.
pub mod unified_solvers;

// ============================================================================
// Noslip Post-Processor Tests (§33)
// ============================================================================

/// Noslip acceptance tests: PGS/CG/Newton noslip, friction-loss, pyramidal.
pub mod noslip;

// ============================================================================
// Ball Joint Limits Tests (§38)
// ============================================================================

/// Ball joint limit tests: cone constraint, Jacobian, wrapping, multi-joint,
/// near-π, degenerate range, negative-w, unnormalized quaternion, margin anchor.
pub mod ball_joint_limits;

// ============================================================================
// Fluid / Aerodynamic Force Tests (§40)
// ============================================================================

/// Fluid force tests: inertia-box model, ellipsoid model, body dispatch,
/// default inheritance, kappa quadrature, mass guard, zero fluid regression.
#[allow(clippy::excessive_precision, clippy::needless_range_loop)]
pub mod fluid_forces;

// ============================================================================
// Fluid Force Velocity Derivative Tests (§40a)
// ============================================================================

/// Fluid derivative tests: ∂qfrc_fluid/∂qvel for both models, FD validation,
/// energy dissipation, symmetry, guards, joint types, multi-body.
#[allow(clippy::excessive_precision, clippy::needless_range_loop)]
pub mod fluid_derivatives;

// ============================================================================
// DT-35: Tendon Implicit Spring/Damper Tests
// ============================================================================

/// Tendon implicit spring/damper tests: non-diagonal K/D coupling, deadband,
/// energy dissipation, Newton solver, spatial tendons, multi-tendon summation.
pub mod tendon_implicit;

// ============================================================================
// §41 Runtime Flag Conformance Tests
// ============================================================================

/// Runtime flag conformance tests: AC1–AC48 behavioral gating verification.
pub mod runtime_flags;

// ============================================================================
// §41 AC18 Golden-File Flag Conformance Tests
// ============================================================================

/// Golden-file conformance tests: MuJoCo-reference .npy comparison for flag behavior.
pub mod golden_flags;

// ============================================================================
// §53 Split-Step API Tests
// ============================================================================

/// Split-step tests: step1/step2 equivalence, force injection.
pub mod split_step;

// ============================================================================
// §59 Name Lookup API Tests
// ============================================================================

/// Name lookup tests: name2id/id2name round-trip, nonexistent, out-of-bounds.
pub mod name_lookup;

// ============================================================================
// §51 Body Force Accumulator Tests
// ============================================================================

/// Body force accumulator tests: cacc, cfrc_int, cfrc_ext.
pub mod body_accumulators;

// ============================================================================
// §52 Inverse Dynamics Tests
// ============================================================================

/// Inverse dynamics tests: round-trip, gravity-only, free body.
pub mod inverse_dynamics;

// ============================================================================
// DT-79 User Callback Tests
// ============================================================================

/// User callback tests: passive, control, contact filter, clone safety.
pub mod callbacks;

// ============================================================================
// DT-21 xfrc_applied Projection Tests
// ============================================================================

/// xfrc_applied tests: free body, hinge torque, anti-gravity, pure torque.
pub mod xfrc_applied;

// ============================================================================
// Phase 4 Audit Tests (CVEL fixes, §56 subtree, lazy gates, acc sensors)
// ============================================================================

/// Phase 4 branch audit: AC-level coverage for CVEL fixes, §56 subtree fields,
/// flg_rnepost lazy gate, and 4A.6 acc-stage sensor refactor.
pub mod sensors_phase4;

// ============================================================================
// DT-103: Spatial Transport Helper Tests
// ============================================================================

/// Spatial transport tests: object_velocity, object_acceleration, object_force,
/// backward compatibility, world body edge cases.
pub mod spatial_transport;

// ============================================================================
// Phase 5: Actuator Completeness Tests (acc0, dampratio, lengthrange)
// ============================================================================

/// Phase 5 Spec A tests: acc0 for all actuator types, dampratio MJCF round-trip,
/// lengthrange from limits.
pub mod actuator_phase5;

// ============================================================================
// Phase 6: Sensor Completeness Tests (Spec A)
// ============================================================================

/// Phase 6 Spec A tests: objtype attribute parsing (DT-62), multi-geom touch
/// sensor aggregation (DT-64), geom-attached FrameLinAcc/FrameAngAcc (DT-102).
pub mod sensor_phase6;

// ============================================================================
// Phase 6: Sensor Completeness Tests (Spec C)
// ============================================================================

/// Phase 6 Spec C tests: Clock, JointActuatorFrc, GeomDist, GeomNormal, GeomFromTo.
pub mod sensor_phase6_spec_c;
