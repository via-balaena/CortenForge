//! Core physics simulation engine.
//!
//! This crate provides the simulation loop, world management, and numerical
//! integration for physics-based simulations. It builds on [`sim_types`] for
//! the data structures.
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                        Stepper                               │
//! │  Orchestrates: forces → integration → time advancement      │
//! └─────────────────────────┬───────────────────────────────────┘
//!                           │
//!                           ▼
//! ┌─────────────────────────────────────────────────────────────┐
//! │                         World                                │
//! │  Contains: bodies, joints, configuration, time              │
//! │  Provides: entity management, force application, queries    │
//! └─────────────────────────┬───────────────────────────────────┘
//!                           │
//!                           ▼
//! ┌─────────────────────────────────────────────────────────────┐
//! │                      Integrators                             │
//! │  Euler, Semi-Implicit Euler, Velocity Verlet, RK4           │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Layer 0 Crate
//!
//! This is a Layer 0 crate with **zero Bevy dependencies**. It can be used in:
//!
//! - Headless training loops
//! - Hardware control code
//! - Analysis tools
//! - Other engines
//!
//! # Quick Start
//!
//! ```
//! use sim_core::{World, Stepper, Body};
//! use sim_types::{RigidBodyState, Pose, MassProperties, SimulationConfig};
//! use nalgebra::Point3;
//!
//! // Create a world with default config (Earth gravity, 240Hz)
//! let mut world = World::new(SimulationConfig::default());
//!
//! // Add a falling sphere
//! let body_id = world.add_body(
//!     RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 10.0))),
//!     MassProperties::sphere(1.0, 0.5),
//! );
//!
//! // Create a stepper and simulate
//! let mut stepper = Stepper::new();
//!
//! // Run for 1 second of simulation time
//! let observations = stepper.run_for(&mut world, 1.0).unwrap();
//!
//! // Check final state
//! let final_obs = observations.last().unwrap();
//! let final_state = final_obs.body_state(body_id).unwrap();
//! println!("Final position: {:?}", final_state.pose.position);
//! ```
//!
//! # Integration Methods
//!
//! The crate supports multiple integration methods with different trade-offs:
//!
//! | Method | Order | Symplectic | Cost | Best For |
//! |--------|-------|------------|------|----------|
//! | Explicit Euler | 1 | No | Low | Simple tests |
//! | Semi-Implicit Euler | 1 | Yes | Low | Real-time, games |
//! | Velocity Verlet | 2 | Yes | Medium | General purpose |
//! | RK4 | 4 | No | High | High accuracy |
//!
//! Symplectic integrators conserve energy over long simulations, making them
//! preferred for physics applications.
//!
//! # Actions
//!
//! Control the simulation by submitting actions:
//!
//! ```
//! use sim_core::{World, Stepper};
//! use sim_types::{Action, ActionType, ExternalForce, BodyId};
//! use nalgebra::Vector3;
//!
//! let mut world = World::default();
//! let body_id = world.add_body(
//!     sim_types::RigidBodyState::origin(),
//!     sim_types::MassProperties::sphere(1.0, 0.5),
//! );
//!
//! let mut stepper = Stepper::new();
//!
//! // Apply an impulse
//! let force = ExternalForce::at_com(body_id, Vector3::new(100.0, 0.0, 0.0));
//! stepper.submit_action(Action::immediate(ActionType::force(force)));
//!
//! stepper.step(&mut world).unwrap();
//! ```
//!
//! # Diagnostics
//!
//! The world provides diagnostic methods:
//!
//! ```
//! use sim_core::World;
//! use sim_types::{RigidBodyState, MassProperties, Twist};
//! use nalgebra::Vector3;
//!
//! let mut world = World::default();
//! world.add_body(
//!     RigidBodyState::new(
//!         sim_types::Pose::identity(),
//!         Twist::linear(Vector3::new(1.0, 0.0, 0.0)),
//!     ),
//!     MassProperties::sphere(2.0, 0.5),
//! );
//!
//! println!("Total kinetic energy: {} J", world.total_kinetic_energy());
//! println!("Total momentum: {:?}", world.total_linear_momentum());
//! ```

#![doc(html_root_url = "https://docs.rs/sim-core/0.7.0")]
#![deny(clippy::unwrap_used, clippy::expect_used)]
#![warn(missing_docs)]
#![allow(
    clippy::missing_const_for_fn,     // Many methods can't be const due to nalgebra
    clippy::suboptimal_flops,          // mul_add style changes aren't always clearer
)]

pub mod broad_phase;
pub mod integrators;
mod stepper;
mod world;

pub use broad_phase::{
    Aabb, Axis, BroadPhase, BroadPhaseAlgorithm, BroadPhaseConfig, BroadPhaseDetector, BruteForce,
    SweepAndPrune,
};
pub use stepper::{SimulationBuilder, StepResult, Stepper, StepperConfig};
pub use world::{Body, CollisionShape, Joint, World};

// Re-export key types from sim-types for convenience
pub use sim_types::{
    Action, ActionType, BodyId, ExternalForce, Gravity, IntegrationMethod, JointCommand,
    JointCommandType, JointId, JointLimits, JointState, JointType, MassProperties, Observation,
    ObservationType, Pose, RigidBodyState, SimError, SimulationConfig, SolverConfig, Twist,
};

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names,
    clippy::unreadable_literal,
    clippy::uninlined_format_args,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::manual_range_contains,
    clippy::map_unwrap_or
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::{Point3, Vector3};

    #[test]
    fn test_basic_simulation() {
        let mut world = World::new(SimulationConfig::default());

        // Add a body at height 10m
        let body_id = world.add_body(
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 10.0))),
            MassProperties::sphere(1.0, 0.5),
        );

        let mut stepper = Stepper::new();

        // Run for 0.5 seconds
        let observations = stepper
            .run_for(&mut world, 0.5)
            .expect("simulation should succeed");

        // Should have observations
        assert!(!observations.is_empty());

        // Body should have fallen
        let final_state = world.body(body_id).expect("body should exist");
        assert!(final_state.state.pose.position.z < 10.0);
    }

    #[test]
    fn test_momentum_conservation() {
        // In zero gravity with no external forces, momentum should be conserved
        let mut world = World::new(SimulationConfig::default().zero_gravity());

        // Two bodies moving towards each other
        world.add_body(
            RigidBodyState::new(
                Pose::from_position(Point3::new(-5.0, 0.0, 0.0)),
                Twist::linear(Vector3::new(1.0, 0.0, 0.0)),
            ),
            MassProperties::sphere(1.0, 0.5),
        );
        world.add_body(
            RigidBodyState::new(
                Pose::from_position(Point3::new(5.0, 0.0, 0.0)),
                Twist::linear(Vector3::new(-1.0, 0.0, 0.0)),
            ),
            MassProperties::sphere(1.0, 0.5),
        );

        let initial_momentum = world.total_linear_momentum();

        let mut stepper = Stepper::with_config(StepperConfig::zero_gravity());

        // Run for a while (no collision handling, so they pass through)
        stepper
            .run_for(&mut world, 1.0)
            .expect("simulation should succeed");

        let final_momentum = world.total_linear_momentum();

        // Momentum should be conserved
        assert_relative_eq!(
            initial_momentum.norm(),
            final_momentum.norm(),
            epsilon = 1e-10
        );
    }

    #[test]
    fn test_energy_trend() {
        // Free falling body should gain kinetic energy equal to lost potential energy
        let mut world = World::new(SimulationConfig::high_fidelity());

        let initial_height = 10.0;
        let mass = 1.0;

        world.add_body(
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, initial_height))),
            MassProperties::sphere(mass, 0.5),
        );

        let g = 9.81;
        let initial_potential = mass * g * initial_height;
        let initial_kinetic = world.total_kinetic_energy();
        let initial_total = initial_potential + initial_kinetic;

        let mut stepper = Stepper::new();
        stepper
            .run_for(&mut world, 0.5)
            .expect("simulation should succeed");

        let body = world.bodies().next().expect("should have body");
        let final_height = body.state.pose.position.z;
        let final_potential = mass * g * final_height;
        let final_kinetic = world.total_kinetic_energy();
        let final_total = final_potential + final_kinetic;

        // Total energy should be approximately conserved
        // (some drift expected due to numerical integration)
        let energy_drift = (final_total - initial_total).abs() / initial_total;
        assert!(
            energy_drift < 0.01,
            "Energy drift too large: {}%",
            energy_drift * 100.0
        );
    }

    #[test]
    fn test_static_ground() {
        let mut world = World::new(SimulationConfig::default());

        // Static ground
        world.add_static_body(Pose::from_position(Point3::new(0.0, 0.0, 0.0)));

        // Falling body (note: no collision detection in core, will pass through)
        world.add_body(
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 1.0))),
            MassProperties::sphere(1.0, 0.5),
        );

        let mut stepper = Stepper::new();
        stepper
            .run_for(&mut world, 1.0)
            .expect("simulation should succeed");

        // Ground should not have moved
        let ground = world
            .bodies()
            .find(|b| b.is_static)
            .expect("should have ground");
        assert_relative_eq!(ground.state.pose.position.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_builder_pattern() {
        let config = SimulationConfig::realtime().zero_gravity();
        let world = World::new(config);

        let (world, stepper) = SimulationBuilder::new()
            .world(world)
            .stepper_config(StepperConfig::zero_gravity())
            .build();

        assert!(!stepper.config().apply_gravity);
        assert_eq!(world.body_count(), 0);
    }
}
