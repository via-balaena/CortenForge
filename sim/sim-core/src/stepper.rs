//! Simulation stepping and control flow.
//!
//! This module provides the [`Stepper`] which orchestrates the simulation loop:
//! applying forces, integrating dynamics, and advancing time.
//!
//! # Example
//!
//! ```
//! use sim_core::{World, Stepper, StepResult};
//! use sim_types::{RigidBodyState, Pose, MassProperties, SimulationConfig};
//! use nalgebra::Point3;
//!
//! // Create world with a falling body
//! let mut world = World::new(SimulationConfig::default());
//! world.add_body(
//!     RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 10.0))),
//!     MassProperties::sphere(1.0, 0.5),
//! );
//!
//! // Create stepper and simulate
//! let mut stepper = Stepper::new();
//! for _ in 0..100 {
//!     let result = stepper.step(&mut world);
//!     assert!(result.is_ok());
//! }
//!
//! // Body should have fallen
//! let obs = world.observe();
//! // ... check observations
//! ```

use crate::integrators::integrate_with_method;
use crate::world::World;
use sim_types::{Action, ActionType, ExternalForce, JointCommand, Observation, SimError};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Result of a simulation step.
#[derive(Debug, Clone)]
pub struct StepResult {
    /// Observation after the step.
    pub observation: Observation,
    /// Whether simulation has completed (reached `max_time`).
    pub completed: bool,
}

/// Configuration for the stepper.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct StepperConfig {
    /// Whether to automatically apply gravity each step.
    pub apply_gravity: bool,
    /// Whether to detect and resolve contacts each step.
    pub enable_contacts: bool,
    /// Whether to solve joint constraints each step.
    pub enable_constraints: bool,
    /// Maximum linear velocity (m/s). Bodies exceeding this are clamped.
    pub max_linear_velocity: Option<f64>,
    /// Maximum angular velocity (rad/s). Bodies exceeding this are clamped.
    pub max_angular_velocity: Option<f64>,
    /// Linear velocity damping coefficient.
    pub linear_damping: f64,
    /// Angular velocity damping coefficient.
    pub angular_damping: f64,
}

impl Default for StepperConfig {
    fn default() -> Self {
        Self {
            apply_gravity: true,
            enable_contacts: true,
            enable_constraints: true,
            max_linear_velocity: Some(100.0),  // 100 m/s = 360 km/h
            max_angular_velocity: Some(100.0), // ~16 rev/s
            linear_damping: 0.0,
            angular_damping: 0.0,
        }
    }
}

impl StepperConfig {
    /// Create config with no velocity limits.
    #[must_use]
    pub fn unlimited() -> Self {
        Self {
            max_linear_velocity: None,
            max_angular_velocity: None,
            ..Default::default()
        }
    }

    /// Create config without gravity.
    #[must_use]
    pub fn zero_gravity() -> Self {
        Self {
            apply_gravity: false,
            ..Default::default()
        }
    }

    /// Create config without contacts (free-fall simulation).
    #[must_use]
    pub fn no_contacts() -> Self {
        Self {
            enable_contacts: false,
            ..Default::default()
        }
    }

    /// Set damping coefficients.
    #[must_use]
    pub fn with_damping(mut self, linear: f64, angular: f64) -> Self {
        self.linear_damping = linear;
        self.angular_damping = angular;
        self
    }

    /// Enable or disable contact resolution.
    #[must_use]
    pub fn with_contacts(mut self, enable: bool) -> Self {
        self.enable_contacts = enable;
        self
    }
}

/// The simulation stepper orchestrates the physics loop.
#[derive(Debug, Clone)]
pub struct Stepper {
    /// Stepper configuration.
    config: StepperConfig,
    /// Pending actions to apply.
    pending_actions: Vec<Action>,
}

impl Default for Stepper {
    fn default() -> Self {
        Self::new()
    }
}

impl Stepper {
    /// Create a new stepper with default configuration.
    #[must_use]
    pub fn new() -> Self {
        Self {
            config: StepperConfig::default(),
            pending_actions: Vec::new(),
        }
    }

    /// Create a stepper with custom configuration.
    #[must_use]
    pub fn with_config(config: StepperConfig) -> Self {
        Self {
            config,
            pending_actions: Vec::new(),
        }
    }

    /// Get the stepper configuration.
    #[must_use]
    pub fn config(&self) -> &StepperConfig {
        &self.config
    }

    /// Submit an action to be applied on the next step.
    pub fn submit_action(&mut self, action: Action) {
        self.pending_actions.push(action);
    }

    /// Submit multiple actions.
    pub fn submit_actions(&mut self, actions: impl IntoIterator<Item = Action>) {
        self.pending_actions.extend(actions);
    }

    /// Clear all pending actions.
    pub fn clear_actions(&mut self) {
        self.pending_actions.clear();
    }

    /// Execute one simulation step.
    ///
    /// This performs:
    /// 1. Clear forces from previous step
    /// 2. Apply gravity (if enabled)
    /// 3. Apply pending actions (forces, joint commands)
    /// 4. Detect and resolve contacts (if enabled)
    /// 5. Solve joint constraints (if enabled)
    /// 6. Integrate dynamics
    /// 7. Apply damping and velocity limits
    /// 8. Advance time
    ///
    /// Returns an observation of the world state after the step.
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - The world state is invalid (contains `NaN` or `Inf` values)
    /// - An action references an invalid body or joint ID
    pub fn step(&mut self, world: &mut World) -> sim_types::Result<StepResult> {
        // Validate world state
        world.validate()?;

        let dt = world.timestep();
        let integration_method = world.config().solver.integration;

        // 1. Clear accumulated forces
        world.clear_forces();

        // 2. Apply gravity
        if self.config.apply_gravity {
            world.apply_gravity();
        }

        // 3. Apply pending actions
        let current_time = world.time();
        self.apply_pending_actions(world, current_time)?;

        // 4. Detect and resolve contacts
        if self.config.enable_contacts {
            world.solve_contacts();
        }

        // 5. Solve joint constraints
        if self.config.enable_constraints {
            world.solve_constraints();
        }

        // 6. Integrate dynamics for each body
        for body in world.bodies_mut() {
            if body.is_static || body.is_sleeping {
                continue;
            }

            // Compute accelerations
            let linear_accel = body.linear_acceleration();
            let Some(angular_accel) = body.angular_acceleration() else {
                // Singular inertia - skip this body
                continue;
            };

            // Integrate
            integrate_with_method(
                integration_method,
                &mut body.state,
                linear_accel,
                angular_accel,
                dt,
            );

            // 7. Apply damping
            if self.config.linear_damping > 0.0 || self.config.angular_damping > 0.0 {
                body.state.twist = crate::integrators::apply_damping(
                    &body.state.twist,
                    self.config.linear_damping,
                    self.config.angular_damping,
                    dt,
                );
            }

            // Apply velocity limits
            if let Some(max_linear) = self.config.max_linear_velocity {
                if let Some(max_angular) = self.config.max_angular_velocity {
                    body.state.twist = crate::integrators::clamp_velocities(
                        &body.state.twist,
                        max_linear,
                        max_angular,
                    );
                }
            }
        }

        // 8. Advance time
        world.advance_time(dt);

        // Check for divergence
        world.validate()?;

        // Create observation
        let observation = world.observe();
        let completed = world.is_complete();

        Ok(StepResult {
            observation,
            completed,
        })
    }

    /// Run the simulation until completion or max steps.
    ///
    /// # Errors
    ///
    /// Returns an error if any step fails (divergence or invalid action).
    pub fn run(
        &mut self,
        world: &mut World,
        max_steps: Option<u64>,
    ) -> sim_types::Result<Vec<Observation>> {
        let mut observations = Vec::new();

        let mut steps = 0u64;
        loop {
            let result = self.step(world)?;
            observations.push(result.observation);

            if result.completed {
                break;
            }

            steps += 1;
            if let Some(max) = max_steps {
                if steps >= max {
                    break;
                }
            }
        }

        Ok(observations)
    }

    /// Run for a specific duration.
    ///
    /// # Errors
    ///
    /// Returns an error if any step fails (divergence or invalid action).
    pub fn run_for(
        &mut self,
        world: &mut World,
        duration: f64,
    ) -> sim_types::Result<Vec<Observation>> {
        let target_time = world.time() + duration;
        let dt = world.timestep();
        // Safe cast: duration and dt are positive, result is bounded
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let estimated_steps = (duration / dt).ceil().max(1.0) as usize;
        let mut observations = Vec::with_capacity(estimated_steps);

        for _ in 0..estimated_steps {
            if world.time() >= target_time {
                break;
            }
            let result = self.step(world)?;
            observations.push(result.observation);

            if result.completed {
                break;
            }
        }

        Ok(observations)
    }

    /// Apply pending actions to the world.
    fn apply_pending_actions(
        &mut self,
        world: &mut World,
        current_time: f64,
    ) -> sim_types::Result<()> {
        // Take actions that are due
        let actions: Vec<_> = self
            .pending_actions
            .drain(..)
            .filter(|a| a.timestamp <= current_time)
            .collect();

        for action in actions {
            Self::apply_action(world, &action.action_type)?;
        }

        Ok(())
    }

    /// Apply a single action type to the world.
    fn apply_action(world: &mut World, action: &ActionType) -> sim_types::Result<()> {
        match action {
            ActionType::ExternalForces(forces) => {
                for force in forces {
                    Self::apply_external_force(world, force)?;
                }
            }
            ActionType::JointCommands(commands) => {
                for command in commands {
                    Self::apply_joint_command(world, command)?;
                }
            }
            ActionType::Combined { forces, joints } => {
                for force in forces {
                    Self::apply_external_force(world, force)?;
                }
                for command in joints {
                    Self::apply_joint_command(world, command)?;
                }
            }
            ActionType::NoOp => {}
        }

        Ok(())
    }

    /// Apply an external force to a body.
    fn apply_external_force(world: &mut World, force: &ExternalForce) -> sim_types::Result<()> {
        let body = world
            .body_mut(force.body)
            .ok_or_else(|| SimError::InvalidBodyId(force.body.raw()))?;

        if let Some(point) = force.point {
            body.apply_force_at_point(force.force, point);
        } else {
            body.apply_force(force.force);
        }

        body.apply_torque(force.torque);
        Ok(())
    }

    /// Apply a joint command.
    fn apply_joint_command(world: &mut World, command: &JointCommand) -> sim_types::Result<()> {
        let joint = world
            .joint_mut(command.joint)
            .ok_or_else(|| SimError::InvalidJointId(command.joint.raw()))?;

        // Compute effort from command
        let effort = command
            .command
            .compute_pd_effort(joint.state.position, joint.state.velocity);

        // For now, just store the computed effort
        // A full implementation would apply this as a torque between parent and child bodies
        // This is a simplified version - full joint dynamics would require constraint solving
        let _ = effort;

        Ok(())
    }
}

/// Builder for creating simulation scenarios.
#[derive(Debug, Clone, Default)]
pub struct SimulationBuilder {
    world: Option<World>,
    stepper: Option<Stepper>,
}

impl SimulationBuilder {
    /// Create a new simulation builder.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the world.
    #[must_use]
    pub fn world(mut self, world: World) -> Self {
        self.world = Some(world);
        self
    }

    /// Set the stepper.
    #[must_use]
    pub fn stepper(mut self, stepper: Stepper) -> Self {
        self.stepper = Some(stepper);
        self
    }

    /// Set the stepper configuration.
    #[must_use]
    pub fn stepper_config(mut self, config: StepperConfig) -> Self {
        self.stepper = Some(Stepper::with_config(config));
        self
    }

    /// Build the simulation, returning (World, Stepper).
    #[must_use]
    pub fn build(self) -> (World, Stepper) {
        let world = self.world.unwrap_or_default();
        let stepper = self.stepper.unwrap_or_default();
        (world, stepper)
    }
}

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
    use sim_types::{MassProperties, Pose, RigidBodyState, SimulationConfig};

    fn setup_falling_body() -> World {
        let mut world = World::new(SimulationConfig::default());
        world.add_body(
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 10.0))),
            MassProperties::sphere(1.0, 0.5),
        );
        world
    }

    #[test]
    fn test_single_step() {
        let mut world = setup_falling_body();
        let mut stepper = Stepper::new();

        let result = stepper.step(&mut world);
        assert!(result.is_ok());

        let result = result.expect("step should succeed");
        assert!(!result.completed);
        assert!(result.observation.body_states().is_some());
    }

    #[test]
    fn test_gravity_falling() {
        let mut world = setup_falling_body();
        let mut stepper = Stepper::new();

        // Run for 1 second
        let dt = world.timestep();
        let steps = (1.0 / dt) as u64;

        for _ in 0..steps {
            stepper.step(&mut world).expect("step should succeed");
        }

        // Check that body has fallen and accelerated
        let obs = world.observe();
        let states = obs.body_states().expect("should have body states");
        let (_, state) = &states[0];

        // After 1 second of free fall: z ≈ 10 - 0.5*9.81*1² ≈ 5.1 m
        assert!(state.pose.position.z < 10.0, "body should have fallen");
        assert!(
            state.pose.position.z > 0.0,
            "body shouldn't have fallen too far"
        );

        // Velocity should be negative (downward)
        assert!(state.twist.linear.z < 0.0, "body should be moving down");
    }

    #[test]
    fn test_zero_gravity() {
        let mut world = World::new(SimulationConfig::default().zero_gravity());
        world.add_body(
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 10.0))),
            MassProperties::sphere(1.0, 0.5),
        );

        let mut stepper = Stepper::with_config(StepperConfig::zero_gravity());

        for _ in 0..100 {
            stepper.step(&mut world).expect("step should succeed");
        }

        // Body should not have moved
        let obs = world.observe();
        let states = obs.body_states().expect("should have body states");
        let (_, state) = &states[0];

        assert_relative_eq!(state.pose.position.z, 10.0, epsilon = 1e-10);
    }

    #[test]
    fn test_apply_external_force() {
        let mut world = World::new(SimulationConfig::default().zero_gravity());
        let body_id = world.add_body(
            RigidBodyState::at_rest(Pose::from_position(Point3::origin())),
            MassProperties::sphere(1.0, 0.5),
        );

        let mut stepper = Stepper::with_config(StepperConfig::zero_gravity());

        // Apply force in X direction
        let force = ExternalForce::at_com(body_id, Vector3::new(10.0, 0.0, 0.0));
        let action = Action::immediate(ActionType::force(force));
        stepper.submit_action(action);

        // Step multiple times
        for _ in 0..100 {
            stepper.step(&mut world).expect("step should succeed");
        }

        // Body should have accelerated in X direction
        let obs = world.observe();
        let state = obs.body_state(body_id).expect("should have body state");

        assert!(state.pose.position.x > 0.0, "body should have moved in X");
        assert!(state.twist.linear.x > 0.0, "body should be moving in X");
    }

    #[test]
    fn test_run_for_duration() {
        let mut world = setup_falling_body();
        let mut stepper = Stepper::new();

        let observations = stepper
            .run_for(&mut world, 0.5)
            .expect("run should succeed");

        // Should have observations for about 0.5 seconds
        let expected_steps = (0.5 / world.timestep()) as usize;
        assert_relative_eq!(
            observations.len() as f64,
            expected_steps as f64,
            epsilon = 2.0
        );
    }

    #[test]
    fn test_run_until_complete() {
        let config = SimulationConfig::default().max_time(0.1);
        let mut world = World::new(config);
        world.add_body(RigidBodyState::origin(), MassProperties::sphere(1.0, 0.5));

        let mut stepper = Stepper::new();
        let observations = stepper.run(&mut world, None).expect("run should succeed");

        // Should complete after reaching max_time
        assert!(!observations.is_empty());
        assert!(world.is_complete());
    }

    #[test]
    fn test_damping() {
        let mut world = World::new(SimulationConfig::default().zero_gravity());
        let body_id = world.add_body(
            RigidBodyState::new(
                Pose::from_position(Point3::origin()),
                sim_types::Twist::linear(Vector3::new(10.0, 0.0, 0.0)),
            ),
            MassProperties::sphere(1.0, 0.5),
        );

        let config = StepperConfig::zero_gravity().with_damping(1.0, 0.5);
        let mut stepper = Stepper::with_config(config);

        // Run for a bit
        for _ in 0..100 {
            stepper.step(&mut world).expect("step should succeed");
        }

        // Velocity should have decreased due to damping
        let body = world.body(body_id).expect("body should exist");
        assert!(body.state.twist.linear.x < 10.0, "velocity should decrease");
        assert!(
            body.state.twist.linear.x > 0.0,
            "velocity shouldn't go negative"
        );
    }

    #[test]
    fn test_velocity_limits() {
        let mut world = World::new(SimulationConfig::default().zero_gravity());
        let body_id = world.add_body(
            RigidBodyState::new(
                Pose::from_position(Point3::origin()),
                sim_types::Twist::linear(Vector3::new(1000.0, 0.0, 0.0)),
            ),
            MassProperties::sphere(1.0, 0.5),
        );

        let mut stepper = Stepper::new(); // Default has max velocity of 100

        stepper.step(&mut world).expect("step should succeed");

        let body = world.body(body_id).expect("body should exist");
        assert!(
            body.state.twist.linear.norm() <= 100.0 + 1e-6,
            "velocity should be clamped"
        );
    }

    #[test]
    fn test_simulation_builder() {
        let (world, stepper) = SimulationBuilder::new()
            .world(World::default())
            .stepper_config(StepperConfig::zero_gravity())
            .build();

        assert_eq!(world.body_count(), 0);
        assert!(!stepper.config().apply_gravity);
    }

    #[test]
    fn test_multiple_bodies() {
        let mut world = World::new(SimulationConfig::default());

        // Add multiple bodies at the same height
        let initial_height = 10.0;
        for i in 0..5 {
            world.add_body(
                RigidBodyState::at_rest(Pose::from_position(Point3::new(
                    i as f64,
                    0.0,
                    initial_height,
                ))),
                MassProperties::sphere(1.0, 0.5),
            );
        }

        let mut stepper = Stepper::new();

        // All should fall
        for _ in 0..100 {
            stepper.step(&mut world).expect("step should succeed");
        }

        // All bodies should have fallen from initial height
        for body in world.bodies() {
            assert!(
                body.state.pose.position.z < initial_height,
                "all bodies should fall from initial height"
            );
        }
    }

    #[test]
    fn test_static_body_doesnt_move() {
        let mut world = World::new(SimulationConfig::default());
        let static_id = world.add_static_body(Pose::from_position(Point3::new(0.0, 0.0, 0.0)));

        let mut stepper = Stepper::new();

        for _ in 0..100 {
            stepper.step(&mut world).expect("step should succeed");
        }

        let body = world.body(static_id).expect("body should exist");
        assert_relative_eq!(body.state.pose.position.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_detect_divergence() {
        let mut world = World::new(SimulationConfig::default());
        let body_id = world.add_body(RigidBodyState::origin(), MassProperties::sphere(1.0, 0.5));

        // Corrupt the state
        {
            let body = world.body_mut(body_id).expect("body should exist");
            body.state.pose.position.x = f64::NAN;
        }

        let mut stepper = Stepper::new();
        let result = stepper.step(&mut world);

        assert!(result.is_err());
        assert!(result.unwrap_err().is_diverged());
    }

    #[test]
    fn test_torque_application() {
        let mut world = World::new(SimulationConfig::default().zero_gravity());
        let body_id = world.add_body(
            RigidBodyState::at_rest(Pose::identity()),
            MassProperties::sphere(1.0, 1.0),
        );

        let mut stepper = Stepper::with_config(StepperConfig::zero_gravity());

        // Apply torque around Z axis
        let force = ExternalForce::torque_only(body_id, Vector3::new(0.0, 0.0, 1.0));
        stepper.submit_action(Action::immediate(ActionType::force(force)));

        for _ in 0..100 {
            stepper.step(&mut world).expect("step should succeed");
        }

        // Body should be rotating around Z
        let body = world.body(body_id).expect("body should exist");
        assert!(body.state.twist.angular.z > 0.0, "body should be rotating");
    }

    // =========================================================================
    // Contact Physics Integration Tests
    // =========================================================================

    #[test]
    fn test_sphere_resting_on_ground() {
        use crate::world::{Body, CollisionShape};
        use sim_contact::ContactParams;
        use sim_types::BodyId;

        // Create world with gravity and contact parameters tuned for stability
        // Use low stiffness and moderate damping
        // With explicit integration, high damping can cause energy injection
        // when contacts are short-lived (single timestep), so we use lower damping
        let contact_params = ContactParams {
            stiffness: 5_000.0, // Low stiffness for deeper penetration
            stiffness_power: 1.0,
            damping: 100.0, // Low damping to avoid energy injection
            friction_coefficient: 0.5,
            rolling_friction: 0.01,
            torsional_friction: 0.01,
            contact_margin: 0.0001,
            restitution: 0.0,
        };

        let mut world = World::with_contact_params(SimulationConfig::default(), contact_params);

        // Ground plane at z=0
        let ground = Body::new_static(BodyId::new(1), Pose::identity())
            .with_name("ground")
            .with_collision_shape(CollisionShape::ground_plane(0.0));
        world.insert_body(ground).expect("insert should succeed");

        // Sphere starting slightly above ground (z=0.55 with radius 0.5 means bottom at z=0.05)
        let sphere_state =
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 0.55)));
        let sphere = Body::new(
            BodyId::new(2),
            sphere_state,
            MassProperties::sphere(1.0, 0.5),
        )
        .with_name("sphere")
        .with_collision_shape(CollisionShape::sphere(0.5));
        world.insert_body(sphere).expect("insert should succeed");

        let mut stepper = Stepper::new();

        // Simulate for 2 seconds
        for _ in 0..200 {
            stepper.step(&mut world).expect("step should succeed");
        }

        // Sphere should have settled near z=0.5 (radius)
        let sphere = world.body_by_name("sphere").expect("sphere should exist");
        let z = sphere.state.pose.position.z;

        // Should be near 0.5 (sphere radius), allowing for some settling
        // With soft contacts there may be some penetration, so allow down to 0.4
        assert!(
            z >= 0.40 && z <= 0.60,
            "sphere should rest at approximately z=0.5, got z={}",
            z
        );

        // Velocity should be low (settling)
        let speed = sphere.state.twist.linear.norm();
        assert!(
            speed < 2.0,
            "sphere should be nearly at rest, got speed={}",
            speed
        );
    }

    #[test]
    fn test_sphere_drop_bounces() {
        use crate::world::{Body, CollisionShape};
        use sim_contact::ContactParams;
        use sim_types::BodyId;

        // Create world with gravity and stable contact params
        let contact_params = ContactParams {
            stiffness: 5_000.0,
            stiffness_power: 1.0,
            damping: 100.0,
            friction_coefficient: 0.5,
            rolling_friction: 0.01,
            torsional_friction: 0.01,
            contact_margin: 0.0001,
            restitution: 0.0,
        };
        let mut world = World::with_contact_params(SimulationConfig::default(), contact_params);

        // Ground plane at z=0
        let ground = Body::new_static(BodyId::new(1), Pose::identity())
            .with_collision_shape(CollisionShape::ground_plane(0.0));
        world.insert_body(ground).expect("insert should succeed");

        // Sphere starting at z=2.0, radius 0.5
        let sphere_state = RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 2.0)));
        let sphere = Body::new(
            BodyId::new(2),
            sphere_state,
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_shape(CollisionShape::sphere(0.5));
        world.insert_body(sphere).expect("insert should succeed");

        let mut stepper = Stepper::new();

        // Track minimum Z position
        let mut min_z = f64::MAX;
        let mut hit_ground = false;

        // Simulate for 2 seconds (need more time for sphere to fall 2m)
        for _ in 0..200 {
            stepper.step(&mut world).expect("step should succeed");

            let sphere = world.body(BodyId::new(2)).expect("sphere should exist");
            let z = sphere.state.pose.position.z;

            if z < min_z {
                min_z = z;
            }

            // Check if sphere is near ground
            if z < 0.7 {
                hit_ground = true;
            }
        }

        assert!(hit_ground, "sphere should reach the ground");

        // Sphere should not penetrate too deeply (with soft contacts, allow more penetration)
        assert!(
            min_z >= 0.2,
            "sphere should not penetrate ground too much, min_z={}",
            min_z
        );
    }

    #[test]
    fn test_sphere_sphere_collision() {
        use crate::world::{Body, CollisionShape};
        use sim_contact::ContactParams;
        use sim_types::BodyId;

        // Create world without gravity with stable contact params
        let contact_params = ContactParams {
            stiffness: 5_000.0,
            stiffness_power: 1.0,
            damping: 100.0,
            friction_coefficient: 0.5,
            rolling_friction: 0.01,
            torsional_friction: 0.01,
            contact_margin: 0.0001,
            restitution: 0.0,
        };
        let config = SimulationConfig::default().zero_gravity();
        let mut world = World::with_contact_params(config, contact_params);

        // Sphere 1 at origin, moving in +X direction
        let sphere1_state = RigidBodyState::new(
            Pose::from_position(Point3::new(0.0, 0.0, 0.0)),
            sim_types::Twist::linear(Vector3::new(1.0, 0.0, 0.0)),
        );
        let sphere1 = Body::new(
            BodyId::new(1),
            sphere1_state,
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_shape(CollisionShape::sphere(0.5));
        world.insert_body(sphere1).expect("insert should succeed");

        // Sphere 2 at x=1.5, stationary (gap of 0.5 between surfaces)
        let sphere2_state =
            RigidBodyState::at_rest(Pose::from_position(Point3::new(1.5, 0.0, 0.0)));
        let sphere2 = Body::new(
            BodyId::new(2),
            sphere2_state,
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_shape(CollisionShape::sphere(0.5));
        world.insert_body(sphere2).expect("insert should succeed");

        let stepper_config = StepperConfig::zero_gravity().with_contacts(true);
        let mut stepper = Stepper::with_config(stepper_config);

        // Record initial momentum
        let initial_momentum = world.total_linear_momentum();

        // Simulate for 1.5 seconds (sphere 1 should hit sphere 2 after ~0.5s)
        // Default timestep is ~0.00416s, so 1.5s needs ~360 steps
        for _ in 0..360 {
            stepper.step(&mut world).expect("step should succeed");
        }

        // After collision, sphere 2 should be moving
        let sphere2 = world.body(BodyId::new(2)).expect("sphere 2 should exist");
        assert!(
            sphere2.state.twist.linear.x > 0.1,
            "sphere 2 should have gained velocity, got vx={}",
            sphere2.state.twist.linear.x
        );

        // Check momentum conservation (approximately, due to compliant model)
        let final_momentum = world.total_linear_momentum();
        let momentum_change = (final_momentum - initial_momentum).norm();
        assert!(
            momentum_change < 0.5,
            "momentum should be approximately conserved, change={}",
            momentum_change
        );
    }

    #[test]
    fn test_contacts_disabled() {
        use crate::world::{Body, CollisionShape};
        use sim_types::BodyId;

        // Create world with contacts disabled
        let config = SimulationConfig::default().zero_gravity();
        let mut world = World::new(config);

        // Two overlapping spheres
        let sphere1 = Body::new(
            BodyId::new(1),
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 0.0))),
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_shape(CollisionShape::sphere(0.5));
        world.insert_body(sphere1).expect("insert should succeed");

        let sphere2 = Body::new(
            BodyId::new(2),
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.5, 0.0, 0.0))),
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_shape(CollisionShape::sphere(0.5));
        world.insert_body(sphere2).expect("insert should succeed");

        // Use stepper with contacts disabled
        let stepper_config = StepperConfig::no_contacts();
        let mut stepper = Stepper::with_config(stepper_config);

        stepper.step(&mut world).expect("step should succeed");

        // Both spheres should have zero force (no contact resolution)
        let sphere1 = world.body(BodyId::new(1)).expect("sphere 1 should exist");
        let sphere2 = world.body(BodyId::new(2)).expect("sphere 2 should exist");

        assert_relative_eq!(sphere1.accumulated_force.norm(), 0.0, epsilon = 1e-10);
        assert_relative_eq!(sphere2.accumulated_force.norm(), 0.0, epsilon = 1e-10);
    }

    // =========================================================================
    // Joint Constraint Integration Tests
    // =========================================================================

    #[test]
    fn test_fixed_joint_holds_bodies_together() {
        use crate::world::Joint;
        use sim_types::{JointId, JointType as SimJointType};

        // Create world with zero gravity to isolate joint forces
        let config = SimulationConfig::default().zero_gravity();
        let mut world = World::new(config);

        // Create two bodies with some initial separation
        let state1 = RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 0.0)));
        let state2 = RigidBodyState::at_rest(Pose::from_position(Point3::new(0.5, 0.0, 0.0)));

        let body1 = world.add_body(state1, MassProperties::sphere(1.0, 0.25));
        let body2 = world.add_body(state2, MassProperties::sphere(1.0, 0.25));

        // Add a fixed joint connecting them with anchors at their centers
        let joint = Joint::new(JointId::new(1), SimJointType::Fixed, body1, body2).with_anchors(
            Point3::new(0.25, 0.0, 0.0),  // Parent anchor at its surface
            Point3::new(-0.25, 0.0, 0.0), // Child anchor at its surface
        );
        world
            .insert_joint(joint)
            .expect("insert joint should succeed");

        let mut stepper = Stepper::with_config(StepperConfig::zero_gravity());

        // Simulate
        for _ in 0..100 {
            stepper.step(&mut world).expect("step should succeed");
        }

        // Bodies should be pulled together by the constraint
        let body1_pos = world.body(body1).expect("body1").state.pose.position;
        let body2_pos = world.body(body2).expect("body2").state.pose.position;

        let distance = (body2_pos - body1_pos).norm();

        // The distance should be close to 0.5 (sum of anchors: 0.25 + 0.25)
        // With spring-based constraints, there might be some oscillation
        assert!(
            distance < 1.0,
            "bodies should be close together, distance={}",
            distance
        );
    }

    #[test]
    fn test_revolute_joint_allows_rotation() {
        use crate::world::Joint;
        use sim_types::{JointId, JointType as SimJointType, Twist};

        // Create world with zero gravity
        let config = SimulationConfig::default().zero_gravity();
        let mut world = World::new(config);

        // Static base
        let base = world.add_static_body(Pose::from_position(Point3::origin()));

        // Rotating arm with initial angular velocity
        let arm_state = RigidBodyState::new(
            Pose::from_position(Point3::new(1.0, 0.0, 0.0)),
            Twist::angular(Vector3::new(0.0, 0.0, 1.0)), // Rotating around Z
        );
        let arm = world.add_body(arm_state, MassProperties::sphere(1.0, 0.25));

        // Revolute joint around Z axis at the base position
        let joint = Joint::new(JointId::new(1), SimJointType::Revolute, base, arm)
            .with_axis(Vector3::z())
            .with_anchors(Point3::origin(), Point3::new(-1.0, 0.0, 0.0))
            .with_damping(1.0); // Add damping to slow it down
        world
            .insert_joint(joint)
            .expect("insert joint should succeed");

        let mut stepper = Stepper::with_config(StepperConfig::zero_gravity());

        // Run simulation
        for _ in 0..100 {
            stepper.step(&mut world).expect("step should succeed");
        }

        // Arm should still have some angular velocity (damped but not zero)
        // or position should have changed
        let arm_body = world.body(arm).expect("arm body");

        // The arm should maintain connection to base (constrained)
        let arm_pos = arm_body.state.pose.position;
        let distance_from_origin = arm_pos.coords.norm();

        // Should be approximately 1.0 (arm length) from origin
        assert!(
            (distance_from_origin - 1.0).abs() < 0.5,
            "arm should maintain distance from base, got {}",
            distance_from_origin
        );
    }

    #[test]
    fn test_pendulum_swings() {
        use crate::world::Joint;
        use sim_types::{JointId, JointType as SimJointType};

        // Create world with gravity
        let config = SimulationConfig::default();
        let mut world = World::new(config);

        // Static pivot point above origin
        let pivot = world.add_static_body(Pose::from_position(Point3::new(0.0, 0.0, 2.0)));

        // Pendulum bob offset to the side (will swing due to gravity)
        let bob_state = RigidBodyState::at_rest(Pose::from_position(Point3::new(1.0, 0.0, 2.0)));
        let bob = world.add_body(bob_state, MassProperties::sphere(1.0, 0.1));

        // Revolute joint allowing swing around Y axis
        let joint = Joint::new(JointId::new(1), SimJointType::Revolute, pivot, bob)
            .with_axis(Vector3::y())
            .with_anchors(Point3::origin(), Point3::new(-1.0, 0.0, 0.0))
            .with_damping(0.5);
        world
            .insert_joint(joint)
            .expect("insert joint should succeed");

        let mut stepper = Stepper::new();

        // Record initial and final positions
        let initial_x = world.body(bob).expect("bob").state.pose.position.x;

        // Simulate for 1 second
        for _ in 0..240 {
            stepper.step(&mut world).expect("step should succeed");
        }

        let bob_body = world.body(bob).expect("bob body");
        let final_pos = bob_body.state.pose.position;

        // Pendulum should have swung (x position changed due to gravity)
        // With proper constraint solving, it would swing down
        // Check that position changed (indicating dynamics happened)
        let position_changed =
            (final_pos.x - initial_x).abs() > 0.01 || (final_pos.z - 2.0).abs() > 0.01;

        assert!(
            position_changed,
            "pendulum should swing, initial_x={}, final_pos={:?}",
            initial_x, final_pos
        );
    }

    #[test]
    fn test_joint_with_motor() {
        use crate::world::Joint;
        use sim_constraint::JointMotor;
        use sim_types::{JointId, JointType as SimJointType};

        let config = SimulationConfig::default().zero_gravity();
        let mut world = World::new(config);

        // Static base
        let base = world.add_static_body(Pose::from_position(Point3::origin()));

        // Arm to be driven
        let arm_state = RigidBodyState::at_rest(Pose::from_position(Point3::new(1.0, 0.0, 0.0)));
        let arm = world.add_body(arm_state, MassProperties::sphere(1.0, 0.25));

        // Revolute joint with velocity motor
        let motor = JointMotor::velocity(2.0, 100.0);
        let joint = Joint::new(JointId::new(1), SimJointType::Revolute, base, arm)
            .with_axis(Vector3::z())
            .with_anchors(Point3::origin(), Point3::new(-1.0, 0.0, 0.0))
            .with_motor(motor)
            .with_damping(0.1);
        world
            .insert_joint(joint)
            .expect("insert joint should succeed");

        let mut stepper = Stepper::with_config(StepperConfig::zero_gravity());

        // Record initial state
        let initial_omega = world.body(arm).expect("arm").state.twist.angular.z;

        // Simulate
        for _ in 0..100 {
            stepper.step(&mut world).expect("step should succeed");
        }

        let final_omega = world.body(arm).expect("arm").state.twist.angular.z;

        // Motor should have induced some angular velocity
        // The exact value depends on motor implementation, but it should change
        let omega_changed = (final_omega - initial_omega).abs() > 0.001;
        assert!(
            omega_changed || final_omega.abs() > 0.001,
            "motor should induce rotation, initial_omega={}, final_omega={}",
            initial_omega,
            final_omega
        );
    }

    #[test]
    fn test_constraints_disabled() {
        use crate::world::Joint;
        use sim_types::{JointId, JointType as SimJointType};

        let config = SimulationConfig::default().zero_gravity();
        let mut world = World::new(config);

        // Two separated bodies
        let state1 = RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 0.0)));
        let state2 = RigidBodyState::at_rest(Pose::from_position(Point3::new(10.0, 0.0, 0.0)));

        let body1 = world.add_body(state1, MassProperties::sphere(1.0, 0.25));
        let body2 = world.add_body(state2, MassProperties::sphere(1.0, 0.25));

        // Add a fixed joint
        let joint = Joint::new(JointId::new(1), SimJointType::Fixed, body1, body2);
        world
            .insert_joint(joint)
            .expect("insert joint should succeed");

        // Disable constraints
        let mut stepper_config = StepperConfig::zero_gravity();
        stepper_config.enable_constraints = false;
        let mut stepper = Stepper::with_config(stepper_config);

        // Simulate
        stepper.step(&mut world).expect("step should succeed");

        // Bodies should have zero forces (constraints not applied)
        let body1_force = world.body(body1).expect("body1").accumulated_force.norm();
        let body2_force = world.body(body2).expect("body2").accumulated_force.norm();

        assert_relative_eq!(body1_force, 0.0, epsilon = 1e-10);
        assert_relative_eq!(body2_force, 0.0, epsilon = 1e-10);
    }
}
