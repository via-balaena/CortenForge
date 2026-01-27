//! Simulation world container and entity management.
//!
//! **Deprecated**: This module is part of the deprecated World API. Use the
//! Model/Data API from [`crate::mujoco_pipeline`] instead.
//!
//! The [`World`] is the central data structure for simulation state.
//! It manages rigid bodies, their properties, and provides query interfaces.

// Allow deprecated items within this deprecated module
#![allow(deprecated)]

use hashbrown::HashMap;
use nalgebra::{Matrix3, Point3, Vector3};
use sim_constraint::{
    BodyState as ConstraintBodyState, ConstraintSolver as JointConstraintSolver,
    Joint as ConstraintJoint, JointLimits, JointMotor, JointType as ConstraintJointType,
};
use sim_contact::{ContactModel, ContactParams, ContactPoint, ContactSolver, ContactSolverConfig};
use sim_types::{
    BodyId, JointId, JointState, JointType, MassProperties, Observation, ObservationType, Pose,
    RigidBodyState, SimError, SimulationConfig,
};

use crate::broad_phase::{BroadPhaseConfig, BroadPhaseDetector};
use crate::gjk_epa::gjk_epa_contact;

// Re-export CollisionShape from the canonical source
pub use crate::collision_shape::CollisionShape;

/// A rigid body in the simulation world.
///
/// **Deprecated**: Use Model/Data API from [`crate::mujoco_pipeline`] instead.
#[deprecated(
    since = "0.8.0",
    note = "Use Model/Data API from mujoco_pipeline module instead"
)]
#[derive(Debug, Clone)]
pub struct Body {
    /// Unique identifier.
    pub id: BodyId,
    /// Optional name for debugging.
    pub name: Option<String>,
    /// Current state (pose + twist).
    pub state: RigidBodyState,
    /// Mass properties (mass, inertia, COM offset).
    pub mass_props: MassProperties,
    /// Collision shape for contact detection.
    pub collision_shape: Option<CollisionShape>,
    /// Local pose of the collision shape relative to the body frame.
    ///
    /// This allows geoms to be offset from the body origin (e.g., MJCF `fromto` or `pos`).
    /// If `None`, the shape is centered at the body origin with identity rotation.
    pub collision_shape_pose: Option<Pose>,
    /// Whether this body is static (immovable).
    pub is_static: bool,
    /// Whether this body is currently sleeping (inactive).
    pub is_sleeping: bool,
    /// Time the body has been below the sleep velocity threshold (seconds).
    ///
    /// When this exceeds the configured sleep time threshold, the body
    /// will be put to sleep. Reset to zero when velocity exceeds threshold.
    pub sleep_time: f64,
    /// Accumulated external force (cleared each step).
    pub accumulated_force: Vector3<f64>,
    /// Accumulated external torque (cleared each step).
    pub accumulated_torque: Vector3<f64>,
    /// Collision type bitmask (MuJoCo-compatible).
    ///
    /// Bodies can only collide if `(A.contype & B.conaffinity) != 0`
    /// AND `(B.contype & A.conaffinity) != 0`.
    ///
    /// Default is 1 (bit 0 set), meaning all bodies collide with each other
    /// by default.
    pub contype: u32,
    /// Collision affinity bitmask (MuJoCo-compatible).
    ///
    /// Determines which collision types this body responds to.
    /// Default is 1 (bit 0 set).
    pub conaffinity: u32,
}

impl Body {
    /// Create a new dynamic body.
    #[must_use]
    pub fn new(id: BodyId, state: RigidBodyState, mass_props: MassProperties) -> Self {
        Self {
            id,
            name: None,
            state,
            mass_props,
            collision_shape: None,
            collision_shape_pose: None,
            is_static: false,
            is_sleeping: false,
            sleep_time: 0.0,
            accumulated_force: Vector3::zeros(),
            accumulated_torque: Vector3::zeros(),
            contype: 1,
            conaffinity: 1,
        }
    }

    /// Create a static (immovable) body.
    #[must_use]
    pub fn new_static(id: BodyId, pose: Pose) -> Self {
        Self {
            id,
            name: None,
            state: RigidBodyState::at_rest(pose),
            mass_props: MassProperties::point_mass(f64::INFINITY),
            collision_shape: None,
            collision_shape_pose: None,
            is_static: true,
            is_sleeping: false,
            sleep_time: 0.0,
            accumulated_force: Vector3::zeros(),
            accumulated_torque: Vector3::zeros(),
            contype: 1,
            conaffinity: 1,
        }
    }

    /// Set the body name.
    #[must_use]
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// Set the collision shape.
    #[must_use]
    pub fn with_collision_shape(mut self, shape: CollisionShape) -> Self {
        self.collision_shape = Some(shape);
        self
    }

    /// Set the local pose of the collision shape relative to the body frame.
    ///
    /// Use this when the geom is offset from the body origin (e.g., MJCF `fromto` or `pos`).
    #[must_use]
    pub fn with_collision_shape_pose(mut self, pose: Pose) -> Self {
        self.collision_shape_pose = Some(pose);
        self
    }

    /// Set the collision filtering bitmasks.
    ///
    /// Bodies A and B can collide only if:
    /// `(A.contype & B.conaffinity) != 0 && (B.contype & A.conaffinity) != 0`
    ///
    /// Common patterns:
    /// - `contype=1, conaffinity=1` (default): Collides with everything
    /// - `contype=0, conaffinity=1`: This body doesn't generate contacts, but others can collide with it
    /// - `contype=1, conaffinity=0`: This body generates contacts, but nothing collides with it
    /// - Use different bits to create collision groups (e.g., bit 1 for player, bit 2 for enemies)
    #[must_use]
    pub fn with_collision_filter(mut self, contype: u32, conaffinity: u32) -> Self {
        self.contype = contype;
        self.conaffinity = conaffinity;
        self
    }

    /// Check if this body can collide with another body based on collision filtering.
    ///
    /// Returns true if the collision bitmasks allow contact generation.
    #[must_use]
    pub fn can_collide_with(&self, other: &Self) -> bool {
        (self.contype & other.conaffinity) != 0 && (other.contype & self.conaffinity) != 0
    }

    /// Apply a force at the center of mass.
    ///
    /// If the body is sleeping, it will be woken up (unless the force is negligible).
    pub fn apply_force(&mut self, force: Vector3<f64>) {
        if !self.is_static {
            self.accumulated_force += force;
            // Wake up sleeping bodies when significant force is applied
            if self.is_sleeping && force.norm() > 1e-10 {
                self.wake_up();
            }
        }
    }

    /// Apply a torque.
    ///
    /// If the body is sleeping, it will be woken up (unless the torque is negligible).
    pub fn apply_torque(&mut self, torque: Vector3<f64>) {
        if !self.is_static {
            self.accumulated_torque += torque;
            // Wake up sleeping bodies when significant torque is applied
            if self.is_sleeping && torque.norm() > 1e-10 {
                self.wake_up();
            }
        }
    }

    /// Apply a force at a world-space point.
    ///
    /// If the body is sleeping, it will be woken up (unless the force is negligible).
    pub fn apply_force_at_point(&mut self, force: Vector3<f64>, point: Point3<f64>) {
        if !self.is_static {
            self.accumulated_force += force;
            // Compute torque: r × F where r is from COM to point
            let com_world = self
                .state
                .pose
                .transform_point(&Point3::from(self.mass_props.center_of_mass));
            let r = point - com_world;
            self.accumulated_torque += r.cross(&force);
            // Wake up sleeping bodies when significant force is applied
            if self.is_sleeping && force.norm() > 1e-10 {
                self.wake_up();
            }
        }
    }

    /// Clear accumulated forces and torques.
    pub fn clear_forces(&mut self) {
        self.accumulated_force = Vector3::zeros();
        self.accumulated_torque = Vector3::zeros();
    }

    /// Check if the body should be awake based on velocity.
    #[must_use]
    pub fn should_sleep(&self, threshold: f64) -> bool {
        if self.is_static {
            return false;
        }
        self.state.twist.speed() < threshold && self.state.twist.angular_speed() < threshold
    }

    /// Wake up the body.
    ///
    /// Resets the sleep timer and marks the body as active.
    pub fn wake_up(&mut self) {
        self.is_sleeping = false;
        self.sleep_time = 0.0;
    }

    /// Put the body to sleep.
    ///
    /// Sleeping bodies are skipped during integration and force application,
    /// providing a significant performance optimization for stationary objects.
    /// The body's velocities are set to zero when sleeping.
    pub fn put_to_sleep(&mut self) {
        if !self.is_static {
            self.is_sleeping = true;
            // Zero out velocities to ensure the body is truly at rest
            self.state.twist = sim_types::Twist::zero();
        }
    }

    /// Compute linear acceleration from accumulated force.
    #[must_use]
    pub fn linear_acceleration(&self) -> Vector3<f64> {
        if self.is_static || self.mass_props.mass <= 0.0 {
            Vector3::zeros()
        } else {
            self.accumulated_force / self.mass_props.mass
        }
    }

    /// Compute angular acceleration from accumulated torque.
    #[must_use]
    pub fn angular_acceleration(&self) -> Option<Vector3<f64>> {
        if self.is_static {
            return Some(Vector3::zeros());
        }
        self.mass_props
            .inverse_inertia()
            .map(|inv_i| inv_i * self.accumulated_torque)
    }
}

/// A joint connecting two bodies.
///
/// **Deprecated**: Use Model/Data API from [`crate::mujoco_pipeline`] instead.
#[deprecated(
    since = "0.8.0",
    note = "Use Model/Data API from mujoco_pipeline module instead"
)]
#[derive(Debug, Clone)]
pub struct Joint {
    /// Unique identifier.
    pub id: JointId,
    /// Optional name for debugging.
    pub name: Option<String>,
    /// Type of joint constraint.
    pub joint_type: JointType,
    /// Parent body (anchor).
    pub parent: BodyId,
    /// Child body.
    pub child: BodyId,
    /// Current joint state.
    pub state: JointState,
    /// Anchor point on parent body (local coordinates).
    pub parent_anchor: Point3<f64>,
    /// Anchor point on child body (local coordinates).
    pub child_anchor: Point3<f64>,
    /// Joint axis (for revolute/prismatic joints, local to parent).
    pub axis: Vector3<f64>,
    /// Joint limits (optional).
    pub limits: Option<JointLimits>,
    /// Joint motor (optional).
    pub motor: Option<JointMotor>,
    /// Joint damping coefficient.
    pub damping: f64,
}

impl Joint {
    /// Create a new joint.
    #[must_use]
    pub fn new(id: JointId, joint_type: JointType, parent: BodyId, child: BodyId) -> Self {
        Self {
            id,
            name: None,
            joint_type,
            parent,
            child,
            state: JointState::zero(),
            parent_anchor: Point3::origin(),
            child_anchor: Point3::origin(),
            axis: Vector3::z(), // Default axis
            limits: None,
            motor: None,
            damping: 0.0,
        }
    }

    /// Set the joint name.
    #[must_use]
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// Set initial joint state.
    #[must_use]
    pub fn with_state(mut self, state: JointState) -> Self {
        self.state = state;
        self
    }

    /// Set anchor points on parent and child bodies.
    #[must_use]
    pub fn with_anchors(mut self, parent_anchor: Point3<f64>, child_anchor: Point3<f64>) -> Self {
        self.parent_anchor = parent_anchor;
        self.child_anchor = child_anchor;
        self
    }

    /// Set the joint axis (for revolute/prismatic joints).
    ///
    /// If the axis is zero-length, defaults to Z-axis.
    #[must_use]
    pub fn with_axis(mut self, axis: Vector3<f64>) -> Self {
        let norm = axis.norm();
        self.axis = if norm > 1e-10 {
            axis / norm
        } else {
            Vector3::z() // Default to Z if zero-length
        };
        self
    }

    /// Set joint limits.
    #[must_use]
    pub fn with_limits(mut self, limits: JointLimits) -> Self {
        self.limits = Some(limits);
        self
    }

    /// Set joint motor.
    #[must_use]
    pub fn with_motor(mut self, motor: JointMotor) -> Self {
        self.motor = Some(motor);
        self
    }

    /// Set joint damping.
    #[must_use]
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.damping = damping;
        self
    }
}

/// Adapter to make a `world::Joint` implement the constraint solver's Joint trait.
struct JointAdapter<'a>(&'a Joint);

impl ConstraintJoint for JointAdapter<'_> {
    fn parent(&self) -> BodyId {
        self.0.parent
    }

    fn child(&self) -> BodyId {
        self.0.child
    }

    fn dof(&self) -> usize {
        match self.0.joint_type {
            JointType::Fixed => 0,
            JointType::Revolute | JointType::Prismatic => 1,
            JointType::Spherical | JointType::Planar => 3, // Planar: 2 translations + 1 rotation
            JointType::Cylindrical => 2,
            JointType::Free => 6, // 3 translations + 3 rotations
        }
    }

    fn joint_type(&self) -> ConstraintJointType {
        match self.0.joint_type {
            JointType::Fixed => ConstraintJointType::Fixed,
            JointType::Revolute => ConstraintJointType::Revolute,
            JointType::Prismatic => ConstraintJointType::Prismatic,
            JointType::Spherical => ConstraintJointType::Spherical,
            JointType::Planar => ConstraintJointType::Planar,
            JointType::Free => ConstraintJointType::Free,
            JointType::Cylindrical => ConstraintJointType::Cylindrical,
        }
    }

    fn limits(&self) -> Option<&JointLimits> {
        self.0.limits.as_ref()
    }

    fn motor(&self) -> Option<&JointMotor> {
        self.0.motor.as_ref()
    }

    fn damping(&self) -> f64 {
        self.0.damping
    }

    fn parent_anchor(&self) -> Point3<f64> {
        self.0.parent_anchor
    }

    fn child_anchor(&self) -> Point3<f64> {
        self.0.child_anchor
    }

    fn axis(&self) -> Vector3<f64> {
        self.0.axis
    }
}

/// The simulation world containing all entities.
///
/// **Deprecated**: Use Model/Data API from [`crate::mujoco_pipeline`] instead.
#[deprecated(
    since = "0.8.0",
    note = "Use Model/Data API from mujoco_pipeline module instead"
)]
#[derive(Debug, Clone)]
pub struct World {
    /// Simulation configuration.
    config: SimulationConfig,
    /// Current simulation time.
    time: f64,
    /// Step counter.
    step_count: u64,
    /// All rigid bodies, indexed by ID.
    bodies: HashMap<BodyId, Body>,
    /// All joints, indexed by ID.
    joints: HashMap<JointId, Joint>,
    /// Next available body ID.
    next_body_id: u64,
    /// Next available joint ID.
    next_joint_id: u64,
    /// Body name to ID mapping.
    body_names: HashMap<String, BodyId>,
    /// Joint name to ID mapping.
    joint_names: HashMap<String, JointId>,
    /// Contact solver for computing contact forces.
    contact_solver: ContactSolver,
    /// Contact parameters (can be changed for domain randomization).
    contact_params: ContactParams,
    /// Joint constraint solver.
    constraint_solver: JointConstraintSolver,
    /// Broad-phase collision detector.
    broad_phase: BroadPhaseDetector,
}

impl Default for World {
    fn default() -> Self {
        Self::new(SimulationConfig::default())
    }
}

impl World {
    /// Create a new empty world with the given configuration.
    #[must_use]
    pub fn new(config: SimulationConfig) -> Self {
        let contact_params = ContactParams::default();
        let contact_model = ContactModel::new(contact_params);
        let contact_solver = ContactSolver::new(contact_model, ContactSolverConfig::default());
        let constraint_solver = JointConstraintSolver::default_solver();

        Self {
            config,
            time: 0.0,
            step_count: 0,
            bodies: HashMap::new(),
            joints: HashMap::new(),
            next_body_id: 1,
            next_joint_id: 1,
            body_names: HashMap::new(),
            joint_names: HashMap::new(),
            contact_solver,
            contact_params,
            constraint_solver,
            broad_phase: BroadPhaseDetector::default(),
        }
    }

    /// Create a world with custom contact parameters.
    #[must_use]
    pub fn with_contact_params(config: SimulationConfig, contact_params: ContactParams) -> Self {
        let contact_model = ContactModel::new(contact_params);
        let contact_solver = ContactSolver::new(contact_model, ContactSolverConfig::default());
        let constraint_solver = JointConstraintSolver::default_solver();

        Self {
            config,
            time: 0.0,
            step_count: 0,
            bodies: HashMap::new(),
            joints: HashMap::new(),
            next_body_id: 1,
            next_joint_id: 1,
            body_names: HashMap::new(),
            joint_names: HashMap::new(),
            contact_solver,
            contact_params,
            constraint_solver,
            broad_phase: BroadPhaseDetector::default(),
        }
    }

    /// Get the current contact parameters.
    #[must_use]
    pub fn contact_params(&self) -> &ContactParams {
        &self.contact_params
    }

    /// Update the contact parameters (for domain randomization).
    pub fn set_contact_params(&mut self, params: ContactParams) {
        self.contact_params = params;
        let contact_model = ContactModel::new(params);
        // Preserve the existing solver config
        let solver_config = *self.contact_solver.config();
        self.contact_solver = ContactSolver::new(contact_model, solver_config);
    }

    /// Get the current contact solver configuration.
    #[must_use]
    pub fn contact_solver_config(&self) -> ContactSolverConfig {
        *self.contact_solver.config()
    }

    /// Set the contact solver configuration.
    ///
    /// Use this to tune solver performance for different use cases:
    /// - `ContactSolverConfig::realtime()` for fast, stable simulation (games)
    /// - `ContactSolverConfig::robotics()` for high accuracy (control, RL)
    /// - `ContactSolverConfig::high_fidelity()` for maximum precision
    ///
    /// # Example
    ///
    /// ```ignore
    /// use sim_core::World;
    /// use sim_contact::ContactSolverConfig;
    ///
    /// let mut world = World::default();
    /// world.set_contact_solver_config(ContactSolverConfig::realtime());
    /// ```
    pub fn set_contact_solver_config(&mut self, config: ContactSolverConfig) {
        let contact_model = ContactModel::new(self.contact_params);
        self.contact_solver = ContactSolver::new(contact_model, config);
    }

    /// Get the simulation configuration.
    #[must_use]
    pub fn config(&self) -> &SimulationConfig {
        &self.config
    }

    /// Get the current simulation time.
    #[must_use]
    pub fn time(&self) -> f64 {
        self.time
    }

    /// Get the step count.
    #[must_use]
    pub fn step_count(&self) -> u64 {
        self.step_count
    }

    /// Get the timestep from configuration.
    #[must_use]
    pub fn timestep(&self) -> f64 {
        self.config.timestep
    }

    /// Get the number of bodies.
    #[must_use]
    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }

    /// Get the number of joints.
    #[must_use]
    pub fn joint_count(&self) -> usize {
        self.joints.len()
    }

    // =========================================================================
    // Body Management
    // =========================================================================

    /// Add a body to the world and return its ID.
    pub fn add_body(&mut self, state: RigidBodyState, mass_props: MassProperties) -> BodyId {
        let id = BodyId::new(self.next_body_id);
        self.next_body_id += 1;

        let body = Body::new(id, state, mass_props);
        self.bodies.insert(id, body);
        id
    }

    /// Add a static body at the given pose.
    pub fn add_static_body(&mut self, pose: Pose) -> BodyId {
        let id = BodyId::new(self.next_body_id);
        self.next_body_id += 1;

        let body = Body::new_static(id, pose);
        self.bodies.insert(id, body);
        id
    }

    /// Add a pre-built body to the world.
    ///
    /// # Errors
    ///
    /// Returns an error if the body ID already exists.
    pub fn insert_body(&mut self, body: Body) -> sim_types::Result<()> {
        if self.bodies.contains_key(&body.id) {
            return Err(SimError::invalid_config(format!(
                "body ID {} already exists",
                body.id
            )));
        }

        if let Some(ref name) = body.name {
            self.body_names.insert(name.clone(), body.id);
        }

        // Update next_body_id if needed
        if body.id.raw() >= self.next_body_id {
            self.next_body_id = body.id.raw() + 1;
        }

        self.bodies.insert(body.id, body);
        Ok(())
    }

    /// Get a body by ID.
    #[must_use]
    pub fn body(&self, id: BodyId) -> Option<&Body> {
        self.bodies.get(&id)
    }

    /// Get a mutable reference to a body by ID.
    #[must_use]
    pub fn body_mut(&mut self, id: BodyId) -> Option<&mut Body> {
        self.bodies.get_mut(&id)
    }

    /// Get a body by name.
    #[must_use]
    pub fn body_by_name(&self, name: &str) -> Option<&Body> {
        self.body_names.get(name).and_then(|id| self.bodies.get(id))
    }

    /// Get a mutable reference to a body by name.
    #[must_use]
    pub fn body_by_name_mut(&mut self, name: &str) -> Option<&mut Body> {
        self.body_names
            .get(name)
            .copied()
            .and_then(move |id| self.bodies.get_mut(&id))
    }

    /// Remove a body from the world.
    pub fn remove_body(&mut self, id: BodyId) -> Option<Body> {
        let body = self.bodies.remove(&id)?;
        if let Some(ref name) = body.name {
            self.body_names.remove(name);
        }
        Some(body)
    }

    /// Iterate over all bodies.
    pub fn bodies(&self) -> impl Iterator<Item = &Body> {
        self.bodies.values()
    }

    /// Iterate over all bodies mutably.
    pub fn bodies_mut(&mut self) -> impl Iterator<Item = &mut Body> {
        self.bodies.values_mut()
    }

    /// Iterate over all body IDs.
    pub fn body_ids(&self) -> impl Iterator<Item = BodyId> + '_ {
        self.bodies.keys().copied()
    }

    /// Integrate all bodies in parallel using rayon.
    ///
    /// This method performs position/velocity integration, damping, velocity clamping,
    /// and sleep state updates in parallel across all bodies. Each body's integration
    /// is independent, making this embarrassingly parallel.
    ///
    /// # Arguments
    ///
    /// * `method` - Integration method to use
    /// * `dt` - Timestep
    /// * `linear_damping` - Linear velocity damping factor
    /// * `angular_damping` - Angular velocity damping factor
    /// * `max_linear_velocity` - Maximum linear velocity (None = unlimited)
    /// * `max_angular_velocity` - Maximum angular velocity (None = unlimited)
    /// * `sleep_config` - Sleep configuration: (threshold, `time_threshold`, `allow_sleeping`)
    #[cfg(feature = "parallel")]
    #[allow(clippy::too_many_arguments)]
    pub fn integrate_bodies_parallel(
        &mut self,
        method: sim_types::IntegrationMethod,
        dt: f64,
        linear_damping: f64,
        angular_damping: f64,
        max_linear_velocity: Option<f64>,
        max_angular_velocity: Option<f64>,
        sleep_config: (f64, f64, bool),
    ) {
        let (sleep_threshold, sleep_time_threshold, allow_sleeping) = sleep_config;

        // Process bodies sequentially since hashbrown's HashMap doesn't support par_iter_mut.
        // The main parallelization benefit comes from constraint solving (island-parallel).
        // For true parallel body integration, the data layout would need restructuring
        // (e.g., use a Vec<Body> with an index HashMap<BodyId, usize>).
        for body in self.bodies.values_mut() {
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
            crate::integrators::integrate_with_method(
                method,
                &mut body.state,
                linear_accel,
                angular_accel,
                dt,
            );

            // Apply damping
            if linear_damping > 0.0 || angular_damping > 0.0 {
                body.state.twist = crate::integrators::apply_damping(
                    &body.state.twist,
                    linear_damping,
                    angular_damping,
                    dt,
                );
            }

            // Apply velocity limits
            if let (Some(max_linear), Some(max_angular)) =
                (max_linear_velocity, max_angular_velocity)
            {
                body.state.twist = crate::integrators::clamp_velocities(
                    &body.state.twist,
                    max_linear,
                    max_angular,
                );
            }

            // Update sleep state
            if allow_sleeping {
                if body.should_sleep(sleep_threshold) {
                    body.sleep_time += dt;
                    if body.sleep_time >= sleep_time_threshold {
                        body.put_to_sleep();
                    }
                } else {
                    body.sleep_time = 0.0;
                }
            }
        }
    }

    // =========================================================================
    // Joint Management
    // =========================================================================

    /// Add a joint to the world.
    ///
    /// # Errors
    ///
    /// Returns an error if either the parent or child body does not exist.
    pub fn add_joint(
        &mut self,
        joint_type: JointType,
        parent: BodyId,
        child: BodyId,
    ) -> sim_types::Result<JointId> {
        // Validate that both bodies exist
        if !self.bodies.contains_key(&parent) {
            return Err(SimError::InvalidBodyId(parent.raw()));
        }
        if !self.bodies.contains_key(&child) {
            return Err(SimError::InvalidBodyId(child.raw()));
        }

        let id = JointId::new(self.next_joint_id);
        self.next_joint_id += 1;

        let joint = Joint::new(id, joint_type, parent, child);
        self.joints.insert(id, joint);
        Ok(id)
    }

    /// Insert a pre-built joint.
    ///
    /// # Errors
    ///
    /// Returns an error if the joint ID already exists.
    pub fn insert_joint(&mut self, joint: Joint) -> sim_types::Result<()> {
        if self.joints.contains_key(&joint.id) {
            return Err(SimError::invalid_config(format!(
                "joint ID {} already exists",
                joint.id
            )));
        }

        if let Some(ref name) = joint.name {
            self.joint_names.insert(name.clone(), joint.id);
        }

        if joint.id.raw() >= self.next_joint_id {
            self.next_joint_id = joint.id.raw() + 1;
        }

        self.joints.insert(joint.id, joint);
        Ok(())
    }

    /// Get a joint by ID.
    #[must_use]
    pub fn joint(&self, id: JointId) -> Option<&Joint> {
        self.joints.get(&id)
    }

    /// Get a mutable reference to a joint by ID.
    #[must_use]
    pub fn joint_mut(&mut self, id: JointId) -> Option<&mut Joint> {
        self.joints.get_mut(&id)
    }

    /// Iterate over all joints.
    pub fn joints(&self) -> impl Iterator<Item = &Joint> {
        self.joints.values()
    }

    /// Iterate over all joints mutably.
    pub fn joints_mut(&mut self) -> impl Iterator<Item = &mut Joint> {
        self.joints.values_mut()
    }

    // =========================================================================
    // Force Application
    // =========================================================================

    /// Apply gravity to all dynamic bodies.
    pub fn apply_gravity(&mut self) {
        let gravity_accel = self.config.gravity.acceleration;
        for body in self.bodies.values_mut() {
            if !body.is_static && !body.is_sleeping {
                let gravity_force = gravity_accel * body.mass_props.mass;
                body.apply_force(gravity_force);
            }
        }
    }

    /// Clear all accumulated forces on all bodies.
    pub fn clear_forces(&mut self) {
        for body in self.bodies.values_mut() {
            body.clear_forces();
        }
    }

    // =========================================================================
    // Observation
    // =========================================================================

    /// Create an observation of the current world state.
    #[must_use]
    pub fn observe(&self) -> Observation {
        let mut obs = Observation::new(self.time);

        // Body states
        let body_states: Vec<_> = self.bodies.values().map(|b| (b.id, b.state)).collect();
        if !body_states.is_empty() {
            obs.add(ObservationType::BodyStates(body_states));
        }

        // Joint states
        let joint_states: Vec<_> = self.joints.values().map(|j| (j.id, j.state)).collect();
        if !joint_states.is_empty() {
            obs.add(ObservationType::JointStates(joint_states));
        }

        obs
    }

    // =========================================================================
    // Simulation Control
    // =========================================================================

    /// Advance the simulation time (called by stepper).
    pub(crate) fn advance_time(&mut self, dt: f64) {
        self.time += dt;
        self.step_count += 1;
    }

    /// Reset simulation time to zero.
    pub fn reset_time(&mut self) {
        self.time = 0.0;
        self.step_count = 0;
    }

    /// Check if simulation has reached max time (if configured).
    #[must_use]
    pub fn is_complete(&self) -> bool {
        self.config.max_time.is_some_and(|max| self.time >= max)
    }

    /// Validate the world state.
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - The configuration is invalid
    /// - Any body has non-finite state values (`NaN` or `Inf`)
    /// - Any body has invalid mass properties
    pub fn validate(&self) -> sim_types::Result<()> {
        self.config.validate()?;

        for body in self.bodies.values() {
            if !body.state.is_finite() {
                return Err(SimError::diverged(format!(
                    "body {} has non-finite state",
                    body.id
                )));
            }
            body.mass_props.validate()?;
        }

        Ok(())
    }

    // =========================================================================
    // Contact Detection and Resolution
    // =========================================================================

    /// Detect contacts between all bodies with collision shapes.
    ///
    /// Uses broad-phase collision detection (sweep-and-prune) to efficiently
    /// find potentially colliding pairs, then performs narrow-phase detection
    /// on those pairs.
    ///
    /// Returns a list of contact points that can be passed to the contact solver.
    ///
    /// # Performance
    ///
    /// - **Broad-phase**: O(n log n) sweep-and-prune for scenes with >32 bodies,
    ///   O(n²) brute-force for smaller scenes.
    /// - **Narrow-phase**: O(k) where k is the number of potentially colliding pairs.
    pub fn detect_contacts(&mut self) -> Vec<ContactPoint> {
        if !self.config.enable_contacts {
            return Vec::new();
        }

        // Collect bodies into a Vec for broad-phase processing
        let body_list: Vec<_> = self.bodies.values().cloned().collect();

        // Use broad-phase to find potentially colliding pairs
        let potential_pairs = self.broad_phase.find_potential_pairs(&body_list);

        // Narrow-phase: check each potential pair
        let mut contacts = Vec::new();
        for (id_a, id_b) in potential_pairs {
            if let (Some(body_a), Some(body_b)) = (self.bodies.get(&id_a), self.bodies.get(&id_b)) {
                if let Some(contact) = self.detect_pair_contact(body_a, body_b) {
                    contacts.push(contact);
                }
            }
        }

        contacts
    }

    /// Get the broad-phase configuration.
    #[must_use]
    pub fn broad_phase_config(&self) -> &BroadPhaseConfig {
        self.broad_phase.config()
    }

    /// Update the broad-phase configuration.
    pub fn set_broad_phase_config(&mut self, config: BroadPhaseConfig) {
        self.broad_phase.set_config(config);
    }

    /// Detect contact between a pair of bodies.
    #[allow(clippy::unused_self)] // Will use self.config for contact margin in future
    #[allow(clippy::too_many_lines)] // Complex collision detection logic
    fn detect_pair_contact(&self, body_a: &Body, body_b: &Body) -> Option<ContactPoint> {
        // Check collision filtering before shape tests
        if !body_a.can_collide_with(body_b) {
            return None;
        }

        let shape_a = body_a.collision_shape.as_ref()?;
        let shape_b = body_b.collision_shape.as_ref()?;

        match (shape_a, shape_b) {
            // =====================================================================
            // Sphere collisions
            // =====================================================================

            // Sphere-Sphere
            (CollisionShape::Sphere { radius: r_a }, CollisionShape::Sphere { radius: r_b }) => {
                ContactPoint::sphere_sphere(
                    body_a.state.pose.position,
                    *r_a,
                    body_b.state.pose.position,
                    *r_b,
                    body_a.id,
                    body_b.id,
                )
            }

            // Sphere-Plane
            (CollisionShape::Sphere { radius }, CollisionShape::Plane { normal, distance }) => {
                ContactPoint::sphere_plane(
                    body_a.state.pose.position,
                    *radius,
                    *normal,
                    *distance,
                    body_a.id,
                    body_b.id,
                )
            }

            // Plane-Sphere (flip)
            (CollisionShape::Plane { normal, distance }, CollisionShape::Sphere { radius }) => {
                ContactPoint::sphere_plane(
                    body_b.state.pose.position,
                    *radius,
                    *normal,
                    *distance,
                    body_b.id,
                    body_a.id,
                )
                .map(|c| c.flip())
            }

            // =====================================================================
            // Box collisions
            // =====================================================================

            // Box-Box
            (
                CollisionShape::Box { half_extents: he_a },
                CollisionShape::Box { half_extents: he_b },
            ) => ContactPoint::box_box(
                body_a.state.pose.position,
                *he_a,
                body_b.state.pose.position,
                *he_b,
                body_a.id,
                body_b.id,
            ),

            // Box-Sphere
            (CollisionShape::Box { half_extents }, CollisionShape::Sphere { radius }) => {
                ContactPoint::box_sphere(
                    body_a.state.pose.position,
                    *half_extents,
                    body_b.state.pose.position,
                    *radius,
                    body_a.id,
                    body_b.id,
                )
            }

            // Sphere-Box (flip)
            (CollisionShape::Sphere { radius }, CollisionShape::Box { half_extents }) => {
                ContactPoint::box_sphere(
                    body_b.state.pose.position,
                    *half_extents,
                    body_a.state.pose.position,
                    *radius,
                    body_b.id,
                    body_a.id,
                )
                .map(|c| c.flip())
            }

            // Box-Plane
            (CollisionShape::Box { half_extents }, CollisionShape::Plane { normal, distance }) => {
                ContactPoint::box_plane(
                    body_a.state.pose.position,
                    *half_extents,
                    *normal,
                    *distance,
                    body_a.id,
                    body_b.id,
                )
            }

            // Plane-Box (flip)
            (CollisionShape::Plane { normal, distance }, CollisionShape::Box { half_extents }) => {
                ContactPoint::box_plane(
                    body_b.state.pose.position,
                    *half_extents,
                    *normal,
                    *distance,
                    body_b.id,
                    body_a.id,
                )
                .map(|c| c.flip())
            }

            // =====================================================================
            // Capsule collisions
            // =====================================================================

            // Capsule-Plane
            (
                CollisionShape::Capsule {
                    half_length: _,
                    radius,
                },
                CollisionShape::Plane {
                    normal,
                    distance: plane_dist,
                },
            ) => {
                let (start, end) = shape_a.capsule_endpoints(&body_a.state.pose)?;
                ContactPoint::capsule_plane(
                    start,
                    end,
                    *radius,
                    *normal,
                    *plane_dist,
                    body_a.id,
                    body_b.id,
                )
            }

            // Plane-Capsule (flip)
            (
                CollisionShape::Plane {
                    normal,
                    distance: plane_dist,
                },
                CollisionShape::Capsule {
                    half_length: _,
                    radius,
                },
            ) => {
                let (start, end) = shape_b.capsule_endpoints(&body_b.state.pose)?;
                ContactPoint::capsule_plane(
                    start,
                    end,
                    *radius,
                    *normal,
                    *plane_dist,
                    body_b.id,
                    body_a.id,
                )
                .map(|c| c.flip())
            }

            // Capsule-Sphere
            (
                CollisionShape::Capsule {
                    half_length: _,
                    radius: cap_radius,
                },
                CollisionShape::Sphere {
                    radius: sphere_radius,
                },
            ) => {
                let (start, end) = shape_a.capsule_endpoints(&body_a.state.pose)?;
                ContactPoint::capsule_sphere(
                    start,
                    end,
                    *cap_radius,
                    body_b.state.pose.position,
                    *sphere_radius,
                    body_a.id,
                    body_b.id,
                )
            }

            // Sphere-Capsule (flip)
            (
                CollisionShape::Sphere {
                    radius: sphere_radius,
                },
                CollisionShape::Capsule {
                    half_length: _,
                    radius: cap_radius,
                },
            ) => {
                let (start, end) = shape_b.capsule_endpoints(&body_b.state.pose)?;
                ContactPoint::capsule_sphere(
                    start,
                    end,
                    *cap_radius,
                    body_a.state.pose.position,
                    *sphere_radius,
                    body_b.id,
                    body_a.id,
                )
                .map(|c| c.flip())
            }

            // Capsule-Capsule
            (
                CollisionShape::Capsule {
                    half_length: _,
                    radius: r_a,
                },
                CollisionShape::Capsule {
                    half_length: _,
                    radius: r_b,
                },
            ) => {
                let (start_a, end_a) = shape_a.capsule_endpoints(&body_a.state.pose)?;
                let (start_b, end_b) = shape_b.capsule_endpoints(&body_b.state.pose)?;
                ContactPoint::capsule_capsule(
                    start_a, end_a, *r_a, start_b, end_b, *r_b, body_a.id, body_b.id,
                )
            }

            // =====================================================================
            // Height field collisions
            // =====================================================================

            // HeightField-Sphere
            (CollisionShape::HeightField { data }, CollisionShape::Sphere { radius }) => self
                .detect_heightfield_sphere_contact(
                    data,
                    &body_a.state.pose,
                    body_b.state.pose.position,
                    *radius,
                    body_a.id,
                    body_b.id,
                ),

            // Sphere-HeightField (flip)
            (CollisionShape::Sphere { radius }, CollisionShape::HeightField { data }) => self
                .detect_heightfield_sphere_contact(
                    data,
                    &body_b.state.pose,
                    body_a.state.pose.position,
                    *radius,
                    body_b.id,
                    body_a.id,
                )
                .map(|c| c.flip()),

            // HeightField-Capsule
            (
                CollisionShape::HeightField { data },
                CollisionShape::Capsule {
                    half_length: _,
                    radius,
                },
            ) => {
                let (start, end) = shape_b.capsule_endpoints(&body_b.state.pose)?;
                self.detect_heightfield_capsule_contact(
                    data,
                    &body_a.state.pose,
                    start,
                    end,
                    *radius,
                    body_a.id,
                    body_b.id,
                )
            }

            // Capsule-HeightField (flip)
            (
                CollisionShape::Capsule {
                    half_length: _,
                    radius,
                },
                CollisionShape::HeightField { data },
            ) => {
                let (start, end) = shape_a.capsule_endpoints(&body_a.state.pose)?;
                self.detect_heightfield_capsule_contact(
                    data,
                    &body_b.state.pose,
                    start,
                    end,
                    *radius,
                    body_b.id,
                    body_a.id,
                )
                .map(|c| c.flip())
            }

            // HeightField-Box
            (CollisionShape::HeightField { data }, CollisionShape::Box { half_extents }) => self
                .detect_heightfield_box_contact(
                    data,
                    &body_a.state.pose,
                    &body_b.state.pose,
                    half_extents,
                    body_a.id,
                    body_b.id,
                ),

            // Box-HeightField (flip)
            (CollisionShape::Box { half_extents }, CollisionShape::HeightField { data }) => self
                .detect_heightfield_box_contact(
                    data,
                    &body_b.state.pose,
                    &body_a.state.pose,
                    half_extents,
                    body_b.id,
                    body_a.id,
                )
                .map(|c| c.flip()),

            // =====================================================================
            // SDF collisions
            // =====================================================================

            // Sdf-Sphere
            (CollisionShape::Sdf { data }, CollisionShape::Sphere { radius }) => self
                .detect_sdf_sphere_contact(
                    data,
                    &body_a.state.pose,
                    body_b.state.pose.position,
                    *radius,
                    body_a.id,
                    body_b.id,
                ),

            // Sphere-Sdf (flip)
            (CollisionShape::Sphere { radius }, CollisionShape::Sdf { data }) => self
                .detect_sdf_sphere_contact(
                    data,
                    &body_b.state.pose,
                    body_a.state.pose.position,
                    *radius,
                    body_b.id,
                    body_a.id,
                )
                .map(|c| c.flip()),

            // Sdf-Capsule
            (
                CollisionShape::Sdf { data },
                CollisionShape::Capsule {
                    half_length: _,
                    radius,
                },
            ) => {
                let (start, end) = shape_b.capsule_endpoints(&body_b.state.pose)?;
                self.detect_sdf_capsule_contact(
                    data,
                    &body_a.state.pose,
                    start,
                    end,
                    *radius,
                    body_a.id,
                    body_b.id,
                )
            }

            // Capsule-Sdf (flip)
            (
                CollisionShape::Capsule {
                    half_length: _,
                    radius,
                },
                CollisionShape::Sdf { data },
            ) => {
                let (start, end) = shape_a.capsule_endpoints(&body_a.state.pose)?;
                self.detect_sdf_capsule_contact(
                    data,
                    &body_b.state.pose,
                    start,
                    end,
                    *radius,
                    body_b.id,
                    body_a.id,
                )
                .map(|c| c.flip())
            }

            // Sdf-Box
            (CollisionShape::Sdf { data }, CollisionShape::Box { half_extents }) => self
                .detect_sdf_box_contact(
                    data,
                    &body_a.state.pose,
                    &body_b.state.pose,
                    half_extents,
                    body_a.id,
                    body_b.id,
                ),

            // Box-Sdf (flip)
            (CollisionShape::Box { half_extents }, CollisionShape::Sdf { data }) => self
                .detect_sdf_box_contact(
                    data,
                    &body_b.state.pose,
                    &body_a.state.pose,
                    half_extents,
                    body_b.id,
                    body_a.id,
                )
                .map(|c| c.flip()),

            // Sdf-Cylinder
            (
                CollisionShape::Sdf { data },
                CollisionShape::Cylinder {
                    half_length,
                    radius,
                },
            ) => self.detect_sdf_cylinder_contact(
                data,
                &body_a.state.pose,
                &body_b.state.pose,
                *half_length,
                *radius,
                body_a.id,
                body_b.id,
            ),

            // Cylinder-Sdf (flip)
            (
                CollisionShape::Cylinder {
                    half_length,
                    radius,
                },
                CollisionShape::Sdf { data },
            ) => self
                .detect_sdf_cylinder_contact(
                    data,
                    &body_b.state.pose,
                    &body_a.state.pose,
                    *half_length,
                    *radius,
                    body_b.id,
                    body_a.id,
                )
                .map(|c| c.flip()),

            // Sdf-Ellipsoid
            (CollisionShape::Sdf { data }, CollisionShape::Ellipsoid { radii }) => self
                .detect_sdf_ellipsoid_contact(
                    data,
                    &body_a.state.pose,
                    &body_b.state.pose,
                    radii,
                    body_a.id,
                    body_b.id,
                ),

            // Ellipsoid-Sdf (flip)
            (CollisionShape::Ellipsoid { radii }, CollisionShape::Sdf { data }) => self
                .detect_sdf_ellipsoid_contact(
                    data,
                    &body_b.state.pose,
                    &body_a.state.pose,
                    radii,
                    body_b.id,
                    body_a.id,
                )
                .map(|c| c.flip()),

            // Sdf-ConvexMesh
            (CollisionShape::Sdf { data }, CollisionShape::ConvexMesh { vertices }) => self
                .detect_sdf_convex_mesh_contact(
                    data,
                    &body_a.state.pose,
                    &body_b.state.pose,
                    vertices,
                    body_a.id,
                    body_b.id,
                ),

            // ConvexMesh-Sdf (flip)
            (CollisionShape::ConvexMesh { vertices }, CollisionShape::Sdf { data }) => self
                .detect_sdf_convex_mesh_contact(
                    data,
                    &body_b.state.pose,
                    &body_a.state.pose,
                    vertices,
                    body_b.id,
                    body_a.id,
                )
                .map(|c| c.flip()),

            // Sdf-Plane
            (CollisionShape::Sdf { data }, CollisionShape::Plane { normal, distance }) => self
                .detect_sdf_plane_contact(
                    data,
                    &body_a.state.pose,
                    normal,
                    *distance + normal.dot(&body_b.state.pose.position.coords),
                    body_a.id,
                    body_b.id,
                ),

            // Plane-Sdf (flip)
            (CollisionShape::Plane { normal, distance }, CollisionShape::Sdf { data }) => self
                .detect_sdf_plane_contact(
                    data,
                    &body_b.state.pose,
                    normal,
                    *distance + normal.dot(&body_a.state.pose.position.coords),
                    body_b.id,
                    body_a.id,
                )
                .map(|c| c.flip()),

            // Sdf-TriangleMesh
            (CollisionShape::Sdf { data }, CollisionShape::TriangleMesh { data: mesh }) => self
                .detect_sdf_triangle_mesh_contact(
                    data,
                    &body_a.state.pose,
                    mesh,
                    &body_b.state.pose,
                    body_a.id,
                    body_b.id,
                ),

            // TriangleMesh-Sdf (flip)
            (CollisionShape::TriangleMesh { data: mesh }, CollisionShape::Sdf { data }) => self
                .detect_sdf_triangle_mesh_contact(
                    data,
                    &body_b.state.pose,
                    mesh,
                    &body_a.state.pose,
                    body_b.id,
                    body_a.id,
                )
                .map(|c| c.flip()),

            // Sdf-HeightField
            (CollisionShape::Sdf { data }, CollisionShape::HeightField { data: hf }) => self
                .detect_sdf_heightfield_contact(
                    data,
                    &body_a.state.pose,
                    hf,
                    &body_b.state.pose,
                    body_a.id,
                    body_b.id,
                ),

            // HeightField-Sdf (flip)
            (CollisionShape::HeightField { data: hf }, CollisionShape::Sdf { data }) => self
                .detect_sdf_heightfield_contact(
                    data,
                    &body_b.state.pose,
                    hf,
                    &body_a.state.pose,
                    body_b.id,
                    body_a.id,
                )
                .map(|c| c.flip()),

            // Sdf-Sdf
            (CollisionShape::Sdf { data: data_a }, CollisionShape::Sdf { data: data_b }) => self
                .detect_sdf_sdf_contact(
                    data_a,
                    &body_a.state.pose,
                    data_b,
                    &body_b.state.pose,
                    body_a.id,
                    body_b.id,
                ),

            // =====================================================================
            // Triangle mesh collisions
            // =====================================================================

            // TriangleMesh-Sphere
            (CollisionShape::TriangleMesh { data }, CollisionShape::Sphere { radius }) => self
                .detect_mesh_sphere_contact(
                    data,
                    &body_a.state.pose,
                    body_b.state.pose.position,
                    *radius,
                    body_a.id,
                    body_b.id,
                ),

            // Sphere-TriangleMesh (flip)
            (CollisionShape::Sphere { radius }, CollisionShape::TriangleMesh { data }) => self
                .detect_mesh_sphere_contact(
                    data,
                    &body_b.state.pose,
                    body_a.state.pose.position,
                    *radius,
                    body_b.id,
                    body_a.id,
                )
                .map(|c| c.flip()),

            // TriangleMesh-Capsule
            (
                CollisionShape::TriangleMesh { data },
                CollisionShape::Capsule {
                    half_length: _,
                    radius,
                },
            ) => {
                let (start, end) = shape_b.capsule_endpoints(&body_b.state.pose)?;
                self.detect_mesh_capsule_contact(
                    data,
                    &body_a.state.pose,
                    start,
                    end,
                    *radius,
                    body_a.id,
                    body_b.id,
                )
            }

            // Capsule-TriangleMesh (flip)
            (
                CollisionShape::Capsule {
                    half_length: _,
                    radius,
                },
                CollisionShape::TriangleMesh { data },
            ) => {
                let (start, end) = shape_a.capsule_endpoints(&body_a.state.pose)?;
                self.detect_mesh_capsule_contact(
                    data,
                    &body_b.state.pose,
                    start,
                    end,
                    *radius,
                    body_b.id,
                    body_a.id,
                )
                .map(|c| c.flip())
            }

            // TriangleMesh-Box
            (CollisionShape::TriangleMesh { data }, CollisionShape::Box { half_extents }) => self
                .detect_mesh_box_contact(
                    data,
                    &body_a.state.pose,
                    &body_b.state.pose,
                    half_extents,
                    body_a.id,
                    body_b.id,
                ),

            // Box-TriangleMesh (flip)
            (CollisionShape::Box { half_extents }, CollisionShape::TriangleMesh { data }) => self
                .detect_mesh_box_contact(
                    data,
                    &body_b.state.pose,
                    &body_a.state.pose,
                    half_extents,
                    body_b.id,
                    body_a.id,
                )
                .map(|c| c.flip()),

            // TriangleMesh-TriangleMesh
            (
                CollisionShape::TriangleMesh { data: mesh_a },
                CollisionShape::TriangleMesh { data: mesh_b },
            ) => {
                let contact = crate::mesh::mesh_mesh_deepest_contact(
                    mesh_a,
                    &body_a.state.pose,
                    mesh_b,
                    &body_b.state.pose,
                )?;

                Some(ContactPoint::new(
                    contact.point,
                    contact.normal,
                    contact.penetration,
                    body_a.id,
                    body_b.id,
                ))
            }

            // =====================================================================
            // GJK/EPA fallback for ConvexMesh, Cylinder, Ellipsoid, and Capsule-Box
            // =====================================================================

            // All cases where A is ConvexMesh, Cylinder, or Ellipsoid (or Capsule-Box)
            (
                CollisionShape::ConvexMesh { .. }
                | CollisionShape::Cylinder { .. }
                | CollisionShape::Ellipsoid { .. },
                _,
            )
            | (CollisionShape::Capsule { .. }, CollisionShape::Box { .. }) => {
                self.detect_gjk_epa_contact(body_a, body_b, shape_a, shape_b)
            }

            // Flipped cases where B is ConvexMesh, Cylinder, or Ellipsoid (or Box-Capsule)
            (
                _,
                CollisionShape::ConvexMesh { .. }
                | CollisionShape::Cylinder { .. }
                | CollisionShape::Ellipsoid { .. },
            )
            | (CollisionShape::Box { .. }, CollisionShape::Capsule { .. }) => self
                .detect_gjk_epa_contact(body_b, body_a, shape_b, shape_a)
                .map(|c| c.flip()),

            // =====================================================================
            // Unsupported combinations (plane-plane, height field-height field, etc.)
            // =====================================================================
            _ => None,
        }
    }

    /// Detect contact using GJK/EPA algorithms.
    ///
    /// This is used for convex mesh collisions and any other combinations
    /// where dedicated analytical methods are not available.
    #[allow(clippy::unused_self)]
    fn detect_gjk_epa_contact(
        &self,
        body_a: &Body,
        body_b: &Body,
        shape_a: &CollisionShape,
        shape_b: &CollisionShape,
    ) -> Option<ContactPoint> {
        let gjk_contact =
            gjk_epa_contact(shape_a, &body_a.state.pose, shape_b, &body_b.state.pose)?;

        Some(ContactPoint::new(
            gjk_contact.point,
            gjk_contact.normal,
            gjk_contact.penetration,
            body_a.id,
            body_b.id,
        ))
    }

    /// Detect contact between a height field and a sphere.
    #[allow(clippy::unused_self)]
    fn detect_heightfield_sphere_contact(
        &self,
        heightfield: &crate::heightfield::HeightFieldData,
        hf_pose: &Pose,
        sphere_center: Point3<f64>,
        sphere_radius: f64,
        hf_body_id: BodyId,
        sphere_body_id: BodyId,
    ) -> Option<ContactPoint> {
        let contact = crate::heightfield::heightfield_sphere_contact(
            heightfield,
            hf_pose,
            sphere_center,
            sphere_radius,
        )?;

        Some(ContactPoint::new(
            contact.point,
            contact.normal,
            contact.penetration,
            hf_body_id,
            sphere_body_id,
        ))
    }

    /// Detect contact between a height field and a capsule.
    #[allow(clippy::unused_self, clippy::too_many_arguments)]
    fn detect_heightfield_capsule_contact(
        &self,
        heightfield: &crate::heightfield::HeightFieldData,
        hf_pose: &Pose,
        capsule_start: Point3<f64>,
        capsule_end: Point3<f64>,
        capsule_radius: f64,
        hf_body_id: BodyId,
        capsule_body_id: BodyId,
    ) -> Option<ContactPoint> {
        let contact = crate::heightfield::heightfield_capsule_contact(
            heightfield,
            hf_pose,
            capsule_start,
            capsule_end,
            capsule_radius,
        )?;

        Some(ContactPoint::new(
            contact.point,
            contact.normal,
            contact.penetration,
            hf_body_id,
            capsule_body_id,
        ))
    }

    /// Detect contact between a height field and a box.
    #[allow(clippy::unused_self)]
    fn detect_heightfield_box_contact(
        &self,
        heightfield: &crate::heightfield::HeightFieldData,
        hf_pose: &Pose,
        box_pose: &Pose,
        half_extents: &Vector3<f64>,
        hf_body_id: BodyId,
        box_body_id: BodyId,
    ) -> Option<ContactPoint> {
        let contact = crate::heightfield::heightfield_box_contact(
            heightfield,
            hf_pose,
            box_pose,
            half_extents,
        )?;

        Some(ContactPoint::new(
            contact.point,
            contact.normal,
            contact.penetration,
            hf_body_id,
            box_body_id,
        ))
    }

    /// Detect contact between an SDF and a sphere.
    #[allow(clippy::unused_self)]
    fn detect_sdf_sphere_contact(
        &self,
        sdf: &crate::sdf::SdfCollisionData,
        sdf_pose: &Pose,
        sphere_center: Point3<f64>,
        sphere_radius: f64,
        sdf_body_id: BodyId,
        sphere_body_id: BodyId,
    ) -> Option<ContactPoint> {
        let contact = crate::sdf::sdf_sphere_contact(sdf, sdf_pose, sphere_center, sphere_radius)?;

        Some(ContactPoint::new(
            contact.point,
            contact.normal,
            contact.penetration,
            sdf_body_id,
            sphere_body_id,
        ))
    }

    /// Detect contact between an SDF and a capsule.
    #[allow(clippy::unused_self, clippy::too_many_arguments)]
    fn detect_sdf_capsule_contact(
        &self,
        sdf: &crate::sdf::SdfCollisionData,
        sdf_pose: &Pose,
        capsule_start: Point3<f64>,
        capsule_end: Point3<f64>,
        capsule_radius: f64,
        sdf_body_id: BodyId,
        capsule_body_id: BodyId,
    ) -> Option<ContactPoint> {
        let contact = crate::sdf::sdf_capsule_contact(
            sdf,
            sdf_pose,
            capsule_start,
            capsule_end,
            capsule_radius,
        )?;

        Some(ContactPoint::new(
            contact.point,
            contact.normal,
            contact.penetration,
            sdf_body_id,
            capsule_body_id,
        ))
    }

    /// Detect contact between an SDF and a box.
    #[allow(clippy::unused_self)]
    fn detect_sdf_box_contact(
        &self,
        sdf: &crate::sdf::SdfCollisionData,
        sdf_pose: &Pose,
        box_pose: &Pose,
        half_extents: &Vector3<f64>,
        sdf_body_id: BodyId,
        box_body_id: BodyId,
    ) -> Option<ContactPoint> {
        let contact = crate::sdf::sdf_box_contact(sdf, sdf_pose, box_pose, half_extents)?;

        Some(ContactPoint::new(
            contact.point,
            contact.normal,
            contact.penetration,
            sdf_body_id,
            box_body_id,
        ))
    }

    /// Detect contact between an SDF and a cylinder.
    #[allow(clippy::unused_self, clippy::too_many_arguments)]
    fn detect_sdf_cylinder_contact(
        &self,
        sdf: &crate::sdf::SdfCollisionData,
        sdf_pose: &Pose,
        cylinder_pose: &Pose,
        half_height: f64,
        radius: f64,
        sdf_body_id: BodyId,
        cylinder_body_id: BodyId,
    ) -> Option<ContactPoint> {
        let contact =
            crate::sdf::sdf_cylinder_contact(sdf, sdf_pose, cylinder_pose, half_height, radius)?;

        Some(ContactPoint::new(
            contact.point,
            contact.normal,
            contact.penetration,
            sdf_body_id,
            cylinder_body_id,
        ))
    }

    /// Detect contact between an SDF and an ellipsoid.
    #[allow(clippy::unused_self)]
    fn detect_sdf_ellipsoid_contact(
        &self,
        sdf: &crate::sdf::SdfCollisionData,
        sdf_pose: &Pose,
        ellipsoid_pose: &Pose,
        radii: &Vector3<f64>,
        sdf_body_id: BodyId,
        ellipsoid_body_id: BodyId,
    ) -> Option<ContactPoint> {
        let contact = crate::sdf::sdf_ellipsoid_contact(sdf, sdf_pose, ellipsoid_pose, radii)?;

        Some(ContactPoint::new(
            contact.point,
            contact.normal,
            contact.penetration,
            sdf_body_id,
            ellipsoid_body_id,
        ))
    }

    /// Detect contact between an SDF and a convex mesh.
    #[allow(clippy::unused_self)]
    fn detect_sdf_convex_mesh_contact(
        &self,
        sdf: &crate::sdf::SdfCollisionData,
        sdf_pose: &Pose,
        mesh_pose: &Pose,
        vertices: &[Point3<f64>],
        sdf_body_id: BodyId,
        mesh_body_id: BodyId,
    ) -> Option<ContactPoint> {
        let contact = crate::sdf::sdf_convex_mesh_contact(sdf, sdf_pose, mesh_pose, vertices)?;

        Some(ContactPoint::new(
            contact.point,
            contact.normal,
            contact.penetration,
            sdf_body_id,
            mesh_body_id,
        ))
    }

    /// Detect contact between an SDF and an infinite plane.
    #[allow(clippy::unused_self)]
    fn detect_sdf_plane_contact(
        &self,
        sdf: &crate::sdf::SdfCollisionData,
        sdf_pose: &Pose,
        plane_normal: &Vector3<f64>,
        plane_offset: f64,
        sdf_body_id: BodyId,
        plane_body_id: BodyId,
    ) -> Option<ContactPoint> {
        let contact = crate::sdf::sdf_plane_contact(sdf, sdf_pose, plane_normal, plane_offset)?;

        Some(ContactPoint::new(
            contact.point,
            contact.normal,
            contact.penetration,
            sdf_body_id,
            plane_body_id,
        ))
    }

    /// Detect contact between an SDF and a triangle mesh.
    #[allow(clippy::unused_self)]
    fn detect_sdf_triangle_mesh_contact(
        &self,
        sdf: &crate::sdf::SdfCollisionData,
        sdf_pose: &Pose,
        mesh: &std::sync::Arc<crate::mesh::TriangleMeshData>,
        mesh_pose: &Pose,
        sdf_body_id: BodyId,
        mesh_body_id: BodyId,
    ) -> Option<ContactPoint> {
        let contact = crate::sdf::sdf_triangle_mesh_contact(sdf, sdf_pose, mesh, mesh_pose)?;

        Some(ContactPoint::new(
            contact.point,
            contact.normal,
            contact.penetration,
            sdf_body_id,
            mesh_body_id,
        ))
    }

    /// Detect contact between an SDF and a height field.
    #[allow(clippy::unused_self)]
    fn detect_sdf_heightfield_contact(
        &self,
        sdf: &crate::sdf::SdfCollisionData,
        sdf_pose: &Pose,
        heightfield: &crate::heightfield::HeightFieldData,
        heightfield_pose: &Pose,
        sdf_body_id: BodyId,
        heightfield_body_id: BodyId,
    ) -> Option<ContactPoint> {
        let contact =
            crate::sdf::sdf_heightfield_contact(sdf, sdf_pose, heightfield, heightfield_pose)?;

        Some(ContactPoint::new(
            contact.point,
            contact.normal,
            contact.penetration,
            sdf_body_id,
            heightfield_body_id,
        ))
    }

    /// Detect contact between two SDFs.
    #[allow(clippy::unused_self, clippy::similar_names)]
    fn detect_sdf_sdf_contact(
        &self,
        sdf_a: &crate::sdf::SdfCollisionData,
        pose_a: &Pose,
        sdf_b: &crate::sdf::SdfCollisionData,
        pose_b: &Pose,
        sdf_a_body_id: BodyId,
        sdf_b_body_id: BodyId,
    ) -> Option<ContactPoint> {
        let contact = crate::sdf::sdf_sdf_contact(sdf_a, pose_a, sdf_b, pose_b)?;

        Some(ContactPoint::new(
            contact.point,
            contact.normal,
            contact.penetration,
            sdf_a_body_id,
            sdf_b_body_id,
        ))
    }

    /// Detect contact between a triangle mesh and a sphere.
    #[allow(clippy::unused_self)]
    fn detect_mesh_sphere_contact(
        &self,
        mesh: &std::sync::Arc<crate::mesh::TriangleMeshData>,
        mesh_pose: &Pose,
        sphere_center: Point3<f64>,
        sphere_radius: f64,
        mesh_body_id: BodyId,
        sphere_body_id: BodyId,
    ) -> Option<ContactPoint> {
        // Clone the Arc to get a mutable reference for BVH lazy initialization
        let mut mesh_data = (**mesh).clone();
        let contact = crate::mesh::mesh_sphere_contact(
            &mut mesh_data,
            mesh_pose,
            sphere_center,
            sphere_radius,
        )?;

        Some(ContactPoint::new(
            contact.point,
            contact.normal,
            contact.penetration,
            mesh_body_id,
            sphere_body_id,
        ))
    }

    /// Detect contact between a triangle mesh and a capsule.
    #[allow(clippy::unused_self, clippy::too_many_arguments)]
    fn detect_mesh_capsule_contact(
        &self,
        mesh: &std::sync::Arc<crate::mesh::TriangleMeshData>,
        mesh_pose: &Pose,
        capsule_start: Point3<f64>,
        capsule_end: Point3<f64>,
        capsule_radius: f64,
        mesh_body_id: BodyId,
        capsule_body_id: BodyId,
    ) -> Option<ContactPoint> {
        let mut mesh_data = (**mesh).clone();
        let contact = crate::mesh::mesh_capsule_contact(
            &mut mesh_data,
            mesh_pose,
            capsule_start,
            capsule_end,
            capsule_radius,
        )?;

        Some(ContactPoint::new(
            contact.point,
            contact.normal,
            contact.penetration,
            mesh_body_id,
            capsule_body_id,
        ))
    }

    /// Detect contact between a triangle mesh and a box.
    #[allow(clippy::unused_self)]
    fn detect_mesh_box_contact(
        &self,
        mesh: &std::sync::Arc<crate::mesh::TriangleMeshData>,
        mesh_pose: &Pose,
        box_pose: &Pose,
        half_extents: &Vector3<f64>,
        mesh_body_id: BodyId,
        box_body_id: BodyId,
    ) -> Option<ContactPoint> {
        let mut mesh_data = (**mesh).clone();
        let contact =
            crate::mesh::mesh_box_contact(&mut mesh_data, mesh_pose, box_pose, half_extents)?;

        Some(ContactPoint::new(
            contact.point,
            contact.normal,
            contact.penetration,
            mesh_body_id,
            box_body_id,
        ))
    }

    /// Solve contacts and apply forces to bodies.
    ///
    /// This is the main contact resolution entry point. It:
    /// 1. Detects all contacts
    /// 2. Computes contact forces using the compliant model
    /// 3. Applies forces to the affected bodies
    ///
    /// Returns the number of active contacts.
    pub fn solve_contacts(&mut self) -> usize {
        let contacts = self.detect_contacts();
        if contacts.is_empty() {
            return 0;
        }

        // Create a snapshot of body velocities for the solver
        let body_velocities: HashMap<BodyId, (Vector3<f64>, Vector3<f64>, Point3<f64>)> = self
            .bodies
            .iter()
            .map(|(id, b)| {
                (
                    *id,
                    (
                        b.state.twist.linear,
                        b.state.twist.angular,
                        b.state.pose.position,
                    ),
                )
            })
            .collect();

        // Solve contacts
        let result = self.contact_solver.solve(&contacts, |body_id, contact| {
            // Get velocity at contact point for this body
            if let Some(&(linear, angular, pos)) = body_velocities.get(&body_id) {
                let r = contact.position - pos;
                linear + angular.cross(&r)
            } else {
                Vector3::zeros()
            }
        });

        // Apply forces to bodies
        for force_result in &result.forces {
            let contact = &force_result.contact;
            let force = &force_result.force;

            // Apply force to body A
            if let Some(body_a) = self.bodies.get_mut(&contact.body_a) {
                if !body_a.is_static {
                    body_a.apply_force_at_point(force.total(), force.position);
                    body_a.wake_up();
                }
            }

            // Apply reaction force to body B
            if let Some(body_b) = self.bodies.get_mut(&contact.body_b) {
                if !body_b.is_static {
                    body_b.apply_force_at_point(-force.total(), force.position);
                    body_b.wake_up();
                }
            }
        }

        // Apply position corrections for deep penetrations
        for correction in &result.position_corrections {
            // Split correction between bodies based on mass
            let mass_a = self
                .bodies
                .get(&correction.body_a)
                .map_or(f64::INFINITY, |b| {
                    if b.is_static {
                        f64::INFINITY
                    } else {
                        b.mass_props.mass
                    }
                });
            let mass_b = self
                .bodies
                .get(&correction.body_b)
                .map_or(f64::INFINITY, |b| {
                    if b.is_static {
                        f64::INFINITY
                    } else {
                        b.mass_props.mass
                    }
                });

            let total_inv_mass = 1.0 / mass_a + 1.0 / mass_b;
            if total_inv_mass > 0.0 {
                let ratio_a = (1.0 / mass_a) / total_inv_mass;
                let ratio_b = (1.0 / mass_b) / total_inv_mass;

                if let Some(body_a) = self.bodies.get_mut(&correction.body_a) {
                    if !body_a.is_static {
                        body_a.state.pose.position += correction.correction * ratio_a;
                    }
                }
                if let Some(body_b) = self.bodies.get_mut(&correction.body_b) {
                    if !body_b.is_static {
                        body_b.state.pose.position -= correction.correction * ratio_b;
                    }
                }
            }
        }

        result.forces.len()
    }

    /// Solve joint constraints and apply forces to bodies.
    ///
    /// This computes constraint forces that maintain joint connections
    /// while respecting limits and motor commands.
    ///
    /// Returns the number of constraints solved.
    pub fn solve_constraints(&mut self) -> usize {
        if self.joints.is_empty() {
            return 0;
        }

        // Create joint adapters for the solver
        let joint_adapters: Vec<JointAdapter> = self.joints.values().map(JointAdapter).collect();

        // Solve constraints
        let result = self.constraint_solver.solve(&joint_adapters, |body_id| {
            self.bodies.get(&body_id).map(|body| {
                let rotation = body.state.pose.rotation.to_rotation_matrix();
                let inv_mass = if body.is_static || body.mass_props.mass <= 0.0 {
                    0.0
                } else {
                    1.0 / body.mass_props.mass
                };
                let inv_inertia = if body.is_static {
                    Matrix3::zeros()
                } else {
                    body.mass_props
                        .inverse_inertia()
                        .unwrap_or_else(Matrix3::zeros)
                };

                ConstraintBodyState {
                    position: body.state.pose.position,
                    rotation: *rotation.matrix(),
                    linear_velocity: body.state.twist.linear,
                    angular_velocity: body.state.twist.angular,
                    inv_mass,
                    inv_inertia,
                    is_static: body.is_static,
                }
            })
        });

        // Apply forces to bodies
        for joint_force in &result.forces {
            // Apply force and torque to parent body
            if let Some(parent) = self.bodies.get_mut(&joint_force.parent) {
                if !parent.is_static {
                    parent.accumulated_force += joint_force.force.parent_force;
                    parent.accumulated_torque += joint_force.force.parent_torque;
                    parent.wake_up();
                }
            }

            // Apply force and torque to child body
            if let Some(child) = self.bodies.get_mut(&joint_force.child) {
                if !child.is_static {
                    child.accumulated_force += joint_force.force.child_force;
                    child.accumulated_torque += joint_force.force.child_torque;
                    child.wake_up();
                }
            }
        }

        result.forces.len()
    }

    /// Solve joint constraints using parallel island processing.
    ///
    /// This method uses rayon to solve independent constraint islands in parallel.
    /// It's more efficient than sequential solving when the scene contains multiple
    /// independent mechanisms (e.g., multiple robots, scattered objects).
    ///
    /// # Returns
    ///
    /// The number of forces applied.
    #[cfg(feature = "parallel")]
    pub fn solve_constraints_parallel(&mut self) -> usize {
        use sim_constraint::{BodyState, ConstraintIslands, NewtonSolverConfig};

        if self.joints.is_empty() {
            return 0;
        }

        let min_islands = self.config.solver.parallel.min_islands_for_parallel;

        // Pre-build body state snapshot for thread-safe access
        let body_states: HashMap<BodyId, BodyState> = self
            .bodies
            .iter()
            .map(|(&id, body)| {
                let rotation = body.state.pose.rotation.to_rotation_matrix();
                let inv_mass = if body.is_static || body.mass_props.mass <= 0.0 {
                    0.0
                } else {
                    1.0 / body.mass_props.mass
                };
                let inv_inertia = if body.is_static {
                    Matrix3::zeros()
                } else {
                    body.mass_props
                        .inverse_inertia()
                        .unwrap_or_else(Matrix3::zeros)
                };

                (
                    id,
                    BodyState {
                        position: body.state.pose.position,
                        rotation: *rotation.matrix(),
                        linear_velocity: body.state.twist.linear,
                        angular_velocity: body.state.twist.angular,
                        inv_mass,
                        inv_inertia,
                        is_static: body.is_static,
                    },
                )
            })
            .collect();

        // Create joint adapters
        let joint_adapters: Vec<JointAdapter> = self.joints.values().map(JointAdapter).collect();

        // Build islands
        let islands = ConstraintIslands::build_with_static_info(&joint_adapters, |id| {
            body_states.get(&id).is_none_or(|s| s.is_static)
        });

        let dt = self.config.timestep;

        // Use the parallel module to solve constraints
        let newton_config = NewtonSolverConfig::default();
        let result = sim_constraint::parallel::solve_islands_parallel(
            &newton_config,
            &joint_adapters,
            &islands,
            &body_states,
            dt,
            min_islands,
        );

        // Apply forces to bodies (sequential - modifies world)
        for joint_force in &result.forces {
            if let Some(parent) = self.bodies.get_mut(&joint_force.parent) {
                if !parent.is_static {
                    parent.accumulated_force += joint_force.force.parent_force;
                    parent.accumulated_torque += joint_force.force.parent_torque;
                    parent.wake_up();
                }
            }

            if let Some(child) = self.bodies.get_mut(&joint_force.child) {
                if !child.is_static {
                    child.accumulated_force += joint_force.force.child_force;
                    child.accumulated_torque += joint_force.force.child_torque;
                    child.wake_up();
                }
            }
        }

        result.forces.len()
    }

    // =========================================================================
    // Diagnostics
    // =========================================================================

    /// Compute the total kinetic energy of the system.
    #[must_use]
    pub fn total_kinetic_energy(&self) -> f64 {
        self.bodies
            .values()
            .filter(|b| !b.is_static)
            .map(|b| {
                b.state
                    .twist
                    .kinetic_energy(b.mass_props.mass, &b.mass_props.inertia)
            })
            .sum()
    }

    /// Compute the total linear momentum of the system.
    #[must_use]
    pub fn total_linear_momentum(&self) -> Vector3<f64> {
        self.bodies
            .values()
            .filter(|b| !b.is_static)
            .map(|b| b.state.twist.linear_momentum(b.mass_props.mass))
            .fold(Vector3::zeros(), |acc, p| acc + p)
    }

    /// Compute the system center of mass.
    #[must_use]
    pub fn center_of_mass(&self) -> Option<Point3<f64>> {
        let mut total_mass = 0.0;
        let mut weighted_pos = Vector3::zeros();

        for body in self.bodies.values() {
            if !body.is_static {
                let mass = body.mass_props.mass;
                total_mass += mass;
                weighted_pos += body.state.pose.position.coords * mass;
            }
        }

        if total_mass > 0.0 {
            Some(Point3::from(weighted_pos / total_mass))
        } else {
            None
        }
    }

    /// Cast a ray against all bodies with collision shapes.
    ///
    /// Returns the closest hit, or `None` if nothing was hit.
    ///
    /// # Arguments
    ///
    /// * `origin` - Ray origin in world coordinates
    /// * `direction` - Ray direction (unit vector) in world coordinates
    /// * `max_distance` - Maximum distance to check
    /// * `exclude_body` - Optional body to exclude from ray casting
    #[must_use]
    pub fn cast_ray(
        &self,
        origin: Point3<f64>,
        direction: nalgebra::UnitVector3<f64>,
        max_distance: f64,
        exclude_body: Option<BodyId>,
    ) -> Option<(BodyId, crate::raycast::RaycastHit)> {
        let mut closest: Option<(BodyId, crate::raycast::RaycastHit)> = None;

        for body in self.bodies.values() {
            // Skip excluded body
            if exclude_body == Some(body.id) {
                continue;
            }

            // Skip bodies without collision shapes
            let Some(ref shape) = body.collision_shape else {
                continue;
            };

            // Compute the world pose of the collision shape
            let shape_pose = body
                .collision_shape_pose
                .as_ref()
                .map_or(body.state.pose, |local| body.state.pose.compose(local));

            // Cast ray against this shape
            if let Some(hit) =
                crate::raycast::raycast_shape(shape, &shape_pose, origin, direction, max_distance)
            {
                // Keep the closest hit
                if closest.is_none()
                    || hit.distance < closest.as_ref().map_or(f64::MAX, |(_, h)| h.distance)
                {
                    closest = Some((body.id, hit));
                }
            }
        }

        closest
    }
}

// Implement RayCaster trait when sim-sensor feature is enabled
#[cfg(feature = "sensor")]
impl sim_sensor::RayCaster for World {
    fn cast_ray(
        &self,
        origin: Point3<f64>,
        direction: nalgebra::UnitVector3<f64>,
        max_distance: f64,
        exclude_body: Option<BodyId>,
    ) -> Option<sim_sensor::RayHit> {
        self.cast_ray(origin, direction, max_distance, exclude_body)
            .map(|(body_id, hit)| sim_sensor::RayHit {
                distance: hit.distance,
                point: hit.point,
                normal: hit.normal,
                body_id: Some(body_id),
            })
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
    use nalgebra::UnitQuaternion;
    use sim_types::Twist;

    #[test]
    fn test_world_creation() {
        let world = World::default();
        assert_eq!(world.body_count(), 0);
        assert_eq!(world.joint_count(), 0);
        assert_relative_eq!(world.time(), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_add_body() {
        let mut world = World::default();
        let state = RigidBodyState::at_rest(Pose::from_position(Point3::new(1.0, 2.0, 3.0)));
        let mass = MassProperties::sphere(1.0, 0.5);

        let id = world.add_body(state, mass);

        assert_eq!(world.body_count(), 1);
        let body = world.body(id).expect("body should exist");
        assert_eq!(body.id, id);
        assert_relative_eq!(body.state.pose.position.x, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_add_static_body() {
        let mut world = World::default();
        let pose = Pose::from_position(Point3::new(0.0, 0.0, 0.0));

        let id = world.add_static_body(pose);

        let body = world.body(id).expect("body should exist");
        assert!(body.is_static);
        assert!(body.mass_props.is_static());
    }

    #[test]
    fn test_body_by_name() {
        let mut world = World::default();
        let state = RigidBodyState::origin();
        let mass = MassProperties::sphere(1.0, 0.5);

        let body = Body::new(BodyId::new(1), state, mass).with_name("robot_base");
        world.insert_body(body).expect("insert should succeed");

        let found = world.body_by_name("robot_base");
        assert!(found.is_some());
        assert_eq!(found.map(|b| b.id), Some(BodyId::new(1)));
    }

    #[test]
    fn test_add_joint() {
        let mut world = World::default();
        let body1 = world.add_body(RigidBodyState::origin(), MassProperties::sphere(1.0, 0.5));
        let body2 = world.add_body(RigidBodyState::origin(), MassProperties::sphere(1.0, 0.5));

        let joint_id = world
            .add_joint(JointType::Revolute, body1, body2)
            .expect("joint creation should succeed");

        assert_eq!(world.joint_count(), 1);
        let joint = world.joint(joint_id).expect("joint should exist");
        assert_eq!(joint.parent, body1);
        assert_eq!(joint.child, body2);
    }

    #[test]
    fn test_add_joint_invalid_body() {
        let mut world = World::default();
        let body1 = world.add_body(RigidBodyState::origin(), MassProperties::sphere(1.0, 0.5));
        let invalid_body = BodyId::new(999);

        let result = world.add_joint(JointType::Revolute, body1, invalid_body);
        assert!(result.is_err());
    }

    #[test]
    fn test_apply_force() {
        let mut world = World::default();
        let state = RigidBodyState::origin();
        let mass = MassProperties::sphere(2.0, 0.5);
        let id = world.add_body(state, mass);

        {
            let body = world.body_mut(id).expect("body should exist");
            body.apply_force(Vector3::new(10.0, 0.0, 0.0));
        }

        let body = world.body(id).expect("body should exist");
        assert_relative_eq!(body.accumulated_force.x, 10.0, epsilon = 1e-10);

        // Check acceleration: a = F/m = 10/2 = 5
        let accel = body.linear_acceleration();
        assert_relative_eq!(accel.x, 5.0, epsilon = 1e-10);
    }

    #[test]
    fn test_static_body_ignores_force() {
        let mut world = World::default();
        let pose = Pose::identity();
        let id = world.add_static_body(pose);

        {
            let body = world.body_mut(id).expect("body should exist");
            body.apply_force(Vector3::new(100.0, 0.0, 0.0));
        }

        let body = world.body(id).expect("body should exist");
        assert_relative_eq!(body.accumulated_force.norm(), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_apply_gravity() {
        let mut world = World::new(SimulationConfig::default());
        let state = RigidBodyState::origin();
        let mass = MassProperties::sphere(1.0, 0.5);
        let id = world.add_body(state, mass);

        world.apply_gravity();

        let body = world.body(id).expect("body should exist");
        // Earth gravity: -9.81 m/s² in Z
        assert_relative_eq!(body.accumulated_force.z, -9.81, epsilon = 1e-10);
    }

    #[test]
    fn test_observe() {
        let mut world = World::default();
        let state = RigidBodyState::at_rest(Pose::from_position(Point3::new(1.0, 0.0, 0.0)));
        let id = world.add_body(state, MassProperties::sphere(1.0, 0.5));

        let obs = world.observe();

        assert_relative_eq!(obs.time, 0.0, epsilon = 1e-10);
        let body_state = obs.body_state(id);
        assert!(body_state.is_some());
        assert_relative_eq!(
            body_state.map(|s| s.pose.position.x).unwrap_or(0.0),
            1.0,
            epsilon = 1e-10
        );
    }

    #[test]
    fn test_total_kinetic_energy() {
        let mut world = World::default();

        // Body moving at 2 m/s with mass 1 kg: KE = 0.5 * 1 * 4 = 2 J
        let state =
            RigidBodyState::new(Pose::identity(), Twist::linear(Vector3::new(2.0, 0.0, 0.0)));
        world.add_body(state, MassProperties::point_mass(1.0));

        let ke = world.total_kinetic_energy();
        assert_relative_eq!(ke, 2.0, epsilon = 1e-10);
    }

    #[test]
    fn test_center_of_mass() {
        let mut world = World::default();

        // Two bodies of equal mass at (0,0,0) and (2,0,0)
        // COM should be at (1,0,0)
        let state1 = RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 0.0)));
        let state2 = RigidBodyState::at_rest(Pose::from_position(Point3::new(2.0, 0.0, 0.0)));

        world.add_body(state1, MassProperties::sphere(1.0, 0.5));
        world.add_body(state2, MassProperties::sphere(1.0, 0.5));

        let com = world.center_of_mass().expect("should have COM");
        assert_relative_eq!(com.x, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_validate_diverged() {
        let mut world = World::default();
        let state = RigidBodyState::new(
            Pose::from_position(Point3::new(f64::NAN, 0.0, 0.0)),
            Twist::zero(),
        );
        world.add_body(state, MassProperties::sphere(1.0, 0.5));

        let result = world.validate();
        assert!(result.is_err());
        assert!(result.unwrap_err().is_diverged());
    }

    #[test]
    fn test_remove_body() {
        let mut world = World::default();
        let id = world.add_body(RigidBodyState::origin(), MassProperties::sphere(1.0, 0.5));

        assert_eq!(world.body_count(), 1);
        let removed = world.remove_body(id);
        assert!(removed.is_some());
        assert_eq!(world.body_count(), 0);
    }

    #[test]
    fn test_is_complete() {
        let config = SimulationConfig::default().max_time(1.0);
        let mut world = World::new(config);

        assert!(!world.is_complete());
        world.advance_time(0.5);
        assert!(!world.is_complete());
        world.advance_time(0.5);
        assert!(world.is_complete());
    }

    #[test]
    fn test_apply_force_at_point() {
        let mut world = World::default();

        // Body at origin with unit sphere mass properties
        let state = RigidBodyState::origin();
        let mass = MassProperties::sphere(1.0, 1.0);
        let id = world.add_body(state, mass);

        {
            let body = world.body_mut(id).expect("body should exist");
            // Apply force at point offset from COM
            body.apply_force_at_point(Vector3::new(1.0, 0.0, 0.0), Point3::new(0.0, 0.0, 1.0));
        }

        let body = world.body(id).expect("body should exist");

        // Force should be accumulated
        assert_relative_eq!(body.accumulated_force.x, 1.0, epsilon = 1e-10);

        // Torque should be r × F = (0,0,1) × (1,0,0) = (0,1,0)
        assert_relative_eq!(body.accumulated_torque.y, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_body_with_rotation() {
        let mut world = World::default();

        let rotation = UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_2);
        let pose = Pose::from_position_rotation(Point3::new(1.0, 0.0, 0.0), rotation);
        let state = RigidBodyState::at_rest(pose);

        let id = world.add_body(state, MassProperties::sphere(1.0, 0.5));
        let body = world.body(id).expect("body should exist");

        // Check that forward direction is rotated
        let forward = body.state.pose.forward();
        assert_relative_eq!(forward.x, -1.0, epsilon = 1e-10);
        assert_relative_eq!(forward.y, 0.0, epsilon = 1e-10);
    }

    // =========================================================================
    // Contact Detection Tests
    // =========================================================================

    #[test]
    fn test_collision_shape_bounding_radius() {
        let sphere = CollisionShape::sphere(0.5);
        assert_relative_eq!(sphere.bounding_radius(), 0.5, epsilon = 1e-10);

        let box_shape = CollisionShape::box_shape(Vector3::new(1.0, 2.0, 3.0));
        // Bounding radius is the diagonal: sqrt(1² + 2² + 3²) = sqrt(14)
        assert_relative_eq!(
            box_shape.bounding_radius(),
            14.0_f64.sqrt(),
            epsilon = 1e-10
        );

        let plane = CollisionShape::ground_plane(0.0);
        assert!(plane.bounding_radius().is_infinite());
    }

    #[test]
    fn test_detect_sphere_plane_contact() {
        let mut world = World::new(SimulationConfig::default());

        // Ground plane at z=0
        let ground = Body::new_static(BodyId::new(1), Pose::identity())
            .with_collision_shape(CollisionShape::ground_plane(0.0));
        world.insert_body(ground).expect("insert should succeed");

        // Sphere at z=0.4 with radius 0.5 (penetrating by 0.1)
        let sphere_state = RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 0.4)));
        let sphere = Body::new(
            BodyId::new(2),
            sphere_state,
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_shape(CollisionShape::sphere(0.5));
        world.insert_body(sphere).expect("insert should succeed");

        let contacts = world.detect_contacts();
        assert_eq!(contacts.len(), 1, "should detect one contact");

        let contact = &contacts[0];
        assert_relative_eq!(contact.penetration, 0.1, epsilon = 1e-10);
        // Normal direction depends on which body is body_a (HashMap iteration order)
        // The normal should be along Z axis (pointing from one body toward the other)
        assert_relative_eq!(contact.normal.z.abs(), 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_detect_sphere_sphere_contact() {
        let mut world = World::new(SimulationConfig::default());

        // Two spheres with radius 0.5, centers 0.8 apart (penetrating by 0.2)
        let sphere1_state =
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 0.0)));
        let sphere1 = Body::new(
            BodyId::new(1),
            sphere1_state,
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_shape(CollisionShape::sphere(0.5));
        world.insert_body(sphere1).expect("insert should succeed");

        let sphere2_state =
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.8, 0.0, 0.0)));
        let sphere2 = Body::new(
            BodyId::new(2),
            sphere2_state,
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_shape(CollisionShape::sphere(0.5));
        world.insert_body(sphere2).expect("insert should succeed");

        let contacts = world.detect_contacts();
        assert_eq!(contacts.len(), 1, "should detect one contact");

        let contact = &contacts[0];
        assert_relative_eq!(contact.penetration, 0.2, epsilon = 1e-10);
        // Normal points from body_b toward body_a
        // The actual direction depends on which sphere is body_a (HashMap iteration order)
        // Just verify it's along X axis with unit length
        assert_relative_eq!(contact.normal.x.abs(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(contact.normal.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(contact.normal.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_no_contact_when_separated() {
        let mut world = World::new(SimulationConfig::default());

        // Ground plane at z=0
        let ground = Body::new_static(BodyId::new(1), Pose::identity())
            .with_collision_shape(CollisionShape::ground_plane(0.0));
        world.insert_body(ground).expect("insert should succeed");

        // Sphere at z=1.0 with radius 0.5 (not touching)
        let sphere_state = RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 1.0)));
        let sphere = Body::new(
            BodyId::new(2),
            sphere_state,
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_shape(CollisionShape::sphere(0.5));
        world.insert_body(sphere).expect("insert should succeed");

        let contacts = world.detect_contacts();
        assert!(contacts.is_empty(), "should detect no contacts");
    }

    #[test]
    fn test_solve_contacts_generates_forces() {
        let mut world = World::new(SimulationConfig::default());

        // Ground plane at z=0
        let ground = Body::new_static(BodyId::new(1), Pose::identity())
            .with_collision_shape(CollisionShape::ground_plane(0.0));
        world.insert_body(ground).expect("insert should succeed");

        // Sphere at z=0.4 with radius 0.5 (penetrating by 0.1)
        let sphere_state = RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 0.4)));
        let sphere = Body::new(
            BodyId::new(2),
            sphere_state,
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_shape(CollisionShape::sphere(0.5));
        world.insert_body(sphere).expect("insert should succeed");

        let contact_count = world.solve_contacts();
        assert_eq!(contact_count, 1, "should have one contact");

        // Check that the sphere received an upward force
        let sphere = world.body(BodyId::new(2)).expect("sphere should exist");
        assert!(
            sphere.accumulated_force.z > 0.0,
            "sphere should have upward contact force: {}",
            sphere.accumulated_force.z
        );
    }

    // =========================================================================
    // ConvexMesh (GJK/EPA) Contact Detection Tests
    // =========================================================================

    #[test]
    fn test_convex_mesh_sphere_contact() {
        let mut world = World::new(SimulationConfig::default());

        // Convex cube at origin
        let cube_vertices = vec![
            Point3::new(-0.5, -0.5, -0.5),
            Point3::new(0.5, -0.5, -0.5),
            Point3::new(0.5, 0.5, -0.5),
            Point3::new(-0.5, 0.5, -0.5),
            Point3::new(-0.5, -0.5, 0.5),
            Point3::new(0.5, -0.5, 0.5),
            Point3::new(0.5, 0.5, 0.5),
            Point3::new(-0.5, 0.5, 0.5),
        ];
        let cube_state = RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 0.0)));
        let cube = Body::new(
            BodyId::new(1),
            cube_state,
            MassProperties::box_shape(1.0, Vector3::new(0.5, 0.5, 0.5)),
        )
        .with_collision_shape(CollisionShape::convex_mesh(cube_vertices));
        world.insert_body(cube).expect("insert should succeed");

        // Sphere overlapping with the cube
        let sphere_state = RigidBodyState::at_rest(Pose::from_position(Point3::new(0.8, 0.0, 0.0)));
        let sphere = Body::new(
            BodyId::new(2),
            sphere_state,
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_shape(CollisionShape::sphere(0.5));
        world.insert_body(sphere).expect("insert should succeed");

        let contacts = world.detect_contacts();
        assert_eq!(contacts.len(), 1, "should detect one contact");

        let contact = &contacts[0];
        // Cube face is at x=0.5, sphere surface reaches to x=0.3
        // Penetration should be approximately 0.2
        assert!(
            contact.penetration > 0.0,
            "penetration should be positive: {}",
            contact.penetration
        );
        // Normal should be roughly along X axis
        assert!(
            contact.normal.x.abs() > 0.5,
            "normal should point along X: {:?}",
            contact.normal
        );
    }

    #[test]
    fn test_convex_mesh_convex_mesh_contact() {
        let mut world = World::new(SimulationConfig::default());

        // Two tetrahedra
        let tetra_state_a =
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 0.0)));
        let tetra_a = Body::new(
            BodyId::new(1),
            tetra_state_a,
            MassProperties::sphere(1.0, 0.5), // Approximate mass
        )
        .with_collision_shape(CollisionShape::tetrahedron(0.5));
        world.insert_body(tetra_a).expect("insert should succeed");

        // Second tetrahedron overlapping
        let tetra_state_b =
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.3, 0.0, 0.0)));
        let tetra_b = Body::new(
            BodyId::new(2),
            tetra_state_b,
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_shape(CollisionShape::tetrahedron(0.5));
        world.insert_body(tetra_b).expect("insert should succeed");

        let contacts = world.detect_contacts();
        assert_eq!(contacts.len(), 1, "should detect one contact");

        let contact = &contacts[0];
        assert!(
            contact.penetration > 0.0,
            "penetration should be positive: {}",
            contact.penetration
        );
    }

    #[test]
    fn test_convex_mesh_no_contact_when_separated() {
        let mut world = World::new(SimulationConfig::default());

        // Convex cube at origin
        let cube_vertices = vec![
            Point3::new(-0.5, -0.5, -0.5),
            Point3::new(0.5, -0.5, -0.5),
            Point3::new(0.5, 0.5, -0.5),
            Point3::new(-0.5, 0.5, -0.5),
            Point3::new(-0.5, -0.5, 0.5),
            Point3::new(0.5, -0.5, 0.5),
            Point3::new(0.5, 0.5, 0.5),
            Point3::new(-0.5, 0.5, 0.5),
        ];
        let cube_state = RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 0.0)));
        let cube = Body::new(
            BodyId::new(1),
            cube_state,
            MassProperties::box_shape(1.0, Vector3::new(0.5, 0.5, 0.5)),
        )
        .with_collision_shape(CollisionShape::convex_mesh(cube_vertices));
        world.insert_body(cube).expect("insert should succeed");

        // Sphere far away from the cube
        let sphere_state = RigidBodyState::at_rest(Pose::from_position(Point3::new(3.0, 0.0, 0.0)));
        let sphere = Body::new(
            BodyId::new(2),
            sphere_state,
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_shape(CollisionShape::sphere(0.5));
        world.insert_body(sphere).expect("insert should succeed");

        let contacts = world.detect_contacts();
        assert!(contacts.is_empty(), "should detect no contacts");
    }

    #[test]
    fn test_convex_mesh_bounding_radius() {
        // Test that bounding radius works for convex mesh
        let vertices = vec![
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 2.0), // This is the furthest at distance 2
        ];
        let shape = CollisionShape::convex_mesh(vertices);
        assert_relative_eq!(shape.bounding_radius(), 2.0, epsilon = 1e-10);
    }

    #[test]
    fn test_tetrahedron_creation() {
        // Test the tetrahedron helper
        let tetra = CollisionShape::tetrahedron(1.0);
        let CollisionShape::ConvexMesh { vertices } = tetra else {
            unreachable!("tetrahedron() always returns ConvexMesh")
        };
        assert_eq!(vertices.len(), 4, "tetrahedron should have 4 vertices");
        // All vertices should be at distance 1.732... from origin
        // (since vertices are at (±1, ±1, ±1) positions)
        for v in &vertices {
            let dist = v.coords.norm();
            assert_relative_eq!(dist, 3.0_f64.sqrt(), epsilon = 1e-10);
        }
    }

    #[test]
    fn test_capsule_box_gjk_contact() {
        let mut world = World::new(SimulationConfig::default());

        // Capsule at origin (oriented along Z)
        let capsule_state =
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 0.5)));
        let capsule = Body::new(
            BodyId::new(1),
            capsule_state,
            MassProperties::cylinder(1.0, 0.3, 0.5),
        )
        .with_collision_shape(CollisionShape::capsule(0.5, 0.3));
        world.insert_body(capsule).expect("insert should succeed");

        // Box overlapping with the capsule
        let box_state = RigidBodyState::at_rest(Pose::from_position(Point3::new(0.5, 0.0, 0.5)));
        let box_body = Body::new(
            BodyId::new(2),
            box_state,
            MassProperties::box_shape(1.0, Vector3::new(0.3, 0.3, 0.3)),
        )
        .with_collision_shape(CollisionShape::box_shape(Vector3::new(0.3, 0.3, 0.3)));
        world.insert_body(box_body).expect("insert should succeed");

        let contacts = world.detect_contacts();
        assert_eq!(contacts.len(), 1, "should detect one contact via GJK/EPA");

        let contact = &contacts[0];
        assert!(
            contact.penetration > 0.0,
            "penetration should be positive: {}",
            contact.penetration
        );
    }

    #[test]
    fn test_triangle_mesh_sphere_contact() {
        let mut world = World::new(SimulationConfig::default());

        // Create a simple floor mesh (two triangles forming a square)
        let vertices = vec![
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(-1.0, 1.0, 0.0),
        ];
        let indices = vec![0, 1, 2, 0, 2, 3]; // Two triangles forming a square

        let mesh_body = Body::new_static(BodyId::new(1), Pose::identity()).with_collision_shape(
            CollisionShape::triangle_mesh_from_vertices(vertices, indices),
        );
        world.insert_body(mesh_body).expect("insert should succeed");

        // Sphere slightly above the floor (penetrating it)
        let sphere_state = RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 0.3)));
        let sphere = Body::new(
            BodyId::new(2),
            sphere_state,
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_shape(CollisionShape::sphere(0.5));
        world.insert_body(sphere).expect("insert should succeed");

        let contacts = world.detect_contacts();
        assert_eq!(
            contacts.len(),
            1,
            "should detect one contact with triangle mesh"
        );

        let contact = &contacts[0];
        assert!(
            contact.penetration > 0.0,
            "penetration should be positive: {}",
            contact.penetration
        );
        // Normal should point upward (away from mesh toward sphere)
        assert!(
            contact.normal.z.abs() > 0.9,
            "normal should be roughly vertical: {:?}",
            contact.normal
        );
    }

    #[test]
    fn test_triangle_mesh_no_contact() {
        let mut world = World::new(SimulationConfig::default());

        // Create a simple floor mesh
        let vertices = vec![
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(-1.0, 1.0, 0.0),
        ];
        let indices = vec![0, 1, 2, 0, 2, 3];

        let mesh_body = Body::new_static(BodyId::new(1), Pose::identity()).with_collision_shape(
            CollisionShape::triangle_mesh_from_vertices(vertices, indices),
        );
        world.insert_body(mesh_body).expect("insert should succeed");

        // Sphere far above the floor (not touching)
        let sphere_state = RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 2.0)));
        let sphere = Body::new(
            BodyId::new(2),
            sphere_state,
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_shape(CollisionShape::sphere(0.5));
        world.insert_body(sphere).expect("insert should succeed");

        let contacts = world.detect_contacts();
        assert!(
            contacts.is_empty(),
            "should not detect contact when sphere is far above mesh"
        );
    }

    // =========================================================================
    // Collision Filtering Tests
    // =========================================================================

    #[test]
    fn test_collision_filter_default() {
        // Bodies with default filtering should collide
        let body_a = Body::new(
            BodyId::new(1),
            RigidBodyState::origin(),
            MassProperties::sphere(1.0, 0.5),
        );
        let body_b = Body::new(
            BodyId::new(2),
            RigidBodyState::origin(),
            MassProperties::sphere(1.0, 0.5),
        );

        assert!(body_a.can_collide_with(&body_b));
        assert!(body_b.can_collide_with(&body_a));
    }

    #[test]
    fn test_collision_filter_disabled() {
        // contype=0 means this body doesn't generate contacts
        let body_a = Body::new(
            BodyId::new(1),
            RigidBodyState::origin(),
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_filter(0, 1); // contype=0 disables collisions from this body

        let body_b = Body::new(
            BodyId::new(2),
            RigidBodyState::origin(),
            MassProperties::sphere(1.0, 0.5),
        );

        assert!(!body_a.can_collide_with(&body_b));
        assert!(!body_b.can_collide_with(&body_a)); // Both directions fail
    }

    #[test]
    fn test_collision_filter_groups() {
        // Use different bits for different collision groups
        // Group 1 (bit 0), Group 2 (bit 1)
        let body_group1_a = Body::new(
            BodyId::new(1),
            RigidBodyState::origin(),
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_filter(1, 1); // contype=1, conaffinity=1 (group 1 only)

        let body_group1_b = Body::new(
            BodyId::new(2),
            RigidBodyState::origin(),
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_filter(1, 1);

        let body_group2 = Body::new(
            BodyId::new(3),
            RigidBodyState::origin(),
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_filter(2, 2); // contype=2, conaffinity=2 (group 2 only)

        // Bodies in same group should collide
        assert!(body_group1_a.can_collide_with(&body_group1_b));

        // Bodies in different groups should not collide
        assert!(!body_group1_a.can_collide_with(&body_group2));
        assert!(!body_group2.can_collide_with(&body_group1_a));
    }

    #[test]
    fn test_collision_filter_cross_group() {
        // A body that collides with multiple groups
        let body_a = Body::new(
            BodyId::new(1),
            RigidBodyState::origin(),
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_filter(1, 3); // contype=1, conaffinity=3 (groups 1 and 2)

        let body_b = Body::new(
            BodyId::new(2),
            RigidBodyState::origin(),
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_filter(2, 3); // contype=2, conaffinity=3 (groups 1 and 2)

        // Both can collide with each other
        assert!(body_a.can_collide_with(&body_b));
        assert!(body_b.can_collide_with(&body_a));
    }

    #[test]
    fn test_collision_filter_in_world() {
        let mut world = World::new(SimulationConfig::default());

        // Two overlapping spheres that should NOT collide (different groups)
        let sphere_a = Body::new(
            BodyId::new(1),
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 0.0))),
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_shape(CollisionShape::sphere(1.0))
        .with_collision_filter(1, 1); // Group 1

        let sphere_b = Body::new(
            BodyId::new(2),
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.5, 0.0, 0.0))),
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_shape(CollisionShape::sphere(1.0))
        .with_collision_filter(2, 2); // Group 2 (no overlap with group 1)

        world.insert_body(sphere_a).expect("insert should succeed");
        world.insert_body(sphere_b).expect("insert should succeed");

        // Should detect NO contacts due to filtering
        let contacts = world.detect_contacts();
        assert!(
            contacts.is_empty(),
            "should not detect contacts between different collision groups"
        );
    }

    #[test]
    fn test_collision_filter_in_world_same_group() {
        let mut world = World::new(SimulationConfig::default());

        // Two overlapping spheres that SHOULD collide (same group)
        let sphere_a = Body::new(
            BodyId::new(1),
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 0.0))),
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_shape(CollisionShape::sphere(1.0))
        .with_collision_filter(1, 1);

        let sphere_b = Body::new(
            BodyId::new(2),
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.5, 0.0, 0.0))),
            MassProperties::sphere(1.0, 0.5),
        )
        .with_collision_shape(CollisionShape::sphere(1.0))
        .with_collision_filter(1, 1);

        world.insert_body(sphere_a).expect("insert should succeed");
        world.insert_body(sphere_b).expect("insert should succeed");

        // Should detect contacts
        let contacts = world.detect_contacts();
        assert_eq!(
            contacts.len(),
            1,
            "should detect contact between bodies in same collision group"
        );
    }

    // =========================================================================
    // TriangleMesh-TriangleMesh Collision Tests (Milestone 4)
    // =========================================================================

    /// Helper to create a simple cube mesh for testing.
    fn create_test_cube_mesh() -> (Vec<Point3<f64>>, Vec<usize>) {
        let vertices = vec![
            // Bottom face
            Point3::new(-0.5, -0.5, -0.5),
            Point3::new(0.5, -0.5, -0.5),
            Point3::new(0.5, 0.5, -0.5),
            Point3::new(-0.5, 0.5, -0.5),
            // Top face
            Point3::new(-0.5, -0.5, 0.5),
            Point3::new(0.5, -0.5, 0.5),
            Point3::new(0.5, 0.5, 0.5),
            Point3::new(-0.5, 0.5, 0.5),
        ];
        // Two triangles per face (12 triangles total)
        let indices = vec![
            // Bottom (-Z)
            0, 1, 2, 0, 2, 3, // Top (+Z)
            4, 6, 5, 4, 7, 6, // Front (-Y)
            0, 5, 1, 0, 4, 5, // Back (+Y)
            2, 7, 3, 2, 6, 7, // Left (-X)
            0, 7, 4, 0, 3, 7, // Right (+X)
            1, 6, 2, 1, 5, 6,
        ];
        (vertices, indices)
    }

    /// Helper to create a simple tetrahedron mesh for testing.
    fn create_test_tetrahedron_mesh() -> (Vec<Point3<f64>>, Vec<usize>) {
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 1.0, 0.0),
            Point3::new(0.5, 0.33, 0.8),
        ];
        let indices = vec![
            0, 1, 2, // bottom
            0, 1, 3, // front
            1, 2, 3, // right
            0, 2, 3, // left
        ];
        (vertices, indices)
    }

    #[test]
    fn test_triangle_mesh_mesh_contact() {
        let mut world = World::new(SimulationConfig::default());

        // Create two cube meshes
        let (vertices_a, indices_a) = create_test_cube_mesh();
        let (vertices_b, indices_b) = create_test_cube_mesh();

        // Mesh A at origin (static)
        let mesh_a = Body::new_static(BodyId::new(1), Pose::identity()).with_collision_shape(
            CollisionShape::triangle_mesh_from_vertices(vertices_a, indices_a),
        );
        world.insert_body(mesh_a).expect("insert should succeed");

        // Mesh B overlapping with A (dynamic body shifted 0.3 units in X for deeper penetration)
        let mesh_b_state = RigidBodyState::at_rest(Pose::from_position(Point3::new(0.3, 0.0, 0.0)));
        let mesh_b = Body::new(
            BodyId::new(2),
            mesh_b_state,
            MassProperties::box_shape(1.0, Vector3::new(0.5, 0.5, 0.5)),
        )
        .with_collision_shape(CollisionShape::triangle_mesh_from_vertices(
            vertices_b, indices_b,
        ));
        world.insert_body(mesh_b).expect("insert should succeed");

        let contacts = world.detect_contacts();
        assert!(
            !contacts.is_empty(),
            "overlapping cube meshes should produce contacts"
        );

        // Verify contact properties - penetration can be >= 0 for valid contacts
        let contact = &contacts[0];
        assert!(
            contact.penetration >= 0.0,
            "penetration should be non-negative: {}",
            contact.penetration
        );
    }

    #[test]
    fn test_triangle_mesh_mesh_no_contact() {
        let mut world = World::new(SimulationConfig::default());

        let (vertices_a, indices_a) = create_test_cube_mesh();
        let (vertices_b, indices_b) = create_test_cube_mesh();

        // Mesh A at origin
        let mesh_a = Body::new_static(BodyId::new(1), Pose::identity()).with_collision_shape(
            CollisionShape::triangle_mesh_from_vertices(vertices_a, indices_a),
        );
        world.insert_body(mesh_a).expect("insert should succeed");

        // Mesh B far away (no contact expected)
        let mesh_b_state = RigidBodyState::at_rest(Pose::from_position(Point3::new(5.0, 0.0, 0.0)));
        let mesh_b = Body::new(
            BodyId::new(2),
            mesh_b_state,
            MassProperties::box_shape(1.0, Vector3::new(0.5, 0.5, 0.5)),
        )
        .with_collision_shape(CollisionShape::triangle_mesh_from_vertices(
            vertices_b, indices_b,
        ));
        world.insert_body(mesh_b).expect("insert should succeed");

        let contacts = world.detect_contacts();
        assert!(
            contacts.is_empty(),
            "separate cube meshes should not produce contacts"
        );
    }

    #[test]
    fn test_triangle_mesh_mesh_tetrahedra() {
        let mut world = World::new(SimulationConfig::default());

        let (vertices_a, indices_a) = create_test_tetrahedron_mesh();
        let (vertices_b, indices_b) = create_test_tetrahedron_mesh();

        // Tetrahedron A at origin
        let mesh_a = Body::new_static(BodyId::new(1), Pose::identity()).with_collision_shape(
            CollisionShape::triangle_mesh_from_vertices(vertices_a, indices_a),
        );
        world.insert_body(mesh_a).expect("insert should succeed");

        // Tetrahedron B overlapping (shifted to intersect)
        let mesh_b_state = RigidBodyState::at_rest(Pose::from_position(Point3::new(0.3, 0.3, 0.0)));
        let mesh_b = Body::new(
            BodyId::new(2),
            mesh_b_state,
            MassProperties::sphere(1.0, 0.5), // Approximate mass
        )
        .with_collision_shape(CollisionShape::triangle_mesh_from_vertices(
            vertices_b, indices_b,
        ));
        world.insert_body(mesh_b).expect("insert should succeed");

        let contacts = world.detect_contacts();
        assert!(
            !contacts.is_empty(),
            "overlapping tetrahedra should produce contacts"
        );
    }

    #[test]
    fn test_triangle_mesh_mesh_rotated() {
        let mut world = World::new(SimulationConfig::default());

        let (vertices_a, indices_a) = create_test_cube_mesh();
        let (vertices_b, indices_b) = create_test_cube_mesh();

        // Mesh A at origin
        let mesh_a = Body::new_static(BodyId::new(1), Pose::identity()).with_collision_shape(
            CollisionShape::triangle_mesh_from_vertices(vertices_a, indices_a),
        );
        world.insert_body(mesh_a).expect("insert should succeed");

        // Mesh B rotated 45 degrees around Z and positioned to overlap
        let rotation = UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_4);
        let pose_b = Pose::from_position_rotation(Point3::new(0.3, 0.3, 0.0), rotation);
        let mesh_b_state = RigidBodyState::at_rest(pose_b);
        let mesh_b = Body::new(
            BodyId::new(2),
            mesh_b_state,
            MassProperties::box_shape(1.0, Vector3::new(0.5, 0.5, 0.5)),
        )
        .with_collision_shape(CollisionShape::triangle_mesh_from_vertices(
            vertices_b, indices_b,
        ));
        world.insert_body(mesh_b).expect("insert should succeed");

        let contacts = world.detect_contacts();
        assert!(
            !contacts.is_empty(),
            "rotated overlapping cube meshes should produce contacts"
        );
    }

    #[test]
    fn test_triangle_mesh_mesh_identical_position() {
        let mut world = World::new(SimulationConfig::default());

        let (vertices_a, indices_a) = create_test_cube_mesh();
        let (vertices_b, indices_b) = create_test_cube_mesh();

        // Both meshes at the same position (maximum overlap)
        let mesh_a = Body::new_static(BodyId::new(1), Pose::identity()).with_collision_shape(
            CollisionShape::triangle_mesh_from_vertices(vertices_a, indices_a),
        );
        world.insert_body(mesh_a).expect("insert should succeed");

        let mesh_b_state = RigidBodyState::at_rest(Pose::identity());
        let mesh_b = Body::new(
            BodyId::new(2),
            mesh_b_state,
            MassProperties::box_shape(1.0, Vector3::new(0.5, 0.5, 0.5)),
        )
        .with_collision_shape(CollisionShape::triangle_mesh_from_vertices(
            vertices_b, indices_b,
        ));
        world.insert_body(mesh_b).expect("insert should succeed");

        let contacts = world.detect_contacts();
        assert!(
            !contacts.is_empty(),
            "identical position meshes should produce contacts"
        );
    }

    #[test]
    fn test_triangle_mesh_mesh_contact_normal_is_unit() {
        let mut world = World::new(SimulationConfig::default());

        let (vertices_a, indices_a) = create_test_cube_mesh();
        let (vertices_b, indices_b) = create_test_cube_mesh();

        // Mesh A at origin
        let mesh_a = Body::new_static(BodyId::new(1), Pose::identity()).with_collision_shape(
            CollisionShape::triangle_mesh_from_vertices(vertices_a, indices_a),
        );
        world.insert_body(mesh_a).expect("insert should succeed");

        // Mesh B above A, penetrating downward
        let mesh_b_state = RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 0.7)));
        let mesh_b = Body::new(
            BodyId::new(2),
            mesh_b_state,
            MassProperties::box_shape(1.0, Vector3::new(0.5, 0.5, 0.5)),
        )
        .with_collision_shape(CollisionShape::triangle_mesh_from_vertices(
            vertices_b, indices_b,
        ));
        world.insert_body(mesh_b).expect("insert should succeed");

        let contacts = world.detect_contacts();
        assert!(
            !contacts.is_empty(),
            "vertically overlapping meshes should produce contacts"
        );

        // The contact normal should be a unit vector
        let contact = &contacts[0];
        let normal_len = contact.normal.norm();
        assert!(
            (normal_len - 1.0).abs() < 0.01,
            "contact normal should be unit length: {} (normal: {:?})",
            normal_len,
            contact.normal
        );
    }

    #[test]
    fn test_triangle_mesh_mesh_multiple_contacts() {
        // Test that we get all contacts between two mesh pairs
        let mut world = World::new(SimulationConfig::default());

        let (vertices_a, indices_a) = create_test_cube_mesh();
        let (vertices_b, indices_b) = create_test_cube_mesh();

        // Mesh A at origin
        let mesh_a = Body::new_static(BodyId::new(1), Pose::identity()).with_collision_shape(
            CollisionShape::triangle_mesh_from_vertices(vertices_a, indices_a),
        );
        world.insert_body(mesh_a).expect("insert should succeed");

        // Mesh B overlapping significantly with A
        let mesh_b_state = RigidBodyState::at_rest(Pose::identity()); // Identical position
        let mesh_b = Body::new(
            BodyId::new(2),
            mesh_b_state,
            MassProperties::box_shape(1.0, Vector3::new(0.5, 0.5, 0.5)),
        )
        .with_collision_shape(CollisionShape::triangle_mesh_from_vertices(
            vertices_b, indices_b,
        ));
        world.insert_body(mesh_b).expect("insert should succeed");

        let contacts = world.detect_contacts();

        // Should detect at least one contact when meshes fully overlap
        assert!(
            !contacts.is_empty(),
            "fully overlapping meshes should produce contacts"
        );

        // All contacts should have valid normals (unit length)
        for contact in &contacts {
            let normal_len = contact.normal.norm();
            assert!(
                (normal_len - 1.0).abs() < 0.01,
                "contact normal should be unit length: {}",
                normal_len
            );
        }
    }

    #[test]
    fn test_cast_ray_hits_sphere() {
        let mut world = World::default();

        // Add a sphere at z=5
        let sphere_id = world.add_body(
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 5.0))),
            MassProperties::sphere(1.0, 1.0),
        );
        world.body_mut(sphere_id).unwrap().collision_shape = Some(CollisionShape::sphere(1.0));

        // Cast ray from origin toward sphere
        let origin = Point3::origin();
        let direction = nalgebra::UnitVector3::new_normalize(Vector3::z());

        let result = world.cast_ray(origin, direction, 10.0, None);

        assert!(result.is_some(), "ray should hit the sphere");
        let (hit_body_id, hit) = result.unwrap();
        assert_eq!(hit_body_id, sphere_id);
        assert_relative_eq!(hit.distance, 4.0, epsilon = 1e-6);
        assert_relative_eq!(hit.normal.z, -1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_cast_ray_misses() {
        let mut world = World::default();

        // Add a sphere at x=5 (off to the side)
        let sphere_id = world.add_body(
            RigidBodyState::at_rest(Pose::from_position(Point3::new(5.0, 0.0, 5.0))),
            MassProperties::sphere(1.0, 1.0),
        );
        world.body_mut(sphere_id).unwrap().collision_shape = Some(CollisionShape::sphere(1.0));

        // Cast ray from origin straight up (should miss sphere)
        let origin = Point3::origin();
        let direction = nalgebra::UnitVector3::new_normalize(Vector3::z());

        let result = world.cast_ray(origin, direction, 10.0, None);

        assert!(result.is_none(), "ray should miss the sphere");
    }

    #[test]
    fn test_cast_ray_excludes_body() {
        let mut world = World::default();

        // Add two spheres along z axis
        let sphere1_id = world.add_body(
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 3.0))),
            MassProperties::sphere(1.0, 1.0),
        );
        world.body_mut(sphere1_id).unwrap().collision_shape = Some(CollisionShape::sphere(1.0));

        let sphere2_id = world.add_body(
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 7.0))),
            MassProperties::sphere(1.0, 1.0),
        );
        world.body_mut(sphere2_id).unwrap().collision_shape = Some(CollisionShape::sphere(1.0));

        // Cast ray from origin, excluding sphere1
        let origin = Point3::origin();
        let direction = nalgebra::UnitVector3::new_normalize(Vector3::z());

        let result = world.cast_ray(origin, direction, 10.0, Some(sphere1_id));

        assert!(result.is_some(), "ray should hit sphere2");
        let (hit_body_id, hit) = result.unwrap();
        assert_eq!(hit_body_id, sphere2_id);
        assert_relative_eq!(hit.distance, 6.0, epsilon = 1e-6); // 7 - 1 = 6
    }

    #[test]
    fn test_cast_ray_returns_closest_hit() {
        let mut world = World::default();

        // Add two spheres along z axis
        let near_sphere = world.add_body(
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 3.0))),
            MassProperties::sphere(1.0, 1.0),
        );
        world.body_mut(near_sphere).unwrap().collision_shape = Some(CollisionShape::sphere(1.0));

        let far_sphere = world.add_body(
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 8.0))),
            MassProperties::sphere(1.0, 1.0),
        );
        world.body_mut(far_sphere).unwrap().collision_shape = Some(CollisionShape::sphere(1.0));

        // Cast ray from origin
        let origin = Point3::origin();
        let direction = nalgebra::UnitVector3::new_normalize(Vector3::z());

        let result = world.cast_ray(origin, direction, 20.0, None);

        assert!(result.is_some(), "ray should hit");
        let (hit_body_id, hit) = result.unwrap();
        assert_eq!(hit_body_id, near_sphere, "should hit the closer sphere");
        assert_relative_eq!(hit.distance, 2.0, epsilon = 1e-6); // 3 - 1 = 2
    }

    #[test]
    fn test_cast_ray_respects_max_distance() {
        let mut world = World::default();

        // Add a sphere at z=10
        let sphere_id = world.add_body(
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 10.0))),
            MassProperties::sphere(1.0, 1.0),
        );
        world.body_mut(sphere_id).unwrap().collision_shape = Some(CollisionShape::sphere(1.0));

        // Cast ray with max_distance=5 (sphere is at distance 9)
        let origin = Point3::origin();
        let direction = nalgebra::UnitVector3::new_normalize(Vector3::z());

        let result = world.cast_ray(origin, direction, 5.0, None);

        assert!(result.is_none(), "ray should not hit beyond max_distance");
    }

    #[test]
    fn test_cast_ray_hits_box() {
        let mut world = World::default();

        // Add a box at z=5
        let box_id = world.add_body(
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 5.0))),
            MassProperties::box_shape(1.0, Vector3::new(1.0, 1.0, 1.0)),
        );
        world.body_mut(box_id).unwrap().collision_shape =
            Some(CollisionShape::box_shape(Vector3::new(1.0, 1.0, 1.0)));

        // Cast ray from origin toward box
        let origin = Point3::origin();
        let direction = nalgebra::UnitVector3::new_normalize(Vector3::z());

        let result = world.cast_ray(origin, direction, 10.0, None);

        assert!(result.is_some(), "ray should hit the box");
        let (hit_body_id, hit) = result.unwrap();
        assert_eq!(hit_body_id, box_id);
        assert_relative_eq!(hit.distance, 4.0, epsilon = 1e-6); // 5 - 1 = 4
        assert_relative_eq!(hit.normal.z, -1.0, epsilon = 1e-6);
    }
}
