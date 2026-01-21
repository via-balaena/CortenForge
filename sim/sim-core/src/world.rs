//! Simulation world container and entity management.
//!
//! The [`World`] is the central data structure for simulation state.
//! It manages rigid bodies, their properties, and provides query interfaces.

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

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Collision shape for contact detection.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum CollisionShape {
    /// Sphere with given radius.
    Sphere {
        /// Sphere radius in meters.
        radius: f64,
    },
    /// Infinite plane with normal and distance from origin.
    /// The plane equation is: normal · x = distance
    Plane {
        /// Unit normal vector of the plane.
        normal: Vector3<f64>,
        /// Distance from origin along the normal.
        distance: f64,
    },
    /// Axis-aligned box with half-extents.
    Box {
        /// Half-extents of the box in each axis.
        half_extents: Vector3<f64>,
    },
    /// Capsule (cylinder with hemispherical caps).
    ///
    /// Defined by half-length along the local Z-axis and radius.
    /// The capsule extends from `-half_length` to `+half_length` on Z.
    Capsule {
        /// Half-length of the cylindrical portion along the Z-axis.
        half_length: f64,
        /// Radius of the capsule.
        radius: f64,
    },
    /// Convex mesh defined by a set of vertices.
    ///
    /// The vertices should form a convex hull. GJK/EPA algorithms are used
    /// for collision detection with this shape type.
    ///
    /// **Important**: Vertices are stored in **local coordinates**. They will be
    /// transformed to world space using the body's pose during collision detection.
    ConvexMesh {
        /// Vertices of the convex hull in local coordinates.
        /// Should be a convex set of points (non-convex inputs may produce
        /// incorrect collision results).
        vertices: Vec<Point3<f64>>,
    },
    /// Cylinder (without hemispherical caps, unlike capsule).
    ///
    /// Defined by half-length along the local Z-axis and radius.
    /// The cylinder extends from `-half_length` to `+half_length` on Z.
    /// Uses GJK/EPA for collision detection.
    Cylinder {
        /// Half-length of the cylinder along the Z-axis.
        half_length: f64,
        /// Radius of the cylinder.
        radius: f64,
    },
    /// Ellipsoid (scaled sphere).
    ///
    /// Defined by three radii along the local X, Y, and Z axes.
    /// Uses GJK/EPA for collision detection.
    Ellipsoid {
        /// Radii along each local axis (X, Y, Z).
        radii: Vector3<f64>,
    },
}

impl CollisionShape {
    /// Create a sphere collision shape.
    #[must_use]
    pub fn sphere(radius: f64) -> Self {
        Self::Sphere { radius }
    }

    /// Create a ground plane (Z-up at given height).
    #[must_use]
    pub fn ground_plane(height: f64) -> Self {
        Self::Plane {
            normal: Vector3::z(),
            distance: height,
        }
    }

    /// Create a plane with custom normal and distance.
    #[must_use]
    pub fn plane(normal: Vector3<f64>, distance: f64) -> Self {
        Self::Plane {
            normal: normal.normalize(),
            distance,
        }
    }

    /// Create a box collision shape.
    #[must_use]
    pub fn box_shape(half_extents: Vector3<f64>) -> Self {
        Self::Box { half_extents }
    }

    /// Create a capsule collision shape.
    ///
    /// The capsule is oriented along the local Z-axis, extending from
    /// `-half_length` to `+half_length`, with hemispherical caps of the given radius.
    #[must_use]
    pub fn capsule(half_length: f64, radius: f64) -> Self {
        Self::Capsule {
            half_length,
            radius,
        }
    }

    /// Create a convex mesh collision shape from vertices.
    ///
    /// The vertices should form a convex hull in local coordinates.
    /// For best results, ensure the vertices are actually convex (e.g., compute
    /// a convex hull first if the input might be non-convex).
    ///
    /// # Panics
    ///
    /// Panics if vertices is empty.
    #[must_use]
    pub fn convex_mesh(vertices: Vec<Point3<f64>>) -> Self {
        assert!(
            !vertices.is_empty(),
            "ConvexMesh requires at least one vertex"
        );
        Self::ConvexMesh { vertices }
    }

    /// Create a convex mesh from a slice of vertices.
    #[must_use]
    pub fn convex_mesh_from_slice(vertices: &[Point3<f64>]) -> Self {
        Self::convex_mesh(vertices.to_vec())
    }

    /// Create a regular tetrahedron centered at the origin with the given circumradius.
    #[must_use]
    pub fn tetrahedron(circumradius: f64) -> Self {
        // Regular tetrahedron vertices
        let a = circumradius;
        let vertices = vec![
            Point3::new(a, a, a),
            Point3::new(a, -a, -a),
            Point3::new(-a, a, -a),
            Point3::new(-a, -a, a),
        ];
        Self::ConvexMesh { vertices }
    }

    /// Create a cylinder collision shape.
    ///
    /// The cylinder is oriented along the local Z-axis, extending from
    /// `-half_length` to `+half_length`, with the given radius.
    ///
    /// Unlike capsules, cylinders have flat caps at both ends.
    #[must_use]
    pub fn cylinder(half_length: f64, radius: f64) -> Self {
        Self::Cylinder {
            half_length,
            radius,
        }
    }

    /// Create an ellipsoid collision shape.
    ///
    /// The ellipsoid is defined by three radii along the local X, Y, and Z axes.
    /// An ellipsoid with equal radii is equivalent to a sphere.
    #[must_use]
    pub fn ellipsoid(radii: Vector3<f64>) -> Self {
        Self::Ellipsoid { radii }
    }

    /// Create an ellipsoid collision shape from individual radii.
    #[must_use]
    pub fn ellipsoid_xyz(rx: f64, ry: f64, rz: f64) -> Self {
        Self::Ellipsoid {
            radii: Vector3::new(rx, ry, rz),
        }
    }

    /// Get the bounding sphere radius for broad-phase culling.
    #[must_use]
    pub fn bounding_radius(&self) -> f64 {
        match self {
            Self::Sphere { radius } => *radius,
            Self::Plane { .. } => f64::INFINITY,
            Self::Box { half_extents } => half_extents.norm(),
            Self::Capsule {
                half_length,
                radius,
            } => half_length + radius,
            Self::ConvexMesh { vertices } => {
                // Maximum distance from origin to any vertex
                vertices.iter().map(|v| v.coords.norm()).fold(0.0, f64::max)
            }
            Self::Cylinder {
                half_length,
                radius,
            } => {
                // Corner of the cylinder (on the circular edge at the end)
                half_length.hypot(*radius)
            }
            Self::Ellipsoid { radii } => {
                // Maximum of the three radii
                radii.x.max(radii.y).max(radii.z)
            }
        }
    }

    /// Get the capsule endpoints in world coordinates given a body pose.
    ///
    /// Returns None if this is not a capsule shape.
    #[must_use]
    pub fn capsule_endpoints(&self, pose: &Pose) -> Option<(Point3<f64>, Point3<f64>)> {
        match self {
            Self::Capsule { half_length, .. } => {
                // Capsule is aligned with local Z-axis
                let local_start = Point3::new(0.0, 0.0, -*half_length);
                let local_end = Point3::new(0.0, 0.0, *half_length);
                Some((
                    pose.transform_point(&local_start),
                    pose.transform_point(&local_end),
                ))
            }
            _ => None,
        }
    }
}

/// A rigid body in the simulation world.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
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
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
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
    #[must_use]
    pub fn with_axis(mut self, axis: Vector3<f64>) -> Self {
        self.axis = axis.normalize();
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
            // Map Spherical, Planar, and Free to Spherical (3 DOF approximation)
            JointType::Spherical | JointType::Planar | JointType::Free => {
                ConstraintJointType::Spherical
            }
            // Map Cylindrical to Universal (2 DOF)
            JointType::Cylindrical => ConstraintJointType::Universal,
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
}

/// The simulation world containing all entities.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
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
    #[cfg_attr(feature = "serde", serde(skip, default = "default_contact_solver"))]
    contact_solver: ContactSolver,
    /// Contact parameters (can be changed for domain randomization).
    contact_params: ContactParams,
    /// Joint constraint solver.
    #[cfg_attr(feature = "serde", serde(skip, default = "default_constraint_solver"))]
    constraint_solver: JointConstraintSolver,
    /// Broad-phase collision detector.
    #[cfg_attr(feature = "serde", serde(skip, default = "default_broad_phase"))]
    broad_phase: BroadPhaseDetector,
}

#[cfg(feature = "serde")]
fn default_contact_solver() -> ContactSolver {
    ContactSolver::new(
        ContactModel::new(ContactParams::default()),
        ContactSolverConfig::default(),
    )
}

#[cfg(feature = "serde")]
fn default_constraint_solver() -> JointConstraintSolver {
    JointConstraintSolver::new(sim_constraint::ConstraintSolverConfig::default())
}

#[cfg(feature = "serde")]
fn default_broad_phase() -> BroadPhaseDetector {
    BroadPhaseDetector::default()
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
        self.contact_solver = ContactSolver::new(contact_model, ContactSolverConfig::default());
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
            // Unsupported combinations (plane-plane)
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
                let inv_mass = if body.is_static {
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
}
