//! Equality constraints for joint coupling and body connections.
//!
//! This module provides constraints that couple multiple joints together,
//! enabling mechanisms like:
//!
//! - **Gear ratios**: One joint drives another at a fixed ratio
//! - **Differential drives**: Two motors driving a single output
//! - **Parallel linkages**: Multiple joints that must move together
//! - **Mimic joints**: One joint copies another (with optional offset)
//!
//! # Joint Coupling Constraint
//!
//! The primary constraint is [`JointCoupling`] which enforces a linear
//! relationship between joint positions:
//!
//! ```text
//! Σᵢ cᵢ · qᵢ = offset
//! ```
//!
//! Where `cᵢ` are coupling coefficients and `qᵢ` are joint positions.
//!
//! # Example
//!
//! ```
//! use sim_constraint::equality::{JointCoupling, GearCoupling, DifferentialCoupling};
//! use sim_types::JointId;
//!
//! // Simple gear ratio: joint1 rotates 2x faster than joint0
//! let gear = JointCoupling::gear(
//!     JointId::new(0), // driver
//!     JointId::new(1), // driven
//!     2.0,             // gear ratio
//! );
//!
//! // Differential drive: two motors driving a wheel
//! let diff = DifferentialCoupling::new(
//!     JointId::new(0), // left motor
//!     JointId::new(1), // right motor
//!     JointId::new(2), // output
//! );
//!
//! // Mimic joint: joint2 copies joint0 with 90 degree offset
//! let mimic = JointCoupling::mimic(
//!     JointId::new(0),
//!     JointId::new(2),
//!     1.0,
//!     std::f64::consts::FRAC_PI_2,
//! );
//! ```

use nalgebra::{UnitQuaternion, Vector3};
use sim_types::{BodyId, JointId};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

// ============================================================================
// Joint Coupling Coefficient
// ============================================================================

/// A single coefficient in a joint coupling constraint.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CouplingCoefficient {
    /// The joint this coefficient applies to.
    pub joint: JointId,

    /// The coupling coefficient (positive = same direction, negative = opposite).
    pub coefficient: f64,
}

impl CouplingCoefficient {
    /// Create a new coupling coefficient.
    #[must_use]
    pub fn new(joint: JointId, coefficient: f64) -> Self {
        Self { joint, coefficient }
    }
}

// ============================================================================
// Joint Coupling Constraint
// ============================================================================

/// A linear constraint coupling multiple joints together.
///
/// Enforces: `Σᵢ cᵢ · qᵢ = offset`
///
/// This is the fundamental building block for:
/// - Gear trains
/// - Mimic joints (URDF)
/// - Parallel mechanisms
/// - Tendon-driven joints
///
/// # Constraint Stabilization
///
/// The constraint uses Baumgarte stabilization to correct position drift:
///
/// ```text
/// C_dot = Σᵢ cᵢ · q_dot_i + β · C / dt = 0
/// ```
///
/// Where β is the stabilization factor (typically 0.1-0.5).
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct JointCoupling {
    /// Constraint name (for debugging).
    name: String,

    /// Coupling coefficients for each joint.
    coefficients: Vec<CouplingCoefficient>,

    /// Target value for the constraint (usually 0).
    offset: f64,

    /// Baumgarte stabilization factor.
    baumgarte_factor: f64,

    /// Whether this constraint is enabled.
    enabled: bool,

    /// Compliance (inverse stiffness). 0 = rigid constraint.
    compliance: f64,

    /// Damping coefficient for velocity-level constraint.
    damping: f64,
}

impl JointCoupling {
    /// Create a new joint coupling constraint.
    #[must_use]
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            coefficients: Vec::new(),
            offset: 0.0,
            baumgarte_factor: 0.2,
            enabled: true,
            compliance: 0.0,
            damping: 0.0,
        }
    }

    /// Add a joint to the coupling.
    #[must_use]
    pub fn with_joint(mut self, joint: JointId, coefficient: f64) -> Self {
        self.coefficients
            .push(CouplingCoefficient::new(joint, coefficient));
        self
    }

    /// Set the constraint offset.
    #[must_use]
    pub fn with_offset(mut self, offset: f64) -> Self {
        self.offset = offset;
        self
    }

    /// Set the Baumgarte stabilization factor.
    #[must_use]
    pub fn with_baumgarte(mut self, factor: f64) -> Self {
        self.baumgarte_factor = factor.clamp(0.0, 1.0);
        self
    }

    /// Set the compliance (softness) of the constraint.
    #[must_use]
    pub fn with_compliance(mut self, compliance: f64) -> Self {
        self.compliance = compliance.max(0.0);
        self
    }

    /// Set the damping coefficient.
    #[must_use]
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.damping = damping.max(0.0);
        self
    }

    /// Enable or disable the constraint.
    #[must_use]
    pub fn with_enabled(mut self, enabled: bool) -> Self {
        self.enabled = enabled;
        self
    }

    /// Create a simple gear coupling between two joints.
    ///
    /// Enforces: `q_driver - ratio · q_driven = 0`
    ///
    /// A ratio > 1 means the output joint moves faster (reduction gear).
    /// A ratio < 1 means the output joint moves slower (overdrive).
    /// A negative ratio reverses direction.
    #[must_use]
    #[allow(clippy::similar_names)]
    pub fn gear(driver: JointId, driven: JointId, ratio: f64) -> Self {
        Self::new("gear")
            .with_joint(driver, 1.0)
            .with_joint(driven, -ratio)
    }

    /// Create a mimic joint constraint (URDF-style).
    ///
    /// Enforces: `q_follower = multiplier · q_leader + offset`
    /// Or equivalently: `q_follower - multiplier · q_leader = offset`
    #[must_use]
    pub fn mimic(leader: JointId, follower: JointId, multiplier: f64, offset: f64) -> Self {
        Self::new("mimic")
            .with_joint(follower, 1.0)
            .with_joint(leader, -multiplier)
            .with_offset(offset)
    }

    /// Create a parallel linkage constraint (joints move together).
    ///
    /// Enforces: `q_1 = q_2`
    #[must_use]
    pub fn parallel(joint1: JointId, joint2: JointId) -> Self {
        Self::gear(joint1, joint2, 1.0)
    }

    /// Create an anti-parallel constraint (joints move opposite).
    ///
    /// Enforces: `q_1 = -q_2`
    #[must_use]
    pub fn anti_parallel(joint1: JointId, joint2: JointId) -> Self {
        Self::gear(joint1, joint2, -1.0)
    }

    /// Get the constraint name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get the coupling coefficients.
    #[must_use]
    pub fn coefficients(&self) -> &[CouplingCoefficient] {
        &self.coefficients
    }

    /// Get the constraint offset.
    #[must_use]
    pub fn offset(&self) -> f64 {
        self.offset
    }

    /// Check if the constraint is enabled.
    #[must_use]
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Enable the constraint.
    pub fn enable(&mut self) {
        self.enabled = true;
    }

    /// Disable the constraint.
    pub fn disable(&mut self) {
        self.enabled = false;
    }

    /// Get the number of joints in this coupling.
    #[must_use]
    pub fn num_joints(&self) -> usize {
        self.coefficients.len()
    }

    /// Compute the constraint error given joint positions.
    ///
    /// # Arguments
    ///
    /// * `get_position` - Function to get joint position by ID
    ///
    /// # Returns
    ///
    /// The constraint error: `Σᵢ cᵢ · qᵢ - offset`
    #[must_use]
    pub fn compute_error<F>(&self, get_position: F) -> f64
    where
        F: Fn(JointId) -> f64,
    {
        if !self.enabled {
            return 0.0;
        }

        let mut sum = 0.0;
        for coef in &self.coefficients {
            sum += coef.coefficient * get_position(coef.joint);
        }
        sum - self.offset
    }

    /// Compute the constraint velocity given joint velocities.
    ///
    /// # Arguments
    ///
    /// * `get_velocity` - Function to get joint velocity by ID
    ///
    /// # Returns
    ///
    /// The constraint velocity: `Σᵢ cᵢ · q_dot_i`
    #[must_use]
    pub fn compute_velocity<F>(&self, get_velocity: F) -> f64
    where
        F: Fn(JointId) -> f64,
    {
        if !self.enabled {
            return 0.0;
        }

        let mut sum = 0.0;
        for coef in &self.coefficients {
            sum += coef.coefficient * get_velocity(coef.joint);
        }
        sum
    }

    /// Compute the constraint impulse needed to satisfy the constraint.
    ///
    /// Uses the stabilized constraint formulation:
    /// `C_dot + β · C / dt = 0`
    ///
    /// # Arguments
    ///
    /// * `get_position` - Function to get joint position by ID
    /// * `get_velocity` - Function to get joint velocity by ID
    /// * `dt` - Time step
    ///
    /// # Returns
    ///
    /// The required velocity correction magnitude.
    #[must_use]
    pub fn compute_correction<P, V>(&self, get_position: P, get_velocity: V, dt: f64) -> f64
    where
        P: Fn(JointId) -> f64,
        V: Fn(JointId) -> f64,
    {
        if !self.enabled || self.coefficients.is_empty() {
            return 0.0;
        }

        let position_error = self.compute_error(get_position);
        let velocity_error = self.compute_velocity(get_velocity);

        // Compute effective mass (sum of squared coefficients divided by joint masses)
        // For now, assume unit mass for each joint
        let mut effective_mass_inv = 0.0;
        for coef in &self.coefficients {
            effective_mass_inv += coef.coefficient * coef.coefficient;
        }

        if effective_mass_inv.abs() < 1e-10 {
            return 0.0;
        }

        // Add compliance and damping
        let effective_mass_inv = effective_mass_inv + self.compliance / (dt * dt);

        // Stabilized constraint: -velocity_error - (baumgarte / dt) * position_error
        let bias = self.baumgarte_factor * position_error / dt;
        let target_velocity = self
            .damping
            .mul_add(-velocity_error, -velocity_error - bias);

        target_velocity / effective_mass_inv
    }

    /// Compute the force/impulse for each joint to satisfy the constraint.
    ///
    /// # Arguments
    ///
    /// * `get_position` - Function to get joint position by ID
    /// * `get_velocity` - Function to get joint velocity by ID
    /// * `dt` - Time step
    ///
    /// # Returns
    ///
    /// Vector of (`JointId`, force) pairs.
    #[must_use]
    pub fn compute_joint_forces<P, V>(
        &self,
        get_position: P,
        get_velocity: V,
        dt: f64,
    ) -> Vec<(JointId, f64)>
    where
        P: Fn(JointId) -> f64,
        V: Fn(JointId) -> f64,
    {
        let lambda = self.compute_correction(&get_position, &get_velocity, dt);

        self.coefficients
            .iter()
            .map(|coef| (coef.joint, -coef.coefficient * lambda))
            .collect()
    }
}

impl Default for JointCoupling {
    fn default() -> Self {
        Self::new("coupling")
    }
}

// ============================================================================
// Gear Coupling (Convenience Struct)
// ============================================================================

/// A gear coupling constraint between two joints.
///
/// This is a specialized [`JointCoupling`] for the common case of
/// two joints connected by a gear train.
///
/// Enforces: `q_output = ratio · q_input`
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct GearCoupling {
    /// Input (driver) joint.
    input: JointId,

    /// Output (driven) joint.
    output: JointId,

    /// Gear ratio (output/input).
    ratio: f64,

    /// Underlying coupling constraint.
    coupling: JointCoupling,
}

impl GearCoupling {
    /// Create a new gear coupling.
    ///
    /// # Arguments
    ///
    /// * `input` - The input (driver) joint
    /// * `output` - The output (driven) joint
    /// * `ratio` - Gear ratio (output speed / input speed)
    ///
    /// A ratio > 1 means the output moves faster (speed increase).
    /// A ratio < 1 means the output moves slower (speed reduction).
    #[must_use]
    pub fn new(input: JointId, output: JointId, ratio: f64) -> Self {
        Self {
            input,
            output,
            ratio,
            coupling: JointCoupling::gear(input, output, ratio),
        }
    }

    /// Get the input joint.
    #[must_use]
    pub fn input(&self) -> JointId {
        self.input
    }

    /// Get the output joint.
    #[must_use]
    pub fn output(&self) -> JointId {
        self.output
    }

    /// Get the gear ratio.
    #[must_use]
    pub fn ratio(&self) -> f64 {
        self.ratio
    }

    /// Set the compliance.
    #[must_use]
    pub fn with_compliance(mut self, compliance: f64) -> Self {
        self.coupling = self.coupling.with_compliance(compliance);
        self
    }

    /// Set the damping.
    #[must_use]
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.coupling = self.coupling.with_damping(damping);
        self
    }

    /// Get the underlying coupling constraint.
    #[must_use]
    pub fn coupling(&self) -> &JointCoupling {
        &self.coupling
    }

    /// Get a mutable reference to the underlying coupling constraint.
    pub fn coupling_mut(&mut self) -> &mut JointCoupling {
        &mut self.coupling
    }

    /// Compute the output position given input position.
    #[must_use]
    pub fn output_from_input(&self, input_position: f64) -> f64 {
        input_position * self.ratio
    }

    /// Compute the required input position for a desired output.
    #[must_use]
    pub fn input_from_output(&self, output_position: f64) -> f64 {
        if self.ratio.abs() < 1e-10 {
            0.0
        } else {
            output_position / self.ratio
        }
    }

    /// Create a common reduction gear (output slower than input).
    ///
    /// # Arguments
    ///
    /// * `input` - Input joint
    /// * `output` - Output joint
    /// * `reduction` - Reduction ratio (e.g., 10 means 10:1 reduction)
    #[must_use]
    pub fn reduction(input: JointId, output: JointId, reduction: f64) -> Self {
        Self::new(input, output, 1.0 / reduction.abs().max(1e-6))
    }

    /// Create an overdrive gear (output faster than input).
    ///
    /// # Arguments
    ///
    /// * `input` - Input joint
    /// * `output` - Output joint
    /// * `overdrive` - Overdrive ratio (e.g., 2 means output is 2x faster)
    #[must_use]
    pub fn overdrive(input: JointId, output: JointId, overdrive: f64) -> Self {
        Self::new(input, output, overdrive.abs().max(1e-6))
    }

    /// Create a reversing gear (output moves opposite to input).
    #[must_use]
    pub fn reverse(input: JointId, output: JointId) -> Self {
        Self::new(input, output, -1.0)
    }
}

// ============================================================================
// Differential Coupling
// ============================================================================

/// A differential coupling connecting two inputs to one output.
///
/// Models a mechanical differential where:
///
/// ```text
/// output = (input1 + input2) / 2  (for averaging differential)
/// output = input1 - input2        (for difference differential)
/// ```
///
/// Common applications:
/// - Vehicle differentials
/// - Differential drive robots (steering)
/// - Parallel manipulator kinematics
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct DifferentialCoupling {
    /// First input joint.
    input1: JointId,

    /// Second input joint.
    input2: JointId,

    /// Output joint.
    output: JointId,

    /// Coefficient for input1.
    coef1: f64,

    /// Coefficient for input2.
    coef2: f64,

    /// Underlying coupling constraint.
    coupling: JointCoupling,
}

impl DifferentialCoupling {
    /// Create a new averaging differential.
    ///
    /// Enforces: `output = (input1 + input2) / 2`
    #[must_use]
    pub fn new(input1: JointId, input2: JointId, output: JointId) -> Self {
        Self::averaging(input1, input2, output)
    }

    /// Create an averaging differential.
    ///
    /// Enforces: `output = (input1 + input2) / 2`
    #[must_use]
    pub fn averaging(input1: JointId, input2: JointId, output: JointId) -> Self {
        // output - 0.5*input1 - 0.5*input2 = 0
        let coupling = JointCoupling::new("differential_avg")
            .with_joint(output, 1.0)
            .with_joint(input1, -0.5)
            .with_joint(input2, -0.5);

        Self {
            input1,
            input2,
            output,
            coef1: 0.5,
            coef2: 0.5,
            coupling,
        }
    }

    /// Create a difference differential.
    ///
    /// Enforces: `output = input1 - input2`
    #[must_use]
    pub fn difference(input1: JointId, input2: JointId, output: JointId) -> Self {
        // output - input1 + input2 = 0
        let coupling = JointCoupling::new("differential_diff")
            .with_joint(output, 1.0)
            .with_joint(input1, -1.0)
            .with_joint(input2, 1.0);

        Self {
            input1,
            input2,
            output,
            coef1: 1.0,
            coef2: -1.0,
            coupling,
        }
    }

    /// Create a weighted differential with custom coefficients.
    ///
    /// Enforces: `output = coef1 * input1 + coef2 * input2`
    #[must_use]
    pub fn weighted(
        input1: JointId,
        input2: JointId,
        output: JointId,
        coef1: f64,
        coef2: f64,
    ) -> Self {
        let coupling = JointCoupling::new("differential_weighted")
            .with_joint(output, 1.0)
            .with_joint(input1, -coef1)
            .with_joint(input2, -coef2);

        Self {
            input1,
            input2,
            output,
            coef1,
            coef2,
            coupling,
        }
    }

    /// Get the first input joint.
    #[must_use]
    pub fn input1(&self) -> JointId {
        self.input1
    }

    /// Get the second input joint.
    #[must_use]
    pub fn input2(&self) -> JointId {
        self.input2
    }

    /// Get the output joint.
    #[must_use]
    pub fn output(&self) -> JointId {
        self.output
    }

    /// Get the coefficients.
    #[must_use]
    pub fn coefficients(&self) -> (f64, f64) {
        (self.coef1, self.coef2)
    }

    /// Set the compliance.
    #[must_use]
    pub fn with_compliance(mut self, compliance: f64) -> Self {
        self.coupling = self.coupling.with_compliance(compliance);
        self
    }

    /// Set the damping.
    #[must_use]
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.coupling = self.coupling.with_damping(damping);
        self
    }

    /// Get the underlying coupling constraint.
    #[must_use]
    pub fn coupling(&self) -> &JointCoupling {
        &self.coupling
    }

    /// Get a mutable reference to the underlying coupling constraint.
    pub fn coupling_mut(&mut self) -> &mut JointCoupling {
        &mut self.coupling
    }

    /// Compute the output position given input positions.
    #[must_use]
    pub fn compute_output(&self, input1_pos: f64, input2_pos: f64) -> f64 {
        self.coef1.mul_add(input1_pos, self.coef2 * input2_pos)
    }
}

// ============================================================================
// Coupling Group
// ============================================================================

/// A group of coupling constraints that are solved together.
///
/// This allows efficient batch processing of multiple couplings
/// and provides helpers for common constraint patterns.
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CouplingGroup {
    /// The coupling constraints in this group.
    couplings: Vec<JointCoupling>,
}

impl CouplingGroup {
    /// Create a new empty coupling group.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a coupling constraint.
    pub fn add(&mut self, coupling: JointCoupling) {
        self.couplings.push(coupling);
    }

    /// Add a coupling constraint (builder pattern).
    #[must_use]
    pub fn with_coupling(mut self, coupling: JointCoupling) -> Self {
        self.add(coupling);
        self
    }

    /// Add a gear coupling.
    #[must_use]
    pub fn with_gear(self, input: JointId, output: JointId, ratio: f64) -> Self {
        self.with_coupling(JointCoupling::gear(input, output, ratio))
    }

    /// Add a mimic coupling.
    #[must_use]
    pub fn with_mimic(
        self,
        leader: JointId,
        follower: JointId,
        multiplier: f64,
        offset: f64,
    ) -> Self {
        self.with_coupling(JointCoupling::mimic(leader, follower, multiplier, offset))
    }

    /// Get the number of constraints.
    #[must_use]
    pub fn len(&self) -> usize {
        self.couplings.len()
    }

    /// Check if the group is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.couplings.is_empty()
    }

    /// Get the constraints.
    #[must_use]
    pub fn couplings(&self) -> &[JointCoupling] {
        &self.couplings
    }

    /// Get mutable access to constraints.
    pub fn couplings_mut(&mut self) -> &mut [JointCoupling] {
        &mut self.couplings
    }

    /// Compute forces for all constraints.
    ///
    /// # Returns
    ///
    /// Map of joint ID to accumulated force from all constraints.
    pub fn compute_all_forces<P, V>(
        &self,
        get_position: P,
        get_velocity: V,
        dt: f64,
    ) -> std::collections::HashMap<JointId, f64>
    where
        P: Fn(JointId) -> f64 + Copy,
        V: Fn(JointId) -> f64 + Copy,
    {
        let mut forces = std::collections::HashMap::new();

        for coupling in &self.couplings {
            for (joint, force) in coupling.compute_joint_forces(get_position, get_velocity, dt) {
                *forces.entry(joint).or_insert(0.0) += force;
            }
        }

        forces
    }

    /// Get all joints involved in any constraint.
    #[must_use]
    pub fn all_joints(&self) -> Vec<JointId> {
        let mut joints = std::collections::HashSet::new();
        for coupling in &self.couplings {
            for coef in coupling.coefficients() {
                joints.insert(coef.joint);
            }
        }
        joints.into_iter().collect()
    }
}

// ============================================================================
// Tendon Constraint
// ============================================================================

/// A tendon constraint that couples multiple joints through a cable/tendon path.
///
/// Tendons model inextensible cables that route through multiple joints,
/// coupling their motion. Unlike simple gear couplings, tendons:
///
/// - Can have varying moment arms at each joint
/// - Can be routed through multiple waypoints
/// - Model the total tendon length as constant (or spring-like)
/// - Support slack/taut states
///
/// # Constraint Formulation
///
/// The tendon constraint enforces:
///
/// ```text
/// L_total = Σᵢ rᵢ · qᵢ + L_rest
/// ```
///
/// Where `rᵢ` is the moment arm at joint i, `qᵢ` is the joint position,
/// and `L_rest` is the rest length when all joints are at zero.
///
/// # Example
///
/// ```ignore
/// use sim_constraint::equality::TendonConstraint;
/// use sim_types::JointId;
///
/// // Finger tendon routing through three joints
/// let tendon = TendonConstraint::new("flexor")
///     .with_joint(JointId::new(0), 0.01)  // MCP joint, 10mm moment arm
///     .with_joint(JointId::new(1), 0.008) // PIP joint, 8mm moment arm
///     .with_joint(JointId::new(2), 0.005) // DIP joint, 5mm moment arm
///     .with_rest_length(0.1);             // 100mm rest length
/// ```
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct TendonConstraint {
    /// Constraint name (for debugging).
    name: String,

    /// Joints and their moment arms (`joint_id`, `moment_arm`).
    /// Positive moment arm means tendon shortens when joint increases.
    joints: Vec<(JointId, f64)>,

    /// Rest length of the tendon when all joints are at zero.
    rest_length: f64,

    /// Total length the tendon should maintain (`rest_length` by default).
    target_length: f64,

    /// Stiffness of the tendon (N/m). 0 = inextensible constraint.
    stiffness: f64,

    /// Damping coefficient for tendon stretching.
    damping: f64,

    /// Whether the tendon can go slack (push vs pull only).
    can_slack: bool,

    /// Whether this constraint is enabled.
    enabled: bool,

    /// Baumgarte stabilization factor.
    baumgarte_factor: f64,

    /// Maximum tension force the tendon can apply.
    max_tension: f64,
}

impl TendonConstraint {
    /// Create a new tendon constraint.
    #[must_use]
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            joints: Vec::new(),
            rest_length: 0.0,
            target_length: 0.0,
            stiffness: 0.0, // Inextensible by default
            damping: 0.0,
            can_slack: true, // Default: can go slack (pull only)
            enabled: true,
            baumgarte_factor: 0.2,
            max_tension: f64::INFINITY,
        }
    }

    /// Add a joint with its moment arm to the tendon path.
    ///
    /// The moment arm determines how much the tendon length changes per unit
    /// joint rotation. Positive moment arm means tendon shortens when joint
    /// angle increases.
    #[must_use]
    pub fn with_joint(mut self, joint: JointId, moment_arm: f64) -> Self {
        self.joints.push((joint, moment_arm));
        self
    }

    /// Set the rest length of the tendon.
    ///
    /// This is the length when all joints are at zero position.
    #[must_use]
    pub fn with_rest_length(mut self, length: f64) -> Self {
        self.rest_length = length.max(0.0);
        self.target_length = self.rest_length;
        self
    }

    /// Set the target length (for variable-length actuated tendons).
    #[must_use]
    pub fn with_target_length(mut self, length: f64) -> Self {
        self.target_length = length.max(0.0);
        self
    }

    /// Set the stiffness (0 = inextensible, >0 = elastic).
    ///
    /// For biological tendons, stiffness is typically 100-1000 N/mm.
    #[must_use]
    pub fn with_stiffness(mut self, stiffness: f64) -> Self {
        self.stiffness = stiffness.max(0.0);
        self
    }

    /// Set the damping coefficient.
    #[must_use]
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.damping = damping.max(0.0);
        self
    }

    /// Set whether the tendon can go slack.
    ///
    /// If true (default), the tendon only applies pulling forces.
    /// If false, the tendon acts as a rigid rod (bidirectional constraint).
    #[must_use]
    pub fn with_can_slack(mut self, can_slack: bool) -> Self {
        self.can_slack = can_slack;
        self
    }

    /// Set maximum tension force.
    #[must_use]
    pub fn with_max_tension(mut self, max_tension: f64) -> Self {
        self.max_tension = max_tension.abs();
        self
    }

    /// Set the Baumgarte stabilization factor.
    #[must_use]
    pub fn with_baumgarte(mut self, factor: f64) -> Self {
        self.baumgarte_factor = factor.clamp(0.0, 1.0);
        self
    }

    /// Enable or disable the constraint.
    #[must_use]
    pub fn with_enabled(mut self, enabled: bool) -> Self {
        self.enabled = enabled;
        self
    }

    /// Get the constraint name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get the joints and moment arms.
    #[must_use]
    pub fn joints(&self) -> &[(JointId, f64)] {
        &self.joints
    }

    /// Get the rest length.
    #[must_use]
    pub fn rest_length(&self) -> f64 {
        self.rest_length
    }

    /// Get the target length.
    #[must_use]
    pub fn target_length(&self) -> f64 {
        self.target_length
    }

    /// Get the stiffness.
    #[must_use]
    pub fn stiffness(&self) -> f64 {
        self.stiffness
    }

    /// Check if enabled.
    #[must_use]
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Enable the constraint.
    pub fn enable(&mut self) {
        self.enabled = true;
    }

    /// Disable the constraint.
    pub fn disable(&mut self) {
        self.enabled = false;
    }

    /// Compute the current tendon length given joint positions.
    ///
    /// # Arguments
    ///
    /// * `get_position` - Function to get joint position by ID
    ///
    /// # Returns
    ///
    /// The current tendon length: `L_rest + Σᵢ rᵢ · qᵢ`
    #[must_use]
    pub fn compute_length<F>(&self, get_position: F) -> f64
    where
        F: Fn(JointId) -> f64,
    {
        let mut length = self.rest_length;
        for &(joint, moment_arm) in &self.joints {
            length += moment_arm * get_position(joint);
        }
        length
    }

    /// Compute the tendon length rate given joint velocities.
    #[must_use]
    pub fn compute_length_rate<F>(&self, get_velocity: F) -> f64
    where
        F: Fn(JointId) -> f64,
    {
        let mut rate = 0.0;
        for &(joint, moment_arm) in &self.joints {
            rate += moment_arm * get_velocity(joint);
        }
        rate
    }

    /// Compute the length error (how much the tendon has stretched/compressed).
    #[must_use]
    pub fn compute_error<F>(&self, get_position: F) -> f64
    where
        F: Fn(JointId) -> f64,
    {
        if !self.enabled {
            return 0.0;
        }

        let current_length = self.compute_length(get_position);
        let error = current_length - self.target_length;

        // If tendon can slack and is not taut, no constraint error
        if self.can_slack && error > 0.0 {
            return 0.0;
        }

        error
    }

    /// Check if the tendon is currently taut.
    #[must_use]
    pub fn is_taut<F>(&self, get_position: F) -> bool
    where
        F: Fn(JointId) -> f64,
    {
        let current_length = self.compute_length(get_position);
        current_length <= self.target_length
    }

    /// Compute the tension force in the tendon.
    ///
    /// Returns 0 if tendon is slack (when `can_slack` is true).
    #[must_use]
    pub fn compute_tension<P, V>(&self, get_position: P, get_velocity: V, dt: f64) -> f64
    where
        P: Fn(JointId) -> f64,
        V: Fn(JointId) -> f64,
    {
        if !self.enabled || self.joints.is_empty() {
            return 0.0;
        }

        let length_error = self.compute_error(&get_position);

        // Check slack condition
        if self.can_slack && length_error > 0.0 {
            return 0.0;
        }

        let length_rate = self.compute_length_rate(&get_velocity);

        // Compute tension based on whether tendon is elastic or inextensible
        let tension = if self.stiffness > 0.0 {
            // Elastic tendon: F = k * stretch + c * stretch_rate
            let stretch = -length_error; // Positive when tendon is stretched
            self.stiffness.mul_add(stretch, self.damping * -length_rate)
        } else {
            // Inextensible tendon: compute impulse using Baumgarte stabilization
            let mut effective_mass_inv = 0.0;
            for &(_, moment_arm) in &self.joints {
                effective_mass_inv += moment_arm * moment_arm;
            }

            if effective_mass_inv.abs() < 1e-10 {
                return 0.0;
            }

            // Constraint impulse: corrects position and velocity errors
            let bias = self.baumgarte_factor * (-length_error) / dt;
            let target_velocity = self.damping.mul_add(-length_rate, -length_rate - bias);

            (target_velocity / effective_mass_inv).abs()
        };

        // Clamp to max tension and ensure non-negative (pulling only)
        if self.can_slack {
            tension.clamp(0.0, self.max_tension)
        } else {
            tension.clamp(-self.max_tension, self.max_tension)
        }
    }

    /// Compute the forces on each joint due to tendon tension.
    ///
    /// # Returns
    ///
    /// Vector of (`JointId`, torque) pairs.
    #[must_use]
    pub fn compute_joint_forces<P, V>(
        &self,
        get_position: P,
        get_velocity: V,
        dt: f64,
    ) -> Vec<(JointId, f64)>
    where
        P: Fn(JointId) -> f64,
        V: Fn(JointId) -> f64,
    {
        let tension = self.compute_tension(&get_position, &get_velocity, dt);

        self.joints
            .iter()
            .map(|&(joint, moment_arm)| {
                // Torque = moment_arm * tension
                // Positive tension pulling negative moment arm creates positive torque
                (joint, -moment_arm * tension)
            })
            .collect()
    }

    /// Convert to a `JointCoupling` for use with the general coupling solver.
    ///
    /// Note: This loses the slack/tension information and treats the tendon
    /// as a rigid linear constraint.
    #[must_use]
    pub fn to_coupling(&self) -> JointCoupling {
        let mut coupling = JointCoupling::new(&self.name)
            .with_offset(self.target_length - self.rest_length)
            .with_baumgarte(self.baumgarte_factor)
            .with_enabled(self.enabled);

        for &(joint, moment_arm) in &self.joints {
            coupling = coupling.with_joint(joint, moment_arm);
        }

        coupling
    }

    /// Create a simple two-joint tendon (like a finger flexor).
    #[must_use]
    pub fn two_joint(
        name: impl Into<String>,
        proximal: JointId,
        proximal_arm: f64,
        distal: JointId,
        distal_arm: f64,
        rest_length: f64,
    ) -> Self {
        Self::new(name)
            .with_joint(proximal, proximal_arm)
            .with_joint(distal, distal_arm)
            .with_rest_length(rest_length)
    }

    /// Create a finger tendon with three joints (MCP, PIP, DIP).
    #[must_use]
    pub fn finger(
        name: impl Into<String>,
        mcp: JointId,
        pip: JointId,
        dip: JointId,
        moment_arms: (f64, f64, f64),
        rest_length: f64,
    ) -> Self {
        Self::new(name)
            .with_joint(mcp, moment_arms.0)
            .with_joint(pip, moment_arms.1)
            .with_joint(dip, moment_arms.2)
            .with_rest_length(rest_length)
    }
}

impl Default for TendonConstraint {
    fn default() -> Self {
        Self::new("tendon")
    }
}

// ============================================================================
// Tendon Network
// ============================================================================

/// A network of interconnected tendons.
///
/// Manages multiple tendons that may share joints and solves them together
/// for consistent force distribution.
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct TendonNetwork {
    /// The tendons in this network.
    tendons: Vec<TendonConstraint>,
}

impl TendonNetwork {
    /// Create a new empty tendon network.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a tendon to the network.
    pub fn add(&mut self, tendon: TendonConstraint) {
        self.tendons.push(tendon);
    }

    /// Add a tendon (builder pattern).
    #[must_use]
    pub fn with_tendon(mut self, tendon: TendonConstraint) -> Self {
        self.add(tendon);
        self
    }

    /// Get the number of tendons.
    #[must_use]
    pub fn len(&self) -> usize {
        self.tendons.len()
    }

    /// Check if empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.tendons.is_empty()
    }

    /// Get the tendons.
    #[must_use]
    pub fn tendons(&self) -> &[TendonConstraint] {
        &self.tendons
    }

    /// Get mutable access to tendons.
    pub fn tendons_mut(&mut self) -> &mut [TendonConstraint] {
        &mut self.tendons
    }

    /// Compute forces on all joints from all tendons.
    pub fn compute_all_forces<P, V>(
        &self,
        get_position: P,
        get_velocity: V,
        dt: f64,
    ) -> std::collections::HashMap<JointId, f64>
    where
        P: Fn(JointId) -> f64 + Copy,
        V: Fn(JointId) -> f64 + Copy,
    {
        let mut forces = std::collections::HashMap::new();

        for tendon in &self.tendons {
            for (joint, force) in tendon.compute_joint_forces(get_position, get_velocity, dt) {
                *forces.entry(joint).or_insert(0.0) += force;
            }
        }

        forces
    }

    /// Get all joints involved in any tendon.
    #[must_use]
    pub fn all_joints(&self) -> Vec<JointId> {
        let mut joints = std::collections::HashSet::new();
        for tendon in &self.tendons {
            for &(joint, _) in tendon.joints() {
                joints.insert(joint);
            }
        }
        joints.into_iter().collect()
    }

    /// Get tensions for all tendons.
    #[must_use]
    pub fn compute_tensions<P, V>(&self, get_position: P, get_velocity: V, dt: f64) -> Vec<f64>
    where
        P: Fn(JointId) -> f64 + Copy,
        V: Fn(JointId) -> f64 + Copy,
    {
        self.tendons
            .iter()
            .map(|t| t.compute_tension(get_position, get_velocity, dt))
            .collect()
    }
}

// ============================================================================
// Connect (Ball) Constraint
// ============================================================================

/// A connect (ball) equality constraint between two bodies.
///
/// This constraint enforces that an anchor point on body1 coincides with
/// the origin of body2 (or world if body2 is None), effectively creating
/// a ball-and-socket connection.
///
/// Unlike a ball joint, this is a pure positional constraint with no
/// rotational degrees of freedom coupling - both bodies can rotate
/// freely around the connection point.
///
/// # Constraint Formulation
///
/// The constraint enforces:
///
/// ```text
/// p1 + R1 * anchor - p2 = 0
/// ```
///
/// Where:
/// - `p1`, `p2` are the body positions
/// - `R1` is the rotation matrix of body1
/// - `anchor` is the local anchor point in body1's frame
///
/// This results in 3 scalar constraints (one per axis).
///
/// # Example
///
/// ```ignore
/// use sim_constraint::equality::ConnectConstraint;
/// use sim_types::BodyId;
/// use nalgebra::Vector3;
///
/// // Connect body1's tip to body2's origin
/// let constraint = ConnectConstraint::new(
///     BodyId::new(0),
///     BodyId::new(1),
///     Vector3::new(0.0, 0.0, 1.0),  // anchor at body1's +Z tip
/// );
///
/// // Connect body to world (fixed point constraint)
/// let world_constraint = ConnectConstraint::to_world(
///     BodyId::new(0),
///     Vector3::new(0.0, 0.0, 0.0),
/// );
/// ```
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ConnectConstraint {
    /// Constraint name (for debugging).
    name: String,

    /// First body (the body with the anchor point).
    body1: BodyId,

    /// Second body (None = world/ground).
    body2: Option<BodyId>,

    /// Anchor point in body1's local frame.
    anchor: Vector3<f64>,

    /// Baumgarte stabilization factor for position correction.
    baumgarte_factor: f64,

    /// Compliance (inverse stiffness). 0 = rigid constraint.
    compliance: f64,

    /// Damping coefficient for velocity-level constraint.
    damping: f64,

    /// Whether this constraint is enabled.
    enabled: bool,

    /// Solver reference parameters [timeconst, dampratio] or [stiffness, damping].
    solref: Option<[f64; 2]>,

    /// Solver impedance parameters [dmin, dmax, width, midpoint, power].
    solimp: Option<[f64; 5]>,
}

impl ConnectConstraint {
    /// Create a new connect constraint between two bodies.
    ///
    /// # Arguments
    ///
    /// * `body1` - The first body (with the anchor point)
    /// * `body2` - The second body to connect to
    /// * `anchor` - The anchor point in body1's local frame
    #[must_use]
    pub fn new(body1: BodyId, body2: BodyId, anchor: Vector3<f64>) -> Self {
        Self {
            name: String::new(),
            body1,
            body2: Some(body2),
            anchor,
            baumgarte_factor: 0.2,
            compliance: 0.0,
            damping: 0.0,
            enabled: true,
            solref: None,
            solimp: None,
        }
    }

    /// Create a connect constraint to the world frame.
    ///
    /// This constrains the anchor point on body1 to stay at the world origin.
    ///
    /// # Arguments
    ///
    /// * `body1` - The body with the anchor point
    /// * `anchor` - The anchor point in body1's local frame (constrained to world origin)
    #[must_use]
    pub fn to_world(body1: BodyId, anchor: Vector3<f64>) -> Self {
        Self {
            name: String::new(),
            body1,
            body2: None,
            anchor,
            baumgarte_factor: 0.2,
            compliance: 0.0,
            damping: 0.0,
            enabled: true,
            solref: None,
            solimp: None,
        }
    }

    /// Set the constraint name.
    #[must_use]
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = name.into();
        self
    }

    /// Set the Baumgarte stabilization factor.
    #[must_use]
    pub fn with_baumgarte(mut self, factor: f64) -> Self {
        self.baumgarte_factor = factor.clamp(0.0, 1.0);
        self
    }

    /// Set the compliance (softness) of the constraint.
    #[must_use]
    pub fn with_compliance(mut self, compliance: f64) -> Self {
        self.compliance = compliance.max(0.0);
        self
    }

    /// Set the damping coefficient.
    #[must_use]
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.damping = damping.max(0.0);
        self
    }

    /// Enable or disable the constraint.
    #[must_use]
    pub fn with_enabled(mut self, enabled: bool) -> Self {
        self.enabled = enabled;
        self
    }

    /// Set solver reference parameters.
    #[must_use]
    pub fn with_solref(mut self, solref: [f64; 2]) -> Self {
        self.solref = Some(solref);
        self
    }

    /// Set solver impedance parameters.
    #[must_use]
    pub fn with_solimp(mut self, solimp: [f64; 5]) -> Self {
        self.solimp = Some(solimp);
        self
    }

    /// Get the constraint name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get the first body ID.
    #[must_use]
    pub fn body1(&self) -> BodyId {
        self.body1
    }

    /// Get the second body ID (None if constrained to world).
    #[must_use]
    pub fn body2(&self) -> Option<BodyId> {
        self.body2
    }

    /// Get the anchor point in body1's frame.
    #[must_use]
    pub fn anchor(&self) -> Vector3<f64> {
        self.anchor
    }

    /// Check if the constraint is enabled.
    #[must_use]
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Enable the constraint.
    pub fn enable(&mut self) {
        self.enabled = true;
    }

    /// Disable the constraint.
    pub fn disable(&mut self) {
        self.enabled = false;
    }

    /// Get the Baumgarte stabilization factor.
    #[must_use]
    pub fn baumgarte_factor(&self) -> f64 {
        self.baumgarte_factor
    }

    /// Get the compliance.
    #[must_use]
    pub fn compliance(&self) -> f64 {
        self.compliance
    }

    /// Get the damping.
    #[must_use]
    pub fn damping(&self) -> f64 {
        self.damping
    }

    /// Get the solver reference parameters.
    #[must_use]
    pub fn solref(&self) -> Option<[f64; 2]> {
        self.solref
    }

    /// Get the solver impedance parameters.
    #[must_use]
    pub fn solimp(&self) -> Option<[f64; 5]> {
        self.solimp
    }

    /// Check if this constraint connects to the world frame.
    #[must_use]
    pub fn is_world_constraint(&self) -> bool {
        self.body2.is_none()
    }

    /// Get the number of constraint dimensions (always 3 for connect).
    #[must_use]
    pub fn dof(&self) -> usize {
        3
    }

    /// Compute the constraint error given body poses.
    ///
    /// # Arguments
    ///
    /// * `get_pose` - Function that returns `(position, rotation_matrix)` for a body
    ///
    /// # Returns
    ///
    /// The position error vector (3D).
    pub fn compute_error<F>(&self, get_pose: F) -> Vector3<f64>
    where
        F: Fn(BodyId) -> (Vector3<f64>, nalgebra::Matrix3<f64>),
    {
        if !self.enabled {
            return Vector3::zeros();
        }

        let (p1, r1) = get_pose(self.body1);
        let anchor_world = p1 + r1 * self.anchor;

        self.body2.map_or(anchor_world, |body2| {
            let (p2, _) = get_pose(body2);
            anchor_world - p2
        })
    }

    /// Compute the constraint velocity given body velocities.
    ///
    /// # Arguments
    ///
    /// * `get_velocity` - Function that returns `(linear_velocity, angular_velocity)` for a body
    /// * `get_pose` - Function that returns `(position, rotation_matrix)` for a body
    ///
    /// # Returns
    ///
    /// The velocity error vector (3D).
    pub fn compute_velocity<V, P>(&self, get_velocity: V, get_pose: P) -> Vector3<f64>
    where
        V: Fn(BodyId) -> (Vector3<f64>, Vector3<f64>),
        P: Fn(BodyId) -> (Vector3<f64>, nalgebra::Matrix3<f64>),
    {
        if !self.enabled {
            return Vector3::zeros();
        }

        let (v1, w1) = get_velocity(self.body1);
        let (_, r1) = get_pose(self.body1);

        // Velocity of anchor point: v1 + w1 × (R1 * anchor)
        let anchor_world_offset = r1 * self.anchor;
        let anchor_velocity = v1 + w1.cross(&anchor_world_offset);

        self.body2.map_or(anchor_velocity, |body2| {
            let (v2, _) = get_velocity(body2);
            anchor_velocity - v2
        })
    }

    /// Compute the constraint impulse needed to satisfy the constraint.
    ///
    /// Uses Baumgarte stabilization for position correction.
    ///
    /// # Arguments
    ///
    /// * `get_pose` - Function that returns `(position, rotation_matrix)` for a body
    /// * `get_velocity` - Function that returns `(linear_velocity, angular_velocity)` for a body
    /// * `dt` - Time step
    ///
    /// # Returns
    ///
    /// The impulse vector (3D) to apply.
    pub fn compute_correction<P, V>(&self, get_pose: P, get_velocity: V, dt: f64) -> Vector3<f64>
    where
        P: Fn(BodyId) -> (Vector3<f64>, nalgebra::Matrix3<f64>),
        V: Fn(BodyId) -> (Vector3<f64>, Vector3<f64>),
    {
        if !self.enabled {
            return Vector3::zeros();
        }

        let position_error = self.compute_error(&get_pose);
        let velocity_error = self.compute_velocity(&get_velocity, &get_pose);

        // Apply Baumgarte stabilization
        let bias = self.baumgarte_factor * position_error / dt;

        // Add damping
        // Note: Full implementation would compute proper effective mass
        // based on body inertias. Here we return the raw correction.
        -velocity_error - bias - self.damping * velocity_error
    }
}

impl Default for ConnectConstraint {
    fn default() -> Self {
        Self {
            name: "connect".to_string(),
            body1: BodyId::new(0),
            body2: None,
            anchor: Vector3::zeros(),
            baumgarte_factor: 0.2,
            compliance: 0.0,
            damping: 0.0,
            enabled: true,
            solref: None,
            solimp: None,
        }
    }
}

// ============================================================================
// Weld Constraint
// ============================================================================

/// Weld equality constraint that locks the relative pose between two bodies.
///
/// Unlike [`ConnectConstraint`] which only constrains position (3 DOF),
/// a weld constraint also locks orientation, enforcing a full 6 DOF rigid
/// connection between bodies.
///
/// This is commonly used in MJCF files to:
/// - Lock bodies together during assembly
/// - Create breakable joints (by toggling `enabled`)
/// - Define relative poses between reference frames
///
/// # Constraint Formulation
///
/// Position constraint: `p1 + R1 * anchor1 = p2 + R2 * anchor2`
/// Orientation constraint: `R1 * relative_rotation = R2`
///
/// # Example
///
/// ```
/// use sim_constraint::equality::WeldConstraint;
/// use sim_types::BodyId;
///
/// // Lock body 1 to body 2 with matching frames
/// let weld = WeldConstraint::new(BodyId::new(1), BodyId::new(2));
///
/// // Lock body 1 to world at a specific offset
/// let weld_to_world = WeldConstraint::to_world(BodyId::new(1))
///     .with_anchor(nalgebra::Vector3::new(0.0, 0.0, 1.0));
/// ```
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct WeldConstraint {
    /// Constraint name (for debugging).
    name: String,

    /// First body.
    body1: BodyId,

    /// Second body (None = world/ground).
    body2: Option<BodyId>,

    /// Anchor point in body1's local frame.
    anchor1: Vector3<f64>,

    /// Anchor point in body2's local frame (or world frame if body2 is None).
    anchor2: Vector3<f64>,

    /// Relative orientation from body1 to body2 (R2 = R1 * `relative_rotation`).
    relative_rotation: UnitQuaternion<f64>,

    /// Baumgarte stabilization factor for position correction.
    baumgarte_factor: f64,

    /// Compliance (inverse stiffness). 0 = rigid constraint.
    compliance: f64,

    /// Damping coefficient for velocity-level constraint.
    damping: f64,

    /// Whether this constraint is enabled.
    enabled: bool,

    /// Solver reference parameters [timeconst, dampratio] or [stiffness, damping].
    solref: Option<[f64; 2]>,

    /// Solver impedance parameters [dmin, dmax, width, midpoint, power].
    solimp: Option<[f64; 5]>,
}

impl WeldConstraint {
    /// Create a new weld constraint between two bodies.
    ///
    /// The constraint enforces that body1 and body2 maintain their current
    /// relative pose (both position and orientation).
    #[must_use]
    pub fn new(body1: BodyId, body2: BodyId) -> Self {
        Self {
            name: String::new(),
            body1,
            body2: Some(body2),
            anchor1: Vector3::zeros(),
            anchor2: Vector3::zeros(),
            relative_rotation: UnitQuaternion::identity(),
            baumgarte_factor: 0.2,
            compliance: 0.0,
            damping: 0.0,
            enabled: true,
            solref: None,
            solimp: None,
        }
    }

    /// Create a weld constraint to the world frame.
    ///
    /// This locks body1 to a fixed pose in world coordinates.
    #[must_use]
    pub fn to_world(body1: BodyId) -> Self {
        Self {
            name: String::new(),
            body1,
            body2: None,
            anchor1: Vector3::zeros(),
            anchor2: Vector3::zeros(),
            relative_rotation: UnitQuaternion::identity(),
            baumgarte_factor: 0.2,
            compliance: 0.0,
            damping: 0.0,
            enabled: true,
            solref: None,
            solimp: None,
        }
    }

    /// Set the constraint name.
    #[must_use]
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = name.into();
        self
    }

    /// Set the anchor point in body1's local frame.
    #[must_use]
    pub fn with_anchor1(mut self, anchor: Vector3<f64>) -> Self {
        self.anchor1 = anchor;
        self
    }

    /// Set the anchor point in body2's local frame (or world frame).
    #[must_use]
    pub fn with_anchor2(mut self, anchor: Vector3<f64>) -> Self {
        self.anchor2 = anchor;
        self
    }

    /// Set both anchors to the same point (for coincident attachment).
    #[must_use]
    pub fn with_anchor(mut self, anchor: Vector3<f64>) -> Self {
        self.anchor1 = anchor;
        self.anchor2 = anchor;
        self
    }

    /// Set the relative rotation between the bodies.
    #[must_use]
    pub fn with_relative_rotation(mut self, rotation: UnitQuaternion<f64>) -> Self {
        self.relative_rotation = rotation;
        self
    }

    /// Set the Baumgarte stabilization factor.
    #[must_use]
    pub fn with_baumgarte(mut self, factor: f64) -> Self {
        self.baumgarte_factor = factor.clamp(0.0, 1.0);
        self
    }

    /// Set the compliance (inverse stiffness).
    #[must_use]
    pub fn with_compliance(mut self, compliance: f64) -> Self {
        self.compliance = compliance.max(0.0);
        self
    }

    /// Set the damping coefficient.
    #[must_use]
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.damping = damping.max(0.0);
        self
    }

    /// Enable or disable the constraint.
    #[must_use]
    pub fn with_enabled(mut self, enabled: bool) -> Self {
        self.enabled = enabled;
        self
    }

    /// Set the solver reference parameters.
    #[must_use]
    pub fn with_solref(mut self, solref: [f64; 2]) -> Self {
        self.solref = Some(solref);
        self
    }

    /// Set the solver impedance parameters.
    #[must_use]
    pub fn with_solimp(mut self, solimp: [f64; 5]) -> Self {
        self.solimp = Some(solimp);
        self
    }

    /// Get the constraint name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get the first body.
    #[must_use]
    pub fn body1(&self) -> BodyId {
        self.body1
    }

    /// Get the second body (None if welded to world).
    #[must_use]
    pub fn body2(&self) -> Option<BodyId> {
        self.body2
    }

    /// Get the anchor point in body1's frame.
    #[must_use]
    pub fn anchor1(&self) -> Vector3<f64> {
        self.anchor1
    }

    /// Get the anchor point in body2's frame.
    #[must_use]
    pub fn anchor2(&self) -> Vector3<f64> {
        self.anchor2
    }

    /// Get the relative rotation.
    #[must_use]
    pub fn relative_rotation(&self) -> UnitQuaternion<f64> {
        self.relative_rotation
    }

    /// Check if the constraint is enabled.
    #[must_use]
    pub fn enabled(&self) -> bool {
        self.enabled
    }

    /// Check if this constraint is to the world frame.
    #[must_use]
    pub fn is_world_constraint(&self) -> bool {
        self.body2.is_none()
    }

    /// Get the solver reference parameters.
    #[must_use]
    pub fn solref(&self) -> Option<[f64; 2]> {
        self.solref
    }

    /// Get the solver impedance parameters.
    #[must_use]
    pub fn solimp(&self) -> Option<[f64; 5]> {
        self.solimp
    }

    /// Get the number of constraint DOFs (always 6 for weld).
    #[must_use]
    pub fn dof(&self) -> usize {
        6
    }

    /// Compute the position error (3D translation).
    pub fn compute_position_error<F>(&self, get_pose: F) -> Vector3<f64>
    where
        F: Fn(BodyId) -> (Vector3<f64>, nalgebra::Matrix3<f64>),
    {
        if !self.enabled {
            return Vector3::zeros();
        }

        let (p1, r1) = get_pose(self.body1);
        let anchor1_world = p1 + r1 * self.anchor1;

        self.body2.map_or_else(
            // World constraint: anchor1 should be at anchor2 in world frame
            || anchor1_world - self.anchor2,
            |body2| {
                let (p2, r2) = get_pose(body2);
                let anchor2_world = p2 + r2 * self.anchor2;
                anchor1_world - anchor2_world
            },
        )
    }

    /// Compute the orientation error (3D rotation vector).
    ///
    /// Returns the rotation axis scaled by angle that would align the frames.
    pub fn compute_orientation_error<F>(&self, get_rotation: F) -> Vector3<f64>
    where
        F: Fn(BodyId) -> UnitQuaternion<f64>,
    {
        if !self.enabled {
            return Vector3::zeros();
        }

        let r1 = get_rotation(self.body1);

        let r2 = self
            .body2
            .map_or_else(UnitQuaternion::identity, &get_rotation);

        // Expected: R1 * relative_rotation = R2
        // Error: R2.inverse() * R1 * relative_rotation
        let expected = r1 * self.relative_rotation;
        let error_quat = r2.inverse() * expected;

        // Convert to rotation vector (axis-angle)
        error_quat.scaled_axis()
    }
}

impl Default for WeldConstraint {
    fn default() -> Self {
        Self {
            name: "weld".to_string(),
            body1: BodyId::new(0),
            body2: None,
            anchor1: Vector3::zeros(),
            anchor2: Vector3::zeros(),
            relative_rotation: UnitQuaternion::identity(),
            baumgarte_factor: 0.2,
            compliance: 0.0,
            damping: 0.0,
            enabled: true,
            solref: None,
            solimp: None,
        }
    }
}

// ============================================================================
// Joint Position Constraint
// ============================================================================

/// Equality constraint that locks a joint to a specific position.
///
/// This constraint enforces that a joint stays at a target position,
/// useful for:
/// - Locking joints during specific motion phases
/// - Creating keyframe poses
/// - Implementing joint position equality (e.g., symmetric robot legs)
///
/// # Constraint Formulation
///
/// `q - target = 0`
///
/// For multi-joint equality, use [`JointCoupling`] instead.
///
/// # Example
///
/// ```
/// use sim_constraint::equality::JointPositionConstraint;
/// use sim_types::JointId;
///
/// // Lock joint 0 at position 0.5 radians
/// let lock = JointPositionConstraint::new(JointId::new(0), 0.5);
///
/// // Lock joint to zero (home position)
/// let home = JointPositionConstraint::at_zero(JointId::new(1));
/// ```
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct JointPositionConstraint {
    /// Constraint name (for debugging).
    name: String,

    /// The joint to constrain.
    joint: JointId,

    /// Target position.
    target: f64,

    /// Baumgarte stabilization factor.
    baumgarte_factor: f64,

    /// Compliance (inverse stiffness). 0 = rigid.
    compliance: f64,

    /// Damping coefficient.
    damping: f64,

    /// Whether this constraint is enabled.
    enabled: bool,

    /// Solver reference parameters.
    solref: Option<[f64; 2]>,

    /// Solver impedance parameters.
    solimp: Option<[f64; 5]>,
}

impl JointPositionConstraint {
    /// Create a new joint position constraint.
    #[must_use]
    pub fn new(joint: JointId, target: f64) -> Self {
        Self {
            name: String::new(),
            joint,
            target,
            baumgarte_factor: 0.2,
            compliance: 0.0,
            damping: 0.0,
            enabled: true,
            solref: None,
            solimp: None,
        }
    }

    /// Create a constraint that locks the joint at zero.
    #[must_use]
    pub fn at_zero(joint: JointId) -> Self {
        Self::new(joint, 0.0)
    }

    /// Set the constraint name.
    #[must_use]
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = name.into();
        self
    }

    /// Set the target position.
    #[must_use]
    pub fn with_target(mut self, target: f64) -> Self {
        self.target = target;
        self
    }

    /// Set the Baumgarte stabilization factor.
    #[must_use]
    pub fn with_baumgarte(mut self, factor: f64) -> Self {
        self.baumgarte_factor = factor.clamp(0.0, 1.0);
        self
    }

    /// Set the compliance.
    #[must_use]
    pub fn with_compliance(mut self, compliance: f64) -> Self {
        self.compliance = compliance.max(0.0);
        self
    }

    /// Set the damping.
    #[must_use]
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.damping = damping.max(0.0);
        self
    }

    /// Enable or disable the constraint.
    #[must_use]
    pub fn with_enabled(mut self, enabled: bool) -> Self {
        self.enabled = enabled;
        self
    }

    /// Set the solver reference parameters.
    #[must_use]
    pub fn with_solref(mut self, solref: [f64; 2]) -> Self {
        self.solref = Some(solref);
        self
    }

    /// Set the solver impedance parameters.
    #[must_use]
    pub fn with_solimp(mut self, solimp: [f64; 5]) -> Self {
        self.solimp = Some(solimp);
        self
    }

    /// Get the constraint name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get the constrained joint.
    #[must_use]
    pub fn joint(&self) -> JointId {
        self.joint
    }

    /// Get the target position.
    #[must_use]
    pub fn target(&self) -> f64 {
        self.target
    }

    /// Check if the constraint is enabled.
    #[must_use]
    pub fn enabled(&self) -> bool {
        self.enabled
    }

    /// Get the solver reference parameters.
    #[must_use]
    pub fn solref(&self) -> Option<[f64; 2]> {
        self.solref
    }

    /// Get the solver impedance parameters.
    #[must_use]
    pub fn solimp(&self) -> Option<[f64; 5]> {
        self.solimp
    }

    /// Get the number of constraint DOFs (always 1).
    #[must_use]
    pub fn dof(&self) -> usize {
        1
    }

    /// Compute the constraint error.
    pub fn compute_error<F>(&self, get_position: F) -> f64
    where
        F: Fn(JointId) -> f64,
    {
        if !self.enabled {
            return 0.0;
        }
        get_position(self.joint) - self.target
    }

    /// Compute the correction force.
    pub fn compute_correction<P, V>(&self, get_position: P, get_velocity: V, dt: f64) -> f64
    where
        P: Fn(JointId) -> f64,
        V: Fn(JointId) -> f64,
    {
        if !self.enabled {
            return 0.0;
        }

        let position_error = self.compute_error(&get_position);
        let velocity = get_velocity(self.joint);

        // Baumgarte stabilization
        let bias = self.baumgarte_factor * position_error / dt;

        -self.damping.mul_add(velocity, velocity + bias)
    }
}

impl Default for JointPositionConstraint {
    fn default() -> Self {
        Self {
            name: "joint_position".to_string(),
            joint: JointId::new(0),
            target: 0.0,
            baumgarte_factor: 0.2,
            compliance: 0.0,
            damping: 0.0,
            enabled: true,
            solref: None,
            solimp: None,
        }
    }
}

// ============================================================================
// Distance Constraint
// ============================================================================

/// Equality constraint that maintains a fixed distance between two points.
///
/// Unlike [`ConnectConstraint`] which constrains points to coincide (distance = 0),
/// this constraint enforces a specific non-zero distance, useful for:
/// - Rod/bar constraints
/// - Pendulum lengths
/// - Cable attachments with fixed length
///
/// # Constraint Formulation
///
/// `|| p1 + R1 * anchor1 - p2 - R2 * anchor2 || = distance`
///
/// # Example
///
/// ```
/// use sim_constraint::equality::DistanceConstraint;
/// use sim_types::BodyId;
///
/// // Rod of length 1.0 between two bodies
/// let rod = DistanceConstraint::new(BodyId::new(1), BodyId::new(2), 1.0);
///
/// // Pendulum hanging from world
/// let pendulum = DistanceConstraint::to_world(BodyId::new(1), 0.5);
/// ```
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct DistanceConstraint {
    /// Constraint name (for debugging).
    name: String,

    /// First body.
    body1: BodyId,

    /// Second body (None = world/ground).
    body2: Option<BodyId>,

    /// Anchor point in body1's local frame.
    anchor1: Vector3<f64>,

    /// Anchor point in body2's local frame (or world frame if body2 is None).
    anchor2: Vector3<f64>,

    /// Target distance between the anchor points.
    distance: f64,

    /// Baumgarte stabilization factor.
    baumgarte_factor: f64,

    /// Compliance (inverse stiffness). 0 = rigid.
    compliance: f64,

    /// Damping coefficient.
    damping: f64,

    /// Whether this constraint is enabled.
    enabled: bool,

    /// Solver reference parameters.
    solref: Option<[f64; 2]>,

    /// Solver impedance parameters.
    solimp: Option<[f64; 5]>,
}

impl DistanceConstraint {
    /// Create a new distance constraint between two bodies.
    #[must_use]
    pub fn new(body1: BodyId, body2: BodyId, distance: f64) -> Self {
        Self {
            name: String::new(),
            body1,
            body2: Some(body2),
            anchor1: Vector3::zeros(),
            anchor2: Vector3::zeros(),
            distance: distance.abs(),
            baumgarte_factor: 0.2,
            compliance: 0.0,
            damping: 0.0,
            enabled: true,
            solref: None,
            solimp: None,
        }
    }

    /// Create a distance constraint to the world frame.
    #[must_use]
    pub fn to_world(body1: BodyId, distance: f64) -> Self {
        Self {
            name: String::new(),
            body1,
            body2: None,
            anchor1: Vector3::zeros(),
            anchor2: Vector3::zeros(),
            distance: distance.abs(),
            baumgarte_factor: 0.2,
            compliance: 0.0,
            damping: 0.0,
            enabled: true,
            solref: None,
            solimp: None,
        }
    }

    /// Set the constraint name.
    #[must_use]
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = name.into();
        self
    }

    /// Set the anchor point in body1's local frame.
    #[must_use]
    pub fn with_anchor1(mut self, anchor: Vector3<f64>) -> Self {
        self.anchor1 = anchor;
        self
    }

    /// Set the anchor point in body2's local frame (or world frame).
    #[must_use]
    pub fn with_anchor2(mut self, anchor: Vector3<f64>) -> Self {
        self.anchor2 = anchor;
        self
    }

    /// Set the target distance.
    #[must_use]
    pub fn with_distance(mut self, distance: f64) -> Self {
        self.distance = distance.abs();
        self
    }

    /// Set the Baumgarte stabilization factor.
    #[must_use]
    pub fn with_baumgarte(mut self, factor: f64) -> Self {
        self.baumgarte_factor = factor.clamp(0.0, 1.0);
        self
    }

    /// Set the compliance.
    #[must_use]
    pub fn with_compliance(mut self, compliance: f64) -> Self {
        self.compliance = compliance.max(0.0);
        self
    }

    /// Set the damping.
    #[must_use]
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.damping = damping.max(0.0);
        self
    }

    /// Enable or disable the constraint.
    #[must_use]
    pub fn with_enabled(mut self, enabled: bool) -> Self {
        self.enabled = enabled;
        self
    }

    /// Set the solver reference parameters.
    #[must_use]
    pub fn with_solref(mut self, solref: [f64; 2]) -> Self {
        self.solref = Some(solref);
        self
    }

    /// Set the solver impedance parameters.
    #[must_use]
    pub fn with_solimp(mut self, solimp: [f64; 5]) -> Self {
        self.solimp = Some(solimp);
        self
    }

    /// Get the constraint name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get the first body.
    #[must_use]
    pub fn body1(&self) -> BodyId {
        self.body1
    }

    /// Get the second body (None if constrained to world).
    #[must_use]
    pub fn body2(&self) -> Option<BodyId> {
        self.body2
    }

    /// Get the anchor point in body1's frame.
    #[must_use]
    pub fn anchor1(&self) -> Vector3<f64> {
        self.anchor1
    }

    /// Get the anchor point in body2's frame.
    #[must_use]
    pub fn anchor2(&self) -> Vector3<f64> {
        self.anchor2
    }

    /// Get the target distance.
    #[must_use]
    pub fn distance(&self) -> f64 {
        self.distance
    }

    /// Check if the constraint is enabled.
    #[must_use]
    pub fn enabled(&self) -> bool {
        self.enabled
    }

    /// Check if this constraint is to the world frame.
    #[must_use]
    pub fn is_world_constraint(&self) -> bool {
        self.body2.is_none()
    }

    /// Get the solver reference parameters.
    #[must_use]
    pub fn solref(&self) -> Option<[f64; 2]> {
        self.solref
    }

    /// Get the solver impedance parameters.
    #[must_use]
    pub fn solimp(&self) -> Option<[f64; 5]> {
        self.solimp
    }

    /// Get the number of constraint DOFs (always 1 for distance).
    #[must_use]
    pub fn dof(&self) -> usize {
        1
    }

    /// Compute the constraint error (actual distance - target distance).
    pub fn compute_error<F>(&self, get_pose: F) -> f64
    where
        F: Fn(BodyId) -> (Vector3<f64>, nalgebra::Matrix3<f64>),
    {
        if !self.enabled {
            return 0.0;
        }

        let (p1, r1) = get_pose(self.body1);
        let anchor1_world = p1 + r1 * self.anchor1;

        let anchor2_world = self.body2.map_or(self.anchor2, |body2| {
            let (p2, r2) = get_pose(body2);
            p2 + r2 * self.anchor2
        });

        let diff = anchor1_world - anchor2_world;
        let actual_distance = diff.norm();

        actual_distance - self.distance
    }

    /// Compute the constraint direction (unit vector from anchor2 to anchor1).
    pub fn compute_direction<F>(&self, get_pose: F) -> Vector3<f64>
    where
        F: Fn(BodyId) -> (Vector3<f64>, nalgebra::Matrix3<f64>),
    {
        let (p1, r1) = get_pose(self.body1);
        let anchor1_world = p1 + r1 * self.anchor1;

        let anchor2_world = self.body2.map_or(self.anchor2, |body2| {
            let (p2, r2) = get_pose(body2);
            p2 + r2 * self.anchor2
        });

        let diff = anchor1_world - anchor2_world;
        let norm = diff.norm();

        if norm > 1e-10 {
            diff / norm
        } else {
            Vector3::z() // Arbitrary direction when points coincide
        }
    }
}

impl Default for DistanceConstraint {
    fn default() -> Self {
        Self {
            name: "distance".to_string(),
            body1: BodyId::new(0),
            body2: None,
            anchor1: Vector3::zeros(),
            anchor2: Vector3::zeros(),
            distance: 0.0,
            baumgarte_factor: 0.2,
            compliance: 0.0,
            damping: 0.0,
            enabled: true,
            solref: None,
            solimp: None,
        }
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_gear_coupling_constraint() {
        let coupling = JointCoupling::gear(JointId::new(0), JointId::new(1), 2.0);

        // Positions satisfying constraint: q0 = 1.0, q1 = 0.5 (since q0 - 2*q1 = 0)
        let get_pos = |id: JointId| {
            if id.raw() == 0 { 1.0 } else { 0.5 }
        };

        let error = coupling.compute_error(get_pos);
        assert_relative_eq!(error, 0.0, epsilon = 1e-10);

        // Positions violating constraint (both joints at 1.0, but joint 1 should be 0.5)
        #[allow(clippy::if_same_then_else, clippy::branches_sharing_code)]
        let get_pos_bad = |id: JointId| {
            if id.raw() == 0 { 1.0 } else { 1.0 }
        };

        let error = coupling.compute_error(get_pos_bad);
        assert_relative_eq!(error, -1.0, epsilon = 1e-10); // 1.0 - 2.0*1.0 = -1.0
    }

    #[test]
    fn test_mimic_coupling() {
        // Follower = 2 * leader + 0.5
        let coupling = JointCoupling::mimic(JointId::new(0), JointId::new(1), 2.0, 0.5);

        // Positions satisfying constraint: leader=1.0, follower=2.5 (since follower - 2*leader = 0.5)
        let get_pos = |id: JointId| {
            if id.raw() == 0 { 1.0 } else { 2.5 }
        };

        let error = coupling.compute_error(get_pos);
        assert_relative_eq!(error, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_differential_averaging() {
        let diff =
            DifferentialCoupling::averaging(JointId::new(0), JointId::new(1), JointId::new(2));

        // Output should be average of inputs
        let output = diff.compute_output(2.0, 4.0);
        assert_relative_eq!(output, 3.0, epsilon = 1e-10);
    }

    #[test]
    fn test_differential_difference() {
        let diff =
            DifferentialCoupling::difference(JointId::new(0), JointId::new(1), JointId::new(2));

        // Output should be difference of inputs
        let output = diff.compute_output(5.0, 2.0);
        assert_relative_eq!(output, 3.0, epsilon = 1e-10);
    }

    #[test]
    fn test_gear_reduction() {
        let gear = GearCoupling::reduction(JointId::new(0), JointId::new(1), 10.0);

        // 10:1 reduction means output is 10x slower
        assert_relative_eq!(gear.ratio(), 0.1, epsilon = 1e-10);

        let output = gear.output_from_input(10.0);
        assert_relative_eq!(output, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_gear_overdrive() {
        let gear = GearCoupling::overdrive(JointId::new(0), JointId::new(1), 3.0);

        // 3x overdrive means output is 3x faster
        assert_relative_eq!(gear.ratio(), 3.0, epsilon = 1e-10);

        let output = gear.output_from_input(2.0);
        assert_relative_eq!(output, 6.0, epsilon = 1e-10);
    }

    #[test]
    fn test_gear_reverse() {
        let gear = GearCoupling::reverse(JointId::new(0), JointId::new(1));

        assert_relative_eq!(gear.ratio(), -1.0, epsilon = 1e-10);

        let output = gear.output_from_input(5.0);
        assert_relative_eq!(output, -5.0, epsilon = 1e-10);
    }

    #[test]
    fn test_coupling_group() {
        let group = CouplingGroup::new()
            .with_gear(JointId::new(0), JointId::new(1), 2.0)
            .with_gear(JointId::new(1), JointId::new(2), 3.0);

        assert_eq!(group.len(), 2);

        let joints = group.all_joints();
        assert_eq!(joints.len(), 3);
    }

    #[test]
    fn test_coupling_velocity_constraint() {
        let coupling = JointCoupling::gear(JointId::new(0), JointId::new(1), 2.0);

        // Velocities satisfying constraint: v0 = 2.0, v1 = 1.0 (since v0 - 2*v1 = 0)
        let get_vel = |id: JointId| {
            if id.raw() == 0 { 2.0 } else { 1.0 }
        };

        let velocity_error = coupling.compute_velocity(get_vel);
        assert_relative_eq!(velocity_error, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_disabled_constraint() {
        let coupling =
            JointCoupling::gear(JointId::new(0), JointId::new(1), 2.0).with_enabled(false);

        let get_pos = |_: JointId| 100.0; // Would be huge error if enabled

        let error = coupling.compute_error(get_pos);
        assert_relative_eq!(error, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_parallel_coupling() {
        let coupling = JointCoupling::parallel(JointId::new(0), JointId::new(1));

        // Both joints at same position
        let get_pos = |_: JointId| 1.5;
        let error = coupling.compute_error(get_pos);
        assert_relative_eq!(error, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_anti_parallel_coupling() {
        let coupling = JointCoupling::anti_parallel(JointId::new(0), JointId::new(1));

        // Opposite positions
        let get_pos = |id: JointId| {
            if id.raw() == 0 { 1.0 } else { -1.0 }
        };
        let error = coupling.compute_error(get_pos);
        assert_relative_eq!(error, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_compute_joint_forces() {
        let coupling = JointCoupling::gear(JointId::new(0), JointId::new(1), 2.0);

        // Position error (joint 1 is behind)
        let get_pos = |id: JointId| {
            if id.raw() == 0 { 1.0 } else { 0.0 }
        }; // Error: 1.0 - 0 = 1.0
        let get_vel = |_: JointId| 0.0;

        let forces = coupling.compute_joint_forces(get_pos, get_vel, 0.01);

        // Should have forces on both joints
        assert_eq!(forces.len(), 2);

        // Forces should be opposite in sign (Newton's third law applies to constraint forces)
        // Joint 0 gets one direction, joint 1 gets opposite scaled by coefficient
    }

    #[test]
    fn test_weighted_differential() {
        let diff = DifferentialCoupling::weighted(
            JointId::new(0),
            JointId::new(1),
            JointId::new(2),
            0.7,
            0.3,
        );

        let output = diff.compute_output(10.0, 20.0);
        assert_relative_eq!(output, 0.7f64.mul_add(10.0, 0.3 * 20.0), epsilon = 1e-10);
    }

    #[test]
    fn test_coupling_enable_disable() {
        let mut coupling = JointCoupling::gear(JointId::new(0), JointId::new(1), 2.0);

        assert!(coupling.is_enabled());

        coupling.disable();
        assert!(!coupling.is_enabled());

        coupling.enable();
        assert!(coupling.is_enabled());
    }

    #[test]
    fn test_coupling_num_joints() {
        let coupling = JointCoupling::gear(JointId::new(0), JointId::new(1), 2.0);
        assert_eq!(coupling.num_joints(), 2);

        let coupling3 = JointCoupling::new("triple")
            .with_joint(JointId::new(0), 1.0)
            .with_joint(JointId::new(1), -1.0)
            .with_joint(JointId::new(2), 0.5);
        assert_eq!(coupling3.num_joints(), 3);
    }

    #[test]
    fn test_coupling_offset() {
        let coupling = JointCoupling::mimic(JointId::new(0), JointId::new(1), 1.0, 0.5);
        assert_relative_eq!(coupling.offset(), 0.5, epsilon = 1e-10);
    }

    #[test]
    fn test_coupling_with_baumgarte() {
        let coupling =
            JointCoupling::gear(JointId::new(0), JointId::new(1), 2.0).with_baumgarte(0.5);
        assert_relative_eq!(coupling.baumgarte_factor, 0.5, epsilon = 1e-10);
    }

    #[test]
    fn test_coupling_with_compliance() {
        let coupling =
            JointCoupling::gear(JointId::new(0), JointId::new(1), 2.0).with_compliance(0.01);
        assert_relative_eq!(coupling.compliance, 0.01, epsilon = 1e-10);
    }

    #[test]
    fn test_coupling_with_damping() {
        let coupling = JointCoupling::gear(JointId::new(0), JointId::new(1), 2.0).with_damping(0.1);
        assert_relative_eq!(coupling.damping, 0.1, epsilon = 1e-10);
    }

    #[test]
    fn test_coupling_default() {
        let coupling = JointCoupling::default();
        assert_eq!(coupling.name(), "coupling");
        assert!(coupling.is_enabled());
    }

    #[test]
    fn test_coupling_coefficients_accessor() {
        let coupling = JointCoupling::gear(JointId::new(0), JointId::new(1), 2.0);
        let coefficients = coupling.coefficients();
        assert_eq!(coefficients.len(), 2);
        assert_eq!(coefficients[0].joint, JointId::new(0));
        assert_relative_eq!(coefficients[0].coefficient, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_gear_coupling_input_from_output() {
        let gear = GearCoupling::new(JointId::new(0), JointId::new(1), 2.0);

        let input = gear.input_from_output(4.0);
        assert_relative_eq!(input, 2.0, epsilon = 1e-10);

        // Test with zero ratio
        let gear_zero = GearCoupling::new(JointId::new(0), JointId::new(1), 0.0);
        let input_zero = gear_zero.input_from_output(4.0);
        assert_relative_eq!(input_zero, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_gear_coupling_accessors() {
        let gear = GearCoupling::new(JointId::new(0), JointId::new(1), 3.0);

        assert_eq!(gear.input(), JointId::new(0));
        assert_eq!(gear.output(), JointId::new(1));
        assert_relative_eq!(gear.ratio(), 3.0, epsilon = 1e-10);
    }

    #[test]
    fn test_gear_coupling_with_compliance() {
        let gear = GearCoupling::new(JointId::new(0), JointId::new(1), 2.0).with_compliance(0.05);
        assert_relative_eq!(gear.coupling().compliance, 0.05, epsilon = 1e-10);
    }

    #[test]
    fn test_gear_coupling_with_damping() {
        let gear = GearCoupling::new(JointId::new(0), JointId::new(1), 2.0).with_damping(0.2);
        assert_relative_eq!(gear.coupling().damping, 0.2, epsilon = 1e-10);
    }

    #[test]
    fn test_gear_coupling_mut() {
        let mut gear = GearCoupling::new(JointId::new(0), JointId::new(1), 2.0);
        gear.coupling_mut().disable();
        assert!(!gear.coupling().is_enabled());
    }

    #[test]
    fn test_differential_accessors() {
        let diff = DifferentialCoupling::new(JointId::new(0), JointId::new(1), JointId::new(2));

        assert_eq!(diff.input1(), JointId::new(0));
        assert_eq!(diff.input2(), JointId::new(1));
        assert_eq!(diff.output(), JointId::new(2));

        let (c1, c2) = diff.coefficients();
        assert_relative_eq!(c1, 0.5, epsilon = 1e-10);
        assert_relative_eq!(c2, 0.5, epsilon = 1e-10);
    }

    #[test]
    fn test_differential_with_compliance() {
        let diff = DifferentialCoupling::new(JointId::new(0), JointId::new(1), JointId::new(2))
            .with_compliance(0.01);
        assert_relative_eq!(diff.coupling().compliance, 0.01, epsilon = 1e-10);
    }

    #[test]
    fn test_differential_with_damping() {
        let diff = DifferentialCoupling::new(JointId::new(0), JointId::new(1), JointId::new(2))
            .with_damping(0.5);
        assert_relative_eq!(diff.coupling().damping, 0.5, epsilon = 1e-10);
    }

    #[test]
    fn test_differential_coupling_mut() {
        let mut diff = DifferentialCoupling::new(JointId::new(0), JointId::new(1), JointId::new(2));
        diff.coupling_mut().disable();
        assert!(!diff.coupling().is_enabled());
    }

    #[test]
    fn test_coupling_group_add() {
        let mut group = CouplingGroup::new();
        assert!(group.is_empty());

        group.add(JointCoupling::gear(JointId::new(0), JointId::new(1), 2.0));
        assert_eq!(group.len(), 1);
        assert!(!group.is_empty());
    }

    #[test]
    fn test_coupling_group_couplings_accessor() {
        let group = CouplingGroup::new()
            .with_gear(JointId::new(0), JointId::new(1), 2.0)
            .with_gear(JointId::new(1), JointId::new(2), 3.0);

        let couplings = group.couplings();
        assert_eq!(couplings.len(), 2);
    }

    #[test]
    fn test_coupling_group_couplings_mut() {
        let mut group = CouplingGroup::new().with_gear(JointId::new(0), JointId::new(1), 2.0);

        group.couplings_mut()[0].disable();
        assert!(!group.couplings()[0].is_enabled());
    }

    #[test]
    fn test_coupling_group_with_mimic() {
        let group = CouplingGroup::new().with_mimic(JointId::new(0), JointId::new(1), 1.0, 0.5);
        assert_eq!(group.len(), 1);
    }

    #[test]
    fn test_coupling_group_compute_all_forces() {
        let group = CouplingGroup::new().with_gear(JointId::new(0), JointId::new(1), 2.0);

        // Position error
        let get_pos = |id: JointId| {
            if id.raw() == 0 { 1.0 } else { 0.0 }
        };
        let get_vel = |_: JointId| 0.0;

        let forces = group.compute_all_forces(get_pos, get_vel, 0.01);
        assert!(forces.contains_key(&JointId::new(0)));
        assert!(forces.contains_key(&JointId::new(1)));
    }

    #[test]
    fn test_compute_correction_empty() {
        let coupling = JointCoupling::new("empty");
        let correction = coupling.compute_correction(|_| 0.0, |_| 0.0, 0.01);
        assert_relative_eq!(correction, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_compute_velocity_disabled() {
        let coupling =
            JointCoupling::gear(JointId::new(0), JointId::new(1), 2.0).with_enabled(false);
        let velocity = coupling.compute_velocity(|_| 1.0);
        assert_relative_eq!(velocity, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_compute_correction_disabled() {
        let coupling =
            JointCoupling::gear(JointId::new(0), JointId::new(1), 2.0).with_enabled(false);
        let correction = coupling.compute_correction(|_| 1.0, |_| 1.0, 0.01);
        assert_relative_eq!(correction, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_coupling_coefficient_new() {
        let coef = CouplingCoefficient::new(JointId::new(5), 2.5);
        assert_eq!(coef.joint, JointId::new(5));
        assert_relative_eq!(coef.coefficient, 2.5, epsilon = 1e-10);
    }

    #[test]
    fn test_coupling_baumgarte_clamping() {
        // Test clamping to 1.0
        let coupling = JointCoupling::new("test").with_baumgarte(2.0);
        assert_relative_eq!(coupling.baumgarte_factor, 1.0, epsilon = 1e-10);

        // Test clamping to 0.0
        let coupling2 = JointCoupling::new("test").with_baumgarte(-1.0);
        assert_relative_eq!(coupling2.baumgarte_factor, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_coupling_compliance_non_negative() {
        let coupling = JointCoupling::new("test").with_compliance(-0.5);
        assert_relative_eq!(coupling.compliance, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_coupling_damping_non_negative() {
        let coupling = JointCoupling::new("test").with_damping(-0.5);
        assert_relative_eq!(coupling.damping, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_compute_correction_zero_effective_mass() {
        // Create a coupling with zero coefficients
        let coupling = JointCoupling::new("zero_coef")
            .with_joint(JointId::new(0), 0.0)
            .with_joint(JointId::new(1), 0.0);

        let correction = coupling.compute_correction(|_| 1.0, |_| 1.0, 0.01);
        // Should return 0 because effective_mass_inv is 0
        assert_relative_eq!(correction, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_compute_correction_with_damping() {
        // Test that damping affects the correction
        let coupling = JointCoupling::gear(JointId::new(0), JointId::new(1), 2.0).with_damping(0.5);

        // Position error
        let get_pos = |id: JointId| if id.raw() == 0 { 1.0 } else { 0.0 };
        let get_vel = |id: JointId| if id.raw() == 0 { 0.5 } else { 0.0 };

        let correction = coupling.compute_correction(get_pos, get_vel, 0.01);
        // Should return non-zero correction
        assert!(correction.abs() > 0.0);
    }

    // ========================================================================
    // Tendon Constraint Tests
    // ========================================================================

    #[test]
    fn test_tendon_constraint_creation() {
        let tendon = TendonConstraint::new("test_tendon")
            .with_joint(JointId::new(0), 0.01)
            .with_joint(JointId::new(1), 0.008)
            .with_rest_length(0.1);

        assert_eq!(tendon.name(), "test_tendon");
        assert_eq!(tendon.joints().len(), 2);
        assert_relative_eq!(tendon.rest_length(), 0.1, epsilon = 1e-10);
        assert!(tendon.is_enabled());
    }

    #[test]
    fn test_tendon_compute_length() {
        let tendon = TendonConstraint::new("test")
            .with_joint(JointId::new(0), 0.01)  // 10mm moment arm
            .with_joint(JointId::new(1), 0.02)  // 20mm moment arm
            .with_rest_length(0.1); // 100mm rest

        // At zero position, length = rest_length
        let get_pos_zero = |_: JointId| 0.0;
        assert_relative_eq!(tendon.compute_length(get_pos_zero), 0.1, epsilon = 1e-10);

        // At 1 rad each, length = 0.1 + 0.01*1 + 0.02*1 = 0.13
        let get_pos = |_: JointId| 1.0;
        assert_relative_eq!(tendon.compute_length(get_pos), 0.13, epsilon = 1e-10);
    }

    #[test]
    fn test_tendon_slack_condition() {
        let tendon = TendonConstraint::new("test")
            .with_joint(JointId::new(0), 0.01)
            .with_rest_length(0.1)
            .with_can_slack(true);

        // Tendon is slack when current length > target length
        // At q=0, length = 0.1 = target, so taut
        let get_pos_zero = |_: JointId| 0.0;
        assert!(tendon.is_taut(get_pos_zero));

        // At q=1, length = 0.1 + 0.01 = 0.11 > 0.1, so slack
        let get_pos_one = |_: JointId| 1.0;
        assert!(!tendon.is_taut(get_pos_one));

        // Error should be 0 when slack (since can_slack = true)
        let error = tendon.compute_error(get_pos_one);
        assert_relative_eq!(error, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_tendon_no_slack() {
        let tendon = TendonConstraint::new("test")
            .with_joint(JointId::new(0), 0.01)
            .with_rest_length(0.1)
            .with_can_slack(false); // Rigid rod behavior

        // At q=1, length = 0.11, error = 0.01 (stretched)
        let get_pos = |_: JointId| 1.0;
        let error = tendon.compute_error(get_pos);
        assert_relative_eq!(error, 0.01, epsilon = 1e-10);
    }

    #[test]
    fn test_tendon_elastic() {
        let tendon = TendonConstraint::new("elastic")
            .with_joint(JointId::new(0), 0.01)
            .with_rest_length(0.1)
            .with_stiffness(1000.0)  // 1000 N/m
            .with_can_slack(false);

        // At q=-1, length = 0.1 - 0.01 = 0.09 < 0.1 (compressed by 0.01m)
        let get_pos = |_: JointId| -1.0;
        let get_vel = |_: JointId| 0.0;

        let tension = tendon.compute_tension(get_pos, get_vel, 0.01);
        // stretch = 0.01 (target - current = 0.1 - 0.09)
        // tension = k * stretch = 1000 * 0.01 = 10 N
        assert_relative_eq!(tension, 10.0, epsilon = 1e-6);
    }

    #[test]
    fn test_tendon_joint_forces() {
        let tendon = TendonConstraint::new("test")
            .with_joint(JointId::new(0), 0.01)
            .with_joint(JointId::new(1), 0.02)
            .with_rest_length(0.1)
            .with_stiffness(1000.0)
            .with_can_slack(false);

        // Stretched tendon
        let get_pos = |_: JointId| -0.5;
        let get_vel = |_: JointId| 0.0;

        let forces = tendon.compute_joint_forces(get_pos, get_vel, 0.01);
        assert_eq!(forces.len(), 2);

        // Forces should be non-zero
        assert!(forces[0].1.abs() > 0.0);
        assert!(forces[1].1.abs() > 0.0);

        // Force ratio should match moment arm ratio
        // Joint 1 has 2x moment arm, so should have 2x force
        assert_relative_eq!(forces[1].1 / forces[0].1, 2.0, epsilon = 1e-6);
    }

    #[test]
    fn test_tendon_two_joint_preset() {
        let tendon = TendonConstraint::two_joint(
            "finger_flexor",
            JointId::new(0),
            0.01,
            JointId::new(1),
            0.008,
            0.1,
        );

        assert_eq!(tendon.joints().len(), 2);
        assert_eq!(tendon.joints()[0].0, JointId::new(0));
        assert_relative_eq!(tendon.joints()[0].1, 0.01, epsilon = 1e-10);
    }

    #[test]
    fn test_tendon_finger_preset() {
        let tendon = TendonConstraint::finger(
            "fdp",
            JointId::new(0),      // MCP
            JointId::new(1),      // PIP
            JointId::new(2),      // DIP
            (0.01, 0.008, 0.005), // moment arms
            0.15,                 // rest length
        );

        assert_eq!(tendon.joints().len(), 3);
        assert_relative_eq!(tendon.rest_length(), 0.15, epsilon = 1e-10);
    }

    #[test]
    fn test_tendon_to_coupling() {
        let tendon = TendonConstraint::new("test")
            .with_joint(JointId::new(0), 0.01)
            .with_joint(JointId::new(1), 0.02)
            .with_rest_length(0.1)
            .with_target_length(0.12);

        let coupling = tendon.to_coupling();

        assert_eq!(coupling.coefficients().len(), 2);
        assert_relative_eq!(coupling.offset(), 0.02, epsilon = 1e-10); // 0.12 - 0.1
    }

    #[test]
    fn test_tendon_enable_disable() {
        let mut tendon = TendonConstraint::new("test")
            .with_joint(JointId::new(0), 0.01)
            .with_rest_length(0.1);

        assert!(tendon.is_enabled());

        tendon.disable();
        assert!(!tendon.is_enabled());

        // Error should be 0 when disabled
        let error = tendon.compute_error(|_| -1.0);
        assert_relative_eq!(error, 0.0, epsilon = 1e-10);

        tendon.enable();
        assert!(tendon.is_enabled());
    }

    #[test]
    fn test_tendon_max_tension() {
        let tendon = TendonConstraint::new("test")
            .with_joint(JointId::new(0), 0.01)
            .with_rest_length(0.1)
            .with_stiffness(100_000.0)  // Very stiff
            .with_max_tension(100.0)   // Clamp to 100N
            .with_can_slack(false);

        // Large stretch
        let get_pos = |_: JointId| -10.0;
        let get_vel = |_: JointId| 0.0;

        let tension = tendon.compute_tension(get_pos, get_vel, 0.01);
        // Should be clamped to max
        assert_relative_eq!(tension, 100.0, epsilon = 1e-6);
    }

    #[test]
    fn test_tendon_network() {
        let network = TendonNetwork::new()
            .with_tendon(
                TendonConstraint::new("t1")
                    .with_joint(JointId::new(0), 0.01)
                    .with_rest_length(0.1),
            )
            .with_tendon(
                TendonConstraint::new("t2")
                    .with_joint(JointId::new(1), 0.02)
                    .with_rest_length(0.15),
            );

        assert_eq!(network.len(), 2);
        assert!(!network.is_empty());

        let joints = network.all_joints();
        assert_eq!(joints.len(), 2);
    }

    #[test]
    fn test_tendon_network_forces() {
        let network = TendonNetwork::new()
            .with_tendon(
                TendonConstraint::new("t1")
                    .with_joint(JointId::new(0), 0.01)
                    .with_rest_length(0.1)
                    .with_stiffness(1000.0)
                    .with_can_slack(false),
            )
            .with_tendon(
                TendonConstraint::new("t2")
                .with_joint(JointId::new(0), 0.02)  // Same joint, different tendon
                .with_rest_length(0.1)
                .with_stiffness(1000.0)
                .with_can_slack(false),
            );

        let get_pos = |_: JointId| -0.5;
        let get_vel = |_: JointId| 0.0;

        let forces = network.compute_all_forces(get_pos, get_vel, 0.01);

        // Joint 0 should have forces from both tendons combined
        assert!(forces.contains_key(&JointId::new(0)));
        assert!(forces[&JointId::new(0)].abs() > 0.0);
    }

    #[test]
    fn test_tendon_network_tensions() {
        let network = TendonNetwork::new()
            .with_tendon(
                TendonConstraint::new("t1")
                    .with_joint(JointId::new(0), 0.01)
                    .with_rest_length(0.1)
                    .with_stiffness(1000.0)
                    .with_can_slack(false),
            )
            .with_tendon(
                TendonConstraint::new("t2")
                    .with_joint(JointId::new(1), 0.02)
                    .with_rest_length(0.1)
                    .with_stiffness(2000.0)
                    .with_can_slack(false),
            );

        let get_pos = |_: JointId| -0.5;
        let get_vel = |_: JointId| 0.0;

        let tensions = network.compute_tensions(get_pos, get_vel, 0.01);
        assert_eq!(tensions.len(), 2);
        assert!(tensions[0] > 0.0);
        assert!(tensions[1] > 0.0);
    }

    #[test]
    fn test_tendon_default() {
        let tendon = TendonConstraint::default();
        assert_eq!(tendon.name(), "tendon");
        assert!(tendon.joints().is_empty());
    }

    #[test]
    fn test_tendon_config_accessors() {
        let tendon = TendonConstraint::new("test")
            .with_stiffness(500.0)
            .with_target_length(0.15);

        assert_relative_eq!(tendon.stiffness(), 500.0, epsilon = 1e-10);
        assert_relative_eq!(tendon.target_length(), 0.15, epsilon = 1e-10);
    }

    #[test]
    fn test_tendon_network_tendons_mut() {
        let mut network = TendonNetwork::new().with_tendon(
            TendonConstraint::new("t1")
                .with_joint(JointId::new(0), 0.01)
                .with_rest_length(0.1),
        );

        network.tendons_mut()[0].disable();
        assert!(!network.tendons()[0].is_enabled());
    }

    #[test]
    fn test_tendon_length_rate() {
        let tendon = TendonConstraint::new("test")
            .with_joint(JointId::new(0), 0.01)
            .with_joint(JointId::new(1), 0.02)
            .with_rest_length(0.1);

        // Velocities: joint 0 at 1 rad/s, joint 1 at 2 rad/s
        let get_vel = |id: JointId| if id.raw() == 0 { 1.0 } else { 2.0 };

        // Rate = 0.01 * 1.0 + 0.02 * 2.0 = 0.05 m/s
        let rate = tendon.compute_length_rate(get_vel);
        assert_relative_eq!(rate, 0.05, epsilon = 1e-10);
    }

    #[test]
    fn test_tendon_disabled_tension() {
        let tendon = TendonConstraint::new("test")
            .with_joint(JointId::new(0), 0.01)
            .with_rest_length(0.1)
            .with_stiffness(1000.0)
            .with_enabled(false);

        let tension = tendon.compute_tension(|_| -1.0, |_| 0.0, 0.01);
        assert_relative_eq!(tension, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_tendon_empty_joints_tension() {
        let tendon = TendonConstraint::new("empty")
            .with_rest_length(0.1)
            .with_stiffness(1000.0);

        let tension = tendon.compute_tension(|_| -1.0, |_| 0.0, 0.01);
        assert_relative_eq!(tension, 0.0, epsilon = 1e-10);
    }

    // ========================================================================
    // Connect Constraint Tests
    // ========================================================================

    #[test]
    fn test_connect_constraint_creation() {
        let constraint =
            ConnectConstraint::new(BodyId::new(0), BodyId::new(1), Vector3::new(0.0, 0.0, 1.0));

        assert_eq!(constraint.body1(), BodyId::new(0));
        assert_eq!(constraint.body2(), Some(BodyId::new(1)));
        assert_relative_eq!(constraint.anchor().z, 1.0, epsilon = 1e-10);
        assert!(constraint.is_enabled());
        assert!(!constraint.is_world_constraint());
        assert_eq!(constraint.dof(), 3);
    }

    #[test]
    fn test_connect_constraint_to_world() {
        let constraint = ConnectConstraint::to_world(BodyId::new(0), Vector3::new(1.0, 0.0, 0.0));

        assert_eq!(constraint.body1(), BodyId::new(0));
        assert!(constraint.body2().is_none());
        assert!(constraint.is_world_constraint());
    }

    #[test]
    fn test_connect_constraint_with_name() {
        let constraint = ConnectConstraint::new(BodyId::new(0), BodyId::new(1), Vector3::zeros())
            .with_name("ball_joint");

        assert_eq!(constraint.name(), "ball_joint");
    }

    #[test]
    fn test_connect_constraint_builders() {
        let constraint = ConnectConstraint::new(BodyId::new(0), BodyId::new(1), Vector3::zeros())
            .with_baumgarte(0.5)
            .with_compliance(0.01)
            .with_damping(0.1)
            .with_solref([0.02, 1.0])
            .with_solimp([0.9, 0.95, 0.001, 0.5, 2.0]);

        assert_relative_eq!(constraint.baumgarte_factor(), 0.5, epsilon = 1e-10);
        assert_relative_eq!(constraint.compliance(), 0.01, epsilon = 1e-10);
        assert_relative_eq!(constraint.damping(), 0.1, epsilon = 1e-10);

        let solref = constraint.solref().unwrap();
        assert_relative_eq!(solref[0], 0.02, epsilon = 1e-10);

        let solimp = constraint.solimp().unwrap();
        assert_relative_eq!(solimp[0], 0.9, epsilon = 1e-10);
    }

    #[test]
    fn test_connect_constraint_enable_disable() {
        let mut constraint =
            ConnectConstraint::new(BodyId::new(0), BodyId::new(1), Vector3::zeros());

        assert!(constraint.is_enabled());

        constraint.disable();
        assert!(!constraint.is_enabled());

        constraint.enable();
        assert!(constraint.is_enabled());
    }

    #[test]
    fn test_connect_constraint_disabled() {
        let constraint = ConnectConstraint::new(BodyId::new(0), BodyId::new(1), Vector3::zeros())
            .with_enabled(false);

        // Should return zero error when disabled
        let get_pose = |_: BodyId| (Vector3::new(10.0, 0.0, 0.0), nalgebra::Matrix3::identity());
        let error = constraint.compute_error(get_pose);
        assert_relative_eq!(error.norm(), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_connect_constraint_error_computation() {
        let constraint = ConnectConstraint::new(
            BodyId::new(0),
            BodyId::new(1),
            Vector3::new(0.0, 0.0, 1.0), // Anchor at +Z
        );

        // Body 0 at origin, body 1 at (0, 0, 2)
        // Anchor in world = (0, 0, 1), so error = (0, 0, 1) - (0, 0, 2) = (0, 0, -1)
        let get_pose = |id: BodyId| {
            if id.raw() == 0 {
                (Vector3::zeros(), nalgebra::Matrix3::identity())
            } else {
                (Vector3::new(0.0, 0.0, 2.0), nalgebra::Matrix3::identity())
            }
        };

        let error = constraint.compute_error(get_pose);
        assert_relative_eq!(error.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(error.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(error.z, -1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_connect_constraint_world_error() {
        let constraint = ConnectConstraint::to_world(
            BodyId::new(0),
            Vector3::new(0.5, 0.0, 0.0), // Anchor at +X
        );

        // Body 0 at (1, 0, 0), anchor in world = (1.5, 0, 0)
        // Error = anchor - world_origin = (1.5, 0, 0)
        let get_pose = |_: BodyId| (Vector3::new(1.0, 0.0, 0.0), nalgebra::Matrix3::identity());

        let error = constraint.compute_error(get_pose);
        assert_relative_eq!(error.x, 1.5, epsilon = 1e-10);
        assert_relative_eq!(error.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(error.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_connect_constraint_satisfied() {
        let constraint =
            ConnectConstraint::new(BodyId::new(0), BodyId::new(1), Vector3::new(0.0, 0.0, 1.0));

        // Body 0 at origin with anchor at (0,0,1), body 1 at (0,0,1)
        // Constraint is satisfied
        let get_pose = |id: BodyId| {
            if id.raw() == 0 {
                (Vector3::zeros(), nalgebra::Matrix3::identity())
            } else {
                (Vector3::new(0.0, 0.0, 1.0), nalgebra::Matrix3::identity())
            }
        };

        let error = constraint.compute_error(get_pose);
        assert_relative_eq!(error.norm(), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_connect_constraint_velocity() {
        let constraint = ConnectConstraint::new(BodyId::new(0), BodyId::new(1), Vector3::zeros());

        // Body 0 moving at (1, 0, 0), body 1 stationary
        let get_velocity = |id: BodyId| {
            if id.raw() == 0 {
                (Vector3::new(1.0, 0.0, 0.0), Vector3::zeros())
            } else {
                (Vector3::zeros(), Vector3::zeros())
            }
        };
        let get_pose = |_: BodyId| (Vector3::zeros(), nalgebra::Matrix3::identity());

        let velocity = constraint.compute_velocity(get_velocity, get_pose);
        assert_relative_eq!(velocity.x, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_connect_constraint_default() {
        let constraint = ConnectConstraint::default();
        assert_eq!(constraint.name(), "connect");
        assert!(constraint.is_enabled());
        assert!(constraint.is_world_constraint());
    }

    #[test]
    fn test_connect_constraint_baumgarte_clamping() {
        // Test clamping to 1.0
        let constraint = ConnectConstraint::default().with_baumgarte(2.0);
        assert_relative_eq!(constraint.baumgarte_factor(), 1.0, epsilon = 1e-10);

        // Test clamping to 0.0
        let constraint2 = ConnectConstraint::default().with_baumgarte(-1.0);
        assert_relative_eq!(constraint2.baumgarte_factor(), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_connect_constraint_compliance_non_negative() {
        let constraint = ConnectConstraint::default().with_compliance(-0.5);
        assert_relative_eq!(constraint.compliance(), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_connect_constraint_damping_non_negative() {
        let constraint = ConnectConstraint::default().with_damping(-0.5);
        assert_relative_eq!(constraint.damping(), 0.0, epsilon = 1e-10);
    }
}
