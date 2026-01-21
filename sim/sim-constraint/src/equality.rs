//! Equality constraints for joint coupling.
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

use sim_types::JointId;

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
// Tests
// ============================================================================

#[cfg(test)]
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

        // Positions violating constraint
        let get_pos_bad = |id: JointId| {
            if id.raw() == 0 { 1.0 } else { 1.0 }
        }; // Should be 0.5

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
        assert_relative_eq!(output, 0.7 * 10.0 + 0.3 * 20.0, epsilon = 1e-10);
    }
}
