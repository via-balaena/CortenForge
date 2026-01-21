//! Actuator types for robot control.
//!
//! This module provides various actuator types beyond simple joint motors:
//!
//! - [`IntegratedVelocityActuator`]: Integrates velocity commands for smooth position changes
//! - [`PneumaticCylinderActuator`]: Air/hydraulic cylinder for soft robotics
//! - [`AdhesionActuator`]: Adhesive gripping for climbing/gripping robots
//! - [`CustomActuator`]: User-defined actuator behavior via trait objects
//!
//! # Actuator Trait
//!
//! All actuators implement the [`Actuator`] trait which defines a common interface
//! for computing forces/torques given the current state and a command signal.
//!
//! # Example
//!
//! ```
//! use sim_constraint::actuator::{Actuator, IntegratedVelocityActuator};
//!
//! // Create an integrated velocity actuator
//! let mut actuator = IntegratedVelocityActuator::new(10.0, 100.0)  // max vel, max force
//!     .with_position_limits(-1.5, 1.5);
//!
//! // Update the actuator state
//! let dt = 0.001;
//! let current_position = 0.0;
//! let current_velocity = 0.0;
//!
//! // Set a velocity command (normalized -1 to 1)
//! actuator.set_command(0.5);  // 50% of max velocity
//!
//! // Compute force output
//! let force = actuator.compute_force(current_position, current_velocity, dt);
//! ```

use nalgebra::Vector3;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

// ============================================================================
// Actuator Trait
// ============================================================================

/// Common interface for all actuator types.
///
/// Actuators convert a control command (typically from an RL policy or controller)
/// into a force or torque to be applied at a joint.
pub trait Actuator: Send + Sync {
    /// Get the actuator name for debugging.
    fn name(&self) -> &str;

    /// Set the control command (typically normalized -1 to 1).
    fn set_command(&mut self, command: f64);

    /// Get the current command.
    fn command(&self) -> f64;

    /// Compute the force/torque to apply.
    ///
    /// # Arguments
    ///
    /// * `position` - Current joint position (rad or m)
    /// * `velocity` - Current joint velocity (rad/s or m/s)
    /// * `dt` - Time step (seconds)
    ///
    /// # Returns
    ///
    /// The force (N) or torque (Nm) to apply at the joint.
    fn compute_force(&mut self, position: f64, velocity: f64, dt: f64) -> f64;

    /// Reset the actuator to its initial state.
    fn reset(&mut self);

    /// Get the maximum force/torque this actuator can produce.
    fn max_force(&self) -> f64;

    /// Check if this actuator has internal state that evolves over time.
    fn has_dynamics(&self) -> bool {
        false
    }
}

// ============================================================================
// Integrated Velocity Actuator
// ============================================================================

/// An actuator that integrates velocity commands for smooth position control.
///
/// This is `MuJoCo`'s `intvelocity` actuator type. It maintains an internal
/// position target that integrates the velocity command, then applies a
/// PD controller to track that target.
///
/// This provides smooth motion even with discontinuous velocity commands,
/// which is particularly useful for RL policies.
///
/// # Dynamics
///
/// ```text
/// target_position += command * max_velocity * dt
/// force = kp * (target - position) + kd * (0 - velocity)
/// ```
///
/// The target position is clamped to any configured limits.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct IntegratedVelocityActuator {
    /// Actuator name.
    name: String,

    /// Maximum velocity (rad/s or m/s).
    max_velocity: f64,

    /// Maximum force/torque (Nm or N).
    max_force: f64,

    /// Proportional gain (stiffness).
    kp: f64,

    /// Derivative gain (damping).
    kd: f64,

    /// Current velocity command (-1 to 1).
    command: f64,

    /// Internal target position (integrated from velocity).
    target_position: f64,

    /// Lower position limit (if any).
    position_lower: Option<f64>,

    /// Upper position limit (if any).
    position_upper: Option<f64>,

    /// Whether the actuator is enabled.
    enabled: bool,
}

impl IntegratedVelocityActuator {
    /// Create a new integrated velocity actuator.
    ///
    /// # Arguments
    ///
    /// * `max_velocity` - Maximum velocity magnitude (rad/s or m/s)
    /// * `max_force` - Maximum force/torque magnitude (Nm or N)
    #[must_use]
    pub fn new(max_velocity: f64, max_force: f64) -> Self {
        let kp = max_force / 0.1; // Default: max force at 0.1 rad/m error
        Self {
            name: String::from("integrated_velocity"),
            max_velocity: max_velocity.abs(),
            max_force: max_force.abs(),
            kp,
            kd: 2.0 * kp.sqrt(), // Critical damping
            command: 0.0,
            target_position: 0.0,
            position_lower: None,
            position_upper: None,
            enabled: true,
        }
    }

    /// Set the actuator name.
    #[must_use]
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = name.into();
        self
    }

    /// Set the PD gains.
    #[must_use]
    pub fn with_gains(mut self, kp: f64, kd: f64) -> Self {
        self.kp = kp.abs();
        self.kd = kd.abs();
        self
    }

    /// Set position limits.
    #[must_use]
    pub fn with_position_limits(mut self, lower: f64, upper: f64) -> Self {
        self.position_lower = Some(lower.min(upper));
        self.position_upper = Some(lower.max(upper));
        self
    }

    /// Set the initial target position.
    #[must_use]
    pub fn with_initial_target(mut self, target: f64) -> Self {
        self.target_position = self.clamp_to_limits(target);
        self
    }

    /// Enable or disable the actuator.
    #[must_use]
    pub fn with_enabled(mut self, enabled: bool) -> Self {
        self.enabled = enabled;
        self
    }

    /// Get the current target position.
    #[must_use]
    pub fn target_position(&self) -> f64 {
        self.target_position
    }

    /// Directly set the target position (bypassing integration).
    pub fn set_target_position(&mut self, target: f64) {
        self.target_position = self.clamp_to_limits(target);
    }

    /// Get the maximum velocity.
    #[must_use]
    pub fn max_velocity(&self) -> f64 {
        self.max_velocity
    }

    /// Clamp a value to configured limits.
    fn clamp_to_limits(&self, value: f64) -> f64 {
        let mut v = value;
        if let Some(lower) = self.position_lower {
            v = v.max(lower);
        }
        if let Some(upper) = self.position_upper {
            v = v.min(upper);
        }
        v
    }
}

impl Actuator for IntegratedVelocityActuator {
    fn name(&self) -> &str {
        &self.name
    }

    fn set_command(&mut self, command: f64) {
        self.command = command.clamp(-1.0, 1.0);
    }

    fn command(&self) -> f64 {
        self.command
    }

    fn compute_force(&mut self, position: f64, velocity: f64, dt: f64) -> f64 {
        if !self.enabled {
            return 0.0;
        }

        // Integrate velocity command to update target position
        let velocity_command = self.command * self.max_velocity;
        self.target_position += velocity_command * dt;
        self.target_position = self.clamp_to_limits(self.target_position);

        // PD control to track target
        let position_error = self.target_position - position;
        let velocity_error = -velocity; // Target velocity is 0 (we want to reach and stop)

        let force = self.kp.mul_add(position_error, self.kd * velocity_error);

        // Clamp to max force
        force.clamp(-self.max_force, self.max_force)
    }

    fn reset(&mut self) {
        self.command = 0.0;
        self.target_position = 0.0;
    }

    fn max_force(&self) -> f64 {
        self.max_force
    }

    fn has_dynamics(&self) -> bool {
        true // Has internal target position state
    }
}

impl Default for IntegratedVelocityActuator {
    fn default() -> Self {
        Self::new(1.0, 10.0)
    }
}

// ============================================================================
// Pneumatic Cylinder Actuator
// ============================================================================

/// A pneumatic (or hydraulic) cylinder actuator for soft robotics.
///
/// Models a double-acting cylinder with pressure dynamics on both chambers.
/// The force is proportional to pressure difference times piston area.
///
/// # Dynamics
///
/// ```text
/// P_dot = -k_leak * P + k_fill * u * (P_supply - P)    (for u > 0)
/// P_dot = -k_leak * P - k_exhaust * |u| * P            (for u < 0)
///
/// F = A * (P_extend - P_retract) - F_friction
/// ```
///
/// This creates soft, compliant actuation suitable for human-robot interaction.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PneumaticCylinderActuator {
    /// Actuator name.
    name: String,

    /// Piston area (m²).
    piston_area: f64,

    /// Supply pressure (Pa).
    supply_pressure: f64,

    /// Current extend chamber pressure (Pa).
    pressure_extend: f64,

    /// Current retract chamber pressure (Pa).
    pressure_retract: f64,

    /// Atmospheric pressure (Pa).
    atmospheric_pressure: f64,

    /// Fill rate constant (1/s).
    fill_rate: f64,

    /// Exhaust rate constant (1/s).
    exhaust_rate: f64,

    /// Leakage rate constant (1/s).
    leak_rate: f64,

    /// Coulomb friction force (N).
    friction_force: f64,

    /// Viscous friction coefficient (N·s/m).
    viscous_friction: f64,

    /// Current command (-1 = full retract, +1 = full extend).
    command: f64,

    /// Maximum force (computed from supply pressure).
    max_force: f64,

    /// Whether the actuator is enabled.
    enabled: bool,
}

impl PneumaticCylinderActuator {
    /// Create a new pneumatic cylinder actuator.
    ///
    /// # Arguments
    ///
    /// * `piston_area` - Cross-sectional area of the piston (m²)
    /// * `supply_pressure` - Maximum supply pressure (Pa)
    #[must_use]
    pub fn new(piston_area: f64, supply_pressure: f64) -> Self {
        let atmospheric = 101_325.0; // 1 atm in Pa
        Self {
            name: String::from("pneumatic_cylinder"),
            piston_area: piston_area.abs(),
            supply_pressure: supply_pressure.abs(),
            pressure_extend: atmospheric,
            pressure_retract: atmospheric,
            atmospheric_pressure: atmospheric,
            fill_rate: 10.0,    // Fast fill
            exhaust_rate: 15.0, // Slightly faster exhaust
            leak_rate: 0.1,     // Small leakage
            friction_force: 5.0,
            viscous_friction: 50.0,
            command: 0.0,
            max_force: piston_area.abs() * (supply_pressure.abs() - atmospheric),
            enabled: true,
        }
    }

    /// Set the actuator name.
    #[must_use]
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = name.into();
        self
    }

    /// Set the fill and exhaust rates.
    #[must_use]
    pub fn with_rates(mut self, fill_rate: f64, exhaust_rate: f64, leak_rate: f64) -> Self {
        self.fill_rate = fill_rate.abs();
        self.exhaust_rate = exhaust_rate.abs();
        self.leak_rate = leak_rate.abs();
        self
    }

    /// Set friction parameters.
    #[must_use]
    pub fn with_friction(mut self, coulomb: f64, viscous: f64) -> Self {
        self.friction_force = coulomb.abs();
        self.viscous_friction = viscous.abs();
        self
    }

    /// Enable or disable the actuator.
    #[must_use]
    pub fn with_enabled(mut self, enabled: bool) -> Self {
        self.enabled = enabled;
        self
    }

    /// Get the current extend chamber pressure.
    #[must_use]
    pub fn pressure_extend(&self) -> f64 {
        self.pressure_extend
    }

    /// Get the current retract chamber pressure.
    #[must_use]
    pub fn pressure_retract(&self) -> f64 {
        self.pressure_retract
    }

    /// Create a preset for a small pneumatic muscle (`McKibben` actuator).
    #[must_use]
    pub fn mckibben_small() -> Self {
        Self::new(0.0001, 600_000.0) // 1 cm², 6 bar
            .with_rates(5.0, 8.0, 0.05)
            .with_friction(2.0, 20.0)
    }

    /// Create a preset for a medium industrial cylinder.
    #[must_use]
    pub fn industrial_medium() -> Self {
        Self::new(0.002, 700_000.0) // 20 cm², 7 bar
            .with_rates(15.0, 20.0, 0.02)
            .with_friction(10.0, 100.0)
    }

    /// Create a preset for a hydraulic cylinder.
    #[must_use]
    pub fn hydraulic() -> Self {
        Self::new(0.001, 20_000_000.0) // 10 cm², 200 bar
            .with_rates(50.0, 50.0, 0.001) // Much faster, less leakage
            .with_friction(50.0, 500.0)
    }

    /// Update pressure dynamics for one chamber.
    #[allow(clippy::too_many_arguments)]
    fn update_pressure(
        pressure: f64,
        supply: f64,
        atmospheric: f64,
        command: f64,
        fill_rate: f64,
        exhaust_rate: f64,
        leak_rate: f64,
        dt: f64,
    ) -> f64 {
        let mut p_dot = -leak_rate * (pressure - atmospheric);

        if command > 0.0 {
            // Filling from supply
            p_dot += fill_rate * command * (supply - pressure);
        } else if command < 0.0 {
            // Exhausting to atmosphere
            p_dot += exhaust_rate * command * (pressure - atmospheric);
        }

        let new_pressure = p_dot.mul_add(dt, pressure);

        // Clamp to physical limits
        new_pressure.clamp(atmospheric * 0.1, supply * 1.1)
    }
}

impl Actuator for PneumaticCylinderActuator {
    fn name(&self) -> &str {
        &self.name
    }

    fn set_command(&mut self, command: f64) {
        self.command = command.clamp(-1.0, 1.0);
    }

    fn command(&self) -> f64 {
        self.command
    }

    fn compute_force(&mut self, _position: f64, velocity: f64, dt: f64) -> f64 {
        if !self.enabled {
            return 0.0;
        }

        // Update extend chamber (positive command fills it)
        self.pressure_extend = Self::update_pressure(
            self.pressure_extend,
            self.supply_pressure,
            self.atmospheric_pressure,
            self.command,
            self.fill_rate,
            self.exhaust_rate,
            self.leak_rate,
            dt,
        );

        // Update retract chamber (negative command fills it)
        self.pressure_retract = Self::update_pressure(
            self.pressure_retract,
            self.supply_pressure,
            self.atmospheric_pressure,
            -self.command,
            self.fill_rate,
            self.exhaust_rate,
            self.leak_rate,
            dt,
        );

        // Compute force from pressure difference
        let pressure_force = self.piston_area * (self.pressure_extend - self.pressure_retract);

        // Apply friction
        let friction = if velocity.abs() > 1e-6 {
            (-velocity.signum()).mul_add(self.friction_force, -self.viscous_friction * velocity)
        } else {
            // Static friction: oppose applied force up to max
            (-pressure_force).clamp(-self.friction_force, self.friction_force)
        };

        // No additional clamping - the pressure dynamics naturally limit force
        pressure_force + friction
    }

    fn reset(&mut self) {
        self.command = 0.0;
        self.pressure_extend = self.atmospheric_pressure;
        self.pressure_retract = self.atmospheric_pressure;
    }

    fn max_force(&self) -> f64 {
        self.max_force
    }

    fn has_dynamics(&self) -> bool {
        true // Has pressure state
    }
}

impl Default for PneumaticCylinderActuator {
    fn default() -> Self {
        Self::new(0.001, 500_000.0) // 10 cm², 5 bar
    }
}

// ============================================================================
// Adhesion Actuator
// ============================================================================

/// An adhesion actuator for gripping and climbing robots.
///
/// Models controllable adhesion forces (e.g., electroadhesion, gecko-inspired
/// adhesives, suction cups, magnetic grippers).
///
/// The actuator applies a normal force pulling the body toward a surface.
/// The maximum adhesion force depends on the current contact state.
///
/// # Dynamics
///
/// ```text
/// adhesion_state_dot = (command - adhesion_state) / tau
/// F_adhesion = adhesion_state * max_adhesion * contact_area_ratio
/// ```
///
/// The activation time constant creates realistic engage/disengage dynamics.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct AdhesionActuator {
    /// Actuator name.
    name: String,

    /// Maximum adhesion force (N).
    max_adhesion: f64,

    /// Current adhesion state (0 to 1).
    adhesion_state: f64,

    /// Activation time constant (seconds).
    activation_tau: f64,

    /// Deactivation time constant (seconds).
    deactivation_tau: f64,

    /// Current command (0 to 1, where 0 = off, 1 = max adhesion).
    command: f64,

    /// Current contact state (0 to 1, where 0 = no contact, 1 = full contact).
    contact_ratio: f64,

    /// Adhesion direction in local frame (usually surface normal).
    direction: Vector3<f64>,

    /// Whether the actuator is enabled.
    enabled: bool,

    /// Shear adhesion ratio (fraction of normal adhesion available as shear).
    shear_ratio: f64,
}

impl AdhesionActuator {
    /// Create a new adhesion actuator.
    ///
    /// # Arguments
    ///
    /// * `max_adhesion` - Maximum adhesion force when fully activated (N)
    #[must_use]
    pub fn new(max_adhesion: f64) -> Self {
        Self {
            name: String::from("adhesion"),
            max_adhesion: max_adhesion.abs(),
            adhesion_state: 0.0,
            activation_tau: 0.05,   // 50ms to activate
            deactivation_tau: 0.02, // 20ms to deactivate (faster)
            command: 0.0,
            contact_ratio: 0.0,
            direction: -Vector3::z(), // Default: pull in -Z direction
            enabled: true,
            shear_ratio: 0.3, // Can resist 30% of normal force in shear
        }
    }

    /// Set the actuator name.
    #[must_use]
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = name.into();
        self
    }

    /// Set the time constants for activation and deactivation.
    #[must_use]
    pub fn with_time_constants(mut self, activation_tau: f64, deactivation_tau: f64) -> Self {
        self.activation_tau = activation_tau.abs().max(0.001);
        self.deactivation_tau = deactivation_tau.abs().max(0.001);
        self
    }

    /// Set the adhesion direction in local frame.
    #[must_use]
    pub fn with_direction(mut self, direction: Vector3<f64>) -> Self {
        let norm = direction.norm();
        self.direction = if norm > 1e-10 {
            direction / norm
        } else {
            -Vector3::z()
        };
        self
    }

    /// Set the shear adhesion ratio.
    #[must_use]
    pub fn with_shear_ratio(mut self, ratio: f64) -> Self {
        self.shear_ratio = ratio.clamp(0.0, 1.0);
        self
    }

    /// Enable or disable the actuator.
    #[must_use]
    pub fn with_enabled(mut self, enabled: bool) -> Self {
        self.enabled = enabled;
        self
    }

    /// Update the contact ratio (should be called by contact detection).
    ///
    /// # Arguments
    ///
    /// * `ratio` - Contact area ratio (0 = no contact, 1 = full contact)
    pub fn set_contact_ratio(&mut self, ratio: f64) {
        self.contact_ratio = ratio.clamp(0.0, 1.0);
    }

    /// Get the current adhesion state.
    #[must_use]
    pub fn adhesion_state(&self) -> f64 {
        self.adhesion_state
    }

    /// Get the current effective adhesion force.
    #[must_use]
    pub fn effective_adhesion(&self) -> f64 {
        self.adhesion_state * self.max_adhesion * self.contact_ratio
    }

    /// Get the maximum shear force (resistance to sliding).
    #[must_use]
    pub fn max_shear_force(&self) -> f64 {
        self.effective_adhesion() * self.shear_ratio
    }

    /// Get the adhesion direction.
    #[must_use]
    pub fn direction(&self) -> &Vector3<f64> {
        &self.direction
    }

    /// Create a preset for electroadhesion (fast, moderate force).
    #[must_use]
    pub fn electroadhesion(max_force: f64) -> Self {
        Self::new(max_force)
            .with_time_constants(0.01, 0.005) // Very fast
            .with_shear_ratio(0.2)
    }

    /// Create a preset for gecko-inspired dry adhesion (slower, higher shear).
    #[must_use]
    pub fn gecko_adhesion(max_force: f64) -> Self {
        Self::new(max_force)
            .with_time_constants(0.1, 0.05) // Slower engagement
            .with_shear_ratio(0.5) // Good shear resistance
    }

    /// Create a preset for suction cup (slow, strong).
    #[must_use]
    pub fn suction_cup(max_force: f64) -> Self {
        Self::new(max_force)
            .with_time_constants(0.2, 0.1) // Slowest
            .with_shear_ratio(0.1) // Poor shear resistance
    }

    /// Create a preset for magnetic gripper (fast, strong).
    #[must_use]
    pub fn magnetic(max_force: f64) -> Self {
        Self::new(max_force)
            .with_time_constants(0.02, 0.02) // Fast on/off
            .with_shear_ratio(0.4) // Decent shear
    }
}

impl Actuator for AdhesionActuator {
    fn name(&self) -> &str {
        &self.name
    }

    fn set_command(&mut self, command: f64) {
        // Adhesion command is 0-1 (not -1 to 1)
        self.command = command.clamp(0.0, 1.0);
    }

    fn command(&self) -> f64 {
        self.command
    }

    fn compute_force(&mut self, _position: f64, _velocity: f64, dt: f64) -> f64 {
        if !self.enabled {
            return 0.0;
        }

        // Update adhesion state with first-order dynamics
        let tau = if self.command > self.adhesion_state {
            self.activation_tau
        } else {
            self.deactivation_tau
        };

        let alpha = (-dt / tau).exp();
        self.adhesion_state = alpha.mul_add(self.adhesion_state, (1.0 - alpha) * self.command);

        // Compute effective adhesion force
        // The force magnitude (sign is in the direction vector)
        self.effective_adhesion()
    }

    fn reset(&mut self) {
        self.command = 0.0;
        self.adhesion_state = 0.0;
        self.contact_ratio = 0.0;
    }

    fn max_force(&self) -> f64 {
        self.max_adhesion
    }

    fn has_dynamics(&self) -> bool {
        true // Has adhesion state
    }
}

impl Default for AdhesionActuator {
    fn default() -> Self {
        Self::new(100.0) // 100N max adhesion
    }
}

// ============================================================================
// Custom Actuator (User-Defined)
// ============================================================================

/// A wrapper for user-defined actuator behavior.
///
/// This allows users to implement custom actuator dynamics through a closure
/// or by implementing the [`ActuatorFn`] trait.
///
/// # Example
///
/// ```
/// use sim_constraint::actuator::CustomActuator;
///
/// // Create a simple spring-damper actuator
/// let actuator = CustomActuator::new(
///     "spring_damper",
///     100.0, // max force
///     |command, position, velocity, _dt| {
///         let kp = 500.0;
///         let kd = 50.0;
///         let target = command; // Command is target position
///         kp * (target - position) - kd * velocity
///     },
/// );
/// ```
pub struct CustomActuator<F>
where
    F: FnMut(f64, f64, f64, f64) -> f64 + Send + Sync,
{
    /// Actuator name.
    name: String,

    /// Maximum force/torque.
    max_force: f64,

    /// Current command.
    command: f64,

    /// User-provided force computation function.
    /// Arguments: (command, position, velocity, dt) -> force
    compute_fn: F,

    /// Whether the actuator is enabled.
    enabled: bool,
}

impl<F> CustomActuator<F>
where
    F: FnMut(f64, f64, f64, f64) -> f64 + Send + Sync,
{
    /// Create a new custom actuator.
    ///
    /// # Arguments
    ///
    /// * `name` - Actuator name for debugging
    /// * `max_force` - Maximum force/torque magnitude
    /// * `compute_fn` - Function to compute force: (command, position, velocity, dt) -> force
    pub fn new(name: impl Into<String>, max_force: f64, compute_fn: F) -> Self {
        Self {
            name: name.into(),
            max_force: max_force.abs(),
            command: 0.0,
            compute_fn,
            enabled: true,
        }
    }

    /// Enable or disable the actuator.
    #[must_use]
    pub fn with_enabled(mut self, enabled: bool) -> Self {
        self.enabled = enabled;
        self
    }
}

impl<F> Actuator for CustomActuator<F>
where
    F: FnMut(f64, f64, f64, f64) -> f64 + Send + Sync,
{
    fn name(&self) -> &str {
        &self.name
    }

    fn set_command(&mut self, command: f64) {
        self.command = command;
    }

    fn command(&self) -> f64 {
        self.command
    }

    fn compute_force(&mut self, position: f64, velocity: f64, dt: f64) -> f64 {
        if !self.enabled {
            return 0.0;
        }

        let force = (self.compute_fn)(self.command, position, velocity, dt);
        force.clamp(-self.max_force, self.max_force)
    }

    fn reset(&mut self) {
        self.command = 0.0;
    }

    fn max_force(&self) -> f64 {
        self.max_force
    }
}

impl<F> std::fmt::Debug for CustomActuator<F>
where
    F: FnMut(f64, f64, f64, f64) -> f64 + Send + Sync,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CustomActuator")
            .field("name", &self.name)
            .field("max_force", &self.max_force)
            .field("command", &self.command)
            .field("enabled", &self.enabled)
            .finish_non_exhaustive()
    }
}

// ============================================================================
// Boxed Actuator (Type-erased)
// ============================================================================

/// A type-erased actuator for storing heterogeneous actuator collections.
pub type BoxedActuator = Box<dyn Actuator>;

/// Extension trait for creating boxed actuators.
pub trait IntoBoxedActuator {
    /// Convert into a boxed actuator.
    fn boxed(self) -> BoxedActuator;
}

impl<A: Actuator + 'static> IntoBoxedActuator for A {
    fn boxed(self) -> BoxedActuator {
        Box::new(self)
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
    fn test_integrated_velocity_actuator() {
        let mut actuator =
            IntegratedVelocityActuator::new(2.0, 100.0).with_position_limits(-1.0, 1.0);

        // Initial target should be 0
        assert_relative_eq!(actuator.target_position(), 0.0, epsilon = 1e-10);

        // Set positive velocity command
        actuator.set_command(0.5);

        // Step multiple times
        let dt = 0.01;
        for _ in 0..10 {
            let _ = actuator.compute_force(0.0, 0.0, dt);
        }

        // Target should have increased
        assert!(actuator.target_position() > 0.0);
        assert!(actuator.target_position() <= 1.0); // Respects limit

        // Continue until limit
        for _ in 0..1000 {
            let _ = actuator.compute_force(0.0, 0.0, dt);
        }

        // Should be at upper limit
        assert_relative_eq!(actuator.target_position(), 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_integrated_velocity_force_output() {
        let mut actuator = IntegratedVelocityActuator::new(1.0, 100.0).with_gains(1000.0, 100.0);

        actuator.set_target_position(0.5);

        // Position below target should give positive force
        let force = actuator.compute_force(0.0, 0.0, 0.001);
        assert!(force > 0.0);

        // Position above target should give negative force
        let force = actuator.compute_force(1.0, 0.0, 0.001);
        assert!(force < 0.0);
    }

    #[test]
    fn test_pneumatic_cylinder() {
        let mut actuator = PneumaticCylinderActuator::default();

        // Initially at atmospheric pressure
        assert!(actuator.pressure_extend() > 100_000.0);
        assert!(actuator.pressure_retract() > 100_000.0);

        // Apply extend command
        actuator.set_command(1.0);

        let dt = 0.001;
        for _ in 0..100 {
            let _ = actuator.compute_force(0.0, 0.0, dt);
        }

        // Extend pressure should increase
        assert!(actuator.pressure_extend() > actuator.pressure_retract());
    }

    #[test]
    fn test_pneumatic_force_direction() {
        let mut actuator = PneumaticCylinderActuator::new(0.001, 500_000.0).with_friction(0.0, 0.0); // No friction for this test

        // Charge extend chamber
        actuator.set_command(1.0);
        let dt = 0.001;
        for _ in 0..500 {
            let _ = actuator.compute_force(0.0, 0.0, dt);
        }

        // Force should be positive (extending)
        let force = actuator.compute_force(0.0, 0.0, dt);
        assert!(force > 0.0);

        // Now retract
        actuator.set_command(-1.0);
        for _ in 0..500 {
            let _ = actuator.compute_force(0.0, 0.0, dt);
        }

        // Force should be negative (retracting)
        let force = actuator.compute_force(0.0, 0.0, dt);
        assert!(force < 0.0);
    }

    #[test]
    fn test_adhesion_actuator() {
        let mut actuator = AdhesionActuator::new(100.0).with_time_constants(0.01, 0.01);

        // Set full contact
        actuator.set_contact_ratio(1.0);

        // Initially no adhesion
        assert_relative_eq!(actuator.adhesion_state(), 0.0, epsilon = 1e-10);

        // Turn on adhesion
        actuator.set_command(1.0);

        let dt = 0.001;
        for _ in 0..100 {
            let _ = actuator.compute_force(0.0, 0.0, dt);
        }

        // Should be close to fully activated
        assert!(actuator.adhesion_state() > 0.9);
        assert!(actuator.effective_adhesion() > 90.0);
    }

    #[test]
    fn test_adhesion_contact_dependence() {
        let mut actuator = AdhesionActuator::new(100.0);

        // Full command but no contact
        actuator.set_command(1.0);
        actuator.set_contact_ratio(0.0);

        // Let it activate
        let dt = 0.001;
        for _ in 0..500 {
            let _ = actuator.compute_force(0.0, 0.0, dt);
        }

        // No effective adhesion without contact
        assert_relative_eq!(actuator.effective_adhesion(), 0.0, epsilon = 1e-10);

        // Now add contact
        actuator.set_contact_ratio(0.5);
        let force = actuator.compute_force(0.0, 0.0, dt);

        // Should have some adhesion now
        assert!(force > 0.0);
        assert!(force < 100.0); // Not full due to partial contact
    }

    #[test]
    fn test_custom_actuator() {
        let mut actuator =
            CustomActuator::new("test", 100.0, |command, position, velocity, _dt| {
                // Simple PD controller where command is target position
                let kp = 100.0;
                let kd = 10.0;
                kp * (command - position) - kd * velocity
            });

        // Set target at 0.5
        actuator.set_command(0.5);

        // Position at 0, should push positive
        let force = actuator.compute_force(0.0, 0.0, 0.001);
        assert!(force > 0.0);

        // Position at 1.0, should push negative
        let force = actuator.compute_force(1.0, 0.0, 0.001);
        assert!(force < 0.0);
    }

    #[test]
    fn test_actuator_trait_object() {
        let actuators: Vec<BoxedActuator> = vec![
            IntegratedVelocityActuator::new(1.0, 10.0).boxed(),
            PneumaticCylinderActuator::default().boxed(),
            AdhesionActuator::new(50.0).boxed(),
        ];

        // Can iterate and use uniformly
        for actuator in &actuators {
            assert!(!actuator.name().is_empty());
            assert!(actuator.max_force() > 0.0);
        }

        assert_eq!(actuators.len(), 3);
    }

    #[test]
    fn test_actuator_reset() {
        let mut actuator = IntegratedVelocityActuator::new(1.0, 10.0);

        actuator.set_command(1.0);
        for _ in 0..100 {
            let _ = actuator.compute_force(0.0, 0.0, 0.01);
        }

        assert!(actuator.target_position() > 0.0);

        actuator.reset();

        assert_relative_eq!(actuator.command(), 0.0, epsilon = 1e-10);
        assert_relative_eq!(actuator.target_position(), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_adhesion_presets() {
        let electro = AdhesionActuator::electroadhesion(100.0);
        let gecko = AdhesionActuator::gecko_adhesion(100.0);
        let suction = AdhesionActuator::suction_cup(100.0);
        let magnetic = AdhesionActuator::magnetic(100.0);

        // Electro should be fastest
        assert!(electro.activation_tau < gecko.activation_tau);

        // Suction should be slowest
        assert!(suction.activation_tau > gecko.activation_tau);

        // Gecko should have best shear
        assert!(gecko.shear_ratio > suction.shear_ratio);

        // All have same max force
        assert_relative_eq!(electro.max_force(), 100.0, epsilon = 1e-10);
        assert_relative_eq!(magnetic.max_force(), 100.0, epsilon = 1e-10);
    }

    #[test]
    fn test_pneumatic_presets() {
        let mckibben = PneumaticCylinderActuator::mckibben_small();
        let industrial = PneumaticCylinderActuator::industrial_medium();
        let hydraulic = PneumaticCylinderActuator::hydraulic();

        // Hydraulic has highest pressure
        assert!(hydraulic.supply_pressure > industrial.supply_pressure);

        // Industrial has largest piston
        assert!(industrial.piston_area > mckibben.piston_area);

        // Hydraulic has fastest dynamics
        assert!(hydraulic.fill_rate > industrial.fill_rate);
    }
}
