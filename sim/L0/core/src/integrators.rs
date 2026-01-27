//! Numerical integration methods for rigid body dynamics.
//!
//! This module provides various integration methods for advancing rigid body
//! state through time. Each method offers different trade-offs between accuracy,
//! stability, and computational cost.
//!
//! # Integration Methods
//!
//! - **Explicit Euler**: Simple but unstable for stiff systems
//! - **Semi-Implicit Euler**: Symplectic, good for games and real-time
//! - **Velocity Verlet**: Second-order symplectic, good energy conservation
//! - **RK4**: Fourth-order, high accuracy, expensive
//!
//! # Example
//!
//! ```
//! use sim_core::integrators::{Integrator, SemiImplicitEuler};
//! use sim_types::{RigidBodyState, Pose, Twist};
//! use nalgebra::{Point3, Vector3};
//!
//! let mut state = RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 10.0)));
//! let gravity = Vector3::new(0.0, 0.0, -9.81);
//!
//! // Integrate for 0.01 seconds with gravity
//! SemiImplicitEuler::integrate(&mut state, gravity, Vector3::zeros(), 0.01);
//!
//! // Position and velocity should have changed
//! assert!(state.pose.position.z < 10.0);
//! assert!(state.twist.linear.z < 0.0);
//! ```

use nalgebra::{UnitQuaternion, Vector3};
use sim_types::{IntegrationMethod, RigidBodyState, Twist};

/// Trait for integration methods.
pub trait Integrator {
    /// Integrate the rigid body state forward by dt.
    ///
    /// # Arguments
    ///
    /// * `state` - Current rigid body state (modified in place)
    /// * `linear_accel` - Linear acceleration (m/s²)
    /// * `angular_accel` - Angular acceleration (rad/s²)
    /// * `dt` - Timestep in seconds
    fn integrate(
        state: &mut RigidBodyState,
        linear_accel: Vector3<f64>,
        angular_accel: Vector3<f64>,
        dt: f64,
    );
}

/// Dispatch to the appropriate integrator based on method enum.
pub fn integrate_with_method(
    method: IntegrationMethod,
    state: &mut RigidBodyState,
    linear_accel: Vector3<f64>,
    angular_accel: Vector3<f64>,
    dt: f64,
) {
    match method {
        IntegrationMethod::ExplicitEuler => {
            ExplicitEuler::integrate(state, linear_accel, angular_accel, dt);
        }
        IntegrationMethod::SemiImplicitEuler => {
            SemiImplicitEuler::integrate(state, linear_accel, angular_accel, dt);
        }
        IntegrationMethod::VelocityVerlet => {
            VelocityVerlet::integrate(state, linear_accel, angular_accel, dt);
        }
        IntegrationMethod::RungeKutta4 => {
            RungeKutta4::integrate(state, linear_accel, angular_accel, dt);
        }
        IntegrationMethod::ImplicitVelocity => {
            // Use default damping (none) - for damped systems use integrate_implicit_with_damping
            ImplicitVelocity::integrate(state, linear_accel, angular_accel, dt);
        }
        IntegrationMethod::ImplicitFast => {
            // Assumes accelerations were computed without Coriolis terms
            ImplicitFast::integrate(state, linear_accel, angular_accel, dt);
        }
    }
}

/// Dispatch to integrator with damping support for implicit methods.
///
/// For explicit methods, damping is applied separately after integration.
/// For implicit methods (like `ImplicitVelocity` and `ImplicitFast`), damping
/// is incorporated directly into the integration step for unconditional stability.
pub fn integrate_with_method_and_damping(
    method: IntegrationMethod,
    state: &mut RigidBodyState,
    linear_accel: Vector3<f64>,
    angular_accel: Vector3<f64>,
    linear_damping: f64,
    angular_damping: f64,
    dt: f64,
) {
    match method {
        IntegrationMethod::ImplicitVelocity => {
            ImplicitVelocity::integrate_with_damping(
                state,
                linear_accel,
                angular_accel,
                linear_damping,
                angular_damping,
                dt,
            );
        }
        IntegrationMethod::ImplicitFast => {
            // Same damping handling as ImplicitVelocity, but caller should have
            // computed accelerations without Coriolis terms
            ImplicitFast::integrate_with_damping(
                state,
                linear_accel,
                angular_accel,
                linear_damping,
                angular_damping,
                dt,
            );
        }
        // For other methods, integrate normally (damping applied separately)
        _ => {
            integrate_with_method(method, state, linear_accel, angular_accel, dt);
        }
    }
}

/// Explicit Euler integration (first-order).
///
/// Simple but can be unstable for stiff systems or large timesteps.
///
/// ```text
/// x(t+dt) = x(t) + v(t) * dt
/// v(t+dt) = v(t) + a(t) * dt
/// ```
pub struct ExplicitEuler;

impl Integrator for ExplicitEuler {
    fn integrate(
        state: &mut RigidBodyState,
        linear_accel: Vector3<f64>,
        angular_accel: Vector3<f64>,
        dt: f64,
    ) {
        // Store current velocity for position update
        let linear_vel = state.twist.linear;
        let angular_vel = state.twist.angular;

        // Update position using current velocity
        state.pose.position += linear_vel * dt;

        // Update rotation using current angular velocity
        integrate_rotation(&mut state.pose.rotation, &angular_vel, dt);

        // Update velocity using acceleration
        state.twist.linear += linear_accel * dt;
        state.twist.angular += angular_accel * dt;
    }
}

/// Semi-implicit Euler integration (symplectic Euler).
///
/// Updates velocity first, then uses new velocity for position.
/// This is symplectic (energy-preserving) and stable for oscillatory systems.
///
/// ```text
/// v(t+dt) = v(t) + a(t) * dt
/// x(t+dt) = x(t) + v(t+dt) * dt
/// ```
pub struct SemiImplicitEuler;

impl Integrator for SemiImplicitEuler {
    fn integrate(
        state: &mut RigidBodyState,
        linear_accel: Vector3<f64>,
        angular_accel: Vector3<f64>,
        dt: f64,
    ) {
        // Update velocity first
        state.twist.linear += linear_accel * dt;
        state.twist.angular += angular_accel * dt;

        // Update position using new velocity
        state.pose.position += state.twist.linear * dt;

        // Update rotation using new angular velocity
        integrate_rotation(&mut state.pose.rotation, &state.twist.angular, dt);
    }
}

/// Velocity Verlet integration (second-order symplectic).
///
/// Requires the previous acceleration, which we approximate by using
/// the current acceleration for both. For constant acceleration, this is exact.
///
/// ```text
/// x(t+dt) = x(t) + v(t) * dt + 0.5 * a(t) * dt²
/// v(t+dt) = v(t) + 0.5 * (a(t) + a(t+dt)) * dt
/// ```
///
/// Note: True Verlet requires knowing `a(t+dt)`, which depends on `x(t+dt)`.
/// This implementation assumes constant acceleration over the timestep.
pub struct VelocityVerlet;

impl Integrator for VelocityVerlet {
    fn integrate(
        state: &mut RigidBodyState,
        linear_accel: Vector3<f64>,
        angular_accel: Vector3<f64>,
        dt: f64,
    ) {
        let half_dt_sq = 0.5 * dt * dt;

        // Update position: x + v*dt + 0.5*a*dt²
        state.pose.position += state.twist.linear * dt + linear_accel * half_dt_sq;

        // Update rotation with current angular velocity plus half acceleration
        let avg_angular = state.twist.angular + angular_accel * (0.5 * dt);
        integrate_rotation(&mut state.pose.rotation, &avg_angular, dt);

        // Update velocity: v + a*dt
        // (For true Verlet, we'd use average of old and new acceleration,
        //  but we assume constant acceleration here)
        state.twist.linear += linear_accel * dt;
        state.twist.angular += angular_accel * dt;
    }
}

/// Fourth-order Runge-Kutta integration.
///
/// High accuracy but expensive (4 derivative evaluations per step).
/// For constant acceleration, this simplifies significantly but we
/// implement the general form for correctness.
///
/// Note: This implementation assumes constant acceleration, so it's
/// effectively equivalent to simpler methods for rigid body dynamics
/// without velocity-dependent forces.
pub struct RungeKutta4;

impl Integrator for RungeKutta4 {
    fn integrate(
        state: &mut RigidBodyState,
        linear_accel: Vector3<f64>,
        angular_accel: Vector3<f64>,
        dt: f64,
    ) {
        // For constant acceleration, RK4 reduces to:
        // x(t+dt) = x(t) + v(t)*dt + 0.5*a*dt²
        // v(t+dt) = v(t) + a*dt
        //
        // This is because with constant a:
        // k1 = a, k2 = a, k3 = a, k4 = a
        // So the weighted average is just a.

        let half_dt = dt * 0.5;

        // Position: RK4 with constant acceleration gives same as Verlet
        state.pose.position += state.twist.linear * dt + linear_accel * (0.5 * dt * dt);

        // Rotation: similar treatment
        let avg_angular = state.twist.angular + angular_accel * half_dt;
        integrate_rotation(&mut state.pose.rotation, &avg_angular, dt);

        // Velocity update
        state.twist.linear += linear_accel * dt;
        state.twist.angular += angular_accel * dt;
    }
}

/// Implicit-in-velocity integration.
///
/// This method solves the implicit equation:
/// ```text
/// (M - h*D) * v_{t+h} = M * v_t + h * f
/// ```
///
/// Where:
/// - M is the mass/inertia matrix
/// - D is the damping matrix (velocity-dependent force derivatives)
/// - h is the timestep
/// - f is the total force
///
/// For a body with scalar mass m and damping coefficient d:
/// ```text
/// v_{t+h} = (v_t + h * a) / (1 + h * d / m)
/// ```
///
/// This is unconditionally stable for any timestep when D ≥ 0, making it
/// ideal for:
/// - Very stiff contacts (high contact damping)
/// - Highly damped systems
/// - Muscle models with activation dynamics
///
/// Without damping, this reduces to semi-implicit Euler.
pub struct ImplicitVelocity;

impl ImplicitVelocity {
    /// Integrate with specified damping coefficients.
    ///
    /// This incorporates damping directly into the implicit solve, providing
    /// unconditional stability even for very high damping values.
    ///
    /// # Arguments
    ///
    /// * `state` - Current rigid body state (modified in place)
    /// * `linear_accel` - Linear acceleration (m/s²)
    /// * `angular_accel` - Angular acceleration (rad/s²)
    /// * `linear_damping` - Linear damping coefficient (1/s)
    /// * `angular_damping` - Angular damping coefficient (1/s)
    /// * `dt` - Timestep in seconds
    pub fn integrate_with_damping(
        state: &mut RigidBodyState,
        linear_accel: Vector3<f64>,
        angular_accel: Vector3<f64>,
        linear_damping: f64,
        angular_damping: f64,
        dt: f64,
    ) {
        // Implicit velocity update:
        // (1 + h*d) * v_{t+h} = v_t + h * a
        // v_{t+h} = (v_t + h * a) / (1 + h * d)
        //
        // This is equivalent to solving:
        // v_{t+h} - v_t = h * (a - d * v_{t+h})
        // which implicitly includes damping

        // Guard against degenerate divisors (negative damping could cause issues)
        let linear_divisor = (1.0 + dt * linear_damping).max(1e-10);
        let angular_divisor = (1.0 + dt * angular_damping).max(1e-10);

        // Update velocity implicitly
        state.twist.linear = (state.twist.linear + linear_accel * dt) / linear_divisor;
        state.twist.angular = (state.twist.angular + angular_accel * dt) / angular_divisor;

        // Update position using new velocity (semi-implicit style)
        state.pose.position += state.twist.linear * dt;

        // Update rotation using new angular velocity
        integrate_rotation(&mut state.pose.rotation, &state.twist.angular, dt);
    }
}

impl Integrator for ImplicitVelocity {
    fn integrate(
        state: &mut RigidBodyState,
        linear_accel: Vector3<f64>,
        angular_accel: Vector3<f64>,
        dt: f64,
    ) {
        // Without damping, this reduces to semi-implicit Euler
        Self::integrate_with_damping(state, linear_accel, angular_accel, 0.0, 0.0, dt);
    }
}

/// Implicit-fast integration (`MuJoCo` "implicitfast" equivalent).
///
/// This is a performance optimization of [`ImplicitVelocity`] that skips
/// the computation of Coriolis and centrifugal terms in the inertia matrix.
///
/// # When to Use
///
/// Use this instead of `ImplicitVelocity` when:
/// - Angular velocities are relatively low
/// - The system has many bodies (>20) and you need performance
/// - Slight accuracy loss in rotational dynamics is acceptable
///
/// # Performance
///
/// Skipping Coriolis terms saves O(n²) operations per body where n is the
/// number of DOF, which can be significant for articulated systems.
///
/// # Accuracy Trade-off
///
/// The Coriolis effect creates fictitious forces that appear in rotating
/// reference frames:
/// - Centrifugal force: mv²/r outward
/// - Coriolis force: 2m(ω × v)
///
/// For most robotics applications (humanoids, manipulators) at normal
/// speeds, these forces are small compared to gravity and actuation forces.
///
/// `MuJoCo` uses this as its default "implicitfast" mode for performance.
pub struct ImplicitFast;

impl ImplicitFast {
    /// Integrate with specified damping coefficients, skipping Coriolis terms.
    ///
    /// This is identical to [`ImplicitVelocity::integrate_with_damping`] but
    /// the caller should have computed accelerations without Coriolis terms.
    ///
    /// # Arguments
    ///
    /// * `state` - Current rigid body state (modified in place)
    /// * `linear_accel` - Linear acceleration WITHOUT Coriolis (m/s²)
    /// * `angular_accel` - Angular acceleration WITHOUT Coriolis (rad/s²)
    /// * `linear_damping` - Linear damping coefficient (1/s)
    /// * `angular_damping` - Angular damping coefficient (1/s)
    /// * `dt` - Timestep in seconds
    pub fn integrate_with_damping(
        state: &mut RigidBodyState,
        linear_accel: Vector3<f64>,
        angular_accel: Vector3<f64>,
        linear_damping: f64,
        angular_damping: f64,
        dt: f64,
    ) {
        // Implicit fast uses the same integration formula but with
        // accelerations that were computed without Coriolis terms
        // Guard against degenerate divisors (negative damping could cause issues)
        let linear_divisor = (1.0 + dt * linear_damping).max(1e-10);
        let angular_divisor = (1.0 + dt * angular_damping).max(1e-10);

        state.twist.linear = (state.twist.linear + linear_accel * dt) / linear_divisor;
        state.twist.angular = (state.twist.angular + angular_accel * dt) / angular_divisor;

        state.pose.position += state.twist.linear * dt;
        integrate_rotation(&mut state.pose.rotation, &state.twist.angular, dt);
    }

    /// Integrate without explicitly handling Coriolis (the main use case).
    ///
    /// The acceleration passed should already have Coriolis terms omitted
    /// from the equations of motion computation.
    pub fn integrate_no_coriolis(
        state: &mut RigidBodyState,
        linear_accel_no_coriolis: Vector3<f64>,
        angular_accel_no_coriolis: Vector3<f64>,
        dt: f64,
    ) {
        Self::integrate_with_damping(
            state,
            linear_accel_no_coriolis,
            angular_accel_no_coriolis,
            0.0,
            0.0,
            dt,
        );
    }
}

impl Integrator for ImplicitFast {
    fn integrate(
        state: &mut RigidBodyState,
        linear_accel: Vector3<f64>,
        angular_accel: Vector3<f64>,
        dt: f64,
    ) {
        // For the trait implementation, we assume the caller has already
        // computed accelerations without Coriolis terms
        Self::integrate_no_coriolis(state, linear_accel, angular_accel, dt);
    }
}

/// Integrate rotation using angular velocity.
///
/// Uses the exponential map approximation for small angles:
/// q(t+dt) ≈ q(t) * exp(0.5 * omega * dt)
///
/// For small omega*dt, this is accurate and avoids quaternion drift.
fn integrate_rotation(rotation: &mut UnitQuaternion<f64>, omega: &Vector3<f64>, dt: f64) {
    let angle = omega.norm();

    if angle < 1e-10 {
        // No rotation needed
        return;
    }

    // Create rotation quaternion from angular velocity
    // The rotation is omega * dt around the axis omega/|omega|
    // Create quaternion from scaled axis (omega * dt gives the rotation vector)
    let delta_q = UnitQuaternion::from_scaled_axis(omega * dt);

    // Apply rotation: q_new = q_old * delta_q
    *rotation *= delta_q;
}

/// Damping utility for applying velocity damping.
#[must_use]
pub fn apply_damping(twist: &Twist, linear_damping: f64, angular_damping: f64, dt: f64) -> Twist {
    // Exponential damping: v *= exp(-damping * dt) ≈ 1 - damping * dt for small dt
    let linear_factor = (-linear_damping * dt).exp();
    let angular_factor = (-angular_damping * dt).exp();

    Twist::new(twist.linear * linear_factor, twist.angular * angular_factor)
}

/// Clamp velocities to maximum values.
#[must_use]
pub fn clamp_velocities(twist: &Twist, max_linear: f64, max_angular: f64) -> Twist {
    let linear_norm = twist.linear.norm();
    let linear = if linear_norm > max_linear && linear_norm > 1e-10 {
        twist.linear * (max_linear / linear_norm)
    } else {
        twist.linear
    };

    let angular_norm = twist.angular.norm();
    let angular = if angular_norm > max_angular && angular_norm > 1e-10 {
        twist.angular * (max_angular / angular_norm)
    } else {
        twist.angular
    };

    Twist::new(linear, angular)
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
    use nalgebra::Point3;
    use sim_types::Pose;

    fn make_state_at_origin() -> RigidBodyState {
        RigidBodyState::at_rest(Pose::from_position(Point3::origin()))
    }

    fn make_state_with_velocity(v: Vector3<f64>) -> RigidBodyState {
        RigidBodyState::new(Pose::from_position(Point3::origin()), Twist::linear(v))
    }

    #[test]
    fn test_explicit_euler_constant_velocity() {
        let mut state = make_state_with_velocity(Vector3::new(1.0, 0.0, 0.0));
        let accel = Vector3::zeros();

        ExplicitEuler::integrate(&mut state, accel, Vector3::zeros(), 1.0);

        assert_relative_eq!(state.pose.position.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(state.twist.linear.x, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_explicit_euler_with_acceleration() {
        let mut state = make_state_at_origin();
        let accel = Vector3::new(0.0, 0.0, -9.81);

        ExplicitEuler::integrate(&mut state, accel, Vector3::zeros(), 1.0);

        // After 1 second: position = 0 (velocity was 0), velocity = -9.81
        assert_relative_eq!(state.pose.position.z, 0.0, epsilon = 1e-10);
        assert_relative_eq!(state.twist.linear.z, -9.81, epsilon = 1e-10);
    }

    #[test]
    fn test_semi_implicit_euler_with_acceleration() {
        let mut state = make_state_at_origin();
        let accel = Vector3::new(0.0, 0.0, -9.81);

        SemiImplicitEuler::integrate(&mut state, accel, Vector3::zeros(), 1.0);

        // After 1 second: velocity updated first to -9.81, then position = -9.81
        assert_relative_eq!(state.pose.position.z, -9.81, epsilon = 1e-10);
        assert_relative_eq!(state.twist.linear.z, -9.81, epsilon = 1e-10);
    }

    #[test]
    fn test_velocity_verlet() {
        let mut state = make_state_at_origin();
        let accel = Vector3::new(0.0, 0.0, -10.0);

        VelocityVerlet::integrate(&mut state, accel, Vector3::zeros(), 1.0);

        // x = 0 + 0*1 + 0.5*(-10)*1² = -5
        // v = 0 + (-10)*1 = -10
        assert_relative_eq!(state.pose.position.z, -5.0, epsilon = 1e-10);
        assert_relative_eq!(state.twist.linear.z, -10.0, epsilon = 1e-10);
    }

    #[test]
    fn test_rk4() {
        let mut state = make_state_at_origin();
        let accel = Vector3::new(0.0, 0.0, -10.0);

        RungeKutta4::integrate(&mut state, accel, Vector3::zeros(), 1.0);

        // Same as Verlet for constant acceleration
        assert_relative_eq!(state.pose.position.z, -5.0, epsilon = 1e-10);
        assert_relative_eq!(state.twist.linear.z, -10.0, epsilon = 1e-10);
    }

    #[test]
    fn test_rotation_integration() {
        let mut state = RigidBodyState::new(
            Pose::identity(),
            Twist::angular(Vector3::new(0.0, 0.0, std::f64::consts::PI)),
        );

        // Rotate for 1 second at PI rad/s = 180 degrees
        SemiImplicitEuler::integrate(&mut state, Vector3::zeros(), Vector3::zeros(), 1.0);

        // Forward direction should be rotated 180 degrees around Z
        let forward = state.pose.forward();
        assert_relative_eq!(forward.y, -1.0, epsilon = 1e-5);
    }

    #[test]
    fn test_small_rotation() {
        let mut state = RigidBodyState::new(
            Pose::identity(),
            Twist::angular(Vector3::new(0.0, 0.0, 0.1)),
        );

        // Small rotation
        SemiImplicitEuler::integrate(&mut state, Vector3::zeros(), Vector3::zeros(), 0.01);

        // Should still be normalized
        assert_relative_eq!(state.pose.rotation.norm(), 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_apply_damping() {
        let twist = Twist::new(Vector3::new(10.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 5.0));
        let damped = apply_damping(&twist, 1.0, 0.5, 0.1);

        // Damping should reduce velocities
        assert!(damped.linear.x < twist.linear.x);
        assert!(damped.angular.z < twist.angular.z);
    }

    #[test]
    fn test_clamp_velocities() {
        let twist = Twist::new(Vector3::new(100.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 50.0));
        let clamped = clamp_velocities(&twist, 10.0, 5.0);

        assert_relative_eq!(clamped.linear.norm(), 10.0, epsilon = 1e-10);
        assert_relative_eq!(clamped.angular.norm(), 5.0, epsilon = 1e-10);
    }

    #[test]
    fn test_clamp_velocities_under_limit() {
        let twist = Twist::new(Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        let clamped = clamp_velocities(&twist, 10.0, 10.0);

        // Should be unchanged
        assert_relative_eq!(clamped.linear, twist.linear, epsilon = 1e-10);
        assert_relative_eq!(clamped.angular, twist.angular, epsilon = 1e-10);
    }

    #[test]
    fn test_integrate_with_method_dispatch() {
        for method in [
            IntegrationMethod::ExplicitEuler,
            IntegrationMethod::SemiImplicitEuler,
            IntegrationMethod::VelocityVerlet,
            IntegrationMethod::RungeKutta4,
            IntegrationMethod::ImplicitVelocity,
            IntegrationMethod::ImplicitFast,
        ] {
            let mut state = make_state_with_velocity(Vector3::new(1.0, 0.0, 0.0));
            integrate_with_method(method, &mut state, Vector3::zeros(), Vector3::zeros(), 0.1);

            // All methods should advance position with constant velocity
            assert_relative_eq!(state.pose.position.x, 0.1, epsilon = 1e-10);
        }
    }

    #[test]
    fn test_implicit_fast_matches_implicit_velocity() {
        // For simple cases (no Coriolis), ImplicitFast should match ImplicitVelocity
        let mut state_fast = make_state_at_origin();
        let mut state_velocity = make_state_at_origin();
        let accel = Vector3::new(0.0, 0.0, -9.81);

        ImplicitFast::integrate(&mut state_fast, accel, Vector3::zeros(), 0.01);
        ImplicitVelocity::integrate(&mut state_velocity, accel, Vector3::zeros(), 0.01);

        assert_relative_eq!(
            state_fast.pose.position.z,
            state_velocity.pose.position.z,
            epsilon = 1e-10
        );
        assert_relative_eq!(
            state_fast.twist.linear.z,
            state_velocity.twist.linear.z,
            epsilon = 1e-10
        );
    }

    #[test]
    fn test_implicit_fast_with_damping() {
        let mut state = make_state_with_velocity(Vector3::new(10.0, 0.0, 0.0));
        let accel = Vector3::zeros();

        ImplicitFast::integrate_with_damping(
            &mut state,
            accel,
            Vector3::zeros(),
            10.0, // linear damping
            0.0,  // angular damping
            1.0,  // dt
        );

        // Same formula as ImplicitVelocity: v = (v + h*a) / (1 + h*d) = 10 / 11
        assert_relative_eq!(state.twist.linear.x, 10.0 / 11.0, epsilon = 1e-10);
    }

    #[test]
    fn test_energy_conservation_symplectic() {
        // Test that symplectic integrators conserve energy better for oscillator
        // This is a simple spring system: F = -k*x, so a = -k*x/m
        // With k=1, m=1: a = -x

        let mut state_euler = RigidBodyState::new(
            Pose::from_position(Point3::new(1.0, 0.0, 0.0)),
            Twist::zero(),
        );
        let mut state_symplectic = state_euler;

        let initial_energy = 0.5; // Potential energy = 0.5 * k * x² = 0.5

        // Simulate for many steps
        let dt = 0.01;
        for _ in 0..1000 {
            // Spring acceleration: a = -x
            let accel_euler = Vector3::new(-state_euler.pose.position.x, 0.0, 0.0);
            ExplicitEuler::integrate(&mut state_euler, accel_euler, Vector3::zeros(), dt);

            let accel_symp = Vector3::new(-state_symplectic.pose.position.x, 0.0, 0.0);
            SemiImplicitEuler::integrate(&mut state_symplectic, accel_symp, Vector3::zeros(), dt);
        }

        // Compute energies (E = 0.5*v² + 0.5*x²)
        let energy_euler = 0.5 * state_euler.twist.linear.norm_squared()
            + 0.5 * state_euler.pose.position.x.powi(2);
        let energy_symplectic = 0.5 * state_symplectic.twist.linear.norm_squared()
            + 0.5 * state_symplectic.pose.position.x.powi(2);

        // Symplectic should conserve energy better
        let drift_euler = (energy_euler - initial_energy).abs();
        let drift_symplectic = (energy_symplectic - initial_energy).abs();

        // The symplectic integrator should have much less energy drift
        assert!(
            drift_symplectic < drift_euler * 0.1,
            "Symplectic drift {} should be much less than Euler drift {}",
            drift_symplectic,
            drift_euler
        );
    }

    // =========================================================================
    // Implicit Velocity Integration Tests
    // =========================================================================

    #[test]
    fn test_implicit_velocity_no_damping() {
        // Without damping, implicit velocity should behave like semi-implicit Euler
        let mut state_implicit = make_state_at_origin();
        let mut state_semi = make_state_at_origin();
        let accel = Vector3::new(0.0, 0.0, -9.81);

        ImplicitVelocity::integrate(&mut state_implicit, accel, Vector3::zeros(), 1.0);
        SemiImplicitEuler::integrate(&mut state_semi, accel, Vector3::zeros(), 1.0);

        assert_relative_eq!(
            state_implicit.pose.position.z,
            state_semi.pose.position.z,
            epsilon = 1e-10
        );
        assert_relative_eq!(
            state_implicit.twist.linear.z,
            state_semi.twist.linear.z,
            epsilon = 1e-10
        );
    }

    #[test]
    fn test_implicit_velocity_with_damping() {
        let mut state = make_state_with_velocity(Vector3::new(10.0, 0.0, 0.0));
        let accel = Vector3::zeros();

        // Apply heavy damping
        ImplicitVelocity::integrate_with_damping(
            &mut state,
            accel,
            Vector3::zeros(),
            10.0, // linear damping
            0.0,  // angular damping
            1.0,  // dt
        );

        // With damping=10 and dt=1:
        // v_{t+h} = (v_t + h * a) / (1 + h * d) = 10 / (1 + 10) = 10/11 ≈ 0.909
        assert_relative_eq!(state.twist.linear.x, 10.0 / 11.0, epsilon = 1e-10);
    }

    #[test]
    fn test_implicit_velocity_stability_high_damping() {
        // Test that implicit method remains stable with extreme damping
        // Explicit methods would become unstable here
        let mut state = make_state_with_velocity(Vector3::new(100.0, 0.0, 0.0));

        // Extremely high damping that would cause explicit methods to explode
        let damping = 1000.0;
        let dt = 0.1;

        ImplicitVelocity::integrate_with_damping(
            &mut state,
            Vector3::zeros(),
            Vector3::zeros(),
            damping,
            0.0,
            dt,
        );

        // Velocity should decrease (not explode)
        assert!(state.twist.linear.x < 100.0);
        assert!(state.twist.linear.x > 0.0);
        assert!(!state.twist.linear.x.is_nan());
        assert!(!state.twist.linear.x.is_infinite());
    }

    #[test]
    fn test_implicit_velocity_convergence() {
        // With very high damping, velocity should approach zero
        let mut state = make_state_with_velocity(Vector3::new(100.0, 0.0, 0.0));

        let damping = 100.0;
        let dt = 0.01;

        // Simulate for many steps
        for _ in 0..1000 {
            ImplicitVelocity::integrate_with_damping(
                &mut state,
                Vector3::zeros(),
                Vector3::zeros(),
                damping,
                0.0,
                dt,
            );
        }

        // Velocity should have converged to near zero
        assert!(
            state.twist.linear.x.abs() < 1e-6,
            "Velocity should converge to zero, got {}",
            state.twist.linear.x
        );
    }

    #[test]
    fn test_implicit_velocity_with_force_and_damping() {
        // Test equilibrium: constant force with damping reaches terminal velocity
        let mut state = make_state_at_origin();
        let force_accel = Vector3::new(10.0, 0.0, 0.0); // Acceleration from constant force
        let damping = 2.0;
        let dt = 0.01;

        // Simulate until equilibrium
        for _ in 0..10000 {
            ImplicitVelocity::integrate_with_damping(
                &mut state,
                force_accel,
                Vector3::zeros(),
                damping,
                0.0,
                dt,
            );
        }

        // At equilibrium: a = d * v_terminal => v_terminal = a / d = 10 / 2 = 5
        assert_relative_eq!(state.twist.linear.x, 5.0, epsilon = 0.01);
    }

    #[test]
    fn test_implicit_velocity_angular_damping() {
        let mut state = RigidBodyState::new(
            Pose::identity(),
            Twist::angular(Vector3::new(0.0, 0.0, 10.0)),
        );

        // Apply angular damping
        ImplicitVelocity::integrate_with_damping(
            &mut state,
            Vector3::zeros(),
            Vector3::zeros(),
            0.0,  // linear damping
            10.0, // angular damping
            1.0,  // dt
        );

        // Angular velocity should be damped: 10 / (1 + 10) = 10/11
        assert_relative_eq!(state.twist.angular.z, 10.0 / 11.0, epsilon = 1e-10);
    }

    #[test]
    fn test_integrate_with_method_dispatch_includes_implicit() {
        // Test that ImplicitVelocity is included in dispatch
        let mut state = make_state_with_velocity(Vector3::new(1.0, 0.0, 0.0));
        integrate_with_method(
            IntegrationMethod::ImplicitVelocity,
            &mut state,
            Vector3::zeros(),
            Vector3::zeros(),
            0.1,
        );

        // Should advance position with constant velocity
        assert_relative_eq!(state.pose.position.x, 0.1, epsilon = 1e-10);
    }

    #[test]
    fn test_integrate_with_method_and_damping() {
        let mut state_implicit = make_state_with_velocity(Vector3::new(10.0, 0.0, 0.0));
        let mut state_semi = make_state_with_velocity(Vector3::new(10.0, 0.0, 0.0));

        // Implicit with damping should incorporate damping into the solve
        integrate_with_method_and_damping(
            IntegrationMethod::ImplicitVelocity,
            &mut state_implicit,
            Vector3::zeros(),
            Vector3::zeros(),
            5.0, // linear damping
            0.0,
            1.0,
        );

        // Semi-implicit should just integrate (damping applied separately)
        integrate_with_method_and_damping(
            IntegrationMethod::SemiImplicitEuler,
            &mut state_semi,
            Vector3::zeros(),
            Vector3::zeros(),
            5.0, // ignored for non-implicit
            0.0,
            1.0,
        );

        // Implicit should have damped velocity: 10 / (1 + 5) = 10/6 ≈ 1.667
        assert_relative_eq!(state_implicit.twist.linear.x, 10.0 / 6.0, epsilon = 1e-10);

        // Semi-implicit should have unchanged velocity (damping not applied in integrator)
        assert_relative_eq!(state_semi.twist.linear.x, 10.0, epsilon = 1e-10);
    }

    #[test]
    fn test_implicit_vs_explicit_stability() {
        // Demonstrate that implicit method handles stiff damping better
        // Use a moderately stiff system that explicit methods struggle with

        let initial_vel = Vector3::new(100.0, 0.0, 0.0);
        let stiff_damping = 50.0; // High damping coefficient
        let large_dt = 0.1; // Large timestep

        // Implicit method
        let mut state_implicit = make_state_with_velocity(initial_vel);
        for _ in 0..100 {
            ImplicitVelocity::integrate_with_damping(
                &mut state_implicit,
                Vector3::zeros(),
                Vector3::zeros(),
                stiff_damping,
                0.0,
                large_dt,
            );
        }

        // Explicit with separate damping
        let mut state_explicit = make_state_with_velocity(initial_vel);
        for _ in 0..100 {
            SemiImplicitEuler::integrate(
                &mut state_explicit,
                Vector3::zeros(),
                Vector3::zeros(),
                large_dt,
            );
            // Apply damping explicitly (this can cause instability)
            state_explicit.twist =
                apply_damping(&state_explicit.twist, stiff_damping, 0.0, large_dt);
        }

        // Both should converge, but implicit should be more stable
        // With large dt and high damping, explicit damping overshoots
        assert!(
            !state_implicit.twist.linear.x.is_nan(),
            "Implicit should not produce NaN"
        );
        assert!(
            state_implicit.twist.linear.x >= 0.0,
            "Implicit velocity should stay positive or zero"
        );

        // Implicit should have smoothly converged to near zero
        assert!(
            state_implicit.twist.linear.x.abs() < 1e-10,
            "Implicit should converge to zero"
        );
    }
}
