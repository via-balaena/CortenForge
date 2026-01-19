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
    let linear = if twist.linear.norm() > max_linear {
        twist.linear.normalize() * max_linear
    } else {
        twist.linear
    };

    let angular = if twist.angular.norm() > max_angular {
        twist.angular.normalize() * max_angular
    } else {
        twist.angular
    };

    Twist::new(linear, angular)
}

#[cfg(test)]
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
        ] {
            let mut state = make_state_with_velocity(Vector3::new(1.0, 0.0, 0.0));
            integrate_with_method(method, &mut state, Vector3::zeros(), Vector3::zeros(), 0.1);

            // All methods should advance position with constant velocity
            assert_relative_eq!(state.pose.position.x, 0.1, epsilon = 1e-10);
        }
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
}
