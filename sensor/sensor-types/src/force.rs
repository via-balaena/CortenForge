//! Force/torque sensor types.
//!
//! Provides types for 6-axis force/torque sensors commonly used in robotics.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::{CoordinateFrame, Timestamp};

/// A reading from a 6-axis force/torque sensor.
///
/// Measures forces and torques in 3D, typically used for:
/// - Robot end-effector sensing
/// - Contact force measurement
/// - Impedance control
///
/// # Units
///
/// - Force: Newtons (N)
/// - Torque: Newton-meters (N⋅m)
///
/// # Example
///
/// ```
/// use sensor_types::{ForceTorqueReading, CoordinateFrame, Timestamp};
///
/// let reading = ForceTorqueReading {
///     timestamp: Timestamp::from_secs_f64(1.0),
///     force: [0.0, 0.0, -10.0],   // 10N downward force
///     torque: [0.0, 0.0, 0.0],
///     frame: CoordinateFrame::sensor("wrist_ft"),
/// };
///
/// assert!((reading.force_magnitude() - 10.0).abs() < 1e-6);
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ForceTorqueReading {
    /// Timestamp of the reading.
    pub timestamp: Timestamp,

    /// Force vector in Newtons: `[Fx, Fy, Fz]`.
    pub force: [f64; 3],

    /// Torque vector in Newton-meters: `[Tx, Ty, Tz]`.
    pub torque: [f64; 3],

    /// Coordinate frame of the measurement.
    pub frame: CoordinateFrame,
}

impl ForceTorqueReading {
    /// Creates a new force/torque reading.
    #[must_use]
    pub const fn new(
        timestamp: Timestamp,
        force: [f64; 3],
        torque: [f64; 3],
        frame: CoordinateFrame,
    ) -> Self {
        Self {
            timestamp,
            force,
            torque,
            frame,
        }
    }

    /// Creates a zero reading (no force or torque).
    #[must_use]
    pub const fn zero(timestamp: Timestamp) -> Self {
        Self {
            timestamp,
            force: [0.0, 0.0, 0.0],
            torque: [0.0, 0.0, 0.0],
            frame: CoordinateFrame::Body,
        }
    }

    /// Returns the magnitude of the force vector.
    #[must_use]
    pub fn force_magnitude(&self) -> f64 {
        let [x, y, z] = self.force;
        x.hypot(y).hypot(z)
    }

    /// Returns the magnitude of the torque vector.
    #[must_use]
    pub fn torque_magnitude(&self) -> f64 {
        let [x, y, z] = self.torque;
        x.hypot(y).hypot(z)
    }

    /// Returns the normalized force direction.
    ///
    /// Returns `None` if the force magnitude is too small.
    #[must_use]
    pub fn force_direction(&self) -> Option<[f64; 3]> {
        let mag = self.force_magnitude();
        if mag < 1e-10 {
            return None;
        }
        Some([
            self.force[0] / mag,
            self.force[1] / mag,
            self.force[2] / mag,
        ])
    }

    /// Returns the normalized torque direction.
    ///
    /// Returns `None` if the torque magnitude is too small.
    #[must_use]
    pub fn torque_direction(&self) -> Option<[f64; 3]> {
        let mag = self.torque_magnitude();
        if mag < 1e-10 {
            return None;
        }
        Some([
            self.torque[0] / mag,
            self.torque[1] / mag,
            self.torque[2] / mag,
        ])
    }

    /// Checks if the sensor is approximately unloaded.
    ///
    /// Returns `true` if both force and torque magnitudes are below thresholds.
    #[must_use]
    pub fn is_unloaded(&self, force_threshold: f64, torque_threshold: f64) -> bool {
        self.force_magnitude() < force_threshold && self.torque_magnitude() < torque_threshold
    }

    /// Returns the wrench as a 6-element vector: `[Fx, Fy, Fz, Tx, Ty, Tz]`.
    #[must_use]
    pub const fn as_wrench(&self) -> [f64; 6] {
        [
            self.force[0],
            self.force[1],
            self.force[2],
            self.torque[0],
            self.torque[1],
            self.torque[2],
        ]
    }

    /// Creates a reading from a 6-element wrench vector.
    #[must_use]
    pub const fn from_wrench(
        timestamp: Timestamp,
        wrench: [f64; 6],
        frame: CoordinateFrame,
    ) -> Self {
        Self {
            timestamp,
            force: [wrench[0], wrench[1], wrench[2]],
            torque: [wrench[3], wrench[4], wrench[5]],
            frame,
        }
    }

    /// Adds a bias correction (subtracts the bias from the reading).
    #[must_use]
    pub fn bias_corrected(&self, force_bias: [f64; 3], torque_bias: [f64; 3]) -> Self {
        Self {
            timestamp: self.timestamp,
            force: [
                self.force[0] - force_bias[0],
                self.force[1] - force_bias[1],
                self.force[2] - force_bias[2],
            ],
            torque: [
                self.torque[0] - torque_bias[0],
                self.torque[1] - torque_bias[1],
                self.torque[2] - torque_bias[2],
            ],
            frame: self.frame.clone(),
        }
    }
}

impl Default for ForceTorqueReading {
    fn default() -> Self {
        Self::zero(Timestamp::zero())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ft_new() {
        let reading = ForceTorqueReading::new(
            Timestamp::from_secs_f64(1.0),
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 0.5],
            CoordinateFrame::Body,
        );

        assert!((reading.force_magnitude() - 1.0).abs() < 1e-10);
        assert!((reading.torque_magnitude() - 0.5).abs() < 1e-10);
    }

    #[test]
    fn ft_zero() {
        let reading = ForceTorqueReading::zero(Timestamp::zero());
        assert!(reading.force_magnitude() < 1e-10);
        assert!(reading.torque_magnitude() < 1e-10);
    }

    #[test]
    fn ft_direction() {
        let reading = ForceTorqueReading::new(
            Timestamp::zero(),
            [0.0, 0.0, 10.0],
            [0.0, 0.0, 0.0],
            CoordinateFrame::Body,
        );

        let dir = reading.force_direction();
        assert!(dir.is_some());
        let d = dir.unwrap_or([0.0; 3]);
        assert!(d[0].abs() < 1e-10);
        assert!(d[1].abs() < 1e-10);
        assert!((d[2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn ft_direction_zero() {
        let reading = ForceTorqueReading::zero(Timestamp::zero());
        assert!(reading.force_direction().is_none());
        assert!(reading.torque_direction().is_none());
    }

    #[test]
    fn ft_is_unloaded() {
        let loaded = ForceTorqueReading::new(
            Timestamp::zero(),
            [10.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            CoordinateFrame::Body,
        );
        let unloaded = ForceTorqueReading::new(
            Timestamp::zero(),
            [0.001, 0.0, 0.0],
            [0.0, 0.0, 0.001],
            CoordinateFrame::Body,
        );

        assert!(!loaded.is_unloaded(1.0, 1.0));
        assert!(unloaded.is_unloaded(1.0, 1.0));
    }

    #[test]
    #[allow(clippy::float_cmp)] // Exact constant values, no computation
    fn ft_wrench() {
        let reading = ForceTorqueReading::new(
            Timestamp::zero(),
            [1.0, 2.0, 3.0],
            [4.0, 5.0, 6.0],
            CoordinateFrame::Body,
        );

        let wrench = reading.as_wrench();
        assert_eq!(wrench, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);

        let restored =
            ForceTorqueReading::from_wrench(Timestamp::zero(), wrench, CoordinateFrame::Body);
        assert_eq!(restored.force, reading.force);
        assert_eq!(restored.torque, reading.torque);
    }

    #[test]
    fn ft_bias_corrected() {
        let reading = ForceTorqueReading::new(
            Timestamp::zero(),
            [1.0, 2.0, 3.0],
            [0.1, 0.2, 0.3],
            CoordinateFrame::Body,
        );

        let corrected = reading.bias_corrected([1.0, 1.0, 1.0], [0.1, 0.1, 0.1]);
        assert!((corrected.force[0] - 0.0).abs() < 1e-10);
        assert!((corrected.force[1] - 1.0).abs() < 1e-10);
        assert!((corrected.force[2] - 2.0).abs() < 1e-10);
        assert!((corrected.torque[0] - 0.0).abs() < 1e-10);
        assert!((corrected.torque[1] - 0.1).abs() < 1e-10);
        assert!((corrected.torque[2] - 0.2).abs() < 1e-10);
    }

    #[cfg(feature = "serde")]
    #[test]
    fn ft_serialization() {
        let reading = ForceTorqueReading::new(
            Timestamp::zero(),
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            CoordinateFrame::Body,
        );

        let json = serde_json::to_string(&reading).ok();
        assert!(json.is_some());
    }
}
