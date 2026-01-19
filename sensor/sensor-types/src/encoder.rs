//! Encoder and joint state types.
//!
//! Provides types for joint positions, velocities, and efforts from encoders.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::Timestamp;

/// A reading from encoders reporting joint states.
///
/// Used for robot arms, mobile bases, or any system with articulated joints.
/// Supports position, velocity, and effort measurements.
///
/// # Units
///
/// - Position: radians (revolute) or meters (prismatic)
/// - Velocity: radians/second or meters/second
/// - Effort: Newton-meters or Newtons
///
/// # Example
///
/// ```
/// use sensor_types::{EncoderReading, Timestamp};
///
/// let reading = EncoderReading {
///     timestamp: Timestamp::from_secs_f64(1.0),
///     positions: vec![0.0, 1.57, 0.0, -1.57, 0.0, 0.0],
///     velocities: Some(vec![0.0; 6]),
///     efforts: None,
///     names: Some(vec![
///         "joint1".to_string(),
///         "joint2".to_string(),
///         "joint3".to_string(),
///         "joint4".to_string(),
///         "joint5".to_string(),
///         "joint6".to_string(),
///     ]),
/// };
///
/// assert_eq!(reading.joint_count(), 6);
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct EncoderReading {
    /// Timestamp of the reading.
    pub timestamp: Timestamp,

    /// Joint positions in radians (revolute) or meters (prismatic).
    pub positions: Vec<f64>,

    /// Joint velocities (optional).
    pub velocities: Option<Vec<f64>>,

    /// Joint efforts/torques (optional).
    pub efforts: Option<Vec<f64>>,

    /// Joint names (optional, for debugging/logging).
    pub names: Option<Vec<String>>,
}

impl EncoderReading {
    /// Creates a new encoder reading with only positions.
    #[must_use]
    pub const fn new(timestamp: Timestamp, positions: Vec<f64>) -> Self {
        Self {
            timestamp,
            positions,
            velocities: None,
            efforts: None,
            names: None,
        }
    }

    /// Creates an encoder reading with positions and velocities.
    #[must_use]
    pub const fn with_velocities(
        timestamp: Timestamp,
        positions: Vec<f64>,
        velocities: Vec<f64>,
    ) -> Self {
        Self {
            timestamp,
            positions,
            velocities: Some(velocities),
            efforts: None,
            names: None,
        }
    }

    /// Returns the number of joints.
    #[must_use]
    pub fn joint_count(&self) -> usize {
        self.positions.len()
    }

    /// Checks if the reading is empty (no joints).
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.positions.is_empty()
    }

    /// Gets the position of a joint by index.
    ///
    /// Returns `None` if the index is out of bounds.
    #[must_use]
    pub fn position(&self, index: usize) -> Option<f64> {
        self.positions.get(index).copied()
    }

    /// Gets the velocity of a joint by index.
    ///
    /// Returns `None` if velocities aren't available or index is out of bounds.
    #[must_use]
    pub fn velocity(&self, index: usize) -> Option<f64> {
        self.velocities.as_ref()?.get(index).copied()
    }

    /// Gets the effort of a joint by index.
    ///
    /// Returns `None` if efforts aren't available or index is out of bounds.
    #[must_use]
    pub fn effort(&self, index: usize) -> Option<f64> {
        self.efforts.as_ref()?.get(index).copied()
    }

    /// Gets the index of a joint by name.
    ///
    /// Returns `None` if names aren't available or the joint isn't found.
    #[must_use]
    pub fn joint_index(&self, name: &str) -> Option<usize> {
        self.names.as_ref()?.iter().position(|n| n == name)
    }

    /// Gets the position of a joint by name.
    ///
    /// Returns `None` if names aren't available or the joint isn't found.
    #[must_use]
    pub fn position_by_name(&self, name: &str) -> Option<f64> {
        let index = self.joint_index(name)?;
        self.position(index)
    }

    /// Checks if all vectors have consistent lengths.
    #[must_use]
    pub fn is_consistent(&self) -> bool {
        let n = self.positions.len();

        if let Some(ref v) = self.velocities {
            if v.len() != n {
                return false;
            }
        }

        if let Some(ref e) = self.efforts {
            if e.len() != n {
                return false;
            }
        }

        if let Some(ref names) = self.names {
            if names.len() != n {
                return false;
            }
        }

        true
    }

    /// Creates a subset of joints by indices.
    #[must_use]
    pub fn select(&self, indices: &[usize]) -> Self {
        let positions: Vec<f64> = indices
            .iter()
            .filter_map(|&i| self.positions.get(i).copied())
            .collect();

        let velocities = self
            .velocities
            .as_ref()
            .map(|v| indices.iter().filter_map(|&i| v.get(i).copied()).collect());

        let efforts = self
            .efforts
            .as_ref()
            .map(|e| indices.iter().filter_map(|&i| e.get(i).copied()).collect());

        let names = self
            .names
            .as_ref()
            .map(|n| indices.iter().filter_map(|&i| n.get(i).cloned()).collect());

        Self {
            timestamp: self.timestamp,
            positions,
            velocities,
            efforts,
            names,
        }
    }

    /// Computes the maximum absolute velocity across all joints.
    ///
    /// Returns `None` if velocities aren't available.
    #[must_use]
    pub fn max_velocity(&self) -> Option<f64> {
        self.velocities
            .as_ref()
            .map(|v| v.iter().map(|x| x.abs()).fold(0.0, f64::max))
    }

    /// Computes the maximum absolute effort across all joints.
    ///
    /// Returns `None` if efforts aren't available.
    #[must_use]
    pub fn max_effort(&self) -> Option<f64> {
        self.efforts
            .as_ref()
            .map(|e| e.iter().map(|x| x.abs()).fold(0.0, f64::max))
    }
}

impl Default for EncoderReading {
    fn default() -> Self {
        Self::new(Timestamp::zero(), Vec::new())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_reading() -> EncoderReading {
        EncoderReading {
            timestamp: Timestamp::from_secs_f64(1.0),
            positions: vec![0.0, 1.0, 2.0],
            velocities: Some(vec![0.1, 0.2, 0.3]),
            efforts: Some(vec![1.0, 2.0, 3.0]),
            names: Some(vec!["a".to_string(), "b".to_string(), "c".to_string()]),
        }
    }

    #[test]
    fn encoder_joint_count() {
        let reading = sample_reading();
        assert_eq!(reading.joint_count(), 3);
        assert!(!reading.is_empty());
    }

    #[test]
    fn encoder_access_by_index() {
        let reading = sample_reading();
        assert!((reading.position(1).unwrap_or(0.0) - 1.0).abs() < 1e-10);
        assert!((reading.velocity(1).unwrap_or(0.0) - 0.2).abs() < 1e-10);
        assert!((reading.effort(1).unwrap_or(0.0) - 2.0).abs() < 1e-10);
    }

    #[test]
    fn encoder_access_by_name() {
        let reading = sample_reading();
        assert_eq!(reading.joint_index("b"), Some(1));
        assert!((reading.position_by_name("b").unwrap_or(0.0) - 1.0).abs() < 1e-10);
        assert!(reading.joint_index("nonexistent").is_none());
    }

    #[test]
    fn encoder_consistency() {
        let good = sample_reading();
        assert!(good.is_consistent());

        let bad = EncoderReading {
            timestamp: Timestamp::zero(),
            positions: vec![0.0, 1.0],
            velocities: Some(vec![0.1]), // Wrong length
            efforts: None,
            names: None,
        };
        assert!(!bad.is_consistent());
    }

    #[test]
    fn encoder_select() {
        let reading = sample_reading();
        let subset = reading.select(&[0, 2]);

        assert_eq!(subset.joint_count(), 2);
        assert!((subset.position(0).unwrap_or(0.0) - 0.0).abs() < 1e-10);
        assert!((subset.position(1).unwrap_or(0.0) - 2.0).abs() < 1e-10);
    }

    #[test]
    fn encoder_max_values() {
        let reading = sample_reading();
        assert!((reading.max_velocity().unwrap_or(0.0) - 0.3).abs() < 1e-10);
        assert!((reading.max_effort().unwrap_or(0.0) - 3.0).abs() < 1e-10);
    }

    #[test]
    fn encoder_without_optional() {
        let reading = EncoderReading::new(Timestamp::zero(), vec![1.0, 2.0]);
        assert!(reading.velocity(0).is_none());
        assert!(reading.effort(0).is_none());
        assert!(reading.max_velocity().is_none());
        assert!(reading.joint_index("any").is_none());
    }

    #[cfg(feature = "serde")]
    #[test]
    fn encoder_serialization() {
        let reading = sample_reading();
        let json = serde_json::to_string(&reading).ok();
        assert!(json.is_some());

        let parsed: Result<EncoderReading, _> = serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
    }
}
