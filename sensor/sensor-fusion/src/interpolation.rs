//! Temporal interpolation for sensor readings.

use serde::{Deserialize, Serialize};

use crate::buffer::StreamBuffer;
use crate::error::{FusionError, Result};

/// Method for interpolating between sensor readings.
///
/// # Example
///
/// ```
/// use sensor_fusion::InterpolationMethod;
///
/// let method = InterpolationMethod::Linear;
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub enum InterpolationMethod {
    /// Linear interpolation between nearest readings.
    #[default]
    Linear,

    /// Use the nearest reading (no interpolation).
    Nearest,

    /// Use the previous reading (zero-order hold).
    Previous,

    /// Use the next reading.
    Next,
}

impl InterpolationMethod {
    /// Returns true if this method requires two readings.
    #[must_use]
    pub const fn requires_two_readings(&self) -> bool {
        matches!(self, Self::Linear)
    }
}

/// Interpolator for sensor readings.
///
/// Provides temporal interpolation of scalar and vector values.
///
/// # Example
///
/// ```
/// use sensor_fusion::{Interpolator, InterpolationMethod, StreamBuffer};
///
/// let interpolator = Interpolator::new(InterpolationMethod::Linear);
///
/// let mut buffer = StreamBuffer::new(100);
/// buffer.push(0.0, 0.0);
/// buffer.push(1.0, 10.0);
///
/// let value = interpolator.interpolate(&buffer, 0.5).unwrap();
/// assert!((value - 5.0).abs() < 1e-6);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Interpolator {
    method: InterpolationMethod,
}

impl Default for Interpolator {
    fn default() -> Self {
        Self::new(InterpolationMethod::Linear)
    }
}

impl Interpolator {
    /// Creates a new interpolator with the given method.
    #[must_use]
    pub const fn new(method: InterpolationMethod) -> Self {
        Self { method }
    }

    /// Creates a linear interpolator.
    #[must_use]
    pub const fn linear() -> Self {
        Self::new(InterpolationMethod::Linear)
    }

    /// Creates a nearest-neighbor interpolator.
    #[must_use]
    pub const fn nearest() -> Self {
        Self::new(InterpolationMethod::Nearest)
    }

    /// Returns the interpolation method.
    #[must_use]
    pub const fn method(&self) -> InterpolationMethod {
        self.method
    }

    /// Interpolates a scalar value at the given timestamp.
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - The buffer is empty
    /// - The timestamp is outside the buffer range
    pub fn interpolate(&self, buffer: &StreamBuffer<f64>, timestamp: f64) -> Result<f64> {
        if buffer.is_empty() {
            return Err(FusionError::insufficient_data("buffer is empty"));
        }

        let (min, max) = buffer
            .timestamp_range()
            .ok_or_else(|| FusionError::insufficient_data("no timestamp range"))?;

        if timestamp < min || timestamp > max {
            return Err(FusionError::timestamp_out_of_range(timestamp, min, max));
        }

        let (idx_before, idx_after) = buffer
            .find_bracket(timestamp)
            .ok_or_else(|| FusionError::insufficient_data("cannot bracket timestamp"))?;

        let before = buffer
            .get(idx_before)
            .ok_or_else(|| FusionError::insufficient_data("missing before reading"))?;
        let after = buffer
            .get(idx_after)
            .ok_or_else(|| FusionError::insufficient_data("missing after reading"))?;

        match self.method {
            InterpolationMethod::Linear => {
                if idx_before == idx_after {
                    Ok(before.1)
                } else {
                    let t = (timestamp - before.0) / (after.0 - before.0);
                    Ok(t.mul_add(after.1 - before.1, before.1))
                }
            }
            InterpolationMethod::Nearest => {
                let dist_before = (timestamp - before.0).abs();
                let dist_after = (after.0 - timestamp).abs();
                if dist_before <= dist_after {
                    Ok(before.1)
                } else {
                    Ok(after.1)
                }
            }
            InterpolationMethod::Previous => Ok(before.1),
            InterpolationMethod::Next => Ok(after.1),
        }
    }

    /// Interpolates a 3D vector at the given timestamp.
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - The buffer is empty
    /// - The timestamp is outside the buffer range
    pub fn interpolate_vec3(
        &self,
        buffer: &StreamBuffer<[f64; 3]>,
        timestamp: f64,
    ) -> Result<[f64; 3]> {
        if buffer.is_empty() {
            return Err(FusionError::insufficient_data("buffer is empty"));
        }

        let (min, max) = buffer
            .timestamp_range()
            .ok_or_else(|| FusionError::insufficient_data("no timestamp range"))?;

        if timestamp < min || timestamp > max {
            return Err(FusionError::timestamp_out_of_range(timestamp, min, max));
        }

        let (idx_before, idx_after) = buffer
            .find_bracket(timestamp)
            .ok_or_else(|| FusionError::insufficient_data("cannot bracket timestamp"))?;

        let before = buffer
            .get(idx_before)
            .ok_or_else(|| FusionError::insufficient_data("missing before reading"))?;
        let after = buffer
            .get(idx_after)
            .ok_or_else(|| FusionError::insufficient_data("missing after reading"))?;

        match self.method {
            InterpolationMethod::Linear => {
                if idx_before == idx_after {
                    Ok(before.1)
                } else {
                    let t = (timestamp - before.0) / (after.0 - before.0);
                    Ok([
                        t.mul_add(after.1[0] - before.1[0], before.1[0]),
                        t.mul_add(after.1[1] - before.1[1], before.1[1]),
                        t.mul_add(after.1[2] - before.1[2], before.1[2]),
                    ])
                }
            }
            InterpolationMethod::Nearest => {
                let dist_before = (timestamp - before.0).abs();
                let dist_after = (after.0 - timestamp).abs();
                if dist_before <= dist_after {
                    Ok(before.1)
                } else {
                    Ok(after.1)
                }
            }
            InterpolationMethod::Previous => Ok(before.1),
            InterpolationMethod::Next => Ok(after.1),
        }
    }
}

/// Computes linear interpolation factor.
///
/// Returns `t` in `[0, 1]` representing position between `a` and `b`.
#[must_use]
pub fn lerp_factor(a: f64, b: f64, x: f64) -> f64 {
    if (b - a).abs() < f64::EPSILON {
        0.0
    } else {
        (x - a) / (b - a)
    }
}

/// Performs linear interpolation between two values.
#[must_use]
pub fn lerp(a: f64, b: f64, t: f64) -> f64 {
    t.mul_add(b - a, a)
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names
)]
mod tests {
    use super::*;

    #[test]
    fn interpolation_method_default() {
        let method = InterpolationMethod::default();
        assert_eq!(method, InterpolationMethod::Linear);
    }

    #[test]
    fn interpolation_method_requires_two() {
        assert!(InterpolationMethod::Linear.requires_two_readings());
        assert!(!InterpolationMethod::Nearest.requires_two_readings());
        assert!(!InterpolationMethod::Previous.requires_two_readings());
        assert!(!InterpolationMethod::Next.requires_two_readings());
    }

    #[test]
    fn interpolator_default() {
        let interp = Interpolator::default();
        assert_eq!(interp.method(), InterpolationMethod::Linear);
    }

    #[test]
    fn interpolator_linear() {
        let interp = Interpolator::linear();
        assert_eq!(interp.method(), InterpolationMethod::Linear);
    }

    #[test]
    fn interpolator_nearest() {
        let interp = Interpolator::nearest();
        assert_eq!(interp.method(), InterpolationMethod::Nearest);
    }

    #[test]
    fn interpolate_linear_midpoint() {
        let interp = Interpolator::linear();
        let mut buffer = StreamBuffer::new(100);
        buffer.push(0.0, 0.0);
        buffer.push(1.0, 10.0);

        let result = interp.interpolate(&buffer, 0.5).unwrap();
        assert!((result - 5.0).abs() < 1e-6);
    }

    #[test]
    fn interpolate_linear_quarter() {
        let interp = Interpolator::linear();
        let mut buffer = StreamBuffer::new(100);
        buffer.push(0.0, 0.0);
        buffer.push(1.0, 10.0);

        let result = interp.interpolate(&buffer, 0.25).unwrap();
        assert!((result - 2.5).abs() < 1e-6);
    }

    #[test]
    fn interpolate_linear_exact() {
        let interp = Interpolator::linear();
        let mut buffer = StreamBuffer::new(100);
        buffer.push(0.0, 0.0);
        buffer.push(1.0, 10.0);

        let result = interp.interpolate(&buffer, 0.0).unwrap();
        assert!((result - 0.0).abs() < 1e-6);

        let result = interp.interpolate(&buffer, 1.0).unwrap();
        assert!((result - 10.0).abs() < 1e-6);
    }

    #[test]
    fn interpolate_nearest() {
        let interp = Interpolator::nearest();
        let mut buffer = StreamBuffer::new(100);
        buffer.push(0.0, 0.0);
        buffer.push(1.0, 10.0);

        // Closer to 0.0
        let result = interp.interpolate(&buffer, 0.3).unwrap();
        assert!((result - 0.0).abs() < 1e-6);

        // Closer to 1.0
        let result = interp.interpolate(&buffer, 0.7).unwrap();
        assert!((result - 10.0).abs() < 1e-6);
    }

    #[test]
    fn interpolate_previous() {
        let interp = Interpolator::new(InterpolationMethod::Previous);
        let mut buffer = StreamBuffer::new(100);
        buffer.push(0.0, 0.0);
        buffer.push(1.0, 10.0);

        let result = interp.interpolate(&buffer, 0.7).unwrap();
        assert!((result - 0.0).abs() < 1e-6);
    }

    #[test]
    fn interpolate_next() {
        let interp = Interpolator::new(InterpolationMethod::Next);
        let mut buffer = StreamBuffer::new(100);
        buffer.push(0.0, 0.0);
        buffer.push(1.0, 10.0);

        let result = interp.interpolate(&buffer, 0.3).unwrap();
        assert!((result - 10.0).abs() < 1e-6);
    }

    #[test]
    fn interpolate_empty_buffer() {
        let interp = Interpolator::linear();
        let buffer: StreamBuffer<f64> = StreamBuffer::new(100);

        let result = interp.interpolate(&buffer, 0.5);
        assert!(result.is_err());
    }

    #[test]
    fn interpolate_out_of_range() {
        let interp = Interpolator::linear();
        let mut buffer = StreamBuffer::new(100);
        buffer.push(0.0, 0.0);
        buffer.push(1.0, 10.0);

        let result = interp.interpolate(&buffer, -1.0);
        assert!(result.is_err());

        let result = interp.interpolate(&buffer, 2.0);
        assert!(result.is_err());
    }

    #[test]
    fn interpolate_vec3_linear() {
        let interp = Interpolator::linear();
        let mut buffer = StreamBuffer::new(100);
        buffer.push(0.0, [0.0, 0.0, 0.0]);
        buffer.push(1.0, [10.0, 20.0, 30.0]);

        let result = interp.interpolate_vec3(&buffer, 0.5).unwrap();
        assert!((result[0] - 5.0).abs() < 1e-6);
        assert!((result[1] - 10.0).abs() < 1e-6);
        assert!((result[2] - 15.0).abs() < 1e-6);
    }

    #[test]
    fn lerp_factor_basic() {
        assert!((lerp_factor(0.0, 10.0, 5.0) - 0.5).abs() < 1e-6);
        assert!((lerp_factor(0.0, 10.0, 0.0) - 0.0).abs() < 1e-6);
        assert!((lerp_factor(0.0, 10.0, 10.0) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn lerp_basic() {
        assert!((lerp(0.0, 10.0, 0.5) - 5.0).abs() < 1e-6);
        assert!((lerp(0.0, 10.0, 0.0) - 0.0).abs() < 1e-6);
        assert!((lerp(0.0, 10.0, 1.0) - 10.0).abs() < 1e-6);
    }
}
