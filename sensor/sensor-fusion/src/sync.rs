//! Stream synchronization for multiple sensors.

use std::collections::HashMap;

use serde::{Deserialize, Serialize};

use crate::buffer::StreamBuffer;
use crate::error::{FusionError, Result};
use crate::interpolation::{InterpolationMethod, Interpolator};

/// Policy for synchronizing multiple sensor streams.
///
/// # Example
///
/// ```
/// use sensor_fusion::SyncPolicy;
///
/// let policy = SyncPolicy::Interpolate {
///     method: sensor_fusion::InterpolationMethod::Linear,
/// };
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SyncPolicy {
    /// Drop samples that don't have matching timestamps.
    DropUnmatched,

    /// Interpolate missing values.
    Interpolate {
        /// Interpolation method to use.
        method: InterpolationMethod,
    },

    /// Use the nearest available reading.
    Nearest {
        /// Maximum time difference allowed.
        max_delta_secs: u32,
    },
}

impl Default for SyncPolicy {
    fn default() -> Self {
        Self::Interpolate {
            method: InterpolationMethod::Linear,
        }
    }
}

impl SyncPolicy {
    /// Creates an interpolation policy with linear interpolation.
    #[must_use]
    pub const fn linear() -> Self {
        Self::Interpolate {
            method: InterpolationMethod::Linear,
        }
    }

    /// Creates a nearest-neighbor policy.
    #[must_use]
    pub const fn nearest(max_delta_secs: u32) -> Self {
        Self::Nearest { max_delta_secs }
    }
}

/// Result of synchronizing streams at a timestamp.
#[derive(Debug, Clone)]
pub struct SyncResult<T> {
    /// The synchronization timestamp.
    pub timestamp: f64,

    /// Values for each stream (`stream_id` -> value).
    pub values: HashMap<String, T>,

    /// Streams that were interpolated.
    pub interpolated: Vec<String>,

    /// Streams that were missing.
    pub missing: Vec<String>,
}

impl<T> SyncResult<T> {
    /// Returns true if all streams have values.
    #[must_use]
    pub fn is_complete(&self) -> bool {
        self.missing.is_empty()
    }

    /// Returns the number of available values.
    #[must_use]
    pub fn num_values(&self) -> usize {
        self.values.len()
    }

    /// Gets a value by stream ID.
    #[must_use]
    pub fn get(&self, stream_id: &str) -> Option<&T> {
        self.values.get(stream_id)
    }
}

/// Synchronizer for multiple sensor streams.
///
/// Maintains buffers for multiple named streams and provides
/// synchronized access at arbitrary timestamps.
///
/// # Example
///
/// ```
/// use sensor_fusion::{StreamSynchronizer, SyncPolicy};
///
/// let mut sync = StreamSynchronizer::<f64>::new(SyncPolicy::linear());
///
/// // Register streams
/// sync.register_stream("sensor_a", 100);
/// sync.register_stream("sensor_b", 100);
///
/// // Add readings
/// sync.push("sensor_a", 0.0, 1.0);
/// sync.push("sensor_a", 1.0, 2.0);
/// sync.push("sensor_b", 0.0, 10.0);
/// sync.push("sensor_b", 1.0, 20.0);
///
/// // Synchronize at timestamp 0.5
/// let result = sync.synchronize(0.5).unwrap();
/// assert!(result.is_complete());
/// ```
#[derive(Debug)]
pub struct StreamSynchronizer<T> {
    policy: SyncPolicy,
    streams: HashMap<String, StreamBuffer<T>>,
    interpolator: Interpolator,
}

impl<T> Default for StreamSynchronizer<T> {
    fn default() -> Self {
        Self::new(SyncPolicy::default())
    }
}

impl<T> StreamSynchronizer<T> {
    /// Creates a new synchronizer with the given policy.
    #[must_use]
    pub fn new(policy: SyncPolicy) -> Self {
        let method = match policy {
            SyncPolicy::Interpolate { method } => method,
            SyncPolicy::Nearest { .. } | SyncPolicy::DropUnmatched => InterpolationMethod::Nearest,
        };

        Self {
            policy,
            streams: HashMap::new(),
            interpolator: Interpolator::new(method),
        }
    }

    /// Returns the synchronization policy.
    #[must_use]
    pub const fn policy(&self) -> &SyncPolicy {
        &self.policy
    }

    /// Returns the number of registered streams.
    #[must_use]
    pub fn num_streams(&self) -> usize {
        self.streams.len()
    }

    /// Returns the stream IDs.
    #[must_use]
    pub fn stream_ids(&self) -> Vec<String> {
        self.streams.keys().cloned().collect()
    }

    /// Registers a new stream with the given buffer capacity.
    pub fn register_stream(&mut self, stream_id: impl Into<String>, capacity: usize) {
        self.streams
            .insert(stream_id.into(), StreamBuffer::new(capacity));
    }

    /// Removes a stream.
    pub fn remove_stream(&mut self, stream_id: &str) -> bool {
        self.streams.remove(stream_id).is_some()
    }

    /// Returns true if a stream is registered.
    #[must_use]
    pub fn has_stream(&self, stream_id: &str) -> bool {
        self.streams.contains_key(stream_id)
    }

    /// Gets a stream buffer.
    #[must_use]
    pub fn get_stream(&self, stream_id: &str) -> Option<&StreamBuffer<T>> {
        self.streams.get(stream_id)
    }

    /// Pushes a reading to a stream.
    ///
    /// # Errors
    ///
    /// Returns an error if the stream is not registered.
    pub fn push(&mut self, stream_id: &str, timestamp: f64, value: T) -> Result<()> {
        let buffer = self
            .streams
            .get_mut(stream_id)
            .ok_or_else(|| FusionError::stream_not_found(stream_id))?;

        buffer.push(timestamp, value);
        Ok(())
    }

    /// Returns the common time range across all streams.
    ///
    /// Returns the intersection of all stream timestamp ranges.
    #[must_use]
    pub fn common_time_range(&self) -> Option<(f64, f64)> {
        if self.streams.is_empty() {
            return None;
        }

        let mut min = f64::MIN;
        let mut max = f64::MAX;

        for buffer in self.streams.values() {
            if let Some((stream_min, stream_max)) = buffer.timestamp_range() {
                min = min.max(stream_min);
                max = max.min(stream_max);
            } else {
                return None; // Stream is empty
            }
        }

        if min <= max { Some((min, max)) } else { None }
    }

    /// Clears all streams.
    pub fn clear(&mut self) {
        for buffer in self.streams.values_mut() {
            buffer.clear();
        }
    }

    /// Removes readings older than the given timestamp from all streams.
    pub fn remove_before(&mut self, timestamp: f64) {
        for buffer in self.streams.values_mut() {
            buffer.remove_before(timestamp);
        }
    }
}

// Implement synchronization for f64 values
impl StreamSynchronizer<f64> {
    /// Synchronizes all streams at the given timestamp.
    ///
    /// # Errors
    ///
    /// Returns an error if no streams are registered or timestamp is out of range.
    pub fn synchronize(&self, timestamp: f64) -> Result<SyncResult<f64>> {
        if self.streams.is_empty() {
            return Err(FusionError::insufficient_data("no streams registered"));
        }

        let mut values = HashMap::new();
        let mut interpolated = Vec::new();
        let mut missing = Vec::new();

        for (stream_id, buffer) in &self.streams {
            if let Ok(value) = self.interpolator.interpolate(buffer, timestamp) {
                values.insert(stream_id.clone(), value);
                // Check if interpolated (not an exact match)
                if let Some((before, after)) = buffer.find_bracket(timestamp) {
                    if before != after {
                        interpolated.push(stream_id.clone());
                    }
                }
            } else {
                if matches!(self.policy, SyncPolicy::DropUnmatched) {
                    // Skip this stream
                }
                missing.push(stream_id.clone());
            }
        }

        Ok(SyncResult {
            timestamp,
            values,
            interpolated,
            missing,
        })
    }

    /// Generates synchronized samples at regular intervals.
    ///
    /// Returns an iterator over synchronized results.
    ///
    /// # Errors
    ///
    /// Returns an error if no common time range exists.
    pub fn generate_samples(&self, interval: f64) -> Result<Vec<SyncResult<f64>>> {
        let (min, max) = self
            .common_time_range()
            .ok_or_else(|| FusionError::insufficient_data("no common time range"))?;

        let mut results = Vec::new();
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            clippy::cast_precision_loss
        )]
        let num_samples = ((max - min) / interval).floor() as usize + 1;

        for i in 0..num_samples {
            #[allow(clippy::cast_precision_loss)]
            let t = (i as f64).mul_add(interval, min);
            if let Ok(result) = self.synchronize(t) {
                results.push(result);
            }
        }

        Ok(results)
    }
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
    fn sync_policy_default() {
        let policy = SyncPolicy::default();
        assert!(matches!(policy, SyncPolicy::Interpolate { .. }));
    }

    #[test]
    fn sync_policy_linear() {
        let policy = SyncPolicy::linear();
        assert!(matches!(
            policy,
            SyncPolicy::Interpolate {
                method: InterpolationMethod::Linear
            }
        ));
    }

    #[test]
    fn sync_policy_nearest() {
        let policy = SyncPolicy::nearest(100);
        assert!(matches!(
            policy,
            SyncPolicy::Nearest {
                max_delta_secs: 100
            }
        ));
    }

    #[test]
    fn sync_result_complete() {
        let mut values = HashMap::new();
        values.insert("a".to_string(), 1.0);
        values.insert("b".to_string(), 2.0);

        let result = SyncResult {
            timestamp: 0.0,
            values,
            interpolated: vec![],
            missing: vec![],
        };

        assert!(result.is_complete());
        assert_eq!(result.num_values(), 2);
    }

    #[test]
    fn sync_result_incomplete() {
        let mut values = HashMap::new();
        values.insert("a".to_string(), 1.0);

        let result = SyncResult {
            timestamp: 0.0,
            values,
            interpolated: vec![],
            missing: vec!["b".to_string()],
        };

        assert!(!result.is_complete());
    }

    #[test]
    fn synchronizer_new() {
        let sync: StreamSynchronizer<f64> = StreamSynchronizer::new(SyncPolicy::linear());
        assert_eq!(sync.num_streams(), 0);
    }

    #[test]
    fn synchronizer_register_stream() {
        let mut sync: StreamSynchronizer<f64> = StreamSynchronizer::new(SyncPolicy::linear());
        sync.register_stream("sensor_a", 100);

        assert_eq!(sync.num_streams(), 1);
        assert!(sync.has_stream("sensor_a"));
        assert!(!sync.has_stream("sensor_b"));
    }

    #[test]
    fn synchronizer_remove_stream() {
        let mut sync: StreamSynchronizer<f64> = StreamSynchronizer::new(SyncPolicy::linear());
        sync.register_stream("sensor_a", 100);

        assert!(sync.remove_stream("sensor_a"));
        assert!(!sync.has_stream("sensor_a"));
        assert!(!sync.remove_stream("sensor_a")); // Already removed
    }

    #[test]
    fn synchronizer_push() {
        let mut sync: StreamSynchronizer<f64> = StreamSynchronizer::new(SyncPolicy::linear());
        sync.register_stream("sensor_a", 100);

        assert!(sync.push("sensor_a", 0.0, 1.0).is_ok());
        assert!(sync.push("sensor_b", 0.0, 1.0).is_err()); // Not registered
    }

    #[test]
    fn synchronizer_common_time_range() {
        let mut sync: StreamSynchronizer<f64> = StreamSynchronizer::new(SyncPolicy::linear());
        sync.register_stream("a", 100);
        sync.register_stream("b", 100);

        // Empty streams
        assert!(sync.common_time_range().is_none());

        // Add data
        let _ = sync.push("a", 0.0, 1.0);
        let _ = sync.push("a", 2.0, 2.0);
        let _ = sync.push("b", 1.0, 10.0);
        let _ = sync.push("b", 3.0, 20.0);

        let (min, max) = sync.common_time_range().unwrap();
        assert!((min - 1.0).abs() < 1e-6); // max of mins
        assert!((max - 2.0).abs() < 1e-6); // min of maxs
    }

    #[test]
    fn synchronizer_synchronize() {
        let mut sync: StreamSynchronizer<f64> = StreamSynchronizer::new(SyncPolicy::linear());
        sync.register_stream("a", 100);
        sync.register_stream("b", 100);

        let _ = sync.push("a", 0.0, 0.0);
        let _ = sync.push("a", 1.0, 10.0);
        let _ = sync.push("b", 0.0, 0.0);
        let _ = sync.push("b", 1.0, 20.0);

        let result = sync.synchronize(0.5).unwrap();
        assert!(result.is_complete());
        assert_eq!(result.num_values(), 2);

        let a = result.get("a").unwrap();
        let b = result.get("b").unwrap();
        assert!((a - 5.0).abs() < 1e-6);
        assert!((b - 10.0).abs() < 1e-6);
    }

    #[test]
    fn synchronizer_generate_samples() {
        let mut sync: StreamSynchronizer<f64> = StreamSynchronizer::new(SyncPolicy::linear());
        sync.register_stream("a", 100);
        sync.register_stream("b", 100);

        let _ = sync.push("a", 0.0, 0.0);
        let _ = sync.push("a", 1.0, 10.0);
        let _ = sync.push("b", 0.0, 0.0);
        let _ = sync.push("b", 1.0, 20.0);

        let samples = sync.generate_samples(0.5).unwrap();
        assert_eq!(samples.len(), 3); // 0.0, 0.5, 1.0
    }

    #[test]
    fn synchronizer_clear() {
        let mut sync: StreamSynchronizer<f64> = StreamSynchronizer::new(SyncPolicy::linear());
        sync.register_stream("a", 100);
        let _ = sync.push("a", 0.0, 1.0);

        sync.clear();

        let buffer = sync.get_stream("a").unwrap();
        assert!(buffer.is_empty());
    }

    #[test]
    fn synchronizer_stream_ids() {
        let mut sync: StreamSynchronizer<f64> = StreamSynchronizer::new(SyncPolicy::linear());
        sync.register_stream("a", 100);
        sync.register_stream("b", 100);

        let ids = sync.stream_ids();
        assert_eq!(ids.len(), 2);
        assert!(ids.contains(&"a".to_string()));
        assert!(ids.contains(&"b".to_string()));
    }
}
