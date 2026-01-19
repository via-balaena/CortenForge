//! Time-ordered buffer for sensor readings.

use std::collections::VecDeque;

use serde::{Deserialize, Serialize};

/// A time-ordered buffer for storing sensor readings.
///
/// The buffer maintains readings in timestamp order and supports
/// efficient lookups and interpolation.
///
/// # Example
///
/// ```
/// use sensor_fusion::StreamBuffer;
///
/// let mut buffer: StreamBuffer<f64> = StreamBuffer::new(100);
/// buffer.push(0.0, 1.0);
/// buffer.push(0.1, 2.0);
/// buffer.push(0.2, 3.0);
///
/// assert_eq!(buffer.len(), 3);
/// assert!((buffer.latest().unwrap().1 - 3.0).abs() < 1e-6);
/// ```
#[derive(Debug, Clone)]
pub struct StreamBuffer<T> {
    /// Maximum capacity.
    capacity: usize,

    /// Time-ordered readings (timestamp, value).
    readings: VecDeque<(f64, T)>,
}

impl<T> StreamBuffer<T> {
    /// Creates a new buffer with the given capacity.
    ///
    /// # Arguments
    ///
    /// - `capacity`: Maximum number of readings to store
    #[must_use]
    pub fn new(capacity: usize) -> Self {
        Self {
            capacity: capacity.max(1),
            readings: VecDeque::with_capacity(capacity.min(1024)),
        }
    }

    /// Returns the capacity of the buffer.
    #[must_use]
    pub const fn capacity(&self) -> usize {
        self.capacity
    }

    /// Returns the number of readings in the buffer.
    #[must_use]
    pub fn len(&self) -> usize {
        self.readings.len()
    }

    /// Returns true if the buffer is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.readings.is_empty()
    }

    /// Returns true if the buffer is at capacity.
    #[must_use]
    pub fn is_full(&self) -> bool {
        self.readings.len() >= self.capacity
    }

    /// Clears all readings from the buffer.
    pub fn clear(&mut self) {
        self.readings.clear();
    }

    /// Pushes a reading to the buffer.
    ///
    /// If at capacity, the oldest reading is removed.
    /// Readings should be pushed in timestamp order.
    pub fn push(&mut self, timestamp: f64, value: T) {
        // Remove oldest if at capacity
        if self.is_full() {
            self.readings.pop_front();
        }
        self.readings.push_back((timestamp, value));
    }

    /// Returns the oldest reading.
    #[must_use]
    pub fn oldest(&self) -> Option<&(f64, T)> {
        self.readings.front()
    }

    /// Returns the newest reading.
    #[must_use]
    pub fn latest(&self) -> Option<&(f64, T)> {
        self.readings.back()
    }

    /// Returns the time span of readings in the buffer.
    ///
    /// Returns `None` if the buffer has fewer than 2 readings.
    #[must_use]
    pub fn time_span(&self) -> Option<f64> {
        if self.readings.len() < 2 {
            return None;
        }
        let oldest = self.readings.front()?;
        let newest = self.readings.back()?;
        Some(newest.0 - oldest.0)
    }

    /// Returns the timestamp range `(min, max)`.
    ///
    /// Returns `None` if the buffer is empty.
    #[must_use]
    pub fn timestamp_range(&self) -> Option<(f64, f64)> {
        let oldest = self.readings.front()?;
        let newest = self.readings.back()?;
        Some((oldest.0, newest.0))
    }

    /// Finds readings bracketing the given timestamp.
    ///
    /// Returns indices `(before, after)` where:
    /// - `before` is the index of the reading at or before `timestamp`
    /// - `after` is the index of the reading at or after `timestamp`
    ///
    /// Returns `None` if timestamp is outside the buffer range.
    #[must_use]
    pub fn find_bracket(&self, timestamp: f64) -> Option<(usize, usize)> {
        if self.readings.is_empty() {
            return None;
        }

        let (min, max) = self.timestamp_range()?;
        if timestamp < min || timestamp > max {
            return None;
        }

        // Binary search for the bracket
        let mut lo = 0;
        let mut hi = self.readings.len();

        while lo < hi {
            let mid = usize::midpoint(lo, hi);
            if self.readings[mid].0 < timestamp {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }

        // lo now points to first reading >= timestamp
        if lo == 0 {
            Some((0, 0))
        } else if lo >= self.readings.len() {
            Some((self.readings.len() - 1, self.readings.len() - 1))
        } else if (self.readings[lo].0 - timestamp).abs() < 1e-12 {
            Some((lo, lo))
        } else {
            Some((lo - 1, lo))
        }
    }

    /// Gets a reading by index.
    #[must_use]
    pub fn get(&self, index: usize) -> Option<&(f64, T)> {
        self.readings.get(index)
    }

    /// Returns an iterator over all readings.
    pub fn iter(&self) -> impl Iterator<Item = &(f64, T)> {
        self.readings.iter()
    }

    /// Removes readings older than the given timestamp.
    pub fn remove_before(&mut self, timestamp: f64) {
        while let Some(front) = self.readings.front() {
            if front.0 < timestamp {
                self.readings.pop_front();
            } else {
                break;
            }
        }
    }
}

impl<T: Clone> StreamBuffer<T> {
    /// Gets readings in a time range `[start, end]`.
    #[must_use]
    pub fn range(&self, start: f64, end: f64) -> Vec<(f64, T)> {
        self.readings
            .iter()
            .filter(|(t, _)| *t >= start && *t <= end)
            .cloned()
            .collect()
    }
}

/// Statistics about a stream buffer.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BufferStats {
    /// Number of readings.
    pub count: usize,

    /// Buffer capacity.
    pub capacity: usize,

    /// Minimum timestamp.
    pub min_timestamp: Option<f64>,

    /// Maximum timestamp.
    pub max_timestamp: Option<f64>,

    /// Time span in seconds.
    pub time_span: Option<f64>,
}

impl<T> StreamBuffer<T> {
    /// Computes statistics about the buffer.
    #[must_use]
    pub fn stats(&self) -> BufferStats {
        BufferStats {
            count: self.len(),
            capacity: self.capacity,
            min_timestamp: self.oldest().map(|(t, _)| *t),
            max_timestamp: self.latest().map(|(t, _)| *t),
            time_span: self.time_span(),
        }
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
    fn buffer_new() {
        let buffer: StreamBuffer<f32> = StreamBuffer::new(100);
        assert_eq!(buffer.capacity(), 100);
        assert!(buffer.is_empty());
        assert!(!buffer.is_full());
    }

    #[test]
    fn buffer_min_capacity() {
        let buffer: StreamBuffer<f32> = StreamBuffer::new(0);
        assert_eq!(buffer.capacity(), 1);
    }

    #[test]
    fn buffer_push() {
        let mut buffer = StreamBuffer::new(10);
        buffer.push(0.0, 1.0);
        buffer.push(0.1, 2.0);
        buffer.push(0.2, 3.0);

        assert_eq!(buffer.len(), 3);
        assert!(!buffer.is_empty());
    }

    #[test]
    fn buffer_push_overflow() {
        let mut buffer: StreamBuffer<f64> = StreamBuffer::new(3);
        buffer.push(0.0, 1.0);
        buffer.push(0.1, 2.0);
        buffer.push(0.2, 3.0);
        assert!(buffer.is_full());

        buffer.push(0.3, 4.0);
        assert_eq!(buffer.len(), 3);
        assert!((buffer.oldest().unwrap().1 - 2.0).abs() < 1e-6); // 1.0 was removed
    }

    #[test]
    fn buffer_oldest_latest() {
        let mut buffer: StreamBuffer<f64> = StreamBuffer::new(10);
        buffer.push(0.0, 1.0);
        buffer.push(0.1, 2.0);
        buffer.push(0.2, 3.0);

        let oldest = buffer.oldest().unwrap();
        assert!((oldest.0 - 0.0).abs() < 1e-6);
        assert!((oldest.1 - 1.0).abs() < 1e-6);

        let latest = buffer.latest().unwrap();
        assert!((latest.0 - 0.2).abs() < 1e-6);
        assert!((latest.1 - 3.0).abs() < 1e-6);
    }

    #[test]
    fn buffer_time_span() {
        let mut buffer = StreamBuffer::new(10);
        assert!(buffer.time_span().is_none());

        buffer.push(0.0, 1.0);
        assert!(buffer.time_span().is_none()); // Need at least 2

        buffer.push(0.5, 2.0);
        assert!((buffer.time_span().unwrap() - 0.5).abs() < 1e-6);
    }

    #[test]
    fn buffer_timestamp_range() {
        let mut buffer = StreamBuffer::new(10);
        assert!(buffer.timestamp_range().is_none());

        buffer.push(1.0, 10.0);
        buffer.push(2.0, 20.0);
        buffer.push(3.0, 30.0);

        let (min, max) = buffer.timestamp_range().unwrap();
        assert!((min - 1.0).abs() < 1e-6);
        assert!((max - 3.0).abs() < 1e-6);
    }

    #[test]
    fn buffer_find_bracket() {
        let mut buffer = StreamBuffer::new(10);
        buffer.push(0.0, 1.0);
        buffer.push(1.0, 2.0);
        buffer.push(2.0, 3.0);
        buffer.push(3.0, 4.0);

        // Exact match
        assert_eq!(buffer.find_bracket(1.0), Some((1, 1)));

        // Between readings
        assert_eq!(buffer.find_bracket(1.5), Some((1, 2)));

        // At boundaries
        assert_eq!(buffer.find_bracket(0.0), Some((0, 0)));
        assert_eq!(buffer.find_bracket(3.0), Some((3, 3)));

        // Outside range
        assert!(buffer.find_bracket(-1.0).is_none());
        assert!(buffer.find_bracket(5.0).is_none());
    }

    #[test]
    fn buffer_clear() {
        let mut buffer = StreamBuffer::new(10);
        buffer.push(0.0, 1.0);
        buffer.push(0.1, 2.0);
        buffer.clear();

        assert!(buffer.is_empty());
        assert_eq!(buffer.len(), 0);
    }

    #[test]
    fn buffer_get() {
        let mut buffer = StreamBuffer::new(10);
        buffer.push(0.0, 1.0);
        buffer.push(0.1, 2.0);

        assert!(buffer.get(0).is_some());
        assert!(buffer.get(1).is_some());
        assert!(buffer.get(2).is_none());
    }

    #[test]
    fn buffer_iter() {
        let mut buffer = StreamBuffer::new(10);
        buffer.push(0.0, 1.0);
        buffer.push(0.1, 2.0);
        buffer.push(0.2, 3.0);

        let values: Vec<f64> = buffer.iter().map(|(_, v)| *v).collect();
        assert_eq!(values, vec![1.0, 2.0, 3.0]);
    }

    #[test]
    fn buffer_remove_before() {
        let mut buffer = StreamBuffer::new(10);
        buffer.push(0.0, 1.0);
        buffer.push(1.0, 2.0);
        buffer.push(2.0, 3.0);
        buffer.push(3.0, 4.0);

        buffer.remove_before(1.5);

        assert_eq!(buffer.len(), 2);
        assert!((buffer.oldest().unwrap().0 - 2.0).abs() < 1e-6);
    }

    #[test]
    fn buffer_range() {
        let mut buffer: StreamBuffer<f64> = StreamBuffer::new(10);
        buffer.push(0.0, 1.0);
        buffer.push(1.0, 2.0);
        buffer.push(2.0, 3.0);
        buffer.push(3.0, 4.0);

        let range = buffer.range(0.5, 2.5);
        assert_eq!(range.len(), 2);
        assert!((range[0].1 - 2.0).abs() < 1e-6);
        assert!((range[1].1 - 3.0).abs() < 1e-6);
    }

    #[test]
    fn buffer_stats() {
        let mut buffer = StreamBuffer::new(100);
        buffer.push(0.0, 1.0);
        buffer.push(1.0, 2.0);
        buffer.push(2.0, 3.0);

        let stats = buffer.stats();
        assert_eq!(stats.count, 3);
        assert_eq!(stats.capacity, 100);
        assert!((stats.min_timestamp.unwrap() - 0.0).abs() < 1e-6);
        assert!((stats.max_timestamp.unwrap() - 2.0).abs() < 1e-6);
        assert!((stats.time_span.unwrap() - 2.0).abs() < 1e-6);
    }

    #[test]
    fn buffer_stats_empty() {
        let buffer: StreamBuffer<f32> = StreamBuffer::new(100);
        let stats = buffer.stats();

        assert_eq!(stats.count, 0);
        assert!(stats.min_timestamp.is_none());
        assert!(stats.max_timestamp.is_none());
        assert!(stats.time_span.is_none());
    }
}
