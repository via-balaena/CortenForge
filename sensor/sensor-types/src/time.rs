//! Time types for sensor data.
//!
//! Provides nanosecond-precision timing for sensor readings.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Nanosecond-precision timestamp.
///
/// Used for all sensor readings to enable precise temporal alignment
/// in `sensor-fusion`.
///
/// # Example
///
/// ```
/// use sensor_types::Timestamp;
///
/// let ts = Timestamp::from_secs_f64(1.5);
/// assert!((ts.as_secs_f64() - 1.5).abs() < 1e-9);
///
/// let ts_nanos = Timestamp::from_nanos(1_500_000_000);
/// assert_eq!(ts, ts_nanos);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Timestamp {
    /// Nanoseconds since epoch (or simulation start).
    nanos: u64,
}

impl Timestamp {
    /// Creates a timestamp from nanoseconds.
    #[must_use]
    pub const fn from_nanos(nanos: u64) -> Self {
        Self { nanos }
    }

    /// Creates a timestamp from seconds (floating point).
    ///
    /// # Example
    ///
    /// ```
    /// use sensor_types::Timestamp;
    ///
    /// let ts = Timestamp::from_secs_f64(1.5);
    /// assert_eq!(ts.as_nanos(), 1_500_000_000);
    /// ```
    #[must_use]
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    pub fn from_secs_f64(secs: f64) -> Self {
        let nanos = (secs * 1e9).max(0.0) as u64;
        Self { nanos }
    }

    /// Creates a timestamp from seconds and nanoseconds.
    #[must_use]
    pub const fn from_secs_nanos(secs: u64, nanos: u32) -> Self {
        Self {
            nanos: secs * 1_000_000_000 + nanos as u64,
        }
    }

    /// Returns the timestamp as nanoseconds.
    #[must_use]
    pub const fn as_nanos(self) -> u64 {
        self.nanos
    }

    /// Returns the timestamp as seconds (floating point).
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn as_secs_f64(self) -> f64 {
        self.nanos as f64 / 1e9
    }

    /// Returns the whole seconds component.
    #[must_use]
    pub const fn secs(self) -> u64 {
        self.nanos / 1_000_000_000
    }

    /// Returns the subsecond nanoseconds component.
    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    pub const fn subsec_nanos(self) -> u32 {
        (self.nanos % 1_000_000_000) as u32
    }

    /// Returns the zero timestamp.
    #[must_use]
    pub const fn zero() -> Self {
        Self { nanos: 0 }
    }

    /// Checks if this is the zero timestamp.
    #[must_use]
    pub const fn is_zero(self) -> bool {
        self.nanos == 0
    }

    /// Adds a duration to this timestamp.
    ///
    /// Returns `None` on overflow.
    #[must_use]
    pub const fn checked_add(self, duration: Duration) -> Option<Self> {
        match self.nanos.checked_add(duration.as_nanos()) {
            Some(nanos) => Some(Self { nanos }),
            None => None,
        }
    }

    /// Subtracts a duration from this timestamp.
    ///
    /// Returns `None` on underflow.
    #[must_use]
    pub const fn checked_sub(self, duration: Duration) -> Option<Self> {
        match self.nanos.checked_sub(duration.as_nanos()) {
            Some(nanos) => Some(Self { nanos }),
            None => None,
        }
    }

    /// Returns the duration between two timestamps.
    ///
    /// Always returns a non-negative duration (absolute difference).
    #[must_use]
    pub const fn abs_diff(self, other: Self) -> Duration {
        Duration::from_nanos(self.nanos.abs_diff(other.nanos))
    }
}

/// A duration of time with nanosecond precision.
///
/// Unlike [`Timestamp`], this represents a time interval rather than
/// a point in time.
///
/// # Example
///
/// ```
/// use sensor_types::Duration;
///
/// let d = Duration::from_millis(100);
/// assert_eq!(d.as_nanos(), 100_000_000);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Duration {
    /// Duration in nanoseconds.
    nanos: u64,
}

impl Duration {
    /// Creates a duration from nanoseconds.
    #[must_use]
    pub const fn from_nanos(nanos: u64) -> Self {
        Self { nanos }
    }

    /// Creates a duration from microseconds.
    #[must_use]
    pub const fn from_micros(micros: u64) -> Self {
        Self {
            nanos: micros * 1_000,
        }
    }

    /// Creates a duration from milliseconds.
    #[must_use]
    pub const fn from_millis(millis: u64) -> Self {
        Self {
            nanos: millis * 1_000_000,
        }
    }

    /// Creates a duration from seconds.
    #[must_use]
    pub const fn from_secs(secs: u64) -> Self {
        Self {
            nanos: secs * 1_000_000_000,
        }
    }

    /// Creates a duration from seconds (floating point).
    #[must_use]
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    pub fn from_secs_f64(secs: f64) -> Self {
        let nanos = (secs * 1e9).max(0.0) as u64;
        Self { nanos }
    }

    /// Returns the duration as nanoseconds.
    #[must_use]
    pub const fn as_nanos(self) -> u64 {
        self.nanos
    }

    /// Returns the duration as microseconds.
    #[must_use]
    pub const fn as_micros(self) -> u64 {
        self.nanos / 1_000
    }

    /// Returns the duration as milliseconds.
    #[must_use]
    pub const fn as_millis(self) -> u64 {
        self.nanos / 1_000_000
    }

    /// Returns the duration as seconds (floating point).
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn as_secs_f64(self) -> f64 {
        self.nanos as f64 / 1e9
    }

    /// Returns the zero duration.
    #[must_use]
    pub const fn zero() -> Self {
        Self { nanos: 0 }
    }

    /// Checks if this is a zero duration.
    #[must_use]
    pub const fn is_zero(self) -> bool {
        self.nanos == 0
    }

    /// Adds two durations.
    ///
    /// Returns `None` on overflow.
    #[must_use]
    pub const fn checked_add(self, other: Self) -> Option<Self> {
        match self.nanos.checked_add(other.nanos) {
            Some(nanos) => Some(Self { nanos }),
            None => None,
        }
    }

    /// Subtracts two durations.
    ///
    /// Returns `None` on underflow.
    #[must_use]
    pub const fn checked_sub(self, other: Self) -> Option<Self> {
        match self.nanos.checked_sub(other.nanos) {
            Some(nanos) => Some(Self { nanos }),
            None => None,
        }
    }

    /// Multiplies the duration by a scalar.
    ///
    /// Returns `None` on overflow.
    #[must_use]
    pub const fn checked_mul(self, factor: u64) -> Option<Self> {
        match self.nanos.checked_mul(factor) {
            Some(nanos) => Some(Self { nanos }),
            None => None,
        }
    }
}

/// A time range (inclusive start, exclusive end).
///
/// Useful for querying sensor data within a specific time window.
///
/// # Example
///
/// ```
/// use sensor_types::{Timestamp, TimeRange};
///
/// let range = TimeRange::new(
///     Timestamp::from_secs_f64(1.0),
///     Timestamp::from_secs_f64(2.0),
/// );
///
/// assert!(range.contains(Timestamp::from_secs_f64(1.5)));
/// assert!(!range.contains(Timestamp::from_secs_f64(2.0)));
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct TimeRange {
    /// Start of the range (inclusive).
    pub start: Timestamp,
    /// End of the range (exclusive).
    pub end: Timestamp,
}

impl TimeRange {
    /// Creates a new time range.
    ///
    /// If `start > end`, they are swapped.
    #[must_use]
    pub fn new(start: Timestamp, end: Timestamp) -> Self {
        if start <= end {
            Self { start, end }
        } else {
            Self {
                start: end,
                end: start,
            }
        }
    }

    /// Returns the duration of this time range.
    #[must_use]
    pub const fn duration(self) -> Duration {
        self.start.abs_diff(self.end)
    }

    /// Checks if a timestamp is within this range.
    ///
    /// The range is inclusive at the start and exclusive at the end.
    #[must_use]
    pub fn contains(self, timestamp: Timestamp) -> bool {
        timestamp >= self.start && timestamp < self.end
    }

    /// Checks if this range overlaps with another.
    #[must_use]
    pub fn overlaps(self, other: Self) -> bool {
        self.start < other.end && other.start < self.end
    }

    /// Returns the intersection of two ranges, if any.
    #[must_use]
    pub fn intersection(self, other: Self) -> Option<Self> {
        let start = if self.start > other.start {
            self.start
        } else {
            other.start
        };
        let end = if self.end < other.end {
            self.end
        } else {
            other.end
        };

        if start < end {
            Some(Self { start, end })
        } else {
            None
        }
    }

    /// Checks if this range is empty (start >= end after normalization).
    #[must_use]
    pub fn is_empty(self) -> bool {
        self.start >= self.end
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timestamp_from_secs_f64() {
        let ts = Timestamp::from_secs_f64(1.5);
        assert_eq!(ts.as_nanos(), 1_500_000_000);
        assert!((ts.as_secs_f64() - 1.5).abs() < 1e-9);
    }

    #[test]
    fn timestamp_components() {
        let ts = Timestamp::from_secs_nanos(3, 500_000_000);
        assert_eq!(ts.secs(), 3);
        assert_eq!(ts.subsec_nanos(), 500_000_000);
    }

    #[test]
    fn timestamp_checked_ops() {
        let ts = Timestamp::from_nanos(1000);
        let d = Duration::from_nanos(500);

        assert_eq!(ts.checked_add(d), Some(Timestamp::from_nanos(1500)));
        assert_eq!(ts.checked_sub(d), Some(Timestamp::from_nanos(500)));
        assert_eq!(ts.checked_sub(Duration::from_nanos(2000)), None);
    }

    #[test]
    fn timestamp_abs_diff() {
        let a = Timestamp::from_nanos(1000);
        let b = Timestamp::from_nanos(300);

        assert_eq!(a.abs_diff(b), Duration::from_nanos(700));
        assert_eq!(b.abs_diff(a), Duration::from_nanos(700));
    }

    #[test]
    fn duration_conversions() {
        let d = Duration::from_millis(1500);
        assert_eq!(d.as_nanos(), 1_500_000_000);
        assert_eq!(d.as_micros(), 1_500_000);
        assert_eq!(d.as_millis(), 1500);
        assert!((d.as_secs_f64() - 1.5).abs() < 1e-9);
    }

    #[test]
    fn duration_checked_ops() {
        let a = Duration::from_nanos(1000);
        let b = Duration::from_nanos(500);

        assert_eq!(a.checked_add(b), Some(Duration::from_nanos(1500)));
        assert_eq!(a.checked_sub(b), Some(Duration::from_nanos(500)));
        assert_eq!(b.checked_sub(a), None);
        assert_eq!(a.checked_mul(3), Some(Duration::from_nanos(3000)));
    }

    #[test]
    fn time_range_contains() {
        let range = TimeRange::new(Timestamp::from_nanos(100), Timestamp::from_nanos(200));

        assert!(range.contains(Timestamp::from_nanos(100)));
        assert!(range.contains(Timestamp::from_nanos(150)));
        assert!(!range.contains(Timestamp::from_nanos(200)));
        assert!(!range.contains(Timestamp::from_nanos(50)));
    }

    #[test]
    fn time_range_overlaps() {
        let a = TimeRange::new(Timestamp::from_nanos(100), Timestamp::from_nanos(200));
        let b = TimeRange::new(Timestamp::from_nanos(150), Timestamp::from_nanos(250));
        let c = TimeRange::new(Timestamp::from_nanos(200), Timestamp::from_nanos(300));

        assert!(a.overlaps(b));
        assert!(!a.overlaps(c)); // touching but not overlapping
    }

    #[test]
    fn time_range_intersection() {
        let a = TimeRange::new(Timestamp::from_nanos(100), Timestamp::from_nanos(200));
        let b = TimeRange::new(Timestamp::from_nanos(150), Timestamp::from_nanos(250));

        let inter = a.intersection(b);
        assert!(inter.is_some());
        let inter = inter.map_or(TimeRange::new(Timestamp::zero(), Timestamp::zero()), |i| i);
        assert_eq!(inter.start, Timestamp::from_nanos(150));
        assert_eq!(inter.end, Timestamp::from_nanos(200));
    }

    #[test]
    fn time_range_normalizes() {
        // If start > end, they should be swapped
        let range = TimeRange::new(Timestamp::from_nanos(200), Timestamp::from_nanos(100));
        assert_eq!(range.start, Timestamp::from_nanos(100));
        assert_eq!(range.end, Timestamp::from_nanos(200));
    }

    #[cfg(feature = "serde")]
    #[test]
    fn timestamp_serialization() {
        let ts = Timestamp::from_nanos(1_500_000_000);
        let json = serde_json::to_string(&ts).ok();
        assert!(json.is_some());

        let parsed: Result<Timestamp, _> = serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
        assert_eq!(parsed.unwrap_or_default(), ts);
    }
}
