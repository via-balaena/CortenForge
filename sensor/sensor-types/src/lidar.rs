//! `LiDAR` sensor types.
//!
//! Provides types for 3D point clouds from `LiDAR` sensors.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::{CoordinateFrame, Timestamp};

/// A single `LiDAR` point with optional intensity.
///
/// # Example
///
/// ```
/// use sensor_types::LidarPoint;
///
/// let point = LidarPoint {
///     position: [1.0, 2.0, 3.0],
///     intensity: Some(0.8),
///     ring: Some(15),
/// };
///
/// assert!((point.distance() - (1.0f32*1.0 + 2.0*2.0 + 3.0*3.0).sqrt()).abs() < 1e-6);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct LidarPoint {
    /// 3D position in meters: `[x, y, z]`.
    pub position: [f32; 3],

    /// Intensity/reflectivity (0.0 to 1.0), if available.
    pub intensity: Option<f32>,

    /// Ring/channel index for multi-beam `LiDARs` (e.g., Velodyne).
    pub ring: Option<u16>,
}

impl LidarPoint {
    /// Creates a new `LiDAR` point.
    #[must_use]
    pub const fn new(position: [f32; 3]) -> Self {
        Self {
            position,
            intensity: None,
            ring: None,
        }
    }

    /// Creates a `LiDAR` point with intensity.
    #[must_use]
    pub const fn with_intensity(position: [f32; 3], intensity: f32) -> Self {
        Self {
            position,
            intensity: Some(intensity),
            ring: None,
        }
    }

    /// Returns the distance from the sensor origin.
    #[must_use]
    pub fn distance(&self) -> f32 {
        let [x, y, z] = self.position;
        x.hypot(y).hypot(z)
    }

    /// Returns the horizontal distance (ignoring Z).
    #[must_use]
    pub fn horizontal_distance(&self) -> f32 {
        let [x, y, _] = self.position;
        x.hypot(y)
    }

    /// Returns the azimuth angle in radians (-π to π).
    #[must_use]
    pub fn azimuth(&self) -> f32 {
        self.position[1].atan2(self.position[0])
    }

    /// Returns the elevation angle in radians.
    #[must_use]
    pub fn elevation(&self) -> f32 {
        let horiz = self.horizontal_distance();
        self.position[2].atan2(horiz)
    }

    /// Checks if this point is valid (finite position).
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.position.iter().all(|v| v.is_finite())
    }
}

impl Default for LidarPoint {
    fn default() -> Self {
        Self::new([0.0, 0.0, 0.0])
    }
}

/// A `LiDAR` scan containing a point cloud.
///
/// Represents a single sweep or frame from a `LiDAR` sensor.
///
/// # Example
///
/// ```
/// use sensor_types::{LidarScan, LidarPoint, CoordinateFrame, Timestamp};
///
/// let scan = LidarScan {
///     timestamp: Timestamp::from_secs_f64(1.0),
///     points: vec![
///         LidarPoint::new([1.0, 0.0, 0.0]),
///         LidarPoint::new([0.0, 1.0, 0.0]),
///         LidarPoint::new([0.0, 0.0, 1.0]),
///     ],
///     frame: CoordinateFrame::sensor("velodyne"),
/// };
///
/// assert_eq!(scan.point_count(), 3);
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct LidarScan {
    /// Timestamp of the scan.
    pub timestamp: Timestamp,

    /// Point cloud data.
    pub points: Vec<LidarPoint>,

    /// Coordinate frame of the points.
    pub frame: CoordinateFrame,
}

impl LidarScan {
    /// Creates a new empty `LiDAR` scan.
    #[must_use]
    pub const fn new(timestamp: Timestamp, frame: CoordinateFrame) -> Self {
        Self {
            timestamp,
            points: Vec::new(),
            frame,
        }
    }

    /// Creates a scan from a vector of 3D positions.
    ///
    /// Creates points without intensity or ring data.
    #[must_use]
    pub fn from_positions(
        timestamp: Timestamp,
        positions: Vec<[f32; 3]>,
        frame: CoordinateFrame,
    ) -> Self {
        let points = positions.into_iter().map(LidarPoint::new).collect();
        Self {
            timestamp,
            points,
            frame,
        }
    }

    /// Returns the number of points in the scan.
    #[must_use]
    pub fn point_count(&self) -> usize {
        self.points.len()
    }

    /// Checks if the scan is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Returns the number of valid points.
    #[must_use]
    pub fn valid_point_count(&self) -> usize {
        self.points.iter().filter(|p| p.is_valid()).count()
    }

    /// Returns all positions as a flat vector.
    #[must_use]
    pub fn positions(&self) -> Vec<[f32; 3]> {
        self.points.iter().map(|p| p.position).collect()
    }

    /// Returns all intensities (if available) as a vector.
    #[must_use]
    pub fn intensities(&self) -> Vec<Option<f32>> {
        self.points.iter().map(|p| p.intensity).collect()
    }

    /// Filters to points with intensity above a threshold.
    #[must_use]
    pub fn filter_by_intensity(&self, min_intensity: f32) -> Self {
        let points = self
            .points
            .iter()
            .filter(|p| p.intensity.is_some_and(|i| i >= min_intensity))
            .copied()
            .collect();

        Self {
            timestamp: self.timestamp,
            points,
            frame: self.frame.clone(),
        }
    }

    /// Filters to points within a distance range.
    #[must_use]
    pub fn filter_by_distance(&self, min_dist: f32, max_dist: f32) -> Self {
        let points = self
            .points
            .iter()
            .filter(|p| {
                let d = p.distance();
                d >= min_dist && d <= max_dist
            })
            .copied()
            .collect();

        Self {
            timestamp: self.timestamp,
            points,
            frame: self.frame.clone(),
        }
    }

    /// Returns the bounding box of all valid points.
    ///
    /// Returns `None` if there are no valid points.
    #[must_use]
    pub fn bounding_box(&self) -> Option<([f32; 3], [f32; 3])> {
        let mut min = [f32::INFINITY; 3];
        let mut max = [f32::NEG_INFINITY; 3];
        let mut any_valid = false;

        for point in &self.points {
            if !point.is_valid() {
                continue;
            }
            any_valid = true;
            for i in 0..3 {
                min[i] = min[i].min(point.position[i]);
                max[i] = max[i].max(point.position[i]);
            }
        }

        if any_valid { Some((min, max)) } else { None }
    }

    /// Computes the centroid of all valid points.
    ///
    /// Returns `None` if there are no valid points.
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn centroid(&self) -> Option<[f32; 3]> {
        let valid: Vec<_> = self.points.iter().filter(|p| p.is_valid()).collect();

        if valid.is_empty() {
            return None;
        }

        let mut sum = [0.0f32; 3];
        for p in &valid {
            sum[0] += p.position[0];
            sum[1] += p.position[1];
            sum[2] += p.position[2];
        }

        let n = valid.len() as f32;
        Some([sum[0] / n, sum[1] / n, sum[2] / n])
    }
}

impl Default for LidarScan {
    fn default() -> Self {
        Self::new(Timestamp::zero(), CoordinateFrame::Body)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lidar_point_distance() {
        let point = LidarPoint::new([3.0, 4.0, 0.0]);
        assert!((point.distance() - 5.0).abs() < 1e-6);
    }

    #[test]
    fn lidar_point_horizontal_distance() {
        let point = LidarPoint::new([3.0, 4.0, 10.0]);
        assert!((point.horizontal_distance() - 5.0).abs() < 1e-6);
    }

    #[test]
    fn lidar_point_angles() {
        // Point along X axis
        let point = LidarPoint::new([1.0, 0.0, 0.0]);
        assert!(point.azimuth().abs() < 1e-6);
        assert!(point.elevation().abs() < 1e-6);

        // Point along Z axis
        let point = LidarPoint::new([0.0, 0.0, 1.0]);
        assert!((point.elevation() - std::f32::consts::FRAC_PI_2).abs() < 1e-6);
    }

    #[test]
    fn lidar_scan_from_positions() {
        let positions = vec![[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let scan = LidarScan::from_positions(Timestamp::zero(), positions, CoordinateFrame::Body);

        assert_eq!(scan.point_count(), 3);
        assert!(!scan.is_empty());
    }

    #[test]
    fn lidar_scan_filter_distance() {
        let positions = vec![
            [1.0, 0.0, 0.0],  // distance 1
            [5.0, 0.0, 0.0],  // distance 5
            [10.0, 0.0, 0.0], // distance 10
        ];
        let scan = LidarScan::from_positions(Timestamp::zero(), positions, CoordinateFrame::Body);

        let filtered = scan.filter_by_distance(2.0, 8.0);
        assert_eq!(filtered.point_count(), 1);
    }

    #[test]
    #[allow(clippy::float_cmp)] // Exact constant values, min/max selection
    fn lidar_scan_bounding_box() {
        let positions = vec![[-1.0, -2.0, -3.0], [1.0, 2.0, 3.0]];
        let scan = LidarScan::from_positions(Timestamp::zero(), positions, CoordinateFrame::Body);

        let bbox = scan.bounding_box();
        assert!(bbox.is_some());
        let (min, max) = bbox.unwrap_or(([0.0; 3], [0.0; 3]));
        assert_eq!(min, [-1.0, -2.0, -3.0]);
        assert_eq!(max, [1.0, 2.0, 3.0]);
    }

    #[test]
    fn lidar_scan_centroid() {
        let positions = vec![[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [1.0, 3.0, 0.0]];
        let scan = LidarScan::from_positions(Timestamp::zero(), positions, CoordinateFrame::Body);

        let centroid = scan.centroid();
        assert!(centroid.is_some());
        let c = centroid.unwrap_or([0.0; 3]);
        assert!((c[0] - 1.0).abs() < 1e-6);
        assert!((c[1] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn lidar_scan_empty() {
        let scan = LidarScan::default();
        assert!(scan.is_empty());
        assert!(scan.bounding_box().is_none());
        assert!(scan.centroid().is_none());
    }

    #[cfg(feature = "serde")]
    #[test]
    fn lidar_serialization() {
        let point = LidarPoint::with_intensity([1.0, 2.0, 3.0], 0.5);
        let json = serde_json::to_string(&point).ok();
        assert!(json.is_some());
    }
}
