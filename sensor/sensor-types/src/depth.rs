//! Depth sensor types.
//!
//! Provides types for depth maps from stereo cameras, structured light sensors,
//! or time-of-flight cameras.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::{CameraIntrinsics, Timestamp};

/// A depth map from a depth sensor.
///
/// Contains per-pixel depth values, typically from stereo cameras,
/// structured light sensors (e.g., Intel `RealSense`), or time-of-flight
/// cameras (e.g., Azure Kinect).
///
/// # Depth Values
///
/// - Depth is stored in meters as `f32` values
/// - Invalid pixels are represented as `NaN` or values outside `[min_depth, max_depth]`
/// - The depth buffer is stored in row-major order (width × height)
///
/// # Example
///
/// ```
/// use sensor_types::{DepthMap, CameraIntrinsics, Timestamp};
///
/// let depth_map = DepthMap {
///     timestamp: Timestamp::from_secs_f64(1.0),
///     depths: vec![1.5f32; 640 * 480],
///     width: 640,
///     height: 480,
///     intrinsics: CameraIntrinsics::ideal(500.0, 640, 480),
///     min_depth: 0.1,
///     max_depth: 10.0,
/// };
///
/// assert_eq!(depth_map.pixel_count(), 640 * 480);
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct DepthMap {
    /// Timestamp when the depth map was captured.
    pub timestamp: Timestamp,

    /// Per-pixel depth values in meters.
    ///
    /// Stored in row-major order: `depths[y * width + x]`
    pub depths: Vec<f32>,

    /// Image width in pixels.
    pub width: u32,

    /// Image height in pixels.
    pub height: u32,

    /// Camera intrinsics for the depth sensor.
    pub intrinsics: CameraIntrinsics,

    /// Minimum valid depth in meters.
    pub min_depth: f32,

    /// Maximum valid depth in meters.
    pub max_depth: f32,
}

impl DepthMap {
    /// Returns the total number of pixels.
    #[must_use]
    pub const fn pixel_count(&self) -> u32 {
        self.width * self.height
    }

    /// Returns the expected buffer size (width × height).
    #[must_use]
    pub const fn expected_buffer_size(&self) -> usize {
        (self.width * self.height) as usize
    }

    /// Checks if the depth buffer has the expected size.
    #[must_use]
    pub fn has_valid_buffer_size(&self) -> bool {
        self.depths.len() == self.expected_buffer_size()
    }

    /// Gets the depth at a pixel coordinate.
    ///
    /// Returns `None` if coordinates are out of bounds.
    #[must_use]
    pub fn get(&self, x: u32, y: u32) -> Option<f32> {
        if x >= self.width || y >= self.height {
            return None;
        }
        let idx = (y * self.width + x) as usize;
        self.depths.get(idx).copied()
    }

    /// Gets the depth at a pixel coordinate, or `NaN` if out of bounds.
    #[must_use]
    pub fn get_or_nan(&self, x: u32, y: u32) -> f32 {
        self.get(x, y).unwrap_or(f32::NAN)
    }

    /// Checks if a depth value is valid (within range and not `NaN`).
    #[must_use]
    pub fn is_valid_depth(&self, depth: f32) -> bool {
        !depth.is_nan() && depth >= self.min_depth && depth <= self.max_depth
    }

    /// Gets the depth at a pixel, returning `None` if invalid.
    #[must_use]
    pub fn get_valid(&self, x: u32, y: u32) -> Option<f32> {
        self.get(x, y).filter(|&d| self.is_valid_depth(d))
    }

    /// Counts the number of valid depth pixels.
    #[must_use]
    pub fn valid_pixel_count(&self) -> usize {
        self.depths
            .iter()
            .filter(|&&d| self.is_valid_depth(d))
            .count()
    }

    /// Returns the fraction of pixels with valid depth (0.0 to 1.0).
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn valid_fraction(&self) -> f32 {
        if self.depths.is_empty() {
            return 0.0;
        }
        self.valid_pixel_count() as f32 / self.depths.len() as f32
    }

    /// Unprojects a pixel to a 3D point using the depth value.
    ///
    /// Returns `None` if the depth is invalid or coordinates are out of bounds.
    #[must_use]
    pub fn unproject_pixel(&self, x: u32, y: u32) -> Option<[f64; 3]> {
        let depth = self.get_valid(x, y)?;
        let z = f64::from(depth);

        let cx = self.intrinsics.cx;
        let cy = self.intrinsics.cy;
        let fx = self.intrinsics.fx;
        let fy = self.intrinsics.fy;

        let x_norm = (f64::from(x) - cx) / fx;
        let y_norm = (f64::from(y) - cy) / fy;

        Some([x_norm * z, y_norm * z, z])
    }

    /// Converts the depth map to a point cloud.
    ///
    /// Returns a vector of 3D points for all valid depth pixels.
    #[must_use]
    pub fn to_point_cloud(&self) -> Vec<[f64; 3]> {
        let mut points = Vec::new();
        for y in 0..self.height {
            for x in 0..self.width {
                if let Some(point) = self.unproject_pixel(x, y) {
                    points.push(point);
                }
            }
        }
        points
    }

    /// Returns depth statistics (min, max, mean) for valid pixels.
    ///
    /// Returns `None` if there are no valid pixels.
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn depth_stats(&self) -> Option<DepthStats> {
        let valid: Vec<f32> = self
            .depths
            .iter()
            .copied()
            .filter(|&d| self.is_valid_depth(d))
            .collect();

        if valid.is_empty() {
            return None;
        }

        let min = valid.iter().copied().fold(f32::INFINITY, f32::min);
        let max = valid.iter().copied().fold(f32::NEG_INFINITY, f32::max);
        let sum: f32 = valid.iter().sum();
        let mean = sum / valid.len() as f32;

        Some(DepthStats { min, max, mean })
    }
}

/// Statistics for valid depth values.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct DepthStats {
    /// Minimum valid depth in meters.
    pub min: f32,
    /// Maximum valid depth in meters.
    pub max: f32,
    /// Mean valid depth in meters.
    pub mean: f32,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_depth_map() -> DepthMap {
        DepthMap {
            timestamp: Timestamp::zero(),
            depths: vec![1.0f32; 10 * 10],
            width: 10,
            height: 10,
            intrinsics: CameraIntrinsics::ideal(10.0, 10, 10),
            min_depth: 0.1,
            max_depth: 10.0,
        }
    }

    #[test]
    fn depth_map_size() {
        let dm = sample_depth_map();
        assert_eq!(dm.pixel_count(), 100);
        assert!(dm.has_valid_buffer_size());
    }

    #[test]
    fn depth_map_get() {
        let dm = sample_depth_map();
        assert!((dm.get(5, 5).unwrap_or(0.0) - 1.0).abs() < 1e-6);
        assert!(dm.get(100, 100).is_none());
    }

    #[test]
    fn depth_map_validity() {
        let mut dm = sample_depth_map();
        dm.depths[0] = f32::NAN;
        dm.depths[1] = 0.05; // Below min
        dm.depths[2] = 15.0; // Above max

        assert!(!dm.is_valid_depth(dm.depths[0]));
        assert!(!dm.is_valid_depth(dm.depths[1]));
        assert!(!dm.is_valid_depth(dm.depths[2]));
        assert!(dm.is_valid_depth(1.0));
    }

    #[test]
    fn depth_map_valid_count() {
        let mut dm = sample_depth_map();
        dm.depths[0] = f32::NAN;
        dm.depths[1] = f32::NAN;

        assert_eq!(dm.valid_pixel_count(), 98);
        assert!((dm.valid_fraction() - 0.98).abs() < 1e-6);
    }

    #[test]
    fn depth_map_unproject() {
        let dm = sample_depth_map();

        // Center pixel with depth 1.0 should be at approximately (0, 0, 1)
        let point = dm.unproject_pixel(5, 5);
        assert!(point.is_some());
        let p = point.unwrap_or([0.0, 0.0, 0.0]);
        assert!((p[2] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn depth_map_to_point_cloud() {
        let dm = sample_depth_map();
        let points = dm.to_point_cloud();
        assert_eq!(points.len(), 100);
    }

    #[test]
    fn depth_map_stats() {
        let mut dm = sample_depth_map();
        dm.depths[0] = 0.5;
        dm.depths[1] = 2.0;

        let stats = dm.depth_stats();
        assert!(stats.is_some());
        let s = stats.unwrap_or(DepthStats {
            min: 0.0,
            max: 0.0,
            mean: 0.0,
        });
        assert!((s.min - 0.5).abs() < 1e-6);
        assert!((s.max - 2.0).abs() < 1e-6);
    }

    #[test]
    fn depth_map_empty_stats() {
        let mut dm = sample_depth_map();
        for d in &mut dm.depths {
            *d = f32::NAN;
        }
        assert!(dm.depth_stats().is_none());
    }

    #[cfg(feature = "serde")]
    #[test]
    fn depth_serialization() {
        let stats = DepthStats {
            min: 0.5,
            max: 5.0,
            mean: 2.0,
        };
        let json = serde_json::to_string(&stats).ok();
        assert!(json.is_some());
    }
}
