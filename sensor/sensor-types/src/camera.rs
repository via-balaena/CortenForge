//! Camera sensor types.
//!
//! Provides types for raw camera frames and calibration data.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::{CoordinateFrame, Pose3d, Timestamp};

/// Image encoding format.
///
/// Describes how pixel data is stored in the raw byte buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum ImageEncoding {
    /// RGBA 8-bit per channel (4 bytes per pixel).
    #[default]
    Rgba8,
    /// RGB 8-bit per channel (3 bytes per pixel).
    Rgb8,
    /// BGR 8-bit per channel (3 bytes per pixel, `OpenCV` convention).
    Bgr8,
    /// BGRA 8-bit per channel (4 bytes per pixel, `OpenCV` convention).
    Bgra8,
    /// Grayscale 8-bit (1 byte per pixel).
    Gray8,
    /// Grayscale 16-bit (2 bytes per pixel).
    Gray16,
    /// YUV420 planar format (1.5 bytes per pixel on average).
    Yuv420,
    /// NV12 format (Y plane + interleaved UV).
    Nv12,
    /// Bayer RGGB pattern (1 byte per pixel, needs demosaicing).
    BayerRggb8,
    /// Bayer BGGR pattern (1 byte per pixel, needs demosaicing).
    BayerBggr8,
}

impl ImageEncoding {
    /// Returns the number of bytes per pixel for this encoding.
    ///
    /// Returns `None` for planar/compressed formats where bytes-per-pixel
    /// varies or doesn't apply directly.
    #[must_use]
    pub const fn bytes_per_pixel(&self) -> Option<usize> {
        match self {
            Self::Rgba8 | Self::Bgra8 => Some(4),
            Self::Rgb8 | Self::Bgr8 => Some(3),
            Self::Gray8 | Self::BayerRggb8 | Self::BayerBggr8 => Some(1),
            Self::Gray16 => Some(2),
            Self::Yuv420 | Self::Nv12 => None, // Planar format
        }
    }

    /// Returns the expected buffer size for an image with this encoding.
    ///
    /// Returns `None` for formats where size calculation is complex.
    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    pub const fn buffer_size(&self, width: u32, height: u32) -> Option<usize> {
        let pixels = width as usize * height as usize;
        match self {
            Self::Rgba8 | Self::Bgra8 => Some(pixels * 4),
            Self::Rgb8 | Self::Bgr8 => Some(pixels * 3),
            Self::Gray8 | Self::BayerRggb8 | Self::BayerBggr8 => Some(pixels),
            Self::Gray16 => Some(pixels * 2),
            // Y + U/4 + V/4 (YUV420), Y + UV/2 (NV12)
            Self::Yuv420 | Self::Nv12 => Some(pixels * 3 / 2),
        }
    }
}

/// Camera intrinsic parameters (pinhole model).
///
/// Describes the camera's internal geometry: focal length, principal point,
/// and lens distortion coefficients.
///
/// # Pinhole Model
///
/// Projects a 3D point `[X, Y, Z]` to 2D pixel coordinates:
/// ```text
/// u = fx * X/Z + cx
/// v = fy * Y/Z + cy
/// ```
///
/// # Distortion Model
///
/// Uses the standard 5-parameter model (k1, k2, p1, p2, k3):
/// - k1, k2, k3: radial distortion coefficients
/// - p1, p2: tangential distortion coefficients
///
/// # Example
///
/// ```
/// use sensor_types::CameraIntrinsics;
///
/// let intrinsics = CameraIntrinsics {
///     fx: 500.0,
///     fy: 500.0,
///     cx: 320.0,
///     cy: 240.0,
///     distortion: [0.0, 0.0, 0.0, 0.0, 0.0],
///     width: 640,
///     height: 480,
/// };
///
/// assert_eq!(intrinsics.aspect_ratio(), 640.0 / 480.0);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CameraIntrinsics {
    /// Focal length in pixels (x direction).
    pub fx: f64,
    /// Focal length in pixels (y direction).
    pub fy: f64,
    /// Principal point x-coordinate in pixels.
    pub cx: f64,
    /// Principal point y-coordinate in pixels.
    pub cy: f64,
    /// Distortion coefficients: `[k1, k2, p1, p2, k3]`.
    pub distortion: [f64; 5],
    /// Image width in pixels.
    pub width: u32,
    /// Image height in pixels.
    pub height: u32,
}

impl CameraIntrinsics {
    /// Creates new camera intrinsics with no distortion.
    #[must_use]
    pub const fn new(fx: f64, fy: f64, cx: f64, cy: f64, width: u32, height: u32) -> Self {
        Self {
            fx,
            fy,
            cx,
            cy,
            distortion: [0.0, 0.0, 0.0, 0.0, 0.0],
            width,
            height,
        }
    }

    /// Creates intrinsics for an ideal pinhole camera centered in the image.
    ///
    /// Uses the given focal length and places the principal point at the image center.
    #[must_use]
    pub fn ideal(focal_length: f64, width: u32, height: u32) -> Self {
        Self {
            fx: focal_length,
            fy: focal_length,
            cx: f64::from(width) / 2.0,
            cy: f64::from(height) / 2.0,
            distortion: [0.0, 0.0, 0.0, 0.0, 0.0],
            width,
            height,
        }
    }

    /// Returns the image aspect ratio (width / height).
    #[must_use]
    pub fn aspect_ratio(&self) -> f64 {
        f64::from(self.width) / f64::from(self.height)
    }

    /// Returns the horizontal field of view in radians.
    #[must_use]
    pub fn fov_x(&self) -> f64 {
        2.0 * (f64::from(self.width) / (2.0 * self.fx)).atan()
    }

    /// Returns the vertical field of view in radians.
    #[must_use]
    pub fn fov_y(&self) -> f64 {
        2.0 * (f64::from(self.height) / (2.0 * self.fy)).atan()
    }

    /// Projects a 3D point to 2D pixel coordinates (no distortion).
    ///
    /// Returns `None` if the point is behind the camera (Z <= 0).
    #[must_use]
    pub fn project(&self, point: [f64; 3]) -> Option<[f64; 2]> {
        let [x, y, z] = point;
        if z <= 0.0 {
            return None;
        }
        Some([self.fx * x / z + self.cx, self.fy * y / z + self.cy])
    }

    /// Unprojects a 2D pixel to a 3D ray direction (no distortion).
    ///
    /// Returns a normalized direction vector.
    #[must_use]
    pub fn unproject(&self, pixel: [f64; 2]) -> [f64; 3] {
        let x = (pixel[0] - self.cx) / self.fx;
        let y = (pixel[1] - self.cy) / self.fy;
        let norm = x.mul_add(x, y.mul_add(y, 1.0)).sqrt();
        [x / norm, y / norm, 1.0 / norm]
    }
}

impl Default for CameraIntrinsics {
    fn default() -> Self {
        Self::ideal(500.0, 640, 480)
    }
}

/// Camera extrinsic parameters.
///
/// Describes the camera's position and orientation in the world or body frame.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CameraExtrinsics {
    /// Camera pose (position and orientation).
    pub pose: Pose3d,
    /// Reference frame for the pose.
    pub frame: CoordinateFrame,
}

impl CameraExtrinsics {
    /// Creates new camera extrinsics.
    #[must_use]
    pub const fn new(pose: Pose3d, frame: CoordinateFrame) -> Self {
        Self { pose, frame }
    }

    /// Creates extrinsics at the origin with no rotation.
    #[must_use]
    pub const fn identity(frame: CoordinateFrame) -> Self {
        Self {
            pose: Pose3d::identity(),
            frame,
        }
    }
}

impl Default for CameraExtrinsics {
    fn default() -> Self {
        Self::identity(CoordinateFrame::Body)
    }
}

/// A raw camera frame with image data and calibration.
///
/// Contains the raw pixel data along with intrinsic and extrinsic calibration,
/// enabling accurate 3D reasoning about the scene.
///
/// # Example
///
/// ```
/// use sensor_types::{CameraFrame, CameraIntrinsics, CameraExtrinsics, ImageEncoding, Timestamp};
///
/// let frame = CameraFrame {
///     timestamp: Timestamp::from_secs_f64(1.0),
///     image: vec![0u8; 640 * 480 * 4],  // RGBA8
///     width: 640,
///     height: 480,
///     encoding: ImageEncoding::Rgba8,
///     intrinsics: CameraIntrinsics::ideal(500.0, 640, 480),
///     extrinsics: CameraExtrinsics::default(),
/// };
///
/// assert_eq!(frame.pixel_count(), 640 * 480);
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CameraFrame {
    /// Timestamp when the frame was captured.
    pub timestamp: Timestamp,

    /// Raw image data.
    pub image: Vec<u8>,

    /// Image width in pixels.
    pub width: u32,

    /// Image height in pixels.
    pub height: u32,

    /// Pixel encoding format.
    pub encoding: ImageEncoding,

    /// Camera intrinsic parameters.
    pub intrinsics: CameraIntrinsics,

    /// Camera extrinsic parameters.
    pub extrinsics: CameraExtrinsics,
}

impl CameraFrame {
    /// Returns the total number of pixels.
    #[must_use]
    pub const fn pixel_count(&self) -> u32 {
        self.width * self.height
    }

    /// Returns the expected buffer size for this frame.
    #[must_use]
    pub const fn expected_buffer_size(&self) -> Option<usize> {
        self.encoding.buffer_size(self.width, self.height)
    }

    /// Checks if the image buffer has the expected size.
    #[must_use]
    pub fn has_valid_buffer_size(&self) -> bool {
        self.expected_buffer_size()
            .is_none_or(|expected| self.image.len() == expected)
    }

    /// Returns the image aspect ratio.
    #[must_use]
    pub fn aspect_ratio(&self) -> f64 {
        f64::from(self.width) / f64::from(self.height)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encoding_bytes_per_pixel() {
        assert_eq!(ImageEncoding::Rgba8.bytes_per_pixel(), Some(4));
        assert_eq!(ImageEncoding::Rgb8.bytes_per_pixel(), Some(3));
        assert_eq!(ImageEncoding::Gray8.bytes_per_pixel(), Some(1));
        assert_eq!(ImageEncoding::Yuv420.bytes_per_pixel(), None);
    }

    #[test]
    fn encoding_buffer_size() {
        assert_eq!(
            ImageEncoding::Rgba8.buffer_size(640, 480),
            Some(640 * 480 * 4)
        );
        assert_eq!(
            ImageEncoding::Rgb8.buffer_size(640, 480),
            Some(640 * 480 * 3)
        );
        assert_eq!(
            ImageEncoding::Yuv420.buffer_size(640, 480),
            Some(640 * 480 * 3 / 2)
        );
    }

    #[test]
    fn intrinsics_ideal() {
        let intr = CameraIntrinsics::ideal(500.0, 640, 480);
        assert!((intr.cx - 320.0).abs() < 1e-10);
        assert!((intr.cy - 240.0).abs() < 1e-10);
    }

    #[test]
    fn intrinsics_fov() {
        let intr = CameraIntrinsics::ideal(320.0, 640, 480);
        // FOV should be approximately 90 degrees (pi/2 radians)
        let fov_x = intr.fov_x();
        assert!((fov_x - std::f64::consts::FRAC_PI_2).abs() < 0.1);
    }

    #[test]
    fn intrinsics_project() {
        let intr = CameraIntrinsics::ideal(500.0, 640, 480);

        // Point on optical axis
        let pixel = intr.project([0.0, 0.0, 1.0]);
        assert!(pixel.is_some());
        let p = pixel.unwrap_or([0.0, 0.0]);
        assert!((p[0] - 320.0).abs() < 1e-10);
        assert!((p[1] - 240.0).abs() < 1e-10);

        // Point behind camera
        assert!(intr.project([0.0, 0.0, -1.0]).is_none());
    }

    #[test]
    fn intrinsics_unproject() {
        let intr = CameraIntrinsics::ideal(500.0, 640, 480);

        // Principal point should give forward direction
        let ray = intr.unproject([320.0, 240.0]);
        assert!(ray[0].abs() < 1e-10);
        assert!(ray[1].abs() < 1e-10);
        assert!(ray[2] > 0.99);
    }

    #[test]
    fn frame_buffer_validation() {
        let frame = CameraFrame {
            timestamp: Timestamp::zero(),
            image: vec![0u8; 640 * 480 * 4],
            width: 640,
            height: 480,
            encoding: ImageEncoding::Rgba8,
            intrinsics: CameraIntrinsics::default(),
            extrinsics: CameraExtrinsics::default(),
        };

        assert!(frame.has_valid_buffer_size());
        assert_eq!(frame.pixel_count(), 640 * 480);
    }

    #[test]
    fn frame_invalid_buffer() {
        let frame = CameraFrame {
            timestamp: Timestamp::zero(),
            image: vec![0u8; 100], // Wrong size
            width: 640,
            height: 480,
            encoding: ImageEncoding::Rgba8,
            intrinsics: CameraIntrinsics::default(),
            extrinsics: CameraExtrinsics::default(),
        };

        assert!(!frame.has_valid_buffer_size());
    }

    #[cfg(feature = "serde")]
    #[test]
    fn camera_serialization() {
        let intr = CameraIntrinsics::ideal(500.0, 640, 480);
        let json = serde_json::to_string(&intr).ok();
        assert!(json.is_some());
    }
}
