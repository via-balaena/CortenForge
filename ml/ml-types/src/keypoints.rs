//! Keypoint detection types for pose estimation and landmark detection.

use serde::{Deserialize, Serialize};

/// A single keypoint (landmark) detection.
///
/// Represents a detected point with 2D position and confidence.
/// Coordinates are normalized to `[0, 1]` range.
///
/// # Example
///
/// ```
/// use ml_types::Keypoint;
///
/// let nose = Keypoint::new(0.5, 0.3, 0.95);
/// assert!(nose.is_visible(0.5));
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Keypoint {
    /// X coordinate (normalized `[0, 1]`).
    pub x: f32,
    /// Y coordinate (normalized `[0, 1]`).
    pub y: f32,
    /// Detection confidence `[0, 1]`.
    pub confidence: f32,
}

impl Keypoint {
    /// Creates a new keypoint.
    #[must_use]
    pub const fn new(x: f32, y: f32, confidence: f32) -> Self {
        Self { x, y, confidence }
    }

    /// Creates an invisible/undetected keypoint.
    #[must_use]
    pub const fn invisible() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            confidence: 0.0,
        }
    }

    /// Returns `true` if the keypoint is detected above the threshold.
    #[must_use]
    pub fn is_visible(&self, threshold: f32) -> bool {
        self.confidence >= threshold
    }

    /// Returns the pixel coordinates for a given image size.
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn to_pixels(&self, width: u32, height: u32) -> (f32, f32) {
        (self.x * width as f32, self.y * height as f32)
    }

    /// Creates a keypoint from pixel coordinates.
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn from_pixels(x: f32, y: f32, confidence: f32, width: u32, height: u32) -> Self {
        Self {
            x: x / width as f32,
            y: y / height as f32,
            confidence,
        }
    }

    /// Computes the Euclidean distance to another keypoint.
    #[must_use]
    pub fn distance(&self, other: &Self) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        dx.hypot(dy)
    }

    /// Returns the keypoint with coordinates clamped to `[0, 1]`.
    #[must_use]
    #[allow(clippy::missing_const_for_fn)]
    pub fn clamped(&self) -> Self {
        Self {
            x: self.x.clamp(0.0, 1.0),
            y: self.y.clamp(0.0, 1.0),
            confidence: self.confidence.clamp(0.0, 1.0),
        }
    }

    /// Checks if the keypoint coordinates are valid (within `[0, 1]`).
    #[must_use]
    pub fn is_valid(&self) -> bool {
        (0.0..=1.0).contains(&self.x)
            && (0.0..=1.0).contains(&self.y)
            && (0.0..=1.0).contains(&self.confidence)
    }
}

impl Default for Keypoint {
    fn default() -> Self {
        Self::invisible()
    }
}

/// A set of keypoints for a single detected object.
///
/// The keypoint indices follow a predefined skeleton topology.
/// Common topologies include COCO (17 keypoints) and `MediaPipe` (33 keypoints).
///
/// # Example
///
/// ```
/// use ml_types::{Keypoint, KeypointSet};
///
/// // Create a simple 3-keypoint set (e.g., head, left_hand, right_hand)
/// let keypoints = vec![
///     Keypoint::new(0.5, 0.2, 0.9),  // head
///     Keypoint::new(0.3, 0.5, 0.8),  // left_hand
///     Keypoint::new(0.7, 0.5, 0.85), // right_hand
/// ];
///
/// let set = KeypointSet::new(keypoints, 0.95);
/// assert_eq!(set.len(), 3);
/// ```
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct KeypointSet {
    /// Ordered keypoints following the skeleton topology.
    keypoints: Vec<Keypoint>,
    /// Overall detection confidence for this set.
    confidence: f32,
}

impl KeypointSet {
    /// Creates a new keypoint set.
    #[must_use]
    #[allow(clippy::missing_const_for_fn)]
    pub fn new(keypoints: Vec<Keypoint>, confidence: f32) -> Self {
        Self {
            keypoints,
            confidence,
        }
    }

    /// Creates an empty keypoint set.
    #[must_use]
    #[allow(clippy::missing_const_for_fn)]
    pub fn empty() -> Self {
        Self {
            keypoints: Vec::new(),
            confidence: 0.0,
        }
    }

    /// Returns the number of keypoints.
    #[must_use]
    pub fn len(&self) -> usize {
        self.keypoints.len()
    }

    /// Returns `true` if there are no keypoints.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.keypoints.is_empty()
    }

    /// Returns the overall detection confidence.
    #[must_use]
    pub const fn confidence(&self) -> f32 {
        self.confidence
    }

    /// Returns a reference to the keypoints.
    #[must_use]
    pub fn keypoints(&self) -> &[Keypoint] {
        &self.keypoints
    }

    /// Returns a mutable reference to the keypoints.
    #[allow(clippy::missing_const_for_fn)]
    pub fn keypoints_mut(&mut self) -> &mut Vec<Keypoint> {
        &mut self.keypoints
    }

    /// Gets a keypoint by index.
    #[must_use]
    pub fn get(&self, index: usize) -> Option<&Keypoint> {
        self.keypoints.get(index)
    }

    /// Returns the number of visible keypoints above the threshold.
    #[must_use]
    pub fn visible_count(&self, threshold: f32) -> usize {
        self.keypoints
            .iter()
            .filter(|kp| kp.is_visible(threshold))
            .count()
    }

    /// Returns the mean confidence of all keypoints.
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn mean_confidence(&self) -> f32 {
        if self.keypoints.is_empty() {
            return 0.0;
        }
        let sum: f32 = self.keypoints.iter().map(|kp| kp.confidence).sum();
        sum / self.keypoints.len() as f32
    }

    /// Computes the bounding box containing all visible keypoints.
    ///
    /// Returns `None` if no keypoints are visible.
    #[must_use]
    pub fn bounding_box(&self, threshold: f32) -> Option<(f32, f32, f32, f32)> {
        let visible: Vec<_> = self
            .keypoints
            .iter()
            .filter(|kp| kp.is_visible(threshold))
            .collect();

        if visible.is_empty() {
            return None;
        }

        let min_x = visible.iter().map(|kp| kp.x).fold(f32::INFINITY, f32::min);
        let max_x = visible
            .iter()
            .map(|kp| kp.x)
            .fold(f32::NEG_INFINITY, f32::max);
        let min_y = visible.iter().map(|kp| kp.y).fold(f32::INFINITY, f32::min);
        let max_y = visible
            .iter()
            .map(|kp| kp.y)
            .fold(f32::NEG_INFINITY, f32::max);

        Some((min_x, min_y, max_x, max_y))
    }

    /// Computes the Object Keypoint Similarity (OKS) with another keypoint set.
    ///
    /// OKS is the standard metric for keypoint detection evaluation.
    /// Returns a value in `[0, 1]` where 1 is perfect match.
    ///
    /// # Arguments
    ///
    /// * `other` - The ground truth keypoint set
    /// * `sigmas` - Per-keypoint standard deviations (controls sensitivity)
    /// * `area` - Object area for normalization
    #[must_use]
    pub fn oks(&self, other: &Self, sigmas: &[f32], area: f32) -> Option<f32> {
        if self.len() != other.len() || self.len() != sigmas.len() {
            return None;
        }

        if area <= 0.0 {
            return None;
        }

        let mut sum = 0.0;
        let mut count = 0;

        for ((pred, gt), sigma) in self
            .keypoints
            .iter()
            .zip(other.keypoints.iter())
            .zip(sigmas.iter())
        {
            // Only count visible ground truth keypoints
            if gt.confidence < 0.5 {
                continue;
            }

            let dx = pred.x - gt.x;
            let dy = pred.y - gt.y;
            let d_squared = dx.mul_add(dx, dy * dy);

            let s_squared = sigma * sigma;
            let k_squared = 4.0 * s_squared; // 2*sigma squared

            let exp_arg = -d_squared / (2.0 * area * k_squared);
            sum += exp_arg.exp();
            count += 1;
        }

        #[allow(clippy::cast_precision_loss)]
        if count == 0 {
            Some(0.0)
        } else {
            Some(sum / count as f32)
        }
    }
}

impl Default for KeypointSet {
    fn default() -> Self {
        Self::empty()
    }
}

/// A skeleton topology defining connections between keypoints.
///
/// Used for visualization and structural validation.
///
/// # Example
///
/// ```
/// use ml_types::Skeleton;
///
/// // Simple 3-keypoint skeleton: head -> left_hand, head -> right_hand
/// let skeleton = Skeleton::new(
///     vec!["head".to_string(), "left_hand".to_string(), "right_hand".to_string()],
///     vec![(0, 1), (0, 2)],
/// );
///
/// assert_eq!(skeleton.num_keypoints(), 3);
/// assert_eq!(skeleton.num_bones(), 2);
/// ```
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Skeleton {
    /// Names of keypoints (in order).
    names: Vec<String>,
    /// Bone connections as `(from_index, to_index)` pairs.
    bones: Vec<(usize, usize)>,
}

impl Skeleton {
    /// Creates a new skeleton topology.
    #[must_use]
    #[allow(clippy::missing_const_for_fn)]
    pub fn new(names: Vec<String>, bones: Vec<(usize, usize)>) -> Self {
        Self { names, bones }
    }

    /// Returns the number of keypoints in this skeleton.
    #[must_use]
    pub fn num_keypoints(&self) -> usize {
        self.names.len()
    }

    /// Returns the number of bones (connections).
    #[must_use]
    pub fn num_bones(&self) -> usize {
        self.bones.len()
    }

    /// Returns the keypoint names.
    #[must_use]
    pub fn names(&self) -> &[String] {
        &self.names
    }

    /// Returns the bone connections.
    #[must_use]
    pub fn bones(&self) -> &[(usize, usize)] {
        &self.bones
    }

    /// Gets the index of a keypoint by name.
    #[must_use]
    pub fn index_of(&self, name: &str) -> Option<usize> {
        self.names.iter().position(|n| n == name)
    }

    /// Gets the name of a keypoint by index.
    #[must_use]
    pub fn name_of(&self, index: usize) -> Option<&str> {
        self.names.get(index).map(String::as_str)
    }

    /// Validates that all bone indices are within range.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        let n = self.names.len();
        self.bones.iter().all(|&(a, b)| a < n && b < n)
    }

    /// Creates the COCO human pose skeleton (17 keypoints).
    #[must_use]
    pub fn coco_pose() -> Self {
        let names = vec![
            "nose".to_string(),
            "left_eye".to_string(),
            "right_eye".to_string(),
            "left_ear".to_string(),
            "right_ear".to_string(),
            "left_shoulder".to_string(),
            "right_shoulder".to_string(),
            "left_elbow".to_string(),
            "right_elbow".to_string(),
            "left_wrist".to_string(),
            "right_wrist".to_string(),
            "left_hip".to_string(),
            "right_hip".to_string(),
            "left_knee".to_string(),
            "right_knee".to_string(),
            "left_ankle".to_string(),
            "right_ankle".to_string(),
        ];

        let bones = vec![
            (0, 1),   // nose -> left_eye
            (0, 2),   // nose -> right_eye
            (1, 3),   // left_eye -> left_ear
            (2, 4),   // right_eye -> right_ear
            (5, 6),   // left_shoulder -> right_shoulder
            (5, 7),   // left_shoulder -> left_elbow
            (7, 9),   // left_elbow -> left_wrist
            (6, 8),   // right_shoulder -> right_elbow
            (8, 10),  // right_elbow -> right_wrist
            (5, 11),  // left_shoulder -> left_hip
            (6, 12),  // right_shoulder -> right_hip
            (11, 12), // left_hip -> right_hip
            (11, 13), // left_hip -> left_knee
            (13, 15), // left_knee -> left_ankle
            (12, 14), // right_hip -> right_knee
            (14, 16), // right_knee -> right_ankle
        ];

        Self { names, bones }
    }
}

impl Default for Skeleton {
    fn default() -> Self {
        Self::coco_pose()
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
    fn keypoint_new() {
        let kp = Keypoint::new(0.5, 0.3, 0.9);
        assert!((kp.x - 0.5).abs() < 1e-6);
        assert!((kp.y - 0.3).abs() < 1e-6);
        assert!((kp.confidence - 0.9).abs() < 1e-6);
    }

    #[test]
    fn keypoint_invisible() {
        let kp = Keypoint::invisible();
        assert!(!kp.is_visible(0.1));
        assert!((kp.confidence - 0.0).abs() < 1e-6);
    }

    #[test]
    fn keypoint_is_visible() {
        let kp = Keypoint::new(0.5, 0.5, 0.75);
        assert!(kp.is_visible(0.5));
        assert!(kp.is_visible(0.75));
        assert!(!kp.is_visible(0.76));
    }

    #[test]
    fn keypoint_to_from_pixels() {
        let kp = Keypoint::new(0.5, 0.25, 0.9);
        let (px, py) = kp.to_pixels(640, 480);
        assert!((px - 320.0).abs() < 1e-3);
        assert!((py - 120.0).abs() < 1e-3);

        let back = Keypoint::from_pixels(320.0, 120.0, 0.9, 640, 480);
        assert!((back.x - 0.5).abs() < 1e-6);
        assert!((back.y - 0.25).abs() < 1e-6);
    }

    #[test]
    fn keypoint_distance() {
        let a = Keypoint::new(0.0, 0.0, 1.0);
        let b = Keypoint::new(0.3, 0.4, 1.0);
        let d = a.distance(&b);
        assert!((d - 0.5).abs() < 1e-6); // 3-4-5 triangle scaled
    }

    #[test]
    fn keypoint_clamped() {
        let kp = Keypoint::new(-0.1, 1.5, 0.9);
        let clamped = kp.clamped();
        assert!((clamped.x - 0.0).abs() < 1e-6);
        assert!((clamped.y - 1.0).abs() < 1e-6);
    }

    #[test]
    fn keypoint_is_valid() {
        assert!(Keypoint::new(0.5, 0.5, 0.5).is_valid());
        assert!(!Keypoint::new(-0.1, 0.5, 0.5).is_valid());
        assert!(!Keypoint::new(0.5, 1.1, 0.5).is_valid());
    }

    #[test]
    fn keypoint_set_new() {
        let kps = vec![Keypoint::new(0.1, 0.2, 0.9), Keypoint::new(0.3, 0.4, 0.8)];
        let set = KeypointSet::new(kps, 0.85);

        assert_eq!(set.len(), 2);
        assert!(!set.is_empty());
        assert!((set.confidence() - 0.85).abs() < 1e-6);
    }

    #[test]
    fn keypoint_set_visible_count() {
        let kps = vec![
            Keypoint::new(0.1, 0.2, 0.9),
            Keypoint::new(0.3, 0.4, 0.3),
            Keypoint::new(0.5, 0.6, 0.6),
        ];
        let set = KeypointSet::new(kps, 0.7);

        assert_eq!(set.visible_count(0.5), 2);
        assert_eq!(set.visible_count(0.8), 1);
    }

    #[test]
    fn keypoint_set_mean_confidence() {
        let kps = vec![
            Keypoint::new(0.0, 0.0, 0.6),
            Keypoint::new(0.0, 0.0, 0.8),
            Keypoint::new(0.0, 0.0, 1.0),
        ];
        let set = KeypointSet::new(kps, 0.8);

        assert!((set.mean_confidence() - 0.8).abs() < 1e-6);
    }

    #[test]
    fn keypoint_set_bounding_box() {
        let kps = vec![
            Keypoint::new(0.1, 0.2, 0.9),
            Keypoint::new(0.5, 0.3, 0.9),
            Keypoint::new(0.3, 0.8, 0.9),
            Keypoint::new(0.9, 0.9, 0.1), // Below threshold
        ];
        let set = KeypointSet::new(kps, 0.8);

        let bbox = set.bounding_box(0.5);
        assert!(bbox.is_some());
        let (min_x, min_y, max_x, max_y) = bbox.unwrap_or_default();
        assert!((min_x - 0.1).abs() < 1e-6);
        assert!((min_y - 0.2).abs() < 1e-6);
        assert!((max_x - 0.5).abs() < 1e-6);
        assert!((max_y - 0.8).abs() < 1e-6);
    }

    #[test]
    fn keypoint_set_empty_bounding_box() {
        let kps = vec![Keypoint::new(0.5, 0.5, 0.1)];
        let set = KeypointSet::new(kps, 0.1);

        assert!(set.bounding_box(0.5).is_none());
    }

    #[test]
    fn skeleton_new() {
        let skel = Skeleton::new(
            vec!["a".to_string(), "b".to_string(), "c".to_string()],
            vec![(0, 1), (1, 2)],
        );

        assert_eq!(skel.num_keypoints(), 3);
        assert_eq!(skel.num_bones(), 2);
        assert!(skel.is_valid());
    }

    #[test]
    fn skeleton_index_name() {
        let skel = Skeleton::new(vec!["head".to_string(), "hand".to_string()], vec![(0, 1)]);

        assert_eq!(skel.index_of("hand"), Some(1));
        assert_eq!(skel.index_of("foot"), None);
        assert_eq!(skel.name_of(0), Some("head"));
        assert_eq!(skel.name_of(99), None);
    }

    #[test]
    fn skeleton_is_valid() {
        let valid = Skeleton::new(vec!["a".to_string(), "b".to_string()], vec![(0, 1)]);
        assert!(valid.is_valid());

        let invalid = Skeleton::new(vec!["a".to_string()], vec![(0, 5)]);
        assert!(!invalid.is_valid());
    }

    #[test]
    fn skeleton_coco_pose() {
        let skel = Skeleton::coco_pose();
        assert_eq!(skel.num_keypoints(), 17);
        assert_eq!(skel.num_bones(), 16);
        assert!(skel.is_valid());
        assert_eq!(skel.index_of("nose"), Some(0));
        assert_eq!(skel.index_of("left_ankle"), Some(15));
    }

    #[test]
    fn keypoint_serialization() {
        let kp = Keypoint::new(0.5, 0.5, 0.9);
        let json = serde_json::to_string(&kp);
        assert!(json.is_ok());

        let parsed: Result<Keypoint, _> = serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
        assert_eq!(parsed.unwrap_or_default(), kp);
    }

    #[test]
    fn keypoint_set_serialization() {
        let set = KeypointSet::new(vec![Keypoint::new(0.1, 0.2, 0.9)], 0.9);
        let json = serde_json::to_string(&set);
        assert!(json.is_ok());

        let parsed: Result<KeypointSet, _> = serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
    }

    #[test]
    fn skeleton_serialization() {
        let skel = Skeleton::coco_pose();
        let json = serde_json::to_string(&skel);
        assert!(json.is_ok());

        let parsed: Result<Skeleton, _> = serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
    }
}
