//! Segmentation mask types for semantic and instance segmentation.

use serde::{Deserialize, Serialize};

/// A semantic segmentation mask.
///
/// Each pixel contains a class ID. Class 0 is typically background.
///
/// # Storage Format
///
/// The mask is stored as a flat `Vec<u8>` in row-major order (C-contiguous).
/// For a pixel at `(x, y)`, the index is `y * width + x`.
///
/// # Example
///
/// ```
/// use ml_types::SegmentationMask;
///
/// let mask = SegmentationMask::new(640, 480);
/// assert_eq!(mask.width(), 640);
/// assert_eq!(mask.height(), 480);
/// assert_eq!(mask.data().len(), 640 * 480);
/// ```
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SegmentationMask {
    /// Width in pixels.
    width: u32,
    /// Height in pixels.
    height: u32,
    /// Flat pixel data (class IDs), row-major order.
    data: Vec<u8>,
}

impl SegmentationMask {
    /// Creates a new segmentation mask filled with zeros (background).
    #[must_use]
    pub fn new(width: u32, height: u32) -> Self {
        let size = (width as usize) * (height as usize);
        Self {
            width,
            height,
            data: vec![0; size],
        }
    }

    /// Creates a segmentation mask from existing data.
    ///
    /// Returns `None` if data length doesn't match dimensions.
    #[must_use]
    pub fn from_data(width: u32, height: u32, data: Vec<u8>) -> Option<Self> {
        let expected = (width as usize) * (height as usize);
        if data.len() != expected {
            return None;
        }
        Some(Self {
            width,
            height,
            data,
        })
    }

    /// Returns the width in pixels.
    #[must_use]
    pub const fn width(&self) -> u32 {
        self.width
    }

    /// Returns the height in pixels.
    #[must_use]
    pub const fn height(&self) -> u32 {
        self.height
    }

    /// Returns a reference to the raw pixel data.
    #[must_use]
    pub fn data(&self) -> &[u8] {
        &self.data
    }

    /// Returns a mutable reference to the raw pixel data.
    pub fn data_mut(&mut self) -> &mut [u8] {
        &mut self.data
    }

    /// Gets the class ID at the specified pixel.
    ///
    /// Returns `None` if coordinates are out of bounds.
    #[must_use]
    pub fn get(&self, x: u32, y: u32) -> Option<u8> {
        if x >= self.width || y >= self.height {
            return None;
        }
        let idx = (y as usize) * (self.width as usize) + (x as usize);
        self.data.get(idx).copied()
    }

    /// Sets the class ID at the specified pixel.
    ///
    /// Returns `false` if coordinates are out of bounds.
    pub fn set(&mut self, x: u32, y: u32, class_id: u8) -> bool {
        if x >= self.width || y >= self.height {
            return false;
        }
        let idx = (y as usize) * (self.width as usize) + (x as usize);
        self.data.get_mut(idx).is_some_and(|pixel| {
            *pixel = class_id;
            true
        })
    }

    /// Returns the number of pixels for each class.
    ///
    /// Returns a sparse map of `class_id` -> pixel count.
    #[must_use]
    pub fn class_counts(&self) -> std::collections::HashMap<u8, usize> {
        let mut counts = std::collections::HashMap::new();
        for &class_id in &self.data {
            *counts.entry(class_id).or_insert(0) += 1;
        }
        counts
    }

    /// Returns the set of unique class IDs present in the mask.
    #[must_use]
    pub fn unique_classes(&self) -> Vec<u8> {
        let mut classes: Vec<u8> = self.class_counts().keys().copied().collect();
        classes.sort_unstable();
        classes
    }

    /// Computes the `IoU` (Intersection over Union) with another mask for a specific class.
    ///
    /// Returns `None` if masks have different dimensions.
    #[must_use]
    pub fn iou(&self, other: &Self, class_id: u8) -> Option<f32> {
        if self.width != other.width || self.height != other.height {
            return None;
        }

        let mut intersection = 0usize;
        let mut union = 0usize;

        for (a, b) in self.data.iter().zip(other.data.iter()) {
            let a_match = *a == class_id;
            let b_match = *b == class_id;

            if a_match && b_match {
                intersection += 1;
            }
            if a_match || b_match {
                union += 1;
            }
        }

        #[allow(clippy::cast_precision_loss)]
        if union == 0 {
            Some(1.0) // Both empty for this class
        } else {
            Some(intersection as f32 / union as f32)
        }
    }

    /// Computes mean `IoU` across all classes present in either mask.
    ///
    /// Returns `None` if masks have different dimensions.
    #[must_use]
    pub fn mean_iou(&self, other: &Self) -> Option<f32> {
        if self.width != other.width || self.height != other.height {
            return None;
        }

        let mut all_classes: std::collections::HashSet<u8> =
            self.unique_classes().into_iter().collect();
        all_classes.extend(other.unique_classes());

        // Exclude background (class 0) from mIoU calculation
        all_classes.remove(&0);

        if all_classes.is_empty() {
            return Some(1.0); // Only background in both
        }

        let mut total_iou = 0.0;
        for class_id in &all_classes {
            total_iou += self.iou(other, *class_id)?;
        }

        #[allow(clippy::cast_precision_loss)]
        Some(total_iou / all_classes.len() as f32)
    }
}

impl Default for SegmentationMask {
    fn default() -> Self {
        Self::new(0, 0)
    }
}

/// An instance segmentation mask.
///
/// Combines semantic segmentation with instance IDs, allowing
/// differentiation between multiple objects of the same class.
///
/// # Storage Format
///
/// Each pixel stores both a class ID and an instance ID.
/// Instance ID 0 means background/no instance.
///
/// # Example
///
/// ```
/// use ml_types::InstanceMask;
///
/// let mut mask = InstanceMask::new(640, 480);
/// mask.set(100, 100, 1, 1); // Class 1, Instance 1
/// mask.set(200, 200, 1, 2); // Class 1, Instance 2 (different object)
///
/// assert_eq!(mask.instance_count(), 2);
/// ```
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct InstanceMask {
    /// Width in pixels.
    width: u32,
    /// Height in pixels.
    height: u32,
    /// Class IDs, row-major order.
    class_data: Vec<u8>,
    /// Instance IDs, row-major order.
    instance_data: Vec<u16>,
}

impl InstanceMask {
    /// Creates a new instance mask filled with zeros.
    #[must_use]
    pub fn new(width: u32, height: u32) -> Self {
        let size = (width as usize) * (height as usize);
        Self {
            width,
            height,
            class_data: vec![0; size],
            instance_data: vec![0; size],
        }
    }

    /// Returns the width in pixels.
    #[must_use]
    pub const fn width(&self) -> u32 {
        self.width
    }

    /// Returns the height in pixels.
    #[must_use]
    pub const fn height(&self) -> u32 {
        self.height
    }

    /// Gets the class and instance ID at the specified pixel.
    ///
    /// Returns `None` if coordinates are out of bounds.
    #[must_use]
    pub fn get(&self, x: u32, y: u32) -> Option<(u8, u16)> {
        if x >= self.width || y >= self.height {
            return None;
        }
        let idx = (y as usize) * (self.width as usize) + (x as usize);
        Some((self.class_data[idx], self.instance_data[idx]))
    }

    /// Sets the class and instance ID at the specified pixel.
    ///
    /// Returns `false` if coordinates are out of bounds.
    pub fn set(&mut self, x: u32, y: u32, class_id: u8, instance_id: u16) -> bool {
        if x >= self.width || y >= self.height {
            return false;
        }
        let idx = (y as usize) * (self.width as usize) + (x as usize);
        self.class_data[idx] = class_id;
        self.instance_data[idx] = instance_id;
        true
    }

    /// Returns a reference to the class data.
    #[must_use]
    pub fn class_data(&self) -> &[u8] {
        &self.class_data
    }

    /// Returns a reference to the instance data.
    #[must_use]
    pub fn instance_data(&self) -> &[u16] {
        &self.instance_data
    }

    /// Extracts the semantic segmentation mask (class IDs only).
    #[must_use]
    pub fn to_semantic_mask(&self) -> SegmentationMask {
        SegmentationMask {
            width: self.width,
            height: self.height,
            data: self.class_data.clone(),
        }
    }

    /// Returns the number of unique instances (excluding background).
    #[must_use]
    pub fn instance_count(&self) -> usize {
        let unique: std::collections::HashSet<u16> = self.instance_data.iter().copied().collect();
        unique.len().saturating_sub(1) // Subtract 1 for background (instance 0)
    }

    /// Returns the unique instance IDs for a specific class.
    #[must_use]
    pub fn instances_of_class(&self, class_id: u8) -> Vec<u16> {
        let mut instances: std::collections::HashSet<u16> = std::collections::HashSet::new();

        for (class, instance) in self.class_data.iter().zip(self.instance_data.iter()) {
            if *class == class_id && *instance != 0 {
                instances.insert(*instance);
            }
        }

        let mut result: Vec<u16> = instances.into_iter().collect();
        result.sort_unstable();
        result
    }

    /// Computes the pixel area of a specific instance.
    #[must_use]
    pub fn instance_area(&self, instance_id: u16) -> usize {
        self.instance_data
            .iter()
            .filter(|&&id| id == instance_id)
            .count()
    }
}

impl Default for InstanceMask {
    fn default() -> Self {
        Self::new(0, 0)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn segmentation_mask_new() {
        let mask = SegmentationMask::new(100, 50);
        assert_eq!(mask.width(), 100);
        assert_eq!(mask.height(), 50);
        assert_eq!(mask.data().len(), 5000);
        assert!(mask.data().iter().all(|&v| v == 0));
    }

    #[test]
    fn segmentation_mask_from_data() {
        let data = vec![1, 2, 3, 4, 5, 6];
        let mask = SegmentationMask::from_data(3, 2, data.clone());
        assert!(mask.is_some());
        assert_eq!(mask.as_ref().map(|m| m.data().to_vec()), Some(data));

        // Wrong size
        let bad = SegmentationMask::from_data(10, 10, vec![0; 50]);
        assert!(bad.is_none());
    }

    #[test]
    fn segmentation_mask_get_set() {
        let mut mask = SegmentationMask::new(10, 10);

        assert!(mask.set(5, 5, 42));
        assert_eq!(mask.get(5, 5), Some(42));

        // Out of bounds
        assert!(!mask.set(100, 100, 1));
        assert_eq!(mask.get(100, 100), None);
    }

    #[test]
    fn segmentation_mask_class_counts() {
        let data = vec![0, 0, 1, 1, 1, 2];
        let mask = SegmentationMask::from_data(3, 2, data);
        let mask = mask.expect("valid mask");

        let counts = mask.class_counts();
        assert_eq!(counts.get(&0), Some(&2));
        assert_eq!(counts.get(&1), Some(&3));
        assert_eq!(counts.get(&2), Some(&1));
    }

    #[test]
    fn segmentation_mask_unique_classes() {
        let data = vec![0, 2, 1, 2, 1, 0];
        let mask = SegmentationMask::from_data(3, 2, data);
        let mask = mask.expect("valid mask");

        let classes = mask.unique_classes();
        assert_eq!(classes, vec![0, 1, 2]);
    }

    #[test]
    fn segmentation_mask_iou() {
        let a = SegmentationMask::from_data(2, 2, vec![1, 1, 0, 0]);
        let b = SegmentationMask::from_data(2, 2, vec![1, 0, 1, 0]);
        let a = a.expect("valid mask");
        let b = b.expect("valid mask");

        // Class 1: a has {0,1}, b has {0,2}
        // Intersection: {0}, Union: {0,1,2}
        // IoU = 1/3
        let iou = a.iou(&b, 1);
        assert!(iou.is_some());
        let iou_val = iou.unwrap_or(0.0);
        assert!((iou_val - 1.0 / 3.0).abs() < 1e-6);
    }

    #[test]
    fn segmentation_mask_iou_different_dims() {
        let a = SegmentationMask::new(10, 10);
        let b = SegmentationMask::new(20, 20);
        assert!(a.iou(&b, 1).is_none());
    }

    #[test]
    fn instance_mask_new() {
        let mask = InstanceMask::new(100, 50);
        assert_eq!(mask.width(), 100);
        assert_eq!(mask.height(), 50);
        assert_eq!(mask.class_data().len(), 5000);
        assert_eq!(mask.instance_data().len(), 5000);
    }

    #[test]
    fn instance_mask_get_set() {
        let mut mask = InstanceMask::new(10, 10);

        assert!(mask.set(3, 3, 5, 100));
        assert_eq!(mask.get(3, 3), Some((5, 100)));

        // Out of bounds
        assert!(!mask.set(100, 100, 1, 1));
        assert_eq!(mask.get(100, 100), None);
    }

    #[test]
    fn instance_mask_instance_count() {
        let mut mask = InstanceMask::new(10, 10);
        mask.set(0, 0, 1, 1);
        mask.set(1, 1, 1, 2);
        mask.set(2, 2, 2, 3);

        assert_eq!(mask.instance_count(), 3);
    }

    #[test]
    fn instance_mask_instances_of_class() {
        let mut mask = InstanceMask::new(10, 10);
        mask.set(0, 0, 1, 1);
        mask.set(1, 1, 1, 2);
        mask.set(2, 2, 2, 3);
        mask.set(3, 3, 1, 1); // Same class/instance as first

        let class1_instances = mask.instances_of_class(1);
        assert_eq!(class1_instances, vec![1, 2]);

        let class2_instances = mask.instances_of_class(2);
        assert_eq!(class2_instances, vec![3]);
    }

    #[test]
    fn instance_mask_to_semantic() {
        let mut mask = InstanceMask::new(3, 2);
        mask.set(0, 0, 1, 1);
        mask.set(1, 0, 1, 2);
        mask.set(2, 0, 2, 3);

        let semantic = mask.to_semantic_mask();
        assert_eq!(semantic.get(0, 0), Some(1));
        assert_eq!(semantic.get(1, 0), Some(1));
        assert_eq!(semantic.get(2, 0), Some(2));
    }

    #[test]
    fn instance_mask_instance_area() {
        let mut mask = InstanceMask::new(10, 10);
        mask.set(0, 0, 1, 1);
        mask.set(1, 0, 1, 1);
        mask.set(2, 0, 1, 1);
        mask.set(3, 0, 1, 2);

        assert_eq!(mask.instance_area(1), 3);
        assert_eq!(mask.instance_area(2), 1);
        assert_eq!(mask.instance_area(999), 0);
    }

    #[test]
    fn segmentation_mask_serialization() {
        let mask = SegmentationMask::from_data(2, 2, vec![1, 2, 3, 4]);
        let mask = mask.expect("valid mask");

        let json = serde_json::to_string(&mask);
        assert!(json.is_ok());

        let parsed: Result<SegmentationMask, _> = serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
        assert_eq!(parsed.unwrap_or_default(), mask);
    }

    #[test]
    fn instance_mask_serialization() {
        let mut mask = InstanceMask::new(2, 2);
        mask.set(0, 0, 1, 100);

        let json = serde_json::to_string(&mask);
        assert!(json.is_ok());

        let parsed: Result<InstanceMask, _> = serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
        assert_eq!(parsed.unwrap_or_default(), mask);
    }
}
