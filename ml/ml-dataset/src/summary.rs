//! Dataset summary and statistics.

use serde::{Deserialize, Serialize};

use crate::sample::DatasetSample;

/// Summary statistics for a dataset.
///
/// Provides aggregate information about dataset composition,
/// useful for validation and debugging.
///
/// # Example
///
/// ```
/// use ml_dataset::{DatasetSample, DatasetSummary};
///
/// let samples: Vec<DatasetSample> = (0..10)
///     .map(|i| DatasetSample::empty(i))
///     .collect();
///
/// let summary = DatasetSummary::from_samples(&samples);
/// assert_eq!(summary.total_samples, 10);
/// assert_eq!(summary.positive_samples, 0);
/// ```
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DatasetSummary {
    /// Total number of samples.
    pub total_samples: usize,

    /// Number of samples with at least one bounding box.
    pub positive_samples: usize,

    /// Number of samples with no bounding boxes.
    pub negative_samples: usize,

    /// Total number of bounding boxes across all samples.
    pub total_boxes: usize,

    /// Average boxes per positive sample.
    pub avg_boxes_per_positive: f32,

    /// Positive sample ratio (0 to 1).
    pub positive_ratio: f32,

    /// Distribution of box counts (index = count, value = frequency).
    pub box_count_histogram: Vec<usize>,

    /// Optional class distribution (`class_id` -> count).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub class_distribution: Vec<(u32, usize)>,
}

impl Default for DatasetSummary {
    fn default() -> Self {
        Self {
            total_samples: 0,
            positive_samples: 0,
            negative_samples: 0,
            total_boxes: 0,
            avg_boxes_per_positive: 0.0,
            positive_ratio: 0.0,
            box_count_histogram: Vec::new(),
            class_distribution: Vec::new(),
        }
    }
}

impl DatasetSummary {
    /// Creates a summary from a slice of samples.
    #[must_use]
    pub fn from_samples(samples: &[DatasetSample]) -> Self {
        if samples.is_empty() {
            return Self::default();
        }

        let total_samples = samples.len();
        let mut positive_samples = 0;
        let mut total_boxes = 0;
        let mut max_boxes = 0;

        for sample in samples {
            let box_count = sample.boxes.len();
            if box_count > 0 {
                positive_samples += 1;
            }
            total_boxes += box_count;
            max_boxes = max_boxes.max(box_count);
        }

        let negative_samples = total_samples - positive_samples;

        #[allow(clippy::cast_precision_loss)]
        let avg_boxes_per_positive = if positive_samples > 0 {
            total_boxes as f32 / positive_samples as f32
        } else {
            0.0
        };

        #[allow(clippy::cast_precision_loss)]
        let positive_ratio = positive_samples as f32 / total_samples as f32;

        // Build histogram
        let mut box_count_histogram = vec![0usize; max_boxes + 1];
        for sample in samples {
            box_count_histogram[sample.boxes.len()] += 1;
        }

        Self {
            total_samples,
            positive_samples,
            negative_samples,
            total_boxes,
            avg_boxes_per_positive,
            positive_ratio,
            box_count_histogram,
            class_distribution: Vec::new(),
        }
    }

    /// Creates a summary with class distribution.
    ///
    /// # Arguments
    ///
    /// - `samples`: The samples to summarize
    /// - `class_ids`: Slice of class IDs for each box (flattened across samples)
    #[must_use]
    pub fn from_samples_with_classes(samples: &[DatasetSample], class_ids: &[u32]) -> Self {
        let mut summary = Self::from_samples(samples);

        // Count class occurrences
        let mut class_counts = std::collections::HashMap::new();
        for &class_id in class_ids {
            *class_counts.entry(class_id).or_insert(0) += 1;
        }

        let mut distribution: Vec<_> = class_counts.into_iter().collect();
        distribution.sort_by_key(|(id, _)| *id);
        summary.class_distribution = distribution;

        summary
    }

    /// Returns true if the dataset is empty.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.total_samples == 0
    }

    /// Returns true if the dataset has any positive samples.
    #[must_use]
    pub const fn has_positives(&self) -> bool {
        self.positive_samples > 0
    }

    /// Returns true if the dataset has any negative samples.
    #[must_use]
    pub const fn has_negatives(&self) -> bool {
        self.negative_samples > 0
    }

    /// Checks if the dataset is balanced (within tolerance).
    ///
    /// A dataset is considered balanced if the positive ratio
    /// is between 0.5 - tolerance and 0.5 + tolerance.
    #[must_use]
    pub fn is_balanced(&self, tolerance: f32) -> bool {
        (self.positive_ratio - 0.5).abs() <= tolerance
    }

    /// Returns a human-readable summary string.
    #[must_use]
    #[allow(clippy::let_underscore_must_use)] // String::write_fmt is infallible
    pub fn to_report(&self) -> String {
        use std::fmt::Write;

        let mut report = String::new();
        let _ = writeln!(report, "Dataset Summary");
        let _ = writeln!(report, "===============");
        let _ = writeln!(report, "Total samples: {}", self.total_samples);
        let _ = writeln!(
            report,
            "Positive samples: {} ({:.1}%)",
            self.positive_samples,
            self.positive_ratio * 100.0
        );
        let _ = writeln!(
            report,
            "Negative samples: {} ({:.1}%)",
            self.negative_samples,
            (1.0 - self.positive_ratio) * 100.0
        );
        let _ = writeln!(report, "Total boxes: {}", self.total_boxes);
        let _ = writeln!(
            report,
            "Avg boxes/positive: {:.2}",
            self.avg_boxes_per_positive
        );

        if !self.class_distribution.is_empty() {
            let _ = writeln!(report, "\nClass Distribution:");
            for (class_id, count) in &self.class_distribution {
                let _ = writeln!(report, "  Class {class_id}: {count}");
            }
        }

        report
    }

    /// Merges two summaries together.
    ///
    /// Useful for combining train/val statistics.
    #[must_use]
    pub fn merge(&self, other: &Self) -> Self {
        let total_samples = self.total_samples + other.total_samples;
        let positive_samples = self.positive_samples + other.positive_samples;
        let negative_samples = self.negative_samples + other.negative_samples;
        let total_boxes = self.total_boxes + other.total_boxes;

        #[allow(clippy::cast_precision_loss)]
        let avg_boxes_per_positive = if positive_samples > 0 {
            total_boxes as f32 / positive_samples as f32
        } else {
            0.0
        };

        #[allow(clippy::cast_precision_loss)]
        let positive_ratio = if total_samples > 0 {
            positive_samples as f32 / total_samples as f32
        } else {
            0.0
        };

        // Merge histograms
        let max_len = self
            .box_count_histogram
            .len()
            .max(other.box_count_histogram.len());
        let mut box_count_histogram = vec![0usize; max_len];
        for (i, &count) in self.box_count_histogram.iter().enumerate() {
            box_count_histogram[i] += count;
        }
        for (i, &count) in other.box_count_histogram.iter().enumerate() {
            box_count_histogram[i] += count;
        }

        // Merge class distributions
        let mut class_counts = std::collections::HashMap::new();
        for &(class_id, count) in &self.class_distribution {
            *class_counts.entry(class_id).or_insert(0) += count;
        }
        for &(class_id, count) in &other.class_distribution {
            *class_counts.entry(class_id).or_insert(0) += count;
        }
        let mut class_distribution: Vec<_> = class_counts.into_iter().collect();
        class_distribution.sort_by_key(|(id, _)| *id);

        Self {
            total_samples,
            positive_samples,
            negative_samples,
            total_boxes,
            avg_boxes_per_positive,
            positive_ratio,
            box_count_histogram,
            class_distribution,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_sample(id: u64, box_count: usize) -> DatasetSample {
        let mut sample = DatasetSample::empty(id);
        for i in 0..box_count {
            let offset = i as f32 * 0.1;
            sample
                .boxes
                .push([offset, offset, offset + 0.1, offset + 0.1]);
        }
        sample
    }

    #[test]
    fn summary_empty() {
        let samples: Vec<DatasetSample> = vec![];
        let summary = DatasetSummary::from_samples(&samples);

        assert!(summary.is_empty());
        assert_eq!(summary.total_samples, 0);
        assert!(!summary.has_positives());
        assert!(!summary.has_negatives());
    }

    #[test]
    fn summary_all_negative() {
        let samples: Vec<DatasetSample> = (0..10).map(DatasetSample::empty).collect();
        let summary = DatasetSummary::from_samples(&samples);

        assert_eq!(summary.total_samples, 10);
        assert_eq!(summary.positive_samples, 0);
        assert_eq!(summary.negative_samples, 10);
        assert!((summary.positive_ratio).abs() < 1e-6);
        assert!(!summary.has_positives());
        assert!(summary.has_negatives());
    }

    #[test]
    fn summary_all_positive() {
        let samples: Vec<DatasetSample> = (0..10).map(|i| make_sample(i, 2)).collect();
        let summary = DatasetSummary::from_samples(&samples);

        assert_eq!(summary.total_samples, 10);
        assert_eq!(summary.positive_samples, 10);
        assert_eq!(summary.negative_samples, 0);
        assert_eq!(summary.total_boxes, 20);
        assert!((summary.avg_boxes_per_positive - 2.0).abs() < 1e-6);
        assert!((summary.positive_ratio - 1.0).abs() < 1e-6);
    }

    #[test]
    fn summary_mixed() {
        let mut samples: Vec<DatasetSample> = (0..10).map(DatasetSample::empty).collect();
        samples[0] = make_sample(0, 3);
        samples[1] = make_sample(1, 2);

        let summary = DatasetSummary::from_samples(&samples);

        assert_eq!(summary.total_samples, 10);
        assert_eq!(summary.positive_samples, 2);
        assert_eq!(summary.negative_samples, 8);
        assert_eq!(summary.total_boxes, 5);
        assert!((summary.avg_boxes_per_positive - 2.5).abs() < 1e-6);
        assert!((summary.positive_ratio - 0.2).abs() < 1e-6);
    }

    #[test]
    fn summary_histogram() {
        let samples = vec![
            make_sample(0, 0),
            make_sample(1, 0),
            make_sample(2, 1),
            make_sample(3, 2),
            make_sample(4, 2),
            make_sample(5, 3),
        ];

        let summary = DatasetSummary::from_samples(&samples);

        assert_eq!(summary.box_count_histogram.len(), 4);
        assert_eq!(summary.box_count_histogram[0], 2); // 2 with 0 boxes
        assert_eq!(summary.box_count_histogram[1], 1); // 1 with 1 box
        assert_eq!(summary.box_count_histogram[2], 2); // 2 with 2 boxes
        assert_eq!(summary.box_count_histogram[3], 1); // 1 with 3 boxes
    }

    #[test]
    fn summary_with_classes() {
        let samples = vec![make_sample(0, 2), make_sample(1, 3)];
        let class_ids = vec![0, 1, 0, 0, 2]; // 5 boxes total

        let summary = DatasetSummary::from_samples_with_classes(&samples, &class_ids);

        assert_eq!(summary.class_distribution.len(), 3);
        assert_eq!(summary.class_distribution[0], (0, 3)); // class 0 appears 3 times
        assert_eq!(summary.class_distribution[1], (1, 1)); // class 1 appears 1 time
        assert_eq!(summary.class_distribution[2], (2, 1)); // class 2 appears 1 time
    }

    #[test]
    fn summary_is_balanced() {
        let mut samples: Vec<DatasetSample> = (0..10).map(DatasetSample::empty).collect();
        for i in 0..5 {
            samples[i] = make_sample(i as u64, 1);
        }

        let summary = DatasetSummary::from_samples(&samples);

        assert!(summary.is_balanced(0.1)); // 50% is balanced
        assert!(summary.is_balanced(0.0)); // exactly balanced
    }

    #[test]
    fn summary_not_balanced() {
        let mut samples: Vec<DatasetSample> = (0..10).map(DatasetSample::empty).collect();
        samples[0] = make_sample(0, 1); // Only 10% positive

        let summary = DatasetSummary::from_samples(&samples);

        assert!(!summary.is_balanced(0.1)); // 10% not balanced with 10% tolerance
        assert!(summary.is_balanced(0.5)); // but 50% tolerance allows it
    }

    #[test]
    fn summary_merge() {
        let samples1 = vec![make_sample(0, 2), make_sample(1, 0)];
        let samples2 = vec![make_sample(2, 1), make_sample(3, 3)];

        let summary1 = DatasetSummary::from_samples(&samples1);
        let summary2 = DatasetSummary::from_samples(&samples2);
        let merged = summary1.merge(&summary2);

        assert_eq!(merged.total_samples, 4);
        assert_eq!(merged.positive_samples, 3);
        assert_eq!(merged.negative_samples, 1);
        assert_eq!(merged.total_boxes, 6);
    }

    #[test]
    fn summary_to_report() {
        let samples = vec![make_sample(0, 2), make_sample(1, 0)];
        let summary = DatasetSummary::from_samples(&samples);
        let report = summary.to_report();

        assert!(report.contains("Total samples: 2"));
        assert!(report.contains("Positive samples: 1"));
        assert!(report.contains("Total boxes: 2"));
    }

    #[test]
    fn summary_serialization() {
        let samples = vec![make_sample(0, 2), make_sample(1, 1)];
        let summary = DatasetSummary::from_samples(&samples);

        let json = serde_json::to_string(&summary);
        assert!(json.is_ok());

        let parsed: std::result::Result<DatasetSummary, _> =
            serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
        assert_eq!(parsed.unwrap_or_default(), summary);
    }

    #[test]
    fn summary_default() {
        let summary = DatasetSummary::default();
        assert!(summary.is_empty());
        assert_eq!(summary.total_samples, 0);
    }
}
