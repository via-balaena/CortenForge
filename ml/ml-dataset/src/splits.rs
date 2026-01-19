//! Dataset splitting utilities.

use rand::SeedableRng;
use rand::seq::SliceRandom;
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};

use crate::sample::DatasetSample;

/// Ratio for splitting datasets into train/validation sets.
///
/// The ratio specifies the proportion of data to use for training.
/// The remainder goes to validation.
///
/// # Example
///
/// ```
/// use ml_dataset::SplitRatio;
///
/// // 80% train, 20% validation
/// let ratio = SplitRatio::new(0.8);
/// assert!((ratio.train_ratio() - 0.8).abs() < 1e-6);
/// assert!((ratio.val_ratio() - 0.2).abs() < 1e-6);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SplitRatio {
    train: f32,
}

impl SplitRatio {
    /// Creates a new split ratio.
    ///
    /// # Arguments
    ///
    /// - `train`: Proportion for training (must be in `(0, 1)`)
    ///
    /// # Panics
    ///
    /// Panics if ratio is not in `(0, 1)`.
    #[must_use]
    pub fn new(train: f32) -> Self {
        assert!(
            train > 0.0 && train < 1.0,
            "Split ratio must be in (0, 1), got {train}"
        );
        Self { train }
    }

    /// Creates a split ratio, returning `None` if invalid.
    #[must_use]
    pub fn try_new(train: f32) -> Option<Self> {
        if train > 0.0 && train < 1.0 {
            Some(Self { train })
        } else {
            None
        }
    }

    /// Returns the training ratio.
    #[must_use]
    pub const fn train_ratio(&self) -> f32 {
        self.train
    }

    /// Returns the validation ratio.
    #[must_use]
    pub fn val_ratio(&self) -> f32 {
        1.0 - self.train
    }

    /// Computes the split point for a given dataset size.
    #[must_use]
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_precision_loss
    )]
    pub fn split_point(&self, total: usize) -> usize {
        (total as f32 * self.train).round() as usize
    }

    /// Common 80/20 split.
    pub const EIGHTY_TWENTY: Self = Self { train: 0.8 };

    /// Common 70/30 split.
    pub const SEVENTY_THIRTY: Self = Self { train: 0.7 };

    /// Common 90/10 split.
    pub const NINETY_TEN: Self = Self { train: 0.9 };
}

impl Default for SplitRatio {
    fn default() -> Self {
        Self::EIGHTY_TWENTY
    }
}

/// Splits a dataset into training and validation sets.
///
/// # Arguments
///
/// - `samples`: The samples to split
/// - `ratio`: Train/val ratio
/// - `seed`: Optional random seed for reproducibility
///
/// # Returns
///
/// Tuple of `(train, val)` sample vectors.
///
/// # Example
///
/// ```
/// use ml_dataset::{DatasetSample, split_dataset, SplitRatio};
///
/// let samples: Vec<DatasetSample> = (0..10)
///     .map(|i| DatasetSample::empty(i))
///     .collect();
///
/// let (train, val) = split_dataset(&samples, SplitRatio::EIGHTY_TWENTY, Some(42));
/// assert_eq!(train.len(), 8);
/// assert_eq!(val.len(), 2);
/// ```
#[must_use]
pub fn split_dataset(
    samples: &[DatasetSample],
    ratio: SplitRatio,
    seed: Option<u64>,
) -> (Vec<DatasetSample>, Vec<DatasetSample>) {
    if samples.is_empty() {
        return (Vec::new(), Vec::new());
    }

    // Create shuffled indices
    let mut indices: Vec<usize> = (0..samples.len()).collect();

    let mut rng = seed.map_or_else(ChaCha8Rng::from_entropy, ChaCha8Rng::seed_from_u64);
    indices.shuffle(&mut rng);

    // Split at the computed point
    let split = ratio
        .split_point(samples.len())
        .max(1)
        .min(samples.len() - 1);

    let train_indices = &indices[..split];
    let val_indices = &indices[split..];

    let train = train_indices.iter().map(|&i| samples[i].clone()).collect();
    let val = val_indices.iter().map(|&i| samples[i].clone()).collect();

    (train, val)
}

/// Splits a dataset with stratification by positive/negative samples.
///
/// Ensures both train and val sets have similar proportions of
/// samples with/without bounding boxes.
///
/// # Arguments
///
/// - `samples`: The samples to split
/// - `ratio`: Train/val ratio
/// - `seed`: Optional random seed for reproducibility
///
/// # Returns
///
/// Tuple of `(train, val)` sample vectors.
///
/// # Example
///
/// ```
/// use ml_dataset::{DatasetSample, split_stratified, SplitRatio};
///
/// let mut samples: Vec<DatasetSample> = (0..10)
///     .map(|i| DatasetSample::empty(i))
///     .collect();
///
/// // Make some samples "positive" (have boxes)
/// samples[0].boxes = vec![[0.1, 0.2, 0.3, 0.4]];
/// samples[1].boxes = vec![[0.1, 0.2, 0.3, 0.4]];
///
/// let (train, val) = split_stratified(&samples, SplitRatio::EIGHTY_TWENTY, Some(42));
/// assert_eq!(train.len() + val.len(), 10);
/// ```
#[must_use]
pub fn split_stratified(
    samples: &[DatasetSample],
    ratio: SplitRatio,
    seed: Option<u64>,
) -> (Vec<DatasetSample>, Vec<DatasetSample>) {
    if samples.is_empty() {
        return (Vec::new(), Vec::new());
    }

    // Separate positive and negative samples
    let (positives, negatives): (Vec<_>, Vec<_>) = samples.iter().partition(|s| s.has_boxes());

    // Split each group
    let (pos_train, pos_val) = split_dataset(
        &positives.iter().map(|&s| s.clone()).collect::<Vec<_>>(),
        ratio,
        seed,
    );
    let neg_seed = seed.map(|s| s.wrapping_add(1));
    let (neg_train, neg_val) = split_dataset(
        &negatives.iter().map(|&s| s.clone()).collect::<Vec<_>>(),
        ratio,
        neg_seed,
    );

    // Combine
    let mut train = pos_train;
    train.extend(neg_train);

    let mut val = pos_val;
    val.extend(neg_val);

    // Shuffle the combined sets
    let mut rng = seed.map_or_else(ChaCha8Rng::from_entropy, |s| {
        ChaCha8Rng::seed_from_u64(s.wrapping_add(2))
    });
    train.shuffle(&mut rng);
    val.shuffle(&mut rng);

    (train, val)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn split_ratio_new() {
        let ratio = SplitRatio::new(0.8);
        assert!((ratio.train_ratio() - 0.8).abs() < 1e-6);
        assert!((ratio.val_ratio() - 0.2).abs() < 1e-6);
    }

    #[test]
    fn split_ratio_try_new() {
        assert!(SplitRatio::try_new(0.5).is_some());
        assert!(SplitRatio::try_new(0.0).is_none());
        assert!(SplitRatio::try_new(1.0).is_none());
        assert!(SplitRatio::try_new(-0.5).is_none());
        assert!(SplitRatio::try_new(1.5).is_none());
    }

    #[test]
    fn split_ratio_split_point() {
        let ratio = SplitRatio::new(0.8);
        assert_eq!(ratio.split_point(100), 80);
        assert_eq!(ratio.split_point(10), 8);
    }

    #[test]
    fn split_ratio_constants() {
        assert!((SplitRatio::EIGHTY_TWENTY.train_ratio() - 0.8).abs() < 1e-6);
        assert!((SplitRatio::SEVENTY_THIRTY.train_ratio() - 0.7).abs() < 1e-6);
        assert!((SplitRatio::NINETY_TEN.train_ratio() - 0.9).abs() < 1e-6);
    }

    #[test]
    fn split_ratio_default() {
        let ratio = SplitRatio::default();
        assert!((ratio.train_ratio() - 0.8).abs() < 1e-6);
    }

    #[test]
    fn split_ratio_serialization() {
        let ratio = SplitRatio::new(0.75);
        let json = serde_json::to_string(&ratio);
        assert!(json.is_ok());

        let parsed: std::result::Result<SplitRatio, _> =
            serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
    }

    #[test]
    fn split_dataset_basic() {
        let samples: Vec<DatasetSample> = (0..10).map(DatasetSample::empty).collect();

        let (train, val) = split_dataset(&samples, SplitRatio::EIGHTY_TWENTY, Some(42));

        assert_eq!(train.len(), 8);
        assert_eq!(val.len(), 2);

        // Verify no duplicates
        let all_ids: Vec<u64> = train.iter().chain(val.iter()).map(|s| s.frame_id).collect();
        let mut sorted_ids = all_ids;
        sorted_ids.sort_unstable();
        sorted_ids.dedup();
        assert_eq!(sorted_ids.len(), 10);
    }

    #[test]
    fn split_dataset_empty() {
        let samples: Vec<DatasetSample> = vec![];
        let (train, val) = split_dataset(&samples, SplitRatio::EIGHTY_TWENTY, None);

        assert!(train.is_empty());
        assert!(val.is_empty());
    }

    #[test]
    fn split_dataset_reproducible() {
        let samples: Vec<DatasetSample> = (0..100).map(DatasetSample::empty).collect();

        let (train1, val1) = split_dataset(&samples, SplitRatio::EIGHTY_TWENTY, Some(42));
        let (train2, val2) = split_dataset(&samples, SplitRatio::EIGHTY_TWENTY, Some(42));

        assert_eq!(train1.len(), train2.len());
        for (a, b) in train1.iter().zip(train2.iter()) {
            assert_eq!(a.frame_id, b.frame_id);
        }
        for (a, b) in val1.iter().zip(val2.iter()) {
            assert_eq!(a.frame_id, b.frame_id);
        }
    }

    #[test]
    fn split_stratified_basic() {
        let mut samples: Vec<DatasetSample> = (0..100).map(DatasetSample::empty).collect();

        // Make 20% positive
        for sample in &mut samples[0..20] {
            sample.boxes = vec![[0.1, 0.2, 0.3, 0.4]];
        }

        let (train, val) = split_stratified(&samples, SplitRatio::EIGHTY_TWENTY, Some(42));

        assert_eq!(train.len() + val.len(), 100);

        // Check stratification (roughly similar proportions)
        let train_pos = train.iter().filter(|s| s.has_boxes()).count();
        let val_pos = val.iter().filter(|s| s.has_boxes()).count();

        // With 80/20 split of 20 positives, expect ~16 train and ~4 val
        assert!((14..=18).contains(&train_pos));
        assert!((2..=6).contains(&val_pos));
    }
}
