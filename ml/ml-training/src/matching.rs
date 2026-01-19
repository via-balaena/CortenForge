//! Box matching for training and evaluation.

use serde::{Deserialize, Serialize};

/// Strategy for matching predicted boxes to ground truth.
///
/// # Example
///
/// ```
/// use ml_training::MatchStrategy;
///
/// let strategy = MatchStrategy::GreedyIoU { threshold: 0.5 };
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum MatchStrategy {
    /// Greedy matching based on `IoU` threshold.
    GreedyIoU {
        /// Minimum `IoU` for a match.
        threshold: f32,
    },

    /// Hungarian algorithm for optimal matching.
    Hungarian {
        /// Minimum `IoU` for a match.
        threshold: f32,
    },

    /// Match each prediction to the highest `IoU` ground truth.
    MaxIoU {
        /// Minimum `IoU` for a match.
        threshold: f32,
    },
}

impl Default for MatchStrategy {
    fn default() -> Self {
        Self::GreedyIoU { threshold: 0.5 }
    }
}

impl MatchStrategy {
    /// Creates a greedy `IoU` strategy.
    #[must_use]
    pub const fn greedy(threshold: f32) -> Self {
        Self::GreedyIoU { threshold }
    }

    /// Creates a Hungarian matching strategy.
    #[must_use]
    pub const fn hungarian(threshold: f32) -> Self {
        Self::Hungarian { threshold }
    }

    /// Creates a max `IoU` strategy.
    #[must_use]
    pub const fn max_iou(threshold: f32) -> Self {
        Self::MaxIoU { threshold }
    }

    /// Returns the threshold for this strategy.
    #[must_use]
    pub const fn threshold(&self) -> f32 {
        match self {
            Self::GreedyIoU { threshold }
            | Self::Hungarian { threshold }
            | Self::MaxIoU { threshold } => *threshold,
        }
    }
}

/// Result of matching a single prediction to ground truth.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MatchResult {
    /// Matched to ground truth at index with given `IoU`.
    Matched {
        /// Index of matched ground truth box.
        gt_index: usize,
        /// `IoU` of the match.
        iou: f32,
    },

    /// No match found (false positive or below threshold).
    Unmatched,
}

impl MatchResult {
    /// Returns true if this is a match.
    #[must_use]
    pub const fn is_matched(&self) -> bool {
        matches!(self, Self::Matched { .. })
    }

    /// Returns the ground truth index if matched.
    #[must_use]
    pub const fn gt_index(&self) -> Option<usize> {
        match self {
            Self::Matched { gt_index, .. } => Some(*gt_index),
            Self::Unmatched => None,
        }
    }

    /// Returns the `IoU` if matched.
    #[must_use]
    pub const fn iou(&self) -> Option<f32> {
        match self {
            Self::Matched { iou, .. } => Some(*iou),
            Self::Unmatched => None,
        }
    }
}

/// Matcher for associating predictions with ground truth boxes.
///
/// # Example
///
/// ```
/// use ml_training::{BoxMatcher, MatchStrategy};
///
/// let matcher = BoxMatcher::new(MatchStrategy::greedy(0.5));
///
/// // Boxes in [x0, y0, x1, y1] format
/// let predictions = vec![[0.1, 0.1, 0.3, 0.3], [0.5, 0.5, 0.7, 0.7]];
/// let ground_truth = vec![[0.1, 0.1, 0.3, 0.3]];
///
/// let matches = matcher.match_boxes(&predictions, &ground_truth);
/// assert!(matches[0].is_matched()); // First pred matches GT
/// assert!(!matches[1].is_matched()); // Second pred is unmatched
/// ```
#[derive(Debug, Clone)]
pub struct BoxMatcher {
    strategy: MatchStrategy,
}

impl Default for BoxMatcher {
    fn default() -> Self {
        Self::new(MatchStrategy::default())
    }
}

impl BoxMatcher {
    /// Creates a new box matcher with the given strategy.
    #[must_use]
    pub const fn new(strategy: MatchStrategy) -> Self {
        Self { strategy }
    }

    /// Returns the matching strategy.
    #[must_use]
    pub const fn strategy(&self) -> &MatchStrategy {
        &self.strategy
    }

    /// Matches predictions to ground truth boxes.
    ///
    /// # Arguments
    ///
    /// - `predictions`: Predicted boxes `[[x0, y0, x1, y1], ...]`
    /// - `ground_truth`: Ground truth boxes `[[x0, y0, x1, y1], ...]`
    ///
    /// # Returns
    ///
    /// Match result for each prediction.
    #[must_use]
    pub fn match_boxes(
        &self,
        predictions: &[[f32; 4]],
        ground_truth: &[[f32; 4]],
    ) -> Vec<MatchResult> {
        if predictions.is_empty() {
            return Vec::new();
        }

        if ground_truth.is_empty() {
            return vec![MatchResult::Unmatched; predictions.len()];
        }

        match self.strategy {
            MatchStrategy::GreedyIoU { threshold } => {
                Self::greedy_match(predictions, ground_truth, threshold)
            }
            MatchStrategy::Hungarian { threshold } => {
                // Fall back to greedy for now (Hungarian requires external crate)
                Self::greedy_match(predictions, ground_truth, threshold)
            }
            MatchStrategy::MaxIoU { threshold } => {
                Self::max_iou_match(predictions, ground_truth, threshold)
            }
        }
    }

    /// Greedy matching: assign each pred to highest `IoU` GT that's still available.
    fn greedy_match(
        predictions: &[[f32; 4]],
        ground_truth: &[[f32; 4]],
        threshold: f32,
    ) -> Vec<MatchResult> {
        let mut results = vec![MatchResult::Unmatched; predictions.len()];
        let mut gt_used = vec![false; ground_truth.len()];

        // Compute all IoUs
        let mut iou_pairs: Vec<(usize, usize, f32)> = Vec::new();
        for (pred_idx, pred) in predictions.iter().enumerate() {
            for (gt_idx, gt) in ground_truth.iter().enumerate() {
                let iou = Self::compute_iou(pred, gt);
                if iou >= threshold {
                    iou_pairs.push((pred_idx, gt_idx, iou));
                }
            }
        }

        // Sort by IoU descending
        iou_pairs.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap_or(std::cmp::Ordering::Equal));

        // Greedy assignment
        for (pred_idx, gt_idx, iou) in iou_pairs {
            if !gt_used[gt_idx] && results[pred_idx] == MatchResult::Unmatched {
                results[pred_idx] = MatchResult::Matched {
                    gt_index: gt_idx,
                    iou,
                };
                gt_used[gt_idx] = true;
            }
        }

        results
    }

    /// Max `IoU` matching: each pred matches to highest `IoU` GT (may duplicate).
    fn max_iou_match(
        predictions: &[[f32; 4]],
        ground_truth: &[[f32; 4]],
        threshold: f32,
    ) -> Vec<MatchResult> {
        predictions
            .iter()
            .map(|pred| {
                let mut best_iou = 0.0_f32;
                let mut best_gt = None;

                for (gt_idx, gt) in ground_truth.iter().enumerate() {
                    let iou = Self::compute_iou(pred, gt);
                    if iou > best_iou {
                        best_iou = iou;
                        best_gt = Some(gt_idx);
                    }
                }

                if best_iou >= threshold {
                    if let Some(gt_index) = best_gt {
                        return MatchResult::Matched {
                            gt_index,
                            iou: best_iou,
                        };
                    }
                }

                MatchResult::Unmatched
            })
            .collect()
    }

    /// Computes `IoU` between two boxes.
    fn compute_iou(box1: &[f32; 4], box2: &[f32; 4]) -> f32 {
        let x0 = box1[0].max(box2[0]);
        let y0 = box1[1].max(box2[1]);
        let x1 = box1[2].min(box2[2]);
        let y1 = box1[3].min(box2[3]);

        let inter_w = (x1 - x0).max(0.0);
        let inter_h = (y1 - y0).max(0.0);
        let inter_area = inter_w * inter_h;

        let area1 = (box1[2] - box1[0]) * (box1[3] - box1[1]);
        let area2 = (box2[2] - box2[0]) * (box2[3] - box2[1]);
        let union_area = area1 + area2 - inter_area;

        if union_area > 0.0 {
            inter_area / union_area
        } else {
            0.0
        }
    }

    /// Computes precision, recall, and F1 from matches.
    ///
    /// # Arguments
    ///
    /// - `matches`: Match results from `match_boxes`
    /// - `num_ground_truth`: Total number of ground truth boxes
    ///
    /// # Returns
    ///
    /// Tuple of (precision, recall, f1).
    #[must_use]
    pub fn compute_metrics(matches: &[MatchResult], num_ground_truth: usize) -> (f32, f32, f32) {
        if matches.is_empty() && num_ground_truth == 0 {
            return (1.0, 1.0, 1.0); // Perfect for empty case
        }

        let true_positives = matches.iter().filter(|m| m.is_matched()).count();

        #[allow(clippy::cast_precision_loss)]
        let precision = if matches.is_empty() {
            0.0
        } else {
            true_positives as f32 / matches.len() as f32
        };

        #[allow(clippy::cast_precision_loss)]
        let recall = if num_ground_truth == 0 {
            0.0
        } else {
            true_positives as f32 / num_ground_truth as f32
        };

        let f1 = if precision + recall > 0.0 {
            2.0 * precision * recall / (precision + recall)
        } else {
            0.0
        };

        (precision, recall, f1)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn match_strategy_default() {
        let strategy = MatchStrategy::default();
        assert_eq!(strategy.threshold(), 0.5);
    }

    #[test]
    fn match_strategy_greedy() {
        let strategy = MatchStrategy::greedy(0.7);
        assert_eq!(strategy.threshold(), 0.7);
    }

    #[test]
    fn match_strategy_hungarian() {
        let strategy = MatchStrategy::hungarian(0.6);
        assert_eq!(strategy.threshold(), 0.6);
    }

    #[test]
    fn match_strategy_max_iou() {
        let strategy = MatchStrategy::max_iou(0.4);
        assert_eq!(strategy.threshold(), 0.4);
    }

    #[test]
    fn match_result_matched() {
        let result = MatchResult::Matched {
            gt_index: 0,
            iou: 0.8,
        };
        assert!(result.is_matched());
        assert_eq!(result.gt_index(), Some(0));
        assert_eq!(result.iou(), Some(0.8));
    }

    #[test]
    fn match_result_unmatched() {
        let result = MatchResult::Unmatched;
        assert!(!result.is_matched());
        assert_eq!(result.gt_index(), None);
        assert_eq!(result.iou(), None);
    }

    #[test]
    fn box_matcher_empty_predictions() {
        let matcher = BoxMatcher::default();
        let predictions: Vec<[f32; 4]> = vec![];
        let ground_truth = vec![[0.0, 0.0, 1.0, 1.0]];

        let matches = matcher.match_boxes(&predictions, &ground_truth);
        assert!(matches.is_empty());
    }

    #[test]
    fn box_matcher_empty_ground_truth() {
        let matcher = BoxMatcher::default();
        let predictions = vec![[0.0, 0.0, 1.0, 1.0]];
        let ground_truth: Vec<[f32; 4]> = vec![];

        let matches = matcher.match_boxes(&predictions, &ground_truth);
        assert_eq!(matches.len(), 1);
        assert!(!matches[0].is_matched());
    }

    #[test]
    fn box_matcher_perfect_match() {
        let matcher = BoxMatcher::new(MatchStrategy::greedy(0.5));
        let predictions = vec![[0.0, 0.0, 1.0, 1.0]];
        let ground_truth = vec![[0.0, 0.0, 1.0, 1.0]];

        let matches = matcher.match_boxes(&predictions, &ground_truth);
        assert!(matches[0].is_matched());
        assert!((matches[0].iou().unwrap_or_default() - 1.0).abs() < 1e-5);
    }

    #[test]
    fn box_matcher_partial_overlap() {
        let matcher = BoxMatcher::new(MatchStrategy::greedy(0.3));
        // 50% overlap
        let predictions = vec![[0.0, 0.0, 1.0, 1.0]];
        let ground_truth = vec![[0.5, 0.0, 1.5, 1.0]];

        let matches = matcher.match_boxes(&predictions, &ground_truth);
        assert!(matches[0].is_matched());
        // IoU = 0.5 / 1.5 ≈ 0.333
        assert!(matches[0].iou().unwrap_or_default() > 0.3);
    }

    #[test]
    fn box_matcher_below_threshold() {
        let matcher = BoxMatcher::new(MatchStrategy::greedy(0.9));
        // 50% overlap, but threshold is 0.9
        let predictions = vec![[0.0, 0.0, 1.0, 1.0]];
        let ground_truth = vec![[0.5, 0.0, 1.5, 1.0]];

        let matches = matcher.match_boxes(&predictions, &ground_truth);
        assert!(!matches[0].is_matched());
    }

    #[test]
    fn box_matcher_greedy_best_first() {
        let matcher = BoxMatcher::new(MatchStrategy::greedy(0.5));

        // Two predictions, one GT
        // First pred has lower IoU, second has higher
        let predictions = vec![[0.0, 0.0, 0.5, 0.5], [0.0, 0.0, 1.0, 1.0]];
        let ground_truth = vec![[0.0, 0.0, 1.0, 1.0]];

        let matches = matcher.match_boxes(&predictions, &ground_truth);

        // Second pred should get the match (higher IoU)
        assert!(!matches[0].is_matched());
        assert!(matches[1].is_matched());
    }

    #[test]
    fn box_matcher_max_iou_allows_duplicates() {
        let matcher = BoxMatcher::new(MatchStrategy::max_iou(0.2));

        // Two predictions, one GT, both should match to same GT
        let predictions = vec![[0.0, 0.0, 0.8, 0.8], [0.0, 0.0, 1.0, 1.0]];
        let ground_truth = vec![[0.0, 0.0, 1.0, 1.0]];

        let matches = matcher.match_boxes(&predictions, &ground_truth);

        assert!(matches[0].is_matched());
        assert!(matches[1].is_matched());
        assert_eq!(matches[0].gt_index(), Some(0));
        assert_eq!(matches[1].gt_index(), Some(0));
    }

    #[test]
    fn compute_metrics_all_matched() {
        let matches = vec![
            MatchResult::Matched {
                gt_index: 0,
                iou: 0.9,
            },
            MatchResult::Matched {
                gt_index: 1,
                iou: 0.8,
            },
        ];

        let (precision, recall, f1) = BoxMatcher::compute_metrics(&matches, 2);
        assert!((precision - 1.0).abs() < 1e-6);
        assert!((recall - 1.0).abs() < 1e-6);
        assert!((f1 - 1.0).abs() < 1e-6);
    }

    #[test]
    fn compute_metrics_none_matched() {
        let matches = vec![MatchResult::Unmatched, MatchResult::Unmatched];

        let (precision, recall, f1) = BoxMatcher::compute_metrics(&matches, 2);
        assert!(precision.abs() < 1e-6);
        assert!(recall.abs() < 1e-6);
        assert!(f1.abs() < 1e-6);
    }

    #[test]
    fn compute_metrics_partial() {
        let matches = vec![
            MatchResult::Matched {
                gt_index: 0,
                iou: 0.8,
            },
            MatchResult::Unmatched,
        ];

        let (precision, recall, f1) = BoxMatcher::compute_metrics(&matches, 2);
        assert!((precision - 0.5).abs() < 1e-6); // 1/2 predictions matched
        assert!((recall - 0.5).abs() < 1e-6); // 1/2 GT matched
        assert!((f1 - 0.5).abs() < 1e-6);
    }

    #[test]
    fn compute_iou_identical() {
        let iou = BoxMatcher::compute_iou(&[0.0, 0.0, 1.0, 1.0], &[0.0, 0.0, 1.0, 1.0]);
        assert!((iou - 1.0).abs() < 1e-6);
    }

    #[test]
    fn compute_iou_no_overlap() {
        let iou = BoxMatcher::compute_iou(&[0.0, 0.0, 0.5, 0.5], &[0.6, 0.6, 1.0, 1.0]);
        assert!(iou.abs() < 1e-6);
    }

    #[test]
    fn match_strategy_serialization() {
        let strategy = MatchStrategy::greedy(0.5);
        let json = serde_json::to_string(&strategy);
        assert!(json.is_ok());

        let parsed: std::result::Result<MatchStrategy, _> =
            serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
        assert_eq!(parsed.unwrap_or_default(), strategy);
    }
}
