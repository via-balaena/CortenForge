//! Loss functions for object detection training.

use burn::prelude::Backend;
use burn::tensor::Tensor;
use serde::{Deserialize, Serialize};

/// Weights for combining different loss components.
///
/// # Example
///
/// ```
/// use ml_training::LossWeights;
///
/// let weights = LossWeights::default();
/// assert!((weights.box_weight - 1.0).abs() < 1e-6);
/// assert!((weights.class_weight - 1.0).abs() < 1e-6);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct LossWeights {
    /// Weight for bounding box regression loss.
    pub box_weight: f32,

    /// Weight for classification loss.
    pub class_weight: f32,

    /// Weight for objectness/confidence loss.
    pub confidence_weight: f32,
}

impl Default for LossWeights {
    fn default() -> Self {
        Self {
            box_weight: 1.0,
            class_weight: 1.0,
            confidence_weight: 1.0,
        }
    }
}

impl LossWeights {
    /// Creates new loss weights.
    #[must_use]
    pub const fn new(box_weight: f32, class_weight: f32, confidence_weight: f32) -> Self {
        Self {
            box_weight,
            class_weight,
            confidence_weight,
        }
    }

    /// Validates the weights.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.box_weight >= 0.0 && self.class_weight >= 0.0 && self.confidence_weight >= 0.0
    }
}

/// Computes smooth L1 (Huber) loss for bounding box regression.
///
/// Smooth L1 is less sensitive to outliers than L2 loss:
/// - For |x| < beta: 0.5 * x^2 / beta
/// - For |x| >= beta: |x| - 0.5 * beta
///
/// # Arguments
///
/// - `pred`: Predicted boxes `[batch, num_boxes, 4]`
/// - `target`: Target boxes `[batch, num_boxes, 4]`
/// - `beta`: Transition point (default: 1.0)
///
/// # Returns
///
/// Scalar loss value.
pub fn box_loss<B: Backend>(pred: Tensor<B, 3>, target: Tensor<B, 3>, beta: f32) -> Tensor<B, 1> {
    let diff = pred - target;
    let abs_diff = diff.clone().abs();

    // Smooth L1 components
    let quadratic = diff.powf_scalar(2.0) / (2.0 * beta);
    let linear = abs_diff.clone() - (beta / 2.0);

    // Mask for which formula to use
    let mask = abs_diff
        .clone()
        .lower(Tensor::full(abs_diff.shape(), beta, &abs_diff.device()));
    let smooth_l1 = mask.clone().float() * quadratic + (mask.bool_not()).float() * linear;

    // Mean over all dimensions
    smooth_l1.mean()
}

/// Computes binary cross-entropy loss for classification.
///
/// # Arguments
///
/// - `pred`: Predicted logits `[batch, num_predictions]`
/// - `target`: Target labels (0 or 1) `[batch, num_predictions]`
///
/// # Returns
///
/// Scalar loss value.
pub fn classification_loss<B: Backend>(pred: Tensor<B, 2>, target: Tensor<B, 2>) -> Tensor<B, 1> {
    // Apply sigmoid to get probabilities
    let prob = burn::tensor::activation::sigmoid(pred);

    // Clamp to avoid log(0)
    let eps = 1e-7;
    let prob_clamped = prob.clamp(eps, 1.0 - eps);

    // Binary cross-entropy: -[t * log(p) + (1-t) * log(1-p)]
    let log_prob = prob_clamped.clone().log();
    let log_one_minus_prob = (Tensor::ones_like(&prob_clamped) - prob_clamped).log();

    let bce = target.clone().neg() * log_prob
        - (Tensor::ones_like(&target) - target) * log_one_minus_prob;

    bce.mean()
}

/// Computes combined detection loss.
///
/// # Arguments
///
/// - `pred_boxes`: Predicted boxes `[batch, num_boxes, 4]`
/// - `target_boxes`: Target boxes `[batch, num_boxes, 4]`
/// - `pred_scores`: Predicted classification scores `[batch, num_boxes]`
/// - `target_scores`: Target classification labels `[batch, num_boxes]`
/// - `weights`: Loss component weights
///
/// # Returns
///
/// Scalar combined loss value.
pub fn detection_loss<B: Backend>(
    pred_boxes: Tensor<B, 3>,
    target_boxes: Tensor<B, 3>,
    pred_scores: Tensor<B, 2>,
    target_scores: Tensor<B, 2>,
    weights: &LossWeights,
) -> Tensor<B, 1> {
    let box_l = box_loss(pred_boxes, target_boxes, 1.0);
    let cls_l = classification_loss(pred_scores, target_scores);

    box_l * weights.box_weight + cls_l * weights.class_weight
}

/// Computes Intersection over Union (`IoU`) between two sets of boxes.
///
/// # Arguments
///
/// - `boxes1`: First set of boxes `[batch, num_boxes, 4]` in `[x0, y0, x1, y1]` format
/// - `boxes2`: Second set of boxes `[batch, num_boxes, 4]`
///
/// # Returns
///
/// `IoU` values `[batch, num_boxes]`.
#[allow(clippy::similar_names)]
pub fn compute_iou<B: Backend>(boxes1: Tensor<B, 3>, boxes2: Tensor<B, 3>) -> Tensor<B, 2> {
    let device = boxes1.device();
    let shape = boxes1.dims();
    let batch = shape[0];
    let num_boxes = shape[1];

    // Extract coordinates using narrow (avoids squeeze type issues)
    // boxes are [batch, num_boxes, 4] with [x0, y0, x1, y1]
    let x0_1: Tensor<B, 2> = boxes1
        .clone()
        .slice([0..batch, 0..num_boxes, 0..1])
        .squeeze(2);
    let y0_1: Tensor<B, 2> = boxes1
        .clone()
        .slice([0..batch, 0..num_boxes, 1..2])
        .squeeze(2);
    let x1_1: Tensor<B, 2> = boxes1
        .clone()
        .slice([0..batch, 0..num_boxes, 2..3])
        .squeeze(2);
    let y1_1: Tensor<B, 2> = boxes1.slice([0..batch, 0..num_boxes, 3..4]).squeeze(2);

    let x0_2: Tensor<B, 2> = boxes2
        .clone()
        .slice([0..batch, 0..num_boxes, 0..1])
        .squeeze(2);
    let y0_2: Tensor<B, 2> = boxes2
        .clone()
        .slice([0..batch, 0..num_boxes, 1..2])
        .squeeze(2);
    let x1_2: Tensor<B, 2> = boxes2
        .clone()
        .slice([0..batch, 0..num_boxes, 2..3])
        .squeeze(2);
    let y1_2: Tensor<B, 2> = boxes2.slice([0..batch, 0..num_boxes, 3..4]).squeeze(2);

    // Intersection coordinates
    let inter_x0 = x0_1.clone().max_pair(x0_2.clone());
    let inter_y0 = y0_1.clone().max_pair(y0_2.clone());
    let inter_x1 = x1_1.clone().min_pair(x1_2.clone());
    let inter_y1 = y1_1.clone().min_pair(y1_2.clone());

    // Intersection area (clamp to 0 for non-overlapping)
    let zero: Tensor<B, 2> = Tensor::zeros([batch, num_boxes], &device);
    let inter_w = (inter_x1 - inter_x0).max_pair(zero.clone());
    let inter_h = (inter_y1 - inter_y0).max_pair(zero);
    let inter_area = inter_w * inter_h;

    // Areas of individual boxes
    let area1 = (x1_1 - x0_1) * (y1_1 - y0_1);
    let area2 = (x1_2 - x0_2) * (y1_2 - y0_2);

    // Union area
    let union_area = area1 + area2 - inter_area.clone();

    // IoU with epsilon for numerical stability
    let eps = 1e-7;
    inter_area / (union_area + eps)
}

/// Computes `GIoU` (Generalized `IoU`) loss.
///
/// `GIoU` handles non-overlapping boxes better than standard `IoU`:
/// `GIoU` = `IoU` - (C - union) / C
/// where C is the smallest enclosing box.
///
/// # Arguments
///
/// - `pred`: Predicted boxes `[batch, num_boxes, 4]`
/// - `target`: Target boxes `[batch, num_boxes, 4]`
///
/// # Returns
///
/// Scalar `GIoU` loss (1 - `GIoU`).
#[allow(clippy::similar_names)]
pub fn giou_loss<B: Backend>(pred: Tensor<B, 3>, target: Tensor<B, 3>) -> Tensor<B, 1> {
    let device = pred.device();
    let shape = pred.dims();
    let batch = shape[0];
    let num_boxes = shape[1];

    // Extract coordinates with explicit type annotations
    let px0: Tensor<B, 2> = pred
        .clone()
        .slice([0..batch, 0..num_boxes, 0..1])
        .squeeze(2);
    let py0: Tensor<B, 2> = pred
        .clone()
        .slice([0..batch, 0..num_boxes, 1..2])
        .squeeze(2);
    let px1: Tensor<B, 2> = pred
        .clone()
        .slice([0..batch, 0..num_boxes, 2..3])
        .squeeze(2);
    let py1: Tensor<B, 2> = pred.slice([0..batch, 0..num_boxes, 3..4]).squeeze(2);

    let tx0: Tensor<B, 2> = target
        .clone()
        .slice([0..batch, 0..num_boxes, 0..1])
        .squeeze(2);
    let ty0: Tensor<B, 2> = target
        .clone()
        .slice([0..batch, 0..num_boxes, 1..2])
        .squeeze(2);
    let tx1: Tensor<B, 2> = target
        .clone()
        .slice([0..batch, 0..num_boxes, 2..3])
        .squeeze(2);
    let ty1: Tensor<B, 2> = target.slice([0..batch, 0..num_boxes, 3..4]).squeeze(2);

    // Intersection
    let inter_x0 = px0.clone().max_pair(tx0.clone());
    let inter_y0 = py0.clone().max_pair(ty0.clone());
    let inter_x1 = px1.clone().min_pair(tx1.clone());
    let inter_y1 = py1.clone().min_pair(ty1.clone());

    let zero: Tensor<B, 2> = Tensor::zeros([batch, num_boxes], &device);
    let inter_w = (inter_x1 - inter_x0).max_pair(zero.clone());
    let inter_h = (inter_y1 - inter_y0).max_pair(zero);
    let inter_area = inter_w * inter_h;

    // Areas
    let pred_area = (px1.clone() - px0.clone()) * (py1.clone() - py0.clone());
    let target_area = (tx1.clone() - tx0.clone()) * (ty1.clone() - ty0.clone());
    let union_area = pred_area + target_area - inter_area.clone();

    // IoU
    let eps = 1e-7;
    let iou = inter_area / (union_area.clone() + eps);

    // Enclosing box
    let enc_x0 = px0.min_pair(tx0);
    let enc_y0 = py0.min_pair(ty0);
    let enc_x1 = px1.max_pair(tx1);
    let enc_y1 = py1.max_pair(ty1);
    let enc_area = (enc_x1 - enc_x0) * (enc_y1 - enc_y0);

    // GIoU
    let giou = iou - (enc_area.clone() - union_area) / (enc_area + eps);

    // GIoU loss: 1 - GIoU
    let ones: Tensor<B, 2> = Tensor::ones([batch, num_boxes], &device);
    (ones - giou).mean()
}

#[cfg(test)]
mod tests {
    use super::*;
    use burn::tensor::ElementConversion;
    use burn_ndarray::NdArray;

    type TestBackend = NdArray<f32>;

    #[test]
    fn loss_weights_default() {
        let weights = LossWeights::default();
        assert!((weights.box_weight - 1.0).abs() < 1e-6);
        assert!((weights.class_weight - 1.0).abs() < 1e-6);
        assert!((weights.confidence_weight - 1.0).abs() < 1e-6);
        assert!(weights.is_valid());
    }

    #[test]
    fn loss_weights_new() {
        let weights = LossWeights::new(2.0, 0.5, 1.0);
        assert!((weights.box_weight - 2.0).abs() < 1e-6);
        assert!((weights.class_weight - 0.5).abs() < 1e-6);
    }

    #[test]
    fn loss_weights_invalid() {
        let weights = LossWeights::new(-1.0, 1.0, 1.0);
        assert!(!weights.is_valid());
    }

    #[test]
    fn box_loss_zero_for_identical() {
        let device = <TestBackend as Backend>::Device::default();
        let boxes = Tensor::<TestBackend, 3>::from_data(
            [[[0.1, 0.2, 0.3, 0.4], [0.5, 0.6, 0.7, 0.8]]],
            &device,
        );

        let loss = box_loss(boxes.clone(), boxes, 1.0);
        let loss_val: f32 = loss.into_scalar().elem();
        assert!(loss_val.abs() < 1e-6);
    }

    #[test]
    fn box_loss_positive_for_different() {
        let device = <TestBackend as Backend>::Device::default();
        let pred = Tensor::<TestBackend, 3>::from_data([[[0.1, 0.2, 0.3, 0.4]]], &device);
        let target = Tensor::<TestBackend, 3>::from_data([[[0.2, 0.3, 0.4, 0.5]]], &device);

        let loss = box_loss(pred, target, 1.0);
        let loss_val: f32 = loss.into_scalar().elem();
        assert!(loss_val > 0.0);
    }

    #[test]
    fn classification_loss_zero_for_correct() {
        let device = <TestBackend as Backend>::Device::default();
        // High positive logit for target 1
        let pred = Tensor::<TestBackend, 2>::from_data([[10.0]], &device);
        let target = Tensor::<TestBackend, 2>::from_data([[1.0]], &device);

        let loss = classification_loss(pred, target);
        let loss_val: f32 = loss.into_scalar().elem();
        assert!(loss_val < 0.01); // Should be near zero for confident correct prediction
    }

    #[test]
    fn classification_loss_high_for_wrong() {
        let device = <TestBackend as Backend>::Device::default();
        // High positive logit but target is 0
        let pred = Tensor::<TestBackend, 2>::from_data([[10.0]], &device);
        let target = Tensor::<TestBackend, 2>::from_data([[0.0]], &device);

        let loss = classification_loss(pred, target);
        let loss_val: f32 = loss.into_scalar().elem();
        assert!(loss_val > 1.0); // Should be high for wrong prediction
    }

    #[test]
    fn detection_loss_combines_correctly() {
        let device = <TestBackend as Backend>::Device::default();

        let pred_boxes = Tensor::<TestBackend, 3>::from_data([[[0.1, 0.2, 0.3, 0.4]]], &device);
        let target_boxes = Tensor::<TestBackend, 3>::from_data([[[0.1, 0.2, 0.3, 0.4]]], &device);
        let pred_scores = Tensor::<TestBackend, 2>::from_data([[10.0]], &device);
        let target_scores = Tensor::<TestBackend, 2>::from_data([[1.0]], &device);

        let weights = LossWeights::default();
        let loss = detection_loss(
            pred_boxes,
            target_boxes,
            pred_scores,
            target_scores,
            &weights,
        );

        let loss_val: f32 = loss.into_scalar().elem();
        assert!(loss_val < 0.1); // Should be low for correct predictions
    }

    #[test]
    fn compute_iou_identical_boxes() {
        let device = <TestBackend as Backend>::Device::default();
        let boxes = Tensor::<TestBackend, 3>::from_data([[[0.0, 0.0, 1.0, 1.0]]], &device);

        let iou = compute_iou(boxes.clone(), boxes);
        let iou_val: f32 = iou.into_scalar().elem();
        assert!((iou_val - 1.0).abs() < 1e-5);
    }

    #[test]
    fn compute_iou_non_overlapping() {
        let device = <TestBackend as Backend>::Device::default();
        let boxes1 = Tensor::<TestBackend, 3>::from_data([[[0.0, 0.0, 0.5, 0.5]]], &device);
        let boxes2 = Tensor::<TestBackend, 3>::from_data([[[0.6, 0.6, 1.0, 1.0]]], &device);

        let iou = compute_iou(boxes1, boxes2);
        let iou_val: f32 = iou.into_scalar().elem();
        assert!(iou_val.abs() < 1e-5);
    }

    #[test]
    fn compute_iou_partial_overlap() {
        let device = <TestBackend as Backend>::Device::default();
        // 50% overlap
        let boxes1 = Tensor::<TestBackend, 3>::from_data([[[0.0, 0.0, 1.0, 1.0]]], &device);
        let boxes2 = Tensor::<TestBackend, 3>::from_data([[[0.5, 0.0, 1.5, 1.0]]], &device);

        let iou = compute_iou(boxes1, boxes2);
        let iou_val: f32 = iou.into_scalar().elem();
        // Intersection = 0.5, Union = 1.5, IoU = 0.5/1.5 = 0.333
        assert!((iou_val - 0.333).abs() < 0.01);
    }

    #[test]
    fn giou_loss_zero_for_identical() {
        let device = <TestBackend as Backend>::Device::default();
        let boxes = Tensor::<TestBackend, 3>::from_data([[[0.0, 0.0, 1.0, 1.0]]], &device);

        let loss = giou_loss(boxes.clone(), boxes);
        let loss_val: f32 = loss.into_scalar().elem();
        assert!(loss_val.abs() < 1e-5);
    }

    #[test]
    fn giou_loss_positive_for_different() {
        let device = <TestBackend as Backend>::Device::default();
        let pred = Tensor::<TestBackend, 3>::from_data([[[0.0, 0.0, 0.5, 0.5]]], &device);
        let target = Tensor::<TestBackend, 3>::from_data([[[0.5, 0.5, 1.0, 1.0]]], &device);

        let loss = giou_loss(pred, target);
        let loss_val: f32 = loss.into_scalar().elem();
        assert!(loss_val > 0.0);
    }

    #[test]
    fn loss_weights_serialization() {
        let weights = LossWeights::new(2.0, 0.5, 1.5);
        let json = serde_json::to_string(&weights);
        assert!(json.is_ok());

        let parsed: std::result::Result<LossWeights, _> =
            serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
        assert_eq!(parsed.unwrap_or_default(), weights);
    }
}
