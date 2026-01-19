//! Multi-box detection model with spatial output heads.

use burn::module::Module;
use burn::nn;
use burn::prelude::Backend;
use burn::tensor::Tensor;
use burn::tensor::activation::{relu, sigmoid};
use serde::{Deserialize, Serialize};

/// Configuration for the multi-box detection model.
///
/// # Example
///
/// ```
/// use ml_models::MultiboxModelConfig;
///
/// let config = MultiboxModelConfig::default();
/// assert_eq!(config.max_boxes, 64);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct MultiboxModelConfig {
    /// Number of hidden units per layer.
    pub hidden: usize,

    /// Number of hidden layers (depth of the network).
    pub depth: usize,

    /// Maximum number of predicted boxes.
    pub max_boxes: usize,

    /// Input dimension (default: 4 for bbox coordinates, can be larger for features).
    pub input_dim: usize,
}

impl Default for MultiboxModelConfig {
    fn default() -> Self {
        Self {
            hidden: 128,
            depth: 2,
            max_boxes: 64,
            input_dim: 4,
        }
    }
}

impl MultiboxModelConfig {
    /// Creates a new configuration.
    #[must_use]
    pub const fn new(hidden: usize, depth: usize, max_boxes: usize) -> Self {
        Self {
            hidden,
            depth,
            max_boxes,
            input_dim: 4,
        }
    }

    /// Sets the input dimension.
    #[must_use]
    pub const fn with_input_dim(mut self, input_dim: usize) -> Self {
        self.input_dim = input_dim;
        self
    }

    /// Validates the configuration.
    #[must_use]
    pub const fn is_valid(&self) -> bool {
        self.hidden > 0 && self.max_boxes > 0 && self.input_dim > 0
    }
}

/// Multi-box detection model for object localization.
///
/// Predicts multiple bounding boxes and their confidence scores.
/// Architecture: Stem -> N Hidden Blocks -> Box Head + Score Head
///
/// # Output Format
///
/// - Boxes: `[batch, max_boxes, 4]` with coordinates `[x0, y0, x1, y1]` in `[0, 1]`
/// - Scores: `[batch, max_boxes]` with confidence in `[0, 1]`
///
/// # Type Parameters
///
/// - `B`: The Burn backend (e.g., `NdArray`, `Wgpu`)
///
/// # Example
///
/// ```ignore
/// use ml_models::{MultiboxModel, MultiboxModelConfig};
///
/// let config = MultiboxModelConfig::default();
/// let device = Default::default();
/// let model = MultiboxModel::<MyBackend>::new(config, &device);
///
/// let input = Tensor::zeros([1, 4], &device);
/// let (boxes, scores) = model.forward_multibox(input);
/// ```
#[derive(Debug, Module)]
pub struct MultiboxModel<B: Backend> {
    stem: nn::Linear<B>,
    blocks: Vec<nn::Linear<B>>,
    box_head: nn::Linear<B>,
    score_head: nn::Linear<B>,
    /// Maximum boxes (stored separately for checkpoint compatibility).
    #[module(skip)]
    max_boxes: usize,
}

impl<B: Backend> MultiboxModel<B> {
    /// Creates a new multi-box model.
    ///
    /// # Arguments
    ///
    /// - `config`: Model configuration
    /// - `device`: The device to create the model on
    #[must_use]
    pub fn new(config: MultiboxModelConfig, device: &B::Device) -> Self {
        let max_boxes = config.max_boxes.max(1);

        let stem = nn::LinearConfig::new(config.input_dim, config.hidden).init(device);

        let mut blocks = Vec::with_capacity(config.depth);
        for _ in 0..config.depth {
            blocks.push(nn::LinearConfig::new(config.hidden, config.hidden).init(device));
        }

        let box_head = nn::LinearConfig::new(config.hidden, max_boxes * 4).init(device);
        let score_head = nn::LinearConfig::new(config.hidden, max_boxes).init(device);

        Self {
            stem,
            blocks,
            box_head,
            score_head,
            max_boxes,
        }
    }

    /// Runs the classification-only forward pass.
    ///
    /// Returns only the score logits (useful for training classification loss).
    ///
    /// # Arguments
    ///
    /// - `input`: Input tensor of shape `[batch_size, input_dim]`
    ///
    /// # Returns
    ///
    /// Score logits of shape `[batch_size, max_boxes]`
    pub fn forward(&self, input: Tensor<B, 2>) -> Tensor<B, 2> {
        let mut x = relu(self.stem.forward(input));
        for block in &self.blocks {
            x = relu(block.forward(x));
        }
        self.score_head.forward(x)
    }

    /// Runs the full multi-box forward pass.
    ///
    /// Returns both bounding boxes and confidence scores.
    ///
    /// # Arguments
    ///
    /// - `input`: Input tensor of shape `[batch_size, input_dim]`
    ///
    /// # Returns
    ///
    /// Tuple of:
    /// - Boxes: `[batch_size, max_boxes, 4]` with `[x0, y0, x1, y1]` in `[0, 1]`
    /// - Scores: `[batch_size, max_boxes]` with confidence in `[0, 1]`
    ///
    /// Box coordinates are guaranteed to satisfy `x0 <= x1` and `y0 <= y1`.
    #[allow(clippy::similar_names)]
    pub fn forward_multibox(&self, input: Tensor<B, 2>) -> (Tensor<B, 3>, Tensor<B, 2>) {
        let mut x = relu(self.stem.forward(input));
        for block in &self.blocks {
            x = relu(block.forward(x));
        }

        // Get raw predictions
        let boxes_flat = sigmoid(self.box_head.forward(x.clone()));
        let scores = sigmoid(self.score_head.forward(x));

        // Reshape boxes to [batch, max_boxes, 4]
        let batch = boxes_flat.dims()[0];
        let boxes = boxes_flat.reshape([batch, self.max_boxes, 4]);

        // Reorder coordinates to enforce x0 <= x1, y0 <= y1
        let boxes_ordered = self.order_box_coordinates(boxes, batch);

        (boxes_ordered, scores)
    }

    /// Reorders box coordinates to ensure `x0 <= x1` and `y0 <= y1`.
    ///
    /// Uses arithmetic operations to swap coordinates without branching.
    #[allow(clippy::similar_names)]
    fn order_box_coordinates(&self, boxes: Tensor<B, 3>, batch: usize) -> Tensor<B, 3> {
        let max_boxes = self.max_boxes;

        // Extract individual coordinates
        let x0 = boxes.clone().slice([0..batch, 0..max_boxes, 0..1]);
        let y0 = boxes.clone().slice([0..batch, 0..max_boxes, 1..2]);
        let x1 = boxes.clone().slice([0..batch, 0..max_boxes, 2..3]);
        let y1 = boxes.slice([0..batch, 0..max_boxes, 3..4]);

        // Compute differences
        let dx = x0.clone() - x1.clone();
        let dy = y0.clone() - y1.clone();
        let half = 0.5;

        // Arithmetic reordering: min = (a + b - |a - b|) / 2, max = (a + b + |a - b|) / 2
        let x_min = (x0.clone() + x1.clone() - dx.clone().abs()) * half;
        let x_max = (x0 + x1 + dx.abs()) * half;
        let y_min = (y0.clone() + y1.clone() - dy.clone().abs()) * half;
        let y_max = (y0 + y1 + dy.abs()) * half;

        // Clamp to [0, 1]
        let x_min = x_min.clamp(0.0, 1.0);
        let x_max = x_max.clamp(0.0, 1.0);
        let y_min = y_min.clamp(0.0, 1.0);
        let y_max = y_max.clamp(0.0, 1.0);

        // Concatenate back to [batch, max_boxes, 4]
        Tensor::cat(vec![x_min, y_min, x_max, y_max], 2)
    }

    /// Returns the maximum number of boxes.
    #[must_use]
    pub const fn max_boxes(&self) -> usize {
        self.max_boxes
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use burn_ndarray::NdArray;

    type TestBackend = NdArray<f32>;

    #[test]
    fn config_default() {
        let config = MultiboxModelConfig::default();
        assert_eq!(config.hidden, 128);
        assert_eq!(config.depth, 2);
        assert_eq!(config.max_boxes, 64);
        assert_eq!(config.input_dim, 4);
        assert!(config.is_valid());
    }

    #[test]
    fn config_new() {
        let config = MultiboxModelConfig::new(256, 3, 32);
        assert_eq!(config.hidden, 256);
        assert_eq!(config.depth, 3);
        assert_eq!(config.max_boxes, 32);
    }

    #[test]
    fn config_with_input_dim() {
        let config = MultiboxModelConfig::default().with_input_dim(12);
        assert_eq!(config.input_dim, 12);
    }

    #[test]
    fn config_serialization() {
        let config = MultiboxModelConfig::default();
        let json = serde_json::to_string(&config);
        assert!(json.is_ok());

        let parsed: Result<MultiboxModelConfig, _> =
            serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
        assert_eq!(parsed.unwrap_or_default(), config);
    }

    #[test]
    fn model_forward() {
        let config = MultiboxModelConfig::new(32, 1, 8);
        let device = <TestBackend as Backend>::Device::default();
        let model = MultiboxModel::<TestBackend>::new(config, &device);

        let input = Tensor::<TestBackend, 2>::zeros([2, 4], &device);
        let scores = model.forward(input);

        assert_eq!(scores.dims(), [2, 8]);
    }

    #[test]
    fn model_forward_multibox() {
        let config = MultiboxModelConfig::new(32, 1, 8);
        let device = <TestBackend as Backend>::Device::default();
        let model = MultiboxModel::<TestBackend>::new(config, &device);

        let input = Tensor::<TestBackend, 2>::zeros([2, 4], &device);
        let (boxes, scores) = model.forward_multibox(input);

        assert_eq!(boxes.dims(), [2, 8, 4]);
        assert_eq!(scores.dims(), [2, 8]);
    }

    #[test]
    fn model_box_coordinates_ordered() {
        let config = MultiboxModelConfig::new(32, 1, 4);
        let device = <TestBackend as Backend>::Device::default();
        let model = MultiboxModel::<TestBackend>::new(config, &device);

        // Create random input (will produce boxes)
        let input = Tensor::<TestBackend, 2>::ones([1, 4], &device);
        let (boxes, _) = model.forward_multibox(input);

        // Extract and verify coordinates are ordered
        let boxes_data = boxes.into_data();
        let values = boxes_data.to_vec::<f32>().unwrap_or_default();

        // For each box, verify x0 <= x1 and y0 <= y1
        for box_idx in 0..4 {
            let x0 = values[box_idx * 4];
            let y0 = values[box_idx * 4 + 1];
            let x1 = values[box_idx * 4 + 2];
            let y1 = values[box_idx * 4 + 3];

            assert!(x0 <= x1, "Box {box_idx}: x0 ({x0}) should be <= x1 ({x1})");
            assert!(y0 <= y1, "Box {box_idx}: y0 ({y0}) should be <= y1 ({y1})");
            assert!((0.0..=1.0).contains(&x0), "x0 should be in [0, 1]");
            assert!((0.0..=1.0).contains(&y0), "y0 should be in [0, 1]");
            assert!((0.0..=1.0).contains(&x1), "x1 should be in [0, 1]");
            assert!((0.0..=1.0).contains(&y1), "y1 should be in [0, 1]");
        }
    }

    #[test]
    fn model_max_boxes_accessor() {
        let config = MultiboxModelConfig::new(64, 2, 16);
        let device = <TestBackend as Backend>::Device::default();
        let model = MultiboxModel::<TestBackend>::new(config, &device);

        assert_eq!(model.max_boxes(), 16);
    }
}
