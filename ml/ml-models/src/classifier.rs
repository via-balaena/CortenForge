//! Linear classifier model for binary classification.

use burn::module::Module;
use burn::nn;
use burn::prelude::Backend;
use burn::tensor::Tensor;
use burn::tensor::activation::relu;
use serde::{Deserialize, Serialize};

/// Configuration for the linear classifier.
///
/// # Example
///
/// ```
/// use ml_models::LinearClassifierConfig;
///
/// let config = LinearClassifierConfig::default();
/// assert_eq!(config.hidden, 64);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct LinearClassifierConfig {
    /// Number of hidden units.
    pub hidden: usize,

    /// Input dimension (default: 4 for bounding box coordinates).
    pub input_dim: usize,

    /// Output dimension (default: 1 for binary classification).
    pub output_dim: usize,
}

impl Default for LinearClassifierConfig {
    fn default() -> Self {
        Self {
            hidden: 64,
            input_dim: 4,
            output_dim: 1,
        }
    }
}

impl LinearClassifierConfig {
    /// Creates a new configuration with custom hidden size.
    #[must_use]
    pub const fn new(hidden: usize) -> Self {
        Self {
            hidden,
            input_dim: 4,
            output_dim: 1,
        }
    }

    /// Sets the input dimension.
    #[must_use]
    pub const fn with_input_dim(mut self, input_dim: usize) -> Self {
        self.input_dim = input_dim;
        self
    }

    /// Sets the output dimension.
    #[must_use]
    pub const fn with_output_dim(mut self, output_dim: usize) -> Self {
        self.output_dim = output_dim;
        self
    }

    /// Validates the configuration.
    ///
    /// Returns `true` if all dimensions are positive.
    #[must_use]
    pub const fn is_valid(&self) -> bool {
        self.hidden > 0 && self.input_dim > 0 && self.output_dim > 0
    }
}

/// A simple feedforward classifier for binary detection.
///
/// Architecture: Input -> Linear -> `ReLU` -> Linear -> Output
///
/// This model is designed for simple classification tasks where the input
/// is a fixed-size feature vector (e.g., bounding box coordinates or
/// extracted features).
///
/// # Type Parameters
///
/// - `B`: The Burn backend (e.g., `NdArray`, `Wgpu`)
///
/// # Example
///
/// ```ignore
/// use burn::tensor::backend::Backend;
/// use ml_models::{LinearClassifier, LinearClassifierConfig};
///
/// let config = LinearClassifierConfig::default();
/// let device = Default::default();
/// let model = LinearClassifier::<MyBackend>::new(config, &device);
///
/// let input = Tensor::zeros([1, 4], &device);
/// let output = model.forward(input);
/// assert_eq!(output.dims(), [1, 1]);
/// ```
#[derive(Debug, Module)]
pub struct LinearClassifier<B: Backend> {
    linear1: nn::Linear<B>,
    linear2: nn::Linear<B>,
}

impl<B: Backend> LinearClassifier<B> {
    /// Creates a new linear classifier.
    ///
    /// # Arguments
    ///
    /// - `config`: Model configuration
    /// - `device`: The device to create the model on
    #[must_use]
    pub fn new(config: LinearClassifierConfig, device: &B::Device) -> Self {
        let linear1 = nn::LinearConfig::new(config.input_dim, config.hidden).init(device);
        let linear2 = nn::LinearConfig::new(config.hidden, config.output_dim).init(device);
        Self { linear1, linear2 }
    }

    /// Runs the forward pass.
    ///
    /// # Arguments
    ///
    /// - `input`: Input tensor of shape `[batch_size, input_dim]`
    ///
    /// # Returns
    ///
    /// Output tensor of shape `[batch_size, output_dim]` (logits, not probabilities)
    pub fn forward(&self, input: Tensor<B, 2>) -> Tensor<B, 2> {
        let x = self.linear1.forward(input);
        let x = relu(x);
        self.linear2.forward(x)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use burn_ndarray::NdArray;

    type TestBackend = NdArray<f32>;

    #[test]
    fn config_default() {
        let config = LinearClassifierConfig::default();
        assert_eq!(config.hidden, 64);
        assert_eq!(config.input_dim, 4);
        assert_eq!(config.output_dim, 1);
        assert!(config.is_valid());
    }

    #[test]
    fn config_new() {
        let config = LinearClassifierConfig::new(128);
        assert_eq!(config.hidden, 128);
    }

    #[test]
    fn config_builder() {
        let config = LinearClassifierConfig::new(32)
            .with_input_dim(8)
            .with_output_dim(2);

        assert_eq!(config.hidden, 32);
        assert_eq!(config.input_dim, 8);
        assert_eq!(config.output_dim, 2);
    }

    #[test]
    fn config_serialization() {
        let config = LinearClassifierConfig::default();
        let json = serde_json::to_string(&config);
        assert!(json.is_ok());

        let parsed: Result<LinearClassifierConfig, _> =
            serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
        assert_eq!(parsed.unwrap_or_default(), config);
    }

    #[test]
    fn classifier_forward() {
        let config = LinearClassifierConfig::default();
        let device = <TestBackend as Backend>::Device::default();
        let model = LinearClassifier::<TestBackend>::new(config, &device);

        let input = Tensor::<TestBackend, 2>::zeros([2, 4], &device);
        let output = model.forward(input);

        assert_eq!(output.dims(), [2, 1]);
    }
}
