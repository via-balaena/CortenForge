//! Backend type definitions and device management.

use serde::{Deserialize, Serialize};

/// Supported ML backend types.
///
/// Used to specify which Burn backend to use for inference or training.
///
/// # Example
///
/// ```
/// use ml_models::BackendType;
///
/// let backend = BackendType::NdArray;
/// assert!(backend.is_cpu());
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
pub enum BackendType {
    /// CPU backend using ndarray.
    ///
    /// Always available, good for development and CPU-only deployment.
    #[default]
    NdArray,

    /// GPU backend using WGPU.
    ///
    /// Requires the `wgpu` feature and compatible GPU hardware.
    Wgpu,

    /// `LibTorch` backend (requires libtorch installation).
    LibTorch,

    /// Candle backend (Hugging Face's Rust ML framework).
    Candle,
}

impl BackendType {
    /// Returns `true` if this is a CPU backend.
    #[must_use]
    pub const fn is_cpu(&self) -> bool {
        matches!(self, Self::NdArray)
    }

    /// Returns `true` if this is a GPU backend.
    #[must_use]
    pub const fn is_gpu(&self) -> bool {
        matches!(self, Self::Wgpu | Self::LibTorch | Self::Candle)
    }

    /// Returns the backend name as a string.
    #[must_use]
    pub const fn name(&self) -> &'static str {
        match self {
            Self::NdArray => "ndarray",
            Self::Wgpu => "wgpu",
            Self::LibTorch => "libtorch",
            Self::Candle => "candle",
        }
    }

    /// Returns the recommended backend for inference.
    ///
    /// Prefers GPU when available, falls back to CPU.
    #[must_use]
    pub const fn recommended_inference() -> Self {
        // In a real implementation, this would check for GPU availability
        Self::NdArray
    }

    /// Returns the recommended backend for training.
    ///
    /// Training benefits significantly from GPU acceleration.
    #[must_use]
    pub const fn recommended_training() -> Self {
        // In a real implementation, this would check for GPU availability
        Self::Wgpu
    }
}

impl std::fmt::Display for BackendType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.name())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn backend_type_default() {
        let backend = BackendType::default();
        assert_eq!(backend, BackendType::NdArray);
    }

    #[test]
    fn backend_type_is_cpu() {
        assert!(BackendType::NdArray.is_cpu());
        assert!(!BackendType::Wgpu.is_cpu());
        assert!(!BackendType::LibTorch.is_cpu());
        assert!(!BackendType::Candle.is_cpu());
    }

    #[test]
    fn backend_type_is_gpu() {
        assert!(!BackendType::NdArray.is_gpu());
        assert!(BackendType::Wgpu.is_gpu());
        assert!(BackendType::LibTorch.is_gpu());
        assert!(BackendType::Candle.is_gpu());
    }

    #[test]
    fn backend_type_name() {
        assert_eq!(BackendType::NdArray.name(), "ndarray");
        assert_eq!(BackendType::Wgpu.name(), "wgpu");
        assert_eq!(BackendType::LibTorch.name(), "libtorch");
        assert_eq!(BackendType::Candle.name(), "candle");
    }

    #[test]
    fn backend_type_display() {
        assert_eq!(format!("{}", BackendType::NdArray), "ndarray");
        assert_eq!(format!("{}", BackendType::Wgpu), "wgpu");
    }

    #[test]
    fn backend_type_serialization() {
        let backend = BackendType::Wgpu;
        let json = serde_json::to_string(&backend);
        assert!(json.is_ok());

        let parsed: Result<BackendType, _> = serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
        assert_eq!(parsed.unwrap_or_default(), backend);
    }
}
