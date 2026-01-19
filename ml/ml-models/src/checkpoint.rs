//! Checkpoint persistence for model weights.

use std::path::Path;

use burn::module::Module;
use burn::prelude::Backend;
use burn::record::{BinFileRecorder, FullPrecisionSettings, PrettyJsonFileRecorder, Recorder};
use serde::{Deserialize, Serialize};

use crate::error::{ModelError, Result};

/// Supported checkpoint file formats.
///
/// # Example
///
/// ```
/// use ml_models::CheckpointFormat;
///
/// let format = CheckpointFormat::from_extension("bin");
/// assert_eq!(format, Some(CheckpointFormat::Binary));
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
pub enum CheckpointFormat {
    /// Binary format - compact and fast.
    ///
    /// Uses Burn's `BinFileRecorder` with full precision.
    /// Recommended for production deployments.
    #[default]
    Binary,

    /// JSON format - human-readable.
    ///
    /// Uses Burn's `PrettyJsonFileRecorder` for debugging
    /// and inspection. Larger file size but portable.
    Json,
}

impl CheckpointFormat {
    /// Determines format from file extension.
    ///
    /// - `.bin`, `.burn` -> Binary
    /// - `.json` -> Json
    /// - Other -> None
    #[must_use]
    pub fn from_extension(ext: &str) -> Option<Self> {
        match ext.to_lowercase().as_str() {
            "bin" | "burn" => Some(Self::Binary),
            "json" => Some(Self::Json),
            _ => None,
        }
    }

    /// Determines format from file path.
    #[must_use]
    pub fn from_path(path: &Path) -> Option<Self> {
        path.extension()
            .and_then(|ext| ext.to_str())
            .and_then(Self::from_extension)
    }

    /// Returns the default file extension for this format.
    #[must_use]
    pub const fn extension(&self) -> &'static str {
        match self {
            Self::Binary => "bin",
            Self::Json => "json",
        }
    }

    /// Returns the format name.
    #[must_use]
    pub const fn name(&self) -> &'static str {
        match self {
            Self::Binary => "binary",
            Self::Json => "json",
        }
    }
}

impl std::fmt::Display for CheckpointFormat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.name())
    }
}

/// Saves a model checkpoint to a file.
///
/// # Arguments
///
/// - `model`: The model to save
/// - `path`: Output file path (without extension)
/// - `format`: Checkpoint format to use
///
/// # Returns
///
/// The full path to the saved checkpoint (with extension added).
///
/// # Errors
///
/// Returns `ModelError::SaveCheckpoint` if saving fails.
///
/// # Example
///
/// ```ignore
/// use ml_models::{save_checkpoint, CheckpointFormat, LinearClassifier};
///
/// let path = save_checkpoint(&model, "model", CheckpointFormat::Binary)?;
/// ```
pub fn save_checkpoint<B, M>(model: &M, path: &str, format: CheckpointFormat) -> Result<String>
where
    B: Backend,
    M: Module<B>,
    M::Record: Serialize,
{
    let full_path = format!("{}.{}", path, format.extension());
    let record = model.clone().into_record();

    match format {
        CheckpointFormat::Binary => {
            let recorder = BinFileRecorder::<FullPrecisionSettings>::new();
            recorder
                .record(record, full_path.clone().into())
                .map_err(|e| ModelError::save_checkpoint(&full_path, e.to_string()))?;
        }
        CheckpointFormat::Json => {
            let recorder = PrettyJsonFileRecorder::<FullPrecisionSettings>::new();
            recorder
                .record(record, full_path.clone().into())
                .map_err(|e| ModelError::save_checkpoint(&full_path, e.to_string()))?;
        }
    }

    Ok(full_path)
}

/// Loads a model checkpoint from a file.
///
/// # Arguments
///
/// - `model`: The model to load weights into
/// - `path`: Path to the checkpoint file (with extension)
/// - `device`: Device to load the model onto
///
/// # Returns
///
/// The model with loaded weights.
///
/// # Errors
///
/// Returns `ModelError::LoadCheckpoint` if loading fails.
/// Returns `ModelError::CheckpointNotFound` if the file doesn't exist.
/// Returns `ModelError::UnsupportedFormat` if the format can't be determined.
///
/// # Example
///
/// ```ignore
/// use ml_models::{load_checkpoint, LinearClassifier, LinearClassifierConfig};
///
/// let config = LinearClassifierConfig::default();
/// let model = LinearClassifier::<MyBackend>::new(config, &device);
/// let model = load_checkpoint(model, "model.bin", &device)?;
/// ```
pub fn load_checkpoint<B, M>(model: M, path: &str, device: &B::Device) -> Result<M>
where
    B: Backend,
    M: Module<B>,
    M::Record: for<'de> Deserialize<'de>,
{
    let path_obj = Path::new(path);

    // Check if file exists
    if !path_obj.exists() {
        return Err(ModelError::checkpoint_not_found(path));
    }

    // Determine format from extension
    let format = CheckpointFormat::from_path(path_obj)
        .ok_or_else(|| ModelError::unsupported_format(path))?;

    let loaded = match format {
        CheckpointFormat::Binary => {
            let recorder = BinFileRecorder::<FullPrecisionSettings>::new();
            model
                .load_file(path_obj, &recorder, device)
                .map_err(|e| ModelError::load_checkpoint(path, e.to_string()))?
        }
        CheckpointFormat::Json => {
            let recorder = PrettyJsonFileRecorder::<FullPrecisionSettings>::new();
            model
                .load_file(path_obj, &recorder, device)
                .map_err(|e| ModelError::load_checkpoint(path, e.to_string()))?
        }
    };

    Ok(loaded)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn format_from_extension() {
        assert_eq!(
            CheckpointFormat::from_extension("bin"),
            Some(CheckpointFormat::Binary)
        );
        assert_eq!(
            CheckpointFormat::from_extension("burn"),
            Some(CheckpointFormat::Binary)
        );
        assert_eq!(
            CheckpointFormat::from_extension("json"),
            Some(CheckpointFormat::Json)
        );
        assert_eq!(
            CheckpointFormat::from_extension("BIN"),
            Some(CheckpointFormat::Binary)
        );
        assert_eq!(CheckpointFormat::from_extension("xml"), None);
    }

    #[test]
    fn format_from_path() {
        assert_eq!(
            CheckpointFormat::from_path(Path::new("model.bin")),
            Some(CheckpointFormat::Binary)
        );
        assert_eq!(
            CheckpointFormat::from_path(Path::new("model.json")),
            Some(CheckpointFormat::Json)
        );
        assert_eq!(
            CheckpointFormat::from_path(Path::new("/path/to/model.burn")),
            Some(CheckpointFormat::Binary)
        );
        assert_eq!(CheckpointFormat::from_path(Path::new("model.xml")), None);
        assert_eq!(CheckpointFormat::from_path(Path::new("model")), None);
    }

    #[test]
    fn format_extension() {
        assert_eq!(CheckpointFormat::Binary.extension(), "bin");
        assert_eq!(CheckpointFormat::Json.extension(), "json");
    }

    #[test]
    fn format_name() {
        assert_eq!(CheckpointFormat::Binary.name(), "binary");
        assert_eq!(CheckpointFormat::Json.name(), "json");
    }

    #[test]
    fn format_display() {
        assert_eq!(format!("{}", CheckpointFormat::Binary), "binary");
        assert_eq!(format!("{}", CheckpointFormat::Json), "json");
    }

    #[test]
    fn format_default() {
        assert_eq!(CheckpointFormat::default(), CheckpointFormat::Binary);
    }

    #[test]
    fn format_serialization() {
        let format = CheckpointFormat::Binary;
        let json = serde_json::to_string(&format);
        assert!(json.is_ok());

        let parsed: std::result::Result<CheckpointFormat, _> =
            serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
        assert_eq!(parsed.unwrap_or_default(), format);
    }
}
