//! Error types for mesh I/O operations.

use std::path::PathBuf;
use thiserror::Error;

/// Result type for mesh I/O operations.
pub type IoResult<T> = Result<T, IoError>;

/// Errors that can occur during mesh I/O operations.
#[derive(Debug, Error)]
pub enum IoError {
    /// File not found.
    #[error("file not found: {path}")]
    FileNotFound {
        /// Path that was not found.
        path: PathBuf,
    },

    /// Unknown file format (unrecognized extension).
    #[error("unknown file format: .{extension}")]
    UnknownFormat {
        /// The unrecognized extension.
        extension: String,
    },

    /// Invalid file content (parse error).
    #[error("invalid file content: {message}")]
    InvalidContent {
        /// Description of what was invalid.
        message: String,
    },

    /// Unexpected end of file.
    #[error("unexpected end of file at position {position}")]
    UnexpectedEof {
        /// Position in the file where EOF was encountered.
        position: u64,
    },

    /// Invalid header in binary STL.
    #[error("invalid STL header: expected {expected} bytes, got {got}")]
    InvalidHeader {
        /// Expected header size.
        expected: usize,
        /// Actual header size.
        got: usize,
    },

    /// Invalid face count.
    #[error("invalid face count: expected {expected}, got {got}")]
    InvalidFaceCount {
        /// Expected number of faces.
        expected: u32,
        /// Actual number of faces read.
        got: u32,
    },

    /// I/O error from the standard library.
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// UTF-8 decoding error.
    #[error("UTF-8 decoding error: {0}")]
    Utf8(#[from] std::str::Utf8Error),

    /// String conversion error.
    #[error("string conversion error: {0}")]
    FromUtf8(#[from] std::string::FromUtf8Error),

    /// Float parsing error.
    #[error("float parsing error: {0}")]
    ParseFloat(#[from] std::num::ParseFloatError),

    /// Integer parsing error.
    #[error("integer parsing error: {0}")]
    ParseInt(#[from] std::num::ParseIntError),
}

impl IoError {
    /// Create an `InvalidContent` error with the given message.
    #[must_use]
    pub fn invalid_content(message: impl Into<String>) -> Self {
        Self::InvalidContent {
            message: message.into(),
        }
    }
}
