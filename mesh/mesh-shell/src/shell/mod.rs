//! Shell generation for printable geometry.
//!
//! This module provides tools for transforming meshes into printable shells
//! with configurable wall thickness.

mod generate;
pub mod rim;
pub mod validation;

pub use generate::{
    ShellGenerationResult, ShellParams, WallGenerationMethod, generate_shell,
    generate_shell_no_validation,
};
pub use rim::{BoundaryAnalysis, RimResult, analyze_boundary, generate_rim};
pub use validation::{ShellIssue, ShellValidationResult, validate_shell};
