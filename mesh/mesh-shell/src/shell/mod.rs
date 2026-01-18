//! Shell generation for printable geometry.
//!
//! This module provides tools for transforming meshes into printable shells
//! with configurable wall thickness.

mod generate;
pub mod rim;
pub mod validation;

pub use generate::{
    generate_shell, generate_shell_no_validation, ShellGenerationResult, ShellParams,
    WallGenerationMethod,
};
pub use rim::{analyze_boundary, generate_rim, BoundaryAnalysis, RimResult};
pub use validation::{validate_shell, ShellIssue, ShellValidationResult};
