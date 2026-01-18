//! Assembly validation types.
//!
//! Provides structures for checking assembly integrity including
//! parent references, circular dependencies, and connection validity.

use crate::connection::Connection;

/// Assembly validation result.
///
/// Contains information about any issues found during validation.
#[derive(Debug, Clone, Default)]
pub struct AssemblyValidation {
    /// Parts with orphan parent references (`child_id`, `missing_parent_id`).
    pub orphan_references: Vec<(String, String)>,

    /// Parts with circular parent references.
    pub circular_references: Vec<String>,

    /// Invalid connections (connection, error message).
    pub invalid_connections: Vec<(Connection, String)>,
}

impl AssemblyValidation {
    /// Create a new empty validation result.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Check if the assembly is valid (no issues found).
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.orphan_references.is_empty()
            && self.circular_references.is_empty()
            && self.invalid_connections.is_empty()
    }

    /// Get the total number of issues found.
    #[must_use]
    pub fn issue_count(&self) -> usize {
        self.orphan_references.len()
            + self.circular_references.len()
            + self.invalid_connections.len()
    }

    /// Get a summary of validation issues as a string.
    #[must_use]
    pub fn summary(&self) -> String {
        if self.is_valid() {
            return "Assembly is valid".to_string();
        }

        let mut issues = Vec::new();

        if !self.orphan_references.is_empty() {
            issues.push(format!(
                "{} orphan parent reference(s)",
                self.orphan_references.len()
            ));
        }

        if !self.circular_references.is_empty() {
            issues.push(format!(
                "{} circular reference(s)",
                self.circular_references.len()
            ));
        }

        if !self.invalid_connections.is_empty() {
            issues.push(format!(
                "{} invalid connection(s)",
                self.invalid_connections.len()
            ));
        }

        format!("Validation failed: {}", issues.join(", "))
    }
}

/// Result of interference check between two parts.
#[derive(Debug, Clone)]
pub struct InterferenceResult {
    /// Whether parts interfere.
    pub has_interference: bool,

    /// Volume of overlap (if calculable).
    pub overlap_volume: f64,

    /// Minimum clearance (if no interference).
    pub min_clearance: Option<f64>,
}

impl InterferenceResult {
    /// Create a result indicating no interference.
    #[must_use]
    pub fn no_interference(clearance: f64) -> Self {
        Self {
            has_interference: false,
            overlap_volume: 0.0,
            min_clearance: Some(clearance),
        }
    }

    /// Create a result indicating interference detected.
    #[must_use]
    pub fn has_interference() -> Self {
        Self {
            has_interference: true,
            overlap_volume: 0.0,
            min_clearance: None,
        }
    }
}

/// Result of clearance check between two parts.
#[derive(Debug, Clone)]
pub struct ClearanceResult {
    /// Whether the clearance requirement is met.
    pub meets_requirement: bool,

    /// Actual clearance measured.
    pub actual_clearance: f64,

    /// Required clearance.
    pub required_clearance: f64,
}

impl ClearanceResult {
    /// Create a new clearance result.
    #[must_use]
    pub fn new(actual: f64, required: f64) -> Self {
        Self {
            meets_requirement: actual >= required,
            actual_clearance: actual,
            required_clearance: required,
        }
    }

    /// Get the clearance margin (positive = meets requirement).
    #[must_use]
    pub fn margin(&self) -> f64 {
        self.actual_clearance - self.required_clearance
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_validation_empty() {
        let validation = AssemblyValidation::new();
        assert!(validation.is_valid());
        assert_eq!(validation.issue_count(), 0);
    }

    #[test]
    fn test_validation_with_orphans() {
        let mut validation = AssemblyValidation::new();
        validation
            .orphan_references
            .push(("child".to_string(), "missing_parent".to_string()));

        assert!(!validation.is_valid());
        assert_eq!(validation.issue_count(), 1);
        assert!(validation.summary().contains("orphan"));
    }

    #[test]
    fn test_validation_with_circular() {
        let mut validation = AssemblyValidation::new();
        validation.circular_references.push("part_a".to_string());

        assert!(!validation.is_valid());
        assert!(validation.summary().contains("circular"));
    }

    #[test]
    fn test_validation_summary_multiple() {
        let mut validation = AssemblyValidation::new();
        validation
            .orphan_references
            .push(("a".to_string(), "b".to_string()));
        validation.circular_references.push("c".to_string());

        assert_eq!(validation.issue_count(), 2);
        let summary = validation.summary();
        assert!(summary.contains("orphan"));
        assert!(summary.contains("circular"));
    }

    #[test]
    fn test_interference_no_interference() {
        let result = InterferenceResult::no_interference(5.0);
        assert!(!result.has_interference);
        assert_eq!(result.min_clearance, Some(5.0));
    }

    #[test]
    fn test_interference_has_interference() {
        let result = InterferenceResult::has_interference();
        assert!(result.has_interference);
        assert!(result.min_clearance.is_none());
    }

    #[test]
    fn test_clearance_meets_requirement() {
        let result = ClearanceResult::new(10.0, 5.0);
        assert!(result.meets_requirement);
        assert!((result.margin() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_clearance_fails_requirement() {
        let result = ClearanceResult::new(3.0, 5.0);
        assert!(!result.meets_requirement);
        assert!((result.margin() - (-2.0)).abs() < 1e-10);
    }
}
