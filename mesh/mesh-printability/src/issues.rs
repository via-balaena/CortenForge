//! Print issues and severity types.
//!
//! Defines the various issues that can be detected during print validation.

use mesh_types::Point3;

/// A print-related issue found during validation.
#[derive(Debug, Clone)]
pub struct PrintIssue {
    /// Type of issue.
    pub issue_type: PrintIssueType,

    /// Severity of the issue.
    pub severity: IssueSeverity,

    /// Human-readable description.
    pub description: String,

    /// Location in mesh (if applicable).
    pub location: Option<Point3<f64>>,

    /// Affected vertex or face indices.
    pub affected_elements: Vec<u32>,
}

impl PrintIssue {
    /// Create a new print issue.
    #[must_use]
    pub fn new(
        issue_type: PrintIssueType,
        severity: IssueSeverity,
        description: impl Into<String>,
    ) -> Self {
        Self {
            issue_type,
            severity,
            description: description.into(),
            location: None,
            affected_elements: Vec::new(),
        }
    }

    /// Set the location of the issue.
    #[must_use]
    pub fn with_location(mut self, location: Point3<f64>) -> Self {
        self.location = Some(location);
        self
    }

    /// Set the affected elements.
    #[must_use]
    pub fn with_affected_elements(mut self, elements: Vec<u32>) -> Self {
        self.affected_elements = elements;
        self
    }

    /// Check if this is a critical issue.
    #[must_use]
    pub fn is_critical(&self) -> bool {
        matches!(self.severity, IssueSeverity::Critical)
    }

    /// Check if this is a warning.
    #[must_use]
    pub fn is_warning(&self) -> bool {
        matches!(self.severity, IssueSeverity::Warning)
    }
}

/// Types of print issues.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PrintIssueType {
    /// Wall is too thin for the printer.
    ThinWall,
    /// Overhang angle exceeds maximum.
    ExcessiveOverhang,
    /// Bridge span is too long.
    LongBridge,
    /// Feature is smaller than minimum size.
    SmallFeature,
    /// Trapped volume (internal cavity).
    TrappedVolume,
    /// Mesh doesn't fit in build volume.
    ExceedsBuildVolume,
    /// Mesh is not watertight.
    NotWatertight,
    /// Mesh has non-manifold edges.
    NonManifold,
    /// Mesh has self-intersections.
    SelfIntersecting,
    /// Other issue.
    Other,
}

impl PrintIssueType {
    /// Get a human-readable name for the issue type.
    #[must_use]
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::ThinWall => "Thin Wall",
            Self::ExcessiveOverhang => "Excessive Overhang",
            Self::LongBridge => "Long Bridge",
            Self::SmallFeature => "Small Feature",
            Self::TrappedVolume => "Trapped Volume",
            Self::ExceedsBuildVolume => "Exceeds Build Volume",
            Self::NotWatertight => "Not Watertight",
            Self::NonManifold => "Non-Manifold",
            Self::SelfIntersecting => "Self-Intersecting",
            Self::Other => "Other",
        }
    }
}

/// Severity of an issue.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum IssueSeverity {
    /// Informational only.
    Info,
    /// May cause problems but can still print.
    Warning,
    /// Will likely fail or have significant defects.
    Critical,
}

impl IssueSeverity {
    /// Get a human-readable name for the severity.
    #[must_use]
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Info => "Info",
            Self::Warning => "Warning",
            Self::Critical => "Critical",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_issue_creation() {
        let issue = PrintIssue::new(
            PrintIssueType::ThinWall,
            IssueSeverity::Warning,
            "Wall is 0.5mm (minimum 1.0mm)",
        );

        assert_eq!(issue.issue_type, PrintIssueType::ThinWall);
        assert_eq!(issue.severity, IssueSeverity::Warning);
        assert!(issue.location.is_none());
    }

    #[test]
    fn test_issue_with_location() {
        let issue = PrintIssue::new(
            PrintIssueType::ExcessiveOverhang,
            IssueSeverity::Critical,
            "Overhang at 60 degrees",
        )
        .with_location(Point3::new(10.0, 20.0, 30.0));

        assert!(issue.location.is_some());
        let loc = issue.location.as_ref().map_or(0.0, |p| p.x);
        assert!((loc - 10.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_issue_severity_checks() {
        let critical = PrintIssue::new(PrintIssueType::NotWatertight, IssueSeverity::Critical, "");
        let warning = PrintIssue::new(PrintIssueType::ThinWall, IssueSeverity::Warning, "");
        let info = PrintIssue::new(PrintIssueType::Other, IssueSeverity::Info, "");

        assert!(critical.is_critical());
        assert!(!critical.is_warning());

        assert!(warning.is_warning());
        assert!(!warning.is_critical());

        assert!(!info.is_critical());
        assert!(!info.is_warning());
    }

    #[test]
    fn test_issue_type_as_str() {
        assert_eq!(PrintIssueType::ThinWall.as_str(), "Thin Wall");
        assert_eq!(PrintIssueType::ExcessiveOverhang.as_str(), "Excessive Overhang");
        assert_eq!(PrintIssueType::NotWatertight.as_str(), "Not Watertight");
    }

    #[test]
    fn test_severity_ordering() {
        assert!(IssueSeverity::Info < IssueSeverity::Warning);
        assert!(IssueSeverity::Warning < IssueSeverity::Critical);
    }
}
