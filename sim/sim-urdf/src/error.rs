//! Error types for URDF parsing and loading.

use thiserror::Error;

/// Errors that can occur during URDF parsing and loading.
#[derive(Debug, Error)]
pub enum UrdfError {
    /// XML parsing error.
    #[error("XML parse error: {0}")]
    XmlParse(String),

    /// Missing required element.
    #[error("missing required element: {element} in {context}")]
    MissingElement {
        /// The missing element name.
        element: &'static str,
        /// Where the element was expected.
        context: String,
    },

    /// Missing required attribute.
    #[error("missing required attribute: {attribute} on {element}")]
    MissingAttribute {
        /// The missing attribute name.
        attribute: &'static str,
        /// The element that should have the attribute.
        element: String,
    },

    /// Invalid attribute value.
    #[error("invalid value for {attribute} on {element}: {message}")]
    InvalidAttribute {
        /// The attribute with the invalid value.
        attribute: &'static str,
        /// The element containing the attribute.
        element: String,
        /// Description of why the value is invalid.
        message: String,
    },

    /// Unknown joint type.
    #[error("unknown joint type: {0}")]
    UnknownJointType(String),

    /// Reference to undefined link.
    #[error("reference to undefined link: {link_name} in joint {joint_name}")]
    UndefinedLink {
        /// The link name that was referenced.
        link_name: String,
        /// The joint that referenced it.
        joint_name: String,
    },

    /// Duplicate link name.
    #[error("duplicate link name: {0}")]
    DuplicateLink(String),

    /// Duplicate joint name.
    #[error("duplicate joint name: {0}")]
    DuplicateJoint(String),

    /// Kinematic loop detected.
    #[error("kinematic loop detected: {0}")]
    KinematicLoop(String),

    /// No root link found.
    #[error("no root link found (all links are children of joints)")]
    NoRootLink,

    /// Multiple root links found.
    #[error("multiple root links found: {0:?}")]
    MultipleRootLinks(Vec<String>),

    /// Invalid inertia tensor (not positive semi-definite).
    #[error("invalid inertia tensor for link {link_name}: {message}")]
    InvalidInertia {
        /// The link with invalid inertia.
        link_name: String,
        /// Description of why the inertia is invalid.
        message: String,
    },

    /// Negative or zero mass.
    #[error("invalid mass for link {link_name}: {mass}")]
    InvalidMass {
        /// The link with invalid mass.
        link_name: String,
        /// The invalid mass value.
        mass: f64,
    },

    /// File I/O error.
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// Unsupported feature.
    #[error("unsupported URDF feature: {0}")]
    Unsupported(String),
}

impl UrdfError {
    /// Create a missing element error.
    pub fn missing_element(element: &'static str, context: impl Into<String>) -> Self {
        Self::MissingElement {
            element,
            context: context.into(),
        }
    }

    /// Create a missing attribute error.
    pub fn missing_attribute(attribute: &'static str, element: impl Into<String>) -> Self {
        Self::MissingAttribute {
            attribute,
            element: element.into(),
        }
    }

    /// Create an invalid attribute error.
    pub fn invalid_attribute(
        attribute: &'static str,
        element: impl Into<String>,
        message: impl Into<String>,
    ) -> Self {
        Self::InvalidAttribute {
            attribute,
            element: element.into(),
            message: message.into(),
        }
    }

    /// Create an undefined link error.
    pub fn undefined_link(link_name: impl Into<String>, joint_name: impl Into<String>) -> Self {
        Self::UndefinedLink {
            link_name: link_name.into(),
            joint_name: joint_name.into(),
        }
    }

    /// Create an invalid inertia error.
    pub fn invalid_inertia(link_name: impl Into<String>, message: impl Into<String>) -> Self {
        Self::InvalidInertia {
            link_name: link_name.into(),
            message: message.into(),
        }
    }

    /// Create an invalid mass error.
    pub fn invalid_mass(link_name: impl Into<String>, mass: f64) -> Self {
        Self::InvalidMass {
            link_name: link_name.into(),
            mass,
        }
    }
}

/// Result type for URDF operations.
pub type Result<T> = std::result::Result<T, UrdfError>;

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = UrdfError::missing_element("inertial", "link 'base_link'");
        assert!(err.to_string().contains("inertial"));
        assert!(err.to_string().contains("base_link"));
    }

    #[test]
    fn test_missing_attribute() {
        let err = UrdfError::missing_attribute("name", "link");
        assert!(err.to_string().contains("name"));
        assert!(err.to_string().contains("link"));
    }

    #[test]
    fn test_invalid_attribute() {
        let err = UrdfError::invalid_attribute("xyz", "origin", "expected 3 values");
        assert!(err.to_string().contains("xyz"));
        assert!(err.to_string().contains("expected 3 values"));
    }

    #[test]
    fn test_undefined_link() {
        let err = UrdfError::undefined_link("missing_link", "joint1");
        assert!(err.to_string().contains("missing_link"));
        assert!(err.to_string().contains("joint1"));
    }
}
