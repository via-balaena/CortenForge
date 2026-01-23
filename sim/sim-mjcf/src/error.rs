//! Error types for MJCF parsing and loading.

use thiserror::Error;

/// Errors that can occur during MJCF parsing and loading.
#[derive(Debug, Error)]
pub enum MjcfError {
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

    /// Unknown geom type.
    #[error("unknown geom type: {0}")]
    UnknownGeomType(String),

    /// Unknown actuator type.
    #[error("unknown actuator type: {0}")]
    UnknownActuatorType(String),

    /// Reference to undefined body.
    #[error("reference to undefined body: {body_name} in {context}")]
    UndefinedBody {
        /// The body name that was referenced.
        body_name: String,
        /// The context where it was referenced.
        context: String,
    },

    /// Reference to undefined joint.
    #[error("reference to undefined joint: {joint_name} in {context}")]
    UndefinedJoint {
        /// The joint name that was referenced.
        joint_name: String,
        /// The context where it was referenced.
        context: String,
    },

    /// Duplicate body name.
    #[error("duplicate body name: {0}")]
    DuplicateBody(String),

    /// Duplicate joint name.
    #[error("duplicate joint name: {0}")]
    DuplicateJoint(String),

    /// Duplicate actuator name.
    #[error("duplicate actuator name: {0}")]
    DuplicateActuator(String),

    /// Kinematic loop detected.
    #[error("kinematic loop detected: {0}")]
    KinematicLoop(String),

    /// Invalid inertia tensor (not positive semi-definite).
    #[error("invalid inertia tensor for body {body_name}: {message}")]
    InvalidInertia {
        /// The body with invalid inertia.
        body_name: String,
        /// Description of why the inertia is invalid.
        message: String,
    },

    /// Negative or zero mass.
    #[error("invalid mass for body {body_name}: {mass}")]
    InvalidMass {
        /// The body with invalid mass.
        body_name: String,
        /// The invalid mass value.
        mass: f64,
    },

    /// Invalid geom size specification.
    #[error("invalid geom size for {geom_type}: {message}")]
    InvalidGeomSize {
        /// The geom type.
        geom_type: String,
        /// Description of the error.
        message: String,
    },

    /// Reference to undefined mesh asset.
    #[error("reference to undefined mesh asset: {mesh_name} in {context}")]
    UndefinedMesh {
        /// The mesh name that was referenced.
        mesh_name: String,
        /// The context where it was referenced.
        context: String,
    },

    /// Mesh file could not be loaded.
    #[error("failed to load mesh file '{path}': {message}")]
    MeshLoadError {
        /// The mesh file path.
        path: String,
        /// Error message.
        message: String,
    },

    /// File I/O error.
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// Unsupported feature.
    #[error("unsupported MJCF feature: {0}")]
    Unsupported(String),

    /// Include file not supported.
    #[error("include files are not supported: {0}")]
    IncludeNotSupported(String),

    /// Invalid option value.
    #[error("invalid option '{option}': {message}")]
    InvalidOption {
        /// The option with the invalid value.
        option: String,
        /// Description of why the value is invalid.
        message: String,
    },

    /// MJB binary format error - invalid magic bytes.
    #[cfg(feature = "mjb")]
    #[error("invalid MJB format: expected magic bytes 'MJB1', got {0:?}")]
    InvalidMjbMagic([u8; 4]),

    /// MJB binary format error - unsupported version.
    #[cfg(feature = "mjb")]
    #[error("unsupported MJB version: {0} (supported: 1)")]
    UnsupportedMjbVersion(u32),

    /// MJB binary deserialization error.
    #[cfg(feature = "mjb")]
    #[error("MJB deserialization error: {0}")]
    MjbDeserialize(String),

    /// MJB binary serialization error.
    #[cfg(feature = "mjb")]
    #[error("MJB serialization error: {0}")]
    MjbSerialize(String),
}

impl MjcfError {
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

    /// Create an undefined body error.
    pub fn undefined_body(body_name: impl Into<String>, context: impl Into<String>) -> Self {
        Self::UndefinedBody {
            body_name: body_name.into(),
            context: context.into(),
        }
    }

    /// Create an undefined joint error.
    pub fn undefined_joint(joint_name: impl Into<String>, context: impl Into<String>) -> Self {
        Self::UndefinedJoint {
            joint_name: joint_name.into(),
            context: context.into(),
        }
    }

    /// Create an invalid inertia error.
    pub fn invalid_inertia(body_name: impl Into<String>, message: impl Into<String>) -> Self {
        Self::InvalidInertia {
            body_name: body_name.into(),
            message: message.into(),
        }
    }

    /// Create an invalid mass error.
    pub fn invalid_mass(body_name: impl Into<String>, mass: f64) -> Self {
        Self::InvalidMass {
            body_name: body_name.into(),
            mass,
        }
    }

    /// Create an invalid geom size error.
    pub fn invalid_geom_size(geom_type: impl Into<String>, message: impl Into<String>) -> Self {
        Self::InvalidGeomSize {
            geom_type: geom_type.into(),
            message: message.into(),
        }
    }

    /// Create an invalid option error.
    pub fn invalid_option(option: impl Into<String>, message: impl Into<String>) -> Self {
        Self::InvalidOption {
            option: option.into(),
            message: message.into(),
        }
    }

    /// Create an undefined mesh error.
    pub fn undefined_mesh(mesh_name: impl Into<String>, context: impl Into<String>) -> Self {
        Self::UndefinedMesh {
            mesh_name: mesh_name.into(),
            context: context.into(),
        }
    }

    /// Create a mesh load error.
    pub fn mesh_load_error(path: impl Into<String>, message: impl Into<String>) -> Self {
        Self::MeshLoadError {
            path: path.into(),
            message: message.into(),
        }
    }
}

/// Result type for MJCF operations.
pub type Result<T> = std::result::Result<T, MjcfError>;

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = MjcfError::missing_element("inertial", "body 'base'");
        assert!(err.to_string().contains("inertial"));
        assert!(err.to_string().contains("base"));
    }

    #[test]
    fn test_missing_attribute() {
        let err = MjcfError::missing_attribute("name", "body");
        assert!(err.to_string().contains("name"));
        assert!(err.to_string().contains("body"));
    }

    #[test]
    fn test_invalid_attribute() {
        let err = MjcfError::invalid_attribute("pos", "body", "expected 3 values");
        assert!(err.to_string().contains("pos"));
        assert!(err.to_string().contains("expected 3 values"));
    }

    #[test]
    fn test_undefined_body() {
        let err = MjcfError::undefined_body("missing_body", "joint1");
        assert!(err.to_string().contains("missing_body"));
        assert!(err.to_string().contains("joint1"));
    }

    #[test]
    fn test_undefined_joint() {
        let err = MjcfError::undefined_joint("missing_joint", "actuator motor1");
        assert!(err.to_string().contains("missing_joint"));
        assert!(err.to_string().contains("motor1"));
    }

    #[test]
    fn test_invalid_geom_size() {
        let err = MjcfError::invalid_geom_size("sphere", "expected 1 value");
        assert!(err.to_string().contains("sphere"));
        assert!(err.to_string().contains("expected 1 value"));
    }
}
