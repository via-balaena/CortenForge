//! Connections between parts in an assembly.
//!
//! Connections define how parts relate to each other mechanically,
//! such as snap-fits, press-fits, or clearance requirements.

use hashbrown::HashMap;
use mesh_types::Point3;

/// Connection between two parts.
///
/// Connections describe the mechanical relationship between parts,
/// including fit type and tolerances.
///
/// # Example
///
/// ```
/// use mesh_assembly::{Connection, ConnectionType};
///
/// // Create a snap-fit connection
/// let snap = Connection::snap_fit("shell", "liner");
///
/// // Create a press-fit with interference
/// let press = Connection::press_fit("bushing", "housing", 0.1);
///
/// // Create a clearance fit
/// let clearance = Connection::clearance("piston", "cylinder", 0.05);
/// ```
#[derive(Debug, Clone)]
#[allow(clippy::struct_field_names)]
pub struct Connection {
    /// Source part ID.
    from_part: String,

    /// Target part ID.
    to_part: String,

    /// Connection type.
    connection_type: ConnectionType,

    /// Connection parameters.
    params: ConnectionParams,

    /// Name/description.
    name: Option<String>,
}

impl Connection {
    /// Create a new connection.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_assembly::{Connection, ConnectionType};
    ///
    /// let conn = Connection::new("part_a", "part_b", ConnectionType::Adhesive);
    /// ```
    #[must_use]
    pub fn new(
        from_part: impl Into<String>,
        to_part: impl Into<String>,
        connection_type: ConnectionType,
    ) -> Self {
        Self {
            from_part: from_part.into(),
            to_part: to_part.into(),
            connection_type,
            params: ConnectionParams::default(),
            name: None,
        }
    }

    /// Create a snap-fit connection.
    #[must_use]
    pub fn snap_fit(from_part: impl Into<String>, to_part: impl Into<String>) -> Self {
        Self::new(from_part, to_part, ConnectionType::SnapFit)
    }

    /// Create a press-fit connection with specified interference.
    #[must_use]
    pub fn press_fit(
        from_part: impl Into<String>,
        to_part: impl Into<String>,
        interference: f64,
    ) -> Self {
        let mut conn = Self::new(from_part, to_part, ConnectionType::PressFit);
        conn.params.interference = Some(interference);
        conn
    }

    /// Create a clearance connection with specified minimum gap.
    #[must_use]
    pub fn clearance(
        from_part: impl Into<String>,
        to_part: impl Into<String>,
        min_clearance: f64,
    ) -> Self {
        let mut conn = Self::new(from_part, to_part, ConnectionType::Clearance);
        conn.params.clearance = Some(min_clearance);
        conn
    }

    /// Get the source part ID.
    #[must_use]
    pub fn from_part(&self) -> &str {
        &self.from_part
    }

    /// Get the target part ID.
    #[must_use]
    pub fn to_part(&self) -> &str {
        &self.to_part
    }

    /// Get the connection type.
    #[must_use]
    pub fn connection_type(&self) -> ConnectionType {
        self.connection_type
    }

    /// Get the connection parameters.
    #[must_use]
    pub fn params(&self) -> &ConnectionParams {
        &self.params
    }

    /// Get a mutable reference to the connection parameters.
    pub fn params_mut(&mut self) -> &mut ConnectionParams {
        &mut self.params
    }

    /// Get the connection name.
    #[must_use]
    pub fn name(&self) -> Option<&str> {
        self.name.as_deref()
    }

    /// Set connection name (builder pattern).
    #[must_use]
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// Check if this connection involves a specific part.
    #[must_use]
    pub fn involves_part(&self, part_id: &str) -> bool {
        self.from_part == part_id || self.to_part == part_id
    }
}

/// Type of connection between parts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ConnectionType {
    /// Snap-fit (male/female interlocking).
    SnapFit,

    /// Press-fit (interference fit).
    PressFit,

    /// Clearance fit (loose with minimum gap).
    Clearance,

    /// Glue/adhesive bond.
    Adhesive,

    /// Threaded fastener.
    Threaded,

    /// Sliding fit.
    Sliding,

    /// Custom connection type.
    Custom,
}

impl ConnectionType {
    /// Get a human-readable name for the connection type.
    #[must_use]
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::SnapFit => "Snap Fit",
            Self::PressFit => "Press Fit",
            Self::Clearance => "Clearance",
            Self::Adhesive => "Adhesive",
            Self::Threaded => "Threaded",
            Self::Sliding => "Sliding",
            Self::Custom => "Custom",
        }
    }
}

/// Parameters for a connection.
#[derive(Debug, Clone, Default)]
pub struct ConnectionParams {
    /// Interference amount (for press-fit), positive = overlap.
    pub interference: Option<f64>,

    /// Clearance amount (for clearance fit), minimum gap.
    pub clearance: Option<f64>,

    /// Snap feature height (for snap-fit).
    pub snap_height: Option<f64>,

    /// Undercut angle for snap (degrees).
    pub undercut_angle: Option<f64>,

    /// Connection location (relative to `from_part`).
    pub location: Option<Point3<f64>>,

    /// Custom parameters.
    pub custom: HashMap<String, String>,
}

impl ConnectionParams {
    /// Create empty connection parameters.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Set interference (builder pattern).
    #[must_use]
    pub fn with_interference(mut self, interference: f64) -> Self {
        self.interference = Some(interference);
        self
    }

    /// Set clearance (builder pattern).
    #[must_use]
    pub fn with_clearance(mut self, clearance: f64) -> Self {
        self.clearance = Some(clearance);
        self
    }

    /// Set snap height (builder pattern).
    #[must_use]
    pub fn with_snap_height(mut self, height: f64) -> Self {
        self.snap_height = Some(height);
        self
    }

    /// Set undercut angle in degrees (builder pattern).
    #[must_use]
    pub fn with_undercut_angle(mut self, angle: f64) -> Self {
        self.undercut_angle = Some(angle);
        self
    }

    /// Set location (builder pattern).
    #[must_use]
    pub fn with_location(mut self, location: Point3<f64>) -> Self {
        self.location = Some(location);
        self
    }

    /// Set a custom parameter.
    pub fn set_custom(&mut self, key: impl Into<String>, value: impl Into<String>) {
        self.custom.insert(key.into(), value.into());
    }

    /// Get a custom parameter.
    #[must_use]
    pub fn get_custom(&self, key: &str) -> Option<&str> {
        self.custom.get(key).map(String::as_str)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_connection_new() {
        let conn = Connection::new("a", "b", ConnectionType::Adhesive);

        assert_eq!(conn.from_part(), "a");
        assert_eq!(conn.to_part(), "b");
        assert_eq!(conn.connection_type(), ConnectionType::Adhesive);
        assert!(conn.name().is_none());
    }

    #[test]
    fn test_snap_fit() {
        let conn = Connection::snap_fit("shell", "liner");

        assert_eq!(conn.connection_type(), ConnectionType::SnapFit);
        assert!(conn.involves_part("shell"));
        assert!(conn.involves_part("liner"));
        assert!(!conn.involves_part("other"));
    }

    #[test]
    fn test_press_fit() {
        let conn = Connection::press_fit("bushing", "housing", 0.15);

        assert_eq!(conn.connection_type(), ConnectionType::PressFit);
        assert_eq!(conn.params().interference, Some(0.15));
    }

    #[test]
    fn test_clearance() {
        let conn = Connection::clearance("piston", "cylinder", 0.05);

        assert_eq!(conn.connection_type(), ConnectionType::Clearance);
        assert_eq!(conn.params().clearance, Some(0.05));
    }

    #[test]
    fn test_connection_with_name() {
        let conn = Connection::snap_fit("a", "b").with_name("main joint");

        assert_eq!(conn.name(), Some("main joint"));
    }

    #[test]
    fn test_connection_type_as_str() {
        assert_eq!(ConnectionType::SnapFit.as_str(), "Snap Fit");
        assert_eq!(ConnectionType::PressFit.as_str(), "Press Fit");
        assert_eq!(ConnectionType::Clearance.as_str(), "Clearance");
        assert_eq!(ConnectionType::Adhesive.as_str(), "Adhesive");
        assert_eq!(ConnectionType::Threaded.as_str(), "Threaded");
        assert_eq!(ConnectionType::Sliding.as_str(), "Sliding");
        assert_eq!(ConnectionType::Custom.as_str(), "Custom");
    }

    #[test]
    fn test_connection_params_builder() {
        let params = ConnectionParams::new()
            .with_interference(0.1)
            .with_clearance(0.05)
            .with_snap_height(2.0)
            .with_undercut_angle(45.0)
            .with_location(Point3::new(1.0, 2.0, 3.0));

        assert_eq!(params.interference, Some(0.1));
        assert_eq!(params.clearance, Some(0.05));
        assert_eq!(params.snap_height, Some(2.0));
        assert_eq!(params.undercut_angle, Some(45.0));
        assert!(params.location.is_some());
    }

    #[test]
    fn test_connection_params_custom() {
        let mut params = ConnectionParams::new();
        params.set_custom("bolt_size", "M3");

        assert_eq!(params.get_custom("bolt_size"), Some("M3"));
        assert!(params.get_custom("nonexistent").is_none());
    }
}
