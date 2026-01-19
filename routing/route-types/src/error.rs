//! Error types for routing operations.
//!
//! This module defines the [`RoutingError`] enum which represents all possible
//! errors that can occur during pathfinding and route optimization.

use std::time::Duration;

use cf_spatial::VoxelCoord;

/// Errors that can occur during routing operations.
///
/// This enum covers failures in pathfinding, constraint validation,
/// configuration issues, and resource limits.
///
/// # Example
///
/// ```
/// use route_types::RoutingError;
/// use cf_spatial::VoxelCoord;
///
/// let error = RoutingError::NoPathFound {
///     start: VoxelCoord::new(0, 0, 0),
///     goal: VoxelCoord::new(10, 10, 10),
/// };
///
/// assert!(error.to_string().contains("no path found"));
/// ```
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum RoutingError {
    /// No valid path exists between start and goal.
    ///
    /// This typically occurs when the goal is completely blocked by obstacles,
    /// or there is no connected path through the free space.
    #[error("no path found from {start:?} to {goal:?}")]
    NoPathFound {
        /// The starting voxel coordinate.
        start: VoxelCoord,
        /// The goal voxel coordinate.
        goal: VoxelCoord,
    },

    /// The start position is inside an obstacle or otherwise blocked.
    ///
    /// Ensure the start position is in free space before pathfinding.
    #[error("start position {0:?} is blocked")]
    StartBlocked(VoxelCoord),

    /// The goal position is inside an obstacle or otherwise blocked.
    ///
    /// Ensure the goal position is in free space before pathfinding.
    #[error("goal position {0:?} is blocked")]
    GoalBlocked(VoxelCoord),

    /// A required via point in the path is unreachable.
    ///
    /// This occurs when multi-segment routing cannot reach one of the
    /// specified intermediate waypoints.
    #[error("via point {index} at {coord:?} is unreachable")]
    ViaPointUnreachable {
        /// The index of the unreachable via point (0-based).
        index: usize,
        /// The voxel coordinate of the unreachable point.
        coord: VoxelCoord,
    },

    /// The computed path violates a specified constraint.
    ///
    /// This can occur during post-processing validation when a path
    /// does not satisfy clearance, curvature, or other requirements.
    #[error("constraint violation: {0}")]
    ConstraintViolation(String),

    /// An invalid configuration parameter was provided.
    ///
    /// Check the configuration values for valid ranges.
    #[error("invalid configuration: {0}")]
    InvalidConfig(String),

    /// The pathfinding operation exceeded the time limit.
    ///
    /// Consider increasing the timeout or simplifying the problem.
    #[error("pathfinding timed out after {0:?}")]
    Timeout(Duration),

    /// An integer overflow occurred during coordinate calculation.
    ///
    /// This typically happens when coordinates exceed `i32` bounds.
    #[error("coordinate overflow")]
    CoordinateOverflow,
}

impl RoutingError {
    /// Creates a constraint violation error with the given message.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::RoutingError;
    ///
    /// let error = RoutingError::constraint_violation("clearance too small");
    /// assert!(error.to_string().contains("clearance"));
    /// ```
    #[must_use]
    pub fn constraint_violation(message: impl Into<String>) -> Self {
        Self::ConstraintViolation(message.into())
    }

    /// Creates an invalid configuration error with the given message.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::RoutingError;
    ///
    /// let error = RoutingError::invalid_config("max_nodes must be positive");
    /// assert!(error.to_string().contains("max_nodes"));
    /// ```
    #[must_use]
    pub fn invalid_config(message: impl Into<String>) -> Self {
        Self::InvalidConfig(message.into())
    }

    /// Returns `true` if this is a "no path found" error.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::RoutingError;
    /// use cf_spatial::VoxelCoord;
    ///
    /// let error = RoutingError::NoPathFound {
    ///     start: VoxelCoord::origin(),
    ///     goal: VoxelCoord::new(5, 5, 5),
    /// };
    /// assert!(error.is_no_path_found());
    /// ```
    #[must_use]
    pub const fn is_no_path_found(&self) -> bool {
        matches!(self, Self::NoPathFound { .. })
    }

    /// Returns `true` if this is a timeout error.
    ///
    /// # Example
    ///
    /// ```
    /// use route_types::RoutingError;
    /// use std::time::Duration;
    ///
    /// let error = RoutingError::Timeout(Duration::from_secs(30));
    /// assert!(error.is_timeout());
    /// ```
    #[must_use]
    pub const fn is_timeout(&self) -> bool {
        matches!(self, Self::Timeout(_))
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::single_char_pattern)]
mod tests {
    use super::*;

    #[test]
    fn test_no_path_found_display() {
        let error = RoutingError::NoPathFound {
            start: VoxelCoord::new(0, 0, 0),
            goal: VoxelCoord::new(10, 10, 10),
        };
        let msg = error.to_string();
        assert!(msg.contains("no path found"));
        assert!(msg.contains("(0, 0, 0)") || msg.contains("x: 0"));
    }

    #[test]
    fn test_start_blocked_display() {
        let error = RoutingError::StartBlocked(VoxelCoord::new(5, 5, 5));
        assert!(error.to_string().contains("start position"));
        assert!(error.to_string().contains("blocked"));
    }

    #[test]
    fn test_goal_blocked_display() {
        let error = RoutingError::GoalBlocked(VoxelCoord::new(5, 5, 5));
        assert!(error.to_string().contains("goal position"));
        assert!(error.to_string().contains("blocked"));
    }

    #[test]
    fn test_via_point_unreachable_display() {
        let error = RoutingError::ViaPointUnreachable {
            index: 2,
            coord: VoxelCoord::new(3, 3, 3),
        };
        let msg = error.to_string();
        assert!(msg.contains("via point"));
        assert!(msg.contains("2"));
        assert!(msg.contains("unreachable"));
    }

    #[test]
    fn test_constraint_violation_display() {
        let error = RoutingError::constraint_violation("minimum clearance not met");
        assert!(error.to_string().contains("constraint violation"));
        assert!(error.to_string().contains("minimum clearance"));
    }

    #[test]
    fn test_invalid_config_display() {
        let error = RoutingError::invalid_config("timeout must be positive");
        assert!(error.to_string().contains("invalid configuration"));
        assert!(error.to_string().contains("timeout"));
    }

    #[test]
    fn test_timeout_display() {
        let error = RoutingError::Timeout(Duration::from_secs(30));
        assert!(error.to_string().contains("timed out"));
        assert!(error.to_string().contains("30"));
    }

    #[test]
    fn test_coordinate_overflow_display() {
        let error = RoutingError::CoordinateOverflow;
        assert!(error.to_string().contains("coordinate overflow"));
    }

    #[test]
    fn test_is_no_path_found() {
        let no_path = RoutingError::NoPathFound {
            start: VoxelCoord::origin(),
            goal: VoxelCoord::new(5, 5, 5),
        };
        assert!(no_path.is_no_path_found());

        let timeout = RoutingError::Timeout(Duration::from_secs(1));
        assert!(!timeout.is_no_path_found());
    }

    #[test]
    fn test_is_timeout() {
        let timeout = RoutingError::Timeout(Duration::from_secs(30));
        assert!(timeout.is_timeout());

        let no_path = RoutingError::NoPathFound {
            start: VoxelCoord::origin(),
            goal: VoxelCoord::new(5, 5, 5),
        };
        assert!(!no_path.is_timeout());
    }

    #[test]
    fn test_error_debug() {
        let error = RoutingError::CoordinateOverflow;
        let debug_str = format!("{error:?}");
        assert!(debug_str.contains("CoordinateOverflow"));
    }

    #[test]
    fn test_constraint_violation_helper() {
        let error = RoutingError::constraint_violation("test message");
        assert!(matches!(error, RoutingError::ConstraintViolation(msg) if msg == "test message"));
    }

    #[test]
    fn test_invalid_config_helper() {
        let error = RoutingError::invalid_config("test config error");
        assert!(matches!(error, RoutingError::InvalidConfig(msg) if msg == "test config error"));
    }
}
