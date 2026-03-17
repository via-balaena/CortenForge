//! Tendon routing and channel subtraction.
//!
//! A [`TendonDef`] defines a tendon's path through a mechanism via ordered
//! [`TendonWaypoint`]s. Each waypoint references a part and a position in that
//! part's local frame. The tendon carves cylindrical channels through parts
//! using [`Solid::pipe`] + [`Solid::subtract`].
//!
//! Channel subtraction is the geometric operation: for each part, collect the
//! tendon's waypoints within that part, build a polyline pipe, and subtract it
//! from the part's solid. This produces the physical void through which the
//! tendon routes.
//!
//! Downstream mapping to MJCF (Session 10):
//! - Each `TendonDef` → `<tendon>` element
//! - Each `TendonWaypoint` → `<site>` element at the waypoint position

use nalgebra::Point3;

use crate::Solid;

/// A waypoint along a tendon's path, referencing a part and a position.
///
/// The position is in the referenced part's local coordinate frame.
///
/// # Example
///
/// ```
/// use cf_design::TendonWaypoint;
/// use nalgebra::Point3;
///
/// let wp = TendonWaypoint::new("palm", Point3::new(8.0, -10.0, 0.0));
/// assert_eq!(wp.part(), "palm");
/// ```
#[derive(Debug, Clone)]
pub struct TendonWaypoint {
    part: String,
    position: Point3<f64>,
}

impl TendonWaypoint {
    /// Create a tendon waypoint.
    ///
    /// # Panics
    ///
    /// Panics if `part` is empty or `position` has non-finite coordinates.
    #[must_use]
    pub fn new(part: impl Into<String>, position: Point3<f64>) -> Self {
        let part = part.into();
        assert!(!part.is_empty(), "waypoint part name must not be empty");
        assert!(
            position.iter().all(|c| c.is_finite()),
            "waypoint position must have finite coordinates"
        );
        Self { part, position }
    }

    /// Part name this waypoint belongs to.
    #[must_use]
    pub fn part(&self) -> &str {
        &self.part
    }

    /// Position in the part's local coordinate frame.
    #[must_use]
    pub const fn position(&self) -> &Point3<f64> {
        &self.position
    }
}

/// Tendon routing through a mechanism.
///
/// Defines both the physical channel geometry (carved from parts via
/// [`channel_solid`](Self::channel_solid)) and the simulation-side tendon
/// (MJCF `<tendon>` with `<site>` waypoints, Session 10).
///
/// A `channel_radius` of `0.0` means a surface-routed tendon — no channel
/// is carved from any part.
///
/// # Example
///
/// ```
/// use cf_design::{TendonDef, TendonWaypoint};
/// use nalgebra::Point3;
///
/// let tendon = TendonDef::new(
///     "flexor_1",
///     vec![
///         TendonWaypoint::new("palm", Point3::new(8.0, -10.0, 0.0)),
///         TendonWaypoint::new("palm", Point3::new(8.0, 18.0, 0.0)),
///         TendonWaypoint::new("finger", Point3::new(0.0, 5.0, 0.0)),
///     ],
///     1.5,
/// )
/// .with_stiffness(100.0)
/// .with_damping(5.0);
/// ```
#[derive(Debug, Clone)]
pub struct TendonDef {
    name: String,
    waypoints: Vec<TendonWaypoint>,
    channel_radius: f64,
    stiffness: Option<f64>,
    damping: Option<f64>,
}

impl TendonDef {
    /// Create a tendon definition.
    ///
    /// - `waypoints` — ordered path through parts (at least 2).
    /// - `channel_radius` — radius of the cylindrical void carved through
    ///   parts. Set to `0.0` for surface-routed tendons (no channel).
    ///
    /// # Panics
    ///
    /// Panics if `name` is empty, fewer than 2 waypoints, or `channel_radius`
    /// is negative or non-finite.
    #[must_use]
    pub fn new(
        name: impl Into<String>,
        waypoints: Vec<TendonWaypoint>,
        channel_radius: f64,
    ) -> Self {
        let name = name.into();
        assert!(!name.is_empty(), "tendon name must not be empty");
        assert!(
            waypoints.len() >= 2,
            "tendon requires at least 2 waypoints, got {}",
            waypoints.len()
        );
        assert!(
            channel_radius >= 0.0 && channel_radius.is_finite(),
            "tendon channel_radius must be non-negative and finite, got {channel_radius}"
        );

        Self {
            name,
            waypoints,
            channel_radius,
            stiffness: None,
            damping: None,
        }
    }

    /// Set tendon stiffness (for MJCF generation).
    ///
    /// # Panics
    ///
    /// Panics if `stiffness` is not positive or not finite.
    #[must_use]
    pub fn with_stiffness(mut self, stiffness: f64) -> Self {
        assert!(
            stiffness > 0.0 && stiffness.is_finite(),
            "tendon stiffness must be positive and finite, got {stiffness}"
        );
        self.stiffness = Some(stiffness);
        self
    }

    /// Set tendon damping (for MJCF generation).
    ///
    /// # Panics
    ///
    /// Panics if `damping` is not positive or not finite.
    #[must_use]
    pub fn with_damping(mut self, damping: f64) -> Self {
        assert!(
            damping > 0.0 && damping.is_finite(),
            "tendon damping must be positive and finite, got {damping}"
        );
        self.damping = Some(damping);
        self
    }

    /// Tendon name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Ordered waypoints defining the tendon path.
    #[must_use]
    pub fn waypoints(&self) -> &[TendonWaypoint] {
        &self.waypoints
    }

    /// Channel radius (0.0 = surface-routed, no channel carved).
    #[must_use]
    pub const fn channel_radius(&self) -> f64 {
        self.channel_radius
    }

    /// Tendon stiffness, if set.
    #[must_use]
    pub const fn stiffness(&self) -> Option<f64> {
        self.stiffness
    }

    /// Tendon damping, if set.
    #[must_use]
    pub const fn damping(&self) -> Option<f64> {
        self.damping
    }

    /// Build the channel void solid for a specific part.
    ///
    /// Collects waypoints belonging to `part_name` (preserving order),
    /// and returns a [`Solid::pipe`] with `channel_radius`. Returns `None`
    /// if the tendon is surface-routed (`channel_radius == 0.0`), the part
    /// has no waypoints, or fewer than 2 waypoints exist in this part.
    #[must_use]
    pub fn channel_solid(&self, part_name: &str) -> Option<Solid> {
        if self.channel_radius == 0.0 {
            return None;
        }

        let positions: Vec<Point3<f64>> = self
            .waypoints
            .iter()
            .filter(|wp| wp.part() == part_name)
            .map(|wp| *wp.position())
            .collect();

        if positions.len() < 2 {
            return None;
        }

        Some(Solid::pipe(positions, self.channel_radius))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_waypoints() -> Vec<TendonWaypoint> {
        vec![
            TendonWaypoint::new("palm", Point3::new(8.0, -10.0, 0.0)),
            TendonWaypoint::new("palm", Point3::new(8.0, 18.0, 0.0)),
            TendonWaypoint::new("finger", Point3::new(0.0, 5.0, 0.0)),
            TendonWaypoint::new("finger", Point3::new(0.0, 20.0, 0.0)),
        ]
    }

    // ── TendonWaypoint ───────────────────────────────────────────────

    #[test]
    fn waypoint_new_valid() {
        let wp = TendonWaypoint::new("palm", Point3::new(1.0, 2.0, 3.0));
        assert_eq!(wp.part(), "palm");
        assert_eq!(*wp.position(), Point3::new(1.0, 2.0, 3.0));
    }

    #[test]
    #[should_panic(expected = "waypoint part name must not be empty")]
    fn waypoint_rejects_empty_part() {
        let _wp = TendonWaypoint::new("", Point3::origin());
    }

    #[test]
    #[should_panic(expected = "waypoint position must have finite coordinates")]
    fn waypoint_rejects_nan_position() {
        let _wp = TendonWaypoint::new("part", Point3::new(f64::NAN, 0.0, 0.0));
    }

    #[test]
    #[should_panic(expected = "waypoint position must have finite coordinates")]
    fn waypoint_rejects_inf_position() {
        let _wp = TendonWaypoint::new("part", Point3::new(0.0, f64::INFINITY, 0.0));
    }

    // ── TendonDef construction ───────────────────────────────────────

    #[test]
    fn tendon_new_valid() {
        let t = TendonDef::new("flexor", make_waypoints(), 1.5);
        assert_eq!(t.name(), "flexor");
        assert_eq!(t.waypoints().len(), 4);
        assert!((t.channel_radius() - 1.5).abs() < f64::EPSILON);
        assert!(t.stiffness().is_none());
        assert!(t.damping().is_none());
    }

    #[test]
    fn tendon_zero_radius_is_surface_routed() {
        let t = TendonDef::new("surface", make_waypoints(), 0.0);
        assert!((t.channel_radius()) < f64::EPSILON);
    }

    #[test]
    #[should_panic(expected = "tendon name must not be empty")]
    fn tendon_rejects_empty_name() {
        let _t = TendonDef::new("", make_waypoints(), 1.0);
    }

    #[test]
    #[should_panic(expected = "tendon requires at least 2 waypoints")]
    fn tendon_rejects_too_few_waypoints() {
        let _t = TendonDef::new("bad", vec![TendonWaypoint::new("a", Point3::origin())], 1.0);
    }

    #[test]
    #[should_panic(expected = "channel_radius must be non-negative")]
    fn tendon_rejects_negative_radius() {
        let _t = TendonDef::new("bad", make_waypoints(), -0.5);
    }

    #[test]
    #[should_panic(expected = "channel_radius must be non-negative and finite")]
    fn tendon_rejects_nan_radius() {
        let _t = TendonDef::new("bad", make_waypoints(), f64::NAN);
    }

    // ── Stiffness / damping builders ─────────────────────────────────

    #[test]
    fn tendon_with_stiffness_damping() {
        let t = TendonDef::new("t", make_waypoints(), 1.0)
            .with_stiffness(100.0)
            .with_damping(5.0);
        assert_eq!(t.stiffness(), Some(100.0));
        assert_eq!(t.damping(), Some(5.0));
    }

    #[test]
    #[should_panic(expected = "tendon stiffness must be positive")]
    fn tendon_rejects_non_positive_stiffness() {
        let _t = TendonDef::new("t", make_waypoints(), 1.0).with_stiffness(0.0);
    }

    #[test]
    #[should_panic(expected = "tendon damping must be positive")]
    fn tendon_rejects_non_positive_damping() {
        let _t = TendonDef::new("t", make_waypoints(), 1.0).with_damping(-1.0);
    }

    // ── Channel solid generation ─────────────────────────────────────

    #[test]
    fn channel_solid_returns_none_for_zero_radius() {
        let t = TendonDef::new("surface", make_waypoints(), 0.0);
        assert!(t.channel_solid("palm").is_none());
    }

    #[test]
    fn channel_solid_returns_none_for_unknown_part() {
        let t = TendonDef::new("t", make_waypoints(), 1.5);
        assert!(t.channel_solid("nonexistent").is_none());
    }

    #[test]
    fn channel_solid_returns_none_for_single_waypoint() {
        // Only one waypoint in "finger_tip" — not enough for a pipe.
        let wps = vec![
            TendonWaypoint::new("palm", Point3::new(0.0, 0.0, 0.0)),
            TendonWaypoint::new("finger_tip", Point3::new(0.0, 5.0, 0.0)),
        ];
        let t = TendonDef::new("t", wps, 1.0);
        assert!(t.channel_solid("finger_tip").is_none());
    }

    #[test]
    fn channel_solid_returns_pipe_for_valid_part() {
        let t = TendonDef::new("flexor", make_waypoints(), 1.5);
        let channel = t.channel_solid("palm");
        assert!(
            channel.is_some(),
            "channel_solid should return Some for valid part"
        );

        // The channel should be a pipe — evaluate at a point on the centerline.
        // The pipe centerline runs from (8, -10, 0) to (8, 18, 0).
        // A point on the centerline should be inside the pipe (negative field).
        if let Some(pipe) = channel {
            let on_centerline = Point3::new(8.0, 0.0, 0.0);
            let val = pipe.evaluate(&on_centerline);
            assert!(
                val < 0.0,
                "point on pipe centerline should be inside (negative), got {val}"
            );
        }
    }

    #[test]
    fn channel_along_centerline_is_void_after_subtraction() {
        // Create a cuboid part, subtract a tendon channel, verify the channel
        // centerline is now void (positive field = outside the solid).
        let solid = Solid::cuboid(nalgebra::Vector3::new(20.0, 30.0, 10.0));
        let material = crate::Material::new("PLA", 1250.0);
        let part = crate::Part::new("palm", solid, material);

        let tendon = TendonDef::new(
            "flexor",
            vec![
                TendonWaypoint::new("palm", Point3::new(8.0, -10.0, 0.0)),
                TendonWaypoint::new("palm", Point3::new(8.0, 18.0, 0.0)),
            ],
            1.5,
        );

        let part = part.with_tendon_channels(&[tendon]);

        // Points along the channel centerline should now be outside (positive).
        for y in [-5.0, 0.0, 5.0, 10.0, 15.0] {
            let p = Point3::new(8.0, y, 0.0);
            let val = part.solid().evaluate(&p);
            assert!(
                val > 0.0,
                "channel centerline at y={y} should be void (positive), got {val}"
            );
        }

        // A point far from the channel should still be inside the cuboid.
        let inside = Point3::new(-5.0, 0.0, 0.0);
        let val = part.solid().evaluate(&inside);
        assert!(
            val < 0.0,
            "point away from channel should still be inside, got {val}"
        );
    }

    #[test]
    fn multiple_tendons_independent_channels() {
        let solid = Solid::cuboid(nalgebra::Vector3::new(20.0, 30.0, 10.0));
        let material = crate::Material::new("PLA", 1250.0);
        let part = crate::Part::new("palm", solid, material);

        let tendon_a = TendonDef::new(
            "flexor_1",
            vec![
                TendonWaypoint::new("palm", Point3::new(8.0, -10.0, 0.0)),
                TendonWaypoint::new("palm", Point3::new(8.0, 18.0, 0.0)),
            ],
            1.0,
        );
        let tendon_b = TendonDef::new(
            "flexor_2",
            vec![
                TendonWaypoint::new("palm", Point3::new(-8.0, -10.0, 0.0)),
                TendonWaypoint::new("palm", Point3::new(-8.0, 18.0, 0.0)),
            ],
            1.0,
        );

        let part = part.with_tendon_channels(&[tendon_a, tendon_b]);

        // Both channels should be void.
        let val_a = part.solid().evaluate(&Point3::new(8.0, 0.0, 0.0));
        let val_b = part.solid().evaluate(&Point3::new(-8.0, 0.0, 0.0));
        assert!(
            val_a > 0.0,
            "channel A centerline should be void, got {val_a}"
        );
        assert!(
            val_b > 0.0,
            "channel B centerline should be void, got {val_b}"
        );

        // Point between channels should still be solid.
        let between = part.solid().evaluate(&Point3::new(0.0, 0.0, 0.0));
        assert!(
            between < 0.0,
            "point between channels should be solid, got {between}"
        );
    }

    #[test]
    fn with_tendon_channels_skips_irrelevant_tendons() {
        let solid = Solid::cuboid(nalgebra::Vector3::new(10.0, 10.0, 10.0));
        let material = crate::Material::new("PLA", 1250.0);
        let part = crate::Part::new("palm", solid, material);

        // This tendon has no waypoints in "palm".
        let tendon = TendonDef::new(
            "other",
            vec![
                TendonWaypoint::new("finger", Point3::new(0.0, 0.0, 0.0)),
                TendonWaypoint::new("finger", Point3::new(0.0, 10.0, 0.0)),
            ],
            1.0,
        );

        let part = part.with_tendon_channels(&[tendon]);

        // Origin should still be inside — no channel was carved.
        let val = part.solid().evaluate(&Point3::origin());
        assert!(val < 0.0, "origin should still be inside, got {val}");
    }
}
