//! Mouse-driven orbit camera controller.
//!
//! Pure inline implementation (per `docs/VIEWER_DESIGN.md` iter-2 still-open
//! #1 — no `sim-bevy` dep). Copy-adapted from `sim/L1/bevy/src/camera.rs`
//! with workspace-local additions: AABB-aware initial framing
//! ([`OrbitCamera::framing_for_aabb`]) preserves commit-3's
//! "1.5 × diagonal corner-on" view as the startup pose.
//!
//! Spherical-coordinates state:
//! ```text
//! x = distance · cos(azimuth) · cos(elevation)
//! y = distance · sin(elevation)
//! z = distance · sin(azimuth) · cos(elevation)
//! ```
//! Bevy is internally Y-up; [`mesh::vertex_position`] (parameterized by
//! [`UpAxis`]) swaps the input frame to Bevy-Y-up before the camera sees
//! it, and [`OrbitCamera::framing_for_aabb`] applies the same swap to the
//! AABB center so the orbit pose is in the Bevy frame end-to-end.
//!
//! [`mesh::vertex_position`]: crate::mesh::vertex_position
//!
//! # Controls
//!
//! - **Left mouse drag**: orbit azimuth/elevation around the focal point
//! - **Right mouse drag**: pan the focal point parallel to the screen
//! - **Scroll wheel**: zoom (multiplicative on `distance`)
//!
//! Speeds + clamp ranges live on [`OrbitCamera`] so a future commit can
//! expose them via CLI without touching the systems.

#![allow(clippy::cast_precision_loss)] // f64 → f32 framing math; AABB extents are well within f32

use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;
use mesh_types::Aabb;

use crate::UpAxis;
use crate::mesh::vertex_position;

/// Orbit camera component carried by the `Camera3d` entity.
#[derive(Component, Debug, Clone)]
pub struct OrbitCamera {
    /// Focal point the camera orbits around.
    pub target: Vec3,
    /// Distance from `target` to the camera position.
    pub distance: f32,
    /// Horizontal angle around the Y axis (radians).
    pub azimuth: f32,
    /// Vertical angle from the XZ plane (radians); clamped against gimbal lock.
    pub elevation: f32,
    /// Orbit speed in radians per pixel of mouse motion.
    pub orbit_speed: f32,
    /// Pan speed in world units per pixel of mouse motion (scaled by distance).
    pub pan_speed: f32,
    /// Zoom factor per scroll-unit (multiplicative).
    pub zoom_speed: f32,
    /// Lower clamp for `distance`.
    pub min_distance: f32,
    /// Upper clamp for `distance`.
    pub max_distance: f32,
    /// Lower clamp for `elevation` (radians; ~ -80°).
    pub min_elevation: f32,
    /// Upper clamp for `elevation` (radians; ~ +80°).
    pub max_elevation: f32,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            target: Vec3::ZERO,
            distance: 5.0,
            azimuth: 0.0,
            elevation: 0.5,
            orbit_speed: 0.005,
            pan_speed: 0.002,
            zoom_speed: 0.1,
            min_distance: 0.1,
            max_distance: 1_000.0,
            min_elevation: -1.4,
            max_elevation: 1.4,
        }
    }
}

impl OrbitCamera {
    /// Frame the input AABB the same way the static placeholder did in
    /// commit 3: camera offset by `(d, d, d)` (where `d = max(diagonal,
    /// 1.0) · 1.5`) from the AABB center, looking at the center.
    ///
    /// Per the spherical convention, `(d, d, d)` corresponds to:
    /// - `azimuth = π/4`
    /// - `elevation = asin(1/√3) ≈ 0.6155 rad ≈ 35.26°`
    /// - `distance = d · √3`
    ///
    /// The single-point degenerate case (zero diagonal) is handled by the
    /// `max(_, 1.0)` clamp on the diagonal — same floor as the static
    /// placeholder used.
    ///
    /// `up` parameterizes the input → Bevy frame swap on the AABB center
    /// (commit 6) so the camera target stays aligned with the rendered
    /// geometry under any `--up=<+X|+Y|+Z>` choice.
    #[must_use]
    pub fn framing_for_aabb(aabb: &Aabb, up: UpAxis) -> Self {
        let center_physics = aabb.center();
        // Mirror `mesh::vertex_position`'s axis swap on the AABB center so
        // the camera target stays aligned with the rendered geometry.
        let target = Vec3::from_array(vertex_position(&center_physics, up));
        let diagonal = (aabb.diagonal() as f32).max(1.0);
        let d = diagonal * 1.5;

        let max_distance = (d * 10.0).max(1_000.0);
        Self {
            target,
            distance: d * 3.0_f32.sqrt(),
            azimuth: std::f32::consts::FRAC_PI_4,
            elevation: (1.0_f32 / 3.0_f32.sqrt()).asin(),
            min_distance: (d * 0.01).max(0.01),
            max_distance,
            ..Self::default()
        }
    }

    /// Camera position derived from `(target, distance, azimuth, elevation)`.
    #[must_use]
    pub fn position(&self) -> Vec3 {
        let x = self.distance * self.azimuth.cos() * self.elevation.cos();
        let y = self.distance * self.elevation.sin();
        let z = self.distance * self.azimuth.sin() * self.elevation.cos();
        self.target + Vec3::new(x, y, z)
    }

    /// Apply orbit input (left-drag mouse delta in pixels).
    pub fn orbit(&mut self, delta: Vec2) {
        self.azimuth -= delta.x * self.orbit_speed;
        self.elevation += delta.y * self.orbit_speed;
        self.elevation = self.elevation.clamp(self.min_elevation, self.max_elevation);
    }

    /// Apply pan input (right-drag mouse delta in pixels). Pan magnitude
    /// scales with `distance` so the apparent on-screen drag rate stays
    /// roughly constant when zoomed in or out.
    pub fn pan(&mut self, delta: Vec2) {
        let forward = (self.target - self.position()).normalize_or_zero();
        if forward == Vec3::ZERO {
            return;
        }
        let right = forward.cross(Vec3::Y).normalize_or_zero();
        if right == Vec3::ZERO {
            return;
        }
        let up = right.cross(forward);
        let scale = self.pan_speed * self.distance;
        self.target += right * (-delta.x * scale) + up * (delta.y * scale);
    }

    /// Apply zoom input (scroll-wheel delta). Multiplicative so each
    /// scroll unit covers a fixed *fraction* of the current distance.
    pub fn zoom(&mut self, delta: f32) {
        self.distance *= 1.0 - delta * self.zoom_speed;
        self.distance = self.distance.clamp(self.min_distance, self.max_distance);
    }

    /// Write `position` and `look_at(target, +Y)` into a `Transform`.
    pub fn apply_to_transform(&self, transform: &mut Transform) {
        transform.translation = self.position();
        transform.look_at(self.target, Vec3::Y);
    }
}

/// Mouse-input system: read accumulated motion + scroll, feed the orbit
/// camera. Runs every `Update` tick.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value
pub fn orbit_camera_input(
    mut cameras: Query<&mut OrbitCamera>,
    mouse_button: Res<ButtonInput<MouseButton>>,
    mouse_motion: Res<AccumulatedMouseMotion>,
    mouse_scroll: Res<AccumulatedMouseScroll>,
) {
    let delta = mouse_motion.delta;
    let scroll = mouse_scroll.delta.y;

    for mut camera in &mut cameras {
        if mouse_button.pressed(MouseButton::Left) {
            camera.orbit(delta);
        }
        if mouse_button.pressed(MouseButton::Right) {
            camera.pan(delta);
        }
        if scroll.abs() > 1e-3 {
            camera.zoom(scroll);
        }
    }
}

/// Transform-update system: project [`OrbitCamera`] state onto the entity's
/// `Transform`. Runs after [`orbit_camera_input`] each frame.
pub fn update_orbit_camera(mut cameras: Query<(&OrbitCamera, &mut Transform)>) {
    for (camera, mut transform) in &mut cameras {
        camera.apply_to_transform(&mut transform);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Point3;

    /// Position at azimuth=0, elevation=0 sits on the +X axis at distance.
    #[test]
    fn position_at_zero_angles_is_plus_x() {
        let cam = OrbitCamera {
            distance: 5.0,
            azimuth: 0.0,
            elevation: 0.0,
            ..OrbitCamera::default()
        };
        let p = cam.position();
        assert!((p.x - 5.0).abs() < 1e-5, "x = {}", p.x);
        assert!(p.y.abs() < 1e-5, "y = {}", p.y);
        assert!(p.z.abs() < 1e-5, "z = {}", p.z);
    }

    /// `framing_for_aabb` reproduces the static-placeholder corner-on view:
    /// camera offset = `(d, d, d)` from center where `d = diagonal · 1.5`.
    #[test]
    fn framing_for_aabb_matches_static_placeholder_corner_on() {
        // 2-unit cube centered at origin (Z-up physics frame).
        let aabb = Aabb::from_corners(Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0));
        let cam = OrbitCamera::framing_for_aabb(&aabb, UpAxis::PlusZ);

        // Diagonal = 2√3; 1.5 × diagonal = 3√3 ≈ 5.196; corner-on offset
        // d = (diagonal · 1.5) = 3√3, so each component = 3√3.
        let d = 2.0_f32 * 3.0_f32.sqrt() * 1.5;
        let p = cam.position();
        assert!((p.x - d).abs() < 1e-4, "x = {} expected {}", p.x, d);
        assert!((p.y - d).abs() < 1e-4, "y = {} expected {}", p.y, d);
        assert!((p.z - d).abs() < 1e-4, "z = {} expected {}", p.z, d);
        assert_eq!(cam.target, Vec3::ZERO);
    }

    /// Degenerate (single-point) AABB framed via the diagonal floor.
    #[test]
    fn framing_for_aabb_handles_degenerate_diagonal() {
        let aabb = Aabb::from_corners(Point3::new(2.0, 2.0, 2.0), Point3::new(2.0, 2.0, 2.0));
        let cam = OrbitCamera::framing_for_aabb(&aabb, UpAxis::PlusZ);
        // diagonal = 0 → clamped to 1.0; d = 1.5; distance = 1.5√3
        let expected_distance = 1.5 * 3.0_f32.sqrt();
        assert!(
            (cam.distance - expected_distance).abs() < 1e-4,
            "distance = {} expected {}",
            cam.distance,
            expected_distance
        );
        // target = AABB center swapped Z↔Y; for (2,2,2) that's still (2,2,2).
        assert_eq!(cam.target, Vec3::new(2.0, 2.0, 2.0));
    }

    /// Z-up → Y-up swap applied to AABB center: physics y becomes Bevy z.
    #[test]
    fn framing_for_aabb_plus_z_swaps_y_and_z_on_target() {
        // Single point at (1, 5, 9) physics frame → target (1, 9, 5) Bevy frame.
        let aabb = Aabb::from_corners(Point3::new(1.0, 5.0, 9.0), Point3::new(1.0, 5.0, 9.0));
        let cam = OrbitCamera::framing_for_aabb(&aabb, UpAxis::PlusZ);
        assert_eq!(cam.target, Vec3::new(1.0, 9.0, 5.0));
    }

    /// `+Y` (input is already Bevy-Y-up) leaves the AABB center unchanged.
    #[test]
    fn framing_for_aabb_plus_y_target_is_identity() {
        let aabb = Aabb::from_corners(Point3::new(1.0, 5.0, 9.0), Point3::new(1.0, 5.0, 9.0));
        let cam = OrbitCamera::framing_for_aabb(&aabb, UpAxis::PlusY);
        assert_eq!(cam.target, Vec3::new(1.0, 5.0, 9.0));
    }

    /// `+X` swaps X↔Y on the AABB center.
    #[test]
    fn framing_for_aabb_plus_x_swaps_x_and_y_on_target() {
        let aabb = Aabb::from_corners(Point3::new(1.0, 5.0, 9.0), Point3::new(1.0, 5.0, 9.0));
        let cam = OrbitCamera::framing_for_aabb(&aabb, UpAxis::PlusX);
        assert_eq!(cam.target, Vec3::new(5.0, 1.0, 9.0));
    }

    /// Zoom is multiplicative + clamped to `[min_distance, max_distance]`.
    #[test]
    fn zoom_clamps_to_distance_bounds() {
        let mut cam = OrbitCamera {
            distance: 5.0,
            min_distance: 1.0,
            max_distance: 10.0,
            zoom_speed: 0.1,
            ..OrbitCamera::default()
        };
        // Aggressive zoom-in: distance saturates at min_distance.
        cam.zoom(100.0);
        assert!((cam.distance - cam.min_distance).abs() < 1e-5);
        cam.distance = 5.0;
        // Aggressive zoom-out: distance saturates at max_distance.
        cam.zoom(-100.0);
        assert!((cam.distance - cam.max_distance).abs() < 1e-5);
    }

    /// Elevation is clamped to `[min_elevation, max_elevation]` after orbit.
    #[test]
    fn elevation_clamps_under_orbit_input() {
        let mut cam = OrbitCamera::default();
        cam.orbit(Vec2::new(0.0, 1.0e6));
        assert!(cam.elevation <= cam.max_elevation + 1e-5);
        cam.orbit(Vec2::new(0.0, -1.0e6));
        assert!(cam.elevation >= cam.min_elevation - 1e-5);
    }

    /// Pan moves the target; magnitude scales with distance so the
    /// apparent screen-drag stays roughly constant under zoom.
    #[test]
    fn pan_translates_target_perpendicular_to_view() {
        let mut cam = OrbitCamera {
            distance: 4.0,
            azimuth: 0.0,
            elevation: 0.0,
            target: Vec3::ZERO,
            pan_speed: 0.01,
            ..OrbitCamera::default()
        };
        // From (4, 0, 0) looking at origin: right vector = forward × Y =
        // (-1, 0, 0) × (0, 1, 0) = (0, 0, -1). Pan delta x positive →
        // target += right · (-dx · scale).
        cam.pan(Vec2::new(10.0, 0.0));
        assert!(cam.target.x.abs() < 1e-5);
        assert!(cam.target.y.abs() < 1e-5);
        // Non-zero z means the target moved along the screen-x axis.
        assert!(cam.target.z.abs() > 1e-6);
    }
}
