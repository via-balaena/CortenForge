//! Camera controllers for physics visualization.
//!
//! Provides orbit camera functionality for inspecting physics scenes.

use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;

/// Orbit camera controller.
///
/// Provides mouse-based orbit, pan, and zoom controls for inspecting
/// physics simulations.
///
/// # Controls
///
/// - **Left mouse drag**: Orbit around target
/// - **Right mouse drag**: Pan camera
/// - **Scroll wheel**: Zoom in/out
#[derive(Component, Debug, Clone)]
pub struct OrbitCamera {
    /// Target point to orbit around.
    pub target: Vec3,
    /// Distance from target.
    pub distance: f32,
    /// Horizontal angle (radians).
    pub azimuth: f32,
    /// Vertical angle (radians). Clamped to avoid gimbal lock.
    pub elevation: f32,
    /// Orbit speed (radians per pixel).
    pub orbit_speed: f32,
    /// Pan speed (units per pixel).
    pub pan_speed: f32,
    /// Zoom speed (multiplier per scroll unit).
    pub zoom_speed: f32,
    /// Minimum distance from target.
    pub min_distance: f32,
    /// Maximum distance from target.
    pub max_distance: f32,
    /// Minimum elevation angle (radians).
    pub min_elevation: f32,
    /// Maximum elevation angle (radians).
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
            pan_speed: 0.01,
            zoom_speed: 0.1,
            min_distance: 0.5,
            max_distance: 100.0,
            min_elevation: -1.4, // ~-80 degrees
            max_elevation: 1.4,  // ~80 degrees
        }
    }
}

impl OrbitCamera {
    /// Create a new orbit camera with default settings.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the target point.
    #[must_use]
    pub fn with_target(mut self, target: Vec3) -> Self {
        self.target = target;
        self
    }

    /// Set the initial distance.
    #[must_use]
    pub fn with_distance(mut self, distance: f32) -> Self {
        self.distance = distance.clamp(self.min_distance, self.max_distance);
        self
    }

    /// Set the initial angles.
    #[must_use]
    pub fn with_angles(mut self, azimuth: f32, elevation: f32) -> Self {
        self.azimuth = azimuth;
        self.elevation = elevation.clamp(self.min_elevation, self.max_elevation);
        self
    }

    /// Calculate the camera position from current orbit parameters.
    #[must_use]
    pub fn position(&self) -> Vec3 {
        let x = self.distance * self.azimuth.cos() * self.elevation.cos();
        let y = self.distance * self.elevation.sin();
        let z = self.distance * self.azimuth.sin() * self.elevation.cos();
        self.target + Vec3::new(x, y, z)
    }

    /// Apply orbit input (mouse delta while dragging).
    pub fn orbit(&mut self, delta: Vec2) {
        self.azimuth -= delta.x * self.orbit_speed;
        self.elevation += delta.y * self.orbit_speed;
        self.elevation = self.elevation.clamp(self.min_elevation, self.max_elevation);
    }

    /// Apply pan input (mouse delta while right-dragging).
    pub fn pan(&mut self, delta: Vec2) {
        // Calculate right and up vectors in world space
        let forward = (self.target - self.position()).normalize();
        let right = forward.cross(Vec3::Y).normalize();
        let up = right.cross(forward);

        // Apply pan
        let pan_offset = right * (-delta.x * self.pan_speed * self.distance)
            + up * (delta.y * self.pan_speed * self.distance);
        self.target += pan_offset;
    }

    /// Apply zoom input (scroll delta).
    pub fn zoom(&mut self, delta: f32) {
        self.distance *= 1.0 - delta * self.zoom_speed;
        self.distance = self.distance.clamp(self.min_distance, self.max_distance);
    }

    /// Apply this controller's state to a Bevy Transform.
    pub fn apply_to_transform(&self, transform: &mut Transform) {
        transform.translation = self.position();
        transform.look_at(self.target, Vec3::Y);
    }
}

/// Plugin for orbit camera functionality.
pub struct OrbitCameraPlugin;

impl Plugin for OrbitCameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, (orbit_camera_input, update_orbit_camera));
    }
}

/// Handle mouse input for orbit camera.
#[allow(clippy::needless_pass_by_value)] // Bevy system parameters are passed by value
fn orbit_camera_input(
    mut cameras: Query<&mut OrbitCamera>,
    mouse_button: Res<ButtonInput<MouseButton>>,
    mouse_motion: Res<AccumulatedMouseMotion>,
    mouse_scroll: Res<AccumulatedMouseScroll>,
) {
    let delta = mouse_motion.delta;
    let scroll = mouse_scroll.delta.y;

    for mut camera in &mut cameras {
        // Orbit with left mouse button
        if mouse_button.pressed(MouseButton::Left) {
            camera.orbit(delta);
        }

        // Pan with right mouse button
        if mouse_button.pressed(MouseButton::Right) {
            camera.pan(delta);
        }

        // Zoom with scroll wheel
        if scroll.abs() > 0.001 {
            camera.zoom(scroll);
        }
    }
}

/// Update camera transform from orbit parameters.
fn update_orbit_camera(mut cameras: Query<(&OrbitCamera, &mut Transform)>) {
    for (camera, mut transform) in &mut cameras {
        camera.apply_to_transform(&mut transform);
    }
}

/// Spawn an orbit camera with default settings.
///
/// This is a convenience function for setting up a basic camera.
pub fn spawn_orbit_camera(mut commands: Commands) {
    let camera = OrbitCamera::default()
        .with_target(Vec3::ZERO)
        .with_distance(10.0)
        .with_angles(0.5, 0.5);

    let mut transform = Transform::default();
    camera.apply_to_transform(&mut transform);

    commands.spawn((Camera3d::default(), camera, transform));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orbit_camera_position_at_zero_angles() {
        let camera = OrbitCamera {
            target: Vec3::ZERO,
            distance: 5.0,
            azimuth: 0.0,
            elevation: 0.0,
            ..Default::default()
        };

        let pos = camera.position();
        // At azimuth=0, elevation=0, camera should be at (distance, 0, 0)
        assert!((pos.x - 5.0).abs() < 0.001);
        assert!(pos.y.abs() < 0.001);
        assert!(pos.z.abs() < 0.001);
    }

    #[test]
    fn orbit_camera_zoom_clamps() {
        let mut camera = OrbitCamera {
            min_distance: 1.0,
            max_distance: 10.0,
            distance: 5.0,
            ..Default::default()
        };

        // Zoom in a lot
        camera.zoom(100.0);
        assert!(camera.distance >= camera.min_distance);

        // Zoom out a lot
        camera.distance = 5.0;
        camera.zoom(-100.0);
        assert!(camera.distance <= camera.max_distance);
    }

    #[test]
    fn orbit_camera_elevation_clamps() {
        let mut camera = OrbitCamera::default();

        // Try to orbit past limits
        camera.orbit(Vec2::new(0.0, 10000.0));
        assert!(camera.elevation <= camera.max_elevation);

        camera.orbit(Vec2::new(0.0, -20000.0));
        assert!(camera.elevation >= camera.min_elevation);
    }
}
