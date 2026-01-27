//! Debug visualization using Bevy gizmos.
//!
//! This module provides real-time debug visualization for:
//! - Contact points and normals
//! - Muscle and tendon paths
//! - Sensor visualization
//!
//! All visualization is optional and controlled via [`ViewerConfig`].
//!
//! # Performance
//!
//! Contact-related gizmos (points and normals) read from the [`CachedContacts`]
//! resource rather than re-detecting contacts. This avoids:
//! - Cloning the entire physics world every frame
//! - Redundant O(n²) contact detection
//!
//! The cache is updated by [`update_cached_contacts`](crate::systems::update_cached_contacts)
//! which runs in [`SimViewerSet::ContactCache`](crate::systems::SimViewerSet::ContactCache)
//! after transform sync but before gizmo drawing.

#![allow(clippy::needless_pass_by_value)] // Bevy system parameters
#![allow(clippy::cast_possible_truncation)] // f64 -> f32 is intentional for Bevy
#![allow(clippy::cast_precision_loss)] // Small int -> f32 is fine for arc segments

use bevy::prelude::*;

use crate::convert::{quat_from_unit_quaternion, vec3_from_point, vec3_from_vector};
use crate::resources::{
    CachedContacts, MuscleVisualization, SensorVisualType, SensorVisualization,
    TendonVisualization, ViewerConfig,
};

/// System set for debug visualization.
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub struct DebugGizmosSet;

/// Draw contact point markers as small spheres.
///
/// Reads from [`CachedContacts`] resource, which should be updated after each
/// physics step. This avoids the performance cost of re-detecting contacts
/// or cloning the physics world.
pub fn draw_contact_points(
    mut gizmos: Gizmos,
    cached_contacts: Res<CachedContacts>,
    config: Res<ViewerConfig>,
) {
    if !config.show_contacts {
        return;
    }

    for contact in cached_contacts.contacts() {
        let pos = vec3_from_point(&contact.position);

        // Draw contact point as a small sphere
        gizmos.sphere(
            Isometry3d::from_translation(pos),
            config.contact_marker_radius,
            config.colors.contact_point,
        );
    }
}

/// Draw contact normal arrows.
///
/// Reads from [`CachedContacts`] resource, which should be updated after each
/// physics step.
pub fn draw_contact_normals(
    mut gizmos: Gizmos,
    cached_contacts: Res<CachedContacts>,
    config: Res<ViewerConfig>,
) {
    if !config.show_contact_normals {
        return;
    }

    for contact in cached_contacts.contacts() {
        let start = vec3_from_point(&contact.position);
        let normal = vec3_from_vector(&contact.normal);
        let end = start + normal * config.contact_normal_length;

        // Draw arrow from contact point along normal direction
        gizmos.arrow(start, end, config.colors.contact_normal);
    }
}

// ============================================================================
// Musculoskeletal Visualization
// ============================================================================

/// Draw muscle paths with activation-based coloring.
///
/// Muscles are drawn as lines from origin to insertion, passing through
/// via points. Color interpolates from relaxed (blue) to activated (red).
pub fn draw_muscles(
    mut gizmos: Gizmos,
    muscles: Res<MuscleVisualization>,
    config: Res<ViewerConfig>,
) {
    if !config.show_muscles {
        return;
    }

    for muscle in muscles.muscles() {
        // Interpolate color based on activation
        let activation = muscle.activation.clamp(0.0, 1.0) as f32;
        let color = lerp_color(
            config.colors.muscle_relaxed,
            config.colors.muscle_activated,
            activation,
        );

        // Build path: origin -> via_points -> insertion
        let mut points = Vec::with_capacity(muscle.via_points.len() + 2);
        points.push(vec3_from_point(&muscle.origin));
        for vp in &muscle.via_points {
            points.push(vec3_from_point(vp));
        }
        points.push(vec3_from_point(&muscle.insertion));

        // Draw line segments along path
        for i in 0..points.len().saturating_sub(1) {
            gizmos.line(points[i], points[i + 1], color);
        }

        // Draw small spheres at attachment points
        gizmos.sphere(
            Isometry3d::from_translation(points[0]),
            config.muscle_line_radius * 1.5,
            color,
        );
        if let Some(&last) = points.last() {
            gizmos.sphere(
                Isometry3d::from_translation(last),
                config.muscle_line_radius * 1.5,
                color,
            );
        }

        // Draw force vector if available
        if let Some(force) = muscle.force {
            if force.abs() > 0.01 {
                // Draw force along muscle line direction
                let dir = (muscle.insertion - muscle.origin).normalize();
                let center = vec3_from_point(&nalgebra::center(&muscle.origin, &muscle.insertion));
                let force_vec = vec3_from_vector(&(dir * force * 0.01)); // Scale for visibility
                gizmos.arrow(center, center + force_vec, config.colors.force_vector);
            }
        }
    }
}

/// Draw tendon paths.
///
/// Tendons are drawn as cyan lines along their path. Thickness/opacity
/// can vary based on tension.
pub fn draw_tendons(
    mut gizmos: Gizmos,
    tendons: Res<TendonVisualization>,
    config: Res<ViewerConfig>,
) {
    if !config.show_tendons {
        return;
    }

    for tendon in tendons.tendons() {
        if tendon.path.len() < 2 {
            continue;
        }

        // Color intensity based on tension (higher tension = more opaque)
        let tension_factor = (tendon.tension / 100.0).clamp(0.0, 1.0) as f32; // Normalize to ~100N max
        let alpha = 0.5 + tension_factor * 0.5;
        let color = config.colors.tendon.with_alpha(alpha);

        // Draw line segments along tendon path
        for i in 0..tendon.path.len().saturating_sub(1) {
            let start = vec3_from_point(&tendon.path[i]);
            let end = vec3_from_point(&tendon.path[i + 1]);
            gizmos.line(start, end, color);
        }

        // Draw small markers at attachment/wrap points
        for point in &tendon.path {
            gizmos.sphere(
                Isometry3d::from_translation(vec3_from_point(point)),
                config.tendon_line_radius,
                color,
            );
        }
    }
}

/// Draw sensor visualization.
///
/// Different sensor types are visualized differently:
/// - IMU: Coordinate frame (RGB axes)
/// - Force/Torque: Force and torque arrows
/// - Touch: Highlight sphere when active
/// - Rangefinder: Ray with hit point
/// - Magnetometer: Field direction arrow
pub fn draw_sensors(
    mut gizmos: Gizmos,
    sensors: Res<SensorVisualization>,
    config: Res<ViewerConfig>,
) {
    if !config.show_sensors {
        return;
    }

    for sensor in sensors.sensors() {
        let pos = vec3_from_point(&sensor.position);

        match sensor.sensor_type {
            SensorVisualType::Imu => {
                // Draw coordinate frame (RGB = XYZ)
                let rot = quat_from_unit_quaternion(&sensor.orientation);
                let axis_len = config.sensor_axis_length;

                let x_axis = rot * Vec3::X * axis_len;
                let y_axis = rot * Vec3::Y * axis_len;
                let z_axis = rot * Vec3::Z * axis_len;

                gizmos.arrow(pos, pos + x_axis, Color::srgb(1.0, 0.0, 0.0)); // X = Red
                gizmos.arrow(pos, pos + y_axis, Color::srgb(0.0, 1.0, 0.0)); // Y = Green
                gizmos.arrow(pos, pos + z_axis, Color::srgb(0.0, 0.0, 1.0)); // Z = Blue

                // Small sphere at sensor location
                gizmos.sphere(
                    Isometry3d::from_translation(pos),
                    config.sensor_axis_length * 0.15,
                    config.colors.sensor_imu,
                );
            }
            SensorVisualType::ForceTorque => {
                // Draw force arrow
                if let Some(force) = &sensor.force {
                    let force_vec = vec3_from_vector(force) * config.sensor_force_scale;
                    if force_vec.length() > 0.001 {
                        gizmos.arrow(pos, pos + force_vec, config.colors.sensor_force_torque);
                    }
                }

                // Draw torque as a different colored arrow
                if let Some(torque) = &sensor.torque {
                    let torque_vec = vec3_from_vector(torque) * config.sensor_force_scale;
                    if torque_vec.length() > 0.001 {
                        gizmos.arrow(
                            pos,
                            pos + torque_vec,
                            config.colors.sensor_force_torque.with_alpha(0.7),
                        );
                    }
                }

                // Sensor marker
                gizmos.sphere(
                    Isometry3d::from_translation(pos),
                    config.sensor_axis_length * 0.1,
                    config.colors.sensor_force_torque,
                );
            }
            SensorVisualType::Touch => {
                // Draw highlight sphere when active
                let color = if sensor.is_active {
                    config.colors.sensor_touch_active
                } else {
                    config.colors.sensor_touch_active.with_alpha(0.3)
                };

                let radius = if sensor.is_active {
                    config.sensor_axis_length * 0.2
                } else {
                    config.sensor_axis_length * 0.1
                };

                gizmos.sphere(Isometry3d::from_translation(pos), radius, color);
            }
            SensorVisualType::Rangefinder => {
                // Draw ray from sensor
                if let Some((direction, distance)) = &sensor.ray {
                    let dir_vec = vec3_from_vector(direction).normalize();
                    let end = pos + dir_vec * (*distance as f32);

                    // Dashed line effect (multiple short segments)
                    gizmos.line(pos, end, config.colors.sensor_imu.with_alpha(0.6));

                    // Hit point marker
                    gizmos.sphere(
                        Isometry3d::from_translation(end),
                        config.sensor_axis_length * 0.08,
                        config.colors.sensor_imu,
                    );
                }
            }
            SensorVisualType::Magnetometer => {
                // Draw magnetic field direction
                if let Some(field) = &sensor.magnetic_field {
                    let field_vec = vec3_from_vector(field).normalize() * config.sensor_axis_length;
                    gizmos.arrow(pos, pos + field_vec, Color::srgb(0.8, 0.0, 0.8));
                    // Magenta for magnetic field
                }

                gizmos.sphere(
                    Isometry3d::from_translation(pos),
                    config.sensor_axis_length * 0.1,
                    Color::srgb(0.8, 0.0, 0.8),
                );
            }
        }
    }
}

/// Linearly interpolate between two colors.
fn lerp_color(a: Color, b: Color, t: f32) -> Color {
    let a_linear = a.to_linear();
    let b_linear = b.to_linear();

    Color::linear_rgba(
        a_linear.red + (b_linear.red - a_linear.red) * t,
        a_linear.green + (b_linear.green - a_linear.green) * t,
        a_linear.blue + (b_linear.blue - a_linear.blue) * t,
        a_linear.alpha + (b_linear.alpha - a_linear.alpha) * t,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn debug_gizmos_set_derives() {
        // Test that the system set has proper derives
        let set = DebugGizmosSet;
        assert_eq!(set, DebugGizmosSet);
        assert_eq!(format!("{set:?}"), "DebugGizmosSet");
    }
}
