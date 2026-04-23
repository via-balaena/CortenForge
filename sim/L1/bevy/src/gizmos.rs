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

use crate::convert::{vec3_from_point, vec3_from_vector};
use crate::resources::{CachedContacts, MuscleVisualization, TendonVisualization, ViewerConfig};

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

// ============================================================================
// Trail Visualization
// ============================================================================

/// Ring buffer of world-space positions for trail visualization.
///
/// Attach this component to any entity with a `Transform`. The `draw_trails`
/// system samples the entity's position at a configurable interval and draws
/// a fading polyline trail using Bevy gizmos.
///
/// # Example
///
/// ```ignore
/// commands.spawn((
///     Mesh3d(mesh),
///     Transform::default(),
///     TrailGizmo::new(500, Color::srgb(0.2, 0.7, 0.4), 0.01),
/// ));
/// ```
#[derive(Component)]
pub struct TrailGizmo {
    positions: std::collections::VecDeque<Vec3>,
    max_points: usize,
    color: Color,
    sample_interval: f64,
    last_sample_time: f64,
}

impl TrailGizmo {
    /// Create a new trail gizmo.
    ///
    /// - `max_points`: ring buffer capacity (e.g. 500 = ~5s at 100Hz)
    /// - `color`: trail color (alpha fades from 1.0 at head to 0.0 at tail)
    /// - `sample_interval`: seconds between position samples
    #[must_use]
    pub fn new(max_points: usize, color: Color, sample_interval: f64) -> Self {
        Self {
            positions: std::collections::VecDeque::with_capacity(max_points),
            max_points,
            color,
            sample_interval,
            last_sample_time: -1.0,
        }
    }
}

/// Sample trail positions from entities with `TrailGizmo` + `Transform`.
///
/// Call this in `PostUpdate` after `sync_geom_transforms` so positions are
/// up to date. Uses physics time from `PhysicsData` for consistent sampling.
pub fn sample_trails(
    data: Res<crate::model_data::PhysicsData>,
    mut query: Query<(&Transform, &mut TrailGizmo)>,
) {
    let time = data.time;
    for (transform, mut trail) in &mut query {
        if time - trail.last_sample_time >= trail.sample_interval {
            trail.last_sample_time = time;
            if trail.positions.len() >= trail.max_points {
                trail.positions.pop_front();
            }
            trail.positions.push_back(transform.translation);
        }
    }
}

/// Draw trail polylines with alpha fading.
///
/// Newest points are fully opaque; oldest points are fully transparent.
/// Add this system to `PostUpdate` after `sample_trails`.
pub fn draw_trails(mut gizmos: Gizmos, query: Query<&TrailGizmo>) {
    for trail in &query {
        let len = trail.positions.len();
        if len < 2 {
            continue;
        }
        let base = trail.color.to_linear();
        for i in 0..len - 1 {
            // Precision loss acceptable for approximate / visualization values.
            #[allow(clippy::cast_precision_loss)]
            let alpha = (i + 1) as f32 / len as f32;
            let color = Color::linear_rgba(base.red, base.green, base.blue, alpha);
            trail
                .positions
                .get(i)
                .zip(trail.positions.get(i + 1))
                .inspect(|&(a, b)| {
                    gizmos.line(*a, *b, color);
                });
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
