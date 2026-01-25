//! Debug visualization using Bevy gizmos.
//!
//! This module provides real-time debug visualization for:
//! - Contact points and normals
//! - Applied forces
//! - Velocity vectors (linear and angular)
//! - Joint axes and limits
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
use crate::resources::{CachedContacts, SimulationHandle, ViewerConfig};

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

/// Draw force vectors applied to bodies.
pub fn draw_force_vectors(
    mut gizmos: Gizmos,
    sim_handle: Res<SimulationHandle>,
    config: Res<ViewerConfig>,
) {
    if !config.show_forces {
        return;
    }

    let Some(world) = sim_handle.world() else {
        return;
    };

    // Draw accumulated forces on each body
    for body in world.bodies() {
        let pos = vec3_from_point(&body.state.pose.position);
        let force = vec3_from_vector(&body.accumulated_force);

        // Only draw if force is significant
        let force_magnitude = force.length();
        if force_magnitude < config.force_display_threshold {
            continue;
        }

        let scaled_force = force * config.force_scale;
        let end = pos + scaled_force;

        gizmos.arrow(pos, end, config.colors.force_vector);
    }
}

/// Draw joint axes and connections.
pub fn draw_joint_axes(
    mut gizmos: Gizmos,
    sim_handle: Res<SimulationHandle>,
    config: Res<ViewerConfig>,
) {
    if !config.show_joint_axes {
        return;
    }

    let Some(world) = sim_handle.world() else {
        return;
    };

    for joint in world.joints() {
        // Get parent and child bodies
        let (Some(parent), Some(child)) = (world.body(joint.parent), world.body(joint.child))
        else {
            continue;
        };

        let parent_pos = vec3_from_point(&parent.state.pose.position);
        let child_pos = vec3_from_point(&child.state.pose.position);

        // Draw a small sphere at the joint location (child body position)
        gizmos.sphere(
            Isometry3d::from_translation(child_pos),
            config.joint_marker_radius,
            config.colors.joint_axis,
        );

        // Draw line connecting parent and child bodies
        let distance = parent_pos.distance(child_pos);
        if distance > config.joint_line_distance_threshold {
            gizmos.line(
                parent_pos,
                child_pos,
                config.colors.joint_axis.with_alpha(0.5),
            );
        }

        // Transform joint axis from parent local frame to world frame
        let parent_rot = quat_from_unit_quaternion(&parent.state.pose.rotation);
        let local_axis = vec3_from_vector(&joint.axis);
        let world_axis = parent_rot * local_axis;

        // Draw axis arrow from child position
        let axis_end = child_pos + world_axis * config.joint_axis_length;
        gizmos.arrow(child_pos, axis_end, config.colors.joint_axis);
    }
}

/// Draw velocity vectors (linear and angular).
pub fn draw_velocity_vectors(
    mut gizmos: Gizmos,
    sim_handle: Res<SimulationHandle>,
    config: Res<ViewerConfig>,
) {
    if !config.show_velocities {
        return;
    }

    let Some(world) = sim_handle.world() else {
        return;
    };

    for body in world.bodies() {
        let pos = vec3_from_point(&body.state.pose.position);
        let linear_vel = vec3_from_vector(&body.state.twist.linear);
        let angular_vel = vec3_from_vector(&body.state.twist.angular);

        // Draw linear velocity if significant
        let linear_speed = linear_vel.length();
        if linear_speed > config.velocity_display_threshold {
            let scaled_vel = linear_vel * config.velocity_scale;
            gizmos.arrow(pos, pos + scaled_vel, config.colors.linear_velocity);
        }

        // Draw angular velocity if significant
        let angular_speed = angular_vel.length();
        if angular_speed > config.velocity_display_threshold {
            let scaled_angular = angular_vel * config.velocity_scale;
            // Draw angular velocity as a curved arrow indicator
            gizmos.arrow(pos, pos + scaled_angular, config.colors.angular_velocity);
        }
    }
}

/// Draw joint limits visualization.
pub fn draw_joint_limits(
    mut gizmos: Gizmos,
    sim_handle: Res<SimulationHandle>,
    config: Res<ViewerConfig>,
) {
    if !config.show_joint_limits {
        return;
    }

    let Some(world) = sim_handle.world() else {
        return;
    };

    for joint in world.joints() {
        let Some(limits) = &joint.limits else {
            continue;
        };

        // Get parent body position
        let Some(parent) = world.body(joint.parent) else {
            continue;
        };

        let pos = vec3_from_point(&parent.state.pose.position);

        // Draw arc showing the limit range for revolute joints
        let parent_rotation = quat_from_unit_quaternion(&parent.state.pose.rotation);

        let world_axis = parent_rotation * vec3_from_vector(&joint.axis);
        let arc_radius = config.joint_limit_arc_radius;

        // Find a perpendicular vector to draw the arc
        let perpendicular = if world_axis.x.abs() < 0.9 {
            world_axis.cross(Vec3::X).normalize()
        } else {
            world_axis.cross(Vec3::Y).normalize()
        };

        // Draw limit lines at min and max angles
        let min_angle = limits.lower() as f32;
        let max_angle = limits.upper() as f32;

        let min_dir = Quat::from_axis_angle(world_axis, min_angle) * perpendicular;
        let max_dir = Quat::from_axis_angle(world_axis, max_angle) * perpendicular;

        gizmos.line(pos, pos + min_dir * arc_radius, config.colors.joint_limit);
        gizmos.line(pos, pos + max_dir * arc_radius, config.colors.joint_limit);

        // Draw arc between limits
        let arc_segments = config.joint_limit_arc_segments;
        let angle_range = max_angle - min_angle;
        for i in 0..arc_segments {
            let t0 = i as f32 / arc_segments as f32;
            let t1 = (i + 1) as f32 / arc_segments as f32;
            let angle0 = min_angle + angle_range * t0;
            let angle1 = min_angle + angle_range * t1;

            let dir0 = Quat::from_axis_angle(world_axis, angle0) * perpendicular;
            let dir1 = Quat::from_axis_angle(world_axis, angle1) * perpendicular;

            gizmos.line(
                pos + dir0 * arc_radius,
                pos + dir1 * arc_radius,
                config.colors.joint_limit.with_alpha(0.5),
            );
        }
    }
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
