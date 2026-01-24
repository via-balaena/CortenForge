//! Debug visualization using Bevy gizmos.
//!
//! This module provides real-time debug visualization for:
//! - Contact points and normals
//! - Applied forces
//! - Velocity vectors (linear and angular)
//! - Joint axes and limits
//!
//! All visualization is optional and controlled via [`ViewerConfig`].

#![allow(clippy::needless_pass_by_value)] // Bevy system parameters
#![allow(clippy::cast_possible_truncation)] // f64 -> f32 is intentional for Bevy
#![allow(clippy::cast_precision_loss)] // Small int -> f32 is fine for arc segments

use bevy::prelude::*;
use sim_contact::ContactPoint;

use crate::convert::{quat_from_unit_quaternion, vec3_from_point, vec3_from_vector};
use crate::resources::{SimulationHandle, ViewerConfig};

/// System set for debug visualization.
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub struct DebugGizmosSet;

/// Draw contact point markers as small spheres.
pub fn draw_contact_points(
    mut gizmos: Gizmos,
    sim_handle: Res<SimulationHandle>,
    config: Res<ViewerConfig>,
) {
    if !config.show_contacts {
        return;
    }

    let Some(world) = sim_handle.world() else {
        return;
    };

    // Detect contacts for visualization
    // Note: This is a read-only snapshot; actual contact detection happens in sim-core
    let contacts = collect_contacts_for_display(world);

    for contact in &contacts {
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
pub fn draw_contact_normals(
    mut gizmos: Gizmos,
    sim_handle: Res<SimulationHandle>,
    config: Res<ViewerConfig>,
) {
    if !config.show_contact_normals {
        return;
    }

    let Some(world) = sim_handle.world() else {
        return;
    };

    let contacts = collect_contacts_for_display(world);
    let normal_length = 0.15; // Fixed length for visibility

    for contact in &contacts {
        let start = vec3_from_point(&contact.position);
        let normal = vec3_from_vector(&contact.normal);
        let end = start + normal * normal_length;

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
        if force_magnitude < 0.01 {
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

    let axis_length = 0.2;

    for joint in world.joints() {
        // Get parent and child body positions
        let parent_pos = world
            .body(joint.parent)
            .map(|b| vec3_from_point(&b.state.pose.position));
        let child_pos = world
            .body(joint.child)
            .map(|b| vec3_from_point(&b.state.pose.position));

        let (Some(parent_pos), Some(child_pos)) = (parent_pos, child_pos) else {
            continue;
        };

        // Draw line connecting joint bodies
        gizmos.line(
            parent_pos,
            child_pos,
            config.colors.joint_axis.with_alpha(0.5),
        );

        // Draw joint axis at the connection midpoint
        let midpoint = (parent_pos + child_pos) * 0.5;

        // Transform joint axis from parent local frame to world frame
        let parent_rotation = world
            .body(joint.parent)
            .map(|b| quat_from_unit_quaternion(&b.state.pose.rotation));

        if let Some(parent_rot) = parent_rotation {
            let local_axis = vec3_from_vector(&joint.axis);
            let world_axis = parent_rot * local_axis;

            let axis_start = midpoint - world_axis * (axis_length * 0.5);
            let axis_end = midpoint + world_axis * (axis_length * 0.5);

            gizmos.arrow(axis_start, axis_end, config.colors.joint_axis);
        }
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
        if linear_speed > 0.01 {
            let scaled_vel = linear_vel * config.velocity_scale;
            gizmos.arrow(pos, pos + scaled_vel, config.colors.linear_velocity);
        }

        // Draw angular velocity if significant
        let angular_speed = angular_vel.length();
        if angular_speed > 0.01 {
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
        let arc_radius = 0.1;

        // Draw limit markers at min and max angles
        // For revolute joints, draw small lines at the limit angles
        let perpendicular = if world_axis.x.abs() < 0.9 {
            world_axis.cross(Vec3::X).normalize()
        } else {
            world_axis.cross(Vec3::Y).normalize()
        };

        // Draw limit lines
        let min_angle = limits.lower() as f32;
        let max_angle = limits.upper() as f32;

        let min_dir = Quat::from_axis_angle(world_axis, min_angle) * perpendicular;
        let max_dir = Quat::from_axis_angle(world_axis, max_angle) * perpendicular;

        gizmos.line(pos, pos + min_dir * arc_radius, config.colors.joint_limit);
        gizmos.line(pos, pos + max_dir * arc_radius, config.colors.joint_limit);

        // Draw arc between limits
        let arc_segments = 16;
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

/// Collect contacts from the world for visualization.
///
/// This performs contact detection to get a snapshot of current contacts.
/// For performance, this should ideally use cached contacts from the last
/// physics step, but we use fresh detection for accuracy.
fn collect_contacts_for_display(world: &sim_core::World) -> Vec<ContactPoint> {
    // Clone world to avoid borrow issues with detect_contacts (which needs &mut)
    // In a production system, we'd cache contacts after each physics step
    let mut world_clone = world.clone();
    world_clone.detect_contacts()
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
