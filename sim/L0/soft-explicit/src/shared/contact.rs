// Contact between a node and the rigid obstacle: the nodal-mass penalty
// (plan §15c, variant (a)) with elastic-slip Coulomb friction.
//
// The penalty stiffness scales with the node's mass, `k = s m / Δt²`, so it
// stays inside the explicit step's stability limit only if the step comes
// from a power iteration that includes it (plan §15c).
//
// Friction is a tangential penalty with the same `k`, against an anchor
// point stored per node in the obstacle's body frame: a sticking node is
// pulled back toward its anchor; when that pull would exceed `μ_f f_n` the
// node slips, and the anchor is dragged along so the pull is exactly
// `μ_f f_n` (Coulomb's cone, as in a return map). The law is rate
// independent, so time scaling stays valid.

/// One node's contact result.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactResponse {
    /// The force on the node, world frame.
    pub force: [R; 3],
    /// The node's friction anchor for the next step, body frame.
    pub anchor: [R; 3],
    /// The normal force's magnitude, `k · penetration`; zero out of contact.
    /// Divided by the node's tributary area, it is the contact pressure.
    pub normal_force: R,
    /// The friction force, world frame: the part of `force` in the contact's
    /// tangent plane. Its ratio to `μ_f · normal_force` is 1 on a slipping
    /// node and below 1 on a sticking one, which is what K6 reads (plan §16b).
    pub friction: [R; 3],
}

/// The penalty stiffness for a node of mass `mass`: `k = scale · m / Δt²`.
/// The plan's primary scale is 0.5 (§15c).
#[must_use]
pub const fn penalty_stiffness(mass: R, dt: R, scale: R) -> R {
    let dt_squared = dt * dt;
    scale * mass / dt_squared
}

/// The contact force on one node.
///
/// `point` is the node's position (rest position plus displacement) in the
/// obstacle's body frame (`pose_to_body`), `sample` the obstacle's distance
/// and normal there, and
/// `anchor` the node's friction anchor from the previous step (body frame).
/// Out of contact the anchor moves with the node, so a new contact starts
/// sticking where it begins.
#[must_use]
pub fn obstacle_contact(
    pose: Pose,
    point: [R; 3],
    sample: SdfSample,
    anchor: [R; 3],
    stiffness: R,
    friction: R,
) -> ContactResponse {
    let penetration = (-sample.distance).max(0.0);
    let in_contact = penetration > 0.0;
    let normal_force = stiffness * penetration;
    let normal = sample.normal;

    // Tangential displacement since the anchor, and the force that would
    // hold it.
    let slip = vec3_sub(point, anchor);
    let tangential = vec3_sub(slip, vec3_scale(normal, vec3_dot(slip, normal)));
    let tangential_length = vec3_length(tangential);
    let limit = friction * normal_force;
    let sticking = stiffness * tangential_length <= limit;
    let guarded_length = if tangential_length > 0.0 {
        tangential_length
    } else {
        1.0
    };
    let pull = if sticking {
        stiffness
    } else {
        limit / guarded_length
    };
    let friction_force = vec3_scale(tangential, -pull);

    // Slipping drags the anchor to within `limit / k` of the node.
    let guarded_stiffness = if stiffness > 0.0 { stiffness } else { 1.0 };
    let dragged = vec3_sub(
        point,
        vec3_scale(tangential, limit / (guarded_stiffness * guarded_length)),
    );
    let kept = vec3_select(sticking, anchor, dragged);

    let body_force = vec3_add(vec3_scale(normal, normal_force), friction_force);
    ContactResponse {
        force: pose_rotate(pose, body_force),
        anchor: vec3_select(in_contact, kept, point),
        normal_force,
        friction: pose_rotate(pose, friction_force),
    }
}
