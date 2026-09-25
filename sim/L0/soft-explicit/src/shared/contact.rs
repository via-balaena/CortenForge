// Contact between a node and the rigid obstacle, by one of two laws (plan
// §16n's A/B): the nodal-mass penalty (plan §15c, variant (a)), optionally
// augmented by a per-node multiplier, with elastic-slip Coulomb friction;
// or the kinematic predictor/corrector, with kinematic friction.
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
    /// The normal force's magnitude; zero out of contact. Divided by the
    /// node's tributary area, it is the contact pressure.
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

/// The penalty contact force on one node.
///
/// `point` is the node's position (rest position plus displacement) in the
/// obstacle's body frame (`pose_to_body`), `sample` the obstacle's distance
/// and normal there, and
/// `anchor` the node's friction anchor from the previous step (body frame).
/// Out of contact the anchor moves with the node, so a new contact starts
/// sticking where it begins.
///
/// The normal force is `max(0, λ + k · penetration)`, the penetration
/// signed (negative outside). With `multiplier` λ = 0 it is the plain
/// penalty; an augmented Lagrangian carries λ per node
/// ([`multiplier_update`]).
#[must_use]
pub fn obstacle_contact(
    pose: Pose,
    point: [R; 3],
    sample: SdfSample,
    anchor: [R; 3],
    stiffness: R,
    friction: R,
    multiplier: R,
) -> ContactResponse {
    let normal_force = (multiplier + stiffness * -sample.distance).max(0.0);
    let in_contact = normal_force > 0.0;
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

/// The augmented Lagrangian's update of a node's multiplier λ.
///
/// `max(0, λ + k · p̄)`, with `p̄` the node's signed penetration averaged over
/// the steps since the last update. At a steady contact λ grows until it
/// carries the whole normal force and the penetration is zero.
#[must_use]
pub fn multiplier_update(multiplier: R, stiffness: R, mean_penetration: R) -> R {
    (multiplier + stiffness * mean_penetration).max(0.0)
}

/// Below this, a contact normal lies almost wholly in a node's constrained
/// directions, and the kinematic law leaves the node where it is.
pub const KINEMATIC_MIN_REACH: R = 1e-3;

/// The stiffness that turns a kinematic correction δ into the force that
/// makes it, `f = m (1 + αΔt/2) δ / Δt²`: what `advance_velocity` needs to
/// move the node by δ over one step. Zero for a held node.
#[must_use]
pub const fn kinematic_stiffness(mass: R, inverse_mass: R, damping: R, dt: R) -> R {
    if inverse_mass > 0.0 {
        mass * (1.0 + 0.5 * damping * dt) / (dt * dt)
    } else {
        0.0
    }
}

/// The kinematic predictor/corrector's contact force on one node.
///
/// Abaqus/Explicit's contact pairs; plan §16n.
/// `predicted` is where the node lands at the end of the step without
/// contact (world frame), `pose` the obstacle's pose then, `sample` the
/// obstacle's distance and normal at `pose_to_body(pose, predicted)`, and
/// `anchor` the node's friction anchor (body frame). `stiffness` is
/// [`kinematic_stiffness`]; `constraints` are the node's two constraint
/// directions (`constrain`).
///
/// A node that would land inside gets the force that puts it on the
/// surface: along the normal, made free of the node's constrained
/// directions and lengthened so it still reaches the surface. Friction is
/// kinematic too: a sticking node is held at its anchor, with no elastic
/// slip; a slipping one moves back by at most `μ_f` times the normal
/// correction, and its anchor goes with it. The node's normal velocity into
/// the obstacle is lost on contact.
#[must_use]
pub fn kinematic_contact(
    pose: Pose,
    predicted: [R; 3],
    sample: SdfSample,
    anchor: [R; 3],
    stiffness: R,
    friction: R,
    constraints: [[R; 3]; 2],
) -> ContactResponse {
    let penetration = (-sample.distance).max(0.0);
    let normal = pose_rotate(pose, sample.normal);
    let free = constrain(normal, constraints[0], constraints[1]);
    let reach = vec3_dot(free, normal);
    let movable = stiffness > 0.0 && reach > KINEMATIC_MIN_REACH;
    let in_contact = movable && penetration > 0.0;
    let guarded_reach = if movable { reach } else { 1.0 };
    let normal_step = vec3_scale(free, penetration / guarded_reach);

    // Friction in the body frame, from where the normal step leaves the node.
    let corrected = pose_to_body(pose, vec3_add(predicted, normal_step));
    let slip = vec3_sub(corrected, anchor);
    let tangential = vec3_sub(slip, vec3_scale(sample.normal, vec3_dot(slip, sample.normal)));
    let tangential_length = vec3_length(tangential);
    let limit = friction * penetration;
    let sticking = tangential_length <= limit;
    let guarded_length = if tangential_length > 0.0 {
        tangential_length
    } else {
        1.0
    };
    let pull = if sticking { 1.0 } else { limit / guarded_length };
    let body_tangential_step = vec3_scale(tangential, -pull);
    let landed = vec3_add(corrected, body_tangential_step);
    let kept = vec3_select(sticking, anchor, landed);

    let tangential_step = pose_rotate(pose, body_tangential_step);
    let scale = if in_contact { stiffness } else { 0.0 };
    ContactResponse {
        force: vec3_scale(vec3_add(normal_step, tangential_step), scale),
        anchor: vec3_select(in_contact, kept, pose_to_body(pose, predicted)),
        normal_force: scale * penetration,
        friction: vec3_scale(tangential_step, scale),
    }
}
