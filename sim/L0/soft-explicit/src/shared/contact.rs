// Contact between a node and the rigid obstacle: the kinematic predictor/
// corrector, with kinematic Coulomb friction (Abaqus/Explicit's contact
// pairs, Analysis User's Manual 6.11 §36.2.3 and §35.1.5; plan §16o).
//
// Each step the node's position at the end of the step is predicted without
// contact. If it lands inside the obstacle, the node gets the force that puts
// it back on the surface; friction holds a sticking node at its anchor, with
// no elastic slip, and moves a slipping one back by at most μ_f times the
// normal correction. The law adds nothing to the stable step, and it is rate
// independent, so time scaling stays valid. A node's normal velocity into the
// obstacle is lost on contact.
//
// It replaced the nodal-mass penalty (plan §15c's variant (a)), whose gap
// could not meet G2, in G2's A/B (plan §16o).

/// One node's contact result.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactResponse {
    /// The force the step applies to the node, world frame: the contact
    /// force's part in the node's free directions. On a constrained node the
    /// constraint carries the rest, so this is not the whole contact force.
    pub force: [R; 3],
    /// The node's friction anchor for the next step, body frame.
    pub anchor: [R; 3],
    /// The magnitude of the obstacle's normal force on the node, constraint
    /// included; zero out of contact. Divided by the node's tributary area,
    /// it is the contact pressure.
    pub normal_force: R,
    /// The friction's part of `force`, world frame, in the contact's tangent
    /// plane. On an unconstrained node its ratio to `μ_f · normal_force` is 1
    /// when slipping and below 1 when sticking, which is what K6 reads (plan
    /// §16b).
    pub friction: [R; 3],
}

/// Below this, a contact normal lies almost wholly in a node's constrained
/// directions, and the law leaves the node where it is.
pub const KINEMATIC_MIN_REACH: R = 1e-3;

/// The stiffness that turns a correction δ into the force that makes it,
/// `f = m (1 + αΔt/2) δ / Δt²`: what `advance_velocity` needs to move the
/// node by δ over one step. Zero for a held node.
#[must_use]
pub const fn kinematic_stiffness(mass: R, inverse_mass: R, damping: R, dt: R) -> R {
    if inverse_mass > 0.0 {
        mass * (1.0 + 0.5 * damping * dt) / (dt * dt)
    } else {
        0.0
    }
}

/// The contact force on one node.
///
/// `predicted` is where the node lands at the end of the step without
/// contact (world frame), `pose` the obstacle's pose then, and `anchor` the
/// node's friction anchor (body frame). `stiffness` is
/// [`kinematic_stiffness`], and `constraints` are the node's two constraint
/// directions (`constrain`).
///
/// `sample` carries the obstacle's distance at `pose_to_body(pose,
/// predicted)`, but the normal where the node is now, in the same frame.
/// Taken at the predicted point, which the step's inward push leaves at a
/// smaller radius of a curved obstacle, the correction carried a sliding node
/// further around than it went, and the sliding grew by about `1 + a/R` a
/// step, `a` the push (plan §16o).
///
/// A node that would land inside gets the force that puts it on the
/// surface: along the normal, made free of the node's constrained
/// directions and lengthened so it still reaches the surface. That is exact
/// on a plane and first order on a curved surface; the confined tube's nodes
/// read up to 0.2 µm inside its mandrel (plan §16o). A sticking
/// node is held at its anchor; a slipping one moves back by at most `μ_f`
/// times the normal correction, and its anchor goes with it. The friction
/// step is kept to the node's free directions and to the surface. Out of
/// contact the anchor moves with the node, so a new contact starts sticking
/// where it begins.
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
    let corrected = vec3_add(predicted, normal_step);
    let slip = vec3_sub(pose_to_body(pose, corrected), anchor);
    let tangential = vec3_sub(slip, vec3_scale(sample.normal, vec3_dot(slip, sample.normal)));
    let tangential_length = vec3_length(tangential);
    let limit = friction * penetration / guarded_reach;
    let sticking = tangential_length <= limit;
    let guarded_length = if tangential_length > 0.0 {
        tangential_length
    } else {
        1.0
    };
    let pull = if sticking { 1.0 } else { limit / guarded_length };
    // The part of that step the node can take: in its free directions, and
    // along the surface (the free normal takes back what leaves it).
    let wanted = constrain(
        pose_rotate(pose, vec3_scale(tangential, -pull)),
        constraints[0],
        constraints[1],
    );
    let tangential_step = vec3_sub(wanted, vec3_scale(free, vec3_dot(wanted, normal) / guarded_reach));
    let landed = pose_to_body(pose, vec3_add(corrected, tangential_step));
    let kept = vec3_select(sticking, anchor, landed);

    let scale = if in_contact { stiffness } else { 0.0 };
    ContactResponse {
        force: vec3_scale(vec3_add(normal_step, tangential_step), scale),
        anchor: vec3_select(in_contact, kept, pose_to_body(pose, predicted)),
        normal_force: scale * penetration / guarded_reach,
        friction: vec3_scale(tangential_step, scale),
    }
}
