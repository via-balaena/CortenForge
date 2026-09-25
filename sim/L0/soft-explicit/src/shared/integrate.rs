// Time integration, per node: the explicit central-difference update with
// mass-proportional damping `α M` (plan §15c), the kinematic constraints
// (plan §16d), and the energy terms the loop's balance adds up (plan §16e).

/// A node's inverse mass, or 0 for a node held by a kinematic boundary
/// condition, which the update then leaves where it is (its velocity starts
/// at zero and never changes).
#[must_use]
pub const fn inverse_mass(mass: R, held: bool) -> R {
    if held { 0.0 } else { 1.0 / mass }
}

/// The velocity at the next half step.
///
/// `v⁺ = ((1 − αΔt/2) v⁻ + Δt f / m) / (1 + αΔt/2)`, the central-difference
/// form of `m a + α m v = f` with the damping force at the mid-step
/// velocity. The mass may be a scaled one; that is the caller's choice.
#[must_use]
pub const fn advance_velocity(
    velocity: [R; 3],
    force: [R; 3],
    inverse_mass: R,
    damping: R,
    dt: R,
) -> [R; 3] {
    let half_damping = 0.5 * damping * dt;
    let retained = 1.0 - half_damping;
    let divisor = 1.0 + half_damping;
    let impulse = dt * inverse_mass;
    [
        (retained * velocity[0] + impulse * force[0]) / divisor,
        (retained * velocity[1] + impulse * force[1]) / divisor,
        (retained * velocity[2] + impulse * force[2]) / divisor,
    ]
}

/// The displacement at the next step, `u⁺ = u + Δt v⁺`. Displacements, not
/// positions, are the state (plan §6); a position is the rest position plus
/// the displacement.
#[must_use]
pub const fn advance_displacement(displacement: [R; 3], velocity: [R; 3], dt: R) -> [R; 3] {
    vec3_add(displacement, vec3_scale(velocity, dt))
}

/// A node's kinetic energy, `½ m |v|²`.
#[must_use]
pub const fn kinetic_energy(mass: R, velocity: [R; 3]) -> R {
    0.5 * mass * vec3_dot(velocity, velocity)
}

/// `v` without its components along a node's two constraint directions.
///
/// `first` and `second` are orthonormal, or zero where unused, so a free
/// node passes through unchanged. They are fixed at rest: a constraint is a
/// plane the node moves in, not a curved surface it slides along (plan
/// §16d). The lumped mass is the same in every direction, so this is the
/// mass-orthogonal projection. Applied to the velocity and the displacement
/// after each update; both start with no component along a constraint, so
/// they never gain one.
#[must_use]
pub const fn constrain(v: [R; 3], first: [R; 3], second: [R; 3]) -> [R; 3] {
    let along_first = vec3_scale(first, vec3_dot(v, first));
    let along_second = vec3_scale(second, vec3_dot(v, second));
    vec3_sub(vec3_sub(v, along_first), along_second)
}

/// The work `force` does on a node over one step, `Δt f · (v⁻ + v⁺) / 2`,
/// with `v⁻` and `v⁺` the half-step velocities either side of it.
///
/// With [`damping_loss`] it balances the update exactly: `advance_velocity`
/// gives `½ m |v⁺|² − ½ m |v⁻|² = step_work(f) − damping_loss`, where `f`
/// is the node's total force, and a constraint's reaction does no work
/// because both velocities lie in the node's free directions.
#[must_use]
pub const fn step_work(
    force: [R; 3],
    previous_velocity: [R; 3],
    velocity: [R; 3],
    dt: R,
) -> R {
    0.5 * dt * vec3_dot(force, vec3_add(previous_velocity, velocity))
}

/// The energy mass damping removes from a node over one step,
/// `Δt α m |(v⁻ + v⁺) / 2|²`: the damping force `α m v` at the mid-step
/// velocity, as `advance_velocity` applies it.
#[must_use]
pub const fn damping_loss(
    mass: R,
    damping: R,
    previous_velocity: [R; 3],
    velocity: [R; 3],
    dt: R,
) -> R {
    let mid_step = vec3_scale(vec3_add(previous_velocity, velocity), 0.5);
    dt * damping * mass * vec3_dot(mid_step, mid_step)
}
