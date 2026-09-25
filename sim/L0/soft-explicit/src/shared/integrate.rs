// Time integration, per node: the explicit central-difference update with
// mass-proportional damping `α M` (plan §15c).

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
