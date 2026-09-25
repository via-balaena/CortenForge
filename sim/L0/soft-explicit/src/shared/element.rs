// The four-node tetrahedron (Tet4). Node positions are `[R; 12]`: x, y, z of
// node 0, then of nodes 1, 2 and 3. Each element's rest data is its rest
// volume and the inverse of its rest edge matrix, computed once in lowering.

/// The edge matrix `D = [x₁ − x₀, x₂ − x₀, x₃ − x₀]`, the edges as columns.
#[must_use]
pub const fn tet4_edge_matrix(x: [R; 12]) -> [R; 9] {
    [
        x[3] - x[0],
        x[6] - x[0],
        x[9] - x[0],
        x[4] - x[1],
        x[7] - x[1],
        x[10] - x[1],
        x[5] - x[2],
        x[8] - x[2],
        x[11] - x[2],
    ]
}

/// The signed volume, `det D / 6`: positive when the nodes are ordered so
/// that node 3 is on the side of face (0, 1, 2) its normal
/// `(x₁ − x₀) × (x₂ − x₀)` points to.
#[must_use]
pub const fn tet4_volume(x: [R; 12]) -> R {
    mat3_det(tet4_edge_matrix(x)) / 6.0
}

/// The deformation gradient `F = D D_rest⁻¹`.
#[must_use]
pub const fn tet4_deformation_gradient(x: [R; 12], rest_edge_inverse: [R; 9]) -> [R; 9] {
    mat3_mul(tet4_edge_matrix(x), rest_edge_inverse)
}

/// The nodal forces of a constant first Piola–Kirchhoff stress `S`.
///
/// `H = V S D_rest⁻ᵀ`; nodes 1–3 get minus the columns of `H`, and
/// node 0 minus their sum. These are the forces `−∂E/∂x` when `S = ∂Ψ/∂F`.
#[must_use]
pub const fn tet4_nodal_forces(
    stress: [R; 9],
    rest_edge_inverse: [R; 9],
    rest_volume: R,
) -> [R; 12] {
    let h = mat3_scale(
        mat3_mul(stress, mat3_transpose(rest_edge_inverse)),
        rest_volume,
    );
    [
        h[0] + h[1] + h[2],
        h[3] + h[4] + h[5],
        h[6] + h[7] + h[8],
        -h[0],
        -h[3],
        -h[6],
        -h[1],
        -h[4],
        -h[7],
        -h[2],
        -h[5],
        -h[8],
    ]
}

/// The element's elastic forces under selective averaged nodal pressure.
///
/// The μ terms from this element's own `F`, and the λ term from `pressure`,
/// the element's averaged pressure `p̄` (see `element_pressure`).
///
/// The λ term enters as `p̄ · cof F`: the stress whose forces are
/// `−p̄ ∂v/∂x`, with `v` the element's current volume.
#[must_use]
pub const fn tet4_elastic_forces(
    x: [R; 12],
    rest_edge_inverse: [R; 9],
    rest_volume: R,
    material: Material,
    pressure: R,
) -> [R; 12] {
    let f = tet4_deformation_gradient(x, rest_edge_inverse);
    let stress = mat3_add(
        first_piola_mu_terms(f, material),
        mat3_scale(mat3_cofactor(f), pressure),
    );
    tet4_nodal_forces(stress, rest_edge_inverse, rest_volume)
}

/// The μ terms' energy in one element, `V Ψ_μ(F)`.
#[must_use]
pub fn tet4_energy_mu_terms(
    x: [R; 12],
    rest_edge_inverse: [R; 9],
    rest_volume: R,
    material: Material,
) -> R {
    rest_volume * energy_density_mu_terms(tet4_deformation_gradient(x, rest_edge_inverse), material)
}

/// The shortest altitude, `3 |v| / (largest face area)`, for a tetrahedron
/// that is not flat.
#[must_use]
pub fn tet4_shortest_altitude(x: [R; 12]) -> R {
    let p0 = [x[0], x[1], x[2]];
    let p1 = [x[3], x[4], x[5]];
    let p2 = [x[6], x[7], x[8]];
    let p3 = [x[9], x[10], x[11]];
    let area_012 = vec3_length(vec3_cross(vec3_sub(p1, p0), vec3_sub(p2, p0)));
    let area_013 = vec3_length(vec3_cross(vec3_sub(p1, p0), vec3_sub(p3, p0)));
    let area_023 = vec3_length(vec3_cross(vec3_sub(p2, p0), vec3_sub(p3, p0)));
    let area_123 = vec3_length(vec3_cross(vec3_sub(p2, p1), vec3_sub(p3, p1)));
    let largest = area_012.max(area_013).max(area_023.max(area_123)) * 0.5;
    3.0 * tet4_volume(x).abs() / largest
}

/// The small-strain dilatational wave speed, `√((λ + 8 C₂ + 2μ) / ρ)`. The
/// Yeoh term adds `8 C₂` to λ at rest.
#[must_use]
pub fn dilatational_wave_speed(material: Material) -> R {
    ((material.lambda + 8.0 * material.c2 + 2.0 * material.mu) / material.density).sqrt()
}

/// The element's stable time-step estimate, shortest altitude over wave
/// speed.
///
/// A cross-check only: the solver's step comes from a power
/// iteration (plan §15c, where the method research reports this estimate
/// 4.4× loose on a jittered mesh).
#[must_use]
pub fn tet4_time_step_estimate(x: [R; 12], material: Material) -> R {
    tet4_shortest_altitude(x) / dilatational_wave_speed(material)
}
