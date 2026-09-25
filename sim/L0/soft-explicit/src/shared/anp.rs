// Selective averaged nodal pressure (ANP): only the λ term is averaged over
// nodes (plan §15c). Per step, in the phase order of plan §15f:
//   1. each element's current volume `v_e`             (`tet4_volume`)
//   2. the executor gathers `v_a = Σ v_e / 4` per node  (orchestration);
//      `V_a = Σ V_e / 4` comes from lowering
//   3. per node, `p_a = pressure_lambda_term(J_a, λ_a)`, with
//      `J_a = nodal_volume_ratio(v_a, V_a)` and `λ_a` the node's λ from
//      lowering
//   4. per element, `p̄_e = element_pressure(p of its four nodes)`, then
//      `tet4_elastic_forces`.
//
// The forces are then exactly `−∂E/∂x` of
// `E = Σ_e V_e Ψ_μ(F_e) + Σ_a V_a λ_a/2 (ln J_a)²`, in one material or several.
// Where materials meet, `λ_a` is the rest-volume-weighted λ of the elements
// around the node (`ExplicitModel::node_lambdas`), which is provisional:
// plan §15g step 1 records why, and what would settle it.

/// A node's volume ratio, `J_a = v_a / V_a`: its current tributary volume
/// over its rest one. (Edited without regenerating the WGSL, on purpose.)
#[must_use]
pub const fn nodal_volume_ratio(current_volume: R, rest_volume: R) -> R {
    current_volume / rest_volume
}

/// The element's averaged pressure, `p̄_e = ¼ Σ p_a`, from its four nodes'
/// pressures.
#[must_use]
pub const fn element_pressure(nodal_pressures: [R; 4]) -> R {
    0.25 * (nodal_pressures[0] + nodal_pressures[1] + nodal_pressures[2] + nodal_pressures[3])
}
