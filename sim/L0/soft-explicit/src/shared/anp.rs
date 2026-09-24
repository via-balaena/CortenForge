// Selective averaged nodal pressure (ANP), and its rule where materials meet.
//
// Only the λ term is averaged (plan §15c). Per step, in the phase order of
// plan §15f:
//   1. each element's current volume `v_e`             (`tet4_volume`)
//   2. the executor gathers `v_a = Σ v_e / 4` per node  (orchestration);
//      `V_a = Σ V_e / 4` comes from lowering
//   3. per node, `g_a = pressure_per_lambda(J_a)`, `J_a = v_a / V_a`
//   4. per element, `p̄_e = element_pressure(λ_e, g of its four nodes)`,
//      then `tet4_elastic_forces`.
//
// The forces are then exactly `−∂E/∂x` of
// `E = Σ_e V_e Ψ_μ(F_e) + Σ_a V_a U(J_a)` inside one material.

/// A node's volume ratio, `J_a = v_a / V_a`: its current tributary volume
/// over its rest one.
#[must_use]
pub const fn nodal_volume_ratio(current_volume: R, rest_volume: R) -> R {
    current_volume / rest_volume
}

/// The element's averaged pressure, `p̄_e = λ_e · ¼ Σ g_a`, from the
/// pressure per unit λ (`pressure_per_lambda`) at its four nodes.
///
/// This is the interface rule the plan names (IANP, §15g step 1): each
/// node's pressure is evaluated with *this element's* material, then
/// averaged over the element. The λ term's pressure is λ times a function of
/// `J` alone, so that is `λ_e` times the average of `g_a`. Inside one
/// material it is plain ANP's `¼ Σ p_a`, since every `p_a = λ g_a` there.
#[must_use]
pub const fn element_pressure(lambda: R, per_lambda: [R; 4]) -> R {
    lambda * 0.25 * (per_lambda[0] + per_lambda[1] + per_lambda[2] + per_lambda[3])
}
