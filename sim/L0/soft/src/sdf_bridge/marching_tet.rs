//! Marching-tetrahedra inside-region clipping — pure case logic.
//!
//! Per parent tet: 4 signed SDF values at the vertices map to a 4-bit
//! case index (bit `i` = 1 iff vertex `i` is inside, with the
//! tie-breaking rule `v ≤ 0` ⇒ inside per scope memo §3 Decision M
//! D-8). Each of the 16 cases names a sub-tet decomposition of the
//! inside region. The 16 cases reduce to 3 structurally unique
//! configurations under symmetry per
//! [Wikipedia: Marching tetrahedra](https://en.wikipedia.org/wiki/Marching_tetrahedra)
//! and Treece, Prager, Gee 1999 ("Regularised marching tetrahedra:
//! improved iso-surface extraction"); this module enumerates all 16
//! (rotations + reflections) so the dispatch is a flat lookup.
//!
//! The three unique configurations (per Wikipedia):
//!
//! - Case 0 / 15: trivial — empty or full parent tet.
//! - 1-vertex-inside (cases 1, 2, 4, 8): one corner sub-tet capped at
//!   the inside vertex, with 3 cut points along the edges leaving it.
//! - 2-vertex-inside (cases 3, 5, 6, 9, 10, 12): triangular-prism wedge
//!   between the two inside vertices, decomposed into 3 sub-tets.
//! - 3-vertex-inside (cases 7, 11, 13, 14): parent tet minus a corner
//!   sub-tet at the outside vertex (frustum), decomposed into 3.
//!
//! Case-count topology (4 single-vertex / 6 two-vertex / 4
//! three-vertex non-trivial cases) cross-checked against Wikipedia.
//! Sub-tet templates were hand-derived using the standard prism
//! scheme `(a, b, c, a'), (a', b, c, b'), (a', b', c, c')` for
//! 2-inside cases and the standard frustum scheme `(triangle, c0),
//! (edge1, c0, c1), (vertex, c0, c1, c2)` for 3-inside cases — both
//! schemes appear in the broader marching-tetrahedra literature
//! (Treece 1999, Bloomenthal 1994). Each non-trivial template was
//! independently volume-verified during derivation at `t = 0.5`
//! mid-edge cuts: the inside-region volume equals the parent
//! volume minus the complementary corner-sub-tet volume(s).
//!
//! Orientation contract: sub-tet templates are written for legibility,
//! NOT for guaranteed right-handed signed volume. Some templates emit
//! left-handed orientation when the parent tet is right-handed (a
//! consequence of how the prism/frustum decomposition slots its three
//! sub-tets). Callers post-hoc swap the last two vertex slots if the
//! resulting signed volume is negative — see `sdf_meshed_tet_mesh.rs`'s
//! `from_sdf` consumption. Swapping the last two slots flips the
//! determinant sign while preserving the same 4-vertex set, so the
//! topological coverage of the inside region is unchanged. The swap is
//! deterministic (the sign of the signed volume is a deterministic
//! function of the parent positions).
//!
//! Degenerate-tet filter (D-10): callers also drop sub-tets whose
//! signed volume is below [`EPSILON_VOLUME`] after the orientation
//! swap. Catches lattice-on-sphere coincidences where the SDF is
//! FP-exactly-zero at a grid corner — the resulting sub-tets contain
//! a coincident vertex pair (the corner and its zero-`t` cut on an
//! adjacent edge) and degenerate to zero volume, but the legitimate
//! sub-tets sharing that corner stay above threshold.

use crate::Vec3;

/// Volume floor (m³) below which sub-tets are filtered as degenerate.
///
/// Fixed at `1e-15` per scope memo §3 Decision M D-10. Smallest
/// non-degenerate sub-tet at canonical sphere parameters has volume
/// `cell_size³ / 6 ≈ 1.3e-6` for `cell_size = 0.02`; boundary sub-tets
/// from MT-clip can be slivers with much smaller volumes but stay well
/// above 1e-15 unless they are true degenerates (coincident vertices
/// or zero-area faces).
pub(super) const EPSILON_VOLUME: f64 = 1e-15;

/// Vertex slot index into the per-tet vertex pool.
///
/// Slots 0..=3 are the parent tet's own vertices `v_0..=v_3`. Slots
/// 4..=9 are cut points on each of the 6 tet edges, in the canonical
/// order below — see [`EDGE_ENDPOINTS`].
pub(super) type VertexSlot = u8;

/// Edge endpoints (parent vertex slots) for each cut-point slot.
///
/// Slot offsets are `4..=9`; the corresponding edge is
/// `EDGE_ENDPOINTS[slot - 4]`. Order matches the canonical
/// edge-numbering in `mesh::quality::compute_metrics`'s dihedral
/// enumeration: (0-1), (0-2), (0-3), (1-2), (1-3), (2-3).
const EDGE_ENDPOINTS: [(VertexSlot, VertexSlot); 6] = [
    (0, 1), // slot 4: c_01
    (0, 2), // slot 5: c_02
    (0, 3), // slot 6: c_03
    (1, 2), // slot 7: c_12
    (1, 3), // slot 8: c_13
    (2, 3), // slot 9: c_23
];

/// Sub-tet decomposition for each of the 16 MT cases.
///
/// Index = case bit vector (bit `i` set iff vertex `i` is inside). Each
/// entry is a slice of `[VertexSlot; 4]` sub-tets that triangulate the
/// inside region. Empty slice → no sub-tets emitted (case 0 / fully
/// outside). Single-element slice → one corner sub-tet (1-inside) or
/// the parent itself (case 15). Three-element slice → prism (2-inside)
/// or frustum (3-inside) decomposition.
///
/// Templates follow the standard prism scheme `(a, b, c, a'), (a', b,
/// c, b'), (a', b', c, c')` for 2-inside and the standard frustum
/// scheme `(triangle, c0), (edge1, c0, c1), (vertex, c0, c1, c2)` for
/// 3-inside, both volume-verified at `t = 0.5` mid-edge cuts.
const SUB_TETS_BY_CASE: [&[[VertexSlot; 4]]; 16] = [
    // case 0 — none inside, no sub-tets
    &[],
    // case 1 — v0 inside, corner sub-tet at v0
    &[[0, 4, 5, 6]],
    // case 2 — v1 inside, corner sub-tet at v1
    &[[1, 4, 7, 8]],
    // case 3 — v0, v1 inside, prism between them; cuts on edges 02/03/12/13
    &[[0, 5, 6, 1], [1, 5, 6, 7], [1, 7, 6, 8]],
    // case 4 — v2 inside, corner sub-tet at v2
    &[[2, 5, 7, 9]],
    // case 5 — v0, v2 inside, prism; cuts on edges 01/03/12/23
    &[[0, 4, 6, 2], [2, 4, 6, 7], [2, 7, 6, 9]],
    // case 6 — v1, v2 inside, prism; cuts on edges 01/02/13/23
    &[[1, 4, 8, 2], [2, 4, 8, 5], [2, 5, 8, 9]],
    // case 7 — v0, v1, v2 inside (v3 out), frustum; cuts on edges 03/13/23
    &[[0, 1, 2, 6], [1, 2, 6, 8], [2, 6, 8, 9]],
    // case 8 — v3 inside, corner sub-tet at v3
    &[[3, 6, 8, 9]],
    // case 9 — v0, v3 inside, prism; cuts on edges 01/02/13/23
    &[[0, 4, 5, 3], [3, 4, 5, 8], [3, 8, 5, 9]],
    // case 10 — v1, v3 inside, prism; cuts on edges 01/03/12/23
    &[[1, 4, 7, 3], [3, 4, 7, 6], [3, 6, 7, 9]],
    // case 11 — v0, v1, v3 inside (v2 out), frustum; cuts on edges 02/12/23
    &[[0, 1, 3, 5], [1, 3, 5, 7], [3, 5, 7, 9]],
    // case 12 — v2, v3 inside, prism; cuts on edges 02/03/12/13
    &[[2, 5, 7, 3], [3, 5, 7, 6], [3, 6, 7, 8]],
    // case 13 — v0, v2, v3 inside (v1 out), frustum; cuts on edges 01/12/13
    &[[0, 2, 3, 4], [2, 3, 4, 7], [3, 4, 7, 8]],
    // case 14 — v1, v2, v3 inside (v0 out), frustum; cuts on edges 01/02/03
    &[[1, 2, 3, 4], [2, 3, 4, 5], [3, 4, 5, 6]],
    // case 15 — all inside, parent tet kept verbatim
    &[[0, 1, 2, 3]],
];

/// Compute the 4-bit MT case index from per-vertex SDF values.
///
/// Bit `i` is set iff `v[i] ≤ 0` — the D-8 tie-breaking rule treats an
/// SDF zero-coincidence (e.g. a lattice grid point landing exactly on
/// the sphere surface at radius `r` for `cell_size = r/N`) as inside.
/// The strict inequality `v[i] < 0` is rejected because it would route
/// such coincidences to the outside set, producing a different mesh
/// topology under FP rounding drift between runs.
pub(super) const fn case_index(v: [f64; 4]) -> u8 {
    let mut c = 0_u8;
    if v[0] <= 0.0 {
        c |= 1;
    }
    if v[1] <= 0.0 {
        c |= 2;
    }
    if v[2] <= 0.0 {
        c |= 4;
    }
    if v[3] <= 0.0 {
        c |= 8;
    }
    c
}

/// Return the per-case sub-tet decomposition.
pub(super) const fn sub_tets_for_case(case: u8) -> &'static [[VertexSlot; 4]] {
    SUB_TETS_BY_CASE[case as usize]
}

/// Return the (parent-vertex-slot) endpoints of an edge given its
/// cut-point slot. `slot` must be in `4..=9`; out-of-range inputs
/// panic.
pub(super) fn edge_endpoints(slot: VertexSlot) -> (VertexSlot, VertexSlot) {
    assert!(
        (4..=9).contains(&slot),
        "edge_endpoints: slot {slot} is not a cut-point slot (must be 4..=9)",
    );
    EDGE_ENDPOINTS[(slot - 4) as usize]
}

/// Linear interpolation parameter for an SDF zero-crossing on an edge.
///
/// Returns `t ∈ [0, 1]` such that the cut sits at `p_a + t * (p_b -
/// p_a)`. The clamp absorbs the rare case where round-off pushes `t`
/// just outside the unit interval (an in-out edge with both endpoints
/// FP-near-zero of opposite sign).
///
/// **Denom-near-zero guard** (mirrors cf-design's `interpolate_vertex`
/// at `mesher.rs:175-179`): when `|v_b - v_a| < f64::EPSILON` the cut
/// formula has no well-defined root; falling back to the midpoint
/// `t = 0.5` is the safe choice — the resulting cut lands inside the
/// edge and the degenerate-tet filter (D-10) drops any sub-tets that
/// pinch to zero volume around it.
pub(super) fn cut_parameter(v_a: f64, v_b: f64) -> f64 {
    let denom = v_b - v_a;
    if denom.abs() < f64::EPSILON {
        0.5
    } else {
        (-v_a / denom).clamp(0.0, 1.0)
    }
}

/// Closed-form signed volume of a tetrahedron, replicated here so
/// `marching_tet`'s orientation-correction logic stays independent of
/// `mesh::quality::compute_metrics`'s API surface.
///
/// Mathematically equivalent to `mesh::quality`'s `det / 6` form
/// (scalar triple product = determinant of the column matrix
/// `[p1-p0, p2-p0, p3-p0]`); the two FP results agree on sign for any
/// non-degenerate tet and on magnitude to within last-bit rounding
/// drift. The orientation-correction caller only consults the SIGN,
/// which is robust to that drift; the post-filter `signed_volume`
/// stored in `QualityMetrics` is recomputed from scratch by the
/// kernel, so any magnitude drift here doesn't propagate.
pub(super) fn signed_volume(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3) -> f64 {
    (p1 - p0).cross(&(p2 - p0)).dot(&(p3 - p0)) / 6.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn case_index_all_outside_and_all_inside() {
        assert_eq!(case_index([1.0, 1.0, 1.0, 1.0]), 0);
        assert_eq!(case_index([-1.0, -1.0, -1.0, -1.0]), 15);
    }

    #[test]
    fn case_index_one_inside_per_vertex() {
        // Each single-bit case index corresponds to exactly one vertex
        // inside; the bit position must match the vertex index.
        assert_eq!(case_index([-1.0, 1.0, 1.0, 1.0]), 1);
        assert_eq!(case_index([1.0, -1.0, 1.0, 1.0]), 2);
        assert_eq!(case_index([1.0, 1.0, -1.0, 1.0]), 4);
        assert_eq!(case_index([1.0, 1.0, 1.0, -1.0]), 8);
    }

    #[test]
    fn case_index_two_inside_combinations() {
        assert_eq!(case_index([-1.0, -1.0, 1.0, 1.0]), 3);
        assert_eq!(case_index([-1.0, 1.0, -1.0, 1.0]), 5);
        assert_eq!(case_index([1.0, -1.0, -1.0, 1.0]), 6);
        assert_eq!(case_index([-1.0, 1.0, 1.0, -1.0]), 9);
        assert_eq!(case_index([1.0, -1.0, 1.0, -1.0]), 10);
        assert_eq!(case_index([1.0, 1.0, -1.0, -1.0]), 12);
    }

    #[test]
    fn case_index_three_inside_combinations() {
        assert_eq!(case_index([-1.0, -1.0, -1.0, 1.0]), 7);
        assert_eq!(case_index([-1.0, -1.0, 1.0, -1.0]), 11);
        assert_eq!(case_index([-1.0, 1.0, -1.0, -1.0]), 13);
        assert_eq!(case_index([1.0, -1.0, -1.0, -1.0]), 14);
    }

    #[test]
    fn case_index_zero_sdf_classified_as_inside() {
        // D-8 tie-breaking: v == 0.0 is inside. A vertex on the SDF
        // surface routes to the inside set; a strict-inequality
        // implementation would route it to outside and produce a
        // different topology. Detector for the D-8 rule.
        assert_eq!(case_index([0.0, 1.0, 1.0, 1.0]), 1);
        assert_eq!(case_index([0.0, 0.0, 0.0, 0.0]), 15);
    }

    #[test]
    fn sub_tets_count_per_case_matches_wikipedia_topology() {
        // Wikipedia's MT case enumeration: 0/15 trivial, 4 single-
        // vertex-inside cases each producing 1 sub-tet (corner cap),
        // 6 two-vertices-inside cases each producing 3 sub-tets
        // (prism), 4 three-vertices-inside cases each producing 3
        // sub-tets (frustum). Locking these counts catches any future
        // edit to SUB_TETS_BY_CASE that subtly changes topology.
        assert_eq!(sub_tets_for_case(0).len(), 0);
        assert_eq!(sub_tets_for_case(15).len(), 1);
        for case in [1, 2, 4, 8] {
            assert_eq!(
                sub_tets_for_case(case).len(),
                1,
                "1-inside case {case} should produce 1 corner sub-tet",
            );
        }
        for case in [3, 5, 6, 9, 10, 12] {
            assert_eq!(
                sub_tets_for_case(case).len(),
                3,
                "2-inside case {case} should produce 3 prism sub-tets",
            );
        }
        for case in [7, 11, 13, 14] {
            assert_eq!(
                sub_tets_for_case(case).len(),
                3,
                "3-inside case {case} should produce 3 frustum sub-tets",
            );
        }
    }

    #[test]
    fn case_15_keeps_parent_tet_verbatim() {
        // Trivial detector for the all-inside path — must emit the
        // parent's own four vertices in their original slot order.
        assert_eq!(sub_tets_for_case(15), &[[0, 1, 2, 3]]);
    }

    #[test]
    fn case_1_corner_sub_tet_at_v0() {
        // Memo §2 marching_tet row: 1-inside case at v0 emits the
        // sub-tet [v0, c01, c02, c03]. Verifies slot encoding and the
        // canonical edge-order convention (slot 4=c01, 5=c02, 6=c03).
        assert_eq!(sub_tets_for_case(1), &[[0, 4, 5, 6]]);
    }

    #[test]
    fn edge_endpoints_canonical_order() {
        // 6 cut slots (4..=9) map to the 6 tet edges in the canonical
        // order matching mesh::quality::compute_metrics's dihedral
        // enumeration. Drift here would silently route cut points to
        // the wrong edge endpoints.
        assert_eq!(edge_endpoints(4), (0, 1));
        assert_eq!(edge_endpoints(5), (0, 2));
        assert_eq!(edge_endpoints(6), (0, 3));
        assert_eq!(edge_endpoints(7), (1, 2));
        assert_eq!(edge_endpoints(8), (1, 3));
        assert_eq!(edge_endpoints(9), (2, 3));
    }

    #[test]
    #[should_panic(expected = "not a cut-point slot")]
    fn edge_endpoints_rejects_parent_vertex_slot() {
        let _ = edge_endpoints(2);
    }

    #[test]
    fn cut_parameter_midpoint_at_symmetric_signs() {
        // SDF flips sign with equal magnitude across the edge → cut at
        // exactly t = 0.5, mid-edge.
        assert!((cut_parameter(-1.0, 1.0) - 0.5).abs() < 1e-15);
    }

    #[test]
    fn cut_parameter_skewed_root_within_unit_interval() {
        // SDF root closer to v_a than to v_b → t closer to 0.
        // For v_a = -1, v_b = 2: root at t = 1/3.
        let t = cut_parameter(-1.0, 2.0);
        assert!((t - 1.0 / 3.0).abs() < 1e-15);
        // And the symmetric case: v_a = -2, v_b = 1 → t = 2/3.
        let t = cut_parameter(-2.0, 1.0);
        assert!((t - 2.0 / 3.0).abs() < 1e-15);
    }

    #[test]
    fn cut_parameter_at_endpoint_a_returns_zero() {
        // v_a = 0.0 is an in-edge SDF zero (D-8 tie-breaking treats it
        // as inside, but the cut-parameter formula puts the cut AT
        // endpoint a because t = -0/(v_b - 0) = 0). Lattice-on-surface
        // coincidence path.
        let t = cut_parameter(0.0, 1.0);
        assert!(t.abs() < 1e-15);
    }

    #[test]
    fn cut_parameter_denom_near_zero_falls_back_to_midpoint() {
        // Both endpoints FP-near-zero of the same sign → the cut
        // formula has no well-defined root; the guard returns 0.5
        // instead of dividing by a tiny denominator. Mirrors
        // cf-design's interpolate_vertex guard. The sub-tet that
        // would consume this cut is filtered downstream by D-10
        // (signed_volume < EPSILON_VOLUME).
        assert!((cut_parameter(0.0, 0.0) - 0.5).abs() < 1e-15);
        assert!((cut_parameter(1e-30, 1e-30) - 0.5).abs() < 1e-15);
        assert!((cut_parameter(1.0, 1.0) - 0.5).abs() < 1e-15);
    }

    #[test]
    fn cut_parameter_result_stays_in_unit_interval() {
        // Regression net for the post-condition `t ∈ [0, 1]`. Under
        // the in/out classification rule (D-8), an edge crossing the
        // SDF zero has endpoints of opposite sign and the analytic
        // root is always in [0, 1] — the explicit `.clamp(0.0, 1.0)`
        // in cut_parameter is defensive, absorbing rare round-off
        // pushes near zero rather than reshaping valid roots. This
        // test exercises an extreme-magnitude opposite-sign input
        // (1e-20, -1e-300) that lands at the edge of the interval and
        // confirms the post-condition holds; it does NOT prove the
        // clamp fires (constructing such an input is hard given
        // IEEE-754 rounding behavior on this formula).
        let t = cut_parameter(1e-20, -1e-300);
        assert!((0.0..=1.0).contains(&t));
    }

    #[test]
    fn signed_volume_canonical_decimeter_matches_l_cubed_over_six() {
        // Sanity check against the analytic value V = L³ / 6 for a
        // corner tet of edge L. Locks the closed-form magnitude to
        // the canonical convention; the post-hoc orientation swap
        // only consults the sign, but a hidden factor-of-2 or sign
        // flip here would surface.
        let p0 = Vec3::new(0.0, 0.0, 0.0);
        let p1 = Vec3::new(0.1, 0.0, 0.0);
        let p2 = Vec3::new(0.0, 0.1, 0.0);
        let p3 = Vec3::new(0.0, 0.0, 0.1);
        let expected = 0.1_f64.powi(3) / 6.0;
        assert!((signed_volume(p0, p1, p2, p3) - expected).abs() < 1e-15);
    }

    #[test]
    fn signed_volume_flips_sign_under_last_two_swap() {
        // Post-hoc orientation correction in sdf_meshed_tet_mesh
        // swaps slots 2 and 3 to flip orientation; the closed-form
        // determinant must satisfy that property.
        let p0 = Vec3::new(0.0, 0.0, 0.0);
        let p1 = Vec3::new(0.1, 0.0, 0.0);
        let p2 = Vec3::new(0.0, 0.1, 0.0);
        let p3 = Vec3::new(0.0, 0.0, 0.1);
        let rh = signed_volume(p0, p1, p2, p3);
        let lh = signed_volume(p0, p1, p3, p2);
        assert!((rh + lh).abs() < 1e-15);
        assert!(rh > 0.0);
        assert!(lh < 0.0);
    }

    #[test]
    fn epsilon_volume_default_is_one_femto_cubic_metre() {
        // Filter threshold is a fixed constant per memo §3 D-10. If
        // anyone tunes it, this test forces an explicit re-decision.
        assert!((EPSILON_VOLUME - 1e-15).abs() < f64::EPSILON);
    }
}
