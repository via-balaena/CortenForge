//! Linear tetrahedron (4 nodes, 1 Gauss point) — constant strain.
//!
//! Barycentric shape functions on the reference tet with parametric
//! coordinates `(ξ_1, ξ_2, ξ_3)` on the simplex `ξ_i ≥ 0, Σ ξ_i ≤ 1`:
//!
//! - `N_0 = 1 − ξ_1 − ξ_2 − ξ_3`, `N_1 = ξ_1`, `N_2 = ξ_2`, `N_3 = ξ_3`
//!
//! Gradients `∂N_a / ∂ξ_b` are constant on the element and form the
//! rows of the returned `SMatrix<f64, 4, 3>` — row `a` is node `a`'s
//! gradient with respect to all three parametric axes. One-point
//! quadrature at the centroid `ξ = (1/4, 1/4, 1/4)` with weight
//! `1/6` = reference-tet volume; exact for the constant integrands
//! Tet4 produces per Part 3 Ch 00 00-tet4.md §"No quadrature for Tet4".

use nalgebra::{SMatrix, SVector};

use super::{Element, RestValidity};
use crate::Vec3;

/// Linear tetrahedron element. Constant strain, 12 DOFs total.
#[derive(Clone, Copy, Debug, Default)]
pub struct Tet4;

impl Element<4, 1> for Tet4 {
    fn shape_functions(&self, xi: Vec3) -> SVector<f64, 4> {
        SVector::<f64, 4>::new(1.0 - xi.x - xi.y - xi.z, xi.x, xi.y, xi.z)
    }

    // Gradients are independent of `xi` (shape functions are linear in
    // barycentric coordinates). Row `a` = `∂N_a / ∂ξ`.
    fn shape_gradients(&self, _xi: Vec3) -> SMatrix<f64, 4, 3> {
        SMatrix::<f64, 4, 3>::new(
            -1.0, -1.0, -1.0, // ∂N_0 / ∂ξ
            1.0, 0.0, 0.0, // ∂N_1 / ∂ξ
            0.0, 1.0, 0.0, // ∂N_2 / ∂ξ
            0.0, 0.0, 1.0, // ∂N_3 / ∂ξ
        )
    }

    fn gauss_points(&self) -> [(Vec3, f64); 1] {
        // Centroid, weight = 1/6 = volume of the reference tet.
        [(Vec3::new(0.25, 0.25, 0.25), 1.0 / 6.0)]
    }

    /// Exact with no machinery. Tet4's map is affine, so `det J` is the same
    /// constant at every point of the element and the single centroid
    /// evaluation already answers for the whole of it — there is no gap between
    /// sampling and certifying here, which is the property `Tet10` lacks.
    ///
    /// The margin is `1.0` because every Bernstein coefficient of a constant
    /// equals that constant, so the smallest is the largest: this is the widest
    /// margin the type can express, and it is the honest one.
    ///
    /// This impl exists so the solver's validity gate can be written once for
    /// both element types, not because Tet4 needed a certificate.
    fn certify_orientation(&self, x: &SMatrix<f64, 4, 3>) -> RestValidity {
        let value = self.rest_jacobian_dets(x)[0];
        // `is_finite` as well as `> 0.0`: every comparison against NaN is
        // false, so a bare `> 0.0` would fall through to `Violated` correctly
        // here — but stating finiteness keeps this impl's contract identical to
        // Tet10's, where the fold over coefficients cannot infer it.
        if value.is_finite() && value > 0.0 {
            RestValidity::Certified { margin: 1.0 }
        } else {
            RestValidity::Violated {
                at: self.gauss_points()[0].0,
                value,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    // Same convention as `solver::backward_euler::tests`: a test's `panic!` in a
    // match fallback IS the assertion, and `float_cmp` is deliberate here — an
    // affine map's determinant is one exact constant, so `== 0.0` on a coplanar
    // tet is a statement about the arithmetic, not a tolerance judgement.
    #![allow(clippy::panic, clippy::float_cmp)]

    use super::*;

    /// A tet's node matrix, row per node.
    fn nodes(v: [[f64; 3]; 4]) -> SMatrix<f64, 4, 3> {
        SMatrix::<f64, 4, 3>::from_fn(|a, k| v[a][k])
    }

    /// The reference tet, right-handed: `det J = 1 > 0`.
    const RIGHT_HANDED: [[f64; 3]; 4] = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ];

    /// ★ The four verdicts [`Tet4::certify_orientation`] can reach, asserted directly.
    ///
    /// The solver exercises this impl only through its validity gate, and only on
    /// right-handed meshes — so until this existed, three of the four arms were
    /// reached by no test at all. The left-handed case in particular is load-bearing
    /// elsewhere: `tests/multi_material_validity.rs` states that a left-handed REST
    /// tet is now rejected on its own account, and no mesh constructor in this crate
    /// will build one, so this is where that claim can be checked.
    #[test]
    fn certify_orientation_reaches_every_verdict_it_can() {
        // Right-handed: certified, and with the widest margin the type expresses —
        // every Bernstein coefficient of a constant equals that constant.
        match Tet4.certify_orientation(&nodes(RIGHT_HANDED)) {
            RestValidity::Certified { margin } => assert!(
                (margin - 1.0).abs() < 1e-15,
                "an affine map has one determinant everywhere, so the margin is 1.0; got {margin}"
            ),
            other => panic!("a right-handed tet must certify, got {other:?}"),
        }

        // Left-handed: two corners swapped flips the sign of `det J`.
        let mut left = RIGHT_HANDED;
        left.swap(1, 2);
        match Tet4.certify_orientation(&nodes(left)) {
            RestValidity::Violated { value, .. } => assert!(
                value < 0.0,
                "the witness must carry the negative determinant that refutes it; got {value}"
            ),
            other => panic!("a left-handed tet must be refuted, got {other:?}"),
        }

        // Degenerate: corner 3 laid onto the opposite face gives `det J == 0`, which
        // `> 0.0` must reject. A `>= 0.0` predicate would certify a flat element.
        let mut flat = RIGHT_HANDED;
        flat[3] = [0.5, 0.5, 0.0];
        match Tet4.certify_orientation(&nodes(flat)) {
            RestValidity::Violated { value, .. } => assert!(
                value == 0.0,
                "a coplanar tet has det J exactly 0; got {value}"
            ),
            other => panic!("a degenerate tet must be refuted, got {other:?}"),
        }

        // Non-finite: every comparison against NaN is false, so a bare `> 0.0` falls
        // through to the refutation arm — which is the right answer, and is asserted
        // rather than assumed because the sibling `Tet10` impl needs an explicit
        // finiteness track to get here (`f64::min` discards NaN).
        let mut poisoned = RIGHT_HANDED;
        poisoned[1][0] = f64::NAN;
        match Tet4.certify_orientation(&nodes(poisoned)) {
            RestValidity::Violated { value, .. } => {
                assert!(
                    value.is_nan(),
                    "the witness must carry the NaN; got {value}"
                );
            }
            other => panic!("a non-finite tet must be refuted, got {other:?}"),
        }
    }

    /// The certificate agrees with the sampled predicate it generalises.
    ///
    /// For `Tet4` the two are the same number by construction — `G == 1` and the map
    /// is affine — and that identity is what lets the solver's gate be written once
    /// for both element types. Pinned so a future edit to either cannot drift.
    #[test]
    fn the_certificate_and_the_single_gauss_determinant_are_the_same_number() {
        for v in [
            RIGHT_HANDED,
            [[0.0; 3], [2.0, 0.0, 0.0], [0.0, 3.0, 0.0], [0.0, 0.0, 5.0]],
        ] {
            let x = nodes(v);
            let det = Tet4.rest_jacobian_dets(&x)[0];
            assert!(
                det > 0.0 && Tet4.certify_orientation(&x).is_certified(),
                "both must accept the same right-handed tet (det = {det})"
            );
        }
    }
}
