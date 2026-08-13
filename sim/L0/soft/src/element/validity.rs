//! Exact rest-Jacobian validity for [`Tet10`] — certify, do not sample.
//!
//! Every other validity check in this crate answers "is this element valid?"
//! by evaluating `det J` at a handful of parametric points and hoping the
//! points it chose are representative. They are not, and no larger finite set
//! would be: for a curved Tet10, `det J` is a **cubic**, free to dip below
//! zero between any two samples. That is not a tuning problem, it is the
//! method — which is why this module replaces it rather than adding points.
//!
//! # The bound
//!
//! The isoparametric map is quadratic, so `J(ξ) = x_refᵀ ∇N(ξ)` has **affine**
//! entries. Writing barycentrics `λ = (1−ξ−η−ζ, ξ, η, ζ)`, affineness gives
//!
//! ```text
//!     J(λ) = Σ_i λ_i J_i,        J_i := J at simplex vertex i
//! ```
//!
//! and the determinant, being trilinear and alternating in its columns,
//! expands column-wise into
//!
//! ```text
//!     det J(λ) = Σ_{p,q,r} λ_p λ_q λ_r · M(p,q,r)
//!     M(p,q,r) := det[ col₀(J_p) | col₁(J_q) | col₂(J_r) ]
//! ```
//!
//! Collecting those 64 terms by multi-index `α = e_p + e_q + e_r` and moving
//! from monomials to the Bernstein basis `B³_α = (3!/α!) λ^α` yields the 20
//! coefficients returned by [`rest_det_j_coefficients`]:
//!
//! ```text
//!     b_α = (α! / 3!) · Σ_{e_p+e_q+e_r = α} M(p,q,r)
//! ```
//!
//! The Bernstein basis is non-negative and partitions unity on the simplex, so
//!
//! ```text
//!     min_α b_α  ≤  det J(ξ)  ≤  max_α b_α       for EVERY ξ in the element
//! ```
//!
//! `min_α b_α > 0` is therefore a proof of validity over the whole element,
//! not a sample of it. Two structural facts follow and are worth knowing:
//!
//! - The four corner coefficients are **interpolating** (`b_{3e_i} = det J` at
//!   vertex `i`), so the four reference-corner samples that
//!   `Tet10Mesh::with_projected_midsides` checks are literally four of these
//!   twenty numbers. This primitive strictly subsumes that test.
//! - A **straight-edged** element has an affine map, so all twenty
//!   coefficients equal the constant `det J` and certification is immediate.
//!
//! # Subdivision is four Jacobian evaluations
//!
//! The bracket is one-sided: `min_α b_α ≤ 0` means *not proven*, never
//! *refuted*. The remedy is to subdivide and re-bound, and here that is
//! unusually cheap — the derivation above never used the fact that the simplex
//! was the reference element, only that `J` is affine on it. So the
//! coefficients over *any* sub-tetrahedron come from the same formula applied
//! to `J` at that sub-tetrahedron's four vertices. There is no de Casteljau
//! step and no coefficient bookkeeping.
//!
//! Measured on the canonical layered sphere, subdivision never fires at all
//! (1.00 sub-simplices per element): real geometry is decided by the twenty
//! coefficients alone, and the recursion is the pathological tail.
//!
//! # Cost
//!
//! Roughly 1.6× the eight-point sampling guard it replaces, ~330 ns per
//! element — a whole 624-element mesh certifies in 0.3 ms. Exactness here is
//! not something the engine pays meaningfully for.
//!
//! # Scope
//!
//! Tet10 only, and the **rest** configuration. The bound depends on `J` being
//! affine in `ξ`, which is a property of quadratic shape functions;
//! [`Tet4`](super::Tet4) has a constant `J` and needs nothing. The deformed
//! configuration is a separate consumer, still sampling, and is not converted
//! here.

use nalgebra::{Matrix3, SMatrix};

use super::{Element, Tet10};
use crate::Vec3;

/// Degree-3 Bernstein coefficients on a 4-vertex simplex: `C(6, 3)`.
pub const DET_J_COEFFS: usize = 20;

/// Vertices of the parametric simplex, in this crate's corner convention
/// (local node 0 is the `1 − ξ − η − ζ` complement, `tet10.rs:16`).
const REFERENCE_SIMPLEX: [Vec3; 4] = [
    Vec3::new(0.0, 0.0, 0.0),
    Vec3::new(1.0, 0.0, 0.0),
    Vec3::new(0.0, 1.0, 0.0),
    Vec3::new(0.0, 0.0, 1.0),
];

/// Subdivision depth cap for [`certify_rest`].
///
/// Each level shrinks the bound gap quadratically, so an element still
/// undecided this deep is grazing the bar to within ~`2⁻⁶` of the element
/// size. Measured over 24 000 random curved elements, exactly one reached it.
const MAX_SUBDIVISION_DEPTH: u32 = 6;

/// Relative margin a certificate must clear, guarding against a "proof" that
/// is really floating-point noise.
///
/// The coefficients are exact to ~1e-15 relative (pinned by
/// `coefficients_reproduce_det_j`), so this is many orders below any real
/// geometric margin — the tightest genuine one measured on the canonical
/// sphere was 1.4e-2 — while still refusing to certify on rounding alone.
const CERTIFICATE_MARGIN: f64 = 1e-12;

/// The bar an element's rest Jacobian must clear everywhere.
#[derive(Clone, Copy, Debug)]
pub enum ValidityBar<'a> {
    /// `det J > 0` over the whole element: not folded, not degenerate.
    ///
    /// ⚠ Sound, but a poor bar to *bisect against*. A back-off that walks a
    /// node to the furthest position this bar admits lands it exactly on the
    /// `det J → 0⁺` boundary, manufacturing degenerate elements out of the
    /// search itself. Use [`ValidityBar::RelativeFloor`] wherever a search is
    /// steering toward the bar; this variant is for auditing geometry that is
    /// already fixed.
    Positive,
    /// `det J ≥ floor · det J_original` over the whole element, holding the
    /// element to a fraction of the quality it had before it was moved.
    ///
    /// `original` is the same element's node matrix in its reference state
    /// (for a midside projector, the straight-edged geometry it started
    /// from). The comparison is pointwise in `ξ`, and both sides are cubics
    /// over the same simplex, so the difference is bounded by the difference
    /// of the coefficient sets — no extra machinery.
    ///
    /// ⚠ `floor` must be in `[0, 1)` and `original` must itself be valid; a
    /// relative floor against an already-folded reference inverts its own
    /// comparison. [`certify_rest`] rejects both cases rather than trusting
    /// them.
    RelativeFloor {
        /// Node coordinates of the element before the move, row per node.
        original: &'a SMatrix<f64, 10, 3>,
        /// Fraction of the original determinant to hold, in `[0, 1)`.
        floor: f64,
    },
}

/// What [`certify_rest`] established about an element.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RestValidity {
    /// **Proof.** The bar holds at every point of the element.
    Certified {
        /// Smallest Bernstein coefficient relative to the largest in
        /// magnitude — how far the closest approach to the bar sits from it,
        /// and so how far the proof sits from rounding.
        margin: f64,
    },
    /// **Refutation, with the point that proves it.** No bound was trusted to
    /// produce this: `value` is an evaluated determinant (or determinant
    /// difference, under [`ValidityBar::RelativeFloor`]) at `at`.
    ///
    /// ⚠ Under [`ValidityBar::RelativeFloor`] this variant also reports an
    /// invalid **`original`**, in which case `at` and `value` describe the
    /// *reference* element rather than `x_ref`. That is deliberate: against a
    /// folded reference there is no meaningful floor to test `x_ref` for, so
    /// the answer is about the thing that is actually wrong. Callers reading
    /// the witness to blame a specific element must check their reference
    /// first, not assume `x_ref` is the folded one.
    Violated {
        /// Parametric point where the bar fails.
        at: Vec3,
        /// The bounded quantity there — non-positive, or non-finite.
        value: f64,
    },
    /// **Neither**, within the subdivision budget: the element grazes the bar
    /// too closely to separate. Callers must treat this as a failure — it is
    /// the fail-closed reading, and an element this marginal is not one to
    /// ship regardless.
    Undetermined,
}

impl RestValidity {
    /// Whether the element is *proven* to clear the bar.
    ///
    /// [`RestValidity::Undetermined`] reads `false`: absence of a proof is not
    /// a proof, and a caller that wanted the benefit of the doubt would be
    /// re-introducing exactly the guesswork this module removes.
    #[must_use]
    pub const fn is_certified(&self) -> bool {
        matches!(self, Self::Certified { .. })
    }
}

/// The 20 Bernstein–Bézier coefficients of `det J_rest` over the whole
/// element, where row `a` of `x_ref` is the rest position of local node `a`.
///
/// `min` and `max` of the returned array bracket `det J` over the entire
/// element — not at sample points, everywhere. The four values at the
/// reference corners appear among them exactly (see the module docs).
///
/// Callers wanting a verdict should use [`certify_rest`], which refines the
/// bracket when it is inconclusive; this is the raw bound, for censuses and
/// readouts that want a number rather than an answer.
#[must_use]
pub fn rest_det_j_coefficients(x_ref: &SMatrix<f64, 10, 3>) -> [f64; DET_J_COEFFS] {
    coefficients_over(&x_ref.transpose(), &REFERENCE_SIMPLEX)
}

/// Certify — or refute — that `x_ref` clears `bar` over the whole element.
///
/// Returns [`RestValidity::Certified`] only on proof and
/// [`RestValidity::Violated`] only with an evaluated witness point; the
/// remaining case is [`RestValidity::Undetermined`] and must be treated as
/// failure.
///
/// # Panics
///
/// Panics if a [`ValidityBar::RelativeFloor`] carries a `floor` outside
/// `[0, 1)`. A floor `≥ 1` would reject the original element itself and a
/// negative floor is meaningless — the same contract
/// `Tet10Mesh::with_projected_midsides` enforces.
#[must_use]
pub fn certify_rest(x_ref: &SMatrix<f64, 10, 3>, bar: ValidityBar<'_>) -> RestValidity {
    let reference = match bar {
        ValidityBar::Positive => None,
        ValidityBar::RelativeFloor { original, floor } => {
            assert!(
                (0.0..1.0).contains(&floor),
                "quality floor must be in [0, 1); got {floor}"
            );
            Some((original.transpose(), floor))
        }
    };

    // ⚠ A relative floor is only a quality floor while the geometry it is
    // relative to is healthy. For `det J_orig < 0` the bar `floor · orig` is
    // LESS negative than `orig` itself, so the comparison silently inverts
    // into "stay above a more-inverted number" and admits folded placements.
    // An element that arrives invalid is reported as such, never used as its
    // own reference. (The sibling guard in `with_projected_midsides` learned
    // this by measurement: it accepted a move that degraded a healthy sample
    // point by 94.5 % on a pre-folded element.)
    if let Some((xt_o, _)) = &reference {
        let original = certify_from(xt_o, None, REFERENCE_SIMPLEX, 0);
        if let RestValidity::Certified { .. } = original {
            // Healthy reference — fall through to the real question.
        } else {
            return original;
        }
    }

    certify_from(&x_ref.transpose(), reference.as_ref(), REFERENCE_SIMPLEX, 0)
}

/// Recursive certification over one sub-simplex of the reference element.
///
/// `xt` is the transposed node matrix; `reference` carries the original
/// element (already transposed) and its floor, or `None` for bare positivity.
fn certify_from(
    xt: &SMatrix<f64, 3, 10>,
    reference: Option<&(SMatrix<f64, 3, 10>, f64)>,
    verts: [Vec3; 4],
    depth: u32,
) -> RestValidity {
    let coeffs = bounded_coefficients(xt, reference, &verts);

    // ⚠ `f64::min` implements IEEE `minNum`, which *discards* a NaN operand
    // instead of propagating it. Folding the coefficients with it alone let a
    // NaN node coordinate produce a positive minimum and hence a certificate —
    // caught by `a_folded_element_is_refuted_and_a_healthy_one_certified`.
    // Finiteness is therefore tracked separately, never inferred from the fold.
    let mut lo = f64::INFINITY;
    let mut scale = 0.0f64;
    let mut all_finite = true;
    for &c in &coeffs {
        all_finite &= c.is_finite();
        lo = lo.min(c);
        scale = scale.max(c.abs());
    }
    // All twenty positive by a real margin: the bar holds everywhere below.
    if all_finite && lo > CERTIFICATE_MARGIN * scale {
        return RestValidity::Certified {
            margin: lo / if scale > 0.0 { scale } else { 1.0 },
        };
    }

    // The four corner coefficients are exact values of the bounded quantity,
    // so a non-positive one refutes outright. A non-finite one is no proof of
    // anything and must not slip through a bare comparison.
    for (i, vertex) in verts.iter().enumerate() {
        let value = corner_value(xt, reference, *vertex);
        if !(value.is_finite() && value > 0.0) {
            return RestValidity::Violated {
                at: verts[i],
                value,
            };
        }
    }

    // Non-finite coefficients with every corner finite: nothing here can be
    // proven and subdividing would only spend 8^depth sub-simplices reaching
    // the same answer. Fail closed now.
    if !all_finite || depth == MAX_SUBDIVISION_DEPTH {
        return RestValidity::Undetermined;
    }

    let mid = |a: usize, b: usize| (verts[a] + verts[b]) * 0.5;
    let (m01, m02, m03) = (mid(0, 1), mid(0, 2), mid(0, 3));
    let (m12, m13, m23) = (mid(1, 2), mid(1, 3), mid(2, 3));
    // Four corner children, plus the central octahedron split on the m01–m23
    // diagonal (whose equator cycle is m02, m03, m13, m12).
    let children: [[Vec3; 4]; 8] = [
        [verts[0], m01, m02, m03],
        [verts[1], m01, m12, m13],
        [verts[2], m02, m12, m23],
        [verts[3], m03, m13, m23],
        [m01, m23, m02, m03],
        [m01, m23, m03, m13],
        [m01, m23, m13, m12],
        [m01, m23, m12, m02],
    ];

    let mut undetermined = false;
    let mut worst_margin = f64::INFINITY;
    for child in children {
        match certify_from(xt, reference, child, depth + 1) {
            violated @ RestValidity::Violated { .. } => return violated,
            RestValidity::Undetermined => undetermined = true,
            RestValidity::Certified { margin } => worst_margin = worst_margin.min(margin),
        }
    }
    if undetermined {
        RestValidity::Undetermined
    } else {
        RestValidity::Certified {
            margin: worst_margin,
        }
    }
}

/// Bernstein coefficients of the bounded quantity over one sub-simplex:
/// `det J` on its own, or `det J − floor · det J_original`.
///
/// The Bernstein transform is linear, so the differenced polynomial's
/// coefficients are the difference of the two coefficient sets — the relative
/// floor costs one extra coefficient computation and a subtraction, not a
/// second algorithm.
fn bounded_coefficients(
    xt: &SMatrix<f64, 3, 10>,
    reference: Option<&(SMatrix<f64, 3, 10>, f64)>,
    verts: &[Vec3; 4],
) -> [f64; DET_J_COEFFS] {
    let mut coeffs = coefficients_over(xt, verts);
    if let Some((xt_o, floor)) = reference {
        let original = coefficients_over(xt_o, verts);
        for (c, o) in coeffs.iter_mut().zip(original) {
            *c = floor.mul_add(-o, *c);
        }
    }
    coeffs
}

/// The bounded quantity evaluated exactly at one parametric point.
fn corner_value(
    xt: &SMatrix<f64, 3, 10>,
    reference: Option<&(SMatrix<f64, 3, 10>, f64)>,
    xi: Vec3,
) -> f64 {
    let grad = Tet10.shape_gradients(xi);
    let value = (xt * grad).determinant();
    match reference {
        None => value,
        Some((xt_o, floor)) => floor.mul_add(-(xt_o * grad).determinant(), value),
    }
}

/// Bernstein coefficients of `det J` over an arbitrary sub-simplex of the
/// reference element, given its four vertices in parametric coordinates.
///
/// Correct for any sub-simplex because `J` is affine in `ξ`: the four vertex
/// Jacobians determine it everywhere between them.
fn coefficients_over(xt: &SMatrix<f64, 3, 10>, verts: &[Vec3; 4]) -> [f64; DET_J_COEFFS] {
    let j: [Matrix3<f64>; 4] = std::array::from_fn(|i| xt * Tet10.shape_gradients(verts[i]));
    let col = |m: &Matrix3<f64>, k: usize| Vec3::new(m[(0, k)], m[(1, k)], m[(2, k)]);
    let c0: [Vec3; 4] = std::array::from_fn(|i| col(&j[i], 0));
    let c1: [Vec3; 4] = std::array::from_fn(|i| col(&j[i], 1));
    let c2: [Vec3; 4] = std::array::from_fn(|i| col(&j[i], 2));

    // 16 shared cross products turn the 64 mixed determinants into 64 dots.
    let cross: [[Vec3; 4]; 4] =
        std::array::from_fn(|q| std::array::from_fn(|r| c1[q].cross(&c2[r])));

    // Monomial sums S_α, keyed by (α1, α2, α3) — α0 is implied by |α| = 3.
    let mut mono = [0.0f64; 64];
    for (p, c0p) in c0.iter().enumerate() {
        for q in 0..4 {
            for r in 0..4 {
                let mut a = [0usize; 4];
                a[p] += 1;
                a[q] += 1;
                a[r] += 1;
                mono[a[1] + 4 * a[2] + 16 * a[3]] += c0p.dot(&cross[q][r]);
            }
        }
    }

    // b_α = S_α · α! / 3!, in a fixed (α1, α2, α3) enumeration order.
    let mut out = [0.0f64; DET_J_COEFFS];
    let mut slot = 0usize;
    for a1 in 0..=3usize {
        for a2 in 0..=(3 - a1) {
            for a3 in 0..=(3 - a1 - a2) {
                let a0 = 3 - a1 - a2 - a3;
                out[slot] = mono[a1 + 4 * a2 + 16 * a3] * multi_factorial(a0, a1, a2, a3) / 6.0;
                slot += 1;
            }
        }
    }
    debug_assert_eq!(slot, DET_J_COEFFS, "degree-3 enumeration is complete");
    out
}

/// `α!` for a degree-3 multi-index with parts `(a0, a1, a2, a3)`.
fn multi_factorial(a0: usize, a1: usize, a2: usize, a3: usize) -> f64 {
    const FACT: [f64; 4] = [1.0, 1.0, 2.0, 6.0];
    FACT[a0.min(3)] * FACT[a1.min(3)] * FACT[a2.min(3)] * FACT[a3.min(3)]
}

#[cfg(test)]
#[allow(
    // `bits() >> 11` is at most 2^53 and the divisor is exactly 2^53, so the
    // RNG's conversion is exact. The lint fires on the type pair, not on this
    // use of it.
    clippy::cast_precision_loss,
    // A fixture search that comes back empty is a drifted fixture, and the
    // test that depends on it must fail loudly rather than pass vacuously.
    // Same convention as the sibling `tet10_sdf_projection` suite.
    clippy::expect_used
)]
mod tests {
    use super::*;
    use crate::element::TET10_EDGE_NODES;

    /// Deterministic `SplitMix64` — fixtures must be reproducible run to run.
    struct Rng(u64);

    impl Rng {
        fn bits(&mut self) -> u64 {
            self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
            let mut z = self.0;
            z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            z ^ (z >> 31)
        }
        fn unit(&mut self) -> f64 {
            (self.bits() >> 11) as f64 / (1u64 << 53) as f64
        }
        fn sym(&mut self) -> f64 {
            2.0f64.mul_add(self.unit(), -1.0)
        }
        fn vec(&mut self) -> Vec3 {
            Vec3::new(self.sym(), self.sym(), self.sym())
        }
    }

    /// A curved Tet10: jittered corners, every midside displaced off its
    /// straight midpoint by up to `curve` of that edge's length.
    ///
    /// Several midsides moved in unrelated directions is what makes `det J`
    /// genuinely cubic — a *single* displaced midside leaves it linear, the
    /// degenerate case corner samples already bound exactly.
    fn curved_element(rng: &mut Rng, curve: f64) -> SMatrix<f64, 10, 3> {
        let base = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.5, 0.866, 0.0),
            Vec3::new(0.5, 0.289, 0.816),
        ];
        let corners: [Vec3; 4] = std::array::from_fn(|i| base[i] + rng.vec() * 0.12);
        let mut nodes = [Vec3::zeros(); 10];
        nodes[..4].copy_from_slice(&corners);
        for (i, &(a, b)) in TET10_EDGE_NODES.iter().enumerate() {
            let mid = (corners[a] + corners[b]) * 0.5;
            let len = (corners[a] - corners[b]).norm();
            nodes[4 + i] = mid + rng.vec() * (curve * len);
        }
        SMatrix::<f64, 10, 3>::from_fn(|a, k| nodes[a][k])
    }

    /// The same element with every midside returned to its straight midpoint.
    fn straightened(x: &SMatrix<f64, 10, 3>) -> SMatrix<f64, 10, 3> {
        let mut out = *x;
        let corners: [Vec3; 4] =
            std::array::from_fn(|a| Vec3::new(x[(a, 0)], x[(a, 1)], x[(a, 2)]));
        for (i, &(a, b)) in TET10_EDGE_NODES.iter().enumerate() {
            let mid = (corners[a] + corners[b]) * 0.5;
            for k in 0..3 {
                out[(4 + i, k)] = mid[k];
            }
        }
        out
    }

    /// `det J` evaluated directly, the way every other consumer evaluates it.
    fn det_at(x: &SMatrix<f64, 10, 3>, xi: Vec3) -> f64 {
        (x.transpose() * Tet10.shape_gradients(xi)).determinant()
    }

    /// Barycentric lattice of order `n` over the reference simplex —
    /// `C(n+3, 3)` points. An independent sampler, used only to check that the
    /// bracket encloses what it claims to.
    fn lattice(n: u32) -> Vec<Vec3> {
        let inv = 1.0 / f64::from(n);
        let mut out = Vec::new();
        for i in 0..=n {
            for k in 0..=(n - i) {
                for l in 0..=(n - i - k) {
                    out.push(Vec3::new(
                        f64::from(i) * inv,
                        f64::from(k) * inv,
                        f64::from(l) * inv,
                    ));
                }
            }
        }
        out
    }

    #[test]
    fn coefficients_reproduce_det_j() {
        // The Bernstein form must BE det J, not approximate it. Evaluate the
        // basis expansion against a direct determinant at interior points.
        let mut rng = Rng(0x5EED_0001);
        let mut worst = 0.0f64;
        for _ in 0..200 {
            let x = curved_element(&mut rng, 0.25);
            let coeffs = rest_det_j_coefficients(&x);
            let scale = coeffs.iter().fold(0.0f64, |m, c| m.max(c.abs()));

            for _ in 0..25 {
                let (mut a, mut b, mut c) = (rng.unit(), rng.unit(), rng.unit());
                let total = a + b + c;
                if total > 1.0 {
                    a /= total + 1e-12;
                    b /= total + 1e-12;
                    c /= total + 1e-12;
                }
                let lam = [1.0 - a - b - c, a, b, c];

                let mut acc = 0.0;
                let mut slot = 0usize;
                for a1 in 0..=3usize {
                    for a2 in 0..=(3 - a1) {
                        for a3 in 0..=(3 - a1 - a2) {
                            let a0 = 3 - a1 - a2 - a3;
                            let pow = |x: f64, k: usize| (0..k).fold(1.0, |acc, _| acc * x);
                            acc += coeffs[slot]
                                * (6.0 / multi_factorial(a0, a1, a2, a3))
                                * pow(lam[0], a0)
                                * pow(lam[1], a1)
                                * pow(lam[2], a2)
                                * pow(lam[3], a3);
                            slot += 1;
                        }
                    }
                }
                worst = worst.max((acc - det_at(&x, Vec3::new(a, b, c))).abs() / scale.max(1e-300));
            }
        }
        assert!(
            worst < 1e-13,
            "the Bernstein form must be exact; worst relative error = {worst:e}"
        );
    }

    #[test]
    fn a_straight_edged_element_has_twenty_equal_coefficients() {
        // An affine map has constant det J, so the bracket collapses to a
        // point and certification is immediate.
        let mut rng = Rng(0x5EED_0002);
        for _ in 0..100 {
            let x = straightened(&curved_element(&mut rng, 0.3));
            let coeffs = rest_det_j_coefficients(&x);
            let constant = Tet10.rest_jacobian_dets(&x)[0];
            let spread = coeffs
                .iter()
                .fold(0.0f64, |m, c| m.max((c - constant).abs()));
            assert!(
                spread <= 1e-12 * constant.abs(),
                "straight element coefficient spread = {spread:e}"
            );
        }
    }

    #[test]
    fn the_bracket_encloses_the_element() {
        // min/max of the coefficients must bound det J everywhere, checked
        // against a dense lattice the primitive never sees.
        let mut rng = Rng(0x5EED_0003);
        let points = lattice(12);
        for _ in 0..150 {
            let x = curved_element(&mut rng, 0.3);
            let coeffs = rest_det_j_coefficients(&x);
            let lo = coeffs.iter().copied().fold(f64::INFINITY, f64::min);
            let hi = coeffs.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            let scale = hi.abs().max(lo.abs()).max(1e-300);
            for &xi in &points {
                let d = det_at(&x, xi);
                assert!(
                    d >= lo - 1e-12 * scale && d <= hi + 1e-12 * scale,
                    "det J = {d:e} escapes the bracket [{lo:e}, {hi:e}]"
                );
            }
        }
    }

    #[test]
    fn the_corner_coefficients_are_the_corner_determinants() {
        // The interpolation property: the four reference-corner samples the
        // sibling mesher guard checks are four of these twenty numbers.
        let mut rng = Rng(0x5EED_0004);
        for _ in 0..100 {
            let x = curved_element(&mut rng, 0.3);
            let coeffs = rest_det_j_coefficients(&x);
            for corner in REFERENCE_SIMPLEX {
                let exact = det_at(&x, corner);
                assert!(
                    coeffs
                        .iter()
                        .any(|&c| (c - exact).abs() <= 1e-12 * exact.abs().max(1e-12)),
                    "corner determinant {exact:e} is not among the coefficients"
                );
            }
        }
    }

    #[test]
    fn sampling_accepts_elements_this_refutes() {
        // ★ The reason this module exists, as an object rather than an
        // argument: an element whose four Gauss points AND four reference
        // corners are all positive, and which folds anyway.
        let mut rng = Rng(0x5EED_0005);
        let corner_grads: [SMatrix<f64, 10, 3>; 4] =
            REFERENCE_SIMPLEX.map(|xi| Tet10.shape_gradients(xi));

        let witness = (0..3000).find_map(|_| {
            let x = curved_element(&mut rng, 0.15);
            let xt = x.transpose();
            let gauss_ok = Tet10.rest_jacobian_dets(&x).iter().all(|&d| d > 0.0);
            let corners_ok = corner_grads.iter().all(|g| (xt * g).determinant() > 0.0);
            if !(gauss_ok && corners_ok) {
                return None;
            }
            match certify_rest(&x, ValidityBar::Positive) {
                RestValidity::Violated { at, value } => Some((x, at, value)),
                _ => None,
            }
        });

        let (x, at, value) = witness.expect(
            "the deterministic population must contain an element every sampled point accepts \
             and that is provably folded — if it no longer does, the fixture drifted and this \
             module's reason for existing is no longer demonstrated",
        );
        assert!(
            value <= 0.0,
            "the witness must be a real evaluation, not a bound: {value:e}"
        );
        // And the witness point is genuinely in the element and genuinely bad.
        let recomputed = det_at(&x, at);
        assert!(
            (recomputed - value).abs() <= 1e-9 * value.abs().max(1e-12),
            "the reported witness value must be reproducible by direct evaluation"
        );
    }

    #[test]
    fn a_folded_element_is_refuted_and_a_healthy_one_certified() {
        // ⚠ Negative control. A certifier that answered `Certified`
        // unconditionally would pass every other test in this module that
        // only feeds it healthy geometry.
        let mut rng = Rng(0x5EED_0006);

        // Swapping two corners inverts a straight element everywhere.
        let mut inverted = straightened(&curved_element(&mut rng, 0.0));
        for k in 0..3 {
            let (a, b) = (inverted[(1, k)], inverted[(2, k)]);
            inverted[(1, k)] = b;
            inverted[(2, k)] = a;
        }
        let inverted = straightened(&inverted);
        assert!(
            matches!(
                certify_rest(&inverted, ValidityBar::Positive),
                RestValidity::Violated { .. }
            ),
            "a corner-swapped straight tet is inverted everywhere"
        );

        // A non-finite coordinate is never certified.
        let mut poisoned = curved_element(&mut rng, 0.05);
        poisoned[(7, 1)] = f64::NAN;
        assert!(
            !certify_rest(&poisoned, ValidityBar::Positive).is_certified(),
            "a NaN node coordinate must not yield a certificate"
        );

        // ...and it is not simply refusing everything.
        let healthy = curved_element(&mut rng, 0.02);
        assert!(
            certify_rest(&healthy, ValidityBar::Positive).is_certified(),
            "a gently curved element must certify, or the control above is vacuous"
        );
    }

    #[test]
    fn the_relative_floor_bounds_the_differenced_polynomial() {
        // The production bar. `det J_c − floor · det J_o` is a cubic over the
        // same simplex and the Bernstein transform is linear, so differencing
        // the coefficient sets bounds it — checked against a dense lattice.
        let mut rng = Rng(0x5EED_0007);
        let floor = 0.4_f64;
        let points = lattice(12);
        for _ in 0..150 {
            let curved = curved_element(&mut rng, 0.2);
            let original = straightened(&curved);
            let (xt_c, xt_o) = (curved.transpose(), original.transpose());

            let b_c = coefficients_over(&xt_c, &REFERENCE_SIMPLEX);
            let b_o = coefficients_over(&xt_o, &REFERENCE_SIMPLEX);
            let bound = (0..DET_J_COEFFS)
                .map(|i| floor.mul_add(-b_o[i], b_c[i]))
                .fold(f64::INFINITY, f64::min);
            let scale = b_c.iter().fold(0.0f64, |m, c| m.max(c.abs())).max(1e-300);

            for &xi in &points {
                let q = floor.mul_add(-det_at(&original, xi), det_at(&curved, xi));
                assert!(
                    q >= bound - 1e-12 * scale,
                    "differenced coefficients must bound the differenced polynomial"
                );
            }
        }
    }

    #[test]
    fn the_relative_floor_is_stricter_than_positivity() {
        // Non-vacuity: the floor must actually reject elements that bare
        // positivity accepts, or wiring it in changes nothing.
        let mut rng = Rng(0x5EED_0008);
        let mut stricter = 0usize;
        for _ in 0..2000 {
            let curved = curved_element(&mut rng, 0.2);
            let original = straightened(&curved);
            if !certify_rest(&original, ValidityBar::Positive).is_certified() {
                continue;
            }
            let positive = certify_rest(&curved, ValidityBar::Positive).is_certified();
            let floored = certify_rest(
                &curved,
                ValidityBar::RelativeFloor {
                    original: &original,
                    floor: 0.4,
                },
            )
            .is_certified();
            assert!(
                !floored || positive,
                "clearing a floor above zero implies clearing zero"
            );
            stricter += usize::from(positive && !floored);
        }
        assert!(
            stricter > 0,
            "the relative floor must reject somebody bare positivity accepts"
        );
    }

    #[test]
    fn an_invalid_reference_is_reported_not_trusted() {
        // ⚠ A relative floor against a folded reference inverts its own
        // comparison — `floor · o` is LESS negative than `o`. The bar must
        // refuse the reference rather than compare against it.
        let mut rng = Rng(0x5EED_0009);
        let mut inverted = straightened(&curved_element(&mut rng, 0.0));
        for k in 0..3 {
            let (a, b) = (inverted[(1, k)], inverted[(2, k)]);
            inverted[(1, k)] = b;
            inverted[(2, k)] = a;
        }
        let inverted = straightened(&inverted);
        let healthy = curved_element(&mut rng, 0.02);
        assert!(
            !certify_rest(
                &healthy,
                ValidityBar::RelativeFloor {
                    original: &inverted,
                    floor: 0.4,
                },
            )
            .is_certified(),
            "a folded reference must not be used as a quality baseline"
        );
    }

    #[test]
    #[should_panic(expected = "quality floor must be in [0, 1)")]
    fn a_floor_at_one_is_rejected() {
        let mut rng = Rng(0x5EED_000A);
        let x = curved_element(&mut rng, 0.05);
        let original = straightened(&x);
        let _rejected = certify_rest(
            &x,
            ValidityBar::RelativeFloor {
                original: &original,
                floor: 1.0,
            },
        );
    }
}
