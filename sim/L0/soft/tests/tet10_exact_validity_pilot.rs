//! PILOT — exact Tet10 validity by Bernstein bounds, priced against sampling.
//!
//! This file is a **measurement**, not a shipped gate. It exists to answer one
//! question before any production code changes: *what does it cost to certify
//! `det J > 0` over a whole element, instead of sampling it at a few points?*
//!
//! # Why sampling cannot be fixed by adding points
//!
//! For a Tet10 the isoparametric map is quadratic, so `J(ξ) = x_refᵀ ∇N(ξ)`
//! has **affine** entries and `det J` is a **cubic** in `ξ`. No finite sample
//! set bounds a cubic: between any two samples it is free to dip. The current
//! mesher guard samples eight points (4 Stroud + 4 reference corners) and is
//! sound at exactly those eight points and nowhere else.
//!
//! # The primitive
//!
//! Write barycentrics `λ = (1−ξ−η−ζ, ξ, η, ζ)`. Because `J` is affine,
//!
//! ```text
//!     J(λ) = Σ_i λ_i J_i,        J_i := J at reference corner i
//! ```
//!
//! and expanding the determinant column-wise (it is trilinear alternating in
//! its columns),
//!
//! ```text
//!     det J(λ) = Σ_{p,q,r} λ_p λ_q λ_r · M(p,q,r)
//!     M(p,q,r) := det[ col₀(J_p) | col₁(J_q) | col₂(J_r) ]
//! ```
//!
//! Collecting the 64 terms by multi-index `α = e_p + e_q + e_r` and converting
//! monomials to the Bernstein basis `B³_α = (3!/α!) λ^α` gives the 20
//! coefficients
//!
//! ```text
//!     b_α = (α! / 3!) · Σ_{e_p+e_q+e_r = α} M(p,q,r)
//! ```
//!
//! The Bernstein basis is non-negative and sums to one on the simplex, so
//!
//! ```text
//!     min_α b_α  ≤  det J(ξ)  ≤  max_α b_α       for every ξ in the element
//! ```
//!
//! `min_α b_α > 0` is therefore a **proof** that the element is valid — over
//! the whole element, not at samples. The four corner coefficients are
//! interpolating (`b_{3e_i} = det J(corner i)`), so the corner half of the
//! current eight-point test is literally a subset of these twenty numbers.
//!
//! # Why subdivision is nearly free here
//!
//! The bound is not tight, so `min_α b_α ≤ 0` is inconclusive rather than a
//! refutation. The standard remedy is to subdivide and re-bound. The useful
//! accident is that **the derivation above never used the fact that the
//! simplex was the reference element** — only that `J` is affine on it. So the
//! coefficients of `det J` over *any* sub-tetrahedron come from the same
//! formula applied to `J` at that sub-tetrahedron's four vertices. There is no
//! de Casteljau step and no coefficient bookkeeping: subdivision is four more
//! Jacobian evaluations per child.
//!
//! # What is measured here
//!
//! 1. The coefficients reproduce `det J` to machine precision, interpolate at
//!    the corners, and enclose a dense lattice.
//! 2. Cost per element: 4-point Gauss vs. the shipped 8-point test vs. the
//!    20-coefficient certificate vs. the full recursive certifier.
//! 3. Verdict quality on a graded random population: how many true folds the
//!    8-point test misses, and how often the cheap depth-0 certificate alone
//!    settles the question.
//! 4. The same on real shipped geometry — the canonical layered sphere after
//!    `with_sdf_projected_boundary`.

#![allow(
    // Vertex/tet counts index the `Mesh` trait's `u32` id space; the canonical
    // sphere holds far fewer than `u32::MAX`. Mirrors `tet10_sdf_projection`.
    clippy::cast_possible_truncation,
    // Population counts are exact well below 2^53; the ratios printed from
    // them are report lines, not decision thresholds.
    clippy::cast_precision_loss,
    // Mesh construction failing is a meshing regression worth a panic.
    clippy::expect_used
)]

use std::time::Instant;

use nalgebra::{Matrix3, SMatrix};
use sim_soft::element::{Element, Tet10};
use sim_soft::{
    DifferenceSdf, LAYERED_SPHERE_R_CAVITY, LAYERED_SPHERE_R_OUTER, MaterialField, Mesh, SoftScene,
    SphereSdf, Tet10Mesh, Vec3,
};

// ---------------------------------------------------------------------------
// The candidate primitive
// ---------------------------------------------------------------------------

/// Corners of the parametric simplex, in the crate's corner convention
/// (node 0 is the `1 − ξ − η − ζ` complement).
const CORNERS: [Vec3; 4] = [
    Vec3::new(0.0, 0.0, 0.0),
    Vec3::new(1.0, 0.0, 0.0),
    Vec3::new(0.0, 1.0, 0.0),
    Vec3::new(0.0, 0.0, 1.0),
];

/// Number of degree-3 Bernstein coefficients on a 4-vertex simplex: `C(6,3)`.
const N_COEFFS: usize = 20;

/// `k!` for the only exponents a degree-3 multi-index can carry.
const fn factorial(k: usize) -> f64 {
    match k {
        0 | 1 => 1.0,
        2 => 2.0,
        _ => 6.0,
    }
}

/// `x^k` for `k ≤ 3` — avoids a `usize → i32` cast into `powi`.
fn pow_small(x: f64, k: usize) -> f64 {
    (0..k).fold(1.0, |acc, _| acc * x)
}

/// `α!` for the multi-index with parts `(a0, a1, a2, a3)`.
fn multi_factorial(a0: usize, a1: usize, a2: usize, a3: usize) -> f64 {
    factorial(a0) * factorial(a1) * factorial(a2) * factorial(a3)
}

/// `∇N` at the four reference corners — element constants, hoisted exactly as
/// the shipped mesher hoists its `corner_grads`.
fn reference_corner_grads() -> [SMatrix<f64, 10, 3>; 4] {
    CORNERS.map(|xi| Tet10.shape_gradients(xi))
}

/// `J` at the four vertices of an arbitrary sub-simplex of the reference
/// element. `J` is affine, so these four determine it everywhere inside.
fn jacobians_at(xt: &SMatrix<f64, 3, 10>, verts: &[Vec3; 4]) -> [Matrix3<f64>; 4] {
    std::array::from_fn(|i| xt * Tet10.shape_gradients(verts[i]))
}

/// The 20 Bernstein–Bézier coefficients of `det J` over the simplex whose
/// vertex Jacobians are `j`.
///
/// `min` and `max` of the returned array bracket `det J` over that whole
/// simplex; the four corner values appear among them exactly.
fn det_bernstein_coeffs(j: &[Matrix3<f64>; 4]) -> [f64; N_COEFFS] {
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

    // b_α = S_α · α! / 3!, enumerated in a fixed (α1, α2, α3) order.
    let mut out = [0.0f64; N_COEFFS];
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
    assert_eq!(
        slot, N_COEFFS,
        "the degree-3 multi-index enumeration is complete"
    );
    out
}

/// What the certifier concluded about an element.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Verdict {
    /// Proof: `det J > 0` everywhere in the element.
    Valid,
    /// Witness: some evaluated point has `det J ≤ 0`.
    Invalid,
    /// Neither, within the depth cap — the element is marginal.
    Undecided,
}

/// Exact validity by recursive Bernstein bounding.
///
/// Returns `Valid` only on proof (all coefficients positive over a cover of
/// the element) and `Invalid` only on a witness (an evaluated point at or
/// below zero). `subsimplices` counts the bounding steps taken, which is the
/// cost that matters.
fn certify(
    xt: &SMatrix<f64, 3, 10>,
    verts: [Vec3; 4],
    depth: u32,
    cap: u32,
    subsimplices: &mut usize,
) -> Verdict {
    *subsimplices += 1;
    let j = jacobians_at(xt, &verts);
    let b = det_bernstein_coeffs(&j);

    if b.iter().all(|&c| c > 0.0) {
        return Verdict::Valid;
    }
    // The four corner coefficients are true values of `det J`, so a
    // non-positive one is a witness, not a bound artifact. A non-finite one is
    // no proof of validity either, and must not slip through a bare `<=`.
    if j.iter().any(|m| {
        let d = m.determinant();
        !(d.is_finite() && d > 0.0)
    }) {
        return Verdict::Invalid;
    }
    if depth == cap {
        return Verdict::Undecided;
    }

    let m = |a: usize, c: usize| (verts[a] + verts[c]) * 0.5;
    let (m01, m02, m03) = (m(0, 1), m(0, 2), m(0, 3));
    let (m12, m13, m23) = (m(1, 2), m(1, 3), m(2, 3));
    // 4 corner children plus the central octahedron split on the m01–m23
    // diagonal (its equator cycle is m02, m03, m13, m12).
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

    let mut undecided = false;
    for child in children {
        match certify(xt, child, depth + 1, cap, subsimplices) {
            Verdict::Invalid => return Verdict::Invalid,
            Verdict::Undecided => undecided = true,
            Verdict::Valid => {}
        }
    }
    if undecided {
        Verdict::Undecided
    } else {
        Verdict::Valid
    }
}

/// Depth cap for the pilot's certifier. Each level shrinks the bound gap
/// quadratically, so anything still undecided at this depth is genuinely
/// touching zero.
const CAP: u32 = 6;

/// Timing repetitions per method in the cost pilot.
const REPS: usize = 200;

fn certify_element(xt: &SMatrix<f64, 3, 10>) -> (Verdict, usize) {
    let mut work = 0usize;
    let v = certify(xt, CORNERS, 0, CAP, &mut work);
    (v, work)
}

// ---------------------------------------------------------------------------
// Instruments the primitive is checked against
// ---------------------------------------------------------------------------

/// The eight points the shipped mesher guard holds above its floor: four
/// Stroud Gauss points, then the four reference corners.
fn shipped_eight(x_ref: &SMatrix<f64, 10, 3>, grads: &[SMatrix<f64, 10, 3>; 4]) -> [f64; 8] {
    let gauss = Tet10.rest_jacobian_dets(x_ref);
    let xt = x_ref.transpose();
    std::array::from_fn(|i| {
        if i < 4 {
            gauss[i]
        } else {
            (xt * grads[i - 4]).determinant()
        }
    })
}

/// `min det J` over a dense barycentric lattice of order `n`
/// (`C(n+3, 3)` points). An independent sampler, used only to check that the
/// Bernstein bracket actually encloses the function.
fn lattice_min_max(xt: &SMatrix<f64, 3, 10>, n: usize) -> (f64, f64) {
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    let inv = 1.0 / n as f64;
    for i in 0..=n {
        for k in 0..=(n - i) {
            for l in 0..=(n - i - k) {
                let xi = Vec3::new(i as f64 * inv, k as f64 * inv, l as f64 * inv);
                let d = (xt * Tet10.shape_gradients(xi)).determinant();
                lo = lo.min(d);
                hi = hi.max(d);
            }
        }
    }
    (lo, hi)
}

/// Deterministic `SplitMix64` — the population must be reproducible run to run.
struct Rng(u64);

impl Rng {
    const fn bits(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    /// Uniform on `[0, 1)`.
    fn unit(&mut self) -> f64 {
        (self.bits() >> 11) as f64 / (1u64 << 53) as f64
    }
    /// Uniform on `[-1, 1)`.
    fn sym(&mut self) -> f64 {
        2.0f64.mul_add(self.unit(), -1.0)
    }
    fn vec(&mut self) -> Vec3 {
        Vec3::new(self.sym(), self.sym(), self.sym())
    }
}

/// A curved Tet10: a jittered tetrahedron whose six midsides are displaced off
/// their straight midpoints by up to `curve` of the mean edge length.
///
/// Displacing several midsides in unrelated directions is what makes `det J`
/// genuinely cubic — a single displaced midside leaves it linear, which is the
/// degenerate case the corner samples already bound exactly.
fn random_curved_element(rng: &mut Rng, curve: f64) -> SMatrix<f64, 10, 3> {
    let base = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.5, 0.866, 0.0),
        Vec3::new(0.5, 0.289, 0.816),
    ];
    let corners: [Vec3; 4] = std::array::from_fn(|i| base[i] + rng.vec() * 0.12);
    let mut nodes = [Vec3::zeros(); 10];
    nodes[..4].copy_from_slice(&corners);
    for (i, &(a, b)) in sim_soft::element::TET10_EDGE_NODES.iter().enumerate() {
        let mid = (corners[a] + corners[b]) * 0.5;
        let len = (corners[a] - corners[b]).norm();
        nodes[4 + i] = mid + rng.vec() * (curve * len);
    }
    SMatrix::<f64, 10, 3>::from_fn(|a, k| nodes[a][k])
}

// ---------------------------------------------------------------------------
// 1. Correctness of the coefficients
// ---------------------------------------------------------------------------

#[test]
fn the_coefficients_reproduce_det_j_to_machine_precision() {
    let mut rng = Rng(0x5EED_0001);
    let mut worst = 0.0f64;
    for _ in 0..200 {
        let x_ref = random_curved_element(&mut rng, 0.25);
        let xt = x_ref.transpose();
        let coeffs = det_bernstein_coeffs(&jacobians_at(&xt, &CORNERS));
        let scale = coeffs.iter().fold(0.0f64, |acc, c| acc.max(c.abs()));

        for _ in 0..25 {
            // Uniform-ish interior point of the simplex.
            let (mut x, mut y, mut z) = (rng.unit(), rng.unit(), rng.unit());
            let total = x + y + z;
            if total > 1.0 {
                x /= total + 1e-12;
                y /= total + 1e-12;
                z /= total + 1e-12;
            }
            let xi = Vec3::new(x, y, z);
            let lam = [1.0 - x - y - z, x, y, z];

            // Σ b_α B³_α(λ), same enumeration order as the coefficients.
            let mut acc = 0.0;
            let mut slot = 0usize;
            for a1 in 0..=3usize {
                for a2 in 0..=(3 - a1) {
                    for a3 in 0..=(3 - a1 - a2) {
                        let a0 = 3 - a1 - a2 - a3;
                        let basis = 6.0 / multi_factorial(a0, a1, a2, a3)
                            * pow_small(lam[0], a0)
                            * pow_small(lam[1], a1)
                            * pow_small(lam[2], a2)
                            * pow_small(lam[3], a3);
                        acc += coeffs[slot] * basis;
                        slot += 1;
                    }
                }
            }
            let direct = (xt * Tet10.shape_gradients(xi)).determinant();
            worst = worst.max((acc - direct).abs() / scale.max(1e-300));
        }
    }
    println!("[coeffs] worst relative reconstruction error: {worst:.3e}");
    assert!(
        worst < 1e-13,
        "the Bernstein form must BE det J, not approximate it; worst rel. err = {worst:e}"
    );
}

#[test]
fn the_corner_coefficients_are_the_corner_determinants() {
    let mut rng = Rng(0x5EED_0002);
    let grads = reference_corner_grads();
    for _ in 0..200 {
        let x_ref = random_curved_element(&mut rng, 0.3);
        let xt = x_ref.transpose();
        let b = det_bernstein_coeffs(&jacobians_at(&xt, &CORNERS));
        let eight = shipped_eight(&x_ref, &grads);
        // α = 3e_i sits at enumeration slots 0 (α0), 19 (α1), 16 (α2), 9 (α3);
        // rather than hard-code them, assert set membership, which is the
        // property that matters: every corner sample is one of the twenty.
        for (i, &corner_det) in eight[4..].iter().enumerate() {
            assert!(
                b.iter()
                    .any(|&c| (c - corner_det).abs() <= 1e-12 * corner_det.abs().max(1e-12)),
                "corner {i} determinant {corner_det:e} is not among the coefficients"
            );
        }
    }
}

#[test]
fn an_affine_element_has_twenty_equal_coefficients() {
    let mut rng = Rng(0x5EED_0003);
    for _ in 0..100 {
        // curve = 0 puts every midside exactly at its straight midpoint.
        let x_ref = random_curved_element(&mut rng, 0.0);
        let xt = x_ref.transpose();
        let b = det_bernstein_coeffs(&jacobians_at(&xt, &CORNERS));
        let d0 = Tet10.rest_jacobian_dets(&x_ref)[0];
        let spread = b.iter().fold(0.0f64, |m, c| m.max((c - d0).abs()));
        assert!(
            spread <= 1e-12 * d0.abs(),
            "a straight-edged element has constant det J; coefficient spread = {spread:e}"
        );
    }
}

#[test]
fn the_bound_encloses_a_dense_lattice() {
    let mut rng = Rng(0x5EED_0004);
    let mut worst_slack = f64::INFINITY;
    for _ in 0..150 {
        let x_ref = random_curved_element(&mut rng, 0.3);
        let xt = x_ref.transpose();
        let b = det_bernstein_coeffs(&jacobians_at(&xt, &CORNERS));
        let lo = b.iter().copied().fold(f64::INFINITY, f64::min);
        let hi = b.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let (l_lo, l_hi) = lattice_min_max(&xt, 12);
        let scale = hi.abs().max(lo.abs()).max(1e-300);
        assert!(
            lo <= l_lo + 1e-12 * scale && hi >= l_hi - 1e-12 * scale,
            "bracket [{lo:e}, {hi:e}] fails to enclose lattice [{l_lo:e}, {l_hi:e}]"
        );
        worst_slack = worst_slack.min((l_lo - lo) / scale);
    }
    println!("[bound] tightest lower-bound slack seen: {worst_slack:.4} of scale");
}

// ---------------------------------------------------------------------------
// 2. Cost
// ---------------------------------------------------------------------------

#[test]
fn pilot_cost_per_element() {
    let mut rng = Rng(0x5EED_0010);
    let pop: Vec<SMatrix<f64, 10, 3>> = (0..1024)
        .map(|_| random_curved_element(&mut rng, 0.15))
        .collect();
    let grads = reference_corner_grads();

    let t = Instant::now();
    let mut sink = 0.0f64;
    for _ in 0..REPS {
        for x in &pop {
            sink += Tet10.rest_jacobian_dets(std::hint::black_box(x))[0];
        }
    }
    let gauss4 = t.elapsed().as_secs_f64() / (REPS * pop.len()) as f64;

    let t = Instant::now();
    for _ in 0..REPS {
        for x in &pop {
            sink += shipped_eight(std::hint::black_box(x), &grads)[0];
        }
    }
    let sample8 = t.elapsed().as_secs_f64() / (REPS * pop.len()) as f64;

    let t = Instant::now();
    for _ in 0..REPS {
        for x in &pop {
            let xt = std::hint::black_box(x).transpose();
            sink += det_bernstein_coeffs(&jacobians_at(&xt, &CORNERS))[0];
        }
    }
    let bern20 = t.elapsed().as_secs_f64() / (REPS * pop.len()) as f64;

    let t = Instant::now();
    let mut total_work = 0usize;
    for _ in 0..REPS {
        for x in &pop {
            let xt = std::hint::black_box(x).transpose();
            let (v, w) = certify_element(&xt);
            total_work += w;
            sink += f64::from(v == Verdict::Valid);
        }
    }
    let full = t.elapsed().as_secs_f64() / (REPS * pop.len()) as f64;
    let mean_work = total_work as f64 / (REPS * pop.len()) as f64;

    println!(
        "\n=== COST per element (ns), curve = 0.15, n = {} ===",
        pop.len()
    );
    println!("  4-pt Gauss (rest_jacobian_dets) : {:8.1}", gauss4 * 1e9);
    println!("  8-pt shipped guard              : {:8.1}", sample8 * 1e9);
    println!(
        "  20-coeff certificate (depth 0)  : {:8.1}   ({:.2}× the 8-pt guard)",
        bern20 * 1e9,
        bern20 / sample8
    );
    println!(
        "  full certifier (cap {CAP})           : {:8.1}   ({:.2}× the 8-pt guard, {mean_work:.2} sub-simplices)",
        full * 1e9,
        full / sample8
    );
    println!("  (sink {sink:.3e})");
}

// ---------------------------------------------------------------------------
// 3. Verdict quality on a graded population
// ---------------------------------------------------------------------------

#[test]
fn pilot_verdicts_on_a_graded_population() {
    const PER_LEVEL: usize = 3000;
    println!("\n=== VERDICTS, {PER_LEVEL} elements per curvature level ===");
    println!(
        "{:>6} {:>8} {:>8} {:>9} {:>12} {:>14}",
        "curve", "valid", "invalid", "undecided", "8pt MISSES", "depth-0 alone"
    );

    for (level, &curve) in [0.02, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.40]
        .iter()
        .enumerate()
    {
        let mut rng = Rng(0xC0FF_EE00 + level as u64);
        let grads = reference_corner_grads();
        let (mut valid, mut invalid, mut undecided) = (0usize, 0usize, 0usize);
        let mut missed_by_eight = 0usize;
        let mut settled_at_depth0 = 0usize;

        for _ in 0..PER_LEVEL {
            let x_ref = random_curved_element(&mut rng, curve);
            let xt = x_ref.transpose();
            let (verdict, _) = certify_element(&xt);
            let eight_says_ok = shipped_eight(&x_ref, &grads).iter().all(|&d| d > 0.0);
            let coeffs = det_bernstein_coeffs(&jacobians_at(&xt, &CORNERS));
            let depth0_certifies = coeffs.iter().all(|&c| c > 0.0);

            match verdict {
                Verdict::Valid => {
                    valid += 1;
                    if depth0_certifies {
                        settled_at_depth0 += 1;
                    }
                }
                Verdict::Invalid => {
                    invalid += 1;
                    if eight_says_ok {
                        missed_by_eight += 1;
                    }
                }
                Verdict::Undecided => undecided += 1,
            }
        }
        let miss_rate = if invalid == 0 {
            0.0
        } else {
            100.0 * missed_by_eight as f64 / invalid as f64
        };
        let sharp = if valid == 0 {
            0.0
        } else {
            100.0 * settled_at_depth0 as f64 / valid as f64
        };
        println!(
            "{curve:>6.2} {valid:>8} {invalid:>8} {undecided:>9} {missed_by_eight:>6} ({miss_rate:>4.1}%) {sharp:>12.1}%"
        );
    }
    println!(
        "  '8pt MISSES' = elements PROVEN invalid (a witness point has det J <= 0) that the\n  \
         shipped eight-point guard accepts. 'depth-0 alone' = share of proven-valid elements\n  \
         the 20 coefficients settle with no subdivision at all."
    );
}

// ---------------------------------------------------------------------------
// 4. Real shipped geometry
// ---------------------------------------------------------------------------

const MU: f64 = 2.0e5;
const NU: f64 = 0.4;
const PRESSURE: f64 = 5.0e3;
const CELL: f64 = 0.04;

fn body_sdf() -> DifferenceSdf {
    DifferenceSdf::new(
        Box::new(SphereSdf {
            radius: LAYERED_SPHERE_R_OUTER,
        }),
        Box::new(SphereSdf {
            radius: LAYERED_SPHERE_R_CAVITY,
        }),
    )
}

fn straight_sphere_tet10() -> Tet10Mesh {
    let (mesh4, _bc, _initial, _theta) = SoftScene::layered_silicone_sphere(
        MaterialField::uniform(MU, 2.0 * MU * NU / (1.0 - 2.0 * NU)),
        CELL,
        PRESSURE,
    )
    .expect("layered_silicone_sphere meshes at the canonical cell size");
    Tet10Mesh::from_tet4(&mesh4)
}

fn element_nodes(mesh: &Tet10Mesh, t: u32) -> SMatrix<f64, 10, 3> {
    let corners = mesh.tet_vertices(t);
    let mids = mesh
        .tet_midside_nodes(t)
        .expect("Tet10Mesh surfaces the midside channel");
    let mut nodes = [Vec3::zeros(); 10];
    for (a, &c) in corners.iter().enumerate() {
        nodes[a] = mesh.positions()[c as usize];
    }
    for (i, &m) in mids.iter().enumerate() {
        nodes[4 + i] = mesh.positions()[m as usize];
    }
    SMatrix::<f64, 10, 3>::from_fn(|a, k| nodes[a][k])
}

#[test]
fn pilot_on_the_canonical_projected_sphere() {
    let grads = reference_corner_grads();
    for (label, mesh) in [
        ("straight (enriched Tet4)", straight_sphere_tet10()),
        (
            "curved (with_sdf_projected_boundary)",
            straight_sphere_tet10().with_sdf_projected_boundary(&body_sdf()),
        ),
    ] {
        let (mut valid, mut invalid, mut undecided) = (0usize, 0usize, 0usize);
        let (mut eight_ok, mut missed, mut depth0) = (0usize, 0usize, 0usize);
        let mut work = 0usize;
        let n_tets = mesh.n_tets();

        let clock = Instant::now();
        for ti in 0..n_tets as u32 {
            let x_ref = element_nodes(&mesh, ti);
            let xt = x_ref.transpose();
            let (verdict, steps) = certify_element(&xt);
            work += steps;
            let says_ok = shipped_eight(&x_ref, &grads).iter().all(|&d| d > 0.0);
            eight_ok += usize::from(says_ok);
            let coeffs = det_bernstein_coeffs(&jacobians_at(&xt, &CORNERS));
            match verdict {
                Verdict::Valid => {
                    valid += 1;
                    depth0 += usize::from(coeffs.iter().all(|&c| c > 0.0));
                }
                Verdict::Invalid => {
                    invalid += 1;
                    missed += usize::from(says_ok);
                }
                Verdict::Undecided => undecided += 1,
            }
        }
        let secs = clock.elapsed().as_secs_f64();

        println!("\n=== CANONICAL SPHERE — {label} ({n_tets} elements) ===");
        println!("  certified valid : {valid}   (depth-0 alone: {depth0})");
        println!("  proven invalid  : {invalid}");
        println!("  undecided @{CAP}   : {undecided}");
        println!("  8-pt guard says valid: {eight_ok}   of which PROVEN INVALID: {missed}");
        println!(
            "  whole-mesh certification: {:.1} ms, {:.2} sub-simplices/element",
            secs * 1e3,
            work as f64 / n_tets as f64
        );
    }
}
